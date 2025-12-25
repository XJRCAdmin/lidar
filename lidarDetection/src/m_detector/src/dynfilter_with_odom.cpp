#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <types.h>
#include <m_detector/DynObjFilter.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <deque>
#include <memory>
#include <sstream>
#include <iomanip>

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();

int     QUAD_LAYER_MAX  = 1;
int     occlude_windows = 3;
int     point_index = 0;
float   VER_RESOLUTION_MAX  = 0.01;
float   HOR_RESOLUTION_MAX  = 0.01;
float   angle_noise     = 0.001;
float   angle_occlude     = 0.02;
float   dyn_windows_dur = 0.5;
bool    dyn_filter_en = true, dyn_filter_dbg_en = true;
string  points_topic, odom_topic;
string  out_folder, out_folder_origin;
double  lidar_end_time = 0;
int     dataset = 0;
int     cur_frame = 0;

deque<M3D> buffer_rots;
deque<V3D> buffer_poss;
deque<double> buffer_times;
deque<PointCloudXYZI::Ptr> buffer_pcs;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_extend;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_std;

class DynFilterNode : public rclcpp::Node
{
public:
    DynFilterNode() : Node("dynfilter_odom")
    {
        // Provide safe non-empty defaults to avoid InvalidTopicNameError
        this->declare_parameter<string>("dyn_obj/points_topic", "/livox/lidar");
        this->declare_parameter<string>("dyn_obj/odom_topic", "/Odometry");
        this->declare_parameter<string>("dyn_obj/out_file", "");
        this->declare_parameter<string>("dyn_obj/out_file_origin", "");

        this->get_parameter("dyn_obj/points_topic", points_topic);
        this->get_parameter("dyn_obj/odom_topic", odom_topic);
        this->get_parameter("dyn_obj/out_file", out_folder);
        this->get_parameter("dyn_obj/out_file_origin", out_folder_origin);

        // Fallbacks if parameters are empty (defensive programming)
        if (points_topic.empty()) {
            points_topic = "/livox/lidar";
            RCLCPP_WARN(this->get_logger(),
                        "Parameter 'dyn_obj/points_topic' not set; defaulting to %s",
                        points_topic.c_str());
        }
        if (odom_topic.empty()) {
            odom_topic = "/Odometry";
            RCLCPP_WARN(this->get_logger(),
                        "Parameter 'dyn_obj/odom_topic' not set; defaulting to %s",
                        odom_topic.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Subscribing points_topic: %s", points_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing odom_topic: %s", odom_topic.c_str());


        pub_pcl_dyn_extend = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/frame_out", 10);
        pub_pcl_dyn = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/point_out", 10);
        pub_pcl_std = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/std_points", 10);

        sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            points_topic, rclcpp::SensorDataQoS(),
            std::bind(&DynFilterNode::PointsCallback, this, std::placeholders::_1));

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 100,
            std::bind(&DynFilterNode::OdomCallback, this, std::placeholders::_1));

        timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DynFilterNode::TimerCallback, this));
    }
    void initialize()
    {
        DynObjFilt->init(this->shared_from_this());
    }
private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Isometry3d T;
        tf2::fromMsg(msg->pose.pose, T);
        cur_rot = T.rotation();
        cur_pos = T.translation();

        buffer_rots.push_back(cur_rot);
        buffer_poss.push_back(cur_pos);

        lidar_end_time = rclcpp::Time(msg->header.stamp).seconds();
        buffer_times.push_back(lidar_end_time);
    }

    void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert robustly: if incoming cloud lacks normal/curvature fields,
        // map from XYZI and fill normals/time with defaults to avoid field-match errors.
        auto has_field = [](const sensor_msgs::msg::PointCloud2 &m, const std::string &name) -> bool {
            for (const auto &f : m.fields) {
                if (f.name == name) return true;
            }
            return false;
        };

        bool has_normals = has_field(*msg, "normal_x") && has_field(*msg, "normal_y") && has_field(*msg, "normal_z");
        bool has_curv = has_field(*msg, "curvature");

        PointCloudXYZI::Ptr pc(new PointCloudXYZI());
        if (has_normals && has_curv) {
            // Directly convert to PointXYZINormal
            pcl::fromROSMsg(*msg, *pc);
        } else {
            // Convert from XYZI, then enrich
            // 先转为中间格式 pcl::PointXYZI，然后遍历每个点，手动将缺少的 normal 和 curvature 字段补零。
            pcl::PointCloud<pcl::PointXYZI> tmp;
            pcl::fromROSMsg(*msg, tmp);
            pc->points.reserve(tmp.points.size());
            for (const auto &q : tmp.points) {
                PointType p;
                p.x = q.x; p.y = q.y; p.z = q.z; p.intensity = q.intensity;
                p.normal_x = 0.0f; p.normal_y = 0.0f; p.normal_z = 0.0f;
                p.curvature = 0.0f;  // fallback: no per-point time available
                pc->points.push_back(p);
            }
            pc->width = static_cast<uint32_t>(pc->points.size());
            pc->height = 1; pc->is_dense = tmp.is_dense;
        }
        buffer_pcs.push_back(pc);
    }

    void TimerCallback()
    {
        if (!buffer_pcs.empty() &&
            !buffer_rots.empty() &&
            !buffer_poss.empty() &&
            !buffer_times.empty())
        {
            auto cur_pc = buffer_pcs.front(); buffer_pcs.pop_front();
            auto cur_r = buffer_rots.front(); buffer_rots.pop_front();
            auto cur_p = buffer_poss.front(); buffer_poss.pop_front();
            auto cur_t = buffer_times.front(); buffer_times.pop_front();

            stringstream ss, sso;
            ss << out_folder << setw(6) << setfill('0') << cur_frame << ".label";
            sso << out_folder_origin << setw(6) << setfill('0') << cur_frame << ".label";

            if (ss.str().length() > 15 || sso.str().length() > 15)
                DynObjFilt->set_path(ss.str(), sso.str());

            DynObjFilt->filter(cur_pc, cur_r, cur_p, cur_t);
            DynObjFilt->publish_dyn(pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, cur_t);

            cur_frame++;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynFilterNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

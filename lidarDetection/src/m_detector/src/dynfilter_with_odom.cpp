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
        this->declare_parameter<string>("dyn_obj/points_topic", "");
        this->declare_parameter<string>("dyn_obj/odom_topic", "");
        this->declare_parameter<string>("dyn_obj/out_file", "");
        this->declare_parameter<string>("dyn_obj/out_file_origin", "");

        this->get_parameter("dyn_obj/points_topic", points_topic);
        this->get_parameter("dyn_obj/odom_topic", odom_topic);
        this->get_parameter("dyn_obj/out_file", out_folder);
        this->get_parameter("dyn_obj/out_file_origin", out_folder_origin);

        DynObjFilt->init(shared_from_this());

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
        PointCloudXYZI::Ptr pc(new PointCloudXYZI());
        pcl::fromROSMsg(*msg, *pc);
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
    rclcpp::spin(std::make_shared<DynFilterNode>());
    rclcpp::shutdown();
    return 0;
}

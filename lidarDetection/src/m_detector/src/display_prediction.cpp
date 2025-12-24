#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
// #include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Core>
#include <types.h>
#include <m_detector/DynObjFilter.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl/filters/random_sample.h>
#include <unistd.h> 
#include <dirent.h> 
#include <iomanip>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;

pcl::PointCloud<pcl::PointXYZINormal> lastcloud;
PointCloudXYZI::Ptr last_pc(new PointCloudXYZI());
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud, pub_iou_view, pub_static, pub_dynamic, pub_tp, pub_fp, pub_fn, pub_tn;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points;

string points_topic = "/velodyne_points_revise";
string frame_id = "odom";
string pc_folder, label_folder, pred_folder, iou_file;


int total_tp = 0, total_fn = 0, total_fp = 0;
         

int frames = 0, minus_num = 1;
void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
{

    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *points_in);

    if(frames < minus_num)
    {
        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = g_node->get_clock()->now();
        pub_pointcloud->publish(pcl_ros_msg);
    }
    else
    {   
        cout << "frame: " << frames << endl;

        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames-minus_num ;
        pred_file += sss.str(); 
        pred_file.append(".label");

        

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_out (new pcl::PointCloud<pcl::PointXYZI>);
        

        int tp = 0, fn = 0, fp = 0, count = 0;
        float iou = 0.0f;
        for (size_t i=0; i<points_in->points.size(); i++) 
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;


            // pred_input >> pred_num;
            int pred_num;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;
            // if(pred_num != 9)
            //     cout<<"label: "<<label_num<<" pred: "<<pred_num<<endl;

            point.intensity = 0;
            
            if(pred_num >= 251)
            {
                point.intensity = 10;
                iou_out->push_back(point);
                dynamic_out->push_back(point);
            }
            else
            {
                point.intensity = 20;
                iou_out->push_back(point);
                static_out->push_back(point);
            }
               
            points_out->push_back(point);
        }
        

        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = g_node->get_clock()->now();
        pub_pointcloud->publish(pcl_ros_msg);

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = frame_id;
        pcl_msg.header.stamp = g_node->get_clock()->now();
        pub_iou_view->publish(pcl_msg); 

        sensor_msgs::msg::PointCloud2 dynamic_msg;
        pcl::toROSMsg(*dynamic_out, dynamic_msg);
        dynamic_msg.header.frame_id = frame_id;
        dynamic_msg.header.stamp = g_node->get_clock()->now();
        pub_dynamic->publish(dynamic_msg); 

        sensor_msgs::msg::PointCloud2 static_msg;
        pcl::toROSMsg(*static_out, static_msg);
        static_msg.header.frame_id = frame_id;
        static_msg.header.stamp = g_node->get_clock()->now();
        pub_static->publish(static_msg); 


        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = g_node->get_clock()->now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        geometry_msgs::msg::Pose pose;
        pose.position.x =  points_out->points[0].x;
        pose.position.y =  points_out->points[0].y;
        pose.position.z =  points_out->points[0].z;
        ostringstream str;
        str<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: "<<iou;
        marker.text=str.str();
        marker.pose=pose;
        pub_marker->publish(marker);
    }
    
    frames ++;
    // pred_input.seekg(0, std::ios::beg);
}
void AviaPointsCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg_in)
{   
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    points_in->resize(msg_in->point_num);
    std::cout << "points size: " << msg_in->point_num << std::endl;
    if(msg_in->point_num == 0) return;
    for(size_t i = 0; i < msg_in->point_num; i++)
    {
        points_in->points[i].x = msg_in->points[i].x;
        points_in->points[i].y = msg_in->points[i].y;
        points_in->points[i].z = msg_in->points[i].z;
    }

    if(frames < minus_num)
    {
        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = g_node->get_clock()->now();
        pub_pointcloud->publish(pcl_ros_msg);
    }
    else
    {   
        cout << "frame: " << frames << endl;

        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames-minus_num ;
        pred_file += sss.str(); 
        pred_file.append(".label");

        

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_out (new pcl::PointCloud<pcl::PointXYZI>);
        

        int tp = 0, fn = 0, fp = 0, count = 0;
        float iou = 0.0f;
        for (size_t i=0; i<points_in->points.size(); i++) 
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;


            // pred_input >> pred_num;
            int pred_num;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;
            // if(pred_num != 9)
            //     cout<<"label: "<<label_num<<" pred: "<<pred_num<<endl;

            point.intensity = 0;
            
            if(pred_num >= 251)
            {
                point.intensity = 10;
                iou_out->push_back(point);
                dynamic_out->push_back(point);
            }
            else
            {
                point.intensity = 20;
                iou_out->push_back(point);
                static_out->push_back(point);
            }
               
            points_out->push_back(point);
        }
        

        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = g_node->get_clock()->now();
        pub_pointcloud->publish(pcl_ros_msg);

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = frame_id;
        pcl_msg.header.stamp = g_node->get_clock()->now();
        pub_iou_view->publish(pcl_msg); 

        sensor_msgs::msg::PointCloud2 dynamic_msg;
        pcl::toROSMsg(*dynamic_out, dynamic_msg);
        dynamic_msg.header.frame_id = frame_id;
        dynamic_msg.header.stamp = g_node->get_clock()->now();
        pub_dynamic->publish(dynamic_msg); 

        sensor_msgs::msg::PointCloud2 static_msg;
        pcl::toROSMsg(*static_out, static_msg);
        static_msg.header.frame_id = frame_id;
        static_msg.header.stamp = g_node->get_clock()->now();
        pub_static->publish(static_msg); 


        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = g_node->get_clock()->now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        geometry_msgs::msg::Pose pose;
        pose.position.x =  points_out->points[0].x;
        pose.position.y =  points_out->points[0].y;
        pose.position.z =  points_out->points[0].z;
        ostringstream str;
        str<<"tp: "<<tp<<" fn: "<<fn<<" fp: "<<fp<<" count: "<<count<<" iou: "<<iou;
        marker.text=str.str();
        marker.pose=pose;
        pub_marker->publish(marker);
    }
    
    frames ++;
    // pred_input.seekg(0, std::ios::beg);
    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("display_prediction");

    pc_folder = g_node->declare_parameter<std::string>("dyn_obj/pc_file", "");
    pred_folder = g_node->declare_parameter<std::string>("dyn_obj/pred_file", "");
    points_topic = g_node->declare_parameter<std::string>("dyn_obj/pc_topic", "/velodyne_points");
    frame_id = g_node->declare_parameter<std::string>("dyn_obj/frame_id", "odom");

    int pred_num = 0;
    DIR* pred_dir;
    pred_dir = opendir(pred_folder.c_str());
    struct dirent* pred_ptr;
    while((pred_ptr = readdir(pred_dir)) != NULL)
    {
        if(pred_ptr->d_name[0] == '.') {continue;}
        pred_num++;
    }
    closedir(pred_dir);

    minus_num = 0;

    pub_pointcloud = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/m_detector/result_view", 100000);
    pub_marker = g_node->create_publisher<visualization_msgs::msg::Marker>(
        "/m_detector/text_view", 10);
    pub_iou_view = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/m_detector/iou_view", 100000);
    pub_static = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/m_detector/std_points", 100000);
    pub_dynamic = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/m_detector/dyn_points", 100000);

    auto sub_pcl = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic, 200000, PointsCallback);

    rclcpp::spin(g_node);
    rclcpp::shutdown();

    return 0;
}
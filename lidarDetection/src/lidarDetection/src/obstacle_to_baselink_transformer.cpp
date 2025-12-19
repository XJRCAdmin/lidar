#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lidar_detection/msg/obstacle_detection.hpp"
#include "lidar_detection/msg/obstacle_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class ObstacleToBaselinkNode : public rclcpp::Node
{
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string obstacle_lidar_topic_;
  std::string obstacle_baselink_topic_;
  std::string source_frame_;
  std::string target_frame_;

  rclcpp::Subscription<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr obstacle_lidar_sub_;
  rclcpp::Publisher<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr obstacle_to_baselink_pub_;

public:
  ObstacleToBaselinkNode() : Node("obstacle_to_baselink_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    obstacle_lidar_topic_ = this->declare_parameter<std::string>("obstacle_lidar_topic", "/detected_objects");
    obstacle_baselink_topic_ =
      this->declare_parameter<std::string>("obstacle_baselink_topic", "/obstacle_information_to_baselink");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "lidar_body");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    obstacle_lidar_sub_ = this->create_subscription<lidar_detection::msg::ObstacleDetectionArray>(
      obstacle_lidar_topic_, 10, std::bind(&ObstacleToBaselinkNode::ObstacleCallback, this, _1));
    obstacle_to_baselink_pub_ =
      this->create_publisher<lidar_detection::msg::ObstacleDetectionArray>(obstacle_baselink_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Obstacle To Baselink Node Started");
    RCLCPP_INFO(this->get_logger(), "Obstacle lidar topic: %s", obstacle_lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Obstacle baselink topic: %s", obstacle_baselink_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Transform from '%s' to '%s'", source_frame_.c_str(), target_frame_.c_str());
  }

  void ObstacleCallback(const lidar_detection::msg::ObstacleDetectionArray::SharedPtr obstacle_msg)
  {
    lidar_detection::msg::ObstacleDetectionArray custom_obstacle_array_msg;
    custom_obstacle_array_msg.header = obstacle_msg->header;
    custom_obstacle_array_msg.header.frame_id = target_frame_;

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Lookup transform failed: %s", ex.what());
      return;
    }

    for (const auto & detection : obstacle_msg->detections) {
      lidar_detection::msg::ObstacleDetection custom_obstacle_msg;
      custom_obstacle_msg = detection;  // copy detection (we'll overwrite fields as needed)

      // Transform bbox.center (use PoseStamped for safe transform)
      geometry_msgs::msg::PoseStamped in_pose, out_pose;
      in_pose.header.frame_id = source_frame_;
      in_pose.header.stamp = this->now();
      in_pose.pose = detection.detection.bbox.center;
      try {
        tf2::doTransform(in_pose, out_pose, transform_stamped);
        custom_obstacle_msg.detection.bbox.center = out_pose.pose;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform bbox center: %s", ex.what());
        continue;
      }

      // Transform linear velocity (Vector3Stamped)
      geometry_msgs::msg::Vector3Stamped linear_in, linear_out;
      linear_in.vector = detection.twist.linear;
      linear_in.header.frame_id = source_frame_;
      linear_in.header.stamp = this->now();
      try {
        tf2::doTransform(linear_in, linear_out, transform_stamped);
        custom_obstacle_msg.twist.linear = linear_out.vector;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform linear velocity: %s", ex.what());
      }

      // Transform angular velocity (Vector3Stamped)
      geometry_msgs::msg::Vector3Stamped angular_in, angular_out;
      angular_in.vector = detection.twist.angular;
      angular_in.header.frame_id = source_frame_;
      angular_in.header.stamp = this->now();
      try {
        tf2::doTransform(angular_in, angular_out, transform_stamped);
        custom_obstacle_msg.twist.angular = angular_out.vector;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform angular velocity: %s", ex.what());
      }

      custom_obstacle_msg.detection.header.frame_id = target_frame_;
      custom_obstacle_array_msg.detections.push_back(custom_obstacle_msg);
    }

    obstacle_to_baselink_pub_->publish(custom_obstacle_array_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleToBaselinkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

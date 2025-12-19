#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class OdomTransformerNode : public rclcpp::Node
{
  private:

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string source_frame_;
  std::string target_frame_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  public:
    OdomTransformerNode() : Node("odom_trans_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    input_odom_topic_ = this->declare_parameter<std::string>("input_odom_topic", "/Odometry");
    output_odom_topic_ = this->declare_parameter<std::string>("output_odom_topic", "/odom_base_link");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "lidar_body");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 10, std::bind(&OdomTransformerNode::odomCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Odom Transformer Node Started");
    RCLCPP_INFO(this->get_logger(), "Input odom topic: %s", input_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output odom topic: %s", output_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Transform from '%s' to '%s'", source_frame_.c_str(), target_frame_.c_str());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    nav_msgs::msg::Odometry transformed_odom;
    transformed_odom.header.stamp = odom_msg->header.stamp;
    transformed_odom.header.frame_id = "lidar_body";
    transformed_odom.child_frame_id = target_frame_;

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Waiting for transform from %s to %s", source_frame_.c_str(),
        target_frame_.c_str());
      transformed_odom.pose = odom_msg->pose;
      transformed_odom.twist = odom_msg->twist;
      odom_pub_->publish(transformed_odom);
      return;
    }

    try {
      geometry_msgs::msg::PoseStamped src_pose, dst_pose;
      src_pose.header.frame_id = source_frame_;
      src_pose.header.stamp = odom_msg->header.stamp;
      src_pose.pose = odom_msg->pose.pose;

      tf2::doTransform(src_pose, dst_pose, transform_stamped);
      transformed_odom.pose.pose = dst_pose.pose;
      transformed_odom.pose.covariance = odom_msg->pose.covariance;

      geometry_msgs::msg::Vector3Stamped src_linear, dst_linear;
      src_linear.header = src_pose.header;
      src_linear.vector = odom_msg->twist.twist.linear;

      geometry_msgs::msg::Vector3Stamped src_angular, dst_angular;
      src_angular.header = src_pose.header;
      src_angular.vector = odom_msg->twist.twist.angular;

      tf2::doTransform(src_linear, dst_linear, transform_stamped);
      tf2::doTransform(src_angular, dst_angular, transform_stamped);

      transformed_odom.twist.twist.linear = dst_linear.vector;
      transformed_odom.twist.twist.angular = dst_angular.vector;
      transformed_odom.twist.covariance = odom_msg->twist.covariance;

      odom_pub_->publish(transformed_odom);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF2 exception: %s", ex.what());
      transformed_odom.pose = odom_msg->pose;
      transformed_odom.twist = odom_msg->twist;
      odom_pub_->publish(transformed_odom);
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTransformerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

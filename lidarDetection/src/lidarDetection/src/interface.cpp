#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lidar_detection/msg/obstacle_detection.hpp"
#include "lidar_detection/msg/obstacle_detection_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lidar_detection/msg/ball_relative.hpp"
#include "lidar_detection/msg/ball_state.hpp"
#include "lidar_detection/msg/robot_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class InterfaceNode : public rclcpp::Node
{
private:
  std::string input_topic_odom_;
  std::string output_topic_robot_state_;
  std::string input_relative_ball_topic_;
  std::string output_relative_ball_topic_;
  std::string input_map_ball_topic_;
  std::string output_map_ball_topic_;
  std::string source_frame_;
  std::string target_frame_;
  double robot_radius_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<lidar_detection::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Subscription<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr relative_ball_sub_;
  rclcpp::Publisher<lidar_detection::msg::BallRelative>::SharedPtr relative_ball_pub_;
  rclcpp::Subscription<lidar_detection::msg::ObstacleDetectionArray>::SharedPtr map_ball_sub_;
  rclcpp::Publisher<lidar_detection::msg::BallState>::SharedPtr map_ball_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  InterfaceNode() : Node("interface_node")
  {
    input_topic_odom_ = this->declare_parameter<std::string>("input_topic_odom", "/odom_base_link");
    output_topic_robot_state_ = this->declare_parameter<std::string>("output_topic_robot_state", "/robot_state");
    // 输入的球相对位置话题,目前使用的是obstacle_information_to_baselink的话题,将所有障碍物当作"球"处理了,后续需要调整成真正的球检测话题
    input_relative_ball_topic_ =
      this->declare_parameter<std::string>("input_relative_ball_topic", "/obstacle_information_to_baselink");
    output_relative_ball_topic_ = this->declare_parameter<std::string>("output_relative_ball_topic", "/ball_relative");
    //下面两行同理
    input_map_ball_topic_ =
      this->declare_parameter<std::string>("input_map_ball_topic", "/obstacle_information_to_baselink");
    output_map_ball_topic_ = this->declare_parameter<std::string>("output_map_ball_topic", "/ball_map");

    source_frame_ = this->declare_parameter<std::string>("source_frame", "base_link");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    robot_radius_ = this->declare_parameter<double>("robot_radius", 0.2);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_odom_, 10, std::bind(&InterfaceNode::odomCallback, this, std::placeholders::_1));
    robot_state_pub_ = this->create_publisher<lidar_detection::msg::RobotState>(output_topic_robot_state_, 10);

    // 后续也需要调整
    relative_ball_sub_ = this->create_subscription<lidar_detection::msg::ObstacleDetectionArray>(
      input_relative_ball_topic_, 10, std::bind(&InterfaceNode::ObstacleRelativeCallback, this, std::placeholders::_1));
    relative_ball_pub_ = this->create_publisher<lidar_detection::msg::BallRelative>(output_relative_ball_topic_, 10);

    map_ball_sub_ = this->create_subscription<lidar_detection::msg::ObstacleDetectionArray>(
      input_map_ball_topic_, 10, std::bind(&InterfaceNode::ObstacleMapCallback, this, std::placeholders::_1));
    map_ball_pub_ = this->create_publisher<lidar_detection::msg::BallState>(output_map_ball_topic_, 10);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    lidar_detection::msg::RobotState robot_state;
    robot_state.header = odom_msg->header;
    robot_state.pose = odom_msg->pose.pose;
    robot_state.twist = odom_msg->twist.twist;
    robot_state.radius = this->robot_radius_;

    robot_state_pub_->publish(robot_state);
  }

  // 只发布最近的一个障碍物的信息,obstacle_detection_array是当前ros帧下所有障碍物的信息
  // 而BallRelative是一个障碍物的相对位置信息,故选择最近的一个障碍物进行发布
  void ObstacleRelativeCallback(const lidar_detection::msg::ObstacleDetectionArray::SharedPtr ball_msg)
  {
    if (!relative_ball_pub_ || ball_msg->detections.empty()) {
      return;
    }

    size_t best_index = 0;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < ball_msg->detections.size(); ++i) {
      const auto & center = ball_msg->detections[i].detection.bbox.center.position;
      const double dist = std::hypot(center.x, center.y);
      if (dist < best_dist) {
        best_dist = dist;
        best_index = i;
      }
    }

    const auto & best_detection = ball_msg->detections[best_index];
    lidar_detection::msg::BallRelative msg;
    msg.header = best_detection.detection.header;

    msg.relative_position.x = best_detection.detection.bbox.center.position.x;
    msg.relative_position.y = best_detection.detection.bbox.center.position.y;
    msg.relative_position.z = best_detection.detection.bbox.center.position.z;

    msg.relative_velocity.x = best_detection.twist.linear.x;
    msg.relative_velocity.y = best_detection.twist.linear.y;
    msg.relative_velocity.z = best_detection.twist.linear.z;

    const auto & size = best_detection.detection.bbox.size;
    msg.radius = 0.5 * std::max({size.x, size.y, size.z});
    relative_ball_pub_->publish(msg);
  }
  void ObstacleMapCallback(const lidar_detection::msg::ObstacleDetectionArray::SharedPtr obstacle_msg)
  {
    if (!map_ball_pub_ || obstacle_msg->detections.empty()) {
      return;
    }
    size_t best_index = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < obstacle_msg->detections.size(); ++i) {
      const auto & c = obstacle_msg->detections[i].detection.bbox.center.position;
      const double dist = std::hypot(c.x, c.y);
      if (dist < best_dist) {
        best_dist = dist;
        best_index = i;
      }
    }
    const auto & best_detection = obstacle_msg->detections[best_index];

    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header = best_detection.detection.header;  // frame: base_link
    base_pose.pose.position = best_detection.detection.bbox.center.position;
    base_pose.pose.orientation = best_detection.detection.bbox.center.orientation;

    geometry_msgs::msg::PoseStamped map_pose;
    try {
      tf_buffer_->transform(base_pose, map_pose, "map", tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform base_link->map failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::Vector3Stamped base_vel, map_vel;
    base_vel.header = best_detection.detection.header;
    base_vel.vector = best_detection.twist.linear;

    try {
      tf_buffer_->transform(base_vel, map_vel, "map", tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Velocity transform base_link->map failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::Vector3Stamped base_angular, map_angular;
    base_angular.header = best_detection.detection.header;
    base_angular.vector = best_detection.twist.angular;

    try {
      tf_buffer_->transform(base_angular, map_angular, "map", tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Angular velocity transform base_link->map failed: %s", ex.what());
      return;
    }

    lidar_detection::msg::BallState ball_state;
    ball_state.header = map_pose.header;
    ball_state.pose = map_pose.pose;
    ball_state.twist.linear = map_vel.vector;
    ball_state.twist.angular = map_angular.vector;

    const auto & size = best_detection.detection.bbox.size;
    ball_state.radius = 0.5 * std::max({size.x, size.y, size.z});

    map_ball_pub_->publish(ball_state);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
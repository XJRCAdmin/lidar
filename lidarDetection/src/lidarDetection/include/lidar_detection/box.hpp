#pragma once
#include <geometry_msgs/msg/point32.hpp>

#include <Eigen/Geometry>
struct Box
{
public:
  int id;
  Eigen::Vector3f position;
  Eigen::Vector3f dimension;
  Eigen::Quaternionf quaternion;
  Eigen::Vector2f velocity;
  std::vector<geometry_msgs::msg::Point32> convex_hull;

  Box(){};

  Box(int id, Eigen::Vector3f position, Eigen::Vector3f dimension)
  : id(id),
    position(position),
    dimension(dimension),
    quaternion(Eigen::Quaternionf(1, 0, 0, 0)),
    velocity(Eigen::Vector2f::Zero())
  {
  }

  // 带四元数的完整构造函数
  Box(int id, Eigen::Vector3f position, Eigen::Vector3f dimension, Eigen::Quaternionf quaternion)
  : id(id), position(position), dimension(dimension), quaternion(quaternion), velocity(Eigen::Vector2f::Zero())
  {
  }

  // 带凸包信息
  Box(
    int id, Eigen::Vector3f position, Eigen::Vector3f dimension, Eigen::Quaternionf quaternion,
    const std::vector<geometry_msgs::msg::Point32> & convex_hull)
  : id(id),
    position(position),
    dimension(dimension),
    quaternion(quaternion),
    velocity(Eigen::Vector2f::Zero()),
    convex_hull(convex_hull)
  {
  }
};

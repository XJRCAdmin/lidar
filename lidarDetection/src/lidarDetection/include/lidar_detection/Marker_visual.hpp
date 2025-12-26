#pragma once

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <iomanip>
#include <sstream>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_detection/box.hpp"
#include "lidar_detection/msg/obstacle_detection_array.hpp"
visualization_msgs::msg::Marker makeFootprintOrCubeMarker(
  const Box & box, const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "obstacle_bboxes";
  m.id = box.id;  // 与障碍物 id 对齐
  m.action = visualization_msgs::msg::Marker::ADD;
  m.frame_locked = false;

  // 设置 lifetime 为 0.1s
  m.lifetime.sec = 0;
  m.lifetime.nanosec = 20000000;

  m.type = visualization_msgs::msg::Marker::CUBE;
  m.pose = pose;
  m.scale.x = box.dimension(0);
  m.scale.y = box.dimension(1);
  m.scale.z = box.dimension(2);
  m.color.r = 0.1f;
  m.color.g = 0.6f;
  m.color.b = 1.0f;
  m.color.a = 0.5f;

  return m;
}

visualization_msgs::msg::Marker makeVelocityArrowMarker(
  const Box & box, const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "obstacle_velocity";
  m.id = box.id;  // 复用 id，ns 区分
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.frame_locked = false;

  // 设置 lifetime 为 0.1s
  m.lifetime.sec = 0;
  m.lifetime.nanosec = 90000000;  // 0.1s

  geometry_msgs::msg::Point p0;
  p0.x = pose.position.x;
  p0.y = pose.position.y;
  p0.z = pose.position.z;

  const double vx = static_cast<double>(box.velocity(0));
  const double vy = static_cast<double>(box.velocity(1));
  const double v_abs = std::hypot(vx, vy);
  const double scale_gain = 0.5;  // 箭长比例因子
  geometry_msgs::msg::Point p1;
  p1.x = p0.x + scale_gain * vx;
  p1.y = p0.y + scale_gain * vy;
  p1.z = p0.z;

  m.points.clear();
  m.points.push_back(p0);
  m.points.push_back(p1);

  // ARROW 的 scale.x 为箭杆直径，scale.y 为箭头直径，scale.z 为箭头长度比例
  m.scale.x = 0.05;  // 杆粗
  m.scale.y = 0.1;   // 头宽
  m.scale.z = 0.3;   // 头长比例
  m.color.r = 1.0f;
  m.color.g = 0.8f;
  m.color.b = 0.0f;
  m.color.a = v_abs > 1e-3 ? 0.9f : 0.0f;  // 静止时不显示箭头
  return m;
}

visualization_msgs::msg::Marker makeTextMarker(
  const Box & box, const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "obstacle_text";
  m.id = box.id;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.frame_locked = false;

  m.lifetime.sec = 0;
  m.lifetime.nanosec = 90000000;

  // 文本位置稍微抬高到盒子顶面上方
  m.pose = pose;
  m.pose.position.z += std::max(0.2f, box.dimension(2) * 0.6f);

  m.scale.z = 0.3;
  m.color.r = 1.0f;
  m.color.g = 1.0f;
  m.color.b = 1.0f;
  m.color.a = 1.0f;

  const double vx = static_cast<double>(box.velocity(0));
  const double vy = static_cast<double>(box.velocity(1));
  const double v_abs = std::hypot(vx, vy);
  std::ostringstream oss;
  oss << " v:" << std::fixed << std::setprecision(2) << v_abs << " m/s";
  m.text = oss.str();
  return m;
}

visualization_msgs::msg::MarkerArray DynamicObstacleCubeMarker(
  lidar_detection::msg::ObstacleDetectionArray DynamicObstacleMsg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < DynamicObstacleMsg.detections.size(); ++i) {
    const auto & obstacle = DynamicObstacleMsg.detections[i];

    visualization_msgs::msg::Marker marker;
    marker.header = obstacle.detection.header;
    marker.ns = "dynamic_obstacle_bboxes";
    marker.id = std::stoi(obstacle.id);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = false;

    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 90000000;

    marker.type = visualization_msgs::msg::Marker::CUBE;

    marker.pose = obstacle.detection.bbox.center;
    marker.scale.x = obstacle.detection.bbox.size.x;
    marker.scale.y = obstacle.detection.bbox.size.y;
    marker.scale.z = obstacle.detection.bbox.size.z;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray DynamicObstacleArrayMarker(
  lidar_detection::msg::ObstacleDetectionArray ObstacleMsg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < ObstacleMsg.detections.size(); ++i) {
    const auto & obstacle = ObstacleMsg.detections[i];

    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header = obstacle.detection.header;
    cube_marker.ns = "dynamic_obstacle_bboxes";
    cube_marker.id = std::stoi(obstacle.id);
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    cube_marker.frame_locked = false;

    cube_marker.lifetime.sec = 0;
    cube_marker.lifetime.nanosec = 90000000;

    cube_marker.type = visualization_msgs::msg::Marker::CUBE;

    cube_marker.pose = obstacle.detection.bbox.center;
    cube_marker.scale.x = obstacle.detection.bbox.size.x;
    cube_marker.scale.y = obstacle.detection.bbox.size.y;
    cube_marker.scale.z = obstacle.detection.bbox.size.z;

    cube_marker.color.r = 1.0f;
    cube_marker.color.g = 0.0f;
    cube_marker.color.b = 0.0f;
    cube_marker.color.a = 0.7f;

    marker_array.markers.push_back(cube_marker);

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header = obstacle.detection.header;
    arrow_marker.ns = "dynamic_obstacle_velocity";
    arrow_marker.id = i;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.frame_locked = false;

    arrow_marker.lifetime.sec = 0;
    arrow_marker.lifetime.nanosec = 90000000;

    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;

    geometry_msgs::msg::Point p0;
    p0.x = obstacle.detection.bbox.center.position.x;
    p0.y = obstacle.detection.bbox.center.position.y;
    p0.z = obstacle.detection.bbox.center.position.z;

    const double vx = obstacle.twist.linear.x;
    const double vy = obstacle.twist.linear.y;
    const double v_abs = std::hypot(vx, vy);
    const double scale_gain = 0.5;
    geometry_msgs::msg::Point p1;
    p1.x = p0.x + scale_gain * vx;
    p1.y = p0.y + scale_gain * vy;
    p1.z = p0.z;

    arrow_marker.points.clear();
    arrow_marker.points.push_back(p0);
    arrow_marker.points.push_back(p1);

    arrow_marker.scale.x = 0.05;  // 杆粗
    arrow_marker.scale.y = 0.1;   // 头宽
    arrow_marker.scale.z = 0.3;   // 头长比例
    arrow_marker.color.r = 1.0f;
    arrow_marker.color.g = 0.8f;
    arrow_marker.color.b = 0.0f;
    arrow_marker.color.a = v_abs > 1e-3 ? 0.9f : 0.0f;  // 静止时不显示箭头

    marker_array.markers.push_back(arrow_marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray DynamicObstacleTextMarker(
  lidar_detection::msg::ObstacleDetectionArray DynamicObstacleMsg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < DynamicObstacleMsg.detections.size(); ++i) {
    const auto & obstacle = DynamicObstacleMsg.detections[i];

    visualization_msgs::msg::Marker marker;
    marker.header = obstacle.detection.header;
    marker.ns = "dynamic_obstacle_text";
    marker.id = std::stoi(obstacle.id);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = false;

    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 90000000;

    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    marker.pose = obstacle.detection.bbox.center;
    marker.pose.position.z += std::max(0.2, obstacle.detection.bbox.size.z * 0.6);

    marker.scale.z = 0.3;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    const double vx = obstacle.twist.linear.x;
    const double vy = obstacle.twist.linear.y;
    const double v_abs = std::hypot(vx, vy);
    std::ostringstream oss;
    oss << "ID:" << obstacle.id << " v:" << std::fixed << std::setprecision(2) << v_abs << " m/s";
    marker.text = oss.str();

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray StaticObstacleCubeMarker(
  lidar_detection::msg::ObstacleDetectionArray StaticObstacleMsg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < StaticObstacleMsg.detections.size(); ++i) {
    const auto & obstacle = StaticObstacleMsg.detections[i];

    visualization_msgs::msg::Marker marker;
    marker.header = obstacle.detection.header;
    marker.ns = "static_obstacle_bboxes";
    marker.id = std::stoi(obstacle.id);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = false;

    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 90000000;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose = obstacle.detection.bbox.center;
    marker.scale.x = obstacle.detection.bbox.size.x;
    marker.scale.y = obstacle.detection.bbox.size.y;
    marker.scale.z = obstacle.detection.bbox.size.z;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

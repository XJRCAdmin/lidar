#pragma once

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <iomanip>
#include <sstream>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "lidar_detection/box.hpp"

static visualization_msgs::msg::Marker makeFootprintOrCubeMarker(
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
  m.lifetime.nanosec = 100000000;

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

static visualization_msgs::msg::Marker makeVelocityArrowMarker(
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
  m.lifetime.nanosec = 100000000;  // 0.1s

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
  m.scale.z = 0.2;   // 头长比例
  m.color.r = 1.0f;
  m.color.g = 0.8f;
  m.color.b = 0.0f;
  m.color.a = v_abs > 1e-3 ? 0.9f : 0.0f;  // 静止时不显示箭头
  return m;
}

static visualization_msgs::msg::Marker makeTextMarker(
  const Box & box, const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "obstacle_text";
  m.id = box.id;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.frame_locked = false;

  // 设置 lifetime 为 0.1s
  m.lifetime.sec = 0;
  m.lifetime.nanosec = 100000000;

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
  oss << "ID:" << box.id << " v:" << std::fixed << std::setprecision(2) << v_abs << " m/s";
  m.text = oss.str();
  return m;
}

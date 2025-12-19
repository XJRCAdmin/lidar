#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>

void ClusteringDebug(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloud_clusters, double CLUSTER_THRESH, int CLUSTER_MIN_SIZE,
  int CLUSTER_MAX_SIZE, const rclcpp::Logger & logger);

void ConvexHullDebug(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & convex_clusters, const rclcpp::Logger & logger);
#include "lidar_detection/utils.hpp"

#include <limits>
#include <rclcpp/rclcpp.hpp>

void ClusteringDebug(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloud_clusters, double CLUSTER_THRESH, int CLUSTER_MIN_SIZE,
  int CLUSTER_MAX_SIZE, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(
    logger, "Clustering result: found %zu clusters (thresh=%.3f, min=%d, max=%d)", cloud_clusters.size(),
    CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

  if (!cloud_clusters.empty()) {
    size_t min_pts = std::numeric_limits<size_t>::max();
    size_t max_pts = 0;
    size_t total_pts = 0;

    for (size_t i = 0; i < cloud_clusters.size(); ++i) {
      size_t cluster_size = cloud_clusters[i]->size();
      min_pts = std::min(min_pts, cluster_size);
      max_pts = std::max(max_pts, cluster_size);
      total_pts += cluster_size;

      RCLCPP_DEBUG(logger, "Cluster[%zu]: %zu points", i, cluster_size);
    }

    double avg_pts = static_cast<double>(total_pts) / cloud_clusters.size();
    RCLCPP_INFO(
      logger, "Cluster stats - total_pts=%zu, min=%zu, max=%zu, avg=%.1f", total_pts, min_pts, max_pts, avg_pts);
  }
}

void ConvexHullDebug(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & convex_clusters, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "ConvexHull computation: hulls %zu", convex_clusters.size());

  if (!convex_clusters.empty()) {
    size_t min_hull_pts = std::numeric_limits<size_t>::max();
    size_t max_hull_pts = 0;
    size_t total_hull_pts = 0;
    size_t empty_hulls = 0;

    for (size_t i = 0; i < convex_clusters.size(); ++i) {
      size_t hull_size = convex_clusters[i]->size();

      if (hull_size == 0) {
        empty_hulls++;
        RCLCPP_DEBUG(logger, "ConvexHull[%zu]: EMPTY", i);
        continue;
      }

      min_hull_pts = std::min(min_hull_pts, hull_size);
      max_hull_pts = std::max(max_hull_pts, hull_size);
      total_hull_pts += hull_size;

      RCLCPP_DEBUG(logger, "ConvexHull[%zu]: %zu points", i, hull_size);

      if (hull_size > 8) {
        RCLCPP_DEBUG(logger, "ConvexHull[%zu] has %zu points (>8)", i, hull_size);
      }
    }

    size_t valid_hulls = convex_clusters.size() - empty_hulls;
    if (valid_hulls > 0) {
      double avg = static_cast<double>(total_hull_pts) / valid_hulls;
      RCLCPP_INFO(
        logger, "ConvexHull stats - valid=%zu, empty=%zu, total_pts=%zu, min=%zu, max=%zu, avg=%.1f", valid_hulls,
        empty_hulls, total_hull_pts, min_hull_pts == std::numeric_limits<size_t>::max() ? 0 : min_hull_pts,
        max_hull_pts, avg);
    } else {
      RCLCPP_WARN(logger, "All convex hulls are empty!");
    }
  }
}

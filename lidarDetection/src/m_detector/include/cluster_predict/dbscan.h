#ifndef DBSCAN_H
#define DBSCAN_H

// Converted for ROS2 (rclcpp). The core algorithm is unchanged, but includes
// and some types were modernized to compile cleanly under ROS2/PCL.

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/common.h>

#include <pcl/features/normal_3d.h>

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>
#include <cstddef>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class DBSCANSimpleCluster {
public:
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    using KdTreePtr = typename pcl::search::KdTree<PointT>::Ptr;

    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method_ = tree;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        if (!input_cloud_ || input_cloud_->points.empty()) {
            return;
        }

        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);

        const std::size_t cloud_size = input_cloud_->points.size();

        for (std::size_t i = 0; i < cloud_size; ++i) {
            if (types[i] == PROCESSED) {
                continue;
            }

            int nn_size = radiusSearch(static_cast<int>(i), eps_, nn_indices, nn_distances);
            if (nn_size < minPts_) {
                is_noise[i] = true;
                continue;
            }

            std::vector<int> seed_queue;
            seed_queue.reserve(static_cast<std::size_t>(nn_size) + 1);
            seed_queue.push_back(static_cast<int>(i));
            types[i] = PROCESSED;

            for (int j = 0; j < nn_size; ++j) {
                if (nn_indices[j] != static_cast<int>(i)) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            }

            std::size_t sq_idx = 1;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
                    types[cloud_index] = PROCESSED;
                    ++sq_idx;
                    continue;
                }

                nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; ++j) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }

                types[cloud_index] = PROCESSED;
                ++sq_idx;
            }

            if (static_cast<int>(seed_queue.size()) >= min_pts_per_cluster_ &&
                static_cast<int>(seed_queue.size()) <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (std::size_t j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                std::sort(r.indices.begin(), r.indices.end());
                r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

                // Keep the PCL header from input cloud (if any)
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
                r.header = input_cloud_->header;
#endif
                cluster_indices.push_back(r);
            }
        }

        std::sort(cluster_indices.rbegin(), cluster_indices.rend(), comparePointClusters);
    }

    void setClusterTolerance(double tolerance) {
        eps_ = tolerance;
    }

    void setMinClusterSize (int min_cluster_size) {
        min_pts_per_cluster_ = min_cluster_size;
    }

    void setMaxClusterSize (int max_cluster_size) {
        max_pts_per_cluster_ = max_cluster_size;
    }

    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;

    double eps_ {0.0};
    int minPts_ {1};
    int min_pts_per_cluster_ {1};
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};

    KdTreePtr search_method_;

    // Simple brute-force radius search (used if user did not set a KdTree).
    // Returns the number of neighbors found and fills k_indices and k_sqr_distances.
    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();

        if (!input_cloud_) {
            return 0;
        }

        k_indices.push_back(index);
        k_sqr_distances.push_back(0.0f);

        const std::size_t size = input_cloud_->points.size();
        const double radius_square = radius * radius;

        for (std::size_t i = 0; i < size; ++i) {
            if (static_cast<int>(i) == index) {
                continue;
            }
            const double dx = static_cast<double>(input_cloud_->points[i].x) - static_cast<double>(input_cloud_->points[index].x);
            const double dy = static_cast<double>(input_cloud_->points[i].y) - static_cast<double>(input_cloud_->points[index].y);
            const double dz = static_cast<double>(input_cloud_->points[i].z) - static_cast<double>(input_cloud_->points[index].z);
            const double dist2 = dx * dx + dy * dy + dz * dz;
            if (dist2 <= radius_square) {
                k_indices.push_back(static_cast<int>(i));
                k_sqr_distances.push_back(static_cast<float>(std::sqrt(dist2)));
            }
        }
        return static_cast<int>(k_indices.size());
    }
};

#endif // DBSCAN_H

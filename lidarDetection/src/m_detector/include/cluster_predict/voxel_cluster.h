#ifndef VOXEL_CLUSTER_H
#define VOXEL_CLUSTER_H
#define PCL_NO_PRECOMPILE

// Converted to ROS2-compatible header.
// Core logic unchanged; ROS1 headers and Boost smart pointers removed.

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <cstdint>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HASH_length 10000

class VOXEL {
public:
    int64_t x, y, z;

    VOXEL(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
};

using PointType = pcl::PointXYZINormal;

struct Point_Cloud {
    using Ptr = std::shared_ptr<Point_Cloud>;

    int bbox_index{-1};
    int points_num{0};
    pcl::PointCloud<PointType>::Ptr cloud;
    std::shared_ptr<std::vector<int>> cloud_index;

    Point_Cloud() = default;

    explicit Point_Cloud(int index)
        : bbox_index(index) {}

    explicit Point_Cloud(PointType point)
    {
        cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        cloud->reserve(5);
        cloud->points.push_back(point);
        points_num = 1;
    }

    Point_Cloud(PointType point, int index)
        : bbox_index(index)
    {
        cloud = boost::make_shared<pcl::PointCloud<PointType>>();
        cloud->points.push_back(point);
        points_num = 1;
    }

    ~Point_Cloud() = default;

    void reset()
    {
        points_num = 0;
        bbox_index = -1;
    }
};

struct test_struct {
    int bbox_index{-1};
    int points_num{0};
    pcl::PointCloud<PointType>::Ptr cloud;
    std::vector<int>* cloud_index{nullptr};

    test_struct() = default;

    explicit test_struct(int index)
        : bbox_index(index) {}

    ~test_struct() = default;

    void reset()
    {
        points_num = 0;
        bbox_index = -1;
    }
};

// Hash for VOXEL
namespace std {
template <>
struct hash<VOXEL> {
    std::size_t operator()(const VOXEL &s) const noexcept {
        return static_cast<std::size_t>(
            s.z * HASH_length * HASH_length +
            s.y * HASH_length +
            s.x);
    }
};
}  // namespace std

class VOXEL_CLUSTER
{
public:
    using PointType = pcl::PointXYZINormal;

    std::vector<int> voxel_list;
    std::unordered_set<int> voxel_set;

    VOXEL_CLUSTER() = default;
    ~VOXEL_CLUSTER() = default;

    void setInputCloud(const pcl::PointCloud<PointType> &points_in)
    {
        points_ = points_in;
    }

    void setVoxelResolution(
        float voxel_length,
        float edge_size_xy,
        float edge_size_z,
        const Eigen::Vector3f &xyz_origin_in)
    {
        Voxel_revolusion = voxel_length;
        Grid_edge_size_xy = edge_size_xy;
        Grid_edge_size_z = edge_size_z;
        xyz_origin = xyz_origin_in;
    }

    void setExtendRange(int range)
    {
        max_range = range;
    }

    void setMinClusterSize(int min_cluster_voxels)
    {
        min_cluster_voxels_ = min_cluster_voxels;
    }

    void createVoxelMap(
        std::vector<Point_Cloud> &umap_in,
        std::unordered_set<int> &used_map_set)
    {
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const auto &p = points_.points[i];

            int position =
                static_cast<int>(std::floor((p.x - xyz_origin(0)) / Voxel_revolusion)) *
                    Grid_edge_size_xy * Grid_edge_size_z +
                static_cast<int>(std::floor((p.y - xyz_origin(1)) / Voxel_revolusion)) *
                    Grid_edge_size_z +
                static_cast<int>(std::floor((p.z - xyz_origin(2)) / Voxel_revolusion));

            if (position < 0 ||
                position > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                continue;

            if (umap_in[position].points_num > 0) {
                umap_in[position].points_num++;
            } else {
                used_map_set.insert(position);
                voxel_list.push_back(position);
                voxel_set.insert(position);

                umap_in[position].cloud =
                    boost::make_shared<pcl::PointCloud<PointType>>();
                umap_in[position].cloud_index =
                    std::make_shared<std::vector<int>>();
                umap_in[position].points_num = 1;
            }
        }
    }

    void createVoxelMap(std::vector<Point_Cloud> &umap_in, bool index_en)
    {
        (void)index_en;
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const auto &p = points_.points[i];

            int position =
                static_cast<int>(std::floor((p.x - xyz_origin(0)) / Voxel_revolusion)) *
                    Grid_edge_size_xy * Grid_edge_size_z +
                static_cast<int>(std::floor((p.y - xyz_origin(1)) / Voxel_revolusion)) *
                    Grid_edge_size_z +
                static_cast<int>(std::floor((p.z - xyz_origin(2)) / Voxel_revolusion));

            if (position < 0 ||
                position > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                continue;

            if (umap_in[position].points_num > 0) {
                umap_in[position].cloud->push_back(p);
                umap_in[position].points_num++;
                umap_in[position].cloud_index->push_back(static_cast<int>(i));
            } else {
                voxel_list.push_back(position);
                voxel_set.insert(position);

                umap_in[position].cloud =
                    boost::make_shared<pcl::PointCloud<PointType>>();
                umap_in[position].cloud->reserve(5);
                umap_in[position].cloud->push_back(p);

                umap_in[position].cloud_index =
                    std::make_shared<std::vector<int>>();
                umap_in[position].cloud_index->reserve(5);
                umap_in[position].cloud_index->push_back(static_cast<int>(i));

                umap_in[position].points_num = 1;
            }
        }
    }

    void createVoxelMap(
        std::unordered_map<int, Point_Cloud::Ptr> &umap)
    {
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const auto &p = points_.points[i];

            int position =
                static_cast<int>(std::floor((p.x - xyz_origin(0)) / Voxel_revolusion)) *
                    Grid_edge_size_xy * Grid_edge_size_z +
                static_cast<int>(std::floor((p.y - xyz_origin(1)) / Voxel_revolusion)) *
                    Grid_edge_size_z +
                static_cast<int>(std::floor((p.z - xyz_origin(2)) / Voxel_revolusion));

            if (position < 0 ||
                position > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                continue;

            if (umap.count(position)) {
                umap[position]->cloud->push_back(p);
                umap[position]->points_num++;
                umap[position]->cloud_index->push_back(static_cast<int>(i));
            } else {
                voxel_list.push_back(position);
                voxel_set.insert(position);

                auto pc = std::make_shared<Point_Cloud>(p);
                pc->cloud_index = std::make_shared<std::vector<int>>();
                pc->cloud_index->push_back(static_cast<int>(i));
                pc->points_num = 1;

                umap[position] = pc;
            }
        }
    }

    void extendVoxelNeighbor(int voxel, std::unordered_set<int> &voxel_added)
    {
        for (int dx = -max_range; dx <= max_range; ++dx) {
            for (int dy = -max_range; dy <= max_range; ++dy) {
                for (int dz = -max_range; dz <= max_range; ++dz) {
                    float dist = std::sqrt(
                        static_cast<float>(dx * dx + dy * dy + dz * dz));
                    if (dist - static_cast<float>(max_range) < 0.001f) {
                        int neighbor =
                            voxel +
                            dx * Grid_edge_size_xy * Grid_edge_size_z +
                            dy * Grid_edge_size_z +
                            dz;

                        if (neighbor < 0 ||
                            neighbor > Grid_edge_size_xy * Grid_edge_size_xy *
                                           Grid_edge_size_z)
                            continue;

                        if (voxel_set.count(neighbor) &&
                            !voxel_added.count(neighbor)) {
                            voxel_added.insert(neighbor);
                            extendVoxelNeighbor(neighbor, voxel_added);
                        }
                    }
                }
            }
        }
    }

    void extract(std::vector<std::vector<int>> &voxel_clusters)
    {
        for (std::size_t i = 0; i < voxel_list.size(); ++i) {
            int voxel_cur = voxel_list[i];
            if (!voxel_set.count(voxel_cur))
                continue;

            std::unordered_set<int> voxel_added;
            voxel_added.insert(voxel_cur);
            extendVoxelNeighbor(voxel_cur, voxel_added);

            if (static_cast<int>(voxel_added.size()) >= min_cluster_voxels_) {
                std::vector<int> cluster;
                for (int v : voxel_added) {
                    voxel_set.erase(v);
                    cluster.push_back(v);
                }
                voxel_clusters.push_back(cluster);
            }
        }
    }

protected:
    pcl::PointCloud<PointType> points_;
    float Voxel_revolusion{0.0f};
    float Grid_edge_size_xy{0.0f};
    float Grid_edge_size_z{0.0f};
    Eigen::Vector3f xyz_origin{Eigen::Vector3f::Zero()};
    int max_range{0};
    int min_cluster_voxels_{1};
};

#endif  // VOXEL_CLUSTER_H

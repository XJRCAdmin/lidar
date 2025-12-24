#include <m_detector/DynObjCluster.h>
#include <cluster_predict/EA_disk.h>
#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_set>

namespace std_msgs { using Header = msg::Header; }
namespace geometry_msgs { using PoseWithCovarianceStamped = msg::PoseWithCovarianceStamped; }
namespace visualization_msgs { using MarkerArray = msg::MarkerArray; }
namespace sensor_msgs { using PointCloud2 = msg::PointCloud2; }

static rclcpp::Clock g_clock(RCL_SYSTEM_TIME);

void DynObjCluster::Init()
{
    xyz_origin << -100., -100., -20.;
    maprange << 200., 200., 40.;
    GridMapedgesize_xy = ceil(maprange(0) / Voxel_revolusion);
    GridMapedgesize_z = ceil(maprange(2) / Voxel_revolusion);
    GridMapsize = GridMapedgesize_xy * GridMapedgesize_xy * GridMapedgesize_z;
    std::cout << "clustering init begin, please wait------------" << GridMapsize << std::endl;
    umap.reserve(GridMapsize);
    umap.resize(GridMapsize);
    umap_ground.reserve(GridMapsize);
    umap_ground.resize(GridMapsize);
    umap_insidebox.reserve(GridMapsize);
    umap_insidebox.resize(GridMapsize);
    std::cout << "clustering init finish------------" << std::endl;
    if(out_file != "") out.open(out_file, std::ios::out  | std::ios::binary);
}

void DynObjCluster::Clusterprocess(std::vector<int> &dyn_tag, pcl::PointCloud<PointType> event_point, const pcl::PointCloud<PointType> &raw_point, const std_msgs::Header &header_in, const Eigen::Matrix3d odom_rot_in, const Eigen::Vector3d odom_pos_in)
{
    cluster_begin = g_clock.now();
    header = header_in;
    odom_rot = odom_rot_in;
    odom_pos = odom_pos_in;
    auto t0 = g_clock.now();
    float delta_t = 0.1;
    pcl::PointCloud<PointType> extend_points;
    pcl::PointCloud<PointType>::Ptr cloud_clean_ptr(new pcl::PointCloud<PointType>);
    cloud_clean_ptr = event_point.makeShared();
    bbox_t bbox_high;
    ClusterAndTrack(dyn_tag, cloud_clean_ptr, pub_pcl_before_high, header, pub_pcl_after_high, cluster_vis_high, predict_path_high, bbox_high, delta_t, raw_point);
    time_total = (g_clock.now() - t0).seconds();
    time_ind++;
    time_total_average = time_total_average * (time_ind - 1) / time_ind + time_total / time_ind;
    cur_frame += 1;
}

void DynObjCluster::ClusterAndTrack(
    std::vector<int>& dyn_tag,
    pcl::PointCloud<PointType>::Ptr& points_in,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_in_pub,
    std_msgs::msg::Header header_in,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub,
    bbox_t& bbox,
    double time,
    const pcl::PointCloud<PointType>& cloud)             
{
    sensor_msgs::msg::PointCloud2 pcl4_ros_msg;
    pcl::toROSMsg(*points_in, pcl4_ros_msg);
    pcl4_ros_msg.header.stamp = header_in.stamp;
    pcl4_ros_msg.header.frame_id = header_in.frame_id;
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<std::vector<int>> voxel_clusters;
    std::unordered_set<int> used_map_set;
    GetClusterResult_voxel(points_in, umap, voxel_clusters, used_map_set);
    PubClusterResult_voxel(dyn_tag, header_in, bbox, time, voxel_clusters, cloud, used_map_set);
}

void DynObjCluster::GetClusterResult(pcl::PointCloud<PointType>::Ptr points_in, std::vector<pcl::PointIndices> &cluster_indices)
{
    if (points_in->size() < 2)
    {
        return;
    }
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(points_in);
    DBSCANKdtreeCluster<PointType> ec;
    ec.setCorePointMinPts(nn_points_size);
    ec.setClusterTolerance(nn_points_radius);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(points_in);
    ec.extract(cluster_indices);
}

void DynObjCluster::GetClusterResult_voxel(pcl::PointCloud<PointType>::Ptr points_in, std::vector<Point_Cloud> &umap_in, std::vector<std::vector<int>> &voxel_clusters, std::unordered_set<int> &used_map_set)
{
    auto t0 = g_clock.now();
    if ((out_file != "") && points_in->size() < 2)
    {   
        out << (g_clock.now() - t0).seconds() << " ";
        return;
    }
    VOXEL_CLUSTER cluster;
    cluster.setInputCloud(*points_in);
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin);
    cluster.setExtendRange(cluster_extend_pixel);
    cluster.setMinClusterSize(cluster_min_pixel_number);
    cluster.createVoxelMap(umap_in, used_map_set);
    cluster.extract(voxel_clusters);
    if(out_file != "") out << (g_clock.now() - t0).seconds() << " ";
}

void DynObjCluster::PubClusterResult_voxel(
    std::vector<int> &dyn_tag,
    const std_msgs::msg::Header &current_header,
    bbox_t &bbox,
    double delta,
    std::vector<std::vector<int>> &voxel_clusters,
    const pcl::PointCloud<PointType> &raw_point,
    std::unordered_set<int> &used_map_set)
{
    int j = 0;
    pcl::PointCloud<PointType> cluster_points;
    pcl::PointCloud<PointType> true_ground;

    visualization_msgs::msg::MarkerArray numbers;
    numbers.markers.reserve(200);

    cluster_points.reserve(raw_point.size());
    true_ground.reserve(raw_point.size());

    Eigen::Matrix3f R = odom_rot.cast<float>();
    Eigen::Vector3f world_z = R.col(2);

    for (auto it = voxel_clusters.begin(); it != voxel_clusters.end(); ++it, ++j)
    {
        Eigen::Vector3f xyz;
        XYZExtract(*(it->begin()), xyz);

        float x_min = xyz(0), x_max = xyz(0);
        float y_min = xyz(1), y_max = xyz(1);
        float z_min = xyz(2), z_max = xyz(2);

        int n = 0;
        for (auto pit = it->begin(); pit != it->end(); ++pit)
        {
            int voxel = *pit;
            umap[voxel].bbox_index = j;
            n += umap[voxel].points_num;

            XYZExtract(voxel, xyz);
            x_min = std::min(x_min, xyz(0));
            y_min = std::min(y_min, xyz(1));
            z_min = std::min(z_min, xyz(2));

            x_max = std::max(x_max, xyz(0) + Voxel_revolusion);
            y_max = std::max(y_max, xyz(1) + Voxel_revolusion);
            z_max = std::max(z_max, xyz(2) + Voxel_revolusion);
        }

        float x_size = x_max - x_min;
        float y_size = y_max - y_min;
        float z_size = z_max - z_min;

        if (cluster_min_pixel_number == 1 ||
            (x_size > Voxel_revolusion && y_size > Voxel_revolusion) ||
            (x_size > Voxel_revolusion && z_size > Voxel_revolusion) ||
            (y_size > Voxel_revolusion && z_size > Voxel_revolusion))
        {
            bbox.Point_cloud.emplace_back();
            bbox.Point_indices.emplace_back();

            geometry_msgs::msg::PoseWithCovarianceStamped center;
            center.header = current_header;

            center.pose.pose.position.x = (x_min + x_max) * 0.5f;
            center.pose.pose.position.y = (y_min + y_max) * 0.5f;
            center.pose.pose.position.z = (z_min + z_max) * 0.5f;
            center.pose.pose.orientation.w = 1.0;

            auto &cov = center.pose.covariance;
            cov[0] = x_size * 0.5;
            cov[7] = y_size * 0.5;
            cov[14] = z_size * 0.5;
            cov[21] = x_size;
            cov[28] = y_size;
            cov[35] = z_size;

            cov[15] = x_max; cov[20] = y_max; cov[25] = z_max;
            cov[18] = x_min; cov[23] = y_min; cov[29] = z_min;

            bbox.Center.push_back(center);
            bbox.Ground_points.emplace_back();
            bbox.true_ground.emplace_back();
            bbox.Ground_voxels_set.emplace_back();
            bbox.Ground_voxels_vec.emplace_back();
            bbox.umap_points_num.push_back(n);
        }
        else
        {
            j--;
            for (int voxel : *it)
                umap[voxel].reset();
        }
    }

    std::vector<int> index_bbox(bbox.Center.size());
    std::iota(index_bbox.begin(), index_bbox.end(), 0);

    std::vector<std::unordered_set<int>> used_map_set_vec(bbox.Center.size());

    for (int ite = 0; ite < raw_point.size(); ite++)
    {
        if (dyn_tag[ite] == -1) continue;

        int voxel =
            floor((raw_point[ite].x - xyz_origin(0)) / Voxel_revolusion) *
                GridMapedgesize_xy * GridMapedgesize_z +
            floor((raw_point[ite].y - xyz_origin(1)) / Voxel_revolusion) *
                GridMapedgesize_z +
            floor((raw_point[ite].z - xyz_origin(2)) / Voxel_revolusion);

        if (voxel < 0 || voxel > GridMapsize) continue;

        if (umap_ground[voxel].bbox_index >= 0)
        {
            int k = umap_ground[voxel].bbox_index;
            bbox.Ground_points[k].push_back(raw_point[ite]);
            dyn_tag[ite] = 0;
        }
        else if (umap[voxel].bbox_index >= 0)
        {
            int k = umap[voxel].bbox_index;
            auto tmp = raw_point[ite];
            tmp.curvature = ite;
            bbox.Point_cloud[k].push_back(tmp);
            bbox.Point_indices[k].push_back(ite);
            dyn_tag[ite] = 1;
        }
        else
        {
            dyn_tag[ite] = 0;
        }
    }

    for (int k = 0; k < bbox.Center.size(); k++)
    {
        Eigen::Vector3f ground_norm;
        Eigen::Vector4f ground_plane;

        auto t_ge = g_clock.now();
        bool ground_detect = ground_estimate(
            bbox.Ground_points[k], world_z, ground_norm,
            ground_plane, bbox.true_ground[k],
            bbox.Ground_voxels_set[k]);

        if (ground_detect)
        {
            event_extend(Eigen::Matrix3f::Identity(), true, bbox, dyn_tag, k);
            ground_remove(
                ground_plane,
                bbox.Point_cloud[k],
                bbox.Point_indices[k],
                dyn_tag,
                bbox.true_ground[k],
                umap);
        }

        isolate_remove(bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag);

        if ((float)bbox.umap_points_num[k] /
                (float)bbox.Point_cloud[k].size() <
            thrustable_thresold)
        {
            for (int id : bbox.Point_indices[k])
                dyn_tag[id] = 0;
        }
        else
        {
            cluster_points += bbox.Point_cloud[k];
            true_ground += bbox.true_ground[k];
        }
    }

    for (int voxel : used_map_set)
    {
        umap[voxel].reset();
        umap_ground[voxel].reset();
        umap_insidebox[voxel].reset();
    }
}

bool DynObjCluster::ground_estimate(
    const pcl::PointCloud<PointType> &ground_pcl,
    const Eigen::Vector3f &world_z,
    Eigen::Vector3f &ground_norm,
    Eigen::Vector4f &ground_plane,
    pcl::PointCloud<PointType> &true_ground,
    std::unordered_set<int> &extend_pixels)
{
    if (ground_pcl.empty())
        return false;

    int BNUM = std::max(4, static_cast<int>(ground_pcl.size()) / 100);
    const float threshold = 0.10f;
    const float max_angle_from_body = 30.0f / 57.3f;

    pcl::PointCloud<PointType> split_pcl;
    int max_count = 0;
    Eigen::Vector3f max_normvec(0, 0, 0);
    pcl::PointCloud<PointType> max_points;

    for (int i = 0; i < ground_pcl.size(); i++)
    {
        split_pcl.push_back(ground_pcl[i]);
        if (split_pcl.size() == BNUM)
        {
            Eigen::Vector4f plane;
            if (esti_plane(plane, split_pcl) && plane[3] < threshold)
            {
                Eigen::Vector3f normvec = plane.head<3>().normalized();
                if (normvec.cross(world_z).norm() < std::sin(max_angle_from_body))
                {
                    int count = 0;
                    pcl::PointCloud<PointType> tmp_points;
                    for (int j = 0; j < ground_pcl.size(); j++)
                    {
                        Eigen::Vector3f p(
                            ground_pcl[j].x,
                            ground_pcl[j].y,
                            ground_pcl[j].z);

                        float dis = std::fabs(p.dot(plane.head<3>()) + 1.0f) /
                                    plane.head<3>().norm();
                        if (dis < threshold)
                        {
                            tmp_points.push_back(ground_pcl[j]);
                            count++;
                        }
                    }

                    if (count > max_count)
                    {
                        max_count = count;
                        max_normvec = normvec;
                        ground_plane = plane;
                        max_points = tmp_points;

                        if (max_count > 0.6f * ground_pcl.size())
                            break;
                    }
                }
            }
            split_pcl.clear();
        }
    }

    if (ground_pcl.size() > 0 &&
        (max_count > 0.2f * ground_pcl.size() || max_count > 500))
    {
        Eigen::Vector4f plane;
        if (esti_plane(plane, max_points) && plane[3] < threshold)
        {
            Eigen::Vector3f normvec = plane.head<3>().normalized();
            if (normvec.cross(world_z).norm() < std::sin(max_angle_from_body))
            {
                max_normvec = normvec;
                ground_plane = plane;
            }
        }

        for (int j = 0; j < ground_pcl.size(); j++)
        {
            Eigen::Vector3f p(
                ground_pcl[j].x,
                ground_pcl[j].y,
                ground_pcl[j].z);

            float dis = std::fabs(p.dot(ground_plane.head<3>()) + 1.0f) /
                        ground_plane.head<3>().norm();
            if (dis < threshold)
            {
                true_ground.push_back(ground_pcl[j]);

                int voxel =
                    static_cast<int>(std::floor((ground_pcl[j].x - xyz_origin(0)) / Voxel_revolusion)) *
                        GridMapedgesize_xy * GridMapedgesize_z +
                    static_cast<int>(std::floor((ground_pcl[j].y - xyz_origin(1)) / Voxel_revolusion)) *
                        GridMapedgesize_z +
                    static_cast<int>(std::floor((ground_pcl[j].z - xyz_origin(2)) / Voxel_revolusion));

                extend_pixels.erase(voxel);
            }
        }

        if (max_normvec[2] < 0)
            max_normvec *= -1.0f;

        ground_norm = max_normvec;
    }

    return std::fabs(ground_norm.norm() - 1.0f) < 0.1f;
}

void DynObjCluster::ground_remove(
    const Eigen::Vector4f &ground_plane,
    pcl::PointCloud<PointType> &cluster_pcl,
    std::vector<int> &cluster_pcl_ind,
    std::vector<int> &dyn_tag,
    pcl::PointCloud<PointType> &true_ground,
    std::vector<Point_Cloud> &umap)
{
    const float threshold = 0.10f;

    pcl::PointCloud<PointType> new_cluster_pcl;
    std::vector<int> new_cluster_pcl_ind;

    for (int i = 0; i < cluster_pcl.size(); i++)
    {
        Eigen::Vector3f p(
            cluster_pcl[i].x,
            cluster_pcl[i].y,
            cluster_pcl[i].z);

        float dis = std::fabs(p.dot(ground_plane.head<3>()) + 1.0f) /
                    ground_plane.head<3>().norm();

        if (dis > threshold)
        {
            new_cluster_pcl.push_back(cluster_pcl[i]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[i]);
        }
        else
        {
            dyn_tag[cluster_pcl_ind[i]] = 0;
            true_ground.push_back(cluster_pcl[i]);
        }
    }

    cluster_pcl.swap(new_cluster_pcl);
    cluster_pcl_ind.swap(new_cluster_pcl_ind);
}

void DynObjCluster::isolate_remove(
    pcl::PointCloud<PointType> &cluster_pcl,
    std::vector<int> &cluster_pcl_ind,
    std::vector<int> &dyn_tag)
{
    if (cluster_pcl.size() < 2)
        return;

    pcl::PointCloud<PointType> new_cluster_pcl;
    std::vector<int> new_cluster_pcl_ind;

    VOXEL_CLUSTER cluster;
    std::unordered_map<int, Point_Cloud::Ptr> umap_cluster;
    std::vector<std::vector<int>> voxel_cluster;

    cluster.setInputCloud(cluster_pcl);
    cluster.setVoxelResolution(
        Voxel_revolusion,
        GridMapedgesize_xy,
        GridMapedgesize_z,
        xyz_origin);
    cluster.setExtendRange(cluster_extend_pixel);
    cluster.setMinClusterSize(cluster_min_pixel_number);

    cluster.createVoxelMap(umap_cluster);
    cluster.extract(voxel_cluster);

    int max_cluster_ind = 0;
    int max_voxel_num = 0;
    for (int i = 0; i < voxel_cluster.size(); i++)
    {
        if (voxel_cluster[i].size() > max_voxel_num)
        {
            max_voxel_num = voxel_cluster[i].size();
            max_cluster_ind = i;
        }
    }

    std::unordered_set<int> dyn_index;
    for (int i = 0; i < max_voxel_num; i++)
    {
        int voxel = voxel_cluster[max_cluster_ind][i];
        auto &cloud = umap_cluster[voxel]->cloud;
        auto &indices = *umap_cluster[voxel]->cloud_index;

        for (int j = 0; j < cloud->size(); j++)
        {
            new_cluster_pcl.push_back(cloud->points[j]);
            int idx = cluster_pcl_ind[indices[j]];
            new_cluster_pcl_ind.push_back(idx);
            dyn_index.insert(idx);
        }
    }

    for (int i = 0; i < cluster_pcl_ind.size(); i++)
    {
        if (!dyn_index.count(cluster_pcl_ind[i]))
            dyn_tag[cluster_pcl_ind[i]] = 0;
    }

    umap_cluster.clear();
    cluster_pcl.swap(new_cluster_pcl);
    cluster_pcl_ind.swap(new_cluster_pcl_ind);
}

void DynObjCluster::oobb_estimate(
    const VoxelMap &vmap,
    const pcl::PointCloud<PointType> &points,
    Eigen::Vector3f &min_point_obj,
    Eigen::Vector3f &max_point_obj,
    Eigen::Matrix3f &R,
    const Eigen::Vector3f ground_norm)
{
    int NMATCH = 5;
    int n = 3;
    EA_disk disk(n);

    std::vector<std::vector<Eigen::Vector4f>> NormVectorMap(disk.size);
    std::vector<std::vector<int>> PointSizeList(disk.size);

    for (int i = 0; i < vmap.size(); i++)
    {
        if (!vmap[i].empty() && vmap[i].points.size() >= NMATCH)
        {
            Eigen::Vector4f plane;
            if (esti_plane(plane, vmap[i]))
            {
                plane.head<3>().normalize();
                if (plane[2] < 0)
                    plane.head<3>() *= -1.0f;

                Eigen::Vector2f sphere_coor, disk_coor;
                disk.CatesianToSphere(plane.head<3>(), sphere_coor);
                disk.SphereToDisk(sphere_coor, disk_coor);

                int index = disk.index_find(disk_coor);
                if (index > std::pow(2 * (n - 1) + 1, 2) + 4 * n)
                {
                    index -= 4 * n;
                    plane.head<3>() *= -1.0f;
                }
                NormVectorMap[index].push_back(plane);
                PointSizeList[index].push_back(vmap[i].size());
            }
        }
    }

    int max_ind = 0, sec_ind = 0;
    float max_award = 0.0f, sec_award = 0.0f;

    for (int i = 0; i < NormVectorMap.size(); i++)
    {
        if (!NormVectorMap[i].empty())
        {
            float award = 0.0f;
            for (int k = 0; k < NormVectorMap[i].size(); k++)
                award += std::sqrt(PointSizeList;

            if (award > max_award)
            {
                sec_award = max_award;
                sec_ind = max_ind;
                max_award = award;
                max_ind = i;
            }
            else if (award > sec_award)
            {
                sec_award = award;
                sec_ind = i;
            }
        }
    }

    Eigen::Vector3f direction_main(0, 0, 1);
    Eigen::Vector3f direction_aux(1, 0, 0);

    if (max_award > 0)
    {
        direction_main.setZero();
        for (int k = 0; k < NormVectorMap[max_ind].size(); k++)
            direction_main += NormVectorMap[max_ind][k].head<3>() *
                              PointSizeList[max_ind][k] /
                              NormVectorMap;
        direction_main.normalize();
    }

    if (sec_award > 0)
    {
        direction_aux.setZero();
        for (int k = 0; k < NormVectorMap[sec_ind].size(); k++)
            direction_aux += NormVectorMap[sec_ind][k].head<3>() *
                             PointSizeList[sec_ind][k] /
                             NormVectorMap;
        direction_aux.normalize();
    }

    if (ground_norm.norm() < 0.1f)
    {
        R.col(0) = direction_main;
        R.col(1) = (direction_aux - direction_aux.dot(R.col(0)) * R.col(0)).normalized();

        Eigen::Vector3f world_z(0, 0, 1);
        if (std::abs(R.col(1).dot(world_z)) > 0.866f)
        {
            R.col(2) = R.col(1);
            R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
        }
        else if (std::abs(R.col(0).dot(world_z)) > 0.866f)
        {
            R.col(1).swap(R.col(0));
            R.col(2) = R.col(1);
            R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
        }
        else
        {
            R.col(2) = (R.col(0).cross(R.col(1))).normalized();
        }
    }
    else
    {
        R.col(0) = ground_norm;
        if (ground_norm.dot(direction_main) > 0.95f)
            direction_main = direction_aux;

        R.col(1) = (direction_main - direction_main.dot(R.col(0)) * R.col(0)).normalized();
        R.col(2) = (R.col(0).cross(R.col(1))).normalized();
    }

    Eigen::Vector3f p(points[0].x, points[0].y, points[0].z);
    Eigen::Vector3f proj = R.transpose() * p;

    float x_min = proj.x(), x_max = proj.x();
    float y_min = proj.y(), y_max = proj.y();
    float z_min = proj.z(), z_max = proj.z();

    for (int i = 0; i < points.size(); i++)
    {
        p << points[i].x, points[i].y, points[i].z;
        proj = R.transpose() * p;

        x_min = std::min(x_min, proj.x());
        y_min = std::min(y_min, proj.y());
        z_min = std::min(z_min, proj.z());
        x_max = std::max(x_max, proj.x());
        y_max = std::max(y_max, proj.y());
        z_max = std::max(z_max, proj.z());
    }

    min_point_obj << x_min, y_min, z_min;
    max_point_obj << x_max, y_max, z_max;
}

void DynObjCluster::event_extend(
    const Eigen::Matrix3f &R,
    bool ground_detect,
    bbox_t &bbox,
    std::vector<int> &dyn_tag,
    const int &bbox_index)
{
    for (int i = 0; i < bbox.Ground_voxels_vec[bbox_index].size(); i++)
    {
        int voxel_cur = bbox.Ground_voxels_vec[bbox_index][i];
        if (!bbox.Ground_voxels_set[bbox_index].count(voxel_cur) ||
            umap_ground[voxel_cur].points_num <= 2)
            continue;

        static const int dx[6] = {1, -1, 0, 0, 0, 0};
        static const int dy[6] = {0, 0, 1, -1, 0, 0};
        static const int dz[6] = {0, 0, 0, 0, 1, -1};

        for (int k = 0; k < 6; k++)
        {
            int nb = voxel_cur +
                     dx[k] * GridMapedgesize_xy * GridMapedgesize_z +
                     dy[k] * GridMapedgesize_z +
                     dz[k];

            if (nb < 0 || nb > GridMapsize)
                continue;

            if ((umap_insidebox[nb].bbox_index == bbox_index &&
                 umap_insidebox[nb].points_num > 0) ||
                (umap[nb].bbox_index == bbox_index &&
                 umap[nb].points_num > 0))
            {
                Eigen::Vector4f plane;
                auto &pts = *umap_ground[voxel_cur].cloud;

                if (ground_detect &&
                    esti_plane(plane, pts) &&
                    std::abs(R.col(0).dot(plane.head<3>().normalized())) < 0.8f)
                {
                    umap[voxel_cur].bbox_index = bbox_index;
                    umap[voxel_cur].cloud = umap_ground[voxel_cur].cloud;
                    bbox.Point_cloud[bbox_index] += pts;

                    for (int id : *umap_ground[voxel_cur].cloud_index)
                    {
                        dyn_tag[id] = 1;
                        bbox.Point_indices[bbox_index].push_back(id);
                    }
                    break;
                }
            }
        }
    }
}

bool DynObjCluster::esti_plane(
    Eigen::Vector4f &pca_result,
    const pcl::PointCloud<PointType> &point)
{
    const float threshold = 0.1f;
    int N = point.size();

    Eigen::MatrixXf A(N, 3);
    Eigen::VectorXf b = Eigen::VectorXf::Constant(N, -1.0f);

    for (int i = 0; i < N; i++)
    {
        A(i, 0) = point[i].x;
        A(i, 1) = point[i].y;
        A(i, 2) = point[i].z;
    }

    Eigen::Vector3f norm = A.colPivHouseholderQr().solve(b);
    float norm_len = norm.norm();
    float avg_dis = 0.0f;

    for (int i = 0; i < N; i++)
    {
        float d = std::abs(norm.dot(A.row(i)) + 1.0f);
        if (d > threshold)
            return false;
        avg_dis += d;
    }

    avg_dis = std::max(0.01f, avg_dis / (N * norm_len));
    pca_result << norm, avg_dis;
    return true;
}

void DynObjCluster::XYZExtract(const int &position, Eigen::Vector3f &xyz)
{
    int left = position;
    int xy = GridMapedgesize_xy * GridMapedgesize_z;

    xyz(0) = xyz_origin(0) +
             std::floor(position / xy) * Voxel_revolusion;
    left -= std::floor(position / xy) * xy;

    xyz(1) = xyz_origin(1) +
             std::floor(left / GridMapedgesize_z) * Voxel_revolusion;
    left -= std::floor(left / GridMapedgesize_z) * GridMapedgesize_z;

    xyz(2) = xyz_origin(2) + left * Voxel_revolusion;
}
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <m_detector/DynObjFilter.h>

void DynObjFilter::init(const rclcpp::Node::SharedPtr& node)
{
    node->declare_parameter<double>("dyn_obj/buffer_delay", 0.1);
    node->declare_parameter<int>("dyn_obj/buffer_size", 300000);
    node->declare_parameter<int>("dyn_obj/points_num_perframe", 150000);
    node->declare_parameter<double>("dyn_obj/depth_map_dur", 0.2);
    node->declare_parameter<int>("dyn_obj/max_depth_map_num", 5);
    node->declare_parameter<int>("dyn_obj/max_pixel_points", 50);
    node->declare_parameter<double>("dyn_obj/frame_dur", 0.1);
    node->declare_parameter<int>("dyn_obj/dataset", 0);
    node->declare_parameter<float>("dyn_obj/self_x_f", 0.15f);
    node->declare_parameter<float>("dyn_obj/self_x_b", 0.15f);
    node->declare_parameter<float>("dyn_obj/self_y_l", 0.15f);
    node->declare_parameter<float>("dyn_obj/self_y_r", 0.5f);
    node->declare_parameter<float>("dyn_obj/blind_dis", 0.15f);
    node->declare_parameter<float>("dyn_obj/fov_up", 0.15f);
    node->declare_parameter<float>("dyn_obj/fov_down", 0.15f);
    node->declare_parameter<float>("dyn_obj/fov_cut", 0.15f);
    node->declare_parameter<float>("dyn_obj/fov_left", 180.0f);
    node->declare_parameter<float>("dyn_obj/fov_right", -180.0f);
    node->declare_parameter<int>("dyn_obj/checkneighbor_range", 1);
    node->declare_parameter<bool>("dyn_obj/stop_object_detect", false);
    node->declare_parameter<float>("dyn_obj/depth_thr1", 0.15f);
    node->declare_parameter<float>("dyn_obj/enter_min_thr1", 0.15f);
    node->declare_parameter<float>("dyn_obj/enter_max_thr1", 0.15f);
    node->declare_parameter<float>("dyn_obj/map_cons_depth_thr1", 0.5f);
    node->declare_parameter<float>("dyn_obj/map_cons_hor_thr1", 0.01f);
    node->declare_parameter<float>("dyn_obj/map_cons_ver_thr1", 0.01f);
    node->declare_parameter<float>("dyn_obj/map_cons_hor_dis1", 0.2f);
    node->declare_parameter<float>("dyn_obj/map_cons_ver_dis1", 0.1f);
    node->declare_parameter<float>("dyn_obj/depth_cons_depth_thr1", 0.5f);
    node->declare_parameter<float>("dyn_obj/depth_cons_depth_max_thr1", 0.5f);
    node->declare_parameter<float>("dyn_obj/depth_cons_hor_thr1", 0.02f);
    node->declare_parameter<float>("dyn_obj/depth_cons_ver_thr1", 0.01f);
    node->declare_parameter<float>("dyn_obj/enlarge_z_thr1", 0.05f);
    node->declare_parameter<float>("dyn_obj/enlarge_angle", 2.0f);
    node->declare_parameter<float>("dyn_obj/enlarge_depth", 3.0f);
    node->declare_parameter<int>("dyn_obj/occluded_map_thr1", 3);
    node->declare_parameter<bool>("dyn_obj/case1_interp_en", false);
    node->declare_parameter<float>("dyn_obj/k_depth_min_thr1", 0.0f);
    node->declare_parameter<float>("dyn_obj/d_depth_min_thr1", 0.15f);
    node->declare_parameter<float>("dyn_obj/k_depth_max_thr1", 0.0f);
    node->declare_parameter<float>("dyn_obj/d_depth_max_thr1", 0.15f);
    node->declare_parameter<float>("dyn_obj/v_min_thr2", 0.5f);
    node->declare_parameter<float>("dyn_obj/acc_thr2", 1.0f);
    node->declare_parameter<float>("dyn_obj/map_cons_depth_thr2", 0.15f);
    node->declare_parameter<float>("dyn_obj/map_cons_hor_thr2", 0.02f);
    node->declare_parameter<float>("dyn_obj/map_cons_ver_thr2", 0.01f);
    node->declare_parameter<float>("dyn_obj/occ_depth_thr2", 0.15f);
    node->declare_parameter<float>("dyn_obj/occ_hor_thr2", 0.02f);
    node->declare_parameter<float>("dyn_obj/occ_ver_thr2", 0.01f);
    node->declare_parameter<float>("dyn_obj/depth_cons_depth_thr2", 0.5f);
    node->declare_parameter<float>("dyn_obj/depth_cons_depth_max_thr2", 0.5f);
    node->declare_parameter<float>("dyn_obj/depth_cons_hor_thr2", 0.02f);
    node->declare_parameter<float>("dyn_obj/depth_cons_ver_thr2", 0.01f);
    node->declare_parameter<float>("dyn_obj/k_depth2", 0.005f);
    node->declare_parameter<int>("dyn_obj/occluded_times_thr2", 3);
    node->declare_parameter<bool>("dyn_obj/case2_interp_en", false);
    node->declare_parameter<float>("dyn_obj/k_depth_max_thr2", 0.0f);
    node->declare_parameter<float>("dyn_obj/d_depth_max_thr2", 0.15f);
    node->declare_parameter<float>("dyn_obj/v_min_thr3", 0.5f);
    node->declare_parameter<float>("dyn_obj/acc_thr3", 1.0f);
    node->declare_parameter<float>("dyn_obj/map_cons_depth_thr3", 0.15f);
    node->declare_parameter<float>("dyn_obj/map_cons_hor_thr3", 0.02f);
    node->declare_parameter<float>("dyn_obj/map_cons_ver_thr3", 0.01f);
    node->declare_parameter<float>("dyn_obj/occ_depth_thr3", 0.15f);
    node->declare_parameter<float>("dyn_obj/occ_hor_thr3", 0.02f);
    node->declare_parameter<float>("dyn_obj/occ_ver_thr3", 0.01f);
    node->declare_parameter<float>("dyn_obj/depth_cons_depth_thr3", 0.5f);
    node->declare_parameter<float>("dyn_obj/depth_cons_depth_max_thr3", 0.5f);
    node->declare_parameter<float>("dyn_obj/depth_cons_hor_thr3", 0.02f);
    node->declare_parameter<float>("dyn_obj/depth_cons_ver_thr3", 0.01f);
    node->declare_parameter<float>("dyn_obj/k_depth3", 0.005f);
    node->declare_parameter<int>("dyn_obj/occluding_times_thr3", 3);
    node->declare_parameter<bool>("dyn_obj/case3_interp_en", false);
    node->declare_parameter<float>("dyn_obj/k_depth_max_thr3", 0.0f);
    node->declare_parameter<float>("dyn_obj/d_depth_max_thr3", 0.15f);
    node->declare_parameter<float>("dyn_obj/interp_hor_thr", 0.01f);
    node->declare_parameter<float>("dyn_obj/interp_ver_thr", 0.01f);
    node->declare_parameter<float>("dyn_obj/interp_thr1", 1.0f);
    node->declare_parameter<float>("dyn_obj/interp_static_max", 10.0f);
    node->declare_parameter<float>("dyn_obj/interp_start_depth1", 20.0f);
    node->declare_parameter<float>("dyn_obj/interp_kp1", 0.1f);
    node->declare_parameter<float>("dyn_obj/interp_kd1", 1.0f);
    node->declare_parameter<float>("dyn_obj/interp_thr2", 0.15f);
    node->declare_parameter<float>("dyn_obj/interp_thr3", 0.15f);
    node->declare_parameter<bool>("dyn_obj/dyn_filter_en", true);
    node->declare_parameter<bool>("dyn_obj/debug_publish", true);
    node->declare_parameter<int>("dyn_obj/laserCloudSteadObj_accu_limit", 5);
    node->declare_parameter<float>("dyn_obj/voxel_filter_size", 0.1f);
    node->declare_parameter<bool>("dyn_obj/cluster_coupled", false);
    node->declare_parameter<bool>("dyn_obj/cluster_future", false);
    node->declare_parameter<int>("dyn_obj/cluster_extend_pixel", 2);
    node->declare_parameter<int>("dyn_obj/cluster_min_pixel_number", 4);
    node->declare_parameter<float>("dyn_obj/cluster_thrustable_thresold", 0.3f);
    node->declare_parameter<float>("dyn_obj/cluster_Voxel_revolusion", 0.3f);
    node->declare_parameter<bool>("dyn_obj/cluster_debug_en", false);
    node->declare_parameter<string>("dyn_obj/cluster_out_file", "");
    node->declare_parameter<float>("dyn_obj/ver_resolution_max", 0.0025f);
    node->declare_parameter<float>("dyn_obj/hor_resolution_max", 0.0025f);
    node->declare_parameter<float>("dyn_obj/buffer_dur", 0.1f);
    node->declare_parameter<int>("dyn_obj/point_index", 0);
    node->declare_parameter<string>("dyn_obj/frame_id", "odom");
    node->declare_parameter<string>("dyn_obj/time_file", "");
    node->declare_parameter<string>("dyn_obj/time_breakdown_file", "");

    node->get_parameter("dyn_obj/buffer_delay", buffer_delay);
    node->get_parameter("dyn_obj/buffer_size", buffer_size);
    node->get_parameter("dyn_obj/points_num_perframe", points_num_perframe);
    node->get_parameter("dyn_obj/depth_map_dur", depth_map_dur);
    node->get_parameter("dyn_obj/max_depth_map_num", max_depth_map_num);
    node->get_parameter("dyn_obj/max_pixel_points", max_pixel_points);
    node->get_parameter("dyn_obj/frame_dur", frame_dur);
    node->get_parameter("dyn_obj/dataset", dataset);
    node->get_parameter("dyn_obj/self_x_f", self_x_f);
    node->get_parameter("dyn_obj/self_x_b", self_x_b);
    node->get_parameter("dyn_obj/self_y_l", self_y_l);
    node->get_parameter("dyn_obj/self_y_r", self_y_r);
    node->get_parameter("dyn_obj/blind_dis", blind_dis);
    node->get_parameter("dyn_obj/fov_up", fov_up);
    node->get_parameter("dyn_obj/fov_down", fov_down);
    node->get_parameter("dyn_obj/fov_cut", fov_cut);
    node->get_parameter("dyn_obj/fov_left", fov_left);
    node->get_parameter("dyn_obj/fov_right", fov_right);
    node->get_parameter("dyn_obj/checkneighbor_range", checkneighbor_range);
    node->get_parameter("dyn_obj/stop_object_detect", stop_object_detect);
    node->get_parameter("dyn_obj/dyn_filter_en", dyn_filter_en);
    node->get_parameter("dyn_obj/debug_publish", debug_en);
    node->get_parameter("dyn_obj/frame_id", frame_id);
    node->get_parameter("dyn_obj/time_file", time_file);
    node->get_parameter("dyn_obj/time_breakdown_file", time_breakdown_file);

    max_ind = std::floor(2.0 * M_PI / hor_resolution_max);

    if (pcl_his_list.empty())
    {
        PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
        first_frame->reserve(400000);
        pcl_his_list.push_back(first_frame);

        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj.reset(new PointCloudXYZI());
        laserCloudDynObj.reset(new PointCloudXYZI());
        laserCloudDynObj_world.reset(new PointCloudXYZI());

        int xy_ind[2] = {-1, 1};
        for (int ind_hor = 0; ind_hor < 2 * hor_num + 1; ++ind_hor)
        {
            for (int ind_ver = 0; ind_ver < 2 * ver_num + 1; ++ind_ver)
            {
                pos_offset.push_back(
                    ((ind_hor) / 2 + ind_hor % 2) * xy_ind[ind_hor % 2] * MAX_1D_HALF +
                    ((ind_ver) / 2 + ind_ver % 2) * xy_ind[ind_ver % 2]);
            }
        }
    }

    map_cons_hor_num1 = std::ceil(map_cons_hor_thr1 / hor_resolution_max);
    map_cons_ver_num1 = std::ceil(map_cons_ver_thr1 / ver_resolution_max);
    interp_hor_num = std::ceil(interp_hor_thr / hor_resolution_max);
    interp_ver_num = std::ceil(interp_ver_thr / ver_resolution_max);
    map_cons_hor_num2 = std::ceil(map_cons_hor_thr2 / hor_resolution_max);
    map_cons_ver_num2 = std::ceil(map_cons_ver_thr2 / ver_resolution_max);
    occ_hor_num2 = std::ceil(occ_hor_thr2 / hor_resolution_max);
    occ_ver_num2 = std::ceil(occ_ver_thr2 / ver_resolution_max);
    depth_cons_hor_num2 = std::ceil(depth_cons_hor_thr2 / hor_resolution_max);
    depth_cons_ver_num2 = std::ceil(depth_cons_ver_thr2 / ver_resolution_max);
    map_cons_hor_num3 = std::ceil(map_cons_hor_thr3 / hor_resolution_max);
    map_cons_ver_num3 = std::ceil(map_cons_ver_thr3 / ver_resolution_max);
    occ_hor_num3 = std::ceil(occ_hor_thr3 / hor_resolution_max);
    occ_ver_num3 = std::ceil(occ_ver_thr3 / ver_resolution_max);
    depth_cons_hor_num3 = std::ceil(depth_cons_hor_thr3 / hor_resolution_max);
    depth_cons_ver_num3 = std::ceil(depth_cons_ver_thr3 / ver_resolution_max);

    buffer.init(buffer_size);

    pixel_fov_up = std::floor((fov_up / 180.0 * PI_MATH + 0.5 * PI_MATH) / ver_resolution_max);
    pixel_fov_down = std::floor((fov_down / 180.0 * PI_MATH + 0.5 * PI_MATH) / ver_resolution_max);
    pixel_fov_cut = std::floor((fov_cut / 180.0 * PI_MATH + 0.5 * PI_MATH) / ver_resolution_max);
    pixel_fov_left = std::floor((fov_left / 180.0 * PI_MATH + PI_MATH) / hor_resolution_max);
    pixel_fov_right = std::floor((fov_right / 180.0 * PI_MATH + PI_MATH) / hor_resolution_max);

    max_pointers_num =
        std::round((max_depth_map_num * depth_map_dur + buffer_delay) / frame_dur) + 1;

    point_soph_pointers.reserve(max_pointers_num);
    for (int i = 0; i < max_pointers_num; ++i)
    {
        point_soph* p = new point_soph[points_num_perframe];
        point_soph_pointers.push_back(p);
    }

    if (!time_file.empty())
        time_out.open(time_file, std::ios::out);

    if (!time_breakdown_file.empty())
        time_breakdown_out.open(time_breakdown_file, std::ios::out);

    Cluster.Init();
}


void DynObjFilter::filter(PointCloudXYZI::Ptr feats_undistort,
                          const M3D & rot_end,
                          const V3D & pos_end,
                          const double & scan_end_time)
{
    double t00 = omp_get_wtime();
    time_search = time_research = time_search_0 = time_build = time_total = time_other0 = 0.0;
    time_interp1 = time_interp2 = 0;
    // int num_build = 0, num_search_0 = 0, num_research = 0;

    if (feats_undistort == nullptr) return;
    int size = static_cast<int>(feats_undistort->points.size());

    if (debug_en)
    {
        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj_hist->reserve(20 * size);
    }

    dyn_tag_origin.clear();
    dyn_tag_origin.reserve(size);
    dyn_tag_origin.resize(size);
    dyn_tag_cluster.clear();
    dyn_tag_cluster.reserve(size);
    dyn_tag_cluster.resize(size);

    laserCloudDynObj.reset(new PointCloudXYZI());
    laserCloudDynObj->reserve(size);
    laserCloudDynObj_world.reset(new PointCloudXYZI());
    laserCloudDynObj_world->reserve(size);
    laserCloudSteadObj.reset(new PointCloudXYZI());
    laserCloudSteadObj->reserve(size);
    laserCloudDynObj_clus.reset(new PointCloudXYZI()); 
    laserCloudDynObj_clus->reserve(size);
    laserCloudSteadObj_clus.reset(new PointCloudXYZI());   
    laserCloudSteadObj_clus->reserve(size);

    std::ofstream out;
    std::ofstream out_origin;
    bool is_rec = false;
    bool is_rec_origin = false;
    if (is_set_path)
    {
        out.open(out_file, std::ios::out | std::ios::binary);
        out_origin.open(out_file_origin, std::ios::out | std::ios::binary);
        if (out.is_open()) is_rec = true;
        if (out_origin.is_open()) is_rec_origin = true;
    }

    time_test1.reserve(size);
    time_test1.resize(size);
    time_test2.reserve(size);
    time_test2.resize(size);
    time_test3.reserve(size);
    time_test3.resize(size);
    time_occ_check.reserve(size);
    time_occ_check.resize(size);
    time_map_cons.reserve(size);
    time_map_cons.resize(size);
    time_proj.reserve(size);
    time_proj.resize(size);

    for (int i = 0; i < size; ++i)
    {
        time_test1[i] = 0.0;
        time_test2[i] = 0.0;
        time_test3[i] = 0.0;
        time_occ_check[i] = 0.0;
        time_map_cons[i] = 0.0;
        time_proj[i] = 0.0;
    }

    // int case2_num = 0;
    double t0 = omp_get_wtime();
    // double time_case1 = 0, time_case2 = 0, time_case3 = 0;

    pcl::PointCloud<PointType> raw_points_world;
    raw_points_world.reserve(size);
    raw_points_world.resize(size);

    std::vector<int> index(size);
    for (int i = 0; i < size; ++i) index[i] = i;

    std::vector<point_soph*> points;
    points.reserve(size);
    points.resize(size);

    point_soph* p = point_soph_pointers[cur_point_soph_pointers];

    if (!time_file.empty()) time_out << size << " ";

    // Parallel processing over points (requires <execution>)
    std::for_each(std::execution::par, index.begin(), index.end(), [&](const int &i)
    {
        p[i].reset();
        V3D p_body(feats_undistort->points[i].x,
                   feats_undistort->points[i].y,
                   feats_undistort->points[i].z);
        int intensity = feats_undistort->points[i].curvature;
        V3D p_glob = rot_end * p_body + pos_end;
        p[i].glob = p_glob;
        p[i].dyn = STATIC;
        p[i].rot = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;
        p[i].intensity = feats_undistort->points[i].intensity;

        if (dataset == 0 && std::fabs(intensity - 666) < 1e-4)
        {
            p[i].is_distort = true;
        }

        if (InvalidPointCheck(p_body, intensity))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            dyn_tag_cluster[i] = -1;
        }
        else if (SelfPointCheck(p_body, p[i].dyn))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
        }
        else if (Case1(p[i]))
        {
            p[i].dyn = CASE1;
            dyn_tag_origin[i] = 1;
        }
        else if (Case2(p[i]))
        {
            p[i].dyn = CASE2;
            dyn_tag_origin[i] = 1;
        }
        else if (Case3(p[i]))
        {
            p[i].dyn = CASE3;
            dyn_tag_origin[i] = 1;
        }
        else
        {
            dyn_tag_origin[i] = 0;
        }

        points[i] = &p[i];
    });

    if (!time_file.empty()) time_out << omp_get_wtime() - t0 << " ";

    // build raw_points_world and separate dynamic/static clouds
    for (int i = 0; i < size; ++i)
    {
        PointType po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];
        po.intensity = points[i]->intensity;

        PointType po_w;
        po_w.x = points[i]->glob[0];
        po_w.y = points[i]->glob[1];
        po_w.z = points[i]->glob[2];

        raw_points_world[i] = po;

        switch (points[i]->dyn)
        {
            case CASE1:
                po.normal_x = 1;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE2:
                po.normal_y = points[i]->occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE3:
                po.normal_z = points[i]->is_occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            default:
                laserCloudSteadObj->push_back(po_w);
                break;
        }
    }

    int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0;
    double clus_before = omp_get_wtime();

    std_msgs::msg::Header header_clus;
    int64_t nsec = static_cast<int64_t>(scan_end_time * 1000000000LL);
    header_clus.stamp.sec = static_cast<int32_t>(nsec / 1000000000LL);
    header_clus.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
    header_clus.frame_id = frame_id;

    if (cluster_coupled || cluster_future)
    {
        Cluster.Clusterprocess(dyn_tag_cluster, *laserCloudDynObj, raw_points_world, header_clus, rot_end, pos_end);

        for (int i = 0; i < size; ++i)
        {
            PointType po;
            po.x = points[i]->glob(0);
            po.y = points[i]->glob(1);
            po.z = points[i]->glob(2);
            po.curvature = i;

            switch (points[i]->dyn)
            {
                case CASE1:
                    if (dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        ++num_neag;
                    }
                    else
                    {
                        po.normal_x = 1;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if (!dyn_filter_en)
                        {
                            po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        }
                        ++num_1;
                    }
                    break;

                case CASE2:
                    if (dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        ++num_neag;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if (!dyn_filter_en)
                        {
                            po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        }
                        ++num_2;
                    }
                    break;

                case CASE3:
                    if (dyn_tag_cluster[i] == 0)
                    {
                        points[i]->dyn = STATIC;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        ++num_neag;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if (!dyn_filter_en)
                        {
                            po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        }
                        ++num_3;
                    }
                    break;

                case STATIC:
                    if (dyn_tag_cluster[i] == 1)
                    {
                        points[i]->dyn = CASE1;
                        points[i]->occu_times = -1;
                        points[i]->is_occu_times = -1;
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        laserCloudDynObj_clus->push_back(po);
                        if (!dyn_filter_en)
                        {
                            po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        }
                        ++num_1;
                    }
                    else
                    {
                        po.normal_x = 0;
                        po.normal_y = points[i]->is_occu_times;
                        po.normal_z = points[i]->occu_times;
                        po.intensity = static_cast<int>(points[i]->local.norm() * 10) + 10;
                        laserCloudSteadObj_clus->push_back(po);
                        ++num_neag;
                    }
                    break;

                default:
                    ++num_inval;
                    break;
            }
        }
    }

    if (!time_file.empty()) time_out << omp_get_wtime() - clus_before << " ";
    double t3 = omp_get_wtime();

    Points2Buffer(points, index);

    // double t4 = omp_get_wtime();
    if (!time_file.empty()) time_out << omp_get_wtime() - t3 << " ";

    Buffer2DepthMap(scan_end_time);

    if (!time_file.empty()) time_out << omp_get_wtime() - t3 << std::endl;

    if (cluster_coupled)
    {
        for (int i = 0; i < size; ++i)
        {
            if (dyn_tag_cluster[i] == 1)
            {
                if (is_rec)
                {
                    int tmp = 251;
                    out.write(reinterpret_cast<char*>(&tmp), sizeof(int));
                }
            }
            else
            {
                if (is_rec)
                {
                    int tmp = 9;
                    out.write(reinterpret_cast<char*>(&tmp), sizeof(int));
                }
            }

            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if (is_rec_origin)
                {
                    int tmp = 251;
                    out_origin.write(reinterpret_cast<char*>(&tmp), sizeof(int));
                }
            }
            else
            {
                if (is_rec_origin)
                {
                    int tmp = 9;
                    out_origin.write(reinterpret_cast<char*>(&tmp), sizeof(int));
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < size; ++i)
        {
            if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2)
            {
                if (is_rec)
                {
                    int tmp = 251;
                    out.write(reinterpret_cast<char*>(&tmp), sizeof(int));
                }
            }
            else
            {
                if (is_rec)
                {
                    int tmp = 9;
                    out.write(reinterpret_cast<char*>(&tmp), sizeof(int));
                }
            }
        }
    }

    double total_test1 = 0, total_test2 = 0, total_test3 = 0, total_proj = 0, total_occ = 0, total_map = 0;
    for (int i = 0; i < size; ++i)
    {
        total_test1 += time_test1[i];
        total_test2 += time_test2[i];
        total_test3 += time_test3[i];
        total_proj += time_proj[i];
        total_occ += time_occ_check[i];
        total_map += time_map_cons[i];
    }

    if (!time_breakdown_file.empty())
    {
        time_breakdown_out << total_test1 << " " << total_test2 << " " << total_test3
                           << " " << total_proj << " " << total_occ << " " << total_map << std::endl;
    }

    frame_num_for_rec++;
    cur_point_soph_pointers = (cur_point_soph_pointers + 1) % max_pointers_num;

    if (is_rec) out.close();

    time_total = omp_get_wtime() - t00;
    time_ind++;
    time_total_avr = time_total_avr * (time_ind - 1) / time_ind + time_total / time_ind;
}

void DynObjFilter::Points2Buffer(std::vector<point_soph*> &points,
                                 std::vector<int> &index_vector)
{
    int cur_tail = buffer.tail;
    buffer.push_parallel_prepare(points.size());

    std::for_each(std::execution::par,
                  index_vector.begin(),
                  index_vector.end(),
                  [&](const int &i)
                  {
                      buffer.push_parallel(points[i], cur_tail + i);
                  });
}

void DynObjFilter::Buffer2DepthMap(double cur_time)
{
    int len = buffer.size();

    for (int k = 0; k < len; k++)
    {
        point_soph *point = buffer.front();

        if ((cur_time - point->time) >= buffer_delay - frame_dur / 2.0)
        {
            if (depth_map_list.empty())
            {
                if (depth_map_list.size() < static_cast<size_t>(max_depth_map_num))
                {
                    map_index++;
                    DepthMap::Ptr new_map =
                        std::make_shared<DepthMap>(point->rot, point->transl,
                                                   point->time, map_index);
                    depth_map_list.push_back(new_map);
                }
                else
                {
                    buffer.pop();
                    continue;
                }
            }
            else if ((point->time - depth_map_list.back()->time) >=
                     depth_map_dur - frame_dur / 2.0)
            {
                map_index++;
                if (depth_map_list.size() == static_cast<size_t>(max_depth_map_num))
                {
                    depth_map_list.front()->Reset(point->rot, point->transl,
                                                  point->time, map_index);
                    auto reuse_map = depth_map_list.front();
                    depth_map_list.pop_front();
                    depth_map_list.push_back(reuse_map);
                }
                else
                {
                    DepthMap::Ptr new_map =
                        std::make_shared<DepthMap>(point->rot, point->transl,
                                                   point->time, map_index);
                    depth_map_list.push_back(new_map);
                }
            }

            switch (point->dyn)
            {
            case STATIC:
            case CASE1:
            case CASE2:
            case CASE3:
                SphericalProjection(*point,
                                    depth_map_list.back()->map_index,
                                    depth_map_list.back()->project_R,
                                    depth_map_list.back()->project_T,
                                    *point);

                if (depth_map_list.back()->depth_map[point->position].size() <
                    max_pixel_points)
                {
                    auto &cell =
                        depth_map_list.back()->depth_map[point->position];
                    cell.push_back(point);

                    double z = point->vec(2);

                    if (z > depth_map_list.back()->max_depth_all[point->position])
                    {
                        depth_map_list.back()->max_depth_all[point->position] = z;
                        depth_map_list.back()->max_depth_index_all[point->position] =
                            cell.size() - 1;
                    }

                    if (z < depth_map_list.back()->min_depth_all[point->position] ||
                        depth_map_list.back()->min_depth_all[point->position] < 1e-5)
                    {
                        depth_map_list.back()->min_depth_all[point->position] = z;
                        depth_map_list.back()->min_depth_index_all[point->position] =
                            cell.size() - 1;
                    }

                    if (point->dyn == STATIC)
                    {
                        if (z < depth_map_list.back()->min_depth_static[point->position] ||
                            depth_map_list.back()->min_depth_static[point->position] < 1e-5)
                        {
                            depth_map_list.back()->min_depth_static[point->position] = z;
                        }
                        if (z > depth_map_list.back()->max_depth_static[point->position])
                        {
                            depth_map_list.back()->max_depth_static[point->position] = z;
                        }
                    }
                }
                break;

            default:
                break;
            }

            buffer.pop();
        }
        else
        {
            break;
        }
    }

    if (debug_en)
    {
        for (size_t i = 0; i < depth_map_list.size(); i++)
        {
            for (size_t j = 0; j < depth_map_list[i]->depth_map.size(); j++)
            {
                for (size_t k = 0;
                     k < depth_map_list[i]->depth_map[j].size(); k++)
                {
                    PointType po;
                    point_soph *pt = depth_map_list[i]->depth_map[j][k];
                    po.x = pt->glob(0);
                    po.y = pt->glob(1);
                    po.z = pt->glob(2);
                    po.intensity = pt->local(2);
                    po.curvature = pt->local(1);
                    po.normal_x = pt->hor_ind;
                    po.normal_y = pt->ver_ind;
                    po.normal_z = pt->dyn;

                    if (pt->dyn == STATIC)
                    {
                        laserCloudSteadObj_hist->push_back(po);
                    }
                }
            }
        }
    }
}

void DynObjFilter::SphericalProjection(point_soph &p,
                                       int depth_index,
                                       const M3D &rot,
                                       const V3D &transl,
                                       point_soph &p_spherical)
{
    const int idx = depth_index % HASH_PRIM;

    if (p.last_vecs[idx].squaredNorm() > 1e-10f)
    {
        p_spherical.vec = p.last_vecs[idx];
        p_spherical.hor_ind = p.last_positions[idx][0];
        p_spherical.ver_ind = p.last_positions[idx][1];
        p_spherical.position = p.last_positions[idx][2];
    }
    else
    {
        V3D p_proj = rot * (p.glob - transl);
        p_spherical.GetVec(p_proj, hor_resolution_max, ver_resolution_max);

        p.last_vecs[idx] = p_spherical.vec;
        p.last_positions[idx][0] = p_spherical.hor_ind;
        p.last_positions[idx][1] = p_spherical.ver_ind;
        p.last_positions[idx][2] = p_spherical.position;
    }
}

bool DynObjFilter::InvalidPointCheck(const V3D &body, const int intensity)
{
    double r2 = body.squaredNorm();

    if (r2 < blind_dis * blind_dis)
        return true;

    if (dataset == 1 &&
        std::fabs(body(0)) < 0.1 &&
        std::fabs(body(1)) < 1.0 &&
        std::fabs(body(2)) < 0.1)
        return true;

    return false;
}

bool DynObjFilter::SelfPointCheck(const V3D &body, const dyn_obj_flg dyn)
{
    (void)dyn;  // ROS2 下避免未使用参数告警

    if (dataset == 0)
    {
        if ((body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 && body(2) > -0.65 && body(2) < -0.4) ||
            (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 && body(2) > -0.75 && body(2) < -0.40) ||
            (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 && body(2) > -0.8 && body(2) < -0.6) ||
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 && body(2) > -1.0 && body(2) < -0.9) ||
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 && body(2) > -1.0 && body(2) < -0.9))
        {
            return true;
        }
    }
    return false;
}

bool DynObjFilter::CheckVerFoV(const point_soph &p, const DepthMap &map_info)
{
    bool ver_up = false, ver_down = false;

    for (int i = p.ver_ind; i >= pixel_fov_down; i--)
    {
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if (!map_info.depth_map[cur_pos].empty())
        {
            ver_down = true;
            break;
        }
    }

    for (int i = p.ver_ind; i <= pixel_fov_up; i++)
    {
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if (!map_info.depth_map[cur_pos].empty())
        {
            ver_up = true;
            break;
        }
    }

    return !(ver_up && ver_down);
}

void DynObjFilter::CheckNeighbor(const point_soph &p,
                                 const DepthMap &map_info,
                                 float &max_depth,
                                 float &min_depth)
{
    int n = checkneighbor_range;

    for (int i = -n; i <= n; i++)
    {
        for (int j = -n; j <= n; j++)
        {
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            if (cur_pos >= 0 && cur_pos < MAX_2D_N &&
                !map_info.depth_map[cur_pos].empty())
            {
                float cur_max = map_info.max_depth_static[cur_pos];
                float cur_min = map_info.min_depth_static[cur_pos];

                if (min_depth > 1e-5)
                    min_depth = std::min(cur_min, min_depth);
                else
                    min_depth = cur_min;

                if (max_depth > 1e-5)
                    max_depth = std::max(cur_max, max_depth);
                else
                    max_depth = cur_max;
            }
        }
    }
}

bool DynObjFilter::Case1(point_soph &p)
{
    int depth_map_num = static_cast<int>(depth_map_list.size());
    int occluded_map = depth_map_num;

    for (int i = depth_map_num - 1; i >= 0; i--)
    {
        SphericalProjection(p,
                            depth_map_list[i]->map_index,
                            depth_map_list[i]->project_R,
                            depth_map_list[i]->project_T,
                            p);

        if (std::fabs(p.hor_ind) > MAX_1D ||
            std::fabs(p.ver_ind) > MAX_1D_HALF ||
            p.vec(2) < 0.0f ||
            p.position < 0 ||
            p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            continue;
        }

        if (Case1Enter(p, *depth_map_list[i]))
        {
            if (Case1FalseRejection(p, *depth_map_list[i]))
            {
                occluded_map--;
            }
        }
        else
        {
            occluded_map--;
        }

        if (occluded_map < occluded_map_thr1)
            return false;

        if (occluded_map - i >= occluded_map_thr1)
            return true;
    }

    return (occluded_map >= occluded_map_thr1);
}

bool DynObjFilter::Case1Enter(const point_soph &p, const DepthMap &map_info)
{
    float max_depth = 0.0f, min_depth = 0.0f;

    if (!map_info.depth_map[p.position].empty())
    {
        max_depth = map_info.max_depth_static[p.position];
        min_depth = map_info.min_depth_static[p.position];
    }
    else
    {
        if (p.ver_ind <= pixel_fov_up &&
            p.ver_ind > pixel_fov_down &&
            p.hor_ind <= pixel_fov_left &&
            p.hor_ind >= pixel_fov_right &&
            CheckVerFoV(p, map_info))
        {
            CheckNeighbor(p, map_info, max_depth, min_depth);
        }
    }

    float cur_min =
        std::max(cutoff_value,
                 k_depth_min_thr1 * (p.vec(2) - d_depth_min_thr1)) +
        enter_min_thr1;
    float cur_max =
        std::max(cutoff_value,
                 k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) +
        enter_max_thr1;
    float cur_depth = depth_thr1;

    if (dataset == 0 && p.is_distort)
    {
        cur_min *= enlarge_distort;
        cur_max *= enlarge_distort;
        cur_depth *= enlarge_distort;
    }

    if (p.vec(2) < min_depth - cur_max ||
        (min_depth < p.vec(2) - cur_min &&
         max_depth > p.vec(2) + cur_max) ||
        (stop_object_detect &&
         min_depth < 1e-5 &&
         max_depth < 1e-5 &&
         !map_info.depth_map[p.position].empty() &&
         p.vec(2) < map_info.max_depth_all[p.position] + 1.0f))
    {
        case1_num++;
        return true;
    }

    return false;
}

bool DynObjFilter::Case1FalseRejection(point_soph &p,
                                      const DepthMap &map_info)
{
    return Case1MapConsistencyCheck(p, map_info, case1_interp_en);
}


bool DynObjFilter::Case1MapConsistencyCheck(point_soph &p,
                                           const DepthMap &map_info,
                                           bool interp)
{
    float hor_half = std::max(map_cons_hor_dis1 / std::max(p.vec(2), blind_dis),
                              map_cons_hor_thr1);
    float ver_half = std::max(map_cons_ver_dis1 / std::max(p.vec(2), blind_dis),
                              map_cons_ver_thr1);

    float cur_map_cons_depth_thr1 =
        std::max(cutoff_value,
                 k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) +
        map_cons_depth_thr1;

    float cur_map_cons_min_thr1 =
        std::max(cutoff_value,
                 k_depth_min_thr1 * (p.vec(2) - d_depth_min_thr1)) +
        enter_min_thr1;

    float cur_map_cons_max_thr1 =
        std::max(cutoff_value,
                 k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) +
        enter_max_thr1;

    if (dataset == 0 && p.is_distort)
    {
        cur_map_cons_depth_thr1 *= enlarge_distort;
        cur_map_cons_min_thr1   *= enlarge_distort;
        cur_map_cons_max_thr1   *= enlarge_distort;
    }

    if (std::fabs(p.vec(1)) < enlarge_z_thr1 / 57.3f)
    {
        hor_half *= enlarge_angle;
        ver_half *= enlarge_angle;
        cur_map_cons_depth_thr1 *= enlarge_depth;
    }

    int cur_map_cons_hor_num1 =
        static_cast<int>(std::ceil(hor_half / hor_resolution_max));
    int cur_map_cons_ver_num1 =
        static_cast<int>(std::ceil(ver_half / ver_resolution_max));

    for (int ind_hor = -cur_map_cons_hor_num1;
         ind_hor <= cur_map_cons_hor_num1; ind_hor++)
    {
        for (int ind_ver = -cur_map_cons_ver_num1;
             ind_ver <= cur_map_cons_ver_num1; ind_ver++)
        {
            int pos_new =
                ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                ((p.ver_ind + ind_ver) % MAX_1D_HALF);

            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;

            const std::vector<point_soph *> &points_in_pixel =
                map_info.depth_map[pos_new];

            if (map_info.max_depth_static[pos_new] <
                    p.vec(2) - cur_map_cons_min_thr1 ||
                map_info.min_depth_static[pos_new] >
                    p.vec(2) + cur_map_cons_max_thr1)
            {
                continue;
            }

            for (const point_soph *point : points_in_pixel)
            {
                if (point->dyn == STATIC &&
                    (std::fabs(p.vec(2) - point->vec(2)) <
                         cur_map_cons_depth_thr1 ||
                     ((p.vec(2) - point->vec(2)) >
                          cur_map_cons_depth_thr1 &&
                      (p.vec(2) - point->vec(2)) <
                          cur_map_cons_min_thr1)) &&
                    std::fabs(p.vec(0) - point->vec(0)) < hor_half &&
                    std::fabs(p.vec(1) - point->vec(1)) < ver_half)
                {
                    return true;
                }
            }
        }
    }

    if (interp &&
        (p.local(0) < self_x_b || p.local(0) > self_x_f ||
         p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float depth_static =
            DepthInterpolationStatic(p,
                                     map_info.map_index,
                                     map_info.depth_map);

        float cur_interp = interp_thr1;
        if (p.vec(2) > interp_start_depth1)
        {
            cur_interp +=
                (p.vec(2) - interp_start_depth1) * interp_kp1 +
                interp_kd1;
        }

        if (std::fabs(depth_static + 1) < 1e-5 ||
            std::fabs(depth_static + 2) < 1e-5)
        {
            return false;
        }

        if (std::fabs(depth_static - p.vec(2)) < cur_interp)
        {
            return true;
        }
    }

    return false;
}

float DynObjFilter::DepthInterpolationStatic(point_soph & p, int map_index, const DepthMap2D &depth_map)
{
    if(std::fabs(p.last_depth_interps.at(map_index - depth_map_list.front()->map_index)) > 10E-4)
    {
        float depth_cal = p.last_depth_interps.at(map_index - depth_map_list.front()->map_index);
        return depth_cal;
    }
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    std::vector<V3F> p_neighbors;
    int all_num = 0, static_num = 0, no_bg_num = 0;     
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor ++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver ++)
        {   
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF); 
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const std::vector<point_soph*> & points_in_pixel = depth_map[pos_new];     
            for (size_t j = 0; j < points_in_pixel.size(); j ++)
            {   
                const point_soph* point = points_in_pixel[j]; 
                if (std::fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                float hor_minus =  point->vec(0) - p.vec(0);
                float ver_minus =  point->vec(1) - p.vec(1);
                if (std::fabs(hor_minus) < interp_hor_thr && std::fabs(ver_minus) < interp_ver_thr)
                {
                    all_num ++;
                    if(point->dyn == STATIC) 
                    {
                        static_num ++;
                    }
                    if((point->vec(2) - p.vec(2)) <= interp_static_max && (p.vec(2) - point->vec(2)) < 5.0)
                    {
                        no_bg_num ++;
                    }
                    if(point->dyn == STATIC)
                    {
                        p_neighbors.push_back(point->vec);
                        if (p_1(2)<0.000001 || std::fabs(hor_minus) + std::fabs(ver_minus) < std::fabs(p_1(0) - p.vec(0)) + std::fabs(p_1(1) - p.vec(1)))
                        {
                            p_1 = point->vec;
                        }
                    }
                }
            }
        }
    }
    if (p_1(2)<10E-5)
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -1.0f;
        return -1.0f;
    }
    int cur_size = p_neighbors.size();
    for(int t_i = 0; t_i < cur_size-2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2.0f * (interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha  = 0, beta = 0;
        for(int i = t_i+1; i < cur_size-1; i++)
        {           
            if(std::fabs(p_neighbors[i](0)-p.vec(0)) + std::fabs(p_neighbors[i](1)-p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = std::fabs(p_neighbors[i](0)-p.vec(0)) + std::fabs(p_neighbors[i](1)-p.vec(1));
                if (single_fabs >= min_fabs) continue;
                for(int ii = i+1; ii < cur_size; ii++)
                {
                    float cur_fabs = std::fabs(p_neighbors[i](0)-p.vec(0)) + std::fabs(p_neighbors[i](1)-p.vec(1)) + \
                                    std::fabs(p_neighbors[ii](0)-p.vec(0)) + std::fabs(p_neighbors[ii](1)-p.vec(1));
                    if( cur_fabs < min_fabs)
                    {
                        float x1 = p_neighbors[i](0) - p_1(0);
                        float x2 = p_neighbors[ii](0) - p_1(0);
                        float y1 = p_neighbors[i](1) - p_1(1);
                        float y2 = p_neighbors[ii](1) - p_1(1);
                        float lower = x1*y2-x2*y1;
                        if(std::fabs(lower) > 10E-5)
                        {
                            alpha = (x*y2-y*x2)/lower;
                            beta = -(x*y1-y*x1)/lower;
                            if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs; 
                            }
                        }
                    }
                }  
            }
        }
        if (p_2(2)<10E-5 || p_3(2)<10E-5)
        {
            continue;
        }
        float depth_cal = (1.0f-alpha-beta)*p_1(2) + alpha*p_2(2) + beta*p_3(2);
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = depth_cal;
        return depth_cal;
    }
    
    p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2.0f;
    return -2.0f;
}


bool  DynObjFilter::Case2(point_soph & p)
{   
    if(dataset == 0 && p.is_distort) return false;
    int first_i = depth_map_list.size();
    first_i -= 1;
    if(first_i < 0) return false;
    point_soph p_spherical = p;   
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);  
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f || \
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }
    int cur_occ_times = 0;
    if (Case2Enter(p_spherical, *depth_map_list[first_i]))
    {
        if (!Case2MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case2_interp_en))
        {
            double ti = 0;
            float vi = 0; 
            float min_hor = occ_hor_thr2, min_ver = occ_ver_thr2;
            bool map_cons = true;    
            for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor ++)
            {
                for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver ++)
                {   
                    int pos_new = ((p_spherical.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p_spherical.ver_ind +ind_ver)%MAX_1D_HALF);       
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    const vector<point_soph*> & points_in_pixel = depth_map_list[first_i]->depth_map[pos_new];                    
                    if (depth_map_list[first_i]->min_depth_all[pos_new] > p_spherical.vec(2))
                    {
                        continue;
                    }   
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph*  p_occ = points_in_pixel[k];                   
                        if(Case2IsOccluded(p_spherical, *p_occ) && Case2DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                        {                        
                            cur_occ_times = 1;
                            if(cur_occ_times >= occluded_times_thr2) break;
                            ti = (p_occ->time + p.time)/2;
                            vi = (p_spherical.vec(2) - p_occ->vec(2))/(p.time - p_occ->time);
                            p.occu_index[0] = depth_map_list[first_i]->map_index;
                            p.occu_index[1] = pos_new;
                            p.occu_index[2] = k;
                            p.occ_vec = p_spherical.vec;
                            p.occu_times = cur_occ_times;
                            point_soph  p0 = p;
                            point_soph p1 = *points_in_pixel[k];                          
                            int i = depth_map_list.size();
                            i = i - 2;
                            V3D t1, t2;
                            t1.setZero();
                            t2.setZero();
                            while(i >= 0)
                            {                              
                                if(p1.occu_index[0] == -1 || p1.occu_index[0] < depth_map_list.front()->map_index)
                                {
                                    SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                    if(Case2SearchPointOccludingP(p1, *depth_map_list[i]))
                                    {
                                        p1.occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        break;
                                    }                                   
                                }                                
                                i = p1.occu_index[0]-depth_map_list.front()->map_index;
                                point_soph*  p2 = depth_map_list[i]->depth_map[p1.occu_index[1]][p1.occu_index[2]];                                   
                                SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
                                if(Case2MapConsistencyCheck(p, *depth_map_list[i], case2_interp_en))
                                {
                                    map_cons = false;
                                    break;
                                }
                                float vc = (p1.occ_vec(2) - p2->vec(2))/(p1.time - p2->time);
                                double tc = (p2->time + p1.time)/2;        
                                if (Case2IsOccluded(p, *p2) &&\
                                    Case2DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case2VelCheck(vi, vc, ti-tc) )
                                {                            
                                    cur_occ_times += 1;
                                    if(cur_occ_times >= occluded_times_thr2)
                                    {
                                        p.occu_times = cur_occ_times;
                                        return true;
                                    }
                                    t2 = p2->glob;
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    break;
                                }
                                i--;
                            }                       
                        } 
                        if(cur_occ_times >= occluded_times_thr2) break;
                    }
                    if(cur_occ_times >= occluded_times_thr2) break;
                }
                if(cur_occ_times >= occluded_times_thr2) break;
            }
        }
    }
    if (cur_occ_times >= occluded_times_thr2) 
    {
        p.occu_times = cur_occ_times;
        return true;
    }
    return false;
}

bool  DynObjFilter::Case2Enter(point_soph & p, const DepthMap &map_info)
{
    if(p.dyn != STATIC)
    {
        return false;
    }
    float max_depth = 0;
    float depth_thr2_final = max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2;
    if(map_info.depth_map[p.position].size() > 0)
    {
        const point_soph* max_point = map_info.depth_map[p.position][map_info.max_depth_index_all[p.position]];
        max_depth = max_point->vec(2); 
        float delta_t = (p.time - max_point->time);
        depth_thr2_final = min(depth_thr2_final, v_min_thr2*delta_t);
    }
    if(p.vec(2) > max_depth + depth_thr2_final) 
    {
        case2_num ++;
        return true;
    }
    else
    {
        return false;
    }
}


bool  DynObjFilter::Case2MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{
    // float cur_hor = map_cons_hor_thr2;
    // float cur_ver = map_cons_ver_thr2;
    float cur_depth = max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + map_cons_depth_thr2;
    for (int ind_hor = -map_cons_hor_num2; ind_hor <= map_cons_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -map_cons_ver_num2; ind_ver <= map_cons_ver_num2; ind_ver ++)  
        {
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];                         
            if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth && \
                map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                if (point->dyn == STATIC && \
                    fabs(p.time-point->time) > frame_dur && \
                    fabs(p.vec(2)-point->vec(2)) <  cur_depth && \
                    fabs(p.vec(0)-point->vec(0)) < map_cons_hor_thr2 && \
                    fabs(p.vec(1)-point->vec(1)) < map_cons_ver_thr2)
                {
                    return true;
                }               
            }         
        }
    }
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r) )
    {
        float cur_interp = interp_thr2*(depth_map_list.back()->map_index - map_info.map_index + 1);
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        if( fabs(p.vec(2) - depth_all)  < cur_interp) 
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool  DynObjFilter::Case2SearchPointOccludingP(point_soph & p, const DepthMap &map_info)
{ 
    for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver ++)  
        {
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);         
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];            
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }   
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* p_cond = points_in_pixel[j];
                if (Case2IsOccluded(p, *p_cond) && Case2DepthConsistencyCheck(*p_cond, map_info)) 
                {
                    p.occu_index[0] = map_info.map_index;
                    p.occu_index[1] = pos_new;
                    p.occu_index[2] = j;
                    p.occ_vec = p.vec;
                    return true;
                }
            }        
        }
    }
    return false;
}

bool  DynObjFilter::Case2IsOccluded(const point_soph & p, const point_soph & p_occ)
{
    if((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;
    if((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || \
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    float delta_t = p.time - p_occ.time;
    float cur_occ_hor = occ_hor_thr2; 
    float cur_occ_ver = occ_ver_thr2; 
    if(delta_t > 0)
    {
        float depth_thr2_final = min(max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2, v_min_thr2*delta_t);
        if (p.vec(2) >  p_occ.vec(2) + depth_thr2_final && \
            fabs(p.vec(0)-p_occ.vec(0)) < cur_occ_hor && \
            fabs(p.vec(1)-p_occ.vec(1)) < cur_occ_ver )
        {
            return true;
        }              
    }
    return false;
}
float DynObjFilter::DepthInterpolationAll(point_soph &p, int map_index, const DepthMap2D &depth_map)
{
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    std::vector<V3F> p_neighbors;
    int all_num = 0;

    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ++ind_hor)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ++ind_ver)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                          ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;

            const std::vector<point_soph*> &points_in_pixel = depth_map[pos_new];
            for (size_t j = 0; j < points_in_pixel.size(); ++j)
            {
                const point_soph* point = points_in_pixel[j];
                if (std::fabs(point->time - p.time) < frame_dur) continue;

                float hor_minus = point->vec(0) - p.vec(0);
                float ver_minus = point->vec(1) - p.vec(1);

                if (std::fabs(hor_minus) < interp_hor_thr && std::fabs(ver_minus) < interp_ver_thr)
                {
                    ++all_num;
                    p_neighbors.push_back(point->vec);
                    if (p_1(2) < 1e-6f || (std::fabs(hor_minus) + std::fabs(ver_minus) <
                                           std::fabs(p_1(0) - p.vec(0)) + std::fabs(p_1(1) - p.vec(1))))
                    {
                        p_1 = point->vec;
                    }
                }
            }
        }
    }

    int cur_size = static_cast<int>(p_neighbors.size());
    if (p_1(2) < 1e-5f || cur_size < 3)
    {
        return -1.0f;
    }

    for (int t_i = 0; t_i < cur_size - 2; ++t_i)
    {
        p_1 = p_neighbors[t_i];
        p_2.setZero();
        p_3.setZero();
        float min_fabs = 2.0f * (interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha = 0.0f, beta = 0.0f;

        for (int i = t_i + 1; i < cur_size - 1; ++i)
        {
            float single_fabs = std::fabs(p_neighbors[i](0) - p.vec(0)) +
                                std::fabs(p_neighbors[i](1) - p.vec(1));
            if (single_fabs < min_fabs)
            {
                p_2 = p_neighbors[i];
                for (int ii = i + 1; ii < cur_size; ++ii)
                {
                    float cur_fabs = single_fabs +
                                     std::fabs(p_neighbors[ii](0) - p.vec(0)) +
                                     std::fabs(p_neighbors[ii](1) - p.vec(1));
                    if (cur_fabs < min_fabs)
                    {
                        float x1 = p_neighbors[i](0) - p_1(0);
                        float x2 = p_neighbors[ii](0) - p_1(0);
                        float y1 = p_neighbors[i](1) - p_1(1);
                        float y2 = p_neighbors[ii](1) - p_1(1);
                        float lower = x1 * y2 - x2 * y1;
                        if (std::fabs(lower) > 1e-5f)
                        {
                            float a = (x * y2 - y * x2) / lower;
                            float b = -(x * y1 - y * x1) / lower;
                            if (a > 0.0f && a < 1.0f && b > 0.0f && b < 1.0f && (a + b) > 0.0f && (a + b) < 1.0f)
                            {
                                alpha = a;
                                beta = b;
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs;
                            }
                        }
                    }
                }
            }
        }

        if (p_2(2) < 1e-5f || p_3(2) < 1e-5f) continue;

        float depth_cal = (1.0f - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
        return depth_cal;
    }

    return -2.0f;
}

bool DynObjFilter::Case2DepthConsistencyCheck(const point_soph &p, const DepthMap &map_info)
{
    float all_minus = 0.0f;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;

    for (int ind_hor = -depth_cons_hor_num2; ind_hor <= depth_cons_hor_num2; ++ind_hor)
    {
        for (int ind_ver = -depth_cons_ver_num2; ind_ver <= depth_cons_ver_num2; ++ind_ver)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                          ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;

            const std::vector<point_soph*> &points_in_pixel = map_info.depth_map[pos_new];
            for (size_t j = 0; j < points_in_pixel.size(); ++j)
            {
                const point_soph* point = points_in_pixel[j];
                if (std::fabs(point->time - p.time) < frame_dur) continue;
                if (std::fabs(point->vec(0) - p.vec(0)) >= depth_cons_hor_thr2) continue;
                if (std::fabs(point->vec(1) - p.vec(1)) >= depth_cons_ver_thr2) continue;

                ++all_num;
                if (point->dyn == STATIC)
                {
                    float cur_minus = p.vec(2) - point->vec(2);
                    if (std::fabs(cur_minus) < depth_cons_depth_max_thr2)
                    {
                        ++num;
                        all_minus += std::fabs(point->vec(2) - p.vec(2));
                    }
                    else if (cur_minus > 0)
                    {
                        ++smaller_num;
                    }
                    else
                    {
                        ++greater_num;
                    }
                }
            }
        }
    }

    if (all_num > 0)
    {
        if (num > 1)
        {
            float cur_depth_thr = std::max(depth_cons_depth_thr2, k_depth2 * p.vec(2));
            if (all_minus / static_cast<float>(num - 1) > cur_depth_thr)
                return false;
        }
        if (greater_num == 0 || smaller_num == 0)
            return true;
        return false;
    }

    return false;
}

bool DynObjFilter::Case2VelCheck(float v1, float v2, double delta_t)
{
    return (std::fabs(v1 - v2) < delta_t * acc_thr2);
}

bool DynObjFilter::Case3VelCheck(float v1, float v2, double delta_t)
{
    return (std::fabs(v1 - v2) < delta_t * acc_thr3);
}

bool DynObjFilter::Case3(point_soph &p)
{
    if (dataset == 0 && p.is_distort) return false;

    int first_i = static_cast<int>(depth_map_list.size()) - 1;
    if (first_i < 0) return false;

    point_soph p_spherical = p;
    SphericalProjection(
        p,
        depth_map_list[first_i]->map_index,
        depth_map_list[first_i]->project_R,
        depth_map_list[first_i]->project_T,
        p_spherical);

    if (std::fabs(p_spherical.hor_ind) >= MAX_1D ||
        std::fabs(p_spherical.ver_ind) >= MAX_1D_HALF ||
        p_spherical.vec(2) < 0.0f ||
        p_spherical.position < 0 ||
        p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }

    int cur_occ_times = 0;

    if (Case3Enter(p_spherical, *depth_map_list[first_i]))
    {
        if (!Case3MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case3_interp_en))
        {
            double ti = 0.0;
            float vi = 0.0f;
            bool map_cons = true;

            for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3 && map_cons; ++ind_hor)
            {
                for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3 && map_cons; ++ind_ver)
                {
                    int pos_new =
                        ((p_spherical.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                        ((p_spherical.ver_ind + ind_ver) % MAX_1D_HALF);

                    if (pos_new < 0 || pos_new >= MAX_2D_N) continue;

                    const std::vector<point_soph*> &points_in_pixel =
                        depth_map_list[first_i]->depth_map[pos_new];

                    if (depth_map_list[first_i]->max_depth_all[pos_new] < p_spherical.vec(2))
                        continue;

                    for (size_t k = 0; k < points_in_pixel.size() && map_cons; ++k)
                    {
                        const point_soph *p_occ = points_in_pixel[k];

                        if (Case3IsOccluding(p_spherical, *p_occ) &&
                            Case3DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                        {
                            cur_occ_times = 1;
                            ti = (p_occ->time + p.time) * 0.5;
                            vi = (p_occ->vec(2) - p_spherical.vec(2)) /
                                 (p.time - p_occ->time);

                            p.is_occu_index[0] = depth_map_list[first_i]->map_index;
                            p.is_occu_index[1] = pos_new;
                            p.is_occu_index[2] = static_cast<int>(k);
                            p.is_occ_vec = p_spherical.vec;
                            p.is_occu_times = cur_occ_times;

                            point_soph p1 = *points_in_pixel[k];
                            int i = static_cast<int>(depth_map_list.size()) - 2;

                            while (i >= 0)
                            {
                                if (p1.is_occu_index[0] == -1 ||
                                    p1.is_occu_index[0] < depth_map_list.front()->map_index)
                                {
                                    SphericalProjection(
                                        p1,
                                        depth_map_list[i]->map_index,
                                        depth_map_list[i]->project_R,
                                        depth_map_list[i]->project_T,
                                        p1);

                                    if (Case3SearchPointOccludedbyP(p1, *depth_map_list[i]))
                                    {
                                        p1.is_occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }

                                i = p1.is_occu_index[0] -
                                    depth_map_list.front()->map_index;

                                point_soph *p2 =
                                    depth_map_list[i]->depth_map
                                        [p1.is_occu_index[1]]
                                        [p1.is_occu_index[2]];

                                SphericalProjection(
                                    p,
                                    depth_map_list[i]->map_index,
                                    depth_map_list[i]->project_R,
                                    depth_map_list[i]->project_T,
                                    p);

                                if (Case3MapConsistencyCheck(p, *depth_map_list[i], case3_interp_en))
                                {
                                    map_cons = false;
                                    break;
                                }

                                float vc =
                                    -(p1.is_occ_vec(2) - p2->vec(2)) /
                                    (p1.time - p2->time);
                                double tc = (p2->time + p1.time) * 0.5;

                                if (Case3IsOccluding(p, *p2) &&
                                    Case3DepthConsistencyCheck(*p2, *depth_map_list[i]) &&
                                    Case3VelCheck(vi, vc, ti - tc))
                                {
                                    ++cur_occ_times;
                                    if (cur_occ_times >= occluding_times_thr3)
                                    {
                                        p.is_occu_times = cur_occ_times;
                                        return true;
                                    }
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    break;
                                }
                                --i;
                            }
                        }
                        if (cur_occ_times >= occluding_times_thr3) break;
                    }
                }
            }
        }
    }

    if (cur_occ_times >= occluding_times_thr3)
    {
        p.is_occu_times = cur_occ_times;
        return true;
    }

    return false;
}

bool DynObjFilter::Case3Enter(point_soph &p, const DepthMap &map_info)
{
    if (p.dyn != STATIC) return false;

    float min_depth = 0.0f;
    float depth_thr3_final =
        std::max(cutoff_value,
                 k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) +
        occ_depth_thr3;

    if (!map_info.depth_map[p.position].empty())
    {
        const point_soph *min_point =
            map_info.depth_map[p.position]
                [map_info.min_depth_index_all[p.position]];
        min_depth = min_point->vec(2);

        float delta_t = static_cast<float>(p.time - min_point->time);
        depth_thr3_final = std::min(depth_thr3_final, v_min_thr3 * delta_t);
    }

    if (dataset == 0 && p.is_distort)
    {
        depth_thr3_final *= enlarge_distort;
    }

    if (p.vec(2) < min_depth - depth_thr3_final)
    {
        ++case3_num;
        return true;
    }

    return false;
}

bool DynObjFilter::Case3MapConsistencyCheck(point_soph &p,
                                           const DepthMap &map_info,
                                           bool interp)
{
    float cur_v_min = v_min_thr3;
    float cur_hor   = map_cons_hor_thr3;
    float cur_ver   = map_cons_ver_thr3;
    float cur_depth =
        std::max(cutoff_value,
                 k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) +
        map_cons_depth_thr3;

    if (dataset == 0 && p.is_distort)
        cur_v_min = enlarge_distort * cur_v_min;

    for (int ind_hor = -map_cons_hor_num3; ind_hor <= map_cons_hor_num3; ++ind_hor)
    {
        for (int ind_ver = -map_cons_ver_num3; ind_ver <= map_cons_ver_num3; ++ind_ver)
        {
            int pos_new =
                ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                ((p.ver_ind + ind_ver) % MAX_1D_HALF);

            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;

            const std::vector<point_soph *> &points_in_pixel =
                map_info.depth_map[pos_new];

            if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth &&
                map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }

            for (size_t j = 0; j < points_in_pixel.size(); ++j)
            {
                const point_soph *point = points_in_pixel[j];
                if (point->dyn == STATIC &&
                    std::fabs(p.time - point->time) > frame_dur &&
                    (point->vec(2) - p.vec(2)) < cur_depth &&
                    std::fabs(p.vec(0) - point->vec(0)) < cur_hor &&
                    std::fabs(p.vec(1) - point->vec(1)) < cur_ver)
                {
                    return true;
                }
            }
        }
    }

    if (interp &&
        (p.local(0) < self_x_b || p.local(0) > self_x_f ||
         p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float cur_interp =
            interp_thr3 *
            (depth_map_list.back()->map_index - map_info.map_index + 1);

        float depth_all =
            DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);

        if (std::fabs(p.vec(2) - depth_all) < cur_interp)
            return true;
        else
            return false;
    }

    return false;
}

bool DynObjFilter::Case3SearchPointOccludedbyP(point_soph &p,
                                              const DepthMap &map_info)
{
    for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ++ind_hor)
    {
        for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ++ind_ver)
        {
            int pos_new =
                ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                ((p.ver_ind + ind_ver) % MAX_1D_HALF);

            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;

            const std::vector<point_soph *> &points_in_pixel =
                map_info.depth_map[pos_new];

            if (map_info.min_depth_all[pos_new] > p.vec(2))
                continue;

            for (size_t j = 0; j < points_in_pixel.size(); ++j)
            {
                const point_soph *p_cond = points_in_pixel[j];
                if (Case3IsOccluding(p, *p_cond) &&
                    Case3DepthConsistencyCheck(*p_cond, map_info))
                {
                    p.is_occu_index[0] = map_info.map_index;
                    p.is_occu_index[1] = pos_new;
                    p.is_occu_index[2] = static_cast<int>(j);
                    p.occ_vec = p.vec;
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynObjFilter::Case3IsOccluding(const point_soph &p,
                                   const point_soph &p_occ)
{
    if ((dataset == 0 && p_occ.is_distort) ||
        (dataset == 0 && p.is_distort) ||
        p_occ.dyn == INVALID)
        return false;

    if ((p.local(0) > self_x_b && p.local(0) < self_x_f &&
         p.local(1) < self_y_l && p.local(1) > self_y_r) ||
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f &&
         p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }

    float delta_t = p.time - p_occ.time;
    if (delta_t > 0.0f)
    {
        float depth_thr3_final =
            std::min(std::max(cutoff_value,
                              k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) +
                         map_cons_depth_thr3,
                     v_min_thr3 * delta_t);

        if (dataset == 0 && p.is_distort)
            depth_thr3_final = enlarge_distort * depth_thr3_final;

        if (p_occ.vec(2) > p.vec(2) + depth_thr3_final &&
            std::fabs(p.vec(0) - p_occ.vec(0)) < occ_hor_thr3 &&
            std::fabs(p.vec(1) - p_occ.vec(1)) < occ_ver_thr3)
        {
            return true;
        }
    }
    return false;
}

bool DynObjFilter::Case3DepthConsistencyCheck(const point_soph &p,
                                             const DepthMap &map_info)
{
    float all_minus = 0.0f;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;

    for (int ind_hor = -depth_cons_hor_num3;
         ind_hor <= depth_cons_hor_num3; ++ind_hor)
    {
        for (int ind_ver = -depth_cons_ver_num3;
             ind_ver <= depth_cons_ver_num3; ++ind_ver)
        {
            int pos_new =
                ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                ((p.ver_ind + ind_ver) % MAX_1D_HALF);

            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;

            const std::vector<point_soph *> &points_in_pixel =
                map_info.depth_map[pos_new];

            for (size_t j = 0; j < points_in_pixel.size(); ++j)
            {
                const point_soph *point = points_in_pixel[j];
                if (std::fabs(point->time - p.time) < frame_dur &&
                    std::fabs(point->vec(0) - p.vec(0)) <
                        depth_cons_hor_thr3 &&
                    std::fabs(point->vec(1) - p.vec(1)) <
                        depth_cons_ver_thr3)
                {
                    ++all_num;
                    if (point->dyn == STATIC)
                    {
                        float cur_minus = p.vec(2) - point->vec(2);
                        if (std::fabs(cur_minus) <
                            depth_cons_depth_max_thr3)
                        {
                            ++num;
                            all_minus +=
                                std::fabs(point->vec(2) - p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            ++smaller_num;
                        }
                        else
                        {
                            ++greater_num;
                        }
                    }
                }
            }
        }
    }

    if (all_num > 0)
    {
        if (num > 1)
        {
            float cur_depth_thr =
                std::max(depth_cons_depth_thr3, k_depth3 * p.vec(2));
            if (all_minus / (num - 1) > cur_depth_thr)
                return false;
        }

        if (greater_num == 0 || smaller_num == 0)
            return true;
        else
            return false;
    }

    return false;
}

void DynObjFilter::publish_dyn(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_point_out,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_frame_out,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_steady_points,
    const double &scan_end_time)
{
    if (cluster_coupled)
    {
        std::cout << "Found Dynamic Objects, numbers: "
                  << laserCloudDynObj_clus->points.size()
                  << " Total time: " << time_total
                  << " Average total time: " << time_total_avr
                  << std::endl;
    }
    else
    {
        std::cout << "Found Dynamic Objects, numbers: "
                  << laserCloudDynObj->points.size()
                  << " Total time: " << time_total
                  << " Average total time: " << time_total_avr
                  << std::endl;
    }

    std::cout << "case1 num: " << case1_num
              << " case2 num: " << case2_num
              << " case3 num: " << case3_num << std::endl;

    case1_num = case2_num = case3_num = 0;

    /* -------- publish dynamic objects (world) -------- */
    sensor_msgs::msg::PointCloud2 cloud_dyn_world;
    pcl::toROSMsg(*laserCloudDynObj_world, cloud_dyn_world);
    cloud_dyn_world.header.stamp =
        rclcpp::Time(static_cast<uint64_t>(scan_end_time * 1e9));
    cloud_dyn_world.header.frame_id = frame_id;
    pub_point_out->publish(cloud_dyn_world);

    /* -------- publish clustered dynamic objects -------- */
    if (cluster_coupled || cluster_future)
    {
        sensor_msgs::msg::PointCloud2 cloud_dyn_clus;
        pcl::toROSMsg(*laserCloudDynObj_clus, cloud_dyn_clus);
        cloud_dyn_clus.header.stamp =
            rclcpp::Time(static_cast<uint64_t>(scan_end_time * 1e9));
        cloud_dyn_clus.header.frame_id = frame_id;
        pub_frame_out->publish(cloud_dyn_clus);
    }

    /* -------- publish steady objects -------- */
    sensor_msgs::msg::PointCloud2 cloud_steady;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);

    if (cluster_coupled)
    {
        if (laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
        }

        for (size_t i = 0; i < laserCloudSteadObj_accu.size(); ++i)
            *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];

        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size,
                                     voxel_filter_size,
                                     voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);

        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, cloud_steady);
    }
    else
    {
        std::cout << "Found Steady Objects, numbers: "
                  << laserCloudSteadObj->points.size() << std::endl;

        if (laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
        }

        for (size_t i = 0; i < laserCloudSteadObj_accu.size(); ++i)
            *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];

        pcl::toROSMsg(*laserCloudSteadObj_pub, cloud_steady);
    }

    cloud_steady.header.stamp =
        rclcpp::Time(static_cast<uint64_t>(scan_end_time * 1e9));
    cloud_steady.header.frame_id = frame_id;
    pub_steady_points->publish(cloud_steady);
}

void DynObjFilter::set_path(const std::string &file_path,
                            const std::string &file_path_origin)
{
    is_set_path = true;
    out_file = file_path;
    out_file_origin = file_path_origin;
}
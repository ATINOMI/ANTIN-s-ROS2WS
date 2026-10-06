// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "mini_nav_core/localization/fastlio2/fastlio_estimator.hpp"
#include "imu_processing.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <ikd-Tree/ikd_Tree.h>
#include <limits>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

namespace mini_nav_core::fastlio2 {
struct FastlioEstimator::Impl {
    /*** Time Log Variables ***/
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN],
        s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool runtime_pos_log = false, extrinsic_est_en = true;
    /**************************/

    float res_last[100000] = {0.0};
    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;

    string root_dir;

    double res_mean_last = 0.05, total_residual = 0.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, first_lidar_time = 0.0;
    int effct_feat_num = 0, time_log_counter = 0;
    int feats_down_size = 0, NUM_MAX_ITERATIONS = 0;
    bool point_selected_surf[100000] = {0};
    bool flg_first_scan = true, flg_EKF_inited;
    bool ikd_tree_pub_en = false;

    vector<vector<int>> pointSearchInd_surf;
    vector<BoxPointType> cub_needrm;
    vector<PointVector> Nearest_Points;
    vector<double> extrinT = vector<double>(3, 0.0);
    vector<double> extrinR = vector<double>(9, 0.0);

    PointCloudXYZI::Ptr featsFromMap{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_undistort{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()}; // downsampled point cloud in body frame
    PointCloudXYZI::Ptr feats_down_world{
        new PointCloudXYZI()}; // downsampled point cloud in world frame, transformed from body frame
    PointCloudXYZI::Ptr feats_down_prior_map{new PointCloudXYZI()};
    PointCloudXYZI::Ptr normvec{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr laserCloudOri{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI(100000, 1)};

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    KD_TREE<PointType> ikdtree;

    V3F XAxisPoint_body{LIDAR_SP_LEN, 0.0, 0.0};
    V3F XAxisPoint_world{LIDAR_SP_LEN, 0.0, 0.0};
    V3D euler_cur;
    V3D position_last{Zero3d};
    V3D Lidar_T_wrt_IMU{Zero3d};
    M3D Lidar_R_wrt_IMU{Eye3d};

    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    vect3 pos_lid;

    shared_ptr<ImuProcess> p_imu{new ImuProcess()};

    bool locate_in_prior_map = false;
    FILE *fp{nullptr};
    ofstream fout_pre, fout_out, fout_dbg;

    FastlioResult result;
    Eigen::Matrix4d initial_transform{Eigen::Matrix4d::Identity()};
    double epsi[23] = {0.001};
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0;
    double aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    explicit Impl(const FastlioConfig &config) {
        NUM_MAX_ITERATIONS = config.iterations;
        filter_size_surf_min = config.filter_size_surf;
        filter_size_map_min = config.filter_size_map;
        cube_len = config.cube_length;
        DET_RANGE = config.detection_range;
        fov_deg = config.fov_degree;
        gyr_cov = config.gyr_cov;
        acc_cov = config.acc_cov;
        b_gyr_cov = config.gyro_bias_cov;
        b_acc_cov = config.acc_bias_cov;
        extrinsic_est_en = config.estimate_extrinsics;
        locate_in_prior_map = config.prior_mode;
        ikd_tree_pub_en = config.capture_map;
        runtime_pos_log = config.runtime_log;
        root_dir = config.log_root;
        extrinT.assign(config.extrinsic_translation.data(), config.extrinsic_translation.data() + 3);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                extrinR[i * 3 + j] = config.extrinsic_rotation(i, j);
        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->snapshot_input = config.snapshot_input;
        if (runtime_pos_log && !root_dir.empty())
            p_imu->debug_log_path = root_dir + "/Log/imu.txt";
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(
            get_f, df_dx, df_dw,
            [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &data) { h_share_model(s, data); },
            NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        if (runtime_pos_log && !root_dir.empty()) {
            string pos_log_dir = root_dir + "/Log/pos_log.txt";
            fp = fopen(pos_log_dir.c_str(), "w");
            fout_pre.open(root_dir + "/Log/mat_pre.txt", ios::out);
            fout_out.open(root_dir + "/Log/mat_out.txt", ios::out);
            fout_dbg.open(root_dir + "/Log/dbg.txt", ios::out);
        }

        result.undistorted = feats_undistort;
        result.downsampled = feats_down_body;
        result.effective = laserCloudOri;
        result.prior_map = feats_down_prior_map;
        result.map_points = featsFromMap;
    }
    ~Impl() {
        fout_out.close();
        fout_pre.close();
        fout_dbg.close();
        if (fp)
            fclose(fp);

        if (runtime_pos_log) {
            vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
            FILE *fp2;
            string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
            fp2 = fopen(log_dir.c_str(), "w");
            if (fp2)
                fprintf(
                    fp2,
                    "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
            for (int i = 0; fp2 && i < time_log_counter; i++) {
                fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i],
                        int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]),
                        int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
                t.push_back(T1[i]);
                s_vec.push_back(s_plot9[i]);
                s_vec2.push_back(s_plot3[i] + s_plot6[i]);
                s_vec3.push_back(s_plot4[i]);
                s_vec5.push_back(s_plot[i]);
            }
            if (fp2)
                fclose(fp2);
        }
    }
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
        double match_start = omp_get_wtime();
        laserCloudOri->resize(feats_down_size);
        corr_normvect->resize(feats_down_size);
        total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size; i++) {
            PointType &point_body = feats_down_body->points[i];
            PointType &point_world = feats_down_world->points[i];

            /* transform to world frame */
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto &points_near = Nearest_Points[i];

            if (ekfom_data.converge) {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                         : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                      : true;
            }

            if (!point_selected_surf[i])
                continue;

            VF(4)
            pabcd;
            point_selected_surf[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f)) {
                float pd2 =
                    pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9) {
                    point_selected_surf[i] = true;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
        }

        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++) {
            if (point_selected_surf[i]) {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                total_residual += res_last[i];
                effct_feat_num++;
            }
        }

        laserCloudOri->resize(effct_feat_num);
        corr_normvect->resize(effct_feat_num);
        if (effct_feat_num < 1) {
            ekfom_data.valid = false;
            std::cerr << "No Effective Points!" << std::endl;
            // ROS_WARN("No Effective Points! \n");
            return;
        }

        res_mean_last = total_residual / effct_feat_num;
        match_time += omp_get_wtime() - match_start;
        double solve_start_ = omp_get_wtime();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
        ekfom_data.h.resize(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++) {
            const PointType &laser_p = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() * norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en) {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                    VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            } else {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time += omp_get_wtime() - solve_start_;
    }
    inline void dump_lio_state_to_log(FILE *fp) {
        if (!fp)
            return;
        V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
        fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
        fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
        fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
        fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
        fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
        fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
        fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
        fprintf(fp, "\r\n");
        fflush(fp);
    }

    void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s) //! not used
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    /**
     * @brief pointBodyToWorld
     * @param pi: input point in body frame
     * @param po: output point in world frame
     * @details transform point from body frame to world frame, not used for residual calculation

    */
    void pointBodyToWorld(PointType const *const pi, PointType *const po) {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                     state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    template <typename T> void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                     state_point.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                     state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
        V3D p_body_lidar(pi->x, pi->y, pi->z);
        V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

        po->x = p_body_imu(0);
        po->y = p_body_imu(1);
        po->z = p_body_imu(2);
        po->intensity = pi->intensity;
    }

    void points_cache_collect() {
        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);
        // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
    }

    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;
    void lasermap_fov_segment() {
        cub_needrm.clear();
        kdtree_delete_counter = 0;
        kdtree_delete_time = 0.0;
        pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
        V3D pos_LiD = pos_lid;
        if (!Localmap_Initialized) {
            for (int i = 0; i < 3; i++) {
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++) {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
                dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
                need_move = true;
        }
        if (!need_move)
            return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                             double(DET_RANGE * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++) {
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        points_cache_collect();
        double delete_begin = omp_get_wtime();
        if (cub_needrm.size() > 0)
            kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
        kdtree_delete_time = omp_get_wtime() - delete_begin;
    }

    int process_increments = 0;
    void map_incremental() {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for (int i = 0; i < feats_down_size; i++) {
            /* transform to world frame */
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            /* decide if need add to map */
            if (!Nearest_Points[i].empty() && flg_EKF_inited) {
                const PointVector &points_near = Nearest_Points[i];
                bool need_add = true;
                BoxPointType Box_of_Point;
                PointType downsample_result, mid_point;
                mid_point.x =
                    floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                    0.5 * filter_size_map_min;
                mid_point.y =
                    floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                    0.5 * filter_size_map_min;
                mid_point.z =
                    floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                    0.5 * filter_size_map_min;
                float dist = calc_dist(feats_down_world->points[i], mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                    fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                    fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                    PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                    if (points_near.size() < NUM_MATCH_POINTS)
                        break;
                    if (calc_dist(points_near[readd_i], mid_point) < dist) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add)
                    PointToAdd.push_back(feats_down_world->points[i]);
            } else {
                PointToAdd.push_back(feats_down_world->points[i]);
            }
        }

        double st_time = omp_get_wtime();
        add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
        kdtree_incremental_time = omp_get_wtime() - st_time;
    }

    void CaptureResult() {
        result.pos = state_point.pos;
        result.vel = state_point.vel;
        result.bg = state_point.bg;
        result.rot = Eigen::Quaterniond(state_point.rot.toRotationMatrix());
        result.offset_R_L_I = Eigen::Quaterniond(state_point.offset_R_L_I.toRotationMatrix());
        result.offset_T_L_I = state_point.offset_T_L_I;
        const auto P = kf.get_P();
        for (int i = 0; i < 6; ++i) {
            const int k = i < 3 ? i + 3 : i - 3;
            for (int j = 0; j < 6; ++j)
                result.covariance(i, j) = P(k, j < 3 ? j + 3 : j - 3);
        }
        result.effective_points = effct_feat_num;
        result.input_points = feats_down_size;
        result.mean_abs_residual =
            effct_feat_num > 0 ? res_mean_last : std::numeric_limits<double>::infinity();
    }
    const FastlioResult &Process(const MeasureGroup &measurements) {
        Measures = measurements;
        result.stamp = Measures.lidar_end_time;
        result.valid = result.match_available = result.prior_initialized = result.map_captured = false;
        if (!Measures.lidar || Measures.lidar->empty() || Measures.imu.empty())
            return result;
        if (flg_first_scan) {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            return result;
        }

        double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

        match_time = 0;
        kdtree_search_time = 0.0;
        solve_time = 0;
        solve_const_H_time = 0;
        svd_time = 0;
        t0 = omp_get_wtime();
        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        // RCLCPP_INFO(this->get_logger(),"pos_lid: %f %f %f", pos_lid(0), pos_lid(1), pos_lid(2));

        if (feats_undistort->empty() || (feats_undistort == NULL)) {
            return result;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        t1 = omp_get_wtime();
        feats_down_size = feats_down_body->points.size();

        // if (scan_pub_en && scan_body_pub_en)
        //     publish_frame_body(pubLaserCloudFull_body_);
        /*** initialize the map kdtree ***/
        if (ikdtree.Root_Node == nullptr) {
            if (locate_in_prior_map) {
                // if (!initial_pose_received)
                // {
                //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Waiting for initial pose...");
                //     return;
                // }
                pcl::transformPointCloud(*feats_down_prior_map, *feats_down_prior_map,
                                         initial_transform.inverse().cast<float>().eval());
                result.prior_initialized = true;
                ikdtree.Build(feats_down_prior_map->points);
            } else if (feats_down_size > 5) {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);
            }
            return result;
        }
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();

        // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5) {
            return result;
        }

        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose()
                 << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                 << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                 << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav
                 << endl;

        if (ikd_tree_pub_en) // If you need to see map point, change to "if(1)"
        {
            PointVector().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
            result.map_captured = true;
        }

        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();

        /*** iterated state estimation ***/
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        double t_update_end = omp_get_wtime();

        CaptureResult();
        result.match_available = true;
        if (locate_in_prior_map && (effct_feat_num < 30 || res_mean_last > 0.15)) {
            return result;
        }
        result.valid = true;
        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        if (!locate_in_prior_map) {
            map_incremental();
        }
        t5 = omp_get_wtime();

        /*** Debug variables ***/
        if (runtime_pos_log && time_log_counter < MAXN) {
            frame_num++;
            kdtree_size_end = ikdtree.size();
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            aver_time_icp =
                aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
            aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
            aver_time_incre =
                aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
            aver_time_solve =
                aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
            aver_time_const_H_time =
                aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = kdtree_incremental_time;
            s_plot4[time_log_counter] = kdtree_search_time;
            s_plot5[time_log_counter] = kdtree_delete_counter;
            s_plot6[time_log_counter] = kdtree_delete_time;
            s_plot7[time_log_counter] = kdtree_size_st;
            s_plot8[time_log_counter] = kdtree_size_end;
            s_plot9[time_log_counter] = aver_time_consu;
            s_plot10[time_log_counter] = add_point_size;
            s_plot11[time_log_counter] = 0.0;
            time_log_counter++;
            printf(
                "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
                t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                aver_time_const_H_time);
            ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose()
                     << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                     << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                     << state_point.bg.transpose() << " " << state_point.ba.transpose() << " "
                     << state_point.grav << " " << feats_undistort->points.size() << endl;
            dump_lio_state_to_log(fp);
        }
        return result;
    }
};
FastlioEstimator::FastlioEstimator(const FastlioConfig &config) : impl_(new Impl(config)) {}
FastlioEstimator::~FastlioEstimator() = default;
void FastlioEstimator::SetPriorMap(const PointCloudXYZI::ConstPtr &map) {
    impl_->downSizeFilterMap.setInputCloud(map);
    impl_->downSizeFilterMap.filter(*impl_->feats_down_prior_map);
}
void FastlioEstimator::SetInitialPose(const Eigen::Matrix4d &pose) { impl_->initial_transform = pose; }
const FastlioResult &FastlioEstimator::Process(const MeasureGroup &measurements) {
    return impl_->Process(measurements);
}
const FastlioResult &FastlioEstimator::Result() const { return impl_->result; }
void FastlioEstimator::PointBodyToWorld(const PointType *input, PointType *output) const {
    impl_->RGBpointBodyToWorld(input, output);
}
void FastlioEstimator::PointLidarToImu(const PointType *input, PointType *output) const {
    impl_->RGBpointBodyLidarToIMU(input, output);
}
}

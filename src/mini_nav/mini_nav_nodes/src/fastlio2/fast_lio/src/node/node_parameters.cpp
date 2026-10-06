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
#include "mini_nav_nodes/fastlio2/fastlio_node.hpp"

namespace mini_nav_nodes::fastlio2 {
using namespace std;
void FastlioNode::read_parameters() {
    this->declare_parameter<bool>("publish.path_en", true);
    this->declare_parameter<bool>("publish.effect_map_en", false);
    this->declare_parameter<bool>("publish.map_en", false);
    this->declare_parameter<bool>("publish.ikd_tree_en", false);
    this->declare_parameter<bool>("publish.scan_publish_en", true);
    this->declare_parameter<bool>("publish.dense_publish_en", true);
    this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    this->declare_parameter<int>("max_iteration", 4);
    this->declare_parameter<string>("map_file_path", "");
    this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<bool>("common.time_sync_en", false);
    this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    this->declare_parameter<string>("common.odom_frame_id", "odom");
    this->declare_parameter<string>("common.sensor_frame_id", "sensor");
    this->declare_parameter<string>("common.base_frame_id", "base_frame");
    this->declare_parameter<bool>("common.send_odom_base_tf", false);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("filter_size_corner", 0.5);
    this->declare_parameter<double>("filter_size_surf", 0.5);
    this->declare_parameter<double>("filter_size_map", 0.5);
    this->declare_parameter<double>("cube_side_length", 200.);
    this->declare_parameter<float>("mapping.det_range", 300.);
    this->declare_parameter<double>("mapping.fov_degree", 180.);
    this->declare_parameter<double>("mapping.gyr_cov", 0.1);
    this->declare_parameter<double>("mapping.acc_cov", 0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    this->declare_parameter<double>("preprocess.blind", 0.01);
    this->declare_parameter<int>("preprocess.lidar_type", AVIA);
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.timestamp_unit", US);
    this->declare_parameter<int>("preprocess.scan_rate", 10);
    this->declare_parameter<int>("point_filter_num", 2);
    this->declare_parameter<bool>("feature_extract_enable", false);
    this->declare_parameter<bool>("runtime_pos_log_enable", false);
    this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    this->declare_parameter<int>("pcd_save.interval", -1);
    this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
    this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
    this->declare_parameter<bool>("locate_in_prior_map", false);
    this->declare_parameter<string>("prior_map_path", "");

    this->get_parameter_or<bool>("publish.path_en", path_en, true);
    this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
    this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
    this->get_parameter_or<bool>("publish.ikd_tree_en", ikd_tree_pub_en, false);
    this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
    this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
    this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
    this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    this->get_parameter_or<string>("map_file_path", map_file_path, "");
    this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
    this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
    this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
    this->get_parameter_or<string>("common.odom_frame_id", odom_frame, "odom");
    this->get_parameter_or<string>("common.sensor_frame_id", sensor_frame, "sensor");
    this->get_parameter_or<string>("common.base_frame_id", base_frame, "base_frame");
    this->get_parameter_or<bool>("common.send_odom_base_tf", send_odom_base_tf, false);
    this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
    this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
    this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
    this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
    this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
    this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
    this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
    this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
    this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
    this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
    this->get_parameter_or<double>("preprocess.blind", p_pre->preprocessing.blind, 0.01);
    this->get_parameter_or<int>("preprocess.lidar_type", p_pre->preprocessing.lidar_type, AVIA);
    this->get_parameter_or<int>("preprocess.scan_line", p_pre->preprocessing.N_SCANS, 16);
    this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->preprocessing.time_unit, US);
    this->get_parameter_or<int>("preprocess.scan_rate", p_pre->preprocessing.SCAN_RATE, 10);
    this->get_parameter_or<int>("point_filter_num", p_pre->preprocessing.point_filter_num, 2);
    this->get_parameter_or<bool>("feature_extract_enable", p_pre->preprocessing.feature_enabled, false);
    this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
    this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
    this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
    this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
    this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
    this->get_parameter_or<bool>("locate_in_prior_map", locate_in_prior_map, false);
    this->get_parameter_or<string>("prior_map_path", prior_map_path, "");
}
}

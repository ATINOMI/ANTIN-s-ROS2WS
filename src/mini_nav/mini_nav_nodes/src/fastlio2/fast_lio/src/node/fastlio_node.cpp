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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <stdexcept>

namespace mini_nav_nodes::fastlio2 {
using namespace std;
FastlioNode::FastlioNode(const rclcpp::NodeOptions &options) : Node("laser_mapping_node", options) {
    read_parameters();
    if (extrinT.size() != 3 || extrinR.size() != 9)
        throw std::invalid_argument("FAST-LIO2 extrinsics require 3 translation and 9 rotation values");
    FastlioConfig config;
    config.iterations = NUM_MAX_ITERATIONS;
    config.filter_size_surf = filter_size_surf_min;
    config.filter_size_map = filter_size_map_min;
    config.cube_length = cube_len;
    config.detection_range = DET_RANGE;
    config.fov_degree = fov_deg;
    config.gyr_cov = gyr_cov;
    config.acc_cov = acc_cov;
    config.gyro_bias_cov = b_gyr_cov;
    config.acc_bias_cov = b_acc_cov;
    config.extrinsic_translation = Eigen::Vector3d(extrinT[0], extrinT[1], extrinT[2]);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            config.extrinsic_rotation(i, j) = extrinR[i * 3 + j];
    config.snapshot_input = p_pre->preprocessing.lidar_type == GAZEBO_SNAPSHOT;
    config.estimate_extrinsics = extrinsic_est_en;
    config.prior_mode = locate_in_prior_map;
    config.capture_map = ikd_tree_pub_en;
    config.runtime_log = runtime_pos_log;
    config.log_root = ROOT_DIR;
    estimator_.reset(new FastlioEstimator(config));
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = odom_frame;
    /*** ROS subscribe initialization ***/
    if (p_pre->preprocessing.lidar_type == AVIA) {
        sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, 20, std::bind(&FastlioNode::livox_pcl_cbk, this, std::placeholders::_1));
    } else {
        sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::SensorDataQoS(),
            std::bind(&FastlioNode::standard_pcl_cbk, this, std::placeholders::_1));
    }
    if (locate_in_prior_map) {
        sub_init_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/icp_result", rclcpp::QoS(1).transient_local(),
            std::bind(&FastlioNode::initial_pose_cbk, this, std::placeholders::_1));
    }
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(), std::bind(&FastlioNode::imu_cbk, this, std::placeholders::_1));
    pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
    pubLaserCloudFull_body_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
    pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
    pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
    pubIkdTree_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ikd_tree", 20);
    pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
    pubMatchQuality_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/scurm/match_quality", 10);
    pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
                                  std::bind(&FastlioNode::timer_callback, this));
    auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms,
                                          std::bind(&FastlioNode::map_publish_callback, this));

    map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "map_save",
        std::bind(&FastlioNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

    if (locate_in_prior_map) {
        RCLCPP_INFO(this->get_logger(), "Loading prior map...");
        PointCloudXYZI::Ptr prior_map(new PointCloudXYZI());
        // load prior map
        if (pcl::io::loadPCDFile<PointType>(prior_map_path, *prior_map) == -1) // Replace with your file name
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file\n");
            return;
        }
        // downsample the prior map
        estimator_->SetPriorMap(prior_map);
    }
    RCLCPP_INFO(this->get_logger(), "Node init finished.");
}
void FastlioNode::timer_callback() {
    if (locate_in_prior_map && !initial_pose_received) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Waiting for initial pose...");
        return;
    }
    if (!sync_packages(Measures))
        return;
    const auto &result = estimator_->Process(Measures);
    lidar_end_time = result.stamp;
    if (result.prior_initialized) {
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*result.prior_map, message);
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = odom_frame;
        pubLaserCloudMap_->publish(message);
    }
    if (result.map_captured)
        publish_ikd_tree(pubIkdTree_);
    if (!result.match_available)
        return;
    geoQuat.x = result.rot.x();
    geoQuat.y = result.rot.y();
    geoQuat.z = result.rot.z();
    geoQuat.w = result.rot.w();
    diagnostic_msgs::msg::DiagnosticArray match;
    match.header.stamp = get_ros_time(lidar_end_time);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fastlio_prior_match";
    status.hardware_id = "scurm_fastlio2";
    const auto add_metric = [&](const std::string &key, double value) {
        diagnostic_msgs::msg::KeyValue metric;
        metric.key = key;
        metric.value = std::to_string(value);
        status.values.push_back(metric);
    };
    add_metric("effective_points", result.effective_points);
    add_metric("matched_ratio",
               static_cast<double>(result.effective_points) / std::max(1, result.input_points));
    add_metric("mean_abs_residual", result.effective_points > 0 ? result.mean_abs_residual
                                                                : std::numeric_limits<double>::infinity());
    status.level = result.effective_points >= 30 && result.mean_abs_residual <= 0.15 ? 0 : 2;
    match.status.push_back(status);
    pubMatchQuality_->publish(match);

    /******* Publish odometry *******/
    if (locate_in_prior_map && (result.effective_points < 30 || result.mean_abs_residual > 0.15)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Withholding odometry: prior-map match is invalid");
        return;
    }
    publish_odometry(pubOdomAftMapped_);

    /******* Publish points *******/
    if (path_en)
        publish_path(pubPath_);
    if (scan_pub_en)
        publish_frame_world(pubLaserCloudFull_);
    if (scan_pub_en && scan_body_pub_en)
        publish_frame_body(pubLaserCloudFull_body_);
    if (effect_pub_en)
        publish_effect_world(pubLaserCloudEffect_);
}
void FastlioNode::map_publish_callback() {
    if (map_pub_en)
        publish_map(pubLaserCloudMap_);
}

void FastlioNode::map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                                    std_srvs::srv::Trigger::Response::SharedPtr res) {
    RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
    if (pcd_save_en) {
        save_to_pcd();
        res->success = true;
        res->message = "Map saved.";
    } else {
        res->success = false;
        res->message = "Map save disabled.";
    }
}

void FastlioNode::initial_pose_cbk(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const auto &p = msg->pose.pose;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond orientation(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-12)
        return;
    pose.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    estimator_->SetInitialPose(pose);
    initial_pose_received = true;
}
void FastlioNode::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_pub->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_pub);
    }
}
}

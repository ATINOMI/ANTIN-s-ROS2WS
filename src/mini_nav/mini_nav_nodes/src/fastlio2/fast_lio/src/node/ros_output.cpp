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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PUBFRAME_PERIOD (20)
namespace mini_nav_nodes::fastlio2 {
using namespace std;
using V3D = Eigen::Vector3d;
template <typename T> void FastlioNode::set_posestamp(T &out) {
    const auto &state_point = estimator_->Result();
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void FastlioNode::publish_frame_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull) {
    const auto &state_point = estimator_->Result();
    const auto &feats_undistort = state_point.undistorted;
    const auto &feats_down_body = state_point.downsampled;
    if (scan_pub_en) {
        PointCloudXYZI::ConstPtr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            estimator_->PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_frame;
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
}

void FastlioNode::publish_frame_body(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body) {
    const auto &state_point = estimator_->Result();
    const auto &feats_undistort = state_point.undistorted;
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        estimator_->PointLidarToImu(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = sensor_frame;
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void FastlioNode::publish_effect_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect) {
    const auto &state_point = estimator_->Result();
    const auto &laserCloudOri = state_point.effective;
    const int effct_feat_num = state_point.effective_points;
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
        estimator_->PointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = odom_frame;
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void FastlioNode::publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap) {
    const auto &state_point = estimator_->Result();
    const auto &feats_undistort = state_point.undistorted;
    const auto &feats_down_body = state_point.downsampled;
    PointCloudXYZI::ConstPtr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        estimator_->PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_frame;
    pubLaserCloudMap->publish(laserCloudmsg);
}

void FastlioNode::publish_ikd_tree(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap) {
    const auto &state_point = estimator_->Result();
    const auto &featsFromMap = state_point.map_points;
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    laserCloudMap.header.frame_id = odom_frame;
    pubLaserCloudMap->publish(laserCloudMap);
}

void FastlioNode::save_to_pcd() {
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

void FastlioNode::publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped) {
    const auto &state_point = estimator_->Result();
    odomAftMapped.header.frame_id = odom_frame;
    odomAftMapped.child_frame_id = sensor_frame;
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    if (send_odom_base_tf) {
        geometry_msgs::msg::TransformStamped base_to_sensor_msg;
        try { // lookupTransform(to_frame, from_frame, ...)
            base_to_sensor_msg = tf_buffer_->lookupTransform(sensor_frame, base_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("laserMapping"), "%s", ex.what());
            return;
        }
        tf2::Transform tf_base_to_sensor;
        tf2::fromMsg(base_to_sensor_msg.transform, tf_base_to_sensor);
        tf2::Transform tf_odom_to_snesor;
        tf2::fromMsg(odomAftMapped.pose.pose, tf_odom_to_snesor);
        tf2::Transform tf_odom_to_base = tf_odom_to_snesor * tf_base_to_sensor;
        nav_msgs::msg::Odometry odom_to_base_msg;
        odom_to_base_msg.header.stamp = odomAftMapped.header.stamp;
        odom_to_base_msg.header.frame_id = odom_frame;
        odom_to_base_msg.child_frame_id = base_frame;
        odom_to_base_msg.pose.pose.position.x = tf_odom_to_base.getOrigin().getX();
        odom_to_base_msg.pose.pose.position.y = tf_odom_to_base.getOrigin().getY();
        odom_to_base_msg.pose.pose.position.z = tf_odom_to_base.getOrigin().getZ();
        odom_to_base_msg.pose.pose.orientation.x = tf_odom_to_base.getRotation().getX();
        odom_to_base_msg.pose.pose.orientation.y = tf_odom_to_base.getRotation().getY();
        odom_to_base_msg.pose.pose.orientation.z = tf_odom_to_base.getRotation().getZ();
        odom_to_base_msg.pose.pose.orientation.w = tf_odom_to_base.getRotation().getW();
        odomAftMapped = odom_to_base_msg;
    }
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            odomAftMapped.pose.covariance[i * 6 + j] = state_point.covariance(i, j);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = odom_frame;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = sensor_frame;
    if (send_odom_base_tf) {
        trans.child_frame_id = base_frame;
    }
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    const V3D body_velocity = state_point.rot.conjugate() * state_point.vel;
    odomAftMapped.twist.twist.linear.x = body_velocity.x();
    odomAftMapped.twist.twist.linear.y = body_velocity.y();
    odomAftMapped.twist.twist.linear.z = body_velocity.z();
    if (!Measures.imu.empty()) {
        const auto &gyro = Measures.imu.back()->angular_velocity;
        odomAftMapped.twist.twist.angular.x = gyro.x - state_point.bg.x();
        odomAftMapped.twist.twist.angular.y = gyro.y - state_point.bg.y();
        odomAftMapped.twist.twist.angular.z = gyro.z - state_point.bg.z();
    }
    pubOdomAftMapped->publish(odomAftMapped);
    if (this->get_parameter("publish_tf").as_bool())
        tf_broadcaster_->sendTransform(trans);
}

void FastlioNode::publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = odom_frame;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        path.poses.push_back(msg_body_pose);
        path.header.stamp = msg_body_pose.header.stamp;
        pubPath->publish(path);
    }
}
}

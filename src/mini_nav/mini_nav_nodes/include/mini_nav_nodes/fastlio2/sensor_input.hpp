#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "mini_nav_core/localization/fastlio2/pointcloud_preprocess.hpp"

namespace mini_nav_nodes::fastlio2 {
using namespace mini_nav_core::fastlio2;
class SensorInput {
  public:
    Preprocess preprocessing;
    void process(const sensor_msgs::msg::PointCloud2::UniquePtr &message, PointCloudXYZI::Ptr &output);
    void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &message, PointCloudXYZI::Ptr &output);
};
double get_time_sec(const builtin_interfaces::msg::Time &time);
rclcpp::Time get_ros_time(double timestamp);
ImuSample::Ptr DecodeImu(const sensor_msgs::msg::Imu &message);
}

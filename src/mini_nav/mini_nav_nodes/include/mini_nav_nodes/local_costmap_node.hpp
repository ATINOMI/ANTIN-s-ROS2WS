#pragma once

#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "mini_nav_core/map/rolling_obstacle_grid.hpp"

namespace mini_nav_nodes
{
    /** 将扫描观测维护为 odom 坐标系中的滚动局部代价图。 */
    class LocalCostmapNode : public rclcpp::Node
    {
    public:
        LocalCostmapNode();

    private:
        void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
        void publishMap();
        void invalidate(const char * reason);

        std::unique_ptr<mini_nav_core::RollingObstacleGrid> grid_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string odom_frame_id_;
        std::string base_frame_id_;
        double max_scan_age_ = 1.0;
        double observation_persistence_ = 2.0;
        double obstacle_max_range_ = 2.5;
        double raytrace_max_range_ = 3.0;
        std::optional<rclcpp::Time> last_scan_stamp_;
        bool valid_ = false;
    };
}

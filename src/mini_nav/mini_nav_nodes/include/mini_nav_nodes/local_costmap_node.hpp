/**
 * @file local_costmap_node.hpp
 * @brief 激光观测到 odom 系滚动局部安全图的适配。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
        /**
         * @brief 建立 odom 系滚动局部图、传感器订阅和发布定时器。
         * @throws std::invalid_argument 图尺寸、膨胀参数、帧名或量程限制不合法。
         */
        LocalCostmapNode();

    private:
        /**
         * @brief 按扫描时刻 TF 清除射线并统一标记命中障碍。
         *
         * 正无穷量程按无返回处理，只清除可见空间而不标记障碍。
         * 一帧先完成全部清除再标记命中，避免束间相互擦除；传感器和机器人使用同一时刻 TF。
         *
         * @param scan 激光扫描；非法时间、元数据、TF 或无可用射线使局部图失效。
         */
        void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
        /**
         * @brief 过期观测后发布局部图与独立的感知有效性标志。
         *
         * 扫描超时先重置为未知；有效时生成膨胀副本。消息发布时间不是观测时间，
         * 消费者必须同时检查 local_costmap_valid 的新鲜度。
         */
        void publishMap();
        /**
         * @brief 使全部观测失效，清除扫描时间并把局部图恢复为未知。
         *
         * @param reason 失效原因，用于有效到无效转换时的警告日志。
         */
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
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
        std::chrono::steady_clock::time_point cloud_received_{};
        bool require_cloud_{false};
        bool valid_ = false;
    };
}

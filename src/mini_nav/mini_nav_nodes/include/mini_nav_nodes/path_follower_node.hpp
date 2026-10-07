/**
 * @file path_follower_node.hpp
 * @brief FollowPath 执行、输入新鲜度检查和候选速度输出。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "mini_nav_core/navigator/path_tracker.hpp"

namespace mini_nav_nodes
{
    /** 持续检查定位、路径与两张安全图，并向 Waffle 发布低速或零速度命令。 */
    class PathFollowerNode : public rclcpp::Node
    {
    public:
        /**
         * @brief 建立路径跟踪 Action、双安全图输入与周期控制。
         *
         * action_mode 禁用话题路径入口；命令发布话题可配置，主入口使用 cmd_vel_raw。
         * 对传感器/TF 丢失的反应时间取配置期限的最大值，用于制动距离预测。
         * @throws std::invalid_argument 帧名、话题、速度或数值限制非法。
         */
        PathFollowerNode();
        /**
         * @brief 撤销 context 预关闭回调，防止已销毁对象被回调访问。
         */
        ~PathFollowerNode() override;

    private:
        using SteadyTime = std::chrono::steady_clock::time_point;

        /**
         * @brief 校验 map 系路径与末点四元数，转换后提交核心跟踪器。
         *
         * 非法帧或数据导致清除路径与零速；Action 路径持续由目标身份管理，
         * 话题模式另外检查路径时间戳和接收时间。
         *
         * @param message 路径消息；空路径清除跟踪状态并停车。
         */
        void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr message);
        /**
         * @brief 转换并保存 map 系膨胀安全图及双时钟新鲜度记录。
         *
         * @param message 全局规划图 OccupancyGrid；转换失败清空缓存并停车。
         */
        void staticMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
        /**
         * @brief 转换并保存 odom 系局部安全图及双时钟新鲜度记录。
         *
         * @param message 滚动局部图 OccupancyGrid；转换失败清空缓存并停车。
         */
        void localMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
        /**
         * @brief 记录局部感知有效性心跳；失效时暂停进展计时并停车。
         *
         * @param message 局部图节点发布的有效性标志。
         */
        void localValidCallback(const std_msgs::msg::Bool::ConstSharedPtr message);
        /**
         * @brief 逐项检查时钟、目标、定位、双图和 TF，再执行一次核心跟踪。
         *
         * 任一前提失败都发送零速；到达或进展超时结束 FollowPath。
         * TF 时间检查允许 AMCL transform_tolerance 引入的有限未来时间戳。
         */
        void controlTick();
        /**
         * @brief 发布带底盘帧与当前时间戳的平面速度及控制状态。
         *
         * 本函数不再限幅；调用者须先完成安全检查。
         *
         * @param linear 前向速度，m/s。
         * @param angular 偏航角速度，rad/s。
         * @param status 控制状态码；只有状态变化时记录日志。
         */
        void publishCommand(double linear, double angular, const std::string & status);
        /**
         * @brief 发布零线速度、零角速度并说明停车状态。
         *
         * @param status 停车原因状态码。
         */
        void publishStop(const std::string & status);
        /**
         * @brief 同时按 ROS 消息时间与稳态接收时间判断输入新鲜度。
         *
         * 两种检查共同防止迟到消息或仿真时钟停滞掩盖输入断流。
         *
         * @param stamp ROS 消息时间戳。
         * @param received 稳态时钟接收时刻。
         * @param max_age 最大年龄，秒。
         * @param current 当前 ROS 时间，与 stamp 同源。
         * @param steady_now 当前稳态时间。
         * @return 消息年龄位于 [-0.1, max_age] 且接收年龄位于 [0, max_age] 时为 true。
         */
        bool fresh(const rclcpp::Time & stamp, const SteadyTime & received,
                   double max_age, const rclcpp::Time & current,
                   const SteadyTime & steady_now) const;

        std::unique_ptr<mini_nav_core::PathTracker> tracker_;
        std::unique_ptr<mini_nav_core::Costmap2D> static_map_;
        std::unique_ptr<mini_nav_core::Costmap2D> local_map_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_subscription_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_subscription_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_valid_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<rclcpp::PreShutdownCallbackHandle> shutdown_callback_handle_;

        using FollowPath = nav2_msgs::action::FollowPath;
        rclcpp_action::Server<FollowPath>::SharedPtr follow_server_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> follow_goal_;
        geometry_msgs::msg::PoseStamped follow_endpoint_;
        bool action_mode_{false};
        bool require_quality_{false};
        bool quality_valid_{false};
        SteadyTime quality_received_{};
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quality_subscription_;
        /**
         * @brief 先停车，再按错误码或取消状态结束当前 FollowPath 并清除路径。
         *
         * @param code FollowPath 结果错误码；NONE 表示成功。
         * @param reason 结果消息和停车状态。
         */
        void finishFollow(uint16_t code, const std::string & reason);
        std::string map_frame_id_;
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string last_status_;
        double max_path_age_{3.0};
        double max_static_map_age_{3.0};
        double max_local_map_age_{0.8};
        double max_tf_age_{1.0};
        double max_tf_future_{0.7};
        double clock_stall_timeout_{1.0};
        bool local_valid_{false};
        std::optional<rclcpp::Time> path_stamp_;
        std::optional<rclcpp::Time> static_map_stamp_;
        std::optional<rclcpp::Time> local_map_stamp_;
        std::optional<rclcpp::Time> last_clock_time_;
        std::optional<SteadyTime> path_received_;
        std::optional<SteadyTime> static_map_received_;
        std::optional<SteadyTime> local_map_received_;
        std::optional<SteadyTime> local_valid_received_;
        std::optional<SteadyTime> last_clock_advance_;
    };
}

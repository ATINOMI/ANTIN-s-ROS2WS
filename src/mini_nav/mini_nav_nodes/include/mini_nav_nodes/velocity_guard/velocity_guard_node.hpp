/**
 * @file velocity_guard_node.hpp
 * @brief 独立稳态时钟命令看门狗与底盘速度转发。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace mini_nav_nodes
{
// Independent wall-clock watchdog. This process must be the sole /cmd_vel publisher.
/**
 * @brief 仅在时钟、运动许可与命令均新鲜有效时转发底盘速度的独立节点。
 */
class VelocityGuardNode : public rclcpp::Node
{
public:
    /**
     * @brief 建立独立的稳态时钟速度看门狗，每 20 ms 检查运动许可和命令。
     *
     * 完整导航入口中本节点应是 /cmd_vel 的唯一发布者。
     * 退出 ROS context 前尝试发布零速；进程被强制终止仍需底盘驱动命令超时。
     * @throws std::invalid_argument command_timeout 不在有限范围 (0.1, 1.0] 秒内。
     */
    VelocityGuardNode();
    /**
     * @brief 移除 ROS context 的预关闭回调，避免对象销毁后继续访问其成员。
     */
    ~VelocityGuardNode() override;
private:
    using Clock = std::chrono::steady_clock;
    /**
     * @brief 校验时钟、任务许可及命令，转发合法速度或持续发布零速。
     *
     * 时钟回跳会清除缓存命令；时钟停滞 350 ms、心跳或命令超时均停车。
     * 仅接受 base_footprint 中的有限平面命令及有效时间戳；不再平滑速度，
     * 保证转发轨迹与跟踪器已经检查的候选运动一致。
     */
    void tick();
    geometry_msgs::msg::TwistStamped command_;
    Clock::time_point command_received_{}, lease_received_{}, clock_advanced_{};
    rclcpp::Time clock_time_{0, 0, RCL_ROS_TIME};
    bool active_{false};
    double timeout_{0.35};
    double max_linear_speed_{0.15}, max_angular_speed_{0.60};
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lease_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<rclcpp::PreShutdownCallbackHandle> shutdown_handle_;
};
}

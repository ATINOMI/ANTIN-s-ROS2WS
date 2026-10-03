/**
 * @file velocity_guard_node.cpp
 * @brief 独立稳态时钟命令看门狗与底盘速度转发。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_nodes/velocity_guard_node.hpp"
#include <cmath>
#include <stdexcept>
namespace mini_nav_nodes
{
/**
 * @brief 建立独立的稳态时钟速度看门狗，每 20 ms 检查运动许可和命令。
 *
 * 完整导航入口中本节点应是 /cmd_vel 的唯一发布者。
 * 退出 ROS context 前尝试发布零速；进程被强制终止仍需底盘驱动命令超时。
 * @throws std::invalid_argument command_timeout 不在有限范围 (0.1, 1.0] 秒内。
 */
VelocityGuardNode::VelocityGuardNode() : Node("velocity_guard")
{
    timeout_ = declare_parameter<double>("command_timeout", 0.35);
    if (!std::isfinite(timeout_) || timeout_ <= 0.1 || timeout_ > 1.0) throw std::invalid_argument("command_timeout");
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.15);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.60);
    if (!std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0 || max_linear_speed_ > 2.0) throw std::invalid_argument("max_linear_speed");
    if (!std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0 || max_angular_speed_ > 2.0) throw std::invalid_argument("max_angular_speed");
    clock_time_ = now(); clock_advanced_ = Clock::now();
    auto qos = rclcpp::QoS(1).reliable();
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>("/mini_nav/velocity_guard_status", qos);
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mini_nav/cmd_vel_raw", qos, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
            command_ = *msg; command_received_ = Clock::now();
        });
    lease_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/mini_nav/task_active", qos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            active_ = msg->data; lease_received_ = Clock::now();
            if (!active_) { command_received_ = Clock::time_point{}; tick(); }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
    shutdown_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
        get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
            geometry_msgs::msg::TwistStamped stop;
            stop.header.stamp = now(); stop.header.frame_id = "base_footprint";
            publisher_->publish(stop);
        }));
}
/**
 * @brief 移除 ROS context 的预关闭回调，避免对象销毁后继续访问其成员。
 */
VelocityGuardNode::~VelocityGuardNode()
{
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(*shutdown_handle_);
}
/**
 * @brief 校验时钟、任务许可及命令，转发合法速度或持续发布零速。
 *
 * 时钟回跳会清除缓存命令；时钟停滞 350 ms、心跳或命令超时均停车。
 * 仅接受 base_footprint 中的有限平面命令及有效时间戳；不再平滑速度，
 * 保证转发轨迹与跟踪器已经检查的候选运动一致。
 */
void VelocityGuardNode::tick()
{
    auto current = Clock::now(); auto stamp = now();
    const bool clock_reversed = stamp < clock_time_;
    if (stamp > clock_time_) clock_advanced_ = current;
    if (clock_reversed) { command_received_ = Clock::time_point{}; clock_advanced_ = current; }
    clock_time_ = stamp;
    std::string reason = "forwarding";
    if (clock_reversed || stamp.nanoseconds() == 0 || current - clock_advanced_ > std::chrono::milliseconds(350)) reason = "clock_stale";
    else if (!active_ || current - lease_received_ > std::chrono::duration<double>(timeout_)) reason = "task_inactive";
    else if (command_received_ == Clock::time_point{} || current - command_received_ > std::chrono::duration<double>(timeout_)) reason = "command_stale";
    else {
        const auto & v = command_.twist.linear; const auto & w = command_.twist.angular;
        const double age = (stamp - rclcpp::Time(command_.header.stamp, get_clock()->get_clock_type())).seconds();
        if (command_.header.frame_id != "base_footprint" || !std::isfinite(v.x) || !std::isfinite(w.z) ||
            std::abs(v.x) > max_linear_speed_ + 1e-9 || std::abs(w.z) > max_angular_speed_ || v.y != 0.0 || v.z != 0.0 ||
            w.x != 0.0 || w.y != 0.0 || !std::isfinite(age) || age < -0.1 || age > timeout_) reason = "invalid_command";
    }
    /* 默认构造即零速度，只有全部前提通过才复制缓存命令；失败分支不能沿用上一帧速度。
     * 转发不做再次平滑，否则底盘会执行未经跟踪器扫掠验证的另一条运动轨迹。 */
    geometry_msgs::msg::TwistStamped output;
    if (reason == "forwarding") output = command_;
    output.header.stamp = stamp; output.header.frame_id = "base_footprint";
    publisher_->publish(output);
    std_msgs::msg::String status; status.data = reason; status_publisher_->publish(status);
}
}

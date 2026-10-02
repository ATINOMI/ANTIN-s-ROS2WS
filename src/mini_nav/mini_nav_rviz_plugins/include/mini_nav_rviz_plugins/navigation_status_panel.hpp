/**
 * @file navigation_status_panel.hpp
 * @brief RViz 中文导航状态、任务反馈和取消交互面板。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

class QLabel;
class QPushButton;

namespace mini_nav_rviz_plugins {

// 常驻状态面板；ROS 回调存储快照，Qt 主线程更新界面并发起用户的取消请求。
/**
 * @brief ROS 回调缓存快照、Qt 主线程显示并发起导航取消的常驻面板。
 */
class NavigationStatusPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    /**
     * @brief 构建中文导航监控面板、取消按钮与 100 ms Qt 刷新定时器。
     *
     * ROS 回调只写共享快照，Qt 主线程刷新控件；取消按钮发送标准 CancelGoal 空 ID 请求，取消当前全部导航目标。
     *
     * @param parent Qt 父控件，负责子控件生命周期。
     */
    explicit NavigationStatusPanel(QWidget * parent = nullptr);
    /**
     * @brief 获取 RViz ROS 节点并订阅任务、跟踪、指令速度与局部感知。
     *
     * 状态话题保留最新值，速度与感知只使用实时消息；节点抽象不可用时直接返回。
     */
    void onInitialize() override;

private:
    struct State;
    /**
     * @brief 在 Qt 线程读取互斥保护的快照，更新状态、超时提示和取消按钮。
     *
     * 新鲜度使用稳态接收时刻；状态/任务超时为 2 秒，速度、感知和反馈为 1 秒。
     * 指令速度不表示实际底盘速度；任务结束状态可覆盖跟踪器清空路径后的 no_path。
     */
    void refresh();

    std::shared_ptr<State> state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_valid_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_subscription_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr feedback_subscription_;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_client_;
    QLabel * task_label_;
    QLabel * feedback_label_;
    QPushButton * cancel_button_;
    QLabel * status_label_;
    QLabel * reason_label_;
    QLabel * connection_label_;
    QLabel * velocity_label_;
    QLabel * local_valid_label_;
};

}  // namespace mini_nav_rviz_plugins

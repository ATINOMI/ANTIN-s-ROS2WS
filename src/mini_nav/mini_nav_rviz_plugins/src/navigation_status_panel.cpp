/**
 * @file navigation_status_panel.cpp
 * @brief RViz 中文导航状态、任务反馈和取消交互面板。
 * @author Antinomy
 * @date 2026-10-01
 */
#include "mini_nav_rviz_plugins/navigation_status_panel.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace mini_nav_rviz_plugins {
namespace {

using Clock = std::chrono::steady_clock;

/**
 * @brief 控制状态码对应的中文标题、原因和显示颜色。
 */
struct StatusDescription {
    const char * code;
    const char * title;
    const char * reason;
    const char * color;
};

constexpr StatusDescription kStatuses[] = {
    {"localization_invalid", "定位质量不足", "等待可信定位和新鲜激光数据。", "#a33323"},
    {"canceled", "已取消", "当前路径任务已取消并停车。", "#555555"},
    {"canceling", "正在取消", "正在停止当前路径任务。", "#975b00"},
    {"preempted", "目标已替换", "旧路径任务由新目标替换。", "#975b00"},
    {"tracking", "正在导航", "正在跟踪路径。", "#17663b"},
    {"avoiding_obstacle", "正在避障", "正在执行已通过碰撞检查的替代运动，并请求重规划。", "#975b00"},
    {"goal_reached", "已到达", "已到达规划终点并停车；不可达目标可能使用容差内的替代终点。", "#17663b"},
    {"no_path", "等待路径", "当前没有可跟踪的路径。", "#555555"},
    {"collision_risk", "碰撞风险停车", "预测运动与障碍或地图边界冲突。", "#a33323"},
    {"off_path", "偏离路径停车", "机器人距离路径超过允许范围。", "#a33323"},
    {"progress_timeout", "行驶无进展", "规定时间内未取得足够移动进展。", "#a33323"},
    {"local_sensor_invalid", "局部感知失效", "局部地图或激光感知无效，控制器已停车。", "#a33323"},
    {"clock_jump", "仿真时钟跳变", "时钟发生跳变，等待有效数据。", "#975b00"},
    {"clock_stale", "仿真时钟停滞", "仿真时钟停止更新，控制器已停车。", "#975b00"},
    {"path_stale", "路径数据超时", "路径未及时更新，控制器已停车。", "#a33323"},
    {"static_map_stale", "全局地图超时", "全局规划地图未及时更新。", "#a33323"},
    {"tf_stale", "定位变换超时", "机器人位姿变换已过期。", "#a33323"},
    {"tf_unavailable", "定位变换缺失", "无法获取机器人位姿变换。", "#a33323"},
    {"invalid_path", "路径无效", "收到的路径无法用于跟踪。", "#a33323"},
    {"invalid_static_map", "全局地图无效", "收到的全局规划地图无效。", "#a33323"},
    {"invalid_local_map", "局部地图无效", "收到的局部地图无效。", "#a33323"},
    {"invalid_pose", "机器人位姿无效", "当前位姿无法用于跟踪。", "#a33323"},
    {"control_error", "控制计算异常", "控制器计算发生异常并停车。", "#a33323"},
};

/**
 * @brief 建立可换行、可选取且使用纯文本渲染的标签。
 *
 * @param name Qt objectName，供查询和测试定位。
 * @param parent 父控件。
 * @return 由 Qt 父控件拥有的新标签。
 */
QLabel * makeLabel(const char * name, QWidget * parent) {
    auto * label = new QLabel(parent);
    label->setObjectName(name);
    label->setWordWrap(true);
    label->setTextFormat(Qt::PlainText);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return label;
}

}  // namespace

/**
 * @brief ROS 回调缓存快照、Qt 主线程显示并发起导航取消的常驻面板。
 */
struct NavigationStatusPanel::State {
    std::mutex mutex;
    std::string status;
    Clock::time_point status_time{};
    Clock::time_point velocity_time{};
    Clock::time_point local_valid_time{};
    double linear_x = 0.0;
    double angular_z = 0.0;
    bool local_valid = false;
    std::string task;
    Clock::time_point task_time{}, feedback_time{};
    float distance_remaining{0.0f};
    int replans{0};
    double elapsed{0.0};
};

/**
 * @brief 构建中文导航监控面板、取消按钮与 100 ms Qt 刷新定时器。
 *
 * ROS 回调只写共享快照，Qt 主线程刷新控件；取消按钮发送标准 CancelGoal 空 ID 请求，取消当前全部导航目标。
 *
 * @param parent Qt 父控件，负责子控件生命周期。
 */
NavigationStatusPanel::NavigationStatusPanel(QWidget * parent)
    : rviz_common::Panel(parent), state_(std::make_shared<State>()) {
    setMinimumWidth(250);
    auto * layout = new QVBoxLayout(this);
    auto * title = new QLabel("MINI NAV · 导航状态", this);
    title->setStyleSheet("font-weight: bold; font-size: 14px;");
    layout->addWidget(title);

    status_label_ = makeLabel("navigation_status", this);
    status_label_->setMinimumHeight(46);
    layout->addWidget(status_label_);
    reason_label_ = makeLabel("navigation_reason", this);
    layout->addWidget(reason_label_);

    auto * form = new QFormLayout();
    connection_label_ = makeLabel("controller_connection", this);
    velocity_label_ = makeLabel("command_velocity", this);
    local_valid_label_ = makeLabel("local_sensor_valid", this);
    form->addRow("控制器", connection_label_);
    form->addRow("指令速度", velocity_label_);
    form->addRow("局部感知", local_valid_label_);
    task_label_ = makeLabel("navigation_task", this);
    feedback_label_ = makeLabel("navigation_feedback", this);
    form->addRow("导航任务", task_label_);
    form->addRow("任务反馈", feedback_label_);
    layout->addLayout(form);
    cancel_button_ = new QPushButton("取消当前导航", this);
    cancel_button_->setObjectName("cancel_navigation");
    cancel_button_->setEnabled(false);
    layout->addWidget(cancel_button_);
    connect(cancel_button_, &QPushButton::clicked, this, [this]() {
        if (!cancel_client_ || !cancel_client_->service_is_ready()) return;
        // Empty goal ID is the standard CancelGoal request for all current goals.
        auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
        cancel_client_->async_send_request(request);
    });
    layout->addStretch();

    auto * hint = makeLabel("navigation_hint", this);
    hint->setText("通过 2D Goal Pose 设置目标。\n此面板显示任务反馈、控制状态和指令速度。");
    hint->setStyleSheet("color: #666666;");
    layout->addWidget(hint);

    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &NavigationStatusPanel::refresh);
    timer->start(100);
    refresh();
}

/**
 * @brief 获取 RViz ROS 节点并订阅任务、跟踪、指令速度与局部感知。
 *
 * 状态话题保留最新值，速度与感知只使用实时消息；节点抽象不可用时直接返回。
 */
void NavigationStatusPanel::onInitialize() {
    auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) {
        return;
    }
    auto node = abstraction->get_raw_node();
    cancel_client_ = node->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
    task_subscription_ = node->create_subscription<std_msgs::msg::String>(
        "/mini_nav/navigation_status", rclcpp::QoS(1).reliable().transient_local(),
        [state = state_](std_msgs::msg::String::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->task = msg->data; state->task_time = Clock::now();
        });
    feedback_subscription_ = node->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        "/navigate_to_pose/_action/feedback", rclcpp::QoS(10).reliable(),
        [state = state_](nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->distance_remaining = msg->feedback.distance_remaining;
            state->replans = msg->feedback.number_of_recoveries;
            state->elapsed = msg->feedback.navigation_time.sec + msg->feedback.navigation_time.nanosec * 1e-9;
            state->feedback_time = Clock::now();
        });
    // 与跟踪器的 QoS 一致；新打开 RViz 时也能收到保留的最后状态。
    status_subscription_ = node->create_subscription<std_msgs::msg::String>(
        "/mini_nav/controller_status", rclcpp::QoS(1).reliable().transient_local(),
        [state = state_](std_msgs::msg::String::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->status = message->data;
            state->status_time = Clock::now();
        });
    velocity_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::QoS(1).reliable().durability_volatile(),
        [state = state_](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->linear_x = message->twist.linear.x;
            state->angular_z = message->twist.angular.z;
            state->velocity_time = Clock::now();
        });
    local_valid_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
        "/mini_nav/local_costmap_valid", rclcpp::QoS(1).reliable().durability_volatile(),
        [state = state_](std_msgs::msg::Bool::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->local_valid = message->data;
            state->local_valid_time = Clock::now();
        });
}

/**
 * @brief 在 Qt 线程读取互斥保护的快照，更新状态、超时提示和取消按钮。
 *
 * 新鲜度使用稳态接收时刻；状态/任务超时为 2 秒，速度、感知和反馈为 1 秒。
 * 指令速度不表示实际底盘速度；任务结束状态可覆盖跟踪器清空路径后的 no_path。
 */
void NavigationStatusPanel::refresh() {
    // 使用接收时刻的稳态时钟，仿真暂停时仍能识别断流；不将旧状态视为在线。
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto now = Clock::now();
    const bool task_fresh = state_->task_time != Clock::time_point{} && now - state_->task_time <= std::chrono::seconds(2);
    const bool task_active = task_fresh && (state_->task == "tracking" || state_->task == "planning" ||
        state_->task == "waiting_for_localization_or_sensor" || state_->task == "replanning" || state_->task == "planning_failed" || state_->task == "controller_failed" ||
        state_->task == "tf_unavailable" || state_->task == "controller_rejected" || state_->task == "controller_unavailable");
    cancel_button_->setEnabled(task_active && cancel_client_ && cancel_client_->service_is_ready());
    const std::pair<const char *, const char *> tasks[] = {
        {"not_ready", "等待定位和局部感知就绪"}, {"navigation_disabled", "导航已暂停"},
        {"map_changed", "地图更新，任务终止"}, {"waiting_for_localization_or_sensor", "等待可信定位或感知恢复"},
        {"idle", "等待目标"}, {"planning", "正在规划"}, {"tracking", "正在执行"},
        {"replanning", "受阻重规划"}, {"planning_failed", "等待可行路径"},
        {"goal_reached", "已完成"}, {"canceled", "已取消"}, {"preempted", "已替换目标"},
        {"blocked_timeout", "受阻超时，已失败"}, {"no_progress", "无进展，已失败"},
        {"task_timeout", "任务超时，已失败"}, {"server_timeout", "服务无响应，已失败"},
        {"localization_reset", "定位已重置，任务终止"}, {"controller_unavailable", "控制器无响应"},
        {"tf_unavailable", "等待定位变换"}, {"controller_failed", "控制失败，重新规划"},
        {"controller_rejected", "控制器拒绝路径"}, {"controller_canceled", "控制任务已取消"}};
    QString task_text = task_fresh ? QString::fromStdString(state_->task) : "—（任务节点无新数据）";
    if (task_fresh) for (const auto & item : tasks) if (state_->task == item.first) task_text = item.second;
    task_label_->setText(task_text);
    if (task_active && now - state_->feedback_time <= std::chrono::seconds(1)) {
        feedback_label_->setText(QString("终点直线距离 %1 m\n已用 %2 s · 重规划 %3 次")
            .arg(state_->distance_remaining, 0, 'f', 2).arg(state_->elapsed, 0, 'f', 1).arg(state_->replans));
    } else feedback_label_->setText("—");
    QString title = "等待控制器";
    QString reason = "尚未收到导航状态。";
    QString color = "#555555";
    if (state_->status_time != Clock::time_point{}) {
        if (now - state_->status_time > std::chrono::seconds(2)) {
            title = "控制器失联";
            reason = "超过 2 秒未收到导航状态；最后状态：" + QString::fromStdString(state_->status);
            color = "#a33323";
            connection_label_->setText("状态超时");
        } else {
            connection_label_->setText("在线");
            title = "未知状态";
            reason = "状态码：" + QString::fromStdString(state_->status);
            color = "#975b00";
            for (const auto & entry : kStatuses) {
                if (state_->status == entry.code) {
                    title = entry.title;
                    reason = QString(entry.reason) + "\n" + entry.code;
                    color = entry.color;
                    break;
                }
            }
        }
    } else {
        connection_label_->setText("等待消息");
    }
    if (task_fresh && state_->status == "no_path") {
        if (state_->task == "goal_reached") { title = "已到达"; reason = "导航任务成功，底盘已停车。"; color = "#17663b"; }
        else if (state_->task == "canceled") { title = "已取消"; reason = "导航任务已取消，需重新设置目标。"; color = "#555555"; }
        else if (state_->task == "navigation_disabled") { title = "导航已暂停"; reason = "任务已终止，速度输出保持为零。"; color = "#555555"; }
        else if (state_->task == "blocked_timeout" || state_->task == "no_progress" || state_->task == "task_timeout" || state_->task == "server_timeout") {
            title = "导航失败"; reason = task_text; color = "#a33323";
        }
    }
    status_label_->setText(title);
    status_label_->setStyleSheet(
        "background-color: " + color + "; color: white; padding: 10px; font-size: 18px; font-weight: bold; border-radius: 5px;");
    reason_label_->setText(reason);

    const bool velocity_fresh = state_->velocity_time != Clock::time_point{} &&
        now - state_->velocity_time <= std::chrono::seconds(1);
    if (velocity_fresh && std::isfinite(state_->linear_x) && std::isfinite(state_->angular_z)) {
        velocity_label_->setText(QString("%1 m/s\n%2 rad/s")
            .arg(state_->linear_x, 0, 'f', 3).arg(state_->angular_z, 0, 'f', 3));
    } else {
        velocity_label_->setText(velocity_fresh ? "无效数值" : "—（无新数据）");
    }
    if (state_->local_valid_time == Clock::time_point{}) {
        local_valid_label_->setText("等待消息");
    } else if (now - state_->local_valid_time > std::chrono::seconds(1)) {
        local_valid_label_->setText("数据超时");
    } else {
        local_valid_label_->setText(state_->local_valid ? "有效" : "无效");
    }
}

}  // namespace mini_nav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(mini_nav_rviz_plugins::NavigationStatusPanel, rviz_common::Panel)

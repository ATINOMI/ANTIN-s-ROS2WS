#include "mini_nav_teleop/ps5_control_panel.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace mini_nav_teleop {
namespace {
using Clock = std::chrono::steady_clock;

QLabel * label(const char * name, QWidget * parent) {
    auto * result = new QLabel(parent);
    result->setObjectName(name);
    result->setWordWrap(true);
    result->setTextFormat(Qt::PlainText);
    return result;
}
}  // namespace

struct Ps5ControlPanel::State {
    std::mutex mutex;
    std::string status;
    Clock::time_point received{};
    QString reply;
    bool pending_limits{false};
    Clock::time_point limits_requested{};
    int64_t limits_request_id{0};
    uint64_t limits_generation{0};
    uint64_t enable_generation{0};
    bool pending_enable{false};
    Clock::time_point enable_requested{};
    int64_t enable_request_id{0};
};

Ps5ControlPanel::Ps5ControlPanel(QWidget * parent)
    : rviz_common::Panel(parent), state_(std::make_shared<State>()) {
    setMinimumWidth(270);
    auto * layout = new QVBoxLayout(this);
    auto * title = new QLabel("PS5 手柄控制", this);
    title->setStyleSheet("font-weight: bold; font-size: 14px;");
    layout->addWidget(title);
    status_label_ = label("ps5_status", this);
    layout->addWidget(status_label_);
    limits_label_ = label("ps5_current_limits", this);
    layout->addWidget(limits_label_);
    auto * form = new QFormLayout();
    linear_ = new QDoubleSpinBox(this);
    angular_ = new QDoubleSpinBox(this);
    linear_->setObjectName("ps5_linear_limit");
    angular_->setObjectName("ps5_angular_limit");
    for (auto * control : {linear_, angular_}) {
        control->setRange(0.0, 2.0);
        control->setDecimals(2);
        control->setSingleStep(0.1);
        control->setValue(2.0);
    }
    linear_->setSuffix(" m/s");
    angular_->setSuffix(" rad/s");
    form->addRow("线速度上限", linear_);
    form->addRow("角速度上限", angular_);
    layout->addLayout(form);
    apply_ = new QPushButton("应用速度上限", this);
    apply_->setObjectName("ps5_apply_limits");
    layout->addWidget(apply_);
    auto * buttons = new QHBoxLayout();
    enable_ = new QPushButton("启用控制", this);
    disable_ = new QPushButton("关闭控制 / 停车", this);
    enable_->setObjectName("ps5_enable");
    disable_->setObjectName("ps5_disable");
    buttons->addWidget(enable_);
    buttons->addWidget(disable_);
    layout->addLayout(buttons);
    reply_label_ = label("ps5_reply", this);
    layout->addWidget(reply_label_);
    auto * hint = label("ps5_hint", this);
    hint->setText("左摇杆前后，右摇杆转向。\nL1 与面板共用开关；蓝灯启用、红灯关闭。\n限速为 0 时，对应方向输出零速度。");
    layout->addWidget(hint);
    layout->addStretch();
    connect(enable_, &QPushButton::clicked, this, [this]() { setEnabled(true); });
    connect(disable_, &QPushButton::clicked, this, [this]() { setEnabled(false); });
    connect(apply_, &QPushButton::clicked, this, &Ps5ControlPanel::applyLimits);
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Ps5ControlPanel::refresh);
    timer->start(100);
    refresh();
}

void Ps5ControlPanel::onInitialize() {
    auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) return;
    auto node = abstraction->get_raw_node();
    enable_client_ = node->create_client<std_srvs::srv::SetBool>("/mini_nav/ps5/set_enabled");
    limits_client_ = node->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        "/ps5_teleop/set_parameters_atomically");
    status_subscription_ = node->create_subscription<std_msgs::msg::String>(
        "/mini_nav/ps5/status", rclcpp::QoS(1).reliable().transient_local(),
        [state = state_](const std_msgs::msg::String & message) {
            if (message.data.size() > 4096) return;
            std::lock_guard<std::mutex> lock(state->mutex);
            state->status = message.data;
            state->received = Clock::now();
        });
}

void Ps5ControlPanel::setEnabled(bool enabled) {
    if (!enable_client_ || !enable_client_->service_is_ready()) return;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->pending_enable) enable_client_->remove_pending_request(state_->enable_request_id);
        state_->pending_enable = true;
        state_->enable_requested = Clock::now();
        generation = ++state_->enable_generation;
        state_->reply = enabled ? "正在启用…" : "正在关闭…";
    }
    auto future = enable_client_->async_send_request(request,
        [state = state_, generation](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            std::lock_guard<std::mutex> lock(state->mutex);
            if (generation != state->enable_generation) return;
            state->pending_enable = false;
            state->reply = response->success ? "开关请求已执行" :
                "请求被拒绝：" + QString::fromStdString(response->message);
        });
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->enable_request_id = future.request_id;
}

void Ps5ControlPanel::applyLimits() {
    if (!limits_client_ || !limits_client_->service_is_ready()) return;
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    request->parameters = {rclcpp::Parameter("linear_limit", linear_->value()).to_parameter_msg(),
                           rclcpp::Parameter("angular_limit", angular_->value()).to_parameter_msg()};
    uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->pending_limits) return;
        state_->pending_limits = true;
        state_->limits_requested = Clock::now();
        generation = ++state_->limits_generation;
        state_->reply = "正在应用限速…";
    }
    auto future = limits_client_->async_send_request(request,
        [state = state_, generation](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
            auto response = future.get();
            std::lock_guard<std::mutex> lock(state->mutex);
            if (generation != state->limits_generation) return;
            state->pending_limits = false;
            state->reply = response->result.successful ? "限速已应用" :
                "限速被拒绝：" + QString::fromStdString(response->result.reason);
        });
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->limits_request_id = future.request_id;
}

void Ps5ControlPanel::refresh() {
    std::string status;
    bool fresh, pending;
    QString reply;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->pending_enable && Clock::now() - state_->enable_requested > std::chrono::seconds(3)) {
            if (enable_client_) enable_client_->remove_pending_request(state_->enable_request_id);
            state_->pending_enable = false;
            ++state_->enable_generation;
            state_->reply = "开关请求超时，请查看实际控制状态";
        }
        if (state_->pending_limits && Clock::now() - state_->limits_requested > std::chrono::seconds(3)) {
            if (limits_client_) limits_client_->remove_pending_request(state_->limits_request_id);
            state_->pending_limits = false;
            ++state_->limits_generation;
            state_->reply = "限速请求超时，请查看实际限速";
        }
        status = state_->status;
        fresh = !status.empty() && Clock::now() - state_->received < std::chrono::seconds(1);
        pending = state_->pending_limits;
        reply = state_->reply;
    }
    const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(status)).object();
    const auto reason = object.value("reason").toString();
    const double linear = object.value("linear_limit").toDouble(-1.0);
    const double angular = object.value("angular_limit").toDouble(-1.0);
    fresh = fresh && object.value("connected").isBool() && object.value("enabled").isBool() &&
        object.value("reason").isString() && std::isfinite(linear) && std::isfinite(angular) &&
        linear >= 0.0 && linear <= 2.0 && angular >= 0.0 && angular <= 2.0;
    const bool active = fresh && object.value("enabled").toBool();
    const bool connected = fresh && object.value("connected").toBool();
    const bool service = enable_client_ && enable_client_->service_is_ready();
    enable_->setEnabled(service && connected && reason == "ready" && !active);
    disable_->setEnabled(service);
    apply_->setEnabled(fresh && limits_client_ && limits_client_->service_is_ready() && !pending);
    if (!fresh) {
        status_label_->setText("等待手柄节点 / 状态超时");
        limits_label_->setText("实际限速：等待状态");
    } else {
        QString text;
        if (!connected) text = "USB 手柄未连接";
        else if (reason == "publisher_conflict") text = "速度发布者冲突，控制已关闭";
        else if (reason == "clock_fault") text = "仿真时钟异常，控制已关闭";
        else if (reason == "waiting_input") text = "等待新鲜 USB 输入，控制已关闭";
        else text = active ? "控制已启用 · 蓝灯" : "控制已关闭 · 红灯";
        status_label_->setText(text);
        status_label_->setStyleSheet(active ? "color: #1565c0;" : "color: #a33323;");
        limits_label_->setText(QString("实际限速：%1 m/s，%2 rad/s").arg(linear, 0, 'f', 2).arg(angular, 0, 'f', 2));
        if (!initialized_limits_) {
            linear_->setValue(linear);
            angular_->setValue(angular);
            initialized_limits_ = true;
        }
    }
    reply_label_->setText(reply);
}

}  // namespace mini_nav_teleop
PLUGINLIB_EXPORT_CLASS(mini_nav_teleop::Ps5ControlPanel, rviz_common::Panel)

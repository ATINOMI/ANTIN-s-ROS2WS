#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

class QLabel;
class QPushButton;
class QDoubleSpinBox;

namespace mini_nav_teleop {

// ROS 回调只更新快照，控件和用户请求由 Qt 主线程处理。
class Ps5ControlPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit Ps5ControlPanel(QWidget * parent = nullptr);
    void onInitialize() override;

private:
    struct State;
    void refresh();
    void setEnabled(bool enabled);
    void applyLimits();
    std::shared_ptr<State> state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr limits_client_;
    QLabel * status_label_;
    QLabel * limits_label_;
    QLabel * reply_label_;
    QDoubleSpinBox * linear_;
    QDoubleSpinBox * angular_;
    QPushButton * enable_;
    QPushButton * disable_;
    QPushButton * apply_;
    bool initialized_limits_{false};
};

}  // namespace mini_nav_teleop

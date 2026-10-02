/**
 * @file test_navigation_status_panel.cpp
 * @brief 验证 navigation_status_panel 模块行为及边界的测试。
 * @author Antinomy
 * @date 2026-10-01
 */
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <memory>

#include <QApplication>
#include <QDir>
#include <QDockWidget>
#include <QLabel>
#include <QPushButton>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <QTemporaryFile>
#include <QThread>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

/**
 * @brief 处理 Qt 事件并等待条件，避免阻塞界面回调。
 *
 * @param condition 可重复查询的条件函数。
 * @param timeout_ms 最大等待时间，毫秒。
 * @return 期限内条件满足为 true，否则 false。
 */
bool waitFor(const std::function<bool()> & condition, int timeout_ms = 5000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        QApplication::processEvents();
        if (condition()) {
            return true;
        }
        QThread::msleep(20);
    }
    return false;
}

/**
 * @brief 验证真实 RViz 插件加载、ROS 状态、超时、反馈与取消交互。
 */
TEST(NavigationStatusPanel, RealRvizPluginAndRosStatusLifecycle) {
    auto publisher_node = std::make_shared<rclcpp::Node>("panel_test_publisher");
    auto status_pub = publisher_node->create_publisher<std_msgs::msg::String>(
        "/mini_nav/controller_status", rclcpp::QoS(1).reliable().transient_local());
    auto velocity_pub = publisher_node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::QoS(1).reliable());
    auto local_pub = publisher_node->create_publisher<std_msgs::msg::Bool>(
        "/mini_nav/local_costmap_valid", rclcpp::QoS(1).reliable());

    auto task_pub = publisher_node->create_publisher<std_msgs::msg::String>(
        "/mini_nav/navigation_status", rclcpp::QoS(1).reliable().transient_local());
    auto feedback_pub = publisher_node->create_publisher<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        "/navigate_to_pose/_action/feedback", rclcpp::QoS(10).reliable());
    bool cancel_received = false;
    auto cancel_server = publisher_node->create_service<action_msgs::srv::CancelGoal>(
        "/navigate_to_pose/_action/cancel_goal",
        [&](std::shared_ptr<action_msgs::srv::CancelGoal::Request> request,
            std::shared_ptr<action_msgs::srv::CancelGoal::Response> response) {
            cancel_received = std::all_of(request->goal_info.goal_id.uuid.begin(),
                request->goal_info.goal_id.uuid.end(), [](auto byte) { return byte == 0; });
            response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_NONE;
        });

    QTemporaryFile config;
    ASSERT_TRUE(config.open());
    config.write("Panels:\n  - Class: mini_nav_rviz_plugins/NavigationStatus\n    Name: 导航状态\n"
        "Visualization Manager:\n  Class: \"\"\n  Global Options:\n    Fixed Frame: map\n");
    config.flush();
    auto rviz_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("panel_test_rviz");
    rviz_common::VisualizationFrame frame(rviz_node);
    frame.setApp(qobject_cast<QApplication *>(QApplication::instance()));
    frame.setSplashPath("");
    frame.setAttribute(Qt::WA_DontShowOnScreen);
    frame.initialize(rviz_node, config.fileName());
    frame.resize(1000, 700);
    frame.show();

    auto * status = frame.findChild<QLabel *>("navigation_status");
    auto * velocity = frame.findChild<QLabel *>("command_velocity");
    auto * local_valid = frame.findChild<QLabel *>("local_sensor_valid");
    auto * connection = frame.findChild<QLabel *>("controller_connection");
    ASSERT_NE(status, nullptr);  // RViz 自己通过插件清单加载，而非直接构造面板。
    ASSERT_NE(velocity, nullptr);
    ASSERT_NE(local_valid, nullptr);
    ASSERT_NE(connection, nullptr);
    EXPECT_EQ(status->text(), QString("等待控制器"));
    EXPECT_EQ(local_valid->text(), QString("等待消息"));
    auto * panel = status->parentWidget();
    auto * dock = qobject_cast<QDockWidget *>(panel->parentWidget());
    ASSERT_NE(dock, nullptr);
    frame.addDockWidget(Qt::RightDockWidgetArea, dock);
    dock->setFloating(false);

    auto screenshot = [&](const QString & name) {
        const char * directory = std::getenv("MINI_NAV_PANEL_SCREENSHOT_DIR");
        if (directory) {
            QDir().mkpath(directory);
            QApplication::processEvents();
            EXPECT_TRUE(panel->grab().save(QString(directory) + "/" + name + ".png"));
        }
    };
    screenshot("waiting");
    ASSERT_TRUE(waitFor([&] {
        return status_pub->get_subscription_count() > 0 &&
               velocity_pub->get_subscription_count() > 0 && local_pub->get_subscription_count() > 0;
    }));
    // 面板不发布速度：该域内只有探针发布测试速度、控制状态和感知。
    ASSERT_EQ(publisher_node->count_publishers("/cmd_vel"), 1u);
    ASSERT_EQ(publisher_node->count_subscribers("/cmd_vel"), 1u);
    EXPECT_EQ(publisher_node->count_publishers("/mini_nav/controller_status"), 1u);

    auto publishStatus = [&](const char * code, const char * title) {
        std_msgs::msg::String message;
        message.data = code;
        status_pub->publish(message);
        EXPECT_TRUE(waitFor([&] { return status->text() == QString(title); }));
        EXPECT_EQ(connection->text(), QString("在线"));
    };
    publishStatus("tracking", "正在导航");
    geometry_msgs::msg::TwistStamped command;
    command.twist.linear.x = 0.15;
    command.twist.angular.z = -0.55;
    velocity_pub->publish(command);
    std_msgs::msg::Bool valid;
    valid.data = true;
    local_pub->publish(valid);
    ASSERT_TRUE(waitFor([&] {
        return velocity->text() == QString("0.150 m/s\n-0.550 rad/s") &&
               local_valid->text() == QString("有效");
    }));
    auto * task_label = frame.findChild<QLabel *>("navigation_task");
    auto * feedback_label = frame.findChild<QLabel *>("navigation_feedback");
    auto * cancel_button = frame.findChild<QPushButton *>("cancel_navigation");
    ASSERT_NE(task_label, nullptr);
    ASSERT_NE(feedback_label, nullptr);
    ASSERT_NE(cancel_button, nullptr);
    std_msgs::msg::String task;
    task.data = "tracking";
    task_pub->publish(task);
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage feedback;
    feedback.feedback.distance_remaining = 0.5;
    feedback.feedback.number_of_recoveries = 2;
    feedback.feedback.navigation_time.sec = 12;
    feedback_pub->publish(feedback);
    ASSERT_TRUE(waitFor([&] { return cancel_button->isEnabled() && task_label->text() == QString("正在执行") &&
        feedback_label->text().contains("0.50 m") && feedback_label->text().contains("重规划 2 次"); }));
    screenshot("tracking");
    cancel_button->click();
    ASSERT_TRUE(waitFor([&] { rclcpp::spin_some(publisher_node); return cancel_received; }));
    task.data = "canceled";
    task_pub->publish(task);
    ASSERT_TRUE(waitFor([&] { return task_label->text() == QString("已取消") && !cancel_button->isEnabled(); }));
    screenshot("canceled");
    publishStatus("collision_risk", "碰撞风险停车");
    command.twist.linear.x = 0.0;
    command.twist.angular.z = 0.0;
    velocity_pub->publish(command);
    ASSERT_TRUE(waitFor([&] { return velocity->text() == QString("0.000 m/s\n0.000 rad/s"); }));
    screenshot("collision_risk");
    valid.data = false;
    local_pub->publish(valid);
    ASSERT_TRUE(waitFor([&] { return local_valid->text() == QString("无效"); }));
    publishStatus("local_sensor_invalid", "局部感知失效");
    publishStatus("goal_reached", "已到达");
    command.twist.linear.x = 0.0;
    command.twist.angular.z = 0.0;
    velocity_pub->publish(command);
    ASSERT_TRUE(waitFor([&] { return velocity->text() == QString("0.000 m/s\n0.000 rad/s"); }));
    screenshot("goal_reached");
    // 新面板无需等待状态变化，就能收到 transient-local 保留的到达状态。
    auto * late_dock = frame.addPanelByName("late subscriber", "mini_nav_rviz_plugins/NavigationStatus",
        Qt::LeftDockWidgetArea, false);
    ASSERT_NE(late_dock, nullptr);
    auto * late_status = late_dock->findChild<QLabel *>("navigation_status");
    ASSERT_NE(late_status, nullptr);
    ASSERT_TRUE(waitFor([&] { return late_status->text() == QString("已到达"); }));
    publishStatus("future_status", "未知状态");
    publishStatus("tracking", "正在导航");
    status_pub.reset();
    velocity_pub.reset();
    local_pub.reset();
    ASSERT_TRUE(waitFor([&] { return status->text() == QString("控制器失联"); }));
    EXPECT_EQ(connection->text(), QString("状态超时"));
    EXPECT_EQ(local_valid->text(), QString("数据超时"));
    EXPECT_EQ(velocity->text(), QString("—（无新数据）"));
    screenshot("disconnected");
}

}  // namespace

/**
 * @brief 初始化 ROS、Qt 和 gtest，运行隔离 RViz 集成用例。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return gtest 结果退出码；非隔离测试域时拒绝运行。
 */
int main(int argc, char ** argv) {
    // 本用例发布非零测试指令，只允许在专用域执行，防止直接运行时进入导航域。
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    if (!domain || std::string(domain) != "212") {
        std::cerr << "Run this GUI integration test with ROS_DOMAIN_ID=212.\n";
        return 2;
    }
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

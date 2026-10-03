#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <csignal>
#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QLabel>
#include <QProcess>
#include <QPushButton>
#include <QTemporaryFile>
#include <QThread>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace {
using Clock = std::chrono::steady_clock;

struct Processes {
    QProcess guard, driver;
    ~Processes() {
        for (auto * process : {&driver, &guard}) {
            if (process->state() != QProcess::NotRunning) {
                ::kill(process->processId(), SIGINT);
                if (!process->waitForFinished(5000)) {
                    process->kill();
                    process->waitForFinished(3000);
                }
            }
        }
    }
};

TEST(Ps5Panel, RealPluginControlsRealBridgeAndGuardWithSimulatedUsb) {
    Processes processes;
    const auto guard = ament_index_cpp::get_package_prefix("mini_nav_nodes") +
        "/lib/mini_nav_nodes/velocity_guard_node";
    processes.guard.start(QString::fromStdString(guard),
        {"--ros-args", "-p", "max_linear_speed:=2.0", "-p", "max_angular_speed:=2.0"});
    ASSERT_TRUE(processes.guard.waitForStarted());
    processes.driver.start("python3", {PS5_TEST_FIXTURE});
    ASSERT_TRUE(processes.driver.waitForStarted());
    auto probe = std::make_shared<rclcpp::Node>("ps5_panel_probe");
    double output_linear = 99.0, output_angular = 99.0;
    QJsonObject hardware;
    auto command = probe->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::TwistStamped & message) {
            output_linear = message.twist.linear.x;
            output_angular = message.twist.angular.z;
        });
    auto feedback = probe->create_subscription<std_msgs::msg::String>(
        "/ps5_test/feedback", rclcpp::QoS(1).reliable().transient_local(),
        [&](const std_msgs::msg::String & message) {
            hardware = QJsonDocument::fromJson(QByteArray::fromStdString(message.data)).object();
        });
    auto usb_client = probe->create_client<std_srvs::srv::SetBool>("/ps5_test/usb_connected");
    auto l1_client = probe->create_client<std_srvs::srv::SetBool>("/ps5_test/l1");
    auto parameters = probe->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        "/ps5_teleop/set_parameters_atomically");
    auto wait = [&](const std::function<bool()> & predicate, int milliseconds = 5000) {
        const auto end = Clock::now() + std::chrono::milliseconds(milliseconds);
        while (Clock::now() < end) {
            QApplication::processEvents();
            rclcpp::spin_some(probe);
            if (predicate()) return true;
            QThread::msleep(10);
        }
        std::cerr << processes.driver.readAllStandardError().toStdString();
        return false;
    };

    QTemporaryFile config;
    ASSERT_TRUE(config.open());
    config.write("Panels:\n- Class: mini_nav_teleop/Ps5Control\n  Name: PS5 手柄控制\n"
                 "Visualization Manager:\n  Class: ''\n  Global Options:\n    Fixed Frame: base_footprint\n");
    config.flush();
    auto abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("ps5_test_rviz");
    rviz_common::VisualizationFrame frame(abstraction);
    frame.setApp(qobject_cast<QApplication *>(QApplication::instance()));
    frame.setSplashPath("");
    frame.setAttribute(Qt::WA_DontShowOnScreen);
    frame.initialize(abstraction, config.fileName());
    frame.resize(1000, 700);
    frame.show();
    auto * status = frame.findChild<QLabel *>("ps5_status");
    auto * limits = frame.findChild<QLabel *>("ps5_current_limits");
    auto * reply = frame.findChild<QLabel *>("ps5_reply");
    auto * linear = frame.findChild<QDoubleSpinBox *>("ps5_linear_limit");
    auto * angular = frame.findChild<QDoubleSpinBox *>("ps5_angular_limit");
    auto * enable = frame.findChild<QPushButton *>("ps5_enable");
    auto * disable = frame.findChild<QPushButton *>("ps5_disable");
    auto * apply = frame.findChild<QPushButton *>("ps5_apply_limits");
    ASSERT_NE(status, nullptr);
    ASSERT_NE(limits, nullptr);
    ASSERT_NE(reply, nullptr);
    ASSERT_NE(linear, nullptr);
    ASSERT_NE(angular, nullptr);
    ASSERT_NE(enable, nullptr);
    ASSERT_NE(disable, nullptr);
    ASSERT_NE(apply, nullptr);
    ASSERT_TRUE(wait([&] { return enable->isEnabled() && apply->isEnabled(); }));
    ASSERT_EQ(probe->count_publishers("/cmd_vel"), 1u);
    ASSERT_TRUE(wait([&] { return output_linear == 0.0 && output_angular == 0.0; }));

    auto screenshot = [&](const char * name) {
        const char * directory = std::getenv("PS5_PANEL_SCREENSHOTS");
        if (directory) {
            QDir().mkpath(directory);
            EXPECT_TRUE(status->parentWidget()->grab().save(QString(directory) + "/" + name + ".png"));
        }
    };
    screenshot("ready");
    enable->click();
    ASSERT_TRUE(wait([&] { return status->text().contains("已启用") &&
        output_linear == 2.0 && output_angular == -2.0; }));
    ASSERT_TRUE(wait([&] { return hardware.value("led").toArray() == QJsonArray({0, 0, 255}); }));
    screenshot("enabled");
    linear->setValue(0.4);
    angular->setValue(0.7);
    apply->click();
    ASSERT_TRUE(wait([&] { return output_linear == 0.4 && output_angular == -0.7 &&
        limits->text().contains("0.40 m/s") && limits->text().contains("0.70 rad/s"); }));
    screenshot("limits_applied");

    auto invalid = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    invalid->parameters = {rclcpp::Parameter("linear_limit", 1.0).to_parameter_msg(),
                           rclcpp::Parameter("angular_limit", 2.1).to_parameter_msg()};
    auto rejected = parameters->async_send_request(invalid);
    ASSERT_TRUE(wait([&] { return rejected.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));
    EXPECT_FALSE(rejected.get()->result.successful);
    EXPECT_EQ(output_linear, 0.4);
    EXPECT_EQ(output_angular, -0.7);

    linear->setValue(0.0);
    apply->click();
    ASSERT_TRUE(wait([&] { return output_linear == 0.0 && output_angular == -0.7; }));
    disable->click();
    ASSERT_TRUE(wait([&] { return status->text().contains("已关闭") &&
        output_linear == 0.0 && output_angular == 0.0; }));
    ASSERT_TRUE(wait([&] { return hardware.value("led").toArray() == QJsonArray({255, 0, 0}); }));
    screenshot("disabled");

    auto setBool = [&](auto client, bool value) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = value;
        auto response = client->async_send_request(request);
        if (!wait([&] { return response.wait_for(std::chrono::seconds(0)) == std::future_status::ready; })) return false;
        return response.get()->success;
    };
    ASSERT_TRUE(setBool(l1_client, true));
    ASSERT_TRUE(wait([&] { return status->text().contains("已启用"); }));
    ASSERT_TRUE(setBool(l1_client, false));
    ASSERT_TRUE(wait([&] { return status->text().contains("已启用"); }));
    disable->click();
    ASSERT_TRUE(wait([&] { return status->text().contains("已关闭"); }));
    enable->click();
    ASSERT_TRUE(wait([&] { return status->text().contains("已启用"); }));
    ASSERT_TRUE(setBool(usb_client, false));
    ASSERT_TRUE(wait([&] { return status->text().contains("未连接") && !enable->isEnabled() &&
        output_linear == 0.0 && output_angular == 0.0; }));
    screenshot("disconnected");
    ASSERT_TRUE(setBool(usb_client, true));
    ASSERT_TRUE(wait([&] { return status->text().contains("已关闭") && enable->isEnabled(); }));
    ASSERT_EQ(probe->count_publishers("/cmd_vel"), 1u);
    processes.driver.terminate();
    ASSERT_TRUE(processes.driver.waitForFinished(5000));
    ASSERT_TRUE(wait([&] { return status->text().contains("超时") && !enable->isEnabled(); }));
    ASSERT_TRUE(wait([&] { return output_linear == 0.0 && output_angular == 0.0; }));
    screenshot("offline");
}
}  // namespace

int main(int argc, char ** argv) {
    if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "222") {
        std::cerr << "Run this simulated USB test with ROS_DOMAIN_ID=222\n";
        return 2;
    }
    rclcpp::init(argc, argv);
    QApplication application(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

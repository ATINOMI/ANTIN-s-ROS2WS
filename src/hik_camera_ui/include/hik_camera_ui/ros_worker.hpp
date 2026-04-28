#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QThread>
#include <QImage>
#include <atomic>

class RosWorker : public QThread
{
    Q_OBJECT

public:
    explicit RosWorker(QObject* parent = nullptr);
    ~RosWorker();
    void stop();
    void setParameter(const std::string& name, double value);

protected:
    void run() override;

signals:
    void imageReceived(QImage image);

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    std::atomic<bool> running_;
};

#pragma once

#include <string>

#include "apple_detect_task/detector.hpp"
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @brief 订阅图像消息并调用 Detector 完成检测。
 */
class ImageMonitor : public rclcpp::Node
{
public:
  /**
   * @brief 构造图像监测节点。
   *
   * 创建输入图像订阅器、处理结果发布器和 OpenCV 检测器。
   */
  ImageMonitor();

private:
  /**
   * @brief 处理收到的 ROS 图像消息。
   *
   * 将 ROS 图像转换为 cv::Mat，调用 Detector 完成检测，
   * 显示并发布带有检测结果的图像。
   *
   * @param msg 收到的 ROS 图像消息。
   */
  void image_call_back(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  /** @brief 输入图像话题名称。 */
  std::string image_topic_;

  /** @brief 检测结果图像话题名称。 */
  std::string result_topic_;

  /** @brief 输入图像订阅器。 */
  image_transport::Subscriber subscriber_;

  /** @brief 检测结果图像发布器。 */
  image_transport::Publisher result_publisher_;

  /** @brief OpenCV 苹果检测器。 */
  Detector detector_;
};

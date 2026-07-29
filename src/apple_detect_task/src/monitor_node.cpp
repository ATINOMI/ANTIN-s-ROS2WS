/* Includes ---------------------------------------------------------------------*/

#include "apple_detect_task/monitor_node.hpp"

#include <functional>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.hpp>

/* Constructor -----------------------------------------------------------------*/

/**
 * @brief 构造函数
 * 
 * 初始化图像监测节点。
 *
 * 创建原始图像订阅器、检测结果发布器和结果显示窗口。
 */
ImageMonitor::ImageMonitor()
: Node("image_monitor")
{

  /** @brief 输入图像话题名称。 */
  image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");

  /** @brief 检测结果图像话题名称。 */
  result_topic_ = declare_parameter<std::string>(
    "result_topic", "/apple_detector/result");

  /** @brief 检测结果图像发布器。 */
  result_publisher_ = image_transport::create_publisher(
    this,
    result_topic_);

  //声明一个窗口，WINDOW_NORMAL表示可缩放
  cv::namedWindow("detected_result", cv::WINDOW_NORMAL);

  /** @brief 输入图像订阅器。 */
  subscriber_ = image_transport::create_subscription(
    this,
    image_topic_,
    std::bind(
      &ImageMonitor::image_call_back,
      this,
      std::placeholders::_1),
    "raw",
    rclcpp::QoS(10).reliable().get_rmw_qos_profile());

  RCLCPP_INFO(
    get_logger(),
    "Subscribed to image topic: %s, publishing result on: %s",
    image_topic_.c_str(),
    result_topic_.c_str());
}

/**
 * @brief 接收并处理一帧图像。
 *
 * 图像先通过 cv_bridge 转换为 cv::Mat，然后交给 Detector 处理。
 * 处理后的图像会显示在窗口中，并发布到 result_topic_。
 */
void ImageMonitor::image_call_back(
  const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    auto cv_image = cv_bridge::toCvCopy(
      msg,
      sensor_msgs::image_encodings::BGR8);

    cv::Mat result;
    const DetectionResult detection =
      detector_.detect(cv_image->image, result);

    // 显示带有凸包边框的处理结果。
    cv::imshow("detected_result", result);
    cv::waitKey(1);

    // 将处理结果重新转换为 ROS 图像并发布。
    auto result_message = cv_bridge::CvImage(
      msg->header,
      sensor_msgs::image_encodings::BGR8,
      result).toImageMsg();

    result_publisher_.publish(result_message);

    if (detection.found) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Apple center: x=%.2f, y=%.2f, radius=%.2f",
        detection.center.x,
        detection.center.y,
        detection.radius);
    } else {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "No apple detected.");
    }
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(
      get_logger(),
      "cv_bridge conversion failed: %s",
      e.what());
  }
}

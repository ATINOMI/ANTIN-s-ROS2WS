/**
  ******************************************************************************
  * @file      image_publisher.hpp
  * @author    ANTINOMI
  * @date      2026-07-30
  * @brief     视频图像发布节点的类声明。
  *            本文件定义 ImagePublisher 节点，用于读取视频并通过
  *            image_transport 发布 ROS 2 图像消息。
  ******************************************************************************
  * @attention
  *
  * 本文件属于 apple_detect_task ROS 2 功能包。
  * 视频文件路径、图像话题和发布周期均由 ROS 2 参数控制。
  ******************************************************************************
  */

#pragma once

#include <string>

#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 读取视频并通过 image_transport 循环发布视频帧。
 */
class ImagePublisher : public rclcpp::Node
{
public:
  /**
   * @brief 构造视频发布节点。
   *
   * 读取 ROS 2 参数、打开视频文件、创建图像发布器和定时器。
   * 当 publish_period_ms 为 0 时，自动使用视频自身的帧率。
   */
  ImagePublisher();

private:
  /**
   * @brief 读取并发布下一帧视频图像。
   *
   * 当视频读取到末尾时，自动跳回第一帧继续发布。
   */
  void publishImage();

  /** @brief 视频文件路径。 */
  std::string video_path_;

  /** @brief 原始图像发布话题。 */
  std::string image_topic_;

  /** @brief 图像消息的坐标系名称。 */
  std::string frame_id_;

  /** @brief 发布周期，单位为毫秒；0 表示自动匹配视频帧率。 */
  int publish_period_ms_{0};

  /** @brief OpenCV 视频读取器。 */
  cv::VideoCapture video_;

  /** @brief 当前待发布的视频帧。 */
  cv::Mat frame_;

  /** @brief ROS 图像发布器。 */
  image_transport::Publisher publisher_;

  /** @brief 定时发布视频帧的 ROS 2 定时器。 */
  rclcpp::TimerBase::SharedPtr timer_;
};

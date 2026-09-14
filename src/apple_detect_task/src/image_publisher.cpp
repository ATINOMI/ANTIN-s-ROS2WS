/**
  ******************************************************************************
  * @file      image_publisher.cpp
  * @author    ANTINOMI
  * @date      2026-07-30
  * @brief     视频图像发布节点的具体实现。
  *            本文件使用 OpenCV 读取视频帧，使用 cv_bridge 转换消息，
  *            并按照视频帧率循环发布到指定的 ROS 2 图像话题。
  ******************************************************************************
  * @attention
  *
  * 本文件属于 apple_detect_task ROS 2 功能包。
  * 当视频读取到末尾时，节点会重新定位到第一帧并循环播放。
  ******************************************************************************
  */

/* Includes --------------------------------------------------------------------------------*/

#include "apple_detect_task/image_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

/* Constructor ----------------------------------------------------------------------------*/

/**
 * @brief 初始化视频发布节点。
 *
 * 节点会打开视频文件，并根据视频帧率或用户指定的周期发布图像。
 */
ImagePublisher::ImagePublisher()
: Node("image_publisher")
{

  /** @brief 视频文件路径。 */
  video_path_ = declare_parameter<std::string>("video_path", "");

  /** @brief 原始图像发布话题。 */
  image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");

  /** @brief 图像消息的坐标系名称。 */
  frame_id_ = declare_parameter<std::string>("frame_id", "camera");

  /** @brief 发布周期，单位为毫秒；0 表示自动匹配视频                  
   *              ├── stamp
                  │   ├── sec
                  │   └── nanosec
                  └── frame_id帧率。 */
  // 0 表示根据视频原始帧率自动设置发布周期。
  publish_period_ms_ = declare_parameter<int>("publish_period_ms", 0);

  //判断路径是否为空
  if (video_path_.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter video_path is empty.");
    throw std::invalid_argument("video_path must not be empty");
  }

  //判断发布周期是否小于0
  if (publish_period_ms_ < 0) {
    RCLCPP_FATAL(get_logger(), "Parameter publish_period_ms must not be negative.");
    throw std::invalid_argument("publish_period_ms must not be negative");
  }

  //打开视频，建立视频读取连接
  video_.open(video_path_);

  //检查是否打开成功
  if (!video_.isOpened()) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to open video: %s",
      video_path_.c_str());
    throw std::runtime_error("failed to open video: " + video_path_);
  }

  //如果发布周期为 0，表示启用自动帧率模式。
  if (publish_period_ms_ == 0) {

    //读取视频的帧率。例如视频是 24 FPS，这里得到 24.0。
    const double video_fps = video_.get(cv::CAP_PROP_FPS);

    /*
      使用三目运算符：

      - 如果视频帧率大于 0，使用视频帧率
      - 如果读取失败，默认使用 24 FPS

    */
    const double fps = video_fps > 0.0 ? video_fps : 24.0;

    /*根据帧率计算每帧之间的间隔：
      其中：

      - std::round()：四舍五入
      - static_cast<int>()：转换为整数
      - std::max(1, ...)：保证周期至少为 1 毫秒，避免得到 0

    */
    publish_period_ms_ = std::max(
      1,
      static_cast<int>(std::round(1000.0 / fps)));

    RCLCPP_INFO(
      get_logger(),
      "Auto playback rate: %.2f FPS, period=%d ms",
      fps,
      publish_period_ms_);
  }

  /*创建图像发布者，
    其中image_transport::create_publisher是image_transport封装的发布者创建函数
    类似于rclcpp::Node::create_publisher
  */
  publisher_ = image_transport::create_publisher(this, image_topic_);

  //创建定时器，用于定时发布图像消息
  timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&ImagePublisher::publishImage, this));

  RCLCPP_INFO(
    get_logger(),
    "Publishing video %s on %s every %d ms",
    video_path_.c_str(),
    image_topic_.c_str(),
    publish_period_ms_);
}

/* CallBack Function ----------------------------------------------------*/

/**
 * @brief 发布视频的下一帧。
 *
 * 视频播放结束后会重新定位到第一帧，从而实现循环播放。
 * 
 * 第一次读取失败，通常表示视频已经播放到结尾。
 * 接着程序把视频位置重新设置到第 0 帧
 * 
 * 然后第二个if语句:
 * 读取第一帧仍然失败,报错并返回
 *
 */
void ImagePublisher::publishImage()
{
  //if (!video_.read(frame_))本身具备读取的作用
  if (!video_.read(frame_)) {
    video_.set(cv::CAP_PROP_POS_FRAMES, 0);

    if (!video_.read(frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to read video frame.");
      return;
    }
  }

  /*
  * ROS 2 中的 Header 是消息里的“公共元数据”
  * 用于说明这条数据是什么时候产生的、来自哪个坐标系。
  * 
  * std_msgs/msg/Header
  *                ├── stamp
  *                │   ├── sec
  *                │   └── nanosec
  *                └── frame_id
  */
  std_msgs::msg::Header header;

  //表示消息的时间戳，即数据产生或采集的时间
  header.stamp = now();

  //表示这条数据是在哪个坐标系下表达的,这里为camera
  header.frame_id = frame_id_;

  //使用cv_bridge将图像封装为ros2消息包
  auto message = cv_bridge::CvImage(
    header,
    sensor_msgs::image_encodings::BGR8,
    frame_).toImageMsg();

  //发布消息  
  publisher_.publish(message);
}

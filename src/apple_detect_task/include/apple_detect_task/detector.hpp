/**
  ******************************************************************************
  * @file      detector.hpp
  * @author    ANTINOMI
  * @date      2026-07-30
  * @brief     苹果目标检测算法的数据结构和接口声明。
  *            本文件定义检测结果结构体以及 Detector 类，负责描述
  *            OpenCV 图像检测模块对外提供的接口。
  ******************************************************************************
  * @attention
  *
  * 本文件属于 apple_detect_task ROS 2 功能包。
  * Detector 只负责图像算法，不负责 ROS 2 节点和话题通信。
  ******************************************************************************
  */

#pragma once

#include <opencv2/opencv.hpp>

/* Detection Result --------------------------------------------------*/

/**
 * @brief 保存一次苹果检测的结果。
 */
struct DetectionResult
{
  /** @brief 是否检测到目标。 */
  bool found{false};

  /** @brief 目标最小外接圆圆心，单位为像素。 */
  cv::Point2f center;

  /** @brief 目标最小外接圆半径，单位为像素。 */
  float radius{0.0F};
};

/* Detector ----------------------------------------------------------*/

/**
 * @brief 使用 OpenCV 对输入图像进行苹果检测。
 *
 * Detector 只负责图像算法，不负责 ROS 节点、视频读取或话题通信。
 */
class Detector
{
public:
  /**
   * @brief 对输入图像进行检测，并在 output 中绘制凸包。
   *
   * @param input 输入的 BGR 图像。
   * @param output 输出图像，包含检测到的凸包。
   * @return 检测结果，包括目标是否找到、圆心和最小外接圆半径。
   */
  DetectionResult detect(const cv::Mat& input, cv::Mat& output) const;
};

/**
 * @file laser_scan_data.hpp
 * @brief 与 ROS 消息解耦的平面激光扫描数据。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <vector>

namespace mini_nav_core::localization
{

/**
 * @brief 激光量程数组与元数据；第 i 束角为 angle_min+i×angle_increment。
 */
struct LaserScanData
{
  /// 按扫描顺序保存的量程，单位米；有效范围与无返回处理由模型决定。
  std::vector<double> ranges;
  /// 第一束角度，弧度。
  double angle_min{0.0};
  /// 相邻束的角度增量，弧度。
  double angle_increment{0.0};
  /// 有效量程下限，米。
  double range_min{0.0};
  /// 有效量程上限，米。
  double range_max{0.0};
};

}  // namespace mini_nav_core::localization

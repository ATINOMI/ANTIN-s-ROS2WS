#pragma once

#include <cmath>

namespace mini_nav_core::nav_types
{
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

  /**
   * @brief 表示二维平面上的位姿，包括位置 (x, y) 和朝向 (yaw)。
   * 
   */
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief 将角度归一化到 [-π, π] 范围内。
 *
 * @param angle 输入角度（弧度）
 * @return double 归一化后的角度（弧度）
 */
inline double NormalizeAngle(double angle)
{
  // 取模运算将角度归一化到 [0, 2π] 范围内
  angle = std::fmod(angle + kPi, kTwoPi);
  // 如果结果为负数，则加上 2π，使其落在 [0, 2π] 范围内
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

/**
 * @brief 计算两个角度之间的有向距离。
 *
 * @param from 起始角度（弧度）
 * @param to 目标角度（弧度）
 * @return double 有向距离（弧度）
 */
inline double AngularDistance(double from, double to)
{
  // 计算两个角度之间的差值，并将其归一化到 [-π, π] 范围内
  return NormalizeAngle(from - to);
}

}

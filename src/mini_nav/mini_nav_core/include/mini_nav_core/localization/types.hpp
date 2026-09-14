/**
 * @file types.hpp
 * @brief 定义了用于粒子滤波器定位的基本类型，包括 Pose2D、Covariance3、Particle 和 PoseEstimate。
 *
 * 这些类型用于表示机器人在二维平面上的位姿、协方差矩阵、粒子以及位姿估计结果。
 */
#pragma once

/* Includes ----------------------------------------------------------------*/
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "mini_nav_core/localization/localization_constants.hpp"

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core::localization
{

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
 * @brief 表示一个粒子，包括位姿和权重。
 *
 * 粒子用于粒子滤波器中，表示机器人可能的状态。
 */
struct Particle
{
  Pose2D pose;
  double weight{0.0};
};


/* Class ------------------------------------------------------------------*/

/**
 * @brief 表示二维平面上的协方差矩阵，大小为 3x3。
 *
 * 该类用于表示位姿的不确定性，包括位置 (x, y) 和朝向 (yaw) 的协方差。
 */
class Covariance3
{
public:
/* Functions --------------------------------------------------------------*/

  // 默认构造函数
  Covariance3() = default;

  /**
   * @brief 创建一个对角协方差矩阵。
   * @param x_variance x 位置方差，单位为 m²
   * @param y_variance y 位置方差，单位为 m²
   * @param yaw_variance yaw 方差，单位为 rad²
   * @return 对角协方差矩阵
   */
  static Covariance3 DiagonalVariances(
    double x_variance, double y_variance, double yaw_variance)
  {
    Covariance3 covariance;
    covariance.values_[0] = x_variance;
    covariance.values_[4] = y_variance;
    covariance.values_[8] = yaw_variance;
    return covariance;
  }

  // Compatibility name retained for existing callers.
  static Covariance3 Diagonal(
    double x_variance, double y_variance, double yaw_variance)
  {
    return DiagonalVariances(x_variance, y_variance, yaw_variance);
  }

  /**
   * @brief 返回行列索引对应的协方差值元素本身，用于修改.
   * 
   * @param row       行索引
   * @param column    列索引
   * @return double& 
   */
  double & At(std::size_t row, std::size_t column)
  {
    if (row >= 3U || column >= 3U) {
      throw std::out_of_range("Covariance index is out of bounds");
    }
    return values_[row * 3U + column];
  }

  /**
   * @brief 返回行列索引对应的协方差值元素的常量引用，用于只读访问。
   * 
   * @param row       行索引
   * @param column    列索引
   * @return double 
   */
  double At(std::size_t row, std::size_t column) const
  {
    if (row >= 3U || column >= 3U) {
      throw std::out_of_range("Covariance index is out of bounds");
    }
    return values_[row * 3U + column];
  }

  /// Return the row-major 3x3 covariance storage for read-only inspection.
  const std::array<double, 9> & Values() const { return values_; }

  /*Private members --------------------------------------------------------*/
  
private:
  // 该类的核心，使用一个一维数组存储 3x3 协方差矩阵的值，按行优先顺序排列。
  std::array<double, 9> values_{};
};

// PoseCovariance is the semantic name; Covariance3 remains source-compatible.
using PoseCovariance = Covariance3;

/**
 * @brief 表示一个位姿估计结果.
 *
 * Pose2D is the estimated base pose in the map frame; covariance diagonal
 * entries are variances in m², m², and rad².
 */
struct PoseEstimate
{
  bool valid{false};
  Pose2D pose;
  Covariance3 covariance;
  // Sum of particle weights used by this estimate, before normalization.
  double weight{0.0};
};

/*inline functions----------------------------------------------------------*/

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

}  // namespace mini_nav_core::localization

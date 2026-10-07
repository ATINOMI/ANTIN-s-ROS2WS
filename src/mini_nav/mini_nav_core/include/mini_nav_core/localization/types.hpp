/**
 * @file types.hpp
 * @brief 定义了用于粒子滤波器定位的基本类型，包括 Pose2D、Covariance3、Particle 和 PoseEstimate。
 *
 * 这些类型用于表示机器人在二维平面上的位姿、协方差矩阵、粒子以及位姿估计结果。
  * @author Antinomy
 * @date 2026-10-01
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
  /**
   * @brief 构造全部元素为零的 3×3 协方差；状态顺序为 x、y、yaw。
   */
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
  /**
   * @brief 按位置和角度方差创建对角矩阵，兼容 DiagonalVariances。
   *
   * 此处不校验方差范围；滤波初始化时检查对称半正定性。
   *
   * @param x_variance x 方差，m²。
   * @param y_variance y 方差，m²。
   * @param yaw_variance 偏航方差，rad²。
   * @return 其余元素为零的对角协方差。
   */
  static Covariance3 Diagonal(
    double x_variance, double y_variance, double yaw_variance)
  {
    return DiagonalVariances(x_variance, y_variance, yaw_variance);
  }

  /**
   * @brief 访问并允许修改协方差元素。
   *
   * @param row 行号，0..2。
   * @param column 列号，0..2。
   * @return 内部元素的可修改引用。
   * @throws std::out_of_range 行或列越界。
   */
  double & At(std::size_t row, std::size_t column)
  {
    if (row >= 3U || column >= 3U) {
      throw std::out_of_range("Covariance index is out of bounds");
    }
    return values_[row * 3U + column];
  }

  /**
   * @brief 读取协方差元素的数值。
   *
   * @param row 行号，0..2。
   * @param column 列号，0..2。
   * @return 元素值的副本。
   * @throws std::out_of_range 行或列越界。
   */
  double At(std::size_t row, std::size_t column) const
  {
    if (row >= 3U || column >= 3U) {
      throw std::out_of_range("Covariance index is out of bounds");
    }
    return values_[row * 3U + column];
  }

  /// Return the row-major 3x3 covariance storage for read-only inspection.
  /**
   * @brief 只读访问按行存储的九个协方差元素。
   * @return 内部数组常引用；索引为 row × 3 + column。
   */
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
  // Dominant connected hypothesis weight divided by all positive particle weights.
  double hypothesis_mass{0.0};
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

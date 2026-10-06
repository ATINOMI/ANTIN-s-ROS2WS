/**
 * @file differential_motion_model.hpp
 * @brief Differential motion model for particle filter localization.
 *
 * This file defines the DifferentialMotionModel class, which implements the
 * motion model for a differential drive robot. It updates the particles based
 * on odometry readings and incorporates noise into the motion model.
 *
 * @author Antinomy
 * @date 2026-08-31
 */

#pragma once
/*Includes ------------------------------------------------------*/

#include <cstdint>
#include <random>

#include "mini_nav_core/localization/amcl/localization_constants.hpp"
#include "mini_nav_core/localization/amcl/motion_model.hpp"

/* Namespace ---------------------------------------------------------------*/

/** @brief Namespace for the localization module. */
namespace mini_nav_core::localization
{

/* Class Definition --------------------------------------------------------*/  

/**
 * @class DifferentialMotionModel
 * @brief 给定一个差分驱动机器人运动模型的粒子滤波器。
 *
 * 该类实现了 MotionModel 接口，使用差分驱动运动模型来更新粒子的位置和方向。
 * 它根据前后里程计位姿的变化来计算每个粒子的新位姿，并引入噪声以模拟实际运动的不确定性。
 * 
 */
class DifferentialMotionModel final : public MotionModel
{
public:
  /**
   * @brief 构造函数，初始化差分运动模型的噪声参数。
   *
   * @param alpha1 旋转增量对旋转噪声方差的系数。
   * @param alpha2 平移增量对旋转噪声方差的系数。
   * @param alpha3 平移增量对平移噪声方差的系数。
   * @param alpha4 旋转增量对平移噪声方差的系数。
   * @param alpha5 全向模型兼容保留；差速运动计算不使用，仍校验有限非负。
   * @param seed 随机数生成器的种子，用于可重复性
   */
  explicit DifferentialMotionModel(
    double alpha1 = kDefaultMotionNoiseCoefficient,
    double alpha2 = kDefaultMotionNoiseCoefficient,
    double alpha3 = kDefaultMotionNoiseCoefficient,
    double alpha4 = kDefaultMotionNoiseCoefficient,
    double alpha5 = kDefaultMotionNoiseCoefficient,
    std::uint64_t seed = kDefaultRandomSeed);

  /**
   * @brief 更新粒子位置和方向
   * 
   * @param particles  引用的粒子向量，每个粒子包含位姿和权重
   * @param previous_odom_pose 前一个里程计位姿
   * @param current_odom_pose 当前里程计位姿
   */
  void UpdateParticles(
    std::vector<Particle> & particles,
    const Pose2D & previous_odom_pose,
    const Pose2D & current_odom_pose) override;

private:
  double alpha1_; // 旋转引起的旋转噪声方差系数。
  double alpha2_; // 平移引起的旋转噪声方差系数。
  double alpha3_; // 平移引起的平移噪声方差系数。
  double alpha4_; // 旋转引起的平移噪声方差系数。
  double alpha5_; // 全向模型兼容保留；当前差速模型运动学不使用该系数
  std::mt19937_64 generator_; // 随机数生成器，用于生成高斯噪声
};

}  // namespace mini_nav_core::localization

/**
 * @file differential_motion_model.cpp
 * @brief 实现 DifferentialMotionModel 类，用于差分驱动机器人运动模型的粒子滤波。
 *
 * 该文件实现了 DifferentialMotionModel 类，该类继承自 MotionModel，并提供了
 * 更新粒子位置和方向的方法。该模型使用差分驱动机器人的运动学方程，并考虑了
 * 线速度和角速度的噪声。
 *
 * @author Your Name
 * @date 2024-06-15
 */

/* Includes ----------------------------------------------------------------*/ 
#include "mini_nav_core/localization/differential_motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core::localization
{

namespace
{
/* Functions ----------------------------------------------------------------*/
/**
 * @brief 验证噪声参数是否为有限且非负数。
 *
 * @param value 噪声参数值
 * @param name 参数名称
 * @throws std::invalid_argument 如果参数不是有限的或为负数
 */
void ValidateAlpha(double value, const char * name)
{
  // 检查参数是否为有限数且非负数。
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

/**
 * @brief 从高斯分布中采样一个随机值。
 *
 * @param generator 随机数生成器
 * @param variance 方差
 * @return double 采样的随机值
 */
double SampleGaussian(std::mt19937_64 & generator, double variance)
{
  // 如果方差为零或负数，则返回零，表示没有噪声。
  if (variance <= 0.0) {
    return 0.0;
  }

  // 使用标准正态分布采样，并根据给定的方差调整。
  std::normal_distribution<double> distribution(0.0, std::sqrt(variance));
  // 返回采样的随机值。
  return distribution(generator);
}
}  // namespace

// 实现 DifferentialMotionModel 类的构造函数和 UpdateParticles 方法。
DifferentialMotionModel::DifferentialMotionModel(
  double alpha1, double alpha2, double alpha3, double alpha4, double alpha5,
  std::uint64_t seed)
: alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4), alpha5_(alpha5),
  generator_(seed)
{
  // 验证噪声参数是否为有限且非负数。
  ValidateAlpha(alpha1_, "alpha1");
  ValidateAlpha(alpha2_, "alpha2");
  ValidateAlpha(alpha3_, "alpha3");
  ValidateAlpha(alpha4_, "alpha4");
  ValidateAlpha(alpha5_, "alpha5");
}

/**
 * @brief 更新粒子的位置和方向
 *
 * @param particles 引用的粒子向量，每个粒子包含位姿和权重
 * @param previous_odom_pose 前一个里程计位姿
 * @param current_odom_pose  当前里程计位姿
 */
void DifferentialMotionModel::UpdateParticles(
  std::vector<Particle> & particles,
  const Pose2D & previous_odom_pose,
  const Pose2D & current_odom_pose)
{
  const double delta_x = current_odom_pose.x - previous_odom_pose.x;
  const double delta_y = current_odom_pose.y - previous_odom_pose.y;
  const double delta_trans = std::hypot(delta_x, delta_y);
  const double delta_rot1 = delta_trans < 0.01 ? 0.0 :
    AngularDistance(std::atan2(delta_y, delta_x), previous_odom_pose.yaw);
  const double delta_rot2 = AngularDistance(
    AngularDistance(current_odom_pose.yaw, previous_odom_pose.yaw), delta_rot1);

  const double rot1_noise = std::min(std::abs(AngularDistance(delta_rot1, 0.0)),
                                     std::abs(AngularDistance(delta_rot1, 3.14159265358979323846)));
  const double rot2_noise = std::min(std::abs(AngularDistance(delta_rot2, 0.0)),
                                     std::abs(AngularDistance(delta_rot2, 3.14159265358979323846)));

  const double rot1_variance = alpha1_ * rot1_noise * rot1_noise + alpha2_ * delta_trans * delta_trans;
  const double trans_variance = alpha3_ * delta_trans * delta_trans +
    alpha4_ * rot1_noise * rot1_noise + alpha4_ * rot2_noise * rot2_noise;
  const double rot2_variance = alpha1_ * rot2_noise * rot2_noise + alpha2_ * delta_trans * delta_trans;

  for (auto & particle : particles) {
    const double noisy_rot1 = AngularDistance(
      delta_rot1, SampleGaussian(generator_, rot1_variance));
    const double noisy_trans = delta_trans - SampleGaussian(generator_, trans_variance);
    const double noisy_rot2 = AngularDistance(
      delta_rot2, SampleGaussian(generator_, rot2_variance));

    particle.pose.x += noisy_trans * std::cos(particle.pose.yaw + noisy_rot1);
    particle.pose.y += noisy_trans * std::sin(particle.pose.yaw + noisy_rot1);
    particle.pose.yaw = NormalizeAngle(particle.pose.yaw + noisy_rot1 + noisy_rot2);
  }
}

}  // namespace mini_nav_core::localization

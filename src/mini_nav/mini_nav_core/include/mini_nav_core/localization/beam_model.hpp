/**
 * @file beam_model.hpp
 * @author Antinomy
 * @brief 这个文件声明了 BeamModel 类，它是激光测距传感器的概率模型，用于粒子滤波器中的定位。
 * @version 0.1
 * @date 2026-09-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

/* Includes ----------------------------------------------------------------*/

#include <cstddef> 

#include "mini_nav_core/localization/laser_model.hpp"

/* Namespace ---------------------------------------------------------------*/

namespace mini_nav_core::localization
{

/**
 * @class BeamModel
 * @brief 激光测距传感器的概率模型，用于粒子滤波器中的定位。
 *
 * 该类实现了 LaserModel 接口，使用 Beam Model 来计算每个粒子在给定激光扫描数据下的权重。
 * 它考虑了测距传感器的各种误差来源，包括命中、短距离、最大距离和随机噪声。
 */
class BeamModel final : public LaserModel
{
public:
/* Public Functions --------------------------------------------------------- */

  /**
   * @brief 设置命中、短测距、最大量程与随机项的混合激光模型。
   *
   * @param z_hit 命中项有限非负权重。
   * @param z_short 短测距项有限非负权重。
   * @param z_max 最大量程项有限非负权重。
   * @param z_rand 随机项有限非负权重；四项和必须为正。
   * @param sigma_hit 命中项标准差，有限正数，米。
   * @param lambda_short 短测距指数分布系数，有限正数，1/米。
   * @param max_beams 期望抽取束数，必须非零。
   * @throws std::invalid_argument 权重、尺度或束数非法。
   */
  BeamModel(
    double z_hit,             
    double z_short,
    double z_max,
    double z_rand,
    double sigma_hit,
    double lambda_short,
    std::size_t max_beams);

  /**
   * @brief 按地图射线预期量程计算混合似然并乘入粒子权重。
   *
   * 激光安装位姿先与每个粒子位姿复合；按束数目标降采样并跳过无效读数，
   * 使用对数域累积避免概率连续相乘下溢。归一化由滤波器完成。
   *
   * @param particles 原地更新的候选位姿和权重。
   * @param scan 量程为米、角度为弧度的观测。
   * @param map 用于预期量程射线投射的地图。
   * @param base_to_laser_pose 激光帧在底盘帧中的外参，米/弧度。
   */
  void ApplyMeasurementLikelihood(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose) const override;

/* Private members ------------------------------------------------------------*/

private:
  double z_hit_;           // 命中测距的权重
  double z_short_;         // 短距离测距的权重
  double z_max_;           // 最大距离测距的权重
  double z_rand_;          // 随机噪声的权重
  double sigma_hit_;       // 命中测距的标准差
  double lambda_short_;    // 短距离测距的指数衰减参数
  std::size_t max_beams_;  // 最大使用的激光束数量
};

}  // namespace mini_nav_core::localization

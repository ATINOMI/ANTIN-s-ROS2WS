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
   * @brief 构造函数，初始化 Beam Model 的参数。
   *
   * @param z_hit 命中测距的权重
   * @param z_short 短距离测距的权重
   * @param z_max 最大距离测距的权重
   * @param z_rand 随机噪声的权重
   * @param sigma_hit 命中测距的标准差
   * @param lambda_short 短距离测距的指数衰减参数
   * @param max_beams 最大使用的激光束数量
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
   * @brief 更新粒子权重
   *
   * @param particles 引用的粒子向量，每个粒子包含位姿和权重
   * @param scan 激光扫描数据
   * @param map 地图数据
   * @param laser_pose_in_base 激光传感器在机器人基座坐标系下的位姿
   */
  void UpdateWeights(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & laser_pose_in_base) const override;

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

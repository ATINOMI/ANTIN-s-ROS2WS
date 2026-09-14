/**
 * @file likelihood_field_model.hpp
 * @brief 声明 LikelihoodFieldModel，用于粒子滤波器的激光观测更新。
 *
 * Likelihood Field Model 不为每个粒子、每一束激光执行射线投射来预测量测距离。
 * 它将观测终点投影到地图中，并查询该位置到最近障碍物的预计算距离。
 *
 * 终点越靠近地图障碍物，说明该粒子位姿与本次激光观测越一致；反之，
 * 命中项的似然会按高斯形式衰减。随机项为异常量测保留非零概率，
 * 防止单束异常激光直接将粒子权重置零。
 *
 * @author Antinomy
 * @date 2026-09-13
 */
#pragma once

/* Includes ----------------------------------------------------------------*/

#include <cstddef>

#include "mini_nav_core/localization/laser_model.hpp"

/* Namespace ---------------------------------------------------------------*/

namespace mini_nav_core::localization
{

/**
 * @class LikelihoodFieldModel
 * @brief 使用障碍物距离场评估激光观测与粒子位姿一致性的激光模型。
 *
 * 对每个被选中的有效量测，本类计算激光终点到最近障碍物的距离 d，
 * 并使用以下混合似然：
 *
 *   p(z | x, m) = z_hit * exp(-d² / (2 * sigma_hit²)) + z_rand / range_max
 *
 * 各束激光的概率会乘入粒子已有权重；实现改在对数域中累加以避免连续小概率
 * 相乘导致下溢。该模型只使用命中项和随机项，不实现 Beam Model 中的
 * 短距离项或最大量程项。
 */
class LikelihoodFieldModel final : public LaserModel
{
public:
  /* Public Functions -------------------------------------------------------*/

  /**
   * @brief 构造 Likelihood Field 激光观测模型。
   *
   * @param z_hit 障碍物命中项的非负混合权重。
   * @param z_rand 均匀随机量测项的非负混合权重。
   * @param sigma_hit 命中项的距离尺度，单位为 m，必须大于 0。
   * @param max_beams 单次更新最多希望使用的激光束数，必须大于 0。
   *
   * @throws std::invalid_argument 当混合权重不是有限非负数、两者之和为 0、
   *         sigma_hit 无效或 max_beams 为 0 时抛出。
   *
   * @note 当前实现要求权重总和为正，但不会将 z_hit 与 z_rand 自动归一化；
   *       它们的绝对比例会直接影响量测似然的尺度。
   */
  LikelihoodFieldModel(
    double z_hit,
    double z_rand,
    double sigma_hit,
    std::size_t max_beams);

  /**
   * @brief 将一帧激光观测的似然乘入每个粒子的当前权重。
   *
   * @param particles 待更新的粒子集合。本函数只在至少使用一束有效激光时
   *        修改对应粒子的 weight。
   * @param scan 当前帧激光数据；range、角度和量程均采用 m / rad 单位。
   * @param map 含有最近障碍物距离场的定位地图。
   * @param base_to_laser_pose 激光坐标系相对机器人基座坐标系的位姿 T_base_laser，
   *        平移单位为 m，偏航角单位为 rad。
   *
   * @note 非有限量测、低于 range_min 的量测，以及大于等于 range_max 的量测会被跳过。
   *       因此本模型不会把最大量程读数作为单独的概率项处理。
   */
  void ApplyMeasurementLikelihood(
    std::vector<Particle> & particles,
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose) const override;

  /* Private Members --------------------------------------------------------*/

private:
  // 命中项与随机项的混合权重；构造阶段保证两者有限、非负且总和大于 0。
  double z_hit_;
  double z_rand_;
  // 命中项高斯衰减的距离尺度，单位为 m。
  double sigma_hit_;
  // 为控制计算量而从一帧扫描中均匀抽取的目标激光束数。
  std::size_t max_beams_;
};

}  // namespace mini_nav_core::localization

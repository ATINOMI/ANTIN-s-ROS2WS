/**
 * @file likelihood_field_model.cpp
 * @brief 实现基于障碍物距离场的激光观测似然模型。
 *
 * 对每个粒子，本实现先由粒子的基座位姿和激光外参得到激光在地图中的位姿，
 * 再将有效量测的终点投影到地图中。终点距最近障碍物越近，命中项给出的
 * 似然越高。
 *
 * 多束激光的似然在对数域中累加：
 *
 *   log(w_new) = log(w_old) + Σ log(p_beam)
 *
 * 这样与直接连续相乘相比，不易因大量小概率而发生浮点数下溢。
 *
 * @author Antinomy
 * @date 2026-09-13
 */

/* Includes ----------------------------------------------------------------*/

#include "mini_nav_core/localization/amcl/likelihood_field_model.hpp"
#include "mini_nav_core/localization/amcl/pose_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

/* Namespace ---------------------------------------------------------------*/

namespace mini_nav_core::localization
{

namespace
{
/* Internal helpers --------------------------------------------------------*/
}  // namespace

/* Functions ---------------------------------------------------------------*/

/**
 * @brief 构造 Likelihood Field 激光观测模型并验证其参数。
 *
 * @param z_hit 障碍物命中项的非负混合权重。
 * @param z_rand 均匀随机量测项的非负混合权重。
 * @param sigma_hit 命中项的距离尺度，单位为 m，必须大于 0。
 * @param max_beams 单帧观测中目标使用的最大激光束数，必须大于 0。
 *
 * @throws std::invalid_argument 当参数无法形成有效的概率模型时抛出。
 */
LikelihoodFieldModel::LikelihoodFieldModel(
  double z_hit, double z_rand, double sigma_hit, std::size_t max_beams)
: z_hit_(z_hit), z_rand_(z_rand), sigma_hit_(sigma_hit), max_beams_(max_beams)
{
  /*
   * 混合权重无需在这里归一化，但必须是有限非负数且至少有一项有效。
   *
   * 若允许负权重或全零权重进入后续计算，单束量测概率可能为负或恒为 0，
   * 随后的 log(probability) 将失去概率意义。尽早在构造阶段失败，
   * 能把配置错误与运行时的粒子权重异常区分开。
   */
  if (!std::isfinite(z_hit_)  || 
      !std::isfinite(z_rand_) || 
      z_hit_ < 0.0  || 
      z_rand_ < 0.0 ||
      z_hit_ + z_rand_ <= 0.0) 
  {
    throw std::invalid_argument("Laser mixture weights must be finite and non-negative");
  }

  /*
   * sigma_hit 是距离高斯项的尺度，必须严格为正；max_beams 为 0 时则无法
   * 选择任何量测。两者都会使观测更新退化，因此与混合权重一同在此验证。
   */
  if (!std::isfinite(sigma_hit_) || sigma_hit_ <= 0.0 || max_beams_ == 0U) {
    throw std::invalid_argument("Invalid likelihood-field parameters");
  }
}

/**
 * @brief 依据最近障碍物距离场，将当前激光观测的似然乘入每个粒子权重。
 *
 * @param particles 待更新的粒子集合；每个粒子的 pose 是其假设的基座地图位姿，
 *        weight 是此前预测或观测阶段累积的权重。
 * @param scan 当前激光扫描数据，距离单位为 m，角度单位为 rad。
 * @param map 为激光终点提供最近障碍物距离的定位地图。
 * @param base_to_laser_pose 激光相对基座的位姿 T_base_laser，平移单位为 m，
 *        偏航角单位为 rad。
 *
 * @note 仅使用落在 [range_min, range_max) 内的有限量测。若整帧没有可用量测，
 *       本函数保持该粒子的原权重不变。
 */
void LikelihoodFieldModel::ApplyMeasurementLikelihood(
  std::vector<Particle> & particles,
  const LaserScanData & scan,
  const LocalizationMap & map,
  const Pose2D & base_to_laser_pose) const
{
  /*
   * 缺少粒子、量测或有效最大量程时，无法构造有意义的观测似然。
   * 直接返回会保留预测阶段的粒子权重，而不是把“没有信息”误当作一次
   * 低置信度观测更新。
   */
  if (particles.empty()     || 
      scan.ranges.empty()   || 
      scan.range_max <= 0.0 ||
      !std::isfinite(scan.range_max)) 
  {
    return;
  }

  /* Step 1：计算均匀抽样步长 ------------------------------------------*/

  /*
   * 在完整扫描中均匀抽取激光束，以 max_beams_ 控制单次观测更新的计算量。
   *
   *   step = max(1, (N - 1) / max(1, max_beams - 1))
   *
   * N 为扫描束数。使用 N - 1 和 max_beams - 1 能在可整除时同时覆盖首尾束；
   * 当请求束数不少于扫描束数时 step 为 1，因而不会丢弃任何量测。
   */
  const std::size_t step = std::max<std::size_t>(
    1U, (scan.ranges.size() - 1U) / std::max<std::size_t>(1U, max_beams_ - 1U));

  /* Step 2：预计算与粒子无关的概率项 ----------------------------------*/

  /*
   * 命中项使用 exp(-d² / (2 * sigma_hit²))，其分母和随机项不依赖粒子或束编号，
   * 因此在循环外计算。random_probability 为 [0, range_max) 上的均匀密度，
   * 为异常反射等情况提供最低概率质量。
   */
  const double hit_denominator    = 2.0 * sigma_hit_ * sigma_hit_;
  const double random_probability = z_rand_ / scan.range_max;

  /* Step 3：为每个粒子累加被选激光束的对数似然 ------------------------*/

  for (auto & particle : particles) 
  {
    /*
     * particle.pose 是该粒子假设的基座在地图中的位姿。与 T_base_laser 组合后，
     * laser_pose 才是此粒子假设下激光原点在地图中的位置与朝向：
     *
     *   map ── particle.pose ──► base ── base_to_laser_pose ──► laser
     *
     * 忽略这一步会把存在安装偏移的雷达误当作位于机器人中心，从而系统性扭曲
     * 所有激光终点的位置。
     */
    const Pose2D laser_pose = ComposePose2D(particle.pose, base_to_laser_pose);

    /*
     * 多束激光似然在普通概率域中需要连乘。改为 log(p) 求和可避免大量小于 1 的
     * 概率快速下溢；kMinimumPositiveDouble 只保护初始权重不传入 log(0)。
     */
    double log_weight = std::log(std::max(particle.weight, kMinimumPositiveDouble));
    bool used_beam = false;

    for (std::size_t index = 0; index < scan.ranges.size(); index += step) 
    {

      const double observed_range = scan.ranges[index];

      /*
       * 当前模型不为无返回和最大量程读数实现专门的概率项，因此只保留有限且落在
       * [range_min, range_max) 内的量测。把 range_max 也纳入命中项会将“未命中”
       * 误解释为一个靠近障碍物的终点，从而错误提高某些粒子的权重。
       */
      if (!std::isfinite(observed_range)  || 
          observed_range < scan.range_min ||
          observed_range >= scan.range_max) 
      {
        continue;
      }

      const double bearing = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const double hit_x = laser_pose.x + observed_range * std::cos(laser_pose.yaw + bearing);
      const double hit_y = laser_pose.y + observed_range * std::sin(laser_pose.yaw + bearing);

      /*
       * 将当前量测投影到该粒子假设下的地图终点，再查询终点到最近障碍物的距离 d。
       *
       *   laser origin ●──────── observed_range ────────► hit point
       *                                                    │
       *                                                    │ d
       *                                                    ▼
       *                                             nearest obstacle
       *
       * 距离查询失败时 distance 保持为地图的最大障碍物距离。这让地图外终点获得
       * 很低的命中似然，而无需把一次查询失败伪造为命中障碍物。
       */
      double distance = map.GetMaxObstacleDistance();
      map.TryGetObstacleDistanceAtWorld(hit_x, hit_y, distance);

      /*
       * 命中项只依赖终点与障碍物的距离，而不依赖地图射线的理论量程：
       *
       *   p_hit = z_hit * exp(-d² / (2 * sigma_hit²))
       *   p      = max(p_hit + z_rand / range_max, kMinimumProbability)
       *
       * sigma_hit_ 越小，对终点偏离障碍物越敏感；随机项及下限则防止单束异常量测
       * 产生 log(0)，使整个粒子的累计权重不可恢复地变为负无穷。
       */
      const double hit_probability = z_hit_ * std::exp(-(distance * distance) / hit_denominator);
      const double probability = std::max(hit_probability + random_probability, kMinimumProbability);
      log_weight += std::log(probability);
      used_beam = true;
    }

    if (used_beam) {
      /*
       * 回到普通概率域之前将指数限制在 double 可表示的安全范围内。
       * 这不是归一化：真正跨粒子的归一化由粒子滤波器后续阶段负责；这里仅避免
       * exp(log_weight) 发生上溢或产生不受控的下溢。
       */
      particle.weight = std::exp(std::max(kExpLowerLimit, std::min(kExpUpperLimit, log_weight)));
    }
  }
}

}  // namespace mini_nav_core::localization

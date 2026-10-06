/**
 * @file beam_model.cpp
 * @author Antinomy
 * @brief 这个文件实现了 BeamModel 类，它是激光测距传感器的概率模型，用于粒子滤波器中的定位。
 * @version 0.1
 * @date 2026-09-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/* Includes ----------------------------------------------------------------*/
#include "mini_nav_core/localization/amcl/beam_model.hpp"
#include "mini_nav_core/localization/amcl/pose_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

/* Namespace ----------------------------------------------------------------*/
namespace mini_nav_core::localization
{

//匿名命名空间，用于封装 BeamModel 类的内部辅助函数，不对外部文件暴露。
namespace
{
/* Functions ----------------------------------------------------------------*/

/**
 * @brief 计算高斯分布的概率密度函数值。
 * @param difference 观测值与期望值的差异
 * @param sigma 高斯分布的标准差
 * @return double 高斯分布的概率密度函数值
 */
double NormalPdf(double difference, double sigma)
{
  //计算公式：e^(-0.5 * (difference^2) / (sigma^2)) / (sqrt(2 * pi) * sigma)
  return std::exp(-0.5 * difference * difference / (sigma * sigma)) /
         (std::sqrt(2.0 * kPi) * sigma);
}
}  // namespace

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
BeamModel::BeamModel(
  double z_hit,
  double z_short, 
  double z_max, 
  double z_rand,
  double sigma_hit, 
  double lambda_short, 
  std::size_t max_beams)
        : z_hit_(z_hit), z_short_(z_short), z_max_(z_max), z_rand_(z_rand),
  sigma_hit_(sigma_hit), lambda_short_(lambda_short), max_beams_(max_beams)
{
  // Mixture weights must be valid before they are used in the likelihood sum.
  if (!std::isfinite(z_hit_) || z_hit_ < 0.0 ||
      !std::isfinite(z_short_) || z_short_ < 0.0 ||
      !std::isfinite(z_max_) || z_max_ < 0.0 ||
      !std::isfinite(z_rand_) || z_rand_ < 0.0 ||
      z_hit_ + z_short_ + z_max_ + z_rand_ <= 0.0 ||
      !std::isfinite(sigma_hit_) || sigma_hit_ <= 0.0 ||
      !std::isfinite(lambda_short_) || lambda_short_ <= 0.0 || max_beams_ == 0U) {
    throw std::invalid_argument("Beam model parameters must be finite and non-negative");
  }
}

/// Apply the scan measurement likelihood to each particle weight.
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
void BeamModel::ApplyMeasurementLikelihood(
  std::vector<Particle> & particles,
  const LaserScanData & scan,
  const LocalizationMap & map,
  const Pose2D & base_to_laser_pose) const
{
  // 如果粒子向量为空，或者激光扫描数据无效，则直接返回，不进行权重更新。 
  if (particles.empty() || scan.ranges.empty() || scan.range_max <= 0.0 ||
      !std::isfinite(scan.range_max)) {
    return;
  }

  /*
    ẑ = h(beam_pose)  根据粒子的位置与朝向计算理论激光距离
                  
    d = z - ẑ         根据观测值与理论值计算误差
            
    p = exp(-0.5 * (d^2) / (sigma^2)) / (sqrt(2 * pi) * sigma)  根据误差计算高斯分布的概率密度函数值
    
    z1, z2, z3 ... zn 假设在一个位置上有n个激光则有n个理论距离
           | 
           ▼
    p1, p2, p3 ... pn 得到n个概率密度函数值
           | 
           ▼
    w_new = w_old * p1 * p2 * ... * pn 累乘得到新的权重
  */

  /*
    为了提高计算效率，使用步长 step 来选择激光束进行权重更新。
    N: 激光束总数，M: 最大使用的激光束数量
    计算公式为：
      step = max(1, (scan.ranges.size() - 1) / max(1, max_beams - 1))
    这样可以确保在激光束数量较多时，仍然能够均匀地选择激光束进行权重更新。  
  */
  const std::size_t step = std::max<std::size_t>(1U, (scan.ranges.size() - 1U) / std::max<std::size_t>(1U, max_beams_ - 1U));

  // 遍历所有粒子，分别根据当前激光扫描结果更新每个粒子的权重。
  for (auto & particle : particles) 
  {
    /* 将激光雷达相对于机器人底盘的位姿，
       变换到当前粒子所代表的世界坐标系位姿中。
     
       particle.pose        ：假设机器人当前所在的位姿
       base_to_laser_pose   ：激光雷达相对于机器人底盘的位姿
       laser_pose           ：该假设下激光雷达在地图中的实际位姿
    */
    const Pose2D laser_pose = ComposePose2D(particle.pose, base_to_laser_pose);
    /* 将粒子原来的权重转换到对数域。
    
      原本最终需要计算：
       w' = w × p1 × p2 × ... × pn
    
       利用：
       log(ab) = log(a) + log(b)
    
       转换为：
       log(w') = log(w) + log(p1) + ... + log(pn)
     
       这样可以避免大量很小的概率连续相乘造成浮点数下溢。
    
       numeric_limits<double>::min() 用来保证输入 log() 的值大于 0，
       防止出现 log(0) = -∞。
    */
    double log_weight = std::log(
      std::max(
        particle.weight,
        kMinimumPositiveDouble
      )
    );


    // 按 step 间隔，从整帧激光扫描中均匀抽取部分激光束。
    for (std::size_t index = 0;index < scan.ranges.size();index += step) 
    {
      // 当前这根激光实际测得的距离 z。
      const double observed_range = scan.ranges[index];

      // 如果测距不是有限值（NaN / ±∞），
      // 或者小于雷达允许的最小测距，则认为该束无效，跳过。
      if (!std::isfinite(observed_range) || observed_range < scan.range_min) 
      {
        continue;
      }


      /* 计算当前激光束相对于激光雷达自身坐标系的角度：
      
         θ_k = angle_min + k × angle_increment
      
         index             ：当前激光束编号
         angle_min         ：第一根激光束的角度
         angle_increment   ：相邻两根激光束之间的角度差
      */
      const double bearing = scan.angle_min + static_cast<double>(index) * scan.angle_increment;


      /* 得到当前激光束在地图坐标系中的射线位姿。
      
         射线起点：
         (laser_pose.x, laser_pose.y)
        
         射线方向：
         laser_pose.yaw + bearing
        
         NormalizeAngle() 将角度限制到标准范围内。
      */
      const Pose2D beam_pose
      {
        laser_pose.x,
        laser_pose.y,
        NormalizeAngle(laser_pose.yaw + bearing)
      };


      /* 初始化理论测距为激光最大量程。
      
         如果射线在地图中没有碰到障碍物，
         expected_range 就保持为 range_max。
      */
      double expected_range = scan.range_max;


      /* 
         从当前粒子假设的激光位置向地图发射一条射线，
         计算如果机器人真的位于这个粒子的位置，
         这根激光理论上应该测到的距离 ẑ。
      */
      map.CastRay(
        beam_pose,
        scan.range_max,
        expected_range
      );


      // ------------------------------
      // 1. 正常命中模型 p_hit
      // ------------------------------
      //
      // 比较：
      //
      // 实际距离 z
      // 理论距离 ẑ
      //
      // 误差：
      //
      // e = z - ẑ
      //
      // 使用高斯分布计算这个误差出现的可能性：
      //
      // p_hit ∝ exp(-(z - ẑ)² / (2σ²))
      //
      // 实际距离越接近理论距离，
      // NormalPdf() 返回值越大，
      // 说明当前粒子越符合地图和激光观测。
      double probability = z_hit_ * NormalPdf(observed_range - expected_range,
                                             sigma_hit_);                                             


      // ------------------------------
      // 2. 短距离模型 p_short
      // ------------------------------
      //
      // 如果：
      //
      // observed_range <= expected_range
      //
      // 即实际激光比地图预测更早碰到东西，
      // 可能意味着出现了地图中没有记录的临时障碍物，
      // 例如人、箱子、移动物体等。
      //
      // 使用指数分布：
      //
      // p_short = λ exp(-λz)
      //
      // 来给这种情况额外的概率。
      if (observed_range <= expected_range) 
      {
        probability +=
          z_short_ *
          lambda_short_ *
          std::exp(-lambda_short_ * observed_range);
      }


      // ------------------------------
      // 3. 最大量程模型 p_max
      // ------------------------------
      //
      // 如果激光测量值达到最大量程，
      // 说明这根激光可能没有碰到障碍物，
      // 因此给这种情况增加一个固定概率 z_max_。
      if (observed_range >= scan.range_max) 
      {
        probability += z_max_;
      }


      // ------------------------------
      // 4. 随机测量模型 p_rand
      // ------------------------------
      //
      // 给任意测距值一个均匀分布的基础概率：
      //
      // p_rand = 1 / range_max
      //
      // 用来描述传感器随机噪声、异常反射等情况，
      // 防止某一根异常激光直接让粒子概率变成 0。
      probability += z_rand_ / scan.range_max;


      // 将当前激光束得到的概率加入粒子的总对数权重：
      //
      // log(w') += log(p_k)
      //
      // 等价于普通概率域中的：
      //
      // w' *= p_k
      //
      // 1e-12 用于避免 probability = 0 时出现 log(0)。
      log_weight += std::log(
          std::max(probability, kMinimumProbability)
        );
    }


    // 所有激光束处理完成后，
    // 将对数权重重新转换回普通权重：
    //
    // w = exp(log(w))
    //
    // 即：
    //
    // w_new = w_old × p1 × p2 × ... × pn
    //
    // 将 log_weight 限制在 [-745, 709]，
    // 是为了避免 double 在执行 exp() 时发生上溢或严重下溢。
    particle.weight = std::exp(
          std::max(kExpLowerLimit,
          std::min(kExpUpperLimit, log_weight)
        )
      );
  }
}

}  // namespace mini_nav_core::localization

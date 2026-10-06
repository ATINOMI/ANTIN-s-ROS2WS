/**
 * @file particle_filter.hpp
 * @brief 与 ROS 解耦的二维粒子滤波定位器声明。
 *
 * ParticleFilter 维护一组带权重的机器人候选位姿。一次定位循环可依次根据
 * 里程计预测粒子、用激光观测更新似然、估计当前位姿，并按归一化权重重采样。
 * 初始化既支持围绕已知位姿的高斯分布，也支持在地图已知自由栅格上的全局分布。
 *
 * 本类仅依赖运动模型与激光模型的抽象接口，以保持滤波状态机与 ROS 消息解耦；
 * 节点层负责提供里程计和传感器数据。
 *
 * @author Antinomy
 * @date 2026-09-21
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "mini_nav_core/localization/amcl/kd_tree.hpp"
#include "mini_nav_core/localization/amcl/localization_constants.hpp"
#include "mini_nav_core/localization/amcl/laser_model.hpp"
#include "mini_nav_core/localization/amcl/motion_model.hpp"

namespace mini_nav_core::localization
{

/**
 * @brief 控制粒子数量与 KLD 自适应重采样的参数。
 *
 * min_particles 与 max_particles 相等时，滤波器保持固定粒子数；否则，
 * Resample() 会由已占用位姿分箱数估计粒子数，并限制在该闭区间内。
 *
 * kld_normal_quantile 大于 1 时直接表示标准正态分位数；位于 (0, 1] 时
 * 表示累积概率，代码会将其转换为分位数。恢复系数控制观测权重的
 * 快速和慢速平均值；当近期平均值下降时，重采样会注入随机姿态。
 */
struct ParticleFilterOptions
{
  /// 粒子数下界，必须大于零。
  std::size_t min_particles{kDefaultMinParticles};
  /// 粒子数上界，不能小于 min_particles。
  std::size_t max_particles{kDefaultMaxParticles};
  /// KLD 自适应粒子数的允许误差，必须为正且有限。
  double pf_err{kDefaultKldError};
  /// 直接使用的正态分位数，或位于 (0, 1] 的累积概率。
  double kld_normal_quantile{kDefaultKldNormalQuantile};
  /// 观测平均权重的快速更新系数，范围 [0, 1]；为零时保持首次观测值。
  double recovery_alpha_fast{0.0};
  /// 观测平均权重的慢速更新系数，范围 [0, 1]；为零时保持首次观测值。
  double recovery_alpha_slow{0.0};
};

/** 一次重采样中按输出顺序记录的真实抽样决策。 */
struct ResampleDraw
{
  double sample{0.0};
  std::size_t source_index{0};
  bool recovery{false};
};

/** 仅供诊断使用；不改变正常定位的重采样路径。 */
struct ResampleTrace
{
  std::vector<double> source_weights;
  double offset{0.0};
  std::vector<ResampleDraw> draws;
};

/**
 * @class ParticleFilter
 * @brief 维护并更新二维机器人候选位姿的带权粒子集合。
 *
 * 每个 Particle 保存以米计的 x/y、以弧度计的 yaw 与非负观测权重。
 * 调用 MotionUpdate()、SensorUpdate() 或 Resample() 前必须完成初始化；
 * 未初始化或权重不能形成有限估计时，Estimate() 返回 invalid 结果。
 *
 * 本类拥有传入的 MotionModel 和 LaserModel；传入更新方法的 LocalizationMap
 * 不转移所有权，仅需在调用期间保持有效。
 */
class ParticleFilter
{
public:
  /**
   * @brief 构造不含运动与激光模型的固定粒子数滤波器。
   *
   * @param particle_count 固定粒子数，必须大于零。
   * @param seed 初始化与重采样共用的伪随机数种子。
   * @throws std::invalid_argument 当 particle_count 为零时抛出。
   */
  explicit ParticleFilter(std::size_t particle_count, std::uint64_t seed = kDefaultRandomSeed);

  /**
   * @brief 构造带运动模型和激光模型的粒子滤波器。
   *
   * @param motion_model 由对象接管的运动模型，不能为空。
   * @param laser_model 由对象接管的激光模型，不能为空。
   * @param options 粒子数量与 KLD 重采样配置。
   * @param seed 伪随机数种子。
   * @throws std::invalid_argument 当配置非法或模型为空时抛出。
   */
  ParticleFilter(
    std::unique_ptr<MotionModel> motion_model,
    std::unique_ptr<LaserModel> laser_model,
    ParticleFilterOptions options = {},
    std::uint64_t seed = kDefaultRandomSeed);

  /**
   * @brief 以给定位姿为中心执行局部初始化。
   *
   * 这是 InitializeLocalized() 的兼容入口。协方差的状态顺序固定为
   * (x, y, yaw)：x/y 的单位为米，yaw 的单位为弧度；各元素相应具有平方
   * 单位或交叉单位。初始化成功后，每个粒子具有相同权重。
   *
   * @param pose 初始粒子分布的均值位姿。
   * @param covariance 对称半正定的 3×3 位姿协方差。
   * @throws std::invalid_argument 当协方差含非有限值、不满足对称容差或
   *                               不是半正定矩阵时抛出。
   */
  void Initialize(const Pose2D & pose, const Covariance3 & covariance);
  /**
   * @brief 在给定位姿周围生成等权重的高斯粒子云。
   *
   * 对标准三维正态随机向量 z 使用协方差的下三角 Cholesky 因子 L，得到
   * 扰动 Lz。yaw 会归一化到 [-π, π]，以避免跨越角度周期边界后出现不连续。
   * 保留当前粒子数；若集合为空，则使用 options.max_particles。
   *
   * @param pose 初始高斯分布的均值位姿。
   * @param covariance 按 (x, y, yaw) 排列的对称半正定位姿协方差。
   * @throws std::invalid_argument 当协方差无法分解为有限、对称且半正定的
   *                               矩阵时抛出。
   */
  void InitializeLocalized(const Pose2D & pose, const Covariance3 & covariance);
  /**
   * @brief 在地图已知自由区域生成全局等权重粒子分布。
   *
   * 每个粒子位于随机采样的已知自由栅格中心，yaw 独立均匀分布在 [-π, π]。
   * 无论此前集合大小如何，粒子数均重置为 options.max_particles。
   *
   * @param map 提供已知自由栅格采样功能的定位地图。
   * @throws std::runtime_error 当地图没有已知自由栅格或采样失败时抛出。
   */
  void InitializeGlobal(const LocalizationMap & map);

  /**
   * @brief 根据相邻两帧里程计位姿预测全部粒子的运动。
   * @param previous_odom_pose 上一帧里程计位姿。
   * @param current_odom_pose 当前帧里程计位姿。
   * @throws std::logic_error 当未初始化或未配置运动模型时抛出。
   * @note 本方法不改变权重；观测更新和重采样须由调用方在适当时机执行。
   */
  void MotionUpdate(const Pose2D & previous_odom_pose, const Pose2D & current_odom_pose);
  /**
   * @brief 用一次激光观测更新粒子的观测似然。
   * @param scan 当前激光扫描数据。
   * @param map 用于射线或似然场查询的定位地图。
   * @param base_to_laser_pose base 坐标系到激光坐标系的外参位姿。
   * @throws std::logic_error 当未初始化或未配置激光模型时抛出。
   * @note 激光模型直接写入权重；之后通常应归一化并按策略重采样。
   */
  void SensorUpdate(
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose);

  /**
   * @brief 批量替换粒子权重，不执行归一化。
   * @param weights 与当前粒子数相同的有限非负权重。
   * @throws std::invalid_argument 当数量不匹配或存在负值、NaN、无穷值时抛出。
   */
  void SetWeights(const std::vector<double> & weights);
  /** @brief 将当前权重归一化，使其总和为一。@return 成功返回 true，否则返回 false。 */
  bool NormalizeWeights();
  /**
   * @brief 按系统重采样生成新的等权重粒子集合。
   * @return 重采样完成返回 true；未初始化、无粒子或权重不可归一化时返回 false。
   * @throws std::logic_error 当需要随机位姿却未提供地图时抛出。
   */
  bool Resample();
  /**
   * @brief 重采样并可选记录实际累计权重区间和粒子来源。
   *
   * @param map 随机恢复时用于采样已知自由空间的地图。
   * @param trace 诊断输出，nullptr 时不记录。
   * @return 成功为 true；状态或权重不可用时 false。
   * @throws std::runtime_error 需要随机恢复而自由空间采样失败。
   */
  bool Resample(const LocalizationMap & map);
  /**
   * @brief 重采样并可选记录实际累计权重区间和粒子来源。
   *
   * @param map 随机恢复时用于采样已知自由空间的地图。
   * @param trace 诊断输出，nullptr 时不记录。
   * @return 成功为 true；状态或权重不可用时 false。
   * @throws std::runtime_error 需要随机恢复而自由空间采样失败。
   */
  bool Resample(const LocalizationMap & map, ResampleTrace * trace);
  /**
   * @brief 计算当前粒子集的加权均值位姿及协方差。
   * @return 初始化状态和总权重有效时返回 valid 估计；否则返回默认 invalid 估计。
   * @note yaw 使用正弦/余弦的圆统计均值，协方差使用最短角距离。
   */
  PoseEstimate Estimate() const;

  /** @brief 查询粒子集合是否已建立。@return 已初始化返回 true。 */
  bool IsInitialized() const;
  /** @brief 只读访问当前粒子集合。@return 当前粒子集合的常量引用。 */
  const std::vector<Particle> & GetParticles() const;

private:
  /** @brief 校验固定粒子数。@param count 待校验数量。@throws std::invalid_argument 当数量为零时抛出。 */
  void ValidateParticleCount(std::size_t count) const;
  /**
   * @brief 对 3×3 对称半正定协方差执行下三角分解。
   * @param covariance 待检验并分解的协方差。
   * @param lower 输出下三角因子，满足 covariance = lower × lower^T。
   * @return 分解成功返回 true；输入非有限、非对称或非半正定时返回 false。
   */
  static bool CholeskyDecompose(const Covariance3 & covariance, double lower[3][3]);
  /** @brief 近似计算标准正态分布的分位数。@param probability 累积概率。@return 对应分位数。 */
  static double NormalQuantile(double probability);
  /** @brief 根据 KLD 公式计算目标粒子数。@return 限制在配置上下界内的粒子数。 */
  std::size_t GetTargetParticleCount() const;
  /**
   * @brief 归一化源权重并执行自适应系统重采样，可选记录真实抽样轨迹。
   *
   * 按位姿分箱估计数量并限制在配置区间；输出粒子等权，记录只用于诊断。
   *
   * @param map 随机恢复位姿的地图来源；未提供而需要恢复时抛出异常。
   * @param trace 可选诊断输出指针，nullptr 禁用记录。
   * @return 完成重采样为 true；未初始化或无法归一化为 false。
   * @throws std::logic_error 需要随机恢复但无地图；自由空间采样失败可能抛出 std::runtime_error。
   */
  bool ResampleImpl(const LocalizationMap * map, ResampleTrace * trace);

  std::vector<Particle> particles_;
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<LaserModel> laser_model_;
  ParticleFilterOptions options_;
  std::mt19937_64 generator_;
  PoseBinIndex pose_bin_index_;
  bool initialized_{false};
  double fast_mean_weight_{0.0};
  double slow_mean_weight_{0.0};
};

}  // namespace mini_nav_core::localization

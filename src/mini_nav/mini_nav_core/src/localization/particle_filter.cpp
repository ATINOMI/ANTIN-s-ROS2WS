/**
 * @file particle_filter.cpp
 * @brief 二维粒子滤波定位器的实现。
 *
 * 实现局部/全局初始化、运动与观测更新的分发、权重归一化、系统重采样、
 * 位姿与协方差估计，以及基于位姿分箱的 KLD 自适应粒子数。
 *
 * @author Antinomy
 * @date 2026-09-21
 */

/* Includes ----------------------------------------------------------------*/

#include "mini_nav_core/localization/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

/* Namespace ----------------------------------------------------------------*/

namespace mini_nav_core::localization
{

namespace
{

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief 校验自适应粒子滤波参数的数值范围。
 * @param options 待校验的粒子滤波配置。
 * @throws std::invalid_argument 当粒子数范围不合法、误差或分位数非正，
 *                               或恢复系数为负/非有限时抛出。
 */
void ValidateOptions(const ParticleFilterOptions & options)
{
  /*
   * kld_normal_quantile 支持两种输入约定：大于 1 时视为已经给出的正态
   * 分位数，否则视为 (0, 1] 内的累积概率，并在计算目标粒子数时转换。
   * 恢复系数在观测更新时维护快慢平均权重，并在重采样时决定随机粒子比例。
   * 系数限制在 [0, 1]，以保持指数平均值非负且有限。
   */
  if (options.min_particles == 0U || options.max_particles < options.min_particles      ||
      !std::isfinite(options.pf_err)              || options.pf_err <= 0.0              ||
      !std::isfinite(options.kld_normal_quantile) || options.kld_normal_quantile <= 0.0 ||
      !std::isfinite(options.recovery_alpha_fast) || options.recovery_alpha_fast < 0.0  ||
      options.recovery_alpha_fast > 1.0 ||
      !std::isfinite(options.recovery_alpha_slow) || options.recovery_alpha_slow < 0.0 ||
      options.recovery_alpha_slow > 1.0)
  {
    throw std::invalid_argument("Invalid particle filter options");
  }
}

/**
 * @brief 保存 Acklam 正态分位数近似在不同区间使用的多项式系数。
 *
 * 系数按公式中的 a、b、c、d 分组，避免在 NormalQuantile() 中散落魔法数字。
 */
struct NormalQuantileCoefficients
{
  double a1; double a2; double a3; double a4; double a5; double a6;
  double b1; double b2; double b3; double b4; double b5;
  double c1; double c2; double c3; double c4; double c5; double c6;
  double d1; double d2; double d3; double d4;
};

constexpr NormalQuantileCoefficients kNormalQuantileCoefficients{
  -39.6968302866538,   220.946098424521,    -275.928510446969,
  138.357751867269,   -30.6647980661472,     2.50662827745924,
  -54.4760987982241,   161.585836858041,    -155.698979859887,
  66.8013118877197,   -13.2806815528857,    -0.00778489400243029,
  -0.322396458041136, -2.40075827716184,    -2.54973253934373,
  4.37466414146497,    2.93816398269878,     0.00778469570904146,
  0.32246712907004,    2.445134137143,       3.75440866190742};

}  // namespace

/**
 * @brief 构造固定粒子数且不含更新模型的滤波器。
 * @param particle_count 固定粒子数。
 * @param seed 伪随机数种子。
 * @throws std::invalid_argument 当 particle_count 为零时抛出。
 */
ParticleFilter::ParticleFilter(std::size_t particle_count, std::uint64_t seed)
: particles_(particle_count),
  options_{particle_count, particle_count, kDefaultKldError, kDefaultKldNormalQuantile, 0.0, 0.0},
  generator_(seed)
{
  // 仅在构造时检查粒子数，避免后续 Resize() 时重复抛出异常。
  ValidateParticleCount(particle_count);
  // 粒子集合在构造时就分配好空间，避免后续 Resize() 时重复抛出异常。
  particles_.resize(particle_count);
}

/**
 * @brief 构造拥有运动模型与激光模型的滤波器。
 * @param motion_model 转移给滤波器的运动模型。
 * @param laser_model 转移给滤波器的激光模型。
 * @param options 粒子数和 KLD 配置。
 * @param seed 伪随机数种子。
 * @throws std::invalid_argument 当模型为空或配置不合法时抛出。
 */
ParticleFilter::ParticleFilter(
  std::unique_ptr<MotionModel> motion_model,
  std::unique_ptr<LaserModel> laser_model,
  ParticleFilterOptions options,
  std::uint64_t seed)
: particles_(options.max_particles), motion_model_(std::move(motion_model)),
  laser_model_(std::move(laser_model)), options_(options), generator_(seed)
{
  // 校验粒子数范围、KLD 误差、分位数和恢复系数的合法性。
  ValidateOptions(options_);

  // 校验粒子滤波器的运动模型和激光模型是否为空，确保滤波器在使用时有有效的模型。
  if (!motion_model_ || !laser_model_) 
  {
    throw std::invalid_argument("Particle filter models must not be null"); 
  }
}

/**
 * @brief 转发至局部初始化实现，保留兼容调用接口。
 * @param pose 初始均值位姿。
 * @param covariance 初始位姿协方差。
 * @throws std::invalid_argument 当协方差不合法时抛出。
 */
void ParticleFilter::Initialize(const Pose2D & pose, const Covariance3 & covariance)
{
  // 生成局部高斯粒子云，使用协方差的下三角 Cholesky 因子生成扰动，确保 x、y 和 yaw 的相关性。
  InitializeLocalized(pose, covariance);
}

/**
 * @brief 以一个给定的位姿和协方差为中心，
 *        通过协方差采样生成局部高斯粒子云。
 *        它既可以用于局部初始化，也可以用于局部重定位。
 * @param pose 初始粒子分布的均值位姿。
 * @param covariance 按 (x, y, yaw) 排列的对称半正定协方差。
 * @throws std::invalid_argument 当协方差无法完成有限的半正定分解时抛出。
 */
void ParticleFilter::InitializeLocalized(const Pose2D & pose, const Covariance3 & covariance)
{
  // 生成协方差的下三角 Cholesky 因子，若失败则抛出异常，确保协方差是半正定的。
  double lower[3][3]{};
  if (!CholeskyDecompose(covariance, lower)) 
  {
    throw std::invalid_argument("Pose covariance must be positive semidefinite");
  }

  /*
   * 对 z ~ N(0, I) 使用协方差的下三角因子 L 生成扰动 Lz；因为
   * L L^T = covariance，同一组标准正态样本会按协方差的交叉项共同改变
   * x、y 和 yaw，而不是把三个维度错误地当成相互独立的噪声。
   */
  /*
   * 局部重定位应保持既有粒子规模；只有尚无粒子时才回退到配置的上限。
   * 这避免 KLD 重采样后的自适应规模在下一次局部初始化时被意外抹除。
   */
  // 计算粒子数，若当前粒子集合为空，则使用配置的最大粒子数, 否则保持当前粒子数不变。
  const std::size_t count = particles_.empty() ? options_.max_particles : particles_.size();

  // 使用标准正态分布生成随机扰动，确保每个粒子在 x、y 和 yaw 上的扰动符合协方差的相关性。
  std::normal_distribution<double> normal(0.0, 1.0);

  // 调整粒子集合的大小以匹配计算出的粒子数，确保粒子集合在局部初始化后具有正确的数量。
  particles_.resize(count);

  // 对每个粒子应用协方差扰动，生成局部高斯分布的粒子云，并将权重初始化为均等值。
  for (auto & particle : particles_) 
  {

    // 生成三个独立的标准正态随机数，用于计算粒子在 x、y 和 yaw 上的扰动。
    const double random[3] = {normal(generator_), normal(generator_), normal(generator_)};

    // 使用协方差的下三角因子将标准正态扰动转换为具有相关性的扰动，并应用到粒子的位姿上。
    particle.pose.x = pose.x + lower[0][0] * random[0];
    particle.pose.y = pose.y + lower[1][0] * random[0] + lower[1][1] * random[1];
    particle.pose.yaw = NormalizeAngle(
      pose.yaw + lower[2][0] * random[0] + lower[2][1] * random[1] + lower[2][2] * random[2]);

    // 将粒子权重初始化为均等值，确保所有粒子在局部初始化后具有相同的重要性。
    particle.weight = 1.0 / static_cast<double>(count);
  }

  // 标记滤波器为已初始化，重置快速和慢速平均权重，并构建位姿分箱索引以支持 KLD 自适应粒子数。
  initialized_ = true;
  fast_mean_weight_ = 0.0;
  slow_mean_weight_ = 0.0;
  pose_bin_index_.BuildPoseBinIndex(particles_);
}

/**
 * @brief 在地图已知自由区域内生成全局均匀分布的粒子云。
 *
 * 从地图的已知自由栅格中随机采样粒子位置，并在 [-pi, pi]
 * 范围内均匀采样粒子朝向，用于机器人初始位姿完全未知或
 * 定位丢失后的全局重定位。
 *
 * 所有粒子初始化为相同权重，并在初始化完成后重置恢复统计量，
 * 同时重建位姿分箱索引以支持后续 KLD 自适应重采样。
 *
 * @param map 用于提供已知自由栅格采样及栅格中心坐标的定位地图。
 *
 * @throws std::runtime_error 当地图中不存在已知自由栅格，
 *                            或自由栅格采样失败时抛出。
 */
void ParticleFilter::InitializeGlobal(const LocalizationMap & map)
{
  // 在地图自由区域均匀抽取位置并随机设置朝向。
  const std::size_t count = options_.max_particles;

  // 检查地图中是否存在已知自由栅格，如果没有则抛出异常，确保全局初始化有可用的自由区域。
  if (map.GetKnownFreeCellCount() == 0U) 
  {
    throw std::runtime_error("Cannot globally initialize on a map without free cells");
  }

  // 使用连续均匀分布模型在 [-π, π] 范围内随机生成粒子的朝向，确保全局初始化的粒子云覆盖整个方向空间。
  std::uniform_real_distribution<double> yaw_distribution(-kPi, kPi);

  // 调整粒子集合的大小以匹配计算出的粒子数，确保粒子集合在全局初始化后具有正确的数量。
  particles_.resize(count);

  /*
   * 位置只从地图已经确认的自由栅格采样，避免全局初始化直接把粒子放进
   * 障碍物；使用栅格中心而不是任意连续坐标，也与 LocalizationMap 的离散
   * 占用语义保持一致。朝向则独立覆盖完整的周期范围。
   */
  for (auto & particle : particles_) 
  {
    // 从地图中采样一个已知自由栅格，如果采样失败则抛出异常，确保每个粒子都位于合法的自由区域。
    MapCell cell;

    // 采样已知自由栅格，如果采样失败则抛出异常，确保每个粒子都位于合法的自由区域。
    if (!map.SampleKnownFreeCell(generator_, cell)) 
    {
      throw std::runtime_error("Failed to sample a free map cell");
    }

    // 将采样到的栅格坐标转换为世界坐标，并设置粒子的位姿，确保粒子位于自由区域的中心。
    map.GetCellCenter(cell, particle.pose.x, particle.pose.y);

    // 随机生成粒子的朝向，确保全局初始化的粒子云覆盖整个方向空间。
    particle.pose.yaw = yaw_distribution(generator_);

    // 将粒子权重初始化为均等值，确保所有粒子在全局初始化后具有相同的重要性。
    particle.weight = 1.0 / static_cast<double>(count);
  }
  // 标记滤波器为已初始化，重置快速和慢速平均权重，并构建位姿分箱索引以支持 KLD 自适应粒子数。
  initialized_ = true;
  fast_mean_weight_ = 0.0;
  slow_mean_weight_ = 0.0;
  pose_bin_index_.BuildPoseBinIndex(particles_);
}

/**
 * @brief 将里程计增量交给运动模型，预测所有粒子位姿。
 * @param previous_odom_pose 上一帧里程计位姿。
 * @param current_odom_pose 当前帧里程计位姿。
 * @throws std::logic_error 当滤波器未初始化或没有运动模型时抛出。
 */
void ParticleFilter::MotionUpdate(
  const Pose2D & previous_odom_pose, const Pose2D & current_odom_pose)
{
  
  // 检查滤波器是否已初始化，如果未初始化则抛出异常，确保运动更新在有效状态下进行。
  if (!initialized_) 
  {
    throw std::logic_error("Particle filter is not initialized");
  }

  // 检查滤波器是否配置了运动模型，如果没有则抛出异常，确保运动更新有有效的模型进行处理。
  if (!motion_model_) 
  {
    throw std::logic_error("Particle filter has no motion model");
  }
  /*
   * 运动模型负责从两帧里程计中提取相对运动，并按其具体实现更新各粒子的
   * 位姿（例如可叠加运动噪声）；滤波器本身只负责状态检查和调用，不在此处修改权重。
   */
  motion_model_->UpdateParticles(particles_, previous_odom_pose, current_odom_pose);
}

/**
 * @brief 将激光观测交给激光模型，给粒子打分，更新粒子权重。
 * @param scan 当前激光扫描。
 * @param map 定位地图。
 * @param base_to_laser_pose base 到激光坐标系的外参。
 * @throws std::logic_error 当滤波器未初始化或没有激光模型时抛出。
 */
void ParticleFilter::SensorUpdate(
  const LaserScanData & scan,
  const LocalizationMap & map,
  const Pose2D & base_to_laser_pose)
{

  // 检查滤波器是否已初始化，如果未初始化则抛出异常，确保传感器更新在有效状态下进行。
  if (!initialized_) 
  {
    throw std::logic_error("Particle filter is not initialized");
  }

  // 检查滤波器是否配置了激光模型，如果没有则抛出异常，确保传感器更新有有效的模型进行处理。
  if (!laser_model_) 
  {
    throw std::logic_error("Particle filter has no laser model");
  }
  /*
   * 激光模型根据当前激光扫描和地图信息，计算每个粒子的观测似然，并将结果写回粒子的权重中。
   * 激光模型直接把观测似然写回每个粒子的 weight，下一步通常是归一化和重采样。
   */
  laser_model_->ApplyMeasurementLikelihood(particles_, scan, map, base_to_laser_pose);
  // 在归一化前记录平均观测权重，快慢变化的差距表示近期观测是否突然变差。
  const double total = std::accumulate(
    particles_.begin(), particles_.end(), 0.0,
    [](double sum, const Particle & particle) { return sum + particle.weight; });
  if (std::isfinite(total) && total > 0.0) {
    const double average = total / static_cast<double>(particles_.size());
    if (slow_mean_weight_ == 0.0) {
      slow_mean_weight_ = average;
    } else {
      slow_mean_weight_ += options_.recovery_alpha_slow * (average - slow_mean_weight_);
    }
    if (fast_mean_weight_ == 0.0) {
      fast_mean_weight_ = average;
    } else {
      fast_mean_weight_ += options_.recovery_alpha_fast * (average - fast_mean_weight_);
    }
  }
}

/**
 * @brief 用调用方提供的权重替换当前权重。
 * @param weights 数量匹配且均为有限非负数的权重。
 * @throws std::invalid_argument 当输入不满足上述约束时抛出。
 */
void ParticleFilter::SetWeights(const std::vector<double> & weights)
{
  // 检查输入权重的数量是否与当前粒子数匹配，如果不匹配则抛出异常，确保权重设置的合法性。
  if (weights.size() != particles_.size()) 
  {
    throw std::invalid_argument("Weight count must match particle count");
  }
  // 检查输入权重是否包含非有限值或负值，如果存在则抛出异常，确保权重设置的合法性。
  const bool has_invalid_weight = std::any_of(
    weights.begin(), 
    weights.end(),
    [](double weight) { return !std::isfinite(weight) || weight < 0.0; });

  if (has_invalid_weight) 
  {
    throw std::invalid_argument("Particle weights must be finite and non-negative");
  }

  /* 零权重是合法输入；此接口故意不归一化，以便调用方控制观测更新的时序。 */
  for (std::size_t index = 0; index < particles_.size(); ++index) 
  {
    particles_[index].weight = weights[index];
  }
}

/** @brief 使粒子权重和归一化为一。@return 成功返回 true，否则返回 false。 */
bool ParticleFilter::NormalizeWeights()
{
  // 检查粒子集合是否为空，如果为空则返回 false，避免对空集合进行归一化操作。
  if (particles_.empty()) 
  {
    return false;
  }
  //  计算粒子权重的总和，如果总和不是有限正数则返回 false，避免除以零或无效值。
  const double total = std::accumulate(
    particles_.begin(),
    particles_.end(), 
    0.0,
    // 使用 lambda 函数累加每个粒子的权重，计算总权重。
    [](double sum, const Particle & particle) { return sum + particle.weight; });
  
  // 检查总权重是否为有限正数，如果不是则返回 false，避免归一化操作导致无效结果。  
  if (!std::isfinite(total) || total <= 0.0)
  {
    return false;
  }
  /*
   * 只有总权重为有限正数时才修改粒子，避免全零、NaN 或无穷权重被除法
   * 放大成不可用状态；在输入权重本身合法时，归一化后其离散概率质量之和为 1。
   */
  for (auto & particle : particles_) 
  {
    particle.weight /= total;
  }
  return true;
}

/**
 * @brief KLD决定保留多少粒子，通过低方差系统重采样淘汰低权重粒子并复制高权重粒子。
 * @return 成功完成返回 true；前置状态或权重不合法时返回 false。
 */
bool ParticleFilter::Resample()
{
  return ResampleImpl(nullptr);
}

bool ParticleFilter::Resample(const LocalizationMap & map)
{
  return ResampleImpl(&map);
}

bool ParticleFilter::ResampleImpl(const LocalizationMap * map)
{
  // 检查滤波器是否已初始化，粒子集合是否为空，以及权重是否已归一化。如果任一条件不满足，则返回 false。
  if (!initialized_ || particles_.empty() || !NormalizeWeights()) 
  {
    return false;
  }

  // 先把当前所有粒子按照（x, y, yaw）分箱索引，便于 KLD 自适应粒子数计算和后续重采样操作。
  pose_bin_index_.BuildPoseBinIndex(particles_);

  // 由已占用位姿分箱数量计算 KLD 自适应重采样的粒子数。
  const std::size_t target_count = GetTargetParticleCount();

  double random_probability = 0.0;
  if (slow_mean_weight_ > 0.0) {
    random_probability = std::clamp(1.0 - fast_mean_weight_ / slow_mean_weight_, 0.0, 1.0);
  }
  if (random_probability > 0.0 && map == nullptr) {
    throw std::logic_error("Recovery resampling requires a localization map");
  }
  if (random_probability > 0.0 && map->GetKnownFreeCellCount() == 0U) {
    throw std::runtime_error("Cannot recover on a map without free cells");
  }

  // 创建一个新的粒子数组，大小就是 KLD 算出来的目标粒子数
  std::vector<Particle> resampled(target_count);

  // 使用均匀分布生成一个随机偏移量，确保系统重采样的起点是随机的，从而降低重复抽样的方差。
  std::uniform_real_distribution<double> offset_distribution(
    0.0, 1.0 / static_cast<double>(target_count));
  /*
   * 系统重采样只随机一次起点，随后等间隔扫描累积分布。相较于为每个输出粒子
   * 独立抽样，这会降低重复抽样的方差；改为独立抽样会使定位结果更易抖动。
   */
  const double offset = offset_distribution(generator_);
  std::uniform_real_distribution<double> recovery_distribution(0.0, 1.0);
  std::uniform_real_distribution<double> yaw_distribution(-kPi, kPi);
  std::size_t source_index = 0;
  double cumulative = particles_[0].weight;

  /*
   * 将 [0, 1) 划分为 target_count 个等距采样点，并沿当前粒子的累积分布
   * 权重大的粒子区间更大，因此会被复制更多次而保留下来；权重小的粒子区间更小，甚至可能被淘汰。
   * 近期观测权重下降时，按随机注入概率改从地图自由区域生成候选位姿。
   */
  for (std::size_t index = 0; index < target_count; ++index) {
    if (random_probability > 0.0 && recovery_distribution(generator_) < random_probability) {
      MapCell cell;
      if (!map->SampleKnownFreeCell(generator_, cell)) {
        throw std::runtime_error("Failed to sample a free map cell for recovery");
      }
      map->GetCellCenter(cell, resampled[index].pose.x, resampled[index].pose.y);
      resampled[index].pose.yaw = yaw_distribution(generator_);
    } else {
      const double sample = offset + static_cast<double>(index) / target_count;
      while (sample > cumulative && source_index + 1U < particles_.size()) {
        ++source_index;
        cumulative += particles_[source_index].weight;
      }
      resampled[index].pose = particles_[source_index].pose;
    }
    resampled[index].weight = 1.0 / static_cast<double>(target_count);
  }

  particles_.swap(resampled);
  // 目标粒子数依赖当前分箱；交换后立即重新分箱，供下一次自适应重采样使用。
  pose_bin_index_.BuildPoseBinIndex(particles_);
  if (random_probability > 0.0) {
    fast_mean_weight_ = 0.0;
    slow_mean_weight_ = 0.0;
  }
  return true;
}

/**
 * @brief 从带权粒子集合估计二维位姿和 3×3 协方差。
 * @return 有效权重存在时返回有效估计，否则返回默认 invalid 结果。
 */
PoseEstimate ParticleFilter::Estimate() const
{
  PoseEstimate estimate;

  // 检查滤波器是否已初始化以及粒子集合是否为空，如果未初始化或粒子集合为空，则返回无效的估计结果。
  if (!initialized_ || particles_.empty()) 
  {
    return estimate;
  }

  // 创建一个索引数组，包含从 0 到粒子数量减一的连续整数，用于遍历粒子集合。
  std::vector<std::size_t> indices(particles_.size());
  std::iota(indices.begin(), indices.end(), 0U);

  double total_weight = 0.0;  // 初始化总权重为零，用于累加所有粒子的权重。
  double mean_x = 0.0;        // 初始化加权平均的 x 坐标为零，用于计算粒子集合的加权平均位置。
  double mean_y = 0.0;        // 初始化加权平均的 y 坐标为零，用于计算粒子集合的加权平均位置。
  double mean_sin = 0.0;      // 初始化加权平均的 sin(yaw) 为零，用于计算粒子集合的加权平均朝向。
  double mean_cos = 0.0;      // 初始化加权平均的 cos(yaw) 为零，用于计算粒子集合的加权平均朝向。

  /* 第一遍先保留原始总权重；这样既能支持未归一化输入，也能统一判定有效性。 */
  for (const std::size_t index : indices) {
    const auto & particle = particles_[index];
    total_weight += particle.weight;
    mean_x += particle.weight * particle.pose.x;
    mean_y += particle.weight * particle.pose.y;
    mean_sin += particle.weight * std::sin(particle.pose.yaw);
    mean_cos += particle.weight * std::cos(particle.pose.yaw);
  }
  if (!std::isfinite(total_weight) || total_weight <= std::numeric_limits<double>::epsilon()) {
    return estimate;
  }

  estimate.valid = true;
  estimate.weight = total_weight;
  estimate.pose.x = mean_x / total_weight;
  estimate.pose.y = mean_y / total_weight;
  /*
   * 角度不能直接做算术平均：接近 π 与 -π 的两个朝向实际相近，算术均值却会
   * 落在零附近。先平均单位圆上的 sin/cos，再 atan2 回到正确的周期角度。
   */
  estimate.pose.yaw = std::atan2(mean_sin, mean_cos);

  for (const std::size_t index : indices) {
    const auto & particle = particles_[index];
    const double dx = particle.pose.x - estimate.pose.x;
    const double dy = particle.pose.y - estimate.pose.y;
    const double dyaw = AngularDistance(particle.pose.yaw, estimate.pose.yaw);
    const double normalized_weight = particle.weight / total_weight;
    /*
     * 第二遍计算中心化二阶矩。dyaw 必须是最短角距离，否则跨越 ±π 时会把
     * 实际很小的朝向误差当成接近 2π 的大误差，进而污染 yaw 方差及交叉协方差。
     */
    estimate.covariance.At(0, 0) += normalized_weight * dx * dx;
    estimate.covariance.At(0, 1) += normalized_weight * dx * dy;
    estimate.covariance.At(0, 2) += normalized_weight * dx * dyaw;
    estimate.covariance.At(1, 0) += normalized_weight * dy * dx;
    estimate.covariance.At(1, 1) += normalized_weight * dy * dy;
    estimate.covariance.At(1, 2) += normalized_weight * dy * dyaw;
    estimate.covariance.At(2, 0) += normalized_weight * dyaw * dx;
    estimate.covariance.At(2, 1) += normalized_weight * dyaw * dy;
    estimate.covariance.At(2, 2) += normalized_weight * dyaw * dyaw;
  }
  return estimate;
}

/** @brief 返回初始化状态。@return 已初始化返回 true。 */
bool ParticleFilter::IsInitialized() const
{
  return initialized_;
}

/** @brief 返回当前粒子集合的只读引用。@return 当前粒子集合。 */
const std::vector<Particle> & ParticleFilter::GetParticles() const
{
  return particles_;
}

/**
 * @brief 校验固定粒子数不得为零。
 * @param count 待校验数量。
 * @throws std::invalid_argument 当数量为零时抛出。
 */
void ParticleFilter::ValidateParticleCount(std::size_t count) const
{
  if (count == 0U) {
    throw std::invalid_argument("Particle count must be greater than zero");
  }
}

/**
 * @brief 验证协方差并计算可处理半正定矩阵的下三角分解。
 * @param covariance 按 (x, y, yaw) 排列的 3×3 协方差。
 * @param lower 输出下三角因子。
 * @return 输入有效且分解成功返回 true，否则返回 false。
 */
bool ParticleFilter::CholeskyDecompose(const Covariance3 & covariance, double lower[3][3])
{
  /*
   * 采样只需要半正定协方差，但数值分解仍要求输入近似对称。先检查全部
   * 元素可表示且满足对称容差，再进行分解，可以避免 NaN 或非对称数据在
   * 后续 sqrt/除法中静默传播。
   */
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      const double value = covariance.At(row, column);
      if (!std::isfinite(value)) {
        return false;
      }
      if (std::abs(value - covariance.At(column, row)) > kCovarianceSymmetryTolerance) {
        return false;
      }
    }
  }

  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column <= row; ++column) {
      double value = covariance.At(row, column);
      for (std::size_t k = 0; k < column; ++k) {
        value -= lower[row][k] * lower[column][k];
      }
      if (row == column) {
        /*
         * 允许略小于零的舍入误差，并将其钳制为零；明显负值表示不是半正定。
         * 非对角计算遇到零对角线时还需检查对应元素是否也接近零，否则无法
         * 构成半正定矩阵，不能用除零得到一个伪造的分解。
         */
        if (value < -kCovariancePositiveSemidefiniteTolerance) {
          return false;
        }
        lower[row][column] = std::sqrt(std::max(0.0, value));
      } else {
        if (lower[column][column] <= std::numeric_limits<double>::epsilon()) {
          if (std::abs(value) > kCovariancePositiveSemidefiniteTolerance) {
            return false;
          }
          lower[row][column] = 0.0;
        } else {
          lower[row][column] = value / lower[column][column];
        }
      }
    }
  }
  return true;
}

/**
 * @brief 以分段有理多项式近似标准正态累积分布的反函数。
 * @param probability 累积概率。
 * @return 对应分位数；概率越界时返回有限钳制值 ±8。
 */
double ParticleFilter::NormalQuantile(double probability)
{
  // Acklam 近似在本实现允许的 KLD 参数范围内具有足够精度。
  if (probability <= 0.0) {
    // 分布反函数在端点趋于无穷；这里用有限下界保护 KLD 公式。
    return -8.0;
  }
  if (probability >= 1.0) {
    // 与下端点对称，用有限上界替代正无穷。
    return 8.0;
  }
  const auto & coefficients = kNormalQuantileCoefficients;
  const double lower = 0.02425;
  const double upper = 1.0 - lower;
  if (probability < lower) {
    // 左尾使用 q = sqrt(-2 log(p))，避免直接在尾部用中心区间公式。
    const double q = std::sqrt(-2.0 * std::log(probability));
    return (((((coefficients.c1 * q + coefficients.c2) * q + coefficients.c3) * q + coefficients.c4) * q + coefficients.c5) * q + coefficients.c6) /
      ((((coefficients.d1 * q + coefficients.d2) * q + coefficients.d3) * q + coefficients.d4) * q + 1.0);
  }
  if (probability > upper) {
    // 右尾对 1 - p 做同样的变换，再利用正态分布的对称性取相反数。
    const double q = std::sqrt(-2.0 * std::log(1.0 - probability));
    return -(((((coefficients.c1 * q + coefficients.c2) * q + coefficients.c3) * q + coefficients.c4) * q + coefficients.c5) * q + coefficients.c6) /
      ((((coefficients.d1 * q + coefficients.d2) * q + coefficients.d3) * q + coefficients.d4) * q + 1.0);
  }
  // 中心区间使用关于 0 对称的有理多项式，q 为 p 相对 0.5 的偏移量。
  const double q = probability - 0.5;
  const double r = q * q;
  return (((((coefficients.a1 * r + coefficients.a2) * r + coefficients.a3) * r + coefficients.a4) * r + coefficients.a5) * r + coefficients.a6) * q /
    (((((coefficients.b1 * r + coefficients.b2) * r + coefficients.b3) * r + coefficients.b4) * r + coefficients.b5) * r + 1.0);
}

/**
 * @brief 由已占用位姿分箱数量计算 KLD 自适应重采样的粒子数。
 * @return 固定模式返回固定数量；自适应模式返回经上下界钳制后的数量。
 */
std::size_t ParticleFilter::GetTargetParticleCount() const
{
  if (options_.min_particles == options_.max_particles) {
    return options_.min_particles;
  }

  /*
   * KLD 公式以 bins - 1 为自由度，至少取两个分箱可避免零自由度及除零。
   * 这是数值保护，不意味着粒子集合实际覆盖了两个不同位姿分箱。
   */
  const std::size_t bins = std::max<std::size_t>(2U, pose_bin_index_.GetOccupiedBinCount());
  const double z = options_.kld_normal_quantile > 1.0 ? options_.kld_normal_quantile : NormalQuantile(options_.kld_normal_quantile);
  const double degrees = static_cast<double>(bins - 1U);
  /*
   * 对 k = bins - 1 个自由度，KLD 上界近似为：
   *
   *   n = k / (2 * pf_err) *
   *       (1 - 2/(9k) + z * sqrt(2/(9k)))^3
   *
   * 之后向上取整并钳制到配置区间，保证既满足误差目标的保守取整，又不会
   * 让一次重采样突破 min_particles/max_particles 的资源边界。
   */
  const double estimate = degrees / (2.0 * options_.pf_err) *
    std::pow(1.0 - 2.0 / (9.0 * degrees) + z * std::sqrt(2.0 / (9.0 * degrees)), 3.0);
  const auto count = static_cast<std::size_t>(std::ceil(std::max(1.0, estimate)));
  return std::min(options_.max_particles, std::max(options_.min_particles, count));
}

}  // namespace mini_nav_core::localization

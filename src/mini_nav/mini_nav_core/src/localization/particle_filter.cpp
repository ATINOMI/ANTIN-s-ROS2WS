#include "mini_nav_core/localization/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace mini_nav_core::localization
{

namespace
{
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

void ValidateOptions(const ParticleFilterOptions & options)
{
  if (options.min_particles == 0U || options.max_particles < options.min_particles ||
      !std::isfinite(options.pf_err) || options.pf_err <= 0.0 ||
      !std::isfinite(options.pf_z) || options.pf_z <= 0.0 ||
      !std::isfinite(options.recovery_alpha_fast) || options.recovery_alpha_fast < 0.0 ||
      !std::isfinite(options.recovery_alpha_slow) || options.recovery_alpha_slow < 0.0) {
    throw std::invalid_argument("Invalid particle filter options");
  }
}
}  // namespace

ParticleFilter::ParticleFilter(std::size_t particle_count, std::uint64_t seed)
: particles_(particle_count), options_{particle_count, particle_count, 0.05, 2.33, 0.0, 0.0}, generator_(seed)
{
  ValidateParticleCount(particle_count);
  particles_.resize(particle_count);
}

ParticleFilter::ParticleFilter(
  std::unique_ptr<MotionModel> motion_model,
  std::unique_ptr<LaserModel> laser_model,
  ParticleFilterOptions options,
  std::uint64_t seed)
: particles_(options.max_particles), motion_model_(std::move(motion_model)),
  laser_model_(std::move(laser_model)), options_(options), generator_(seed)
{
  ValidateOptions(options_);
  if (!motion_model_ || !laser_model_) {
    throw std::invalid_argument("Particle filter models must not be null");
  }
}

void ParticleFilter::Initialize(const Pose2D & pose, const Covariance3 & covariance)
{
  InitializeLocalized(pose, covariance);
}

void ParticleFilter::InitializeLocalized(const Pose2D & pose, const Covariance3 & covariance)
{
  double lower[3][3]{};
  if (!CholeskyDecompose(covariance, lower)) {
    throw std::invalid_argument("Pose covariance must be positive semidefinite");
  }

  const std::size_t count = particles_.empty() ? options_.max_particles : particles_.size();
  std::normal_distribution<double> normal(0.0, 1.0);
  particles_.resize(count);
  for (auto & particle : particles_) {
    const double random[3] = {normal(generator_), normal(generator_), normal(generator_)};
    particle.pose.x = pose.x + lower[0][0] * random[0];
    particle.pose.y = pose.y + lower[1][0] * random[0] + lower[1][1] * random[1];
    particle.pose.yaw = NormalizeAngle(
      pose.yaw + lower[2][0] * random[0] + lower[2][1] * random[1] + lower[2][2] * random[2]);
    particle.weight = 1.0 / static_cast<double>(count);
  }
  initialized_ = true;
  w_fast_ = 0.0;
  w_slow_ = 0.0;
  kd_tree_.Build(particles_);
}

void ParticleFilter::InitializeGlobal(const LocalizationMap & map)
{
  const std::size_t count = options_.max_particles;
  if (map.GetFreeCellCount() == 0U) {
    throw std::runtime_error("Cannot globally initialize on a map without free cells");
  }

  std::uniform_real_distribution<double> yaw_distribution(
    -3.14159265358979323846, 3.14159265358979323846);
  particles_.resize(count);
  for (auto & particle : particles_) {
    MapCell cell;
    if (!map.SampleFreeCell(generator_, cell)) {
      throw std::runtime_error("Failed to sample a free map cell");
    }
    map.GetCellCenter(cell, particle.pose.x, particle.pose.y);
    particle.pose.yaw = yaw_distribution(generator_);
    particle.weight = 1.0 / static_cast<double>(count);
  }
  initialized_ = true;
  w_fast_ = 0.0;
  w_slow_ = 0.0;
  kd_tree_.Build(particles_);
}

void ParticleFilter::MotionUpdate(
  const Pose2D & previous_odom_pose, const Pose2D & current_odom_pose)
{
  if (!initialized_) {
    throw std::logic_error("Particle filter is not initialized");
  }
  if (!motion_model_) {
    throw std::logic_error("Particle filter has no motion model");
  }
  motion_model_->UpdateParticles(particles_, previous_odom_pose, current_odom_pose);
}

void ParticleFilter::SensorUpdate(
  const LaserScanData & scan,
  const LocalizationMap & map,
  const Pose2D & laser_pose_in_base)
{
  if (!initialized_) {
    throw std::logic_error("Particle filter is not initialized");
  }
  if (!laser_model_) {
    throw std::logic_error("Particle filter has no laser model");
  }
  laser_model_->UpdateWeights(particles_, scan, map, laser_pose_in_base);
}

void ParticleFilter::SetWeights(const std::vector<double> & weights)
{
  if (weights.size() != particles_.size()) {
    throw std::invalid_argument("Weight count must match particle count");
  }
  for (double weight : weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      throw std::invalid_argument("Particle weights must be finite and non-negative");
    }
  }
  for (std::size_t index = 0; index < particles_.size(); ++index) {
    particles_[index].weight = weights[index];
  }
}

bool ParticleFilter::NormalizeWeights()
{
  if (particles_.empty()) {
    return false;
  }
  const double total = std::accumulate(
    particles_.begin(), particles_.end(), 0.0,
    [](double sum, const Particle & particle) { return sum + particle.weight; });
  if (!std::isfinite(total) || total <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  for (auto & particle : particles_) {
    particle.weight /= total;
  }
  return true;
}

bool ParticleFilter::Resample()
{
  if (!initialized_ || particles_.empty() || !NormalizeWeights()) {
    return false;
  }

  kd_tree_.Build(particles_);
  const std::size_t target_count = GetTargetParticleCount();
  std::vector<Particle> resampled(target_count);
  std::uniform_real_distribution<double> offset_distribution(
    0.0, 1.0 / static_cast<double>(target_count));
  const double offset = offset_distribution(generator_);
  std::size_t source_index = 0;
  double cumulative = particles_[0].weight;

  for (std::size_t index = 0; index < target_count; ++index) {
    const double sample = offset + static_cast<double>(index) / target_count;
    while (sample > cumulative && source_index + 1U < particles_.size()) {
      ++source_index;
      cumulative += particles_[source_index].weight;
    }
    resampled[index].pose = particles_[source_index].pose;
    resampled[index].weight = 1.0 / static_cast<double>(target_count);
  }

  particles_.swap(resampled);
  kd_tree_.Build(particles_);
  return true;
}

PoseEstimate ParticleFilter::Estimate() const
{
  PoseEstimate estimate;
  if (!initialized_ || particles_.empty()) {
    return estimate;
  }

  std::vector<std::size_t> indices(particles_.size());
  std::iota(indices.begin(), indices.end(), 0U);

  double total_weight = 0.0;
  double mean_x = 0.0;
  double mean_y = 0.0;
  double mean_sin = 0.0;
  double mean_cos = 0.0;
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
  estimate.pose.yaw = std::atan2(mean_sin, mean_cos);

  for (const std::size_t index : indices) {
    const auto & particle = particles_[index];
    const double dx = particle.pose.x - estimate.pose.x;
    const double dy = particle.pose.y - estimate.pose.y;
    const double dyaw = AngularDistance(particle.pose.yaw, estimate.pose.yaw);
    const double normalized_weight = particle.weight / total_weight;
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

bool ParticleFilter::IsInitialized() const
{
  return initialized_;
}

const std::vector<Particle> & ParticleFilter::GetParticles() const
{
  return particles_;
}

void ParticleFilter::ValidateParticleCount(std::size_t count) const
{
  if (count == 0U) {
    throw std::invalid_argument("Particle count must be greater than zero");
  }
}

bool ParticleFilter::CholeskyDecompose(const Covariance3 & covariance, double lower[3][3])
{
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      const double value = covariance.At(row, column);
      if (!std::isfinite(value)) {
        return false;
      }
      if (std::abs(value - covariance.At(column, row)) > 1.0e-9) {
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
        if (value < -1.0e-10) {
          return false;
        }
        lower[row][column] = std::sqrt(std::max(0.0, value));
      } else {
        if (lower[column][column] <= std::numeric_limits<double>::epsilon()) {
          if (std::abs(value) > 1.0e-10) {
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

double ParticleFilter::NormalQuantile(double probability)
{
  // Acklam's approximation is sufficient for the bounded KLD parameter range.
  if (probability <= 0.0) {
    return -8.0;
  }
  if (probability >= 1.0) {
    return 8.0;
  }
  const double a1 = -39.6968302866538;
  const double a2 = 220.946098424521;
  const double a3 = -275.928510446969;
  const double a4 = 138.357751867269;
  const double a5 = -30.6647980661472;
  const double a6 = 2.50662827745924;
  const double b1 = -54.4760987982241;
  const double b2 = 161.585836858041;
  const double b3 = -155.698979859887;
  const double b4 = 66.8013118877197;
  const double b5 = -13.2806815528857;
  const double c1 = -0.00778489400243029;
  const double c2 = -0.322396458041136;
  const double c3 = -2.40075827716184;
  const double c4 = -2.54973253934373;
  const double c5 = 4.37466414146497;
  const double c6 = 2.93816398269878;
  const double d1 = 0.00778469570904146;
  const double d2 = 0.32246712907004;
  const double d3 = 2.445134137143;
  const double d4 = 3.75440866190742;
  const double lower = 0.02425;
  const double upper = 1.0 - lower;
  if (probability < lower) {
    const double q = std::sqrt(-2.0 * std::log(probability));
    return (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
      ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }
  if (probability > upper) {
    const double q = std::sqrt(-2.0 * std::log(1.0 - probability));
    return -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
      ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }
  const double q = probability - 0.5;
  const double r = q * q;
  return (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) * q /
    (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r + 1.0);
}

std::size_t ParticleFilter::GetTargetParticleCount() const
{
  if (options_.min_particles == options_.max_particles) {
    return options_.min_particles;
  }

  const std::size_t bins = std::max<std::size_t>(2U, kd_tree_.GetOccupiedBinCount());
  const double z = options_.pf_z > 1.0 ? options_.pf_z : NormalQuantile(options_.pf_z);
  const double degrees = static_cast<double>(bins - 1U);
  const double estimate = degrees / (2.0 * options_.pf_err) *
    std::pow(1.0 - 2.0 / (9.0 * degrees) + z * std::sqrt(2.0 / (9.0 * degrees)), 3.0);
  const auto count = static_cast<std::size_t>(std::ceil(std::max(1.0, estimate)));
  return std::min(options_.max_particles, std::max(options_.min_particles, count));
}

}  // namespace mini_nav_core::localization

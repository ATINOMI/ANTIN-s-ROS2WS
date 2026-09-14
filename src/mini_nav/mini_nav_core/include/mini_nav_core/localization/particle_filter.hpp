#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "mini_nav_core/localization/kd_tree.hpp"
#include "mini_nav_core/localization/localization_constants.hpp"
#include "mini_nav_core/localization/laser_model.hpp"
#include "mini_nav_core/localization/motion_model.hpp"

namespace mini_nav_core::localization
{

struct ParticleFilterOptions
{
  std::size_t min_particles{kDefaultMinParticles};
  std::size_t max_particles{kDefaultMaxParticles};
  double pf_err{kDefaultKldError};
  double kld_normal_quantile{kDefaultKldNormalQuantile};
  double recovery_alpha_fast{0.0};
  double recovery_alpha_slow{0.0};
};

class ParticleFilter
{
public:
  explicit ParticleFilter(std::size_t particle_count, std::uint64_t seed = kDefaultRandomSeed);

  ParticleFilter(
    std::unique_ptr<MotionModel> motion_model,
    std::unique_ptr<LaserModel> laser_model,
    ParticleFilterOptions options = {},
    std::uint64_t seed = kDefaultRandomSeed);

  void Initialize(const Pose2D & pose, const Covariance3 & covariance);
  void InitializeLocalized(const Pose2D & pose, const Covariance3 & covariance);
  void InitializeGlobal(const LocalizationMap & map);

  void MotionUpdate(const Pose2D & previous_odom_pose, const Pose2D & current_odom_pose);
  void SensorUpdate(
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & base_to_laser_pose);

  void SetWeights(const std::vector<double> & weights);
  bool NormalizeWeights();
  bool Resample();
  PoseEstimate Estimate() const;

  bool IsInitialized() const;
  const std::vector<Particle> & GetParticles() const;

private:
  void ValidateParticleCount(std::size_t count) const;
  static bool CholeskyDecompose(const Covariance3 & covariance, double lower[3][3]);
  static double NormalQuantile(double probability);
  std::size_t GetTargetParticleCount() const;

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

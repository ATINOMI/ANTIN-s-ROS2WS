#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "mini_nav_core/localization/kd_tree.hpp"
#include "mini_nav_core/localization/laser_model.hpp"
#include "mini_nav_core/localization/motion_model.hpp"

namespace mini_nav_core::localization
{

struct ParticleFilterOptions
{
  std::size_t min_particles{500};
  std::size_t max_particles{2000};
  double pf_err{0.05};
  double pf_z{2.33};
  double recovery_alpha_fast{0.0};
  double recovery_alpha_slow{0.0};
};

class ParticleFilter
{
public:
  explicit ParticleFilter(std::size_t particle_count, std::uint64_t seed = 1);

  ParticleFilter(
    std::unique_ptr<MotionModel> motion_model,
    std::unique_ptr<LaserModel> laser_model,
    ParticleFilterOptions options = {},
    std::uint64_t seed = 1);

  void Initialize(const Pose2D & pose, const Covariance3 & covariance);
  void InitializeLocalized(const Pose2D & pose, const Covariance3 & covariance);
  void InitializeGlobal(const LocalizationMap & map);

  void MotionUpdate(const Pose2D & previous_odom_pose, const Pose2D & current_odom_pose);
  void SensorUpdate(
    const LaserScanData & scan,
    const LocalizationMap & map,
    const Pose2D & laser_pose_in_base);

  void SetWeights(const std::vector<double> & weights);
  bool NormalizeWeights();
  bool Resample();
  PoseEstimate Estimate() const;

  bool IsInitialized() const;
  const std::vector<Particle> & GetParticles() const;

private:
  void ValidateParticleCount(std::size_t count) const;
  void InitializeWithSamples(const std::vector<Pose2D> & poses);
  static bool CholeskyDecompose(const Covariance3 & covariance, double lower[3][3]);
  static double NormalQuantile(double probability);
  std::size_t GetTargetParticleCount() const;

  std::vector<Particle> particles_;
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<LaserModel> laser_model_;
  ParticleFilterOptions options_;
  std::mt19937_64 generator_;
  KdTree kd_tree_;
  bool initialized_{false};
  double w_fast_{0.0};
  double w_slow_{0.0};
};

}  // namespace mini_nav_core::localization

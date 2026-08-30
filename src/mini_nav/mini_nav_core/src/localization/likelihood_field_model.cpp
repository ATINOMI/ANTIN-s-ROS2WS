#include "mini_nav_core/localization/likelihood_field_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mini_nav_core::localization
{

namespace
{
Pose2D ComposePose(const Pose2D & base, const Pose2D & relative)
{
  const double cosine = std::cos(base.yaw);
  const double sine = std::sin(base.yaw);
  return Pose2D{
    base.x + cosine * relative.x - sine * relative.y,
    base.y + sine * relative.x + cosine * relative.y,
    NormalizeAngle(base.yaw + relative.yaw)};
}
}  // namespace

LikelihoodFieldModel::LikelihoodFieldModel(
  double z_hit, double z_rand, double sigma_hit, std::size_t max_beams)
: z_hit_(z_hit), z_rand_(z_rand), sigma_hit_(sigma_hit), max_beams_(max_beams)
{
  if (!std::isfinite(z_hit_) || !std::isfinite(z_rand_) || z_hit_ < 0.0 || z_rand_ < 0.0 ||
      z_hit_ + z_rand_ <= 0.0) {
    throw std::invalid_argument("Laser mixture weights must be finite and non-negative");
  }
  if (!std::isfinite(sigma_hit_) || sigma_hit_ <= 0.0 || max_beams_ == 0U) {
    throw std::invalid_argument("Invalid likelihood-field parameters");
  }
}

void LikelihoodFieldModel::UpdateWeights(
  std::vector<Particle> & particles,
  const LaserScanData & scan,
  const LocalizationMap & map,
  const Pose2D & laser_pose_in_base) const
{
  if (particles.empty() || scan.ranges.empty() || scan.range_max <= 0.0 ||
      !std::isfinite(scan.range_max)) {
    return;
  }

  const std::size_t step = std::max<std::size_t>(
    1U, (scan.ranges.size() - 1U) / std::max<std::size_t>(1U, max_beams_ - 1U));
  const double hit_denominator = 2.0 * sigma_hit_ * sigma_hit_;
  const double random_probability = z_rand_ / scan.range_max;

  for (auto & particle : particles) {
    const Pose2D laser_pose = ComposePose(particle.pose, laser_pose_in_base);
    double log_weight = std::log(std::max(particle.weight, std::numeric_limits<double>::min()));
    bool used_beam = false;

    for (std::size_t index = 0; index < scan.ranges.size(); index += step) {
      const double observed_range = scan.ranges[index];
      if (!std::isfinite(observed_range) || observed_range < scan.range_min ||
          observed_range >= scan.range_max) {
        continue;
      }

      const double bearing = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const double hit_x = laser_pose.x + observed_range * std::cos(laser_pose.yaw + bearing);
      const double hit_y = laser_pose.y + observed_range * std::sin(laser_pose.yaw + bearing);
      const double distance = map.GetObstacleDistanceAtWorld(hit_x, hit_y);
      const double hit_probability = z_hit_ * std::exp(-(distance * distance) / hit_denominator);
      const double probability = std::max(hit_probability + random_probability, 1.0e-12);
      log_weight += std::log(probability);
      used_beam = true;
    }

    if (used_beam) {
      particle.weight = std::exp(std::max(-745.0, std::min(709.0, log_weight)));
    }
  }
}

}  // namespace mini_nav_core::localization

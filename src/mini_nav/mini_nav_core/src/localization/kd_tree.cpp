#include "mini_nav_core/localization/kd_tree.hpp"

#include <cmath>
#include <stdexcept>

namespace mini_nav_core::localization
{

KdTree::KdTree(double linear_bin_size, double angular_bin_size)
: linear_bin_size_(linear_bin_size), angular_bin_size_(angular_bin_size)
{
  if (!std::isfinite(linear_bin_size_) || linear_bin_size_ <= 0.0 ||
      !std::isfinite(angular_bin_size_) || angular_bin_size_ <= 0.0) {
    throw std::invalid_argument("KdTree bin sizes must be finite and greater than zero");
  }
}

void KdTree::Build(const std::vector<Particle> & particles)
{
  bin_ids_.clear();
  particle_bins_.clear();
  bin_particles_.clear();
  particle_bins_.reserve(particles.size());

  for (std::size_t index = 0; index < particles.size(); ++index) {
    const Key key = MakeKey(particles[index].pose);
    const auto [iterator, inserted] = bin_ids_.emplace(key, bin_particles_.size());
    if (inserted) {
      bin_particles_.emplace_back();
    }
    particle_bins_.push_back(iterator->second);
    bin_particles_[iterator->second].push_back(index);
  }
}

std::size_t KdTree::GetOccupiedBinCount() const
{
  return bin_particles_.size();
}

std::size_t KdTree::KeyHash::operator()(const Key & key) const
{
  std::size_t hash = std::hash<int>{}(key.x);
  hash ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
  hash ^= std::hash<int>{}(key.yaw) + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
  return hash;
}

KdTree::Key KdTree::MakeKey(const Pose2D & pose) const
{
  return Key{
    static_cast<int>(std::floor(pose.x / linear_bin_size_)),
    static_cast<int>(std::floor(pose.y / linear_bin_size_)),
    static_cast<int>(std::floor((NormalizeAngle(pose.yaw) + 3.14159265358979323846) /
                                angular_bin_size_))};
}

}  // namespace mini_nav_core::localization

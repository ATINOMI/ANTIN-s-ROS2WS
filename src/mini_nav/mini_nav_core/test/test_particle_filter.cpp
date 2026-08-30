/* Includes ----------------------------------------------------------------*/
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mini_nav_core/localization/particle_filter.hpp"

/* Type aliases ------------------------------------------------------------*/
using mini_nav_core::localization::Covariance3;
using mini_nav_core::localization::ParticleFilter;
using mini_nav_core::localization::Pose2D;

namespace
{
double AngularDistance(double from, double to)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  double difference = std::fmod(from - to + kPi, kTwoPi);
  if (difference < 0.0) {
    difference += kTwoPi;
  }
  return difference - kPi;
}
}  // namespace

/* Test cases --------------------------------------------------------------*/
TEST(ParticleFilter, RejectsZeroParticles)
{
  EXPECT_THROW(ParticleFilter(0, 1), std::invalid_argument);
}

TEST(ParticleFilter, InitializesParticlesAtPoseWithZeroCovariance)
{
  ParticleFilter filter(4, 7);
  const Pose2D initial_pose{1.2, -0.7, 3.5};
  const Covariance3 covariance = Covariance3::Diagonal(0.0, 0.0, 0.0);

  filter.Initialize(initial_pose, covariance);

  ASSERT_TRUE(filter.IsInitialized());
  ASSERT_EQ(filter.GetParticles().size(), 4U);

  const double expected_yaw = std::atan2(std::sin(initial_pose.yaw),
                                         std::cos(initial_pose.yaw));
  for (const auto & particle : filter.GetParticles()) {
    EXPECT_DOUBLE_EQ(particle.pose.x, initial_pose.x);
    EXPECT_DOUBLE_EQ(particle.pose.y, initial_pose.y);
    EXPECT_NEAR(particle.pose.yaw, expected_yaw, 1.0e-12);
    EXPECT_DOUBLE_EQ(particle.weight, 0.25);
  }
}

TEST(ParticleFilter, GaussianInitializationProducesEqualWeightedSpread)
{
  ParticleFilter filter(2000, 11);
  filter.Initialize(
    Pose2D{0.0, 0.0, 0.0},
    Covariance3::Diagonal(1.0, 1.0, 0.01));

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  for (const auto & particle : filter.GetParticles()) {
    min_x = std::min(min_x, particle.pose.x);
    max_x = std::max(max_x, particle.pose.x);
    EXPECT_DOUBLE_EQ(particle.weight, 1.0 / 2000.0);
  }

  EXPECT_LT(min_x, -1.0);
  EXPECT_GT(max_x, 1.0);
}

TEST(ParticleFilter, RejectsInvalidWeights)
{
  ParticleFilter filter(3, 1);
  filter.Initialize(Pose2D{}, Covariance3::Diagonal(0.0, 0.0, 0.0));

  EXPECT_THROW(filter.SetWeights({1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW(
    filter.SetWeights({1.0, -1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(
    filter.SetWeights({1.0, std::numeric_limits<double>::quiet_NaN(), 1.0}),
    std::invalid_argument);
}

TEST(ParticleFilter, NormalizesWeights)
{
  ParticleFilter filter(3, 1);
  filter.Initialize(Pose2D{}, Covariance3::Diagonal(0.0, 0.0, 0.0));
  filter.SetWeights({1.0, 2.0, 3.0});

  ASSERT_TRUE(filter.NormalizeWeights());
  ASSERT_EQ(filter.GetParticles().size(), 3U);
  EXPECT_DOUBLE_EQ(filter.GetParticles()[0].weight, 1.0 / 6.0);
  EXPECT_DOUBLE_EQ(filter.GetParticles()[1].weight, 2.0 / 6.0);
  EXPECT_DOUBLE_EQ(filter.GetParticles()[2].weight, 3.0 / 6.0);
}

TEST(ParticleFilter, RejectsAllZeroWeights)
{
  ParticleFilter filter(3, 1);
  filter.Initialize(Pose2D{}, Covariance3::Diagonal(0.0, 0.0, 0.0));
  filter.SetWeights({0.0, 0.0, 0.0});

  EXPECT_FALSE(filter.NormalizeWeights());
  EXPECT_FALSE(filter.Resample());
}

TEST(ParticleFilter, ResamplingFavorsHighWeightParticles)
{
  ParticleFilter filter(1000, 23);
  filter.Initialize(
    Pose2D{0.0, 0.0, 0.0},
    Covariance3::Diagonal(1.0, 0.0, 0.0));

  std::vector<double> weights;
  weights.reserve(filter.GetParticles().size());
  for (const auto & particle : filter.GetParticles()) {
    weights.push_back(particle.pose.x > 0.0 ? 100.0 : 1.0);
  }
  filter.SetWeights(weights);

  ASSERT_TRUE(filter.Resample());

  std::size_t positive_count = 0;
  for (const auto & particle : filter.GetParticles()) {
    positive_count += particle.pose.x > 0.0 ? 1U : 0U;
    EXPECT_DOUBLE_EQ(particle.weight, 1.0 / 1000.0);
  }
  EXPECT_GT(positive_count, 900U);
}

TEST(ParticleFilter, ComputesEstimateWithCircularYawMean)
{
  constexpr double kPi = 3.14159265358979323846;
  ParticleFilter filter(2000, 31);
  filter.Initialize(
    Pose2D{2.0, -1.0, kPi},
    Covariance3::Diagonal(0.01, 0.01, 0.01));

  const auto estimate = filter.Estimate();

  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.pose.x, 2.0, 0.05);
  EXPECT_NEAR(estimate.pose.y, -1.0, 0.05);
  EXPECT_LT(std::abs(AngularDistance(estimate.pose.yaw, kPi)), 0.05);
  EXPECT_GT(estimate.covariance.At(0, 0), 0.0);
  EXPECT_GT(estimate.covariance.At(1, 1), 0.0);
  EXPECT_GT(estimate.covariance.At(2, 2), 0.0);
}

TEST(ParticleFilter, EstimateIsInvalidBeforeInitialization)
{
  ParticleFilter filter(3, 1);

  EXPECT_FALSE(filter.Estimate().valid);
}

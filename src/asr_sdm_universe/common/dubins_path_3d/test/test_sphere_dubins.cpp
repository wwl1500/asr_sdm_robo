// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "dubins_path_3d/math_utils.hpp"
#include "dubins_path_3d/sphere_dubins.hpp"

namespace
{

using dubins_path_3d::Mat3;
using dubins_path_3d::Vec3;
using dubins_path_3d::sphere::Config;

constexpr double kTwoPi = 2.0 * M_PI;

/// Random configuration on a sphere of the given radius centred on the origin.
Config randomConfig(std::mt19937 & generator, double radius)
{
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const double colatitude = uniform(generator) * M_PI;
  const double longitude = uniform(generator) * kTwoPi;

  const Vec3 position(
    radius * std::sin(colatitude) * std::cos(longitude),
    radius * std::sin(colatitude) * std::sin(longitude),
    radius * std::cos(colatitude));

  Vec3 candidate(uniform(generator), uniform(generator), uniform(generator));
  const Vec3 radial = position / radius;
  Vec3 tangent = candidate - candidate.dot(radial) * radial;
  while (tangent.norm() < 1e-6) {
    candidate = Vec3(uniform(generator), uniform(generator), uniform(generator));
    tangent = candidate - candidate.dot(radial) * radial;
  }
  tangent.normalize();

  return dubins_path_3d::sphere::makeConfig(position, Vec3::Zero(), tangent);
}

}  // namespace

TEST(SphereDubins, SegmentOperatorsStayOnTheSphere)
{
  const double sphere_radius = 40.0;
  const double turn_radius = 31.0;

  std::mt19937 generator(4242U);
  const Config start = randomConfig(generator, sphere_radius);

  for (const char segment : {'l', 'r', 'g'}) {
    for (double phi = 0.0; phi < kTwoPi; phi += 0.37) {
      const Config reached = dubins_path_3d::sphere::applySegment(
        start, phi, turn_radius, sphere_radius, segment);

      EXPECT_NEAR(reached.col(0).norm(), sphere_radius, 1e-9) << "segment " << segment;
      EXPECT_NEAR(reached.col(1).norm(), 1.0, 1e-9) << "segment " << segment;
      EXPECT_NEAR(reached.col(2).norm(), 1.0, 1e-9) << "segment " << segment;
      // The tangent must stay in the tangent plane of the sphere.
      EXPECT_NEAR(reached.col(0).dot(reached.col(1)), 0.0, 1e-7) << "segment " << segment;
    }
  }
}

TEST(SphereDubins, GreatCircleLengthMatchesCentralAngle)
{
  const double sphere_radius = 10.0;
  const Config start = dubins_path_3d::sphere::makeConfig(
    Vec3(sphere_radius, 0.0, 0.0), Vec3::Zero(), Vec3::UnitY());

  const double phi = 0.8;
  const Config reached =
    dubins_path_3d::sphere::applySegment(start, phi, 5.0, sphere_radius, 'g');

  const double central_angle = std::acos(
    std::clamp(
      start.col(0).normalized().dot(reached.col(0).normalized()), -1.0, 1.0));
  EXPECT_NEAR(central_angle, phi, 1e-9);
}

TEST(SphereDubins, EveryCandidateReachesTheGoal)
{
  // r/R above 1/sqrt(2) activates all four path families.
  const double sphere_radius = 40.0;
  const double turn_radius = 31.0;

  std::mt19937 generator(24680U);
  int feasible_count = 0;

  for (int trial = 0; trial < 60; ++trial) {
    const Config start = randomConfig(generator, sphere_radius);
    const Config goal = randomConfig(generator, sphere_radius);

    for (const auto & solution :
      dubins_path_3d::sphere::allPaths(start, goal, turn_radius, sphere_radius))
    {
      ASSERT_TRUE(solution.valid());
      ++feasible_count;

      const Config reached = dubins_path_3d::sphere::finalConfig(
        start, turn_radius, sphere_radius, solution.angles, solution.type);
      EXPECT_LT((reached - goal).cwiseAbs().maxCoeff(), 1e-2 * sphere_radius)
        << "type " << solution.type;
    }
  }

  EXPECT_GT(feasible_count, 60) << "expected at least one feasible path per trial";
}

TEST(SphereDubins, LengthMatchesSumOfArcs)
{
  const double sphere_radius = 40.0;
  const double turn_radius = 31.0;

  std::mt19937 generator(11223344U);
  for (int trial = 0; trial < 40; ++trial) {
    const Config start = randomConfig(generator, sphere_radius);
    const Config goal = randomConfig(generator, sphere_radius);

    for (const auto & solution :
      dubins_path_3d::sphere::allPaths(start, goal, turn_radius, sphere_radius))
    {
      double expected = 0.0;
      for (std::size_t i = 0; i < solution.type.size(); ++i) {
        expected += (solution.type[i] == 'g') ?
          solution.angles[i] * sphere_radius :
          solution.angles[i] * turn_radius;
      }
      EXPECT_NEAR(solution.length, expected, 1e-6) << "type " << solution.type;
    }
  }
}

TEST(SphereDubins, OptimalPathIsFoundForEveryPair)
{
  const double sphere_radius = 50.0;
  const double turn_radius = 31.22;

  std::mt19937 generator(55555U);
  for (int trial = 0; trial < 100; ++trial) {
    const Config start = randomConfig(generator, sphere_radius);
    const Config goal = randomConfig(generator, sphere_radius);

    const auto solution =
      dubins_path_3d::sphere::optimalPath(start, goal, turn_radius, sphere_radius);
    ASSERT_TRUE(solution.valid()) << "no path found on trial " << trial;

    EXPECT_NEAR(
      dubins_path_3d::sphere::optimalPathLength(start, goal, turn_radius, sphere_radius),
      solution.length, 1e-12);

    // A path on the sphere cannot be shorter than the great-circle distance.
    const double central_angle = std::acos(
      std::clamp(
        start.col(0).normalized().dot(goal.col(0).normalized()), -1.0, 1.0));
    EXPECT_GE(solution.length, central_angle * sphere_radius - 1e-6);
  }
}

TEST(SphereDubins, SamplesStartAtTheInitialConfiguration)
{
  const double sphere_radius = 40.0;
  const double turn_radius = 31.0;

  std::mt19937 generator(777U);
  const Config start = randomConfig(generator, sphere_radius);
  const Config goal = randomConfig(generator, sphere_radius);

  const auto solution =
    dubins_path_3d::sphere::optimalPath(start, goal, turn_radius, sphere_radius);
  ASSERT_TRUE(solution.valid());

  const auto samples = dubins_path_3d::sphere::pathSamples(
    start, turn_radius, sphere_radius, solution.angles, solution.type, 1.0);
  ASSERT_FALSE(samples.positions.empty());

  EXPECT_LT((samples.positions.front() - start.col(0)).norm(), 1e-9);
  EXPECT_LT((samples.tangents.front() - start.col(1)).norm(), 1e-9);

  for (const Vec3 & position : samples.positions) {
    EXPECT_NEAR(position.norm(), sphere_radius, 1e-6);
  }
  for (std::size_t i = 1; i < samples.positions.size(); ++i) {
    EXPECT_LT((samples.positions[i] - samples.positions[i - 1]).norm(), 2.0);
  }
}

TEST(SphereDubins, SmallRadiusRatioSkipsTheAbnormalFamilies)
{
  // r/R below 1/2 leaves only the six three-segment families.
  const double sphere_radius = 100.0;
  const double turn_radius = 20.0;

  std::mt19937 generator(31415U);
  for (int trial = 0; trial < 20; ++trial) {
    const Config start = randomConfig(generator, sphere_radius);
    const Config goal = randomConfig(generator, sphere_radius);

    for (const auto & solution :
      dubins_path_3d::sphere::allPaths(start, goal, turn_radius, sphere_radius))
    {
      EXPECT_LE(solution.type.size(), 3U) << "unexpected type " << solution.type;
    }
  }
}

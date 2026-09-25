// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dubins_path_3d/planner.hpp"

namespace
{

using dubins_path_3d::Configuration;
using dubins_path_3d::DubinsPath3D;
using dubins_path_3d::PlannerOptions;
using dubins_path_3d::Vec3;

/// Options matching the reference Python example, but with a coarse sweep so the
/// tests stay fast.
PlannerOptions testOptions()
{
  PlannerOptions options;
  options.pitch_radius = 40.0;
  options.yaw_radius = 50.0;
  options.location_samples = 5;
  options.heading_samples = 5;
  options.sample_spacing = 2.0;
  options.num_threads = 2;
  return options;
}

/// Largest turn the vehicle can make anywhere on the reachable surfaces; the
/// curvature of a feasible path never exceeds this.
double maxCurvature(const PlannerOptions & options)
{
  return 1.0 / options.resolvedTightTurnRadius();
}

}  // namespace

TEST(PlannerOptionsTest, TightTurnRadiusCombinesBothRates)
{
  PlannerOptions options = testOptions();
  options.tight_turn_radius = -1.0;

  const double expected = 1.0 / std::sqrt(
    1.0 / (options.pitch_radius * options.pitch_radius) +
    1.0 / (options.yaw_radius * options.yaw_radius));
  EXPECT_NEAR(options.resolvedTightTurnRadius(), expected, 1e-12);

  // The combined turn is tighter than either individual one.
  EXPECT_LT(options.resolvedTightTurnRadius(), options.pitch_radius);
  EXPECT_LT(options.resolvedTightTurnRadius(), options.yaw_radius);

  options.tight_turn_radius = 10.0;
  EXPECT_NEAR(options.resolvedTightTurnRadius(), 10.0, 1e-12);
}

TEST(PlannerOptionsTest, InconsistentOptionsAreRejected)
{
  PlannerOptions options = testOptions();

  options.pitch_radius = 0.0;
  EXPECT_THROW(options.validate(), std::invalid_argument);

  options = testOptions();
  options.sample_spacing = -1.0;
  EXPECT_THROW(options.validate(), std::invalid_argument);

  options = testOptions();
  options.location_samples = 0;
  EXPECT_THROW(options.validate(), std::invalid_argument);

  options = testOptions();
  // A tight-turn radius larger than the sphere it lives on is not realisable.
  options.tight_turn_radius = 1000.0;
  EXPECT_THROW(options.validate(), std::invalid_argument);

  EXPECT_THROW(DubinsPath3D{options}, std::invalid_argument);
}

TEST(ConfigurationTest, EulerRoundTrip)
{
  const Vec3 position(1.0, -2.0, 3.0);
  for (const double heading : {-2.0, 0.0, 0.7, 3.0}) {
    for (const double pitch : {-1.2, 0.0, 0.5}) {
      for (const double roll : {-2.5, 0.0, 1.1}) {
        const Configuration config =
          dubins_path_3d::configurationFromEuler(position, heading, pitch, roll);
        ASSERT_TRUE(config.isOrthonormal());

        double recovered_heading = 0.0;
        double recovered_pitch = 0.0;
        double recovered_roll = 0.0;
        dubins_path_3d::eulerFromConfiguration(
          config, recovered_heading, recovered_pitch, recovered_roll);

        const Configuration rebuilt = dubins_path_3d::configurationFromEuler(
          position, recovered_heading, recovered_pitch, recovered_roll);
        EXPECT_LT((rebuilt.rotation() - config.rotation()).cwiseAbs().maxCoeff(), 1e-9)
          << "heading " << heading << " pitch " << pitch << " roll " << roll;
      }
    }
  }
}

TEST(ConfigurationTest, NormalizeRepairsASkewedFrame)
{
  Configuration config;
  config.tangent = Vec3(2.0, 0.0, 0.0);
  config.tangent_normal = Vec3(0.3, 1.0, 0.0);
  config.surface_normal = Vec3(0.0, 0.1, 4.0);

  config.normalize();
  EXPECT_TRUE(config.isOrthonormal());
  // The tangent direction is preserved; only the other two axes move.
  EXPECT_LT((config.tangent - Vec3::UnitX()).norm(), 1e-12);
}

TEST(ConfigurationTest, DegenerateFrameIsRejected)
{
  Configuration config;
  config.tangent = Vec3::Zero();
  EXPECT_THROW(config.normalize(), std::invalid_argument);

  Configuration parallel;
  parallel.tangent = Vec3::UnitX();
  parallel.tangent_normal = Vec3::UnitX();
  parallel.surface_normal = Vec3::UnitX();
  EXPECT_THROW(parallel.normalize(), std::invalid_argument);
}

TEST(PlannerTest, ReferenceExampleIsSolved)
{
  // Same endpoints as the reference Python main script.
  const Configuration start =
    dubins_path_3d::configurationFromEuler(Vec3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
  const Configuration goal = dubins_path_3d::configurationFromEuler(
    Vec3(100.0, 50.0, 40.0), 0.5 * M_PI, 0.25 * M_PI, 0.0);

  const PlannerOptions options = testOptions();
  const auto result = DubinsPath3D(options).plan(start, goal);

  ASSERT_TRUE(result.success());
  EXPECT_FALSE(result.best.type.empty());
  EXPECT_GT(result.best.length, (goal.position - start.position).norm() - 1e-6);
  EXPECT_GE(result.candidates.size(), 3U);

  // The reported length is the best among the candidates.
  for (const auto & [name, length] : result.candidates) {
    EXPECT_GE(length, result.best.length - 1e-6) << "candidate " << name;
  }

  // Endpoints are honoured.
  const auto & samples = result.best.samples;
  ASSERT_GE(samples.size(), 2U);
  EXPECT_LT((samples.front().position - start.position).norm(), 1e-6);
  EXPECT_LT((samples.front().tangent - start.tangent).norm(), 1e-6);
  EXPECT_LT((samples.back().position - goal.position).norm(), options.sample_spacing);
  EXPECT_LT((samples.back().tangent - goal.tangent).norm(), 1e-2);
}

TEST(PlannerTest, SamplesRespectTheCurvatureBound)
{
  const Configuration start =
    dubins_path_3d::configurationFromEuler(Vec3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
  const Configuration goal = dubins_path_3d::configurationFromEuler(
    Vec3(120.0, -60.0, 30.0), -0.4 * M_PI, 0.1 * M_PI, 0.0);

  const PlannerOptions options = testOptions();
  const auto result = DubinsPath3D(options).plan(start, goal);
  ASSERT_TRUE(result.success());

  const auto & samples = result.best.samples;
  const double limit = maxCurvature(options);

  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double step = (samples[i].position - samples[i - 1].position).norm();
    if (step < 1e-9) {
      continue;
    }
    EXPECT_NEAR(samples[i].tangent.norm(), 1.0, 1e-6);

    // Discrete curvature: the turn of the tangent per unit arc length. A 20%
    // margin absorbs the error of the finite-difference estimate at junctions
    // between segments.
    const double turn = std::acos(
      std::clamp(samples[i].tangent.dot(samples[i - 1].tangent), -1.0, 1.0));
    EXPECT_LT(turn / step, 1.2 * limit) << "between samples " << i - 1 << " and " << i;
  }
}

TEST(PlannerTest, IdenticalEndpointsGiveAShortPath)
{
  const Configuration start =
    dubins_path_3d::configurationFromEuler(Vec3(10.0, 10.0, 10.0), 0.3, 0.1, 0.0);

  const auto result = DubinsPath3D(testOptions()).plan(start, start);
  ASSERT_TRUE(result.success());
  EXPECT_LT((result.best.samples.front().position - start.position).norm(), 1e-6);
}

TEST(PlannerTest, RefiningTheSweepDoesNotLengthenThePath)
{
  const Configuration start =
    dubins_path_3d::configurationFromEuler(Vec3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
  const Configuration goal = dubins_path_3d::configurationFromEuler(
    Vec3(200.0, 100.0, -50.0), 0.75 * M_PI, -0.2 * M_PI, 0.0);

  PlannerOptions coarse = testOptions();
  coarse.location_samples = 3;
  coarse.heading_samples = 3;

  PlannerOptions fine = testOptions();
  fine.location_samples = 9;
  fine.heading_samples = 9;

  const auto coarse_result = DubinsPath3D(coarse).plan(start, goal);
  const auto fine_result = DubinsPath3D(fine).plan(start, goal);

  ASSERT_TRUE(coarse_result.success());
  ASSERT_TRUE(fine_result.success());
  EXPECT_LE(fine_result.best.length, coarse_result.best.length + 1e-6);
}

TEST(PlannerTest, ResultIsInvariantUnderRigidMotion)
{
  const Configuration start =
    dubins_path_3d::configurationFromEuler(Vec3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
  const Configuration goal = dubins_path_3d::configurationFromEuler(
    Vec3(90.0, 30.0, 20.0), 0.6 * M_PI, 0.15 * M_PI, 0.0);

  const auto reference = DubinsPath3D(testOptions()).plan(start, goal);
  ASSERT_TRUE(reference.success());

  // Rotating about the z-axis and translating cannot change the length, because
  // the constraint set is defined in the body frame.
  const double yaw = 0.7;
  const Eigen::AngleAxisd rotation(yaw, Vec3::UnitZ());
  const Vec3 offset(-30.0, 15.0, 5.0);

  const auto transform = [&](const Configuration & config) {
      Configuration moved;
      moved.position = rotation * config.position + offset;
      moved.tangent = rotation * config.tangent;
      moved.tangent_normal = rotation * config.tangent_normal;
      moved.surface_normal = rotation * config.surface_normal;
      return moved;
    };

  const auto moved =
    DubinsPath3D(testOptions()).plan(transform(start), transform(goal));
  ASSERT_TRUE(moved.success());
  EXPECT_NEAR(moved.best.length, reference.best.length, 1e-6);
}

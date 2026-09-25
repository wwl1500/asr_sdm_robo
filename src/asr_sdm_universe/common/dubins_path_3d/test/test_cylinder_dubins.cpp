// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "dubins_path_3d/cylinder_dubins.hpp"

namespace
{

using dubins_path_3d::Vec3;

constexpr double kTwoPi = 2.0 * M_PI;

/// Point on a cylinder of the given radius, at angle `theta` and height `z`.
Vec3 surfacePoint(double radius, double theta, double z)
{
  return Vec3(radius * std::cos(theta), radius * std::sin(theta), z);
}

/// Unit tangent on the cylinder surface making angle `heading` with the
/// circumferential direction.
Vec3 surfaceTangent(double theta, double heading)
{
  const Vec3 circumferential(-std::sin(theta), std::cos(theta), 0.0);
  return std::cos(heading) * circumferential + std::sin(heading) * Vec3::UnitZ();
}

}  // namespace

TEST(CylinderDubins, PureHelixAlongTheSurface)
{
  // Start and goal on the same generator line: going straight up is feasible and
  // the geodesic distance is just the height difference.
  const double cylinder_radius = 20.0;
  const double turn_radius = 15.0;

  const Vec3 start_position = surfacePoint(cylinder_radius, 0.0, 0.0);
  const Vec3 start_tangent = Vec3::UnitZ();
  const Vec3 goal_position = surfacePoint(cylinder_radius, 0.0, 30.0);
  const Vec3 goal_tangent = Vec3::UnitZ();

  const double length = dubins_path_3d::cylinder::optimalPathLength(
    start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
    turn_radius);

  ASSERT_TRUE(std::isfinite(length));
  EXPECT_NEAR(length, 30.0, 1e-6);
}

TEST(CylinderDubins, SamplesStayOnTheSurface)
{
  const double cylinder_radius = 25.0;
  const double turn_radius = 18.0;

  const Vec3 start_position = surfacePoint(cylinder_radius, 0.3, -10.0);
  const Vec3 start_tangent = surfaceTangent(0.3, 0.6);
  const Vec3 goal_position = surfacePoint(cylinder_radius, 2.1, 25.0);
  const Vec3 goal_tangent = surfaceTangent(2.1, -0.4);

  const auto result = dubins_path_3d::cylinder::optimalPath(
    start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
    turn_radius, 1.0);

  ASSERT_TRUE(result.valid());
  ASSERT_FALSE(result.positions.empty());
  ASSERT_EQ(result.positions.size(), result.tangents.size());
  ASSERT_EQ(result.positions.size(), result.normals.size());

  for (std::size_t i = 0; i < result.positions.size(); ++i) {
    const Vec3 & position = result.positions[i];
    EXPECT_NEAR(std::hypot(position.x(), position.y()), cylinder_radius, 1e-6);
    EXPECT_NEAR(result.tangents[i].norm(), 1.0, 1e-6);
    EXPECT_NEAR(result.normals[i].norm(), 1.0, 1e-6);
    // The normal points radially outward and the tangent lies in the surface.
    EXPECT_NEAR(result.normals[i].z(), 0.0, 1e-9);
    EXPECT_NEAR(result.tangents[i].dot(result.normals[i]), 0.0, 1e-6);
  }

  EXPECT_LT((result.positions.front() - start_position).norm(), 1e-6);
  EXPECT_LT((result.tangents.front() - start_tangent).norm(), 1e-6);
}

TEST(CylinderDubins, EndsAtTheGoalConfiguration)
{
  const double cylinder_radius = 30.0;
  const double turn_radius = 22.0;

  std::mt19937 generator(864213579U);
  std::uniform_real_distribution<double> angle(0.0, kTwoPi);
  std::uniform_real_distribution<double> height(-40.0, 40.0);
  std::uniform_real_distribution<double> heading(-M_PI, M_PI);

  int feasible_count = 0;
  for (int trial = 0; trial < 60; ++trial) {
    const double start_theta = angle(generator);
    const double goal_theta = angle(generator);

    const Vec3 start_position = surfacePoint(cylinder_radius, start_theta, height(generator));
    const Vec3 start_tangent = surfaceTangent(start_theta, heading(generator));
    const Vec3 goal_position = surfacePoint(cylinder_radius, goal_theta, height(generator));
    const Vec3 goal_tangent = surfaceTangent(goal_theta, heading(generator));

    const auto result = dubins_path_3d::cylinder::optimalPath(
      start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
      turn_radius, 0.5);
    if (!result.valid()) {
      continue;
    }
    ++feasible_count;

    // The samples end within one spacing of the goal, and the last tangent
    // matches the requested one.
    EXPECT_LT((result.positions.back() - goal_position).norm(), 1.0)
      << "trial " << trial << " type " << result.type;
    EXPECT_LT((result.tangents.back() - goal_tangent).norm(), 0.1)
      << "trial " << trial << " type " << result.type;

    // A path on the cylinder is at least as long as the straight-line distance.
    EXPECT_GE(result.length, (goal_position - start_position).norm() - 1e-6);
  }

  EXPECT_GT(feasible_count, 50) << "most random pairs on a cylinder should be connectable";
}

TEST(CylinderDubins, LengthAgreesWithTheGeometryVariant)
{
  const double cylinder_radius = 18.0;
  const double turn_radius = 12.0;

  std::mt19937 generator(1928374650U);
  std::uniform_real_distribution<double> angle(0.0, kTwoPi);
  std::uniform_real_distribution<double> height(-25.0, 25.0);
  std::uniform_real_distribution<double> heading(-M_PI, M_PI);

  for (int trial = 0; trial < 60; ++trial) {
    const double start_theta = angle(generator);
    const double goal_theta = angle(generator);

    const Vec3 start_position = surfacePoint(cylinder_radius, start_theta, height(generator));
    const Vec3 start_tangent = surfaceTangent(start_theta, heading(generator));
    const Vec3 goal_position = surfacePoint(cylinder_radius, goal_theta, height(generator));
    const Vec3 goal_tangent = surfaceTangent(goal_theta, heading(generator));

    const double length = dubins_path_3d::cylinder::optimalPathLength(
      start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
      turn_radius);
    const auto result = dubins_path_3d::cylinder::optimalPath(
      start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
      turn_radius, 1.0);

    EXPECT_EQ(std::isfinite(length), result.valid()) << "trial " << trial;
    if (std::isfinite(length)) {
      EXPECT_NEAR(length, result.length, 1e-9) << "trial " << trial;
    }
  }
}

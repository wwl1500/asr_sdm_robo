// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include "dubins_path_3d/cylinder_dubins.hpp"

#include <algorithm>
#include <cmath>

#include "dubins_path_3d/math_utils.hpp"
#include "dubins_path_3d/planar_dubins.hpp"

namespace dubins_path_3d::cylinder
{

namespace
{

struct Development
{
  /// Goal images on the developed plane; they differ by one turn of the profile.
  planar::Config goal_image_a;
  planar::Config goal_image_b;
  double start_angle{0.0};
};

/// Develops the cylinder onto a plane whose origin is the start configuration.
/// A point at profile angle theta and height z maps to (R * dtheta, dz).
Development develop(
  const Vec3 & start_position, const Vec3 & start_tangent,
  const Vec3 & goal_position, const Vec3 & goal_tangent,
  double cylinder_radius, double & start_heading)
{
  Development development;
  development.start_angle = std::atan2(start_position.y(), start_position.x());
  const double goal_angle = std::atan2(goal_position.y(), goal_position.x());

  const double delta_angle = wrapPi(goal_angle - development.start_angle);
  double angle_a = delta_angle;
  double angle_b = delta_angle;
  if (delta_angle < 0.0) {
    angle_a = delta_angle + 2.0 * M_PI;
  } else if (delta_angle > 0.0) {
    angle_b = delta_angle - 2.0 * M_PI;
  }

  start_heading = std::atan2(
    start_tangent.z(),
    -start_tangent.x() * std::sin(development.start_angle) +
    start_tangent.y() * std::cos(development.start_angle));
  const double goal_heading = std::atan2(
    goal_tangent.z(),
    -goal_tangent.x() * std::sin(goal_angle) + goal_tangent.y() * std::cos(goal_angle));

  const double delta_height = goal_position.z() - start_position.z();
  development.goal_image_a = {cylinder_radius * angle_a, delta_height, goal_heading};
  development.goal_image_b = {cylinder_radius * angle_b, delta_height, goal_heading};
  return development;
}

}  // namespace

double optimalPathLength(
  const Vec3 & start_position, const Vec3 & start_tangent,
  const Vec3 & goal_position, const Vec3 & goal_tangent,
  double cylinder_radius, double turn_radius)
{
  double start_heading = 0.0;
  const Development development = develop(
    start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
    start_heading);

  const planar::Config start{0.0, 0.0, start_heading};
  const double length_a =
    planar::optimalPath(start, development.goal_image_a, turn_radius).length;
  const double length_b =
    planar::optimalPath(start, development.goal_image_b, turn_radius).length;
  return std::min(length_a, length_b);
}

Result optimalPath(
  const Vec3 & start_position, const Vec3 & start_tangent,
  const Vec3 & goal_position, const Vec3 & goal_tangent,
  double cylinder_radius, double turn_radius, double spacing)
{
  double start_heading = 0.0;
  const Development development = develop(
    start_position, start_tangent, goal_position, goal_tangent, cylinder_radius,
    start_heading);

  const planar::Config start{0.0, 0.0, start_heading};
  const planar::Solution solution_a =
    planar::optimalPath(start, development.goal_image_a, turn_radius);
  const planar::Solution solution_b =
    planar::optimalPath(start, development.goal_image_b, turn_radius);

  const planar::Solution & best =
    (solution_a.length <= solution_b.length) ? solution_a : solution_b;

  Result result;
  if (!best.valid()) {
    return result;
  }
  result.length = best.length;
  result.type = best.type;

  const std::vector<planar::Config> planar_samples =
    planar::pathSamples(start, turn_radius, best.params, best.type, spacing);

  result.positions.reserve(planar_samples.size());
  result.tangents.reserve(planar_samples.size());
  result.normals.reserve(planar_samples.size());

  for (const planar::Config & sample : planar_samples) {
    // Wrap the developed plane back onto the cylinder.
    const double angle = development.start_angle + sample.x / cylinder_radius;
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);

    result.positions.emplace_back(
      cylinder_radius * cos_angle, cylinder_radius * sin_angle,
      start_position.z() + sample.y);
    result.tangents.emplace_back(
      -sin_angle * std::cos(sample.heading),
      cos_angle * std::cos(sample.heading),
      std::sin(sample.heading));
    result.normals.emplace_back(cos_angle, sin_angle, 0.0);
  }

  return result;
}

}  // namespace dubins_path_3d::cylinder

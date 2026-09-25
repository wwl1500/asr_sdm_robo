// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__CYLINDER_DUBINS_HPP_
#define DUBINS_PATH_3D__CYLINDER_DUBINS_HPP_

#include <string>
#include <vector>

#include "dubins_path_3d/types.hpp"

namespace dubins_path_3d::cylinder
{

/// Shortest path between two configurations on a right circular cylinder.
///
/// All inputs are expressed in the cylinder body frame: the axis is the z-axis
/// and the profile circle is centred on the origin. The cylinder is developed
/// onto a plane, where the goal has two images (differing by one turn around the
/// profile), and the shorter of the two planar Dubins paths wins.
struct Result
{
  double length{kInfinity};
  std::string type;
  std::vector<Vec3> positions;
  std::vector<Vec3> tangents;
  /// Outward surface normals of the cylinder along the path.
  std::vector<Vec3> normals;

  bool valid() const {return std::isfinite(length);}
};

/// Length only; skips generating the geometry.
double optimalPathLength(
  const Vec3 & start_position, const Vec3 & start_tangent,
  const Vec3 & goal_position, const Vec3 & goal_tangent,
  double cylinder_radius, double turn_radius);

/// Length together with samples along the path.
Result optimalPath(
  const Vec3 & start_position, const Vec3 & start_tangent,
  const Vec3 & goal_position, const Vec3 & goal_tangent,
  double cylinder_radius, double turn_radius, double spacing);

}  // namespace dubins_path_3d::cylinder

#endif  // DUBINS_PATH_3D__CYLINDER_DUBINS_HPP_

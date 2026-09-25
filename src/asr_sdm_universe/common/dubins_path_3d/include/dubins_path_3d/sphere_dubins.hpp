// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__SPHERE_DUBINS_HPP_
#define DUBINS_PATH_3D__SPHERE_DUBINS_HPP_

#include <string>
#include <vector>

#include "dubins_path_3d/types.hpp"

namespace dubins_path_3d::sphere
{

/// Configuration on a sphere, stored as a rotation-like matrix whose columns are
/// (position relative to the sphere centre, tangent, tangent-normal). The first
/// column has norm equal to the sphere radius.
using Config = Mat3;

/// Builds a configuration from a point on the sphere and a tangent direction.
Config makeConfig(const Vec3 & point, const Vec3 & centre, const Vec3 & tangent);

/// Matrix that advances a configuration along one segment.
///
/// `segment` is 'l' (left tight turn), 'r' (right tight turn) or 'g' (great
/// circle). `radius` is the tight-turn radius and `sphere_radius` the radius of
/// the sphere.
Mat3 segmentOperator(double phi, double radius, double sphere_radius, char segment);

/// Applies one segment to a configuration.
Config applySegment(
  const Config & start, double phi, double radius, double sphere_radius, char segment);

/// Configuration reached after following `type` with the given arc angles.
Config finalConfig(
  const Config & start, double radius, double sphere_radius,
  const std::vector<double> & angles, const std::string & type);

/// A candidate path on the sphere.
struct Solution
{
  std::string type;
  double length{kInfinity};
  std::vector<double> angles;

  bool valid() const {return std::isfinite(length) && !angles.empty();}
};

/// Positions (relative to the sphere centre) and tangents along a path.
struct Samples
{
  std::vector<Vec3> positions;
  std::vector<Vec3> tangents;
};

/// Every feasible path of the families that can be optimal for the given
/// radius ratio.
///
/// LGL, RGR, LGR, RGL, LRL and RLR are always considered. When
/// `radius / sphere_radius` exceeds 1/2 the four-segment families LRLR and RLRL
/// are added, and above 1/sqrt(2) the LR(pi)L, RL(pi)R, LRLRL and RLRLR families
/// join as well.
std::vector<Solution> allPaths(
  const Config & start, const Config & goal, double radius, double sphere_radius,
  double tolerance = 1e-4);

/// Shortest feasible path between two configurations on the sphere.
Solution optimalPath(
  const Config & start, const Config & goal, double radius, double sphere_radius,
  double tolerance = 1e-4);

/// Length of the shortest feasible path, or infinity when none exists. Cheaper
/// than optimalPath when the geometry is not needed.
double optimalPathLength(
  const Config & start, const Config & goal, double radius, double sphere_radius,
  double tolerance = 1e-4);

/// Samples a path. Positions are relative to the sphere centre.
Samples pathSamples(
  const Config & start, double radius, double sphere_radius,
  const std::vector<double> & angles, const std::string & type, double spacing);

}  // namespace dubins_path_3d::sphere

#endif  // DUBINS_PATH_3D__SPHERE_DUBINS_HPP_

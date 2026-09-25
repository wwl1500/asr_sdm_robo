// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__SURFACE_CONNECTIONS_HPP_
#define DUBINS_PATH_3D__SURFACE_CONNECTIONS_HPP_

#include <vector>

#include "dubins_path_3d/types.hpp"

namespace dubins_path_3d
{

/// Geometry of the line joining the two osculating spheres a construction uses.
struct SphereBridge
{
  Vec3 initial_centre{Vec3::Zero()};
  Vec3 final_centre{Vec3::Zero()};
  /// Unit vector from initial_centre to final_centre.
  Vec3 axis{Vec3::UnitZ()};
  /// Distance between the two centres.
  double separation{0.0};
};

/// Result of one construction: the best path found over the parameter sweep.
struct SurfaceResult
{
  double length{kInfinity};
  std::vector<PathSample> samples;

  bool valid() const {return std::isfinite(length);}
};

/// Sphere - cylindrical envelope - sphere.
///
/// The two osculating spheres of the same kind are joined by a cylinder of the
/// same radius. The sweep is over the entry and exit locations on the cylinder
/// profile and the headings there.
SurfaceResult sphereCylinderSphere(
  const Configuration & start, const Configuration & goal, const SphereBridge & bridge,
  SphereChoice choice, const PlannerOptions & options);

/// Sphere - cross-tangent plane - sphere.
///
/// An inner sphere at one end is joined to an outer sphere at the other (or a
/// left sphere to a right one) by a plane tangent to both. The sweep is over the
/// choice of tangent plane and the headings at the two tangency points.
SurfaceResult spherePlaneSphere(
  const Configuration & start, const Configuration & goal, const SphereBridge & bridge,
  SphereChoice choice, const PlannerOptions & options);

/// Sphere - intermediary sphere - sphere.
///
/// A third sphere of the same radius touches both osculating spheres. The sweep
/// is over its position on the locus of valid centres and the headings at the
/// two tangency points.
SurfaceResult sphereSphereSphere(
  const Configuration & start, const Configuration & goal, const SphereBridge & bridge,
  SphereChoice choice, const PlannerOptions & options);

}  // namespace dubins_path_3d

#endif  // DUBINS_PATH_3D__SURFACE_CONNECTIONS_HPP_

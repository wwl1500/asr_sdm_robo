// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__PLANNER_HPP_
#define DUBINS_PATH_3D__PLANNER_HPP_

#include "dubins_path_3d/types.hpp"

namespace dubins_path_3d
{

/// Feasible-path planner for a Dubins vehicle in 3D with bounded pitch and yaw
/// rates.
///
/// Four osculating spheres surround each configuration: two for the pitch rate
/// (pitch up on the inner sphere, pitch down on the outer one) and two for the
/// yaw rate (left and right turns). The planner joins a sphere at the start to a
/// sphere at the goal with a cylindrical envelope, a cross-tangent plane or an
/// intermediary sphere, sweeps the free parameters of each construction, and
/// returns the shortest feasible result.
///
/// Instances are stateless apart from the options, so `plan` may be called from
/// several threads at once.
class DubinsPath3D
{
public:
  DubinsPath3D() = default;

  /// Throws std::invalid_argument when `options` are inconsistent.
  explicit DubinsPath3D(PlannerOptions options);

  /// Plans between two configurations.
  ///
  /// Both configurations are re-orthonormalised before use. Throws
  /// std::invalid_argument when the frames are degenerate.
  PlanningResult plan(const Configuration & start, const Configuration & goal) const;

  const PlannerOptions & options() const {return options_;}

  /// Throws std::invalid_argument when the new options are inconsistent.
  void setOptions(PlannerOptions options);

private:
  PlannerOptions options_{};
};

}  // namespace dubins_path_3d

#endif  // DUBINS_PATH_3D__PLANNER_HPP_

// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include "dubins_path_3d/planner.hpp"

#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "dubins_path_3d/surface_connections.hpp"

namespace dubins_path_3d
{

namespace
{

constexpr double kDegenerateSeparation = 1e-9;

/// Centre of the osculating sphere of the requested kind.
Vec3 sphereCentre(
  const Configuration & config, SphereChoice choice, const PlannerOptions & options)
{
  switch (choice) {
    case SphereChoice::Inner:
      return config.position + options.pitch_radius * config.surface_normal;
    case SphereChoice::Outer:
      return config.position - options.pitch_radius * config.surface_normal;
    case SphereChoice::Left:
      return config.position + options.yaw_radius * config.tangent_normal;
    case SphereChoice::Right:
      return config.position - options.yaw_radius * config.tangent_normal;
  }
  throw std::invalid_argument("sphereCentre: unknown sphere choice");
}

/// The sphere on the far side of the same rate constraint, which is the one a
/// cross-tangent plane connects to.
SphereChoice oppositeChoice(SphereChoice choice)
{
  switch (choice) {
    case SphereChoice::Inner:
      return SphereChoice::Outer;
    case SphereChoice::Outer:
      return SphereChoice::Inner;
    case SphereChoice::Left:
      return SphereChoice::Right;
    case SphereChoice::Right:
      return SphereChoice::Left;
  }
  throw std::invalid_argument("oppositeChoice: unknown sphere choice");
}

SphereBridge makeBridge(const Vec3 & initial_centre, const Vec3 & final_centre)
{
  SphereBridge bridge;
  bridge.initial_centre = initial_centre;
  bridge.final_centre = final_centre;
  const Vec3 offset = final_centre - initial_centre;
  bridge.separation = offset.norm();
  if (bridge.separation > kDegenerateSeparation) {
    bridge.axis = offset / bridge.separation;
  } else {
    bridge.separation = 0.0;
  }
  return bridge;
}

}  // namespace

DubinsPath3D::DubinsPath3D(PlannerOptions options)
{
  setOptions(std::move(options));
}

void DubinsPath3D::setOptions(PlannerOptions options)
{
  options.validate();
  options_ = std::move(options);
}

PlanningResult DubinsPath3D::plan(
  const Configuration & start, const Configuration & goal) const
{
  options_.validate();

  Configuration start_config = start;
  Configuration goal_config = goal;
  if (start_config.tangent.norm() < kDegenerateSeparation ||
    goal_config.tangent.norm() < kDegenerateSeparation)
  {
    throw std::invalid_argument("plan: tangent vectors must be non-zero");
  }
  start_config.normalize();
  goal_config.normalize();

  const auto started_at = std::chrono::steady_clock::now();

  PlanningResult result;
  result.candidates.reserve(12);

  static constexpr std::array<SphereChoice, 4> kChoices{
    SphereChoice::Inner, SphereChoice::Outer, SphereChoice::Left, SphereChoice::Right};

  const auto consider = [&](const std::string & name, SurfaceResult && candidate) {
      result.candidates.emplace_back(name, candidate.length);
      if (candidate.valid() && !candidate.samples.empty() &&
        candidate.length < result.best.length)
      {
        result.best.type = name;
        result.best.length = candidate.length;
        result.best.samples = std::move(candidate.samples);
      }
    };

  for (const SphereChoice choice : kChoices) {
    const SphereBridge bridge = makeBridge(
      sphereCentre(start_config, choice, options_),
      sphereCentre(goal_config, choice, options_));

    consider(
      std::string("cyc_") + toString(choice),
      sphereCylinderSphere(start_config, goal_config, bridge, choice, options_));
  }

  for (const SphereChoice choice : kChoices) {
    const SphereChoice far_choice = oppositeChoice(choice);
    const SphereBridge bridge = makeBridge(
      sphereCentre(start_config, choice, options_),
      sphereCentre(goal_config, far_choice, options_));

    consider(
      std::string("plane_") + toString(choice) + "_" + toString(far_choice),
      spherePlaneSphere(start_config, goal_config, bridge, choice, options_));
  }

  for (const SphereChoice choice : kChoices) {
    const SphereBridge bridge = makeBridge(
      sphereCentre(start_config, choice, options_),
      sphereCentre(goal_config, choice, options_));

    consider(
      std::string("sphere_") + toString(choice),
      sphereSphereSphere(start_config, goal_config, bridge, choice, options_));
  }

  const auto finished_at = std::chrono::steady_clock::now();
  result.planning_time =
    std::chrono::duration<double>(finished_at - started_at).count();
  return result;
}

}  // namespace dubins_path_3d

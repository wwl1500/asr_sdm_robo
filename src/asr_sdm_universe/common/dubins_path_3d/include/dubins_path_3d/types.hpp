// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__TYPES_HPP_
#define DUBINS_PATH_3D__TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace dubins_path_3d
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

/// Configuration of the vehicle: a position together with a right-handed
/// orthonormal body frame (tangent, tangent-normal, surface-normal).
///
/// The roll angle is encoded by the rotation of (tangent_normal, surface_normal)
/// about tangent, which is why a plain position/heading pair is not sufficient to
/// describe the endpoints of a path.
struct Configuration
{
  Vec3 position{Vec3::Zero()};
  Vec3 tangent{Vec3::UnitX()};
  Vec3 tangent_normal{Vec3::UnitY()};
  Vec3 surface_normal{Vec3::UnitZ()};

  /// Rotation whose columns are (tangent, tangent_normal, surface_normal).
  Mat3 rotation() const;

  /// True when the frame is orthonormal and right-handed within `tolerance`.
  bool isOrthonormal(double tolerance = 1e-6) const;

  /// Re-orthonormalises the frame in place, keeping the `tangent` direction.
  ///
  /// Throws std::invalid_argument when the frame cannot be repaired, that is
  /// when the tangent vanishes or the tangent-normal is parallel to it.
  void normalize();
};

/// Builds a configuration from the aeronautical angles used by the reference
/// implementation.
///
/// `heading` is measured from the x-axis in the xy-plane, `pitch` is positive
/// when the vehicle noses up out of the xy-plane, and `roll` rotates the body
/// frame about the tangent.
Configuration configurationFromEuler(
  const Vec3 & position, double heading, double pitch, double roll);

/// Inverse of configurationFromEuler. Roll is recovered in (-pi, pi].
void eulerFromConfiguration(
  const Configuration & config, double & heading, double & pitch, double & roll);

/// A sample along a path, carrying the full body frame so that the vehicle
/// attitude (including roll) is available to a controller or visualiser.
struct PathSample
{
  Vec3 position{Vec3::Zero()};
  Vec3 tangent{Vec3::UnitX()};
  Vec3 tangent_normal{Vec3::UnitY()};
  Vec3 surface_normal{Vec3::UnitZ()};
};

/// A feasible path through the intermediary surfaces.
struct Path3D
{
  std::string type;
  double length{kInfinity};
  std::vector<PathSample> samples;

  bool valid() const {return std::isfinite(length);}
};

/// Which pair of osculating spheres a construction is built on.
enum class SphereChoice
{
  Inner,   ///< pitch sphere on the +surface_normal side
  Outer,   ///< pitch sphere on the -surface_normal side
  Left,    ///< yaw sphere on the +tangent_normal side
  Right,   ///< yaw sphere on the -tangent_normal side
};

const char * toString(SphereChoice choice);

/// Tuning parameters of the heuristic search.
struct PlannerOptions
{
  /// Radius of the sphere traced when the pitch rate is maximal and the yaw rate
  /// is zero.
  double pitch_radius{40.0};

  /// Radius of the sphere traced when the yaw rate is maximal and the pitch rate
  /// is zero.
  double yaw_radius{50.0};

  /// Tight-turn radius on the spherical surfaces. Non-positive requests the
  /// value implied by pitch_radius and yaw_radius.
  double tight_turn_radius{-1.0};

  /// Number of samples of the location parameter of the intermediary surface.
  int location_samples{15};

  /// Number of samples of the heading parameter at the entry to and exit from the
  /// intermediary surface.
  int heading_samples{15};

  /// Target spacing in metres between consecutive samples of the output path.
  double sample_spacing{2.0};

  /// Worker threads used for the parameter sweep. Zero requests the hardware
  /// concurrency.
  int num_threads{0};

  /// Tolerance used when verifying that an inverse-kinematics solution on a
  /// sphere really reaches the requested configuration.
  double sphere_tolerance{1e-4};

  /// Effective tight-turn radius, resolving the non-positive sentinel.
  double resolvedTightTurnRadius() const;

  /// Throws std::invalid_argument when the options are inconsistent.
  void validate() const;
};

/// Outcome of a planning query, including per-construction diagnostics.
struct PlanningResult
{
  Path3D best;
  std::vector<std::pair<std::string, double>> candidates;
  double planning_time{0.0};

  bool success() const {return best.valid() && !best.samples.empty();}
};

}  // namespace dubins_path_3d

#endif  // DUBINS_PATH_3D__TYPES_HPP_

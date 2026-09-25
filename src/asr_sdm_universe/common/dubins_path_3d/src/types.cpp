// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include "dubins_path_3d/types.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace dubins_path_3d
{

Mat3 Configuration::rotation() const
{
  Mat3 rot;
  rot.col(0) = tangent;
  rot.col(1) = tangent_normal;
  rot.col(2) = surface_normal;
  return rot;
}

bool Configuration::isOrthonormal(double tolerance) const
{
  const Mat3 rot = rotation();
  if ((rot.transpose() * rot - Mat3::Identity()).cwiseAbs().maxCoeff() > tolerance) {
    return false;
  }
  return std::abs(rot.determinant() - 1.0) <= tolerance;
}

void Configuration::normalize()
{
  constexpr double kMinNorm = 1e-9;

  if (!tangent.allFinite() || tangent.norm() < kMinNorm) {
    throw std::invalid_argument("Configuration::normalize: the tangent is zero or not finite.");
  }
  tangent.normalize();

  if (!tangent_normal.allFinite()) {
    throw std::invalid_argument("Configuration::normalize: the tangent-normal is not finite.");
  }
  // Gram-Schmidt against the tangent. A tangent-normal parallel to the tangent
  // leaves nothing to normalise, so the frame cannot be repaired.
  tangent_normal -= tangent_normal.dot(tangent) * tangent;
  if (tangent_normal.norm() < kMinNorm) {
    throw std::invalid_argument(
      "Configuration::normalize: the tangent-normal is parallel to the tangent.");
  }
  tangent_normal.normalize();

  surface_normal = tangent.cross(tangent_normal);
}

Configuration configurationFromEuler(
  const Vec3 & position, double heading, double pitch, double roll)
{
  Configuration config;
  config.position = position;
  config.tangent = Vec3(
    std::cos(heading) * std::cos(pitch),
    std::sin(heading) * std::cos(pitch),
    std::sin(pitch));

  // Wing direction with zero roll: the in-plane normal to the heading.
  const Vec3 level_wing(std::cos(heading + M_PI_2), std::sin(heading + M_PI_2), 0.0);
  config.tangent_normal =
    std::cos(roll) * level_wing + std::sin(roll) * config.tangent.cross(level_wing);
  config.surface_normal = config.tangent.cross(config.tangent_normal);
  config.normalize();
  return config;
}

void eulerFromConfiguration(
  const Configuration & config, double & heading, double & pitch, double & roll)
{
  const Vec3 tangent = config.tangent.normalized();
  heading = std::atan2(tangent.y(), tangent.x());
  pitch = std::asin(std::clamp(tangent.z(), -1.0, 1.0));

  const Vec3 level_wing(std::cos(heading + M_PI_2), std::sin(heading + M_PI_2), 0.0);
  const Vec3 wing = config.tangent_normal.normalized();
  roll = std::atan2(tangent.cross(level_wing).dot(wing), level_wing.dot(wing));
}

const char * toString(SphereChoice choice)
{
  switch (choice) {
    case SphereChoice::Inner:
      return "inner";
    case SphereChoice::Outer:
      return "outer";
    case SphereChoice::Left:
      return "left";
    case SphereChoice::Right:
      return "right";
  }
  return "unknown";
}

double PlannerOptions::resolvedTightTurnRadius() const
{
  if (tight_turn_radius > 0.0) {
    return tight_turn_radius;
  }
  const double inv_pitch = 1.0 / pitch_radius;
  const double inv_yaw = 1.0 / yaw_radius;
  return 1.0 / std::sqrt(inv_pitch * inv_pitch + inv_yaw * inv_yaw);
}

void PlannerOptions::validate() const
{
  std::ostringstream problem;
  if (!(pitch_radius > 0.0)) {
    problem << "pitch_radius must be positive (got " << pitch_radius << "). ";
  }
  if (!(yaw_radius > 0.0)) {
    problem << "yaw_radius must be positive (got " << yaw_radius << "). ";
  }
  if (location_samples < 1) {
    problem << "location_samples must be at least 1 (got " << location_samples << "). ";
  }
  if (heading_samples < 1) {
    problem << "heading_samples must be at least 1 (got " << heading_samples << "). ";
  }
  if (!(sample_spacing > 0.0)) {
    problem << "sample_spacing must be positive (got " << sample_spacing << "). ";
  }

  if (pitch_radius > 0.0 && yaw_radius > 0.0) {
    const double tight = resolvedTightTurnRadius();
    if (!(tight > 0.0)) {
      problem << "tight_turn_radius must be positive (got " << tight << "). ";
    } else if (tight >= pitch_radius || tight >= yaw_radius) {
      // A turn on a sphere of radius R has radius r < R by construction; r >= R
      // makes sqrt(1 - (r/R)^2) imaginary in the segment operators.
      problem << "tight_turn_radius (" << tight
              << ") must be smaller than both pitch_radius (" << pitch_radius
              << ") and yaw_radius (" << yaw_radius << "). ";
    }
  }

  const std::string message = problem.str();
  if (!message.empty()) {
    throw std::invalid_argument("Invalid PlannerOptions: " + message);
  }
}

}  // namespace dubins_path_3d

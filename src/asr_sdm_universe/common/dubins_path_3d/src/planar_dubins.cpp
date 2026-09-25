// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include "dubins_path_3d/planar_dubins.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dubins_path_3d/math_utils.hpp"

namespace dubins_path_3d::planar
{

namespace
{

constexpr double kZeroSegmentTolerance = 1e-6;
constexpr double kUnitTolerance = 1e-6;

}  // namespace

const std::vector<std::string> & pathTypes()
{
  static const std::vector<std::string> types{"lsl", "rsr", "lsr", "rsl", "lrl", "rlr"};
  return types;
}

Config applySegment(const Config & start, double param, double radius, char segment)
{
  Config out;
  switch (segment) {
    case 'l':
      out.x = start.x + radius * std::sin(start.heading + param) -
        radius * std::sin(start.heading);
      out.y = start.y - radius * std::cos(start.heading + param) +
        radius * std::cos(start.heading);
      out.heading = start.heading + param;
      break;
    case 'r':
      out.x = start.x - radius * std::sin(start.heading - param) +
        radius * std::sin(start.heading);
      out.y = start.y + radius * std::cos(start.heading - param) -
        radius * std::cos(start.heading);
      out.heading = start.heading - param;
      break;
    case 's':
      out.x = start.x + param * std::cos(start.heading);
      out.y = start.y + param * std::sin(start.heading);
      out.heading = start.heading;
      break;
    default:
      throw std::invalid_argument(
        std::string("Unknown planar segment type '") + segment + "'");
  }
  return out;
}

Config finalConfig(
  const Config & start, double radius, const std::array<double, 3> & params,
  const std::string & type)
{
  Config current = start;
  for (std::size_t i = 0; i < type.size(); ++i) {
    current = applySegment(current, params[i], radius, type[i]);
  }
  return current;
}

std::vector<Config> segmentSamples(
  const Config & start, double param, double radius, char segment, double spacing)
{
  const double arc_length = (segment == 's') ? param : param * radius;
  const int count = sampleCount(arc_length, spacing);

  std::vector<Config> samples;
  samples.reserve(static_cast<std::size_t>(std::max(count, 0)));
  for (int i = 0; i < count; ++i) {
    samples.push_back(applySegment(start, linspaceValue(0.0, param, count, i), radius, segment));
  }
  return samples;
}

std::vector<Config> pathSamples(
  const Config & start, double radius, const std::array<double, 3> & params,
  const std::string & type, double spacing)
{
  std::vector<Config> samples;
  Config current = start;

  for (std::size_t i = 0; i < type.size(); ++i) {
    const char segment = type[i];
    const double arc_length = (segment == 's') ? params[i] : params[i] * radius;

    // Each segment contributes its own start and interior points but not its
    // end point, which belongs to the next segment; the end of the path is
    // appended once after the loop. Sampling the closed interval instead would
    // drop the start of any segment shorter than `spacing`, because such a
    // segment gets a single sample that is then discarded as a junction.
    const int count = sampleCount(arc_length, spacing);
    samples.reserve(samples.size() + static_cast<std::size_t>(count) + 1U);
    for (int k = 0; k < count; ++k) {
      const double param = params[i] * static_cast<double>(k) / static_cast<double>(count);
      samples.push_back(applySegment(current, param, radius, segment));
    }

    current = applySegment(current, params[i], radius, segment);
  }

  samples.push_back(current);
  return samples;
}

namespace
{

/// Translates `start` to the origin and rotates so that `goal` lies on the
/// positive x-axis, which is the frame the closed-form expressions assume.
void toCanonicalFrame(
  const Config & start, const Config & goal, double & alpha_i, double & alpha_f, double & d)
{
  const double connection_angle = std::atan2(goal.y - start.y, goal.x - start.x);
  d = std::hypot(goal.x - start.x, goal.y - start.y);
  alpha_i = wrapTwoPi(start.heading - connection_angle);
  alpha_f = wrapTwoPi(goal.heading - connection_angle);
}

}  // namespace

Solution cscPath(const Config & start, const Config & goal, double radius, const std::string & type)
{
  double alpha_i = 0.0;
  double alpha_f = 0.0;
  double d = 0.0;
  toCanonicalFrame(start, goal, alpha_i, alpha_f, d);

  const double r = radius;
  const double sin_i = std::sin(alpha_i);
  const double sin_f = std::sin(alpha_f);
  const double cos_i = std::cos(alpha_i);
  const double cos_f = std::cos(alpha_f);
  const double cos_diff = std::cos(alpha_f - alpha_i);

  double straight_squared = 0.0;
  if (type == "lsl") {
    straight_squared = d * d + 2.0 * r * r + 2.0 * d * r * (sin_i - sin_f) -
      2.0 * r * r * cos_diff;
  } else if (type == "rsr") {
    straight_squared = d * d + 2.0 * r * r + 2.0 * d * r * (sin_f - sin_i) -
      2.0 * r * r * cos_diff;
  } else if (type == "lsr") {
    straight_squared = d * d + 2.0 * d * r * (sin_i + sin_f) + 2.0 * r * r * (cos_diff - 1.0);
  } else if (type == "rsl") {
    straight_squared = d * d - 2.0 * d * r * (sin_i + sin_f) + 2.0 * r * r * (cos_diff - 1.0);
  } else {
    throw std::invalid_argument("cscPath: unsupported path type '" + type + "'");
  }

  if (straight_squared < 0.0 && straight_squared >= -kZeroSegmentTolerance) {
    straight_squared = 0.0;
  }

  Solution solution;
  solution.type = type;
  if (straight_squared < 0.0) {
    return solution;  // infeasible
  }

  if (straight_squared <= kZeroSegmentTolerance && type[0] == type[2]) {
    // Degenerates into a single arc.
    const double phi_1 =
      (type[0] == 'l') ? wrapTwoPi(alpha_f - alpha_i) : wrapTwoPi(alpha_i - alpha_f);
    solution.params = {phi_1, 0.0, 0.0};
    solution.length = r * phi_1;
    return solution;
  }

  if (straight_squared <= kZeroSegmentTolerance) {
    // Degenerates into two abutting arcs; the tangent-line angle collapses to pi/2.
    double phi_1 = 0.0;
    double phi_3 = 0.0;
    if (type[0] == 'l') {
      phi_1 = wrapTwoPi(
        M_PI_2 - alpha_i +
        std::atan2(-(r * cos_i + r * cos_f), d + r * sin_i + r * sin_f));
      phi_3 = wrapTwoPi(alpha_i - alpha_f + phi_1);
    } else {
      phi_1 = wrapTwoPi(
        M_PI_2 + alpha_i -
        std::atan2(r * cos_i + r * cos_f, d - r * sin_i - r * sin_f));
      phi_3 = wrapTwoPi(alpha_f - alpha_i + phi_1);
    }
    solution.params = {phi_1, 0.0, phi_3};
    solution.length = r * (phi_1 + phi_3);
    return solution;
  }

  const double straight = std::sqrt(straight_squared);
  double phi_1 = 0.0;
  double phi_3 = 0.0;

  if (type == "lsl") {
    phi_1 = wrapTwoPi(
      std::atan2(r * (cos_f - cos_i), d - r * (sin_f - sin_i)) - alpha_i);
    phi_3 = wrapTwoPi(alpha_f - alpha_i - phi_1);
  } else if (type == "rsr") {
    phi_1 = wrapTwoPi(
      -std::atan2(-r * (cos_f - cos_i), d + r * (sin_f - sin_i)) + alpha_i);
    phi_3 = wrapTwoPi(alpha_i - alpha_f - phi_1);
  } else if (type == "lsr") {
    phi_1 = wrapTwoPi(
      std::atan2(2.0 * r, straight) - alpha_i +
      std::atan2(-(r * cos_i + r * cos_f), d + r * sin_i + r * sin_f));
    phi_3 = wrapTwoPi(alpha_i - alpha_f + phi_1);
  } else {  // rsl
    phi_1 = wrapTwoPi(
      std::atan2(2.0 * r, straight) + alpha_i -
      std::atan2(r * cos_i + r * cos_f, d - r * sin_i - r * sin_f));
    phi_3 = wrapTwoPi(alpha_f - alpha_i + phi_1);
  }

  solution.params = {phi_1, straight, phi_3};
  solution.length = r * (phi_1 + phi_3) + straight;
  return solution;
}

Solution cccPath(const Config & start, const Config & goal, double radius, const std::string & type)
{
  double alpha_i = 0.0;
  double alpha_f = 0.0;
  double d = 0.0;
  toCanonicalFrame(start, goal, alpha_i, alpha_f, d);

  Solution solution;
  solution.type = type;

  const double r = radius;
  const double sin_i = std::sin(alpha_i);
  const double sin_f = std::sin(alpha_f);
  const double cos_i = std::cos(alpha_i);
  const double cos_f = std::cos(alpha_f);
  const double cos_diff = std::cos(alpha_f - alpha_i);

  double cos_phi_2 = 0.0;
  if (type == "lrl") {
    cos_phi_2 = 1.0 - (1.0 / (2.0 * std::pow(2.0 * r, 2))) *
      (d * d + 2.0 * r * r - 2.0 * d * r * (sin_f - sin_i) - 2.0 * r * r * cos_diff);
  } else if (type == "rlr") {
    cos_phi_2 = 1.0 - (1.0 / (2.0 * std::pow(2.0 * r, 2))) *
      (d * d + 2.0 * r * r + 2.0 * d * r * (sin_f - sin_i) - 2.0 * r * r * cos_diff);
  } else {
    throw std::invalid_argument("cccPath: unsupported path type '" + type + "'");
  }

  if (std::abs(cos_phi_2) > 1.0 && std::abs(cos_phi_2) <= 1.0 + kUnitTolerance) {
    cos_phi_2 = (cos_phi_2 > 0.0) ? 1.0 : -1.0;
  }
  if (std::abs(cos_phi_2) > 1.0 || cos_phi_2 == 1.0) {
    // Infeasible, or a degenerate single arc already covered by the CSC family.
    return solution;
  }

  // From the maximum principle the middle arc of an optimal CCC path exceeds pi.
  const double phi_2 = 2.0 * M_PI - std::acos(cos_phi_2);
  double phi_1 = 0.0;
  double phi_3 = 0.0;

  if (type == "lrl") {
    phi_1 = wrapTwoPi(
      std::atan2(r * (cos_f - cos_i), d - r * (sin_f - sin_i)) - alpha_i + phi_2 / 2.0);
    phi_3 = wrapTwoPi(alpha_f - alpha_i - phi_1 + phi_2);
  } else {
    phi_1 = wrapTwoPi(
      -std::atan2(-r * (cos_f - cos_i), d + r * (sin_f - sin_i)) + alpha_i + phi_2 / 2.0);
    phi_3 = wrapTwoPi(alpha_i - alpha_f - phi_1 + phi_2);
  }

  if (phi_1 <= kZeroSegmentTolerance || phi_2 <= kZeroSegmentTolerance ||
    phi_3 <= kZeroSegmentTolerance)
  {
    return solution;  // degenerate, covered by a shorter family
  }

  solution.params = {phi_1, phi_2, phi_3};
  solution.length = r * (phi_1 + phi_2 + phi_3);
  return solution;
}

Solution optimalPath(const Config & start, const Config & goal, double radius)
{
  Solution best;
  for (const std::string & type : pathTypes()) {
    const Solution candidate = (type[1] == 's') ?
      cscPath(start, goal, radius, type) :
      cccPath(start, goal, radius, type);
    if (candidate.valid() && candidate.length < best.length) {
      best = candidate;
    }
  }
  return best;
}

}  // namespace dubins_path_3d::planar

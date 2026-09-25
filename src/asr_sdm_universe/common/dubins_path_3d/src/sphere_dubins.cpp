// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0
//
// Inverse kinematics for Dubins paths on a sphere. The closed-form expressions
// follow "3D Motion Planning for a Generalized Dubins Vehicle considering Pitch
// and Yaw Rate Constraints"; every candidate produced by them is verified by
// forward-propagating the segment operators, which keeps spurious branches of
// the trigonometric inverses out of the result.

#include "dubins_path_3d/sphere_dubins.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dubins_path_3d/math_utils.hpp"

namespace dubins_path_3d::sphere
{

namespace
{

constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kAngleSnapTolerance = 1e-8;
constexpr double kUnitSnapTolerance = 1e-12;
constexpr double kNearZeroAngle = 1e-8;
constexpr double kSpecialCaseTolerance = 1e-6;

/// Reflects a unit-sphere configuration about the xy-plane, which maps an
/// R-leading path onto an L-leading one. The third column is negated a second
/// time so that the result stays a rotation matrix.
Mat3 reflectAboutXyPlane(const Mat3 & matrix)
{
  Mat3 out = matrix;
  out.row(2) = -matrix.row(2);
  out.col(2) = -out.col(2);
  return out;
}

/// Expresses the goal on a unit sphere in a frame where the start is the
/// identity, since Rfinal = Rinitial * Rnet.
Mat3 toUnitSphereFrame(const Config & start, const Config & goal, double sphere_radius)
{
  Mat3 scaled_start = start;
  Mat3 scaled_goal = goal;
  scaled_start.col(0) /= sphere_radius;
  scaled_goal.col(0) /= sphere_radius;
  return scaled_start.transpose() * scaled_goal;
}

/// Branches of acos(rhs) + offset in [0, 2*pi), with angles that round up to a
/// full turn collapsed to zero.
std::vector<double> angleBranches(double rhs, double offset)
{
  if (!snapToUnitInterval(rhs, kAngleSnapTolerance)) {
    return {};
  }

  const double principal = std::acos(rhs);
  std::vector<double> branches;
  branches.push_back(wrapTwoPi(principal + offset));
  if (std::abs(rhs) != 1.0) {
    branches.push_back(wrapTwoPi(kTwoPi - principal + offset));
  }

  for (double & angle : branches) {
    if (kTwoPi - angle <= kNearZeroAngle) {
      angle = 0.0;
    }
  }
  return branches;
}

/// True when propagating `angles` from the identity reaches `goal_unit`.
bool reachesGoal(
  const Mat3 & goal_unit, double scaled_radius, const std::vector<double> & angles,
  const std::string & type, double tolerance)
{
  const Mat3 achieved = finalConfig(Mat3::Identity(), scaled_radius, 1.0, angles, type);
  return (achieved - goal_unit).cwiseAbs().maxCoeff() <= tolerance;
}

void pushSolution(
  std::vector<Solution> & out, const std::string & type, double length,
  std::vector<double> angles)
{
  Solution solution;
  solution.type = type;
  solution.length = length;
  solution.angles = std::move(angles);
  out.push_back(std::move(solution));
}

// ---------------------------------------------------------------------------
// Three-segment families: LGL, RGR, LGR, RGL, LRL, RLR
// ---------------------------------------------------------------------------

void appendThreeSegmentPaths(
  const Mat3 & goal_unit, double radius, double sphere_radius, const std::string & requested,
  double tolerance, std::vector<Solution> & out)
{
  const double rb = radius / sphere_radius;
  const double rb2 = rb * rb;
  const double comp = std::sqrt(1.0 - rb2);   // sqrt(1 - (r/R)^2)

  // R-leading paths are solved as their mirrored L-leading counterparts.
  std::string type = requested;
  Mat3 goal_for_solve = goal_unit;
  if (requested == "rgl") {
    type = "lgr";
    goal_for_solve = reflectAboutXyPlane(goal_unit);
  } else if (requested == "rlr") {
    type = "lrl";
    goal_for_solve = reflectAboutXyPlane(goal_unit);
  }

  const double a11 = goal_for_solve(0, 0);
  const double a12 = goal_for_solve(0, 1);
  const double a13 = goal_for_solve(0, 2);
  const double a21 = goal_for_solve(1, 0);
  const double a22 = goal_for_solve(1, 1);
  const double a31 = goal_for_solve(2, 0);
  const double a33 = goal_for_solve(2, 2);

  double cos_phi2 = 0.0;
  if (type == "lgl") {
    cos_phi2 = (a11 + rb * comp * (a13 + a31) + rb2 * (a33 - a11 - 1.0)) / (1.0 - rb2);
  } else if (type == "lgr") {
    cos_phi2 = ((1.0 - rb2) * a11 + rb * comp * (a31 - a13) + rb2 * (1.0 - a33)) / (1.0 - rb2);
  } else if (type == "rgr") {
    cos_phi2 = (a11 - rb * comp * (a13 + a31) + rb2 * (a33 - a11 - 1.0)) / (1.0 - rb2);
  } else if (type == "lrl") {
    cos_phi2 = ((1.0 - rb2) * a11 + rb * comp * (a13 + a31) + rb2 * a33 -
      std::pow(1.0 - 2.0 * rb2, 2)) / (4.0 * rb2 * (1.0 - rb2));
  } else {
    throw std::invalid_argument("appendThreeSegmentPaths: unsupported type '" + requested + "'");
  }

  // A middle segment that is very nearly absent or exactly pi must be snapped,
  // but only with a tight tolerance: a loose one discards genuine short
  // great-circle segments.
  if (std::abs(cos_phi2) > 1.0) {
    if (std::abs(cos_phi2) <= 1.0 + kUnitSnapTolerance) {
      cos_phi2 = (cos_phi2 > 0.0) ? 1.0 : -1.0;
    } else {
      return;  // infeasible
    }
  } else if (std::abs(cos_phi2) >= 1.0 - kUnitSnapTolerance) {
    cos_phi2 = (cos_phi2 > 0.0) ? 1.0 : -1.0;
  }

  const bool degenerate_family =
    (type == "lgl" || type == "rgr" || type == "lrl" || type == "rlr");
  if (cos_phi2 == 1.0 && degenerate_family) {
    // Collapses to a single tight turn.
    double phi1 = wrapTwoPi(std::atan2(a21, rb * a22));
    if (kTwoPi - phi1 < kNearZeroAngle) {
      phi1 = 0.0;
    }
    const std::vector<double> angles{phi1, 0.0, 0.0};
    if (reachesGoal(goal_unit, rb, angles, requested, tolerance)) {
      pushSolution(out, requested, radius * phi1, angles);
    }
    return;
  }

  std::vector<double> phi2_candidates;
  if (std::abs(cos_phi2) == 1.0) {
    // The middle arc of an optimal LRL path cannot be pi; that case is handled
    // by the dedicated LR(pi)L construction.
    if (type != "lrl") {
      phi2_candidates.push_back(std::acos(cos_phi2));
    }
  } else if (type == "lrl") {
    phi2_candidates.push_back(kTwoPi - std::acos(cos_phi2));
  } else {
    phi2_candidates.push_back(std::acos(cos_phi2));
    phi2_candidates.push_back(kTwoPi - std::acos(cos_phi2));
  }

  for (double phi2 : phi2_candidates) {
    const double cos_2 = std::cos(phi2);
    const double sin_2 = std::sin(phi2);

    std::vector<double> phi1_candidates;
    std::vector<double> phi3_candidates;

    if (type == "lgr" && cos_phi2 == -1.0) {
      // A pi-long great circle lets the path collapse to a degenerate LG or GR.
      double phi1 = wrapTwoPi(std::atan2(a12, -rb * a22));
      if (kTwoPi - phi1 <= kNearZeroAngle) {
        phi1 = 0.0;
      }
      phi2 = M_PI;
      phi1_candidates = {phi1};
      phi3_candidates = {0.0};
    } else {
      double phi1_rhs = 0.0;
      double phi3_rhs = 0.0;
      double offset = 0.0;

      if (type == "lgl") {
        const double denominator =
          std::sqrt(rb2 * std::pow(1.0 - cos_2, 2) + sin_2 * sin_2);
        phi1_rhs = ((a33 - a11) * rb - a13 * rb2 / comp + a31 * comp) / denominator;
        phi3_rhs = ((a33 - a11) * rb + a13 * comp - a31 * rb2 / comp) / denominator;
        offset = std::atan2(sin_2, rb * (1.0 - cos_2));
      } else if (type == "rgr") {
        const double denominator =
          std::sqrt(rb2 * std::pow(1.0 - cos_2, 2) + sin_2 * sin_2);
        phi1_rhs = ((a33 - a11) * rb + a13 * rb2 / comp - a31 * comp) / denominator;
        phi3_rhs = ((a33 - a11) * rb - a13 * comp + a31 * rb2 / comp) / denominator;
        offset = std::atan2(sin_2, rb * (1.0 - cos_2));
      } else if (type == "lgr") {
        const double denominator = std::sqrt(
          std::pow(rb * comp * cos_2 + rb * comp, 2) + (1.0 - rb2) * sin_2 * sin_2);
        phi1_rhs = (rb * comp * a11 - (1.0 - rb2) * a31 - rb2 * a13 + rb * comp * a33) /
          denominator;
        phi3_rhs = (rb * comp * a11 + (1.0 - rb2) * a13 + rb2 * a31 + rb * comp * a33) /
          denominator;
        // Negated because the analytic expression subtracts this offset.
        offset = -std::atan2(sin_2, rb * (cos_2 + 1.0));
      } else {  // lrl
        const double coeff_a = (2.0 * rb2 - 1.0) * (1.0 - cos_2);
        const double coeff_b = sin_2;
        const double shared = 8.0 * std::pow(rb, 6) - 12.0 * std::pow(rb, 4) + 6.0 * rb2 -
          1.0 - 4.0 * (2.0 * std::pow(rb, 6) - 3.0 * std::pow(rb, 4) + rb2) * cos_2;
        const double scale = 4.0 * rb2 * (1.0 - rb2);
        const double norm = std::sqrt(coeff_a * coeff_a + coeff_b * coeff_b);

        phi1_rhs = ((rb2 - 1.0) * a11 + rb * comp * (a31 - a13) + rb2 * a33 - shared) /
          scale / norm;
        phi3_rhs = ((rb2 - 1.0) * a11 + rb * comp * (a13 - a31) + rb2 * a33 - shared) /
          scale / norm;
        offset = std::atan2(coeff_b, coeff_a);
      }

      phi1_candidates = angleBranches(phi1_rhs, offset);
      phi3_candidates = angleBranches(phi3_rhs, offset);
    }

    for (const double phi1 : phi1_candidates) {
      for (const double phi3 : phi3_candidates) {
        const std::vector<double> angles{phi1, phi2, phi3};
        if (!reachesGoal(goal_unit, rb, angles, requested, tolerance)) {
          continue;
        }
        const double length = (type == "lrl") ?
          radius * (phi1 + phi2 + phi3) :
          radius * (phi1 + phi3) + sphere_radius * phi2;
        pushSolution(out, requested, length, angles);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// LR(pi)L and RL(pi)R, which can be optimal for r/R > 1/sqrt(2)
// ---------------------------------------------------------------------------

void appendCPiCPaths(
  const Mat3 & goal_unit, double radius, double sphere_radius, const std::string & type,
  double tolerance, std::vector<Solution> & out)
{
  const double rb = radius / sphere_radius;
  const double rb2 = rb * rb;
  const double comp = std::sqrt(1.0 - rb2);

  const Mat3 goal_for_solve =
    (type == "rlr") ? reflectAboutXyPlane(goal_unit) : goal_unit;

  const double a11 = goal_for_solve(0, 0);
  const double a13 = goal_for_solve(0, 2);
  const double a21 = goal_for_solve(1, 0);
  const double a22 = goal_for_solve(1, 1);
  const double a31 = goal_for_solve(2, 0);
  const double a33 = goal_for_solve(2, 2);

  std::vector<double> phi1_candidates;
  std::vector<double> phi3_candidates;

  if (std::abs(rb - 1.0 / std::sqrt(2.0)) > kSpecialCaseTolerance) {
    const double scale = 1.0 / (8.0 * (rb2 - 1.0) * rb2);
    const double bulk = 1.0 - 8.0 * rb2 + 8.0 * std::pow(rb, 4);
    const double shared = 1.0 / (1.0 - 2.0 * rb2);

    const double phi1_rhs = scale *
      (bulk + shared * (a11 * (rb2 - 1.0) + rb * ((a31 - a13) * comp + a33 * rb)));
    const double phi3_rhs = scale *
      (bulk + shared * (a11 * (rb2 - 1.0) + rb * ((a13 - a31) * comp + a33 * rb)));

    phi1_candidates = angleBranches(phi1_rhs, 0.0);
    phi3_candidates = angleBranches(phi3_rhs, 0.0);
  } else {
    // At r/R = 1/sqrt(2) only the difference phi1 - phi3 is determined, so the
    // free angle is pinned to zero.
    const double difference = std::atan2(std::sqrt(2.0) * a21, -a22);
    if (difference < 0.0) {
      phi1_candidates = {0.0};
      phi3_candidates = {wrapTwoPi(-difference)};
    } else {
      phi1_candidates = {wrapTwoPi(difference)};
      phi3_candidates = {0.0};
    }
  }

  for (const double phi1 : phi1_candidates) {
    for (const double phi3 : phi3_candidates) {
      const std::vector<double> angles{phi1, M_PI, phi3};
      if (reachesGoal(goal_unit, rb, angles, type, tolerance)) {
        pushSolution(out, type, radius * (phi1 + M_PI + phi3), angles);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// LRLR and RLRL, which can be optimal for r/R > 1/2
// ---------------------------------------------------------------------------

void appendFourSegmentPaths(
  const Mat3 & goal_unit, double radius, double sphere_radius, const std::string & type,
  double tolerance, std::vector<Solution> & out)
{
  const double rb = radius / sphere_radius;
  const double rb2 = rb * rb;
  const double comp = std::sqrt(1.0 - rb2);

  const Mat3 goal_for_solve =
    (type == "rlrl") ? reflectAboutXyPlane(goal_unit) : goal_unit;

  const double a11 = goal_for_solve(0, 0);
  const double a12 = goal_for_solve(0, 1);
  const double a13 = goal_for_solve(0, 2);
  const double a22 = goal_for_solve(1, 1);
  const double a31 = goal_for_solve(2, 0);
  const double a33 = goal_for_solve(2, 2);

  // The two middle arcs share an angle, which satisfies a quadratic in its cosine.
  const double quad_a = 8.0 * std::pow(rb, 4) * (rb2 - 1.0);
  const double quad_b = -8.0 * (rb2 - 3.0 * std::pow(rb, 4) + 2.0 * std::pow(rb, 6));
  const double quad_c = -1.0 + 10.0 * rb2 - 16.0 * std::pow(rb, 4) + 8.0 * std::pow(rb, 6) -
    (a11 * (rb2 - 1.0) + rb * (comp * (a13 - a31) + a33 * rb));

  double discriminant = quad_b * quad_b - 4.0 * quad_a * quad_c;
  if (discriminant < 0.0 && discriminant >= -kUnitSnapTolerance) {
    discriminant = 0.0;
  }
  if (discriminant < 0.0) {
    return;
  }

  std::vector<double> cos_candidates;
  if (discriminant > 0.0) {
    const double root = std::sqrt(discriminant);
    cos_candidates = {
      (-quad_b - root) / (2.0 * quad_a),
      (-quad_b + root) / (2.0 * quad_a)};
  } else {
    cos_candidates = {-quad_b / (2.0 * quad_a)};
  }

  std::vector<double> phi2_candidates;
  for (double cos_phi2 : cos_candidates) {
    if (std::abs(cos_phi2) > 1.0) {
      if (std::abs(cos_phi2) <= 1.0 + kUnitSnapTolerance) {
        cos_phi2 = (cos_phi2 > 0.0) ? 1.0 : -1.0;
      }
    } else if (std::abs(cos_phi2) >= 1.0 - kUnitSnapTolerance) {
      cos_phi2 = (cos_phi2 > 0.0) ? 1.0 : -1.0;
    }
    if (std::abs(cos_phi2) < 1.0) {
      phi2_candidates.push_back(kTwoPi - std::acos(cos_phi2));
    }
  }

  for (const double phi2 : phi2_candidates) {
    const double cos_2 = std::cos(phi2);
    const double sin_2 = std::sin(phi2);

    if (std::abs(cos_2 - (1.0 - 1.0 / (2.0 * rb2))) <= kSpecialCaseTolerance) {
      // The system degenerates; fix the last arc and solve the first directly.
      const double discriminant_term = std::sqrt(4.0 * rb2 - 1.0);
      const double cos_phi1 = (1.0 / rb2) *
        ((discriminant_term / (2.0 * rb2)) * a12 - ((2.0 * rb2 - 1.0) / (2.0 * rb)) * a22);
      const double sin_phi1 = (1.0 / rb2) *
        (((2.0 * rb2 - 1.0) / (2.0 * rb2)) * a12 + (discriminant_term / (2.0 * rb)) * a22);

      if (std::abs(sin_phi1 * sin_phi1 + cos_phi1 * cos_phi1 - 1.0) > kUnitSnapTolerance) {
        continue;
      }
      const double phi1 = wrapTwoPi(std::atan2(sin_phi1, cos_phi1));
      const std::vector<double> angles{phi1, phi2, phi2, 0.0};
      if (reachesGoal(goal_unit, rb, angles, type, tolerance)) {
        pushSolution(out, type, radius * (phi1 + 2.0 * phi2), angles);
      }
      continue;
    }

    const double common = 4.0 * rb2 * (1.0 - rb2) * (2.0 * rb2 * cos_2 - 2.0 * rb2 + 1.0);
    const double coeff_a = common * ((2.0 * rb2 - 1.0) * cos_2 - 2.0 * rb2 + 2.0);
    const double coeff_b = common * (-sin_2);
    const double coeff_c = (2.0 * rb2 - 1.0) *
      (12.0 * std::pow(rb, 6) - 20.0 * std::pow(rb, 4) + 10.0 * rb2 +
      4.0 * (rb2 - 1.0) * std::pow(rb, 4) * std::cos(2.0 * phi2) -
      8.0 * (2.0 * std::pow(rb, 6) - 3.0 * std::pow(rb, 4) + rb2) * cos_2 - 1.0);

    const double norm = std::sqrt(coeff_a * coeff_a + coeff_b * coeff_b);
    const double phi1_rhs =
      (a11 * (1.0 - rb2) + rb * (-comp * (a13 + a31) + a33 * rb) - coeff_c) / norm;
    const double phi3_rhs =
      (a11 * (1.0 - rb2) + rb * (comp * (a13 + a31) + a33 * rb) - coeff_c) / norm;
    const double offset = std::atan2(coeff_b, coeff_a);

    for (const double phi1 : angleBranches(phi1_rhs, offset)) {
      for (const double phi3 : angleBranches(phi3_rhs, offset)) {
        const std::vector<double> angles{phi1, phi2, phi2, phi3};
        if (reachesGoal(goal_unit, rb, angles, type, tolerance)) {
          pushSolution(out, type, radius * (phi1 + 2.0 * phi2 + phi3), angles);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// LRLRL and RLRLR, which can be optimal for r/R > 1/sqrt(2)
// ---------------------------------------------------------------------------

void appendFiveSegmentPaths(
  const Mat3 & goal_unit, double radius, double sphere_radius, const std::string & type,
  double tolerance, std::vector<Solution> & out)
{
  const double rb = radius / sphere_radius;
  const double rb2 = rb * rb;
  const double comp = std::sqrt(1.0 - rb2);

  const Mat3 goal_for_solve =
    (type == "rlrlr") ? reflectAboutXyPlane(goal_unit) : goal_unit;

  const double a11 = goal_for_solve(0, 0);
  const double a12 = goal_for_solve(0, 1);
  const double a13 = goal_for_solve(0, 2);
  const double a22 = goal_for_solve(1, 1);
  const double a31 = goal_for_solve(2, 0);
  const double a33 = goal_for_solve(2, 2);

  // The three middle arcs share an angle, whose cosine solves a cubic.
  const double cubic_a = 16.0 * std::pow(rb, 6) * (1.0 - rb2);
  const double cubic_b = 16.0 * std::pow(rb, 4) * (2.0 - 5.0 * rb2 + 3.0 * std::pow(rb, 4));
  const double cubic_c = -16.0 * rb2 * std::pow(1.0 - rb2, 2) * (3.0 * rb2 - 1.0);
  const double cubic_d = 16.0 * std::pow(rb, 8) - 48.0 * std::pow(rb, 6) +
    48.0 * std::pow(rb, 4) - 16.0 * rb2 + 1.0 -
    (a11 * (1.0 - rb2) + rb * (comp * (a13 + a31) + a33 * rb));

  const std::vector<double> roots =
    solveCubicRealParts(cubic_a, cubic_b, cubic_c, cubic_d);
  if (roots.empty()) {
    return;
  }

  std::vector<double> phi2_candidates;
  for (double cos_phi2 : roots) {
    if (std::abs(cos_phi2) > 1.0) {
      if (std::abs(cos_phi2) <= 1.0 + kUnitSnapTolerance) {
        cos_phi2 = (cos_phi2 > 0.0) ? 1.0 : -1.0;
      }
    } else if (std::abs(cos_phi2) >= 1.0 - kUnitSnapTolerance) {
      cos_phi2 = (cos_phi2 > 0.0) ? 1.0 : -1.0;
    }
    if (std::abs(cos_phi2) < 1.0) {
      phi2_candidates.push_back(kTwoPi - std::acos(cos_phi2));
    }
  }

  for (const double phi2 : phi2_candidates) {
    const double cos_2 = std::cos(phi2);
    const double sin_2 = std::sin(phi2);

    if (std::abs(cos_2 - (1.0 - 1.0 / rb2)) <= kSpecialCaseTolerance) {
      const double discriminant_term = std::sqrt(2.0 * rb2 - 1.0);
      const double cos_phi1 = -(1.0 / std::pow(rb, 4)) *
        (discriminant_term * rb * a12 - rb2 * (rb2 - 1.0) * a22);
      const double sin_phi1 = -(1.0 / std::pow(rb, 4)) *
        ((rb2 - 1.0) * rb * a12 + rb2 * discriminant_term * a22);

      if (std::abs(sin_phi1 * sin_phi1 + cos_phi1 * cos_phi1 - 1.0) > kUnitSnapTolerance) {
        continue;
      }
      const double phi1 = wrapTwoPi(std::atan2(sin_phi1, cos_phi1));
      const std::vector<double> angles{phi1, phi2, phi2, phi2, 0.0};
      if (reachesGoal(goal_unit, rb, angles, type, tolerance)) {
        pushSolution(out, type, radius * (phi1 + 3.0 * phi2), angles);
      }
      continue;
    }

    const double half_sin_squared = std::pow(std::sin(phi2 / 2.0), 2);
    const double coeff_a = 16.0 * rb2 * (rb2 - 1.0) * half_sin_squared *
      (-6.0 * std::pow(rb, 6) + 11.0 * std::pow(rb, 4) - 7.0 * rb2 +
      (std::pow(rb, 4) - 2.0 * std::pow(rb, 6)) * std::cos(2.0 * phi2) +
      (8.0 * std::pow(rb, 4) - 12.0 * rb2 + 3.0) * rb2 * cos_2 + 1.0);
    const double coeff_b = 8.0 * rb2 * (1.0 - rb2) * sin_2 *
      (std::pow(rb, 4) * std::cos(2.0 * phi2) + 3.0 * std::pow(rb, 4) - 3.0 * rb2 +
      (3.0 * rb2 - 4.0 * std::pow(rb, 4)) * cos_2 + 1.0);
    const double coeff_c = (1.0 - 2.0 * rb2) *
      (4.0 * std::pow(rb, 8) * std::cos(3.0 * phi2) - 40.0 * std::pow(rb, 8) -
      4.0 * std::pow(rb, 6) * std::cos(3.0 * phi2) + 88.0 * std::pow(rb, 6) -
      64.0 * std::pow(rb, 4) + 16.0 * rb2 -
      8.0 * (3.0 * std::pow(rb, 4) - 5.0 * rb2 + 2.0) * std::pow(rb, 4) *
      std::cos(2.0 * phi2) +
      4.0 * (15.0 * std::pow(rb, 6) - 31.0 * std::pow(rb, 4) + 20.0 * rb2 - 4.0) * rb2 *
      cos_2 - 1.0);

    const double norm = std::sqrt(coeff_a * coeff_a + coeff_b * coeff_b);
    const double phi1_rhs =
      (a11 * (rb2 - 1.0) + rb * (comp * (a31 - a13) + a33 * rb) - coeff_c) / norm;
    const double phi3_rhs =
      (a11 * (rb2 - 1.0) + rb * (comp * (a13 - a31) + a33 * rb) - coeff_c) / norm;
    const double offset = std::atan2(coeff_b, coeff_a);

    for (const double phi1 : angleBranches(phi1_rhs, offset)) {
      for (const double phi3 : angleBranches(phi3_rhs, offset)) {
        const std::vector<double> angles{phi1, phi2, phi2, phi2, phi3};
        if (reachesGoal(goal_unit, rb, angles, type, tolerance)) {
          pushSolution(out, type, radius * (phi1 + 3.0 * phi2 + phi3), angles);
        }
      }
    }
  }
}

}  // namespace

Config makeConfig(const Vec3 & point, const Vec3 & centre, const Vec3 & tangent)
{
  const Vec3 radial = point - centre;
  const Vec3 tangent_normal = radial.cross(tangent).normalized();

  Config config;
  config.col(0) = radial;
  config.col(1) = tangent;
  config.col(2) = tangent_normal;
  return config;
}

Mat3 segmentOperator(double phi, double radius, double sphere_radius, char segment)
{
  const double rb = radius / sphere_radius;
  const double cos_phi = std::cos(phi);
  const double sin_phi = std::sin(phi);

  Mat3 operator_matrix;
  if (segment == 'g') {
    operator_matrix << cos_phi, -sin_phi / sphere_radius, 0.0,
      sphere_radius * sin_phi, cos_phi, 0.0,
      0.0, 0.0, 1.0;
    return operator_matrix;
  }

  if (segment != 'l' && segment != 'r') {
    throw std::invalid_argument(
      std::string("segmentOperator: unknown segment type '") + segment + "'");
  }

  // Right turns mirror left turns about the tangent plane's normal direction.
  const double sign = (segment == 'l') ? 1.0 : -1.0;
  const double comp = std::sqrt(1.0 - rb * rb);
  const double one_minus_cos = 1.0 - cos_phi;

  operator_matrix <<
    1.0 - one_minus_cos * rb * rb,
    -(rb / sphere_radius) * sin_phi,
    sign * (1.0 / sphere_radius) * one_minus_cos * rb * comp,

    radius * sin_phi,
    cos_phi,
    -sign * sin_phi * comp,

    sign * one_minus_cos * radius * comp,
    sign * sin_phi * comp,
    cos_phi + one_minus_cos * rb * rb;

  return operator_matrix;
}

Config applySegment(
  const Config & start, double phi, double radius, double sphere_radius, char segment)
{
  return start * segmentOperator(phi, radius, sphere_radius, segment);
}

Config finalConfig(
  const Config & start, double radius, double sphere_radius,
  const std::vector<double> & angles, const std::string & type)
{
  if (angles.size() < type.size()) {
    throw std::invalid_argument("finalConfig: fewer arc angles than path segments");
  }

  Config current = start;
  for (std::size_t i = 0; i < type.size(); ++i) {
    current = applySegment(current, angles[i], radius, sphere_radius, type[i]);
  }
  return current;
}

std::vector<Solution> allPaths(
  const Config & start, const Config & goal, double radius, double sphere_radius,
  double tolerance)
{
  const Mat3 goal_unit = toUnitSphereFrame(start, goal, sphere_radius);
  const double ratio = radius / sphere_radius;

  std::vector<Solution> solutions;
  solutions.reserve(16);

  for (const char * type : {"lgl", "rgr", "lgr", "rgl", "lrl", "rlr"}) {
    appendThreeSegmentPaths(goal_unit, radius, sphere_radius, type, tolerance, solutions);
  }

  if (ratio > 1.0 / std::sqrt(2.0)) {
    for (const char * type : {"lrl", "rlr"}) {
      appendCPiCPaths(goal_unit, radius, sphere_radius, type, tolerance, solutions);
    }
  }

  if (ratio > 0.5) {
    for (const char * type : {"lrlr", "rlrl"}) {
      appendFourSegmentPaths(goal_unit, radius, sphere_radius, type, tolerance, solutions);
    }
  }

  if (ratio > 1.0 / std::sqrt(2.0)) {
    for (const char * type : {"lrlrl", "rlrlr"}) {
      appendFiveSegmentPaths(goal_unit, radius, sphere_radius, type, tolerance, solutions);
    }
  }

  return solutions;
}

Solution optimalPath(
  const Config & start, const Config & goal, double radius, double sphere_radius,
  double tolerance)
{
  Solution best;
  for (Solution & candidate : allPaths(start, goal, radius, sphere_radius, tolerance)) {
    if (candidate.valid() && candidate.length < best.length) {
      best = std::move(candidate);
    }
  }
  return best;
}

double optimalPathLength(
  const Config & start, const Config & goal, double radius, double sphere_radius,
  double tolerance)
{
  double best = kInfinity;
  for (const Solution & candidate : allPaths(start, goal, radius, sphere_radius, tolerance)) {
    if (candidate.valid()) {
      best = std::min(best, candidate.length);
    }
  }
  return best;
}

Samples pathSamples(
  const Config & start, double radius, double sphere_radius,
  const std::vector<double> & angles, const std::string & type, double spacing)
{
  Samples samples;
  if (angles.size() < type.size()) {
    return samples;
  }

  Config current = start;
  for (std::size_t i = 0; i < type.size(); ++i) {
    const char segment = type[i];
    const double phi = angles[i];
    const double arc_length = (segment == 'g') ? phi * sphere_radius : phi * radius;

    // Each segment contributes its own start and interior points but not its
    // end point, which belongs to the next segment; the end of the path is
    // appended once after the loop. Sampling the closed interval instead would
    // drop the start of any segment shorter than `spacing`, because such a
    // segment gets a single sample that is then discarded as a junction.
    const int count = sampleCount(arc_length, spacing);
    samples.positions.reserve(samples.positions.size() + static_cast<std::size_t>(count) + 1U);
    samples.tangents.reserve(samples.tangents.size() + static_cast<std::size_t>(count) + 1U);
    for (int k = 0; k < count; ++k) {
      const double at_angle = phi * static_cast<double>(k) / static_cast<double>(count);
      const Config at = applySegment(current, at_angle, radius, sphere_radius, segment);
      samples.positions.push_back(at.col(0));
      samples.tangents.push_back(at.col(1));
    }

    current = applySegment(current, phi, radius, sphere_radius, segment);
  }

  samples.positions.push_back(current.col(0));
  samples.tangents.push_back(current.col(1));
  return samples;
}

}  // namespace dubins_path_3d::sphere

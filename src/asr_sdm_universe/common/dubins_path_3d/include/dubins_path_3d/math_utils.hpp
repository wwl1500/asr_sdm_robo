// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__MATH_UTILS_HPP_
#define DUBINS_PATH_3D__MATH_UTILS_HPP_

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

#include "dubins_path_3d/types.hpp"

namespace dubins_path_3d
{

/// Wraps an angle to [0, 2*pi).
inline double wrapTwoPi(double angle)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  double wrapped = std::fmod(angle, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

/// Wraps an angle to (-pi, pi].
inline double wrapPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Clamps a value that should lie in [-1, 1] before being passed to acos/asin,
/// rejecting values that are outside the interval by more than `tolerance`.
inline bool snapToUnitInterval(double & value, double tolerance)
{
  if (std::abs(value) > 1.0) {
    if (std::abs(value) <= 1.0 + tolerance) {
      value = (value > 0.0) ? 1.0 : -1.0;
    } else {
      return false;
    }
  }
  return true;
}

/// Number of samples used for an arc of the given length, matching
/// `ceil(length / spacing)` of the reference implementation. Zero-length
/// segments yield no samples.
inline int sampleCount(double arc_length, double spacing)
{
  if (!(arc_length > 0.0) || !(spacing > 0.0)) {
    return 0;
  }
  return static_cast<int>(std::ceil(arc_length / spacing));
}

/// i-th value of `count` points spread over [start, stop], mirroring
/// numpy.linspace. A single point sits at `start`.
inline double linspaceValue(double start, double stop, int count, int index)
{
  if (count <= 1) {
    return start;
  }
  return start + (stop - start) * static_cast<double>(index) / static_cast<double>(count - 1);
}

/// i-th value of `count` points spread over [start, stop), mirroring
/// numpy.linspace(..., endpoint=False).
inline double linspaceValueOpen(double start, double stop, int count, int index)
{
  if (count <= 0) {
    return start;
  }
  return start + (stop - start) * static_cast<double>(index) / static_cast<double>(count);
}

/// Picks a unit vector orthogonal to `axis` by orthonormalising the first
/// coordinate axis that is sufficiently independent of it.
Vec3 orthogonalUnitVector(const Vec3 & axis, double tolerance = 1e-2);

/// Unit vector orthogonal to `axis`, orthonormalising whichever column of
/// `reference` has the largest component orthogonal to the axis.
///
/// Pass the body frame of the start configuration to obtain a direction that
/// rotates with the problem. That makes the parameter sweeps of the
/// intermediary surfaces equivariant under rigid motions, so translating and
/// rotating a query does not change the planned length. Selecting a global
/// coordinate axis instead would tie the sweep to the world frame.
///
/// `reference` must have orthonormal columns and `axis` must be a unit vector.
Vec3 orthogonalUnitVector(const Vec3 & axis, const Mat3 & reference);

/// Roots of a*x^3 + b*x^2 + c*x + d = 0, degrading gracefully to the quadratic
/// and linear cases when the leading coefficients vanish.
std::vector<std::complex<double>> solveCubic(double a, double b, double c, double d);

/// Real parts of every root of the cubic.
///
/// Candidates taken from complex roots are harmless because each one is later
/// checked against the requested final configuration; keeping them recovers
/// near-double roots that round to a complex conjugate pair.
std::vector<double> solveCubicRealParts(double a, double b, double c, double d);

}  // namespace dubins_path_3d

#endif  // DUBINS_PATH_3D__MATH_UTILS_HPP_

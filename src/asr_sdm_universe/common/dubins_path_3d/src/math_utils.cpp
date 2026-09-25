// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include "dubins_path_3d/math_utils.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace dubins_path_3d
{

namespace
{

/// Real cube root, defined for negative arguments as well.
double cubeRoot(double value)
{
  return std::cbrt(value);
}

}  // namespace

Vec3 orthogonalUnitVector(const Vec3 & axis, double tolerance)
{
  const Vec3 unit_axis = axis.normalized();
  const std::array<Vec3, 3> candidates{Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};
  for (const Vec3 & candidate : candidates) {
    const Vec3 projected = candidate - candidate.dot(unit_axis) * unit_axis;
    if (projected.norm() > tolerance) {
      return projected.normalized();
    }
  }
  throw std::runtime_error("orthogonalUnitVector: axis is not a valid direction");
}

Vec3 orthogonalUnitVector(const Vec3 & axis, const Mat3 & reference)
{
  const Vec3 unit_axis = axis.normalized();

  // The three columns are orthonormal, so their components orthogonal to the
  // axis have squared norms summing to two: the largest is at least 2/3 and the
  // choice is well conditioned for every axis.
  int best_column = 0;
  double best_norm = -1.0;
  for (int column = 0; column < 3; ++column) {
    const Vec3 projected =
      reference.col(column) - reference.col(column).dot(unit_axis) * unit_axis;
    const double norm = projected.norm();
    if (norm > best_norm) {
      best_norm = norm;
      best_column = column;
    }
  }

  if (!(best_norm > 0.0)) {
    throw std::runtime_error(
      "orthogonalUnitVector: the reference frame is degenerate or not orthonormal");
  }

  const Vec3 chosen = reference.col(best_column);
  return (chosen - chosen.dot(unit_axis) * unit_axis).normalized();
}

std::vector<std::complex<double>> solveCubic(double a, double b, double c, double d)
{
  using Complex = std::complex<double>;

  if (a == 0.0 && b == 0.0) {
    if (c == 0.0) {
      return {};
    }
    return {Complex(-d / c, 0.0)};
  }

  if (a == 0.0) {
    const double discriminant = c * c - 4.0 * b * d;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      return {Complex((-c + root) / (2.0 * b), 0.0), Complex((-c - root) / (2.0 * b), 0.0)};
    }
    const double root = std::sqrt(-discriminant);
    return {
      Complex(-c / (2.0 * b), root / (2.0 * b)),
      Complex(-c / (2.0 * b), -root / (2.0 * b))};
  }

  // Depressed cubic t^3 + f*t + g with t = x + b/(3a).
  const double f = ((3.0 * c / a) - (b * b) / (a * a)) / 3.0;
  const double g = ((2.0 * b * b * b) / (a * a * a) - (9.0 * b * c) / (a * a) +
    (27.0 * d / a)) / 27.0;
  const double h = (g * g) / 4.0 + (f * f * f) / 27.0;
  const double shift = b / (3.0 * a);

  if (f == 0.0 && g == 0.0 && h == 0.0) {
    const double root = -cubeRoot(d / a);
    return {Complex(root, 0.0), Complex(root, 0.0), Complex(root, 0.0)};
  }

  if (h <= 0.0) {
    // Three real roots, expressed through the trigonometric solution.
    const double i = std::sqrt((g * g) / 4.0 - h);
    const double j = cubeRoot(i);
    double cos_argument = -(g / (2.0 * i));
    cos_argument = std::clamp(cos_argument, -1.0, 1.0);
    const double k = std::acos(cos_argument);
    const double m = std::cos(k / 3.0);
    const double n = std::sqrt(3.0) * std::sin(k / 3.0);

    return {
      Complex(2.0 * j * m - shift, 0.0),
      Complex(-j * (m + n) - shift, 0.0),
      Complex(-j * (m - n) - shift, 0.0)};
  }

  const double r = -(g / 2.0) + std::sqrt(h);
  const double s = cubeRoot(r);
  const double t = -(g / 2.0) - std::sqrt(h);
  const double u = cubeRoot(t);

  const double real_part = -(s + u) / 2.0 - shift;
  const double imaginary_part = (s - u) * std::sqrt(3.0) * 0.5;
  return {
    Complex((s + u) - shift, 0.0),
    Complex(real_part, imaginary_part),
    Complex(real_part, -imaginary_part)};
}

std::vector<double> solveCubicRealParts(double a, double b, double c, double d)
{
  const std::vector<std::complex<double>> roots = solveCubic(a, b, c, d);
  std::vector<double> real_parts;
  real_parts.reserve(roots.size());
  for (const std::complex<double> & root : roots) {
    real_parts.push_back(root.real());
  }
  return real_parts;
}

}  // namespace dubins_path_3d

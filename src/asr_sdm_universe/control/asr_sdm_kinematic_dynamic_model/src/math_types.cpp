#include "asr_sdm_kinematic_dynamic_model/math_types.hpp"

#include <algorithm>
#include <cmath>

namespace asr_sdm_kinematic_dynamic_model
{

Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(double s, const Vec3 & a)
{
  return {s * a.x, s * a.y, s * a.z};
}

Vec3 operator*(const Vec3 & a, double s)
{
  return s * a;
}

Vec3 operator/(const Vec3 & a, double s)
{
  return {a.x / s, a.y / s, a.z / s};
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double norm(const Vec3 & a)
{
  return std::sqrt(dot(a, a));
}

Vec3 normalize(const Vec3 & a)
{
  const double n = norm(a);
  return n < 1.0e-12 ? Vec3{1.0, 0.0, 0.0} : a / n;
}

Vec3 projectPerpendicular(const Vec3 & value, const Vec3 & axis)
{
  return value - axis * dot(axis, value);
}

Vec3 column(const Mat3 & m, std::size_t j)
{
  return {m.v[0][j], m.v[1][j], m.v[2][j]};
}

Mat3 fromColumns(const Vec3 & c0, const Vec3 & c1, const Vec3 & c2)
{
  return {{{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}}};
}

Mat3 identityFrame()
{
  return fromColumns({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0});
}

Mat3 multiply(const Mat3 & a, const Mat3 & b)
{
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      for (std::size_t k = 0; k < 3; ++k) {
        out.v[i][j] += a.v[i][k] * b.v[k][j];
      }
    }
  }
  return out;
}

Vec3 multiply(const Mat3 & m, const Vec3 & a)
{
  return {
    m.v[0][0] * a.x + m.v[0][1] * a.y + m.v[0][2] * a.z,
    m.v[1][0] * a.x + m.v[1][1] * a.y + m.v[1][2] * a.z,
    m.v[2][0] * a.x + m.v[2][1] * a.y + m.v[2][2] * a.z};
}

Mat3 rotationY(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

Mat3 rotationZ(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 frameFromAxis(const Vec3 & axis)
{
  const Vec3 ex = normalize(axis);
  const Vec3 reference = std::abs(dot(ex, {0.0, 0.0, 1.0})) > 0.92 ?
    Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0};
  const Vec3 ey = normalize(cross(reference, ex));
  const Vec3 ez = cross(ex, ey);
  return fromColumns(ex, ey, ez);
}

Mat3 orthonormalize(const Mat3 & frame)
{
  const Vec3 ex = normalize(column(frame, 0));
  Vec3 ey = projectPerpendicular(column(frame, 1), ex);
  if (norm(ey) < 1.0e-9) {
    return frameFromAxis(ex);
  }
  ey = normalize(ey);
  return fromColumns(ex, ey, cross(ex, ey));
}

}  // namespace asr_sdm_kinematic_dynamic_model

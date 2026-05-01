#ifndef POINT_HPP_
#define POINT_HPP_


#include "base.hpp"

namespace asr
{
namespace geometry
{

struct Point
{
  Real x{0};
  Real y{0};
  Real z{0};

  Point() = default;
  Point(Real x_, Real y_, Real z_ = static_cast<Real>(0)) : x(x_), y(y_), z(z_) {}

  // ---- arithmetic ----
  Point operator+(const Point & p) const { return {x + p.x, y + p.y, z + p.z}; }
  Point operator-(const Point & p) const { return {x - p.x, y - p.y, z - p.z}; }
  Point operator-() const { return {-x, -y, -z}; }
  Point operator*(Real d) const { return {x * d, y * d, z * d}; }
  Point operator/(Real d) const { return {x / d, y / d, z / d}; }

  Point & operator+=(const Point & p) { x += p.x, y += p.y, z += p.z; return *this; }
  Point & operator-=(const Point & p) { x -= p.x, y -= p.y, z -= p.z; return *this; }
  Point & operator*=(Real d) { x *= d, y *= d, z *= d; return *this; }
  Point & operator/=(Real d) { x /= d, y /= d, z /= d; return *this; }

  // ---- comparison (epsilon-aware) ----
  bool operator==(const Point & p) const
  {
    return equals(x, p.x) && equals(y, p.y) && equals(z, p.z);
  }
  bool operator!=(const Point & p) const { return !(*this == p); }
};

inline Point operator*(Real d, const Point & p) { return p * d; }

// ---- magnitude ----
inline Real norm(const Point & p) { return p.x * p.x + p.y * p.y + p.z * p.z; }
inline Real abs(const Point & p) { return std::sqrt(norm(p)); }

// ---- I/O ----
inline istream & operator>>(istream & is, Point & p) { return is >> p.x >> p.y >> p.z; }
inline ostream & operator<<(ostream & os, const Point & p)
{
  return os << p.x << " " << p.y << " " << p.z;
}

// ---- vector products ----
inline Real dot(const Point & a, const Point & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 3D cross product (vector-valued).
inline Point cross(const Point & a, const Point & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Scalar (z-component) cross product for 2D / xy-plane algorithms.
inline Real cross_2d(const Point & a, const Point & b) { return a.x * b.y - a.y * b.x; }

// ---- rotations ----
// Rotate p counterclockwise by theta rad around the z-axis (xy-plane).
inline Point rotate(Real theta, const Point & p)
{
  const Real c = std::cos(theta);
  const Real s = std::sin(theta);
  return {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
}

// Rotate p around an arbitrary axis by theta rad (Rodrigues' formula).
inline Point rotate(Real theta, const Point & axis, const Point & p)
{
  const Real n = abs(axis);
  if (equals(n, 0)) return p;
  const Point k{axis.x / n, axis.y / n, axis.z / n};
  const Real c = std::cos(theta);
  const Real s = std::sin(theta);
  return p * c + cross(k, p) * s + k * (dot(k, p) * (1 - c));
}

// ---- ordering helpers ----
inline bool compare_x(const Point & a, const Point & b)
{
  if (!equals(a.x, b.x)) return a.x < b.x;
  if (!equals(a.y, b.y)) return a.y < b.y;
  return a.z < b.z;
}

inline bool compare_y(const Point & a, const Point & b)
{
  if (!equals(a.y, b.y)) return a.y < b.y;
  if (!equals(a.x, b.x)) return a.x < b.x;
  return a.z < b.z;
}

inline bool compare_z(const Point & a, const Point & b)
{
  if (!equals(a.z, b.z)) return a.z < b.z;
  if (!equals(a.x, b.x)) return a.x < b.x;
  return a.y < b.y;
}

using Points = vector<Point>;

}  // namespace geometry
}  // namespace asr

#endif  // POINT_HPP_

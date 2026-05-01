#ifndef COMMON_AREA_CP_HPP_
#define COMMON_AREA_CP_HPP_


#include "cross_point_cl.hpp"
#include "distance_sp.hpp"
#include "is_intersect_cs.hpp"
#include "polygon.hpp"

namespace asr
{
namespace geometry
{

namespace detail
{

// Signed area enclosed by triangle (c.p, a, b) intersected with circle c (xy-plane).
// Helper for `common_area_cp`.
inline Real common_area_cp_impl(const Circle & c, const Point & a, const Point & b)
{
  const Point va = c.p - a;
  const Point vb = c.p - b;
  const Real f = cross_2d(va, vb);
  if (sign(f) == 0) return 0;
  if (sign(max(abs(va), abs(vb)) - c.r) <= 0) return f;

  if (sign(distance_sp(Segment(a, b), c.p) - c.r) >= 0) {
    // signed angle from va to vb
    return c.r * c.r * atan2(cross_2d(va, vb), dot(va, vb));
  }

  auto tot = cross_point_cl(c, Line(a, b));
  if (is_intersect_cs(c, Segment(a, b)) != 2 && dot(a - tot[0], b - tot[0]) < 0) {
    swap(tot[0], tot[1]);
  }
  tot.emplace(begin(tot), a);
  tot.emplace_back(b);

  Real ret = 0;
  for (int i = 1; i < static_cast<int>(tot.size()); ++i) {
    ret += common_area_cp_impl(c, tot[i - 1], tot[i]);
  }
  return ret;
}

}  // namespace detail

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_7_H
// Area of intersection between circle c and polygon p (xy-plane).
inline Real common_area_cp(const Circle & c, const Polygon & p)
{
  if (p.size() < 3) return 0;
  const int n = static_cast<int>(p.size());
  Real A = 0;
  for (int i = 0; i < n; ++i) {
    A += detail::common_area_cp_impl(c, p[i], p[(i + 1) % n]);
  }
  return A * 0.5;
}

}  // namespace geometry
}  // namespace asr

#endif  // COMMON_AREA_CP_HPP_

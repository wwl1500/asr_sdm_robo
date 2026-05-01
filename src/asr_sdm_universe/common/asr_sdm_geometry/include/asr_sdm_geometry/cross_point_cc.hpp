#ifndef CROSS_POINT_CC_HPP_
#define CROSS_POINT_CC_HPP_


#include "circle.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_7_E
// Intersection of two circles in the xy-plane.
inline Points cross_point_cc(const Circle & c1, const Circle & c2)
{
  const Real d = abs(c1.p - c2.p);
  if (sign(d - (c1.r + c2.r)) > 0) return {};
  if (sign(d + c1.r - c2.r) < 0) return {};
  const Real a = acos((c1.r * c1.r - c2.r * c2.r + d * d) / (2 * c1.r * d));
  const Point delta = c2.p - c1.p;
  const Real t = atan2(delta.y, delta.x);
  const Point p = c1.p + Point(c1.r * cos(t + a), c1.r * sin(t + a));
  const Point q = c1.p + Point(c1.r * cos(t - a), c1.r * sin(t - a));
  if (p == q) return {p};
  return {p, q};
}

}  // namespace geometry
}  // namespace asr

#endif  // CROSS_POINT_CC_HPP_

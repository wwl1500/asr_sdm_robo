#ifndef CROSS_POINT_CL_HPP_
#define CROSS_POINT_CL_HPP_


#include "circle.hpp"
#include "projection.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_7_D
inline Points cross_point_cl(const Circle & c, const Line & l)
{
  const Point pr = projection(l, c.p);
  if (equals(abs(pr - c.p), c.r)) return {pr};
  const Point e = (l.b - l.a) / abs(l.b - l.a);
  const Real k = sqrt(c.r * c.r - norm(pr - c.p));
  return {pr - e * k, pr + e * k};
}

}  // namespace geometry
}  // namespace asr

#endif  // CROSS_POINT_CL_HPP_

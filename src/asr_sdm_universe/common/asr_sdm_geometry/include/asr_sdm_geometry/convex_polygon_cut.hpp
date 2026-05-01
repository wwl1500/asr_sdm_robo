#ifndef CONVEX_POLYGON_CUT_HPP_
#define CONVEX_POLYGON_CUT_HPP_


#include "cross_point_ll.hpp"
#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_C
// Cuts polygon U with line l (xy-plane) and returns the convex polygon on the left.
inline Polygon convex_polygon_cut(const Polygon & U, const Line & l)
{
  Polygon ret;
  const int n = static_cast<int>(U.size());
  for (int i = 0; i < n; ++i) {
    const Point & now = U[i];
    const Point & nxt = U[(i + 1) % n];
    const Real cf = cross_2d(l.a - now, l.b - now);
    const Real cs = cross_2d(l.a - nxt, l.b - nxt);
    if (sign(cf) >= 0) ret.emplace_back(now);
    if (sign(cf) * sign(cs) < 0) ret.emplace_back(cross_point_ll(Line(now, nxt), l));
  }
  return ret;
}

}  // namespace geometry
}  // namespace asr

#endif  // CONVEX_POLYGON_CUT_HPP_

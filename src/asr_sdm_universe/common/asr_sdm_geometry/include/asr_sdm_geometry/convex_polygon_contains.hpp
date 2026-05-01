#ifndef CONVEX_POLYGON_CONTAINS_HPP_
#define CONVEX_POLYGON_CONTAINS_HPP_


#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// Point-in-convex-polygon test in the xy-plane (binary search). O(log n).
inline int convex_polygon_contains(const Polygon & Q, const Point & p)
{
  const int N = static_cast<int>(Q.size());
  const Point g = (Q[0] + Q[N / 3] + Q[N * 2 / 3]) / 3.0;
  if (equals(g.x, p.x) && equals(g.y, p.y)) return IN;
  const Point gp = p - g;
  int l = 0, r = N;
  while (r - l > 1) {
    const int mid = (l + r) / 2;
    const Point gl = Q[l] - g;
    const Point gm = Q[mid] - g;
    if (cross_2d(gl, gm) > 0) {
      if (cross_2d(gl, gp) >= 0 && cross_2d(gm, gp) <= 0)
        r = mid;
      else
        l = mid;
    } else {
      if (cross_2d(gl, gp) <= 0 && cross_2d(gm, gp) >= 0)
        l = mid;
      else
        r = mid;
    }
  }
  r %= N;
  const Real v = cross_2d(Q[l] - p, Q[r] - p);
  return sign(v) == 0 ? ON : sign(v) == -1 ? OUT : IN;
}

}  // namespace geometry
}  // namespace asr

#endif  // CONVEX_POLYGON_CONTAINS_HPP_

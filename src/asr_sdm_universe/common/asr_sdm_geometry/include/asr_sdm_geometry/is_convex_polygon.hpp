#ifndef IS_CONVEX_POLYGON_HPP_
#define IS_CONVEX_POLYGON_HPP_


#include "ccw.hpp"
#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_B
inline bool is_convex_polygon(const Polygon & p)
{
  const int n = static_cast<int>(p.size());
  for (int i = 0; i < n; ++i) {
    if (ccw(p[(i + n - 1) % n], p[i], p[(i + 1) % n]) == CLOCKWISE) return false;
  }
  return true;
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_CONVEX_POLYGON_HPP_

#ifndef CONVEX_POLYGON_DIAMETER_HPP_
#define CONVEX_POLYGON_DIAMETER_HPP_


#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_B
// Rotating-calipers diameter of a convex polygon (xy-plane).
// Returns the indices (i, j) of the diametrically opposite vertices.
inline pair<int, int> convex_polygon_diameter(const Polygon & p)
{
  const int N = static_cast<int>(p.size());
  int is = 0;
  int js = 0;
  for (int i = 1; i < N; ++i) {
    if (p[i].y > p[is].y) is = i;
    if (p[i].y < p[js].y) js = i;
  }
  Real maxdis = norm(p[is] - p[js]);

  int maxi = is;
  int maxj = js;
  int i = is;
  int j = js;
  do {
    if (cross_2d(p[(i + 1) % N] - p[i], p[(j + 1) % N] - p[j]) >= 0) {
      j = (j + 1) % N;
    } else {
      i = (i + 1) % N;
    }
    if (norm(p[i] - p[j]) > maxdis) {
      maxdis = norm(p[i] - p[j]);
      maxi = i;
      maxj = j;
    }
  } while (i != is || j != js);
  return minmax(maxi, maxj);
}

}  // namespace geometry
}  // namespace asr

#endif  // CONVEX_POLYGON_DIAMETER_HPP_

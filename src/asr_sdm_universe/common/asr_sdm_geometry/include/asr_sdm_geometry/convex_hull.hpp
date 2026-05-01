#ifndef CONVEX_HULL_HPP_
#define CONVEX_HULL_HPP_


#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_A
// Andrew's monotone chain in the xy-plane.
// If `strict`, collinear points are dropped from the hull.
inline Polygon convex_hull(Polygon & p, bool strict = true)
{
  const int n = static_cast<int>(p.size());
  if (n <= 2) return p;
  sort(begin(p), end(p), compare_x);

  int k = 0;
  vector<Point> ch(2 * n);
  const auto check = [&](int i) {
    return sign(cross_2d(ch[k - 1] - ch[k - 2], p[i] - ch[k - 1])) <= -1 + strict;
  };
  for (int i = 0; i < n; ch[k++] = p[i++]) {
    while (k >= 2 && check(i)) --k;
  }
  for (int i = n - 2, t = k + 1; i >= 0; ch[k++] = p[i--]) {
    while (k >= t && check(i)) --k;
  }
  ch.resize(k - 1);
  return ch;
}

}  // namespace geometry
}  // namespace asr

#endif  // CONVEX_HULL_HPP_

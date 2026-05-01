#ifndef CONTAINS_HPP_
#define CONTAINS_HPP_


#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_C
// Returns IN, ON, or OUT depending on whether p lies inside, on, or outside Q (xy-plane).
inline int contains(const Polygon & Q, const Point & p)
{
  bool in = false;
  const int n = static_cast<int>(Q.size());
  for (int i = 0; i < n; ++i) {
    Point a = Q[i] - p;
    Point b = Q[(i + 1) % n] - p;
    if (a.y > b.y) swap(a, b);
    if (sign(a.y) <= 0 && sign(b.y) > 0 && sign(cross_2d(a, b)) < 0) in = !in;
    if (equals(cross_2d(a, b), 0) && sign(dot(a, b)) <= 0) return ON;
  }
  return in ? IN : OUT;
}

}  // namespace geometry
}  // namespace asr

#endif  // CONTAINS_HPP_

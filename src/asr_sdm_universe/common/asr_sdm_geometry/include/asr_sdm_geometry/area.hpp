#ifndef AREA_HPP_
#define AREA_HPP_


#include "polygon.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_3_A
// Signed area of a simple polygon in the xy-plane.
inline Real area(const Polygon & p)
{
  const int n = static_cast<int>(p.size());
  Real A = 0;
  for (int i = 0; i < n; ++i) {
    A += cross_2d(p[i], p[(i + 1) % n]);
  }
  return A * 0.5;
}

}  // namespace geometry
}  // namespace asr

#endif  // AREA_HPP_

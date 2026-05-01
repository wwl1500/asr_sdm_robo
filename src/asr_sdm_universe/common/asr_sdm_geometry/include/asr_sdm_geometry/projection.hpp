#ifndef PROJECTION_HPP_
#define PROJECTION_HPP_


#include "line.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_A
inline Point projection(const Line & l, const Point & p)
{
  const Real t = dot(p - l.a, l.a - l.b) / norm(l.a - l.b);
  return l.a + (l.a - l.b) * t;
}

}  // namespace geometry
}  // namespace asr

#endif  // PROJECTION_HPP_

#ifndef REFLECTION_HPP_
#define REFLECTION_HPP_


#include "projection.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_B
inline Point reflection(const Line & l, const Point & p)
{
  return p + (projection(l, p) - p) * 2;
}

}  // namespace geometry
}  // namespace asr

#endif  // REFLECTION_HPP_

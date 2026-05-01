#ifndef CROSS_POINT_LL_HPP_
#define CROSS_POINT_LL_HPP_


#include "line.hpp"

namespace asr
{
namespace geometry
{

inline Point cross_point_ll(const Line & l, const Line & m)
{
  const Real A = cross_2d(l.b - l.a, m.b - m.a);
  const Real B = cross_2d(l.b - l.a, l.b - m.a);
  if (equals(A, 0) && equals(B, 0)) return m.a;
  return m.a + (m.b - m.a) * (B / A);
}

}  // namespace geometry
}  // namespace asr

#endif  // CROSS_POINT_LL_HPP_

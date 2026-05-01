#ifndef IS_INTERSECT_LL_HPP_
#define IS_INTERSECT_LL_HPP_


#include "is_parallel.hpp"
#include "line.hpp"

namespace asr
{
namespace geometry
{

inline bool is_intersect_ll(const Line & l, const Line & m)
{
  const Real A = cross_2d(l.b - l.a, m.b - m.a);
  const Real B = cross_2d(l.b - l.a, l.b - m.a);
  if (equals(A, 0) && equals(B, 0)) return true;
  return !is_parallel(l, m);
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_INTERSECT_LL_HPP_

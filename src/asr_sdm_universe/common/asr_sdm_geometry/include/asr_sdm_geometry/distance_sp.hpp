#ifndef DISTANCE_SP_HPP_
#define DISTANCE_SP_HPP_


#include "is_intersect_sp.hpp"
#include "projection.hpp"
#include "segment.hpp"

namespace asr
{
namespace geometry
{

inline Real distance_sp(const Segment & s, const Point & p)
{
  const Point r = projection(s, p);
  if (is_intersect_sp(s, r)) return abs(r - p);
  return min(abs(s.a - p), abs(s.b - p));
}

}  // namespace geometry
}  // namespace asr

#endif  // DISTANCE_SP_HPP_

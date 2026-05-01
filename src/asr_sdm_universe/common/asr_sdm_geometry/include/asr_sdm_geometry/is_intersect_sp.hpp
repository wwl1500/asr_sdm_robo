#ifndef IS_INTERSECT_SP_HPP_
#define IS_INTERSECT_SP_HPP_


#include "ccw.hpp"
#include "segment.hpp"

namespace asr
{
namespace geometry
{

inline bool is_intersect_sp(const Segment & s, const Point & p)
{
  return ccw(s.a, s.b, p) == ON_SEGMENT;
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_INTERSECT_SP_HPP_

#ifndef IS_INTERSECT_CS_HPP_
#define IS_INTERSECT_CS_HPP_


#include "circle.hpp"
#include "projection.hpp"
#include "segment.hpp"

namespace asr
{
namespace geometry
{

// Returns the number of intersections between circle c and segment l (0, 1, or 2).
inline int is_intersect_cs(const Circle & c, const Segment & l)
{
  const Point h = projection(l, c.p);
  if (sign(norm(h - c.p) - c.r * c.r) > 0) return 0;
  const Real d1 = abs(c.p - l.a);
  const Real d2 = abs(c.p - l.b);
  if (sign(c.r - d1) >= 0 && sign(c.r - d2) >= 0) return 0;
  if ((sign(c.r - d1) < 0 && sign(d2 - c.r) > 0) ||
      (sign(d1 - c.r) > 0 && sign(c.r - d2) < 0))
    return 1;
  if (sign(dot(l.a - h, l.b - h)) < 0) return 2;
  return 0;
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_INTERSECT_CS_HPP_

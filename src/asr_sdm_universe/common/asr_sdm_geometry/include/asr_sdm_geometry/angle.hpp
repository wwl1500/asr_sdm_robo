#ifndef ANGLE_HPP_
#define ANGLE_HPP_


#include "point.hpp"

namespace asr
{
namespace geometry
{

inline Real radian_to_degree(Real theta) { return theta * 180.0 / PI; }
inline Real degree_to_radian(Real deg) { return deg * PI / 180.0; }

// Smaller angle (in rad) of the corner at b in the triple (a, b, c), in xy-plane.
inline Real get_smaller_angle(const Point & a, const Point & b, const Point & c)
{
  const Point v = a - b;
  const Point w = c - b;
  Real alpha = atan2(v.y, v.x);
  Real beta = atan2(w.y, w.x);
  if (alpha > beta) swap(alpha, beta);
  const Real theta = beta - alpha;
  return min(theta, 2 * PI - theta);
}

}  // namespace geometry
}  // namespace asr

#endif  // ANGLE_HPP_

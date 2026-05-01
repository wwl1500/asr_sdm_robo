#ifndef DISTANCE_LP_HPP_
#define DISTANCE_LP_HPP_


#include "projection.hpp"

namespace asr
{
namespace geometry
{

inline Real distance_lp(const Line & l, const Point & p)
{
  return abs(p - projection(l, p));
}

}  // namespace geometry
}  // namespace asr

#endif  // DISTANCE_LP_HPP_

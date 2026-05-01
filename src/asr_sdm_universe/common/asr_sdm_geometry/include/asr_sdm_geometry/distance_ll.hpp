#ifndef DISTANCE_LL_HPP_
#define DISTANCE_LL_HPP_


#include "distance_lp.hpp"
#include "is_intersect_ll.hpp"

namespace asr
{
namespace geometry
{

inline Real distance_ll(const Line & l, const Line & m)
{
  return is_intersect_ll(l, m) ? 0 : distance_lp(l, m.a);
}

}  // namespace geometry
}  // namespace asr

#endif  // DISTANCE_LL_HPP_

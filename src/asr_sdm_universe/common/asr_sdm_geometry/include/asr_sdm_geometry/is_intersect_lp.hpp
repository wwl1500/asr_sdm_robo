#ifndef IS_INTERSECT_LP_HPP_
#define IS_INTERSECT_LP_HPP_


#include "ccw.hpp"
#include "line.hpp"

namespace asr
{
namespace geometry
{

inline bool is_intersect_lp(const Line & l, const Point & p)
{
  return std::abs(ccw(l.a, l.b, p)) != 1;
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_INTERSECT_LP_HPP_

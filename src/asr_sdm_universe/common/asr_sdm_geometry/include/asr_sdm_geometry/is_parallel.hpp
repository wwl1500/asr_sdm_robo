#ifndef IS_PARALLEL_HPP_
#define IS_PARALLEL_HPP_


#include "line.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_A
inline bool is_parallel(const Line & a, const Line & b)
{
  return equals(cross_2d(a.b - a.a, b.b - b.a), 0);
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_PARALLEL_HPP_

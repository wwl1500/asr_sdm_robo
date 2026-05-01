#ifndef IS_ORTHOGONAL_HPP_
#define IS_ORTHOGONAL_HPP_


#include "line.hpp"

namespace asr
{
namespace geometry
{

// http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_2_A
inline bool is_orthogonal(const Line & a, const Line & b)
{
  return equals(dot(a.a - a.b, b.a - b.b), 0);
}

}  // namespace geometry
}  // namespace asr

#endif  // IS_ORTHOGONAL_HPP_

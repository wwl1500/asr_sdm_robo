#ifndef BASE_HPP_
#define BASE_HPP_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>
#include <utility>
#include <vector>

namespace geometry
{

using Real = double;
inline constexpr Real EPS = static_cast<Real>(1e-6);
inline const Real PI = std::acos(static_cast<Real>(-1));

enum { OUT, ON, IN };

inline int sign(const Real & r)
{
  return r <= -EPS ? -1 : r >= EPS ? 1 : 0;
}
inline bool equals(const Real & a, const Real & b)
{
  return sign(a - b) == 0;
}

// Curated re-exports so library headers don't have to std::-qualify every name.
using std::abs;
using std::acos;
using std::atan2;
using std::begin;
using std::cos;
using std::end;
using std::istream;
using std::max;
using std::min;
using std::minmax;
using std::ostream;
using std::pair;
using std::sin;
using std::sort;
using std::sqrt;
using std::swap;
using std::vector;

}  // namespace geometry

#endif  // BASE_HPP_

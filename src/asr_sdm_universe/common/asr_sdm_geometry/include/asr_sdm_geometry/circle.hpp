#ifndef CIRCLE_HPP_
#define CIRCLE_HPP_


#include "point.hpp"

namespace asr
{
namespace geometry
{

struct Circle
{
  Point p;
  Real r{0};

  Circle() = default;
  Circle(const Point & p_, Real r_) : p(p_), r(r_) {}
};

using Circles = vector<Circle>;

}  // namespace geometry
}  // namespace asr

#endif  // CIRCLE_HPP_

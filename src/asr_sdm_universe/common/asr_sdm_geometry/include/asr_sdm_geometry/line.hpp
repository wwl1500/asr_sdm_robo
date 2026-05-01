#ifndef LINE_HPP_
#define LINE_HPP_


#include "point.hpp"

namespace asr
{
namespace geometry
{

struct Line
{
  Point a;
  Point b;

  Line() = default;
  Line(const Point & a_, const Point & b_) : a(a_), b(b_) {}

  // Construct the line Ax + By = C in the xy-plane (z = 0).
  Line(Real A, Real B, Real C)
  {
    if (equals(A, 0)) {
      assert(!equals(B, 0));
      a = Point(0, C / B);
      b = Point(1, C / B);
    } else if (equals(B, 0)) {
      a = Point(C / A, 0);
      b = Point(C / A, 1);
    } else {
      a = Point(0, C / B);
      b = Point(C / A, 0);
    }
  }

  friend ostream & operator<<(ostream & os, const Line & l)
  {
    return os << l.a << " to " << l.b;
  }
  friend istream & operator>>(istream & is, Line & l) { return is >> l.a >> l.b; }
};

using Lines = vector<Line>;

}  // namespace geometry
}  // namespace asr

#endif  // LINE_HPP_

#ifndef SEGMENT_HPP_
#define SEGMENT_HPP_


#include "line.hpp"

namespace asr
{
namespace geometry
{

struct Segment : Line
{
  Segment() = default;
  using Line::Line;
};

using Segments = vector<Segment>;

}  // namespace geometry
}  // namespace asr

#endif  // SEGMENT_HPP_

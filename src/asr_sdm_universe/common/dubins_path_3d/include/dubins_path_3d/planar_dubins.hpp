// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__PLANAR_DUBINS_HPP_
#define DUBINS_PATH_3D__PLANAR_DUBINS_HPP_

#include <array>
#include <string>
#include <vector>

#include "dubins_path_3d/types.hpp"

namespace dubins_path_3d::planar
{

/// Configuration on the plane: position and heading.
struct Config
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
};

/// Parameters of a three-segment planar Dubins path. For a CSC path the entries
/// are (arc angle, straight length, arc angle); for a CCC path they are three arc
/// angles.
struct Solution
{
  std::string type;
  double length{kInfinity};
  std::array<double, 3> params{{0.0, 0.0, 0.0}};

  bool valid() const {return std::isfinite(length);}
};

/// Applies a left turn ('l'), right turn ('r') or straight segment ('s').
///
/// For turns `param` is the arc angle and `radius` the turning radius; for
/// straight segments `param` is the length and `radius` is ignored.
Config applySegment(const Config & start, double param, double radius, char segment);

/// Configuration reached after following `type` with the given `params`.
Config finalConfig(
  const Config & start, double radius, const std::array<double, 3> & params,
  const std::string & type);

/// Samples a single segment, spacing points about `spacing` metres apart.
///
/// The number of points matches the reference implementation:
/// ceil(arc_length / spacing), which means very short segments yield a single
/// point at the start of the segment.
std::vector<Config> segmentSamples(
  const Config & start, double param, double radius, char segment, double spacing);

/// Samples a full path, dropping the duplicated junction points between
/// consecutive segments.
std::vector<Config> pathSamples(
  const Config & start, double radius, const std::array<double, 3> & params,
  const std::string & type, double spacing);

/// Closed-form CSC path ("lsl", "rsr", "lsr", "rsl").
Solution cscPath(
  const Config & start, const Config & goal, double radius,
  const std::string & type);

/// Closed-form CCC path ("lrl", "rlr"). Only the branch whose middle arc exceeds
/// pi is returned, since that is the one that can be optimal.
Solution cccPath(
  const Config & start, const Config & goal, double radius,
  const std::string & type);

/// Shortest of the six Dubins families.
Solution optimalPath(const Config & start, const Config & goal, double radius);

/// The six path types considered, in the order used by the reference code.
const std::vector<std::string> & pathTypes();

}  // namespace dubins_path_3d::planar

#endif  // DUBINS_PATH_3D__PLANAR_DUBINS_HPP_

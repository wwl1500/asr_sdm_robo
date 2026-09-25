#ifndef SPATIAL_REEDS_SHEPP_HPP_
#define SPATIAL_REEDS_SHEPP_HPP_

#include <Eigen/Core>

#include <vector>

namespace shortest_curve_path
{

/** SE(2)×pitch pose. Heading follows R = Rz(yaw) * Ry(pitch), so body +x is
 *  (cos(pitch) cos(yaw), cos(pitch) sin(yaw), -sin(pitch)). */
struct Pose3d
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double pitch = 0.0;
};

/** Constant-rate segment. Arc length is unsigned; gear = +1 forward, -1 reverse.
 *  Yaw/pitch rates are derivatives with respect to arc length and stay inside
 *  ±1/ρ_yaw and ±1/ρ_pitch. */
struct Path3dSegment
{
  double length = 0.0;
  double gear = 1.0;
  double kappa_yaw = 0.0;
  double kappa_pitch = 0.0;
};

struct Path3d
{
  std::vector<Path3dSegment> segments;
  double length = 0.0;
  double position_error = 0.0;

  bool empty() const { return segments.empty(); }
  bool feasible(double tol = 1.0e-3) const { return !empty() && position_error <= tol; }
};

/** Constructive 3-D Reeds–Shepp analogue: CSC over a free intermediate heading,
 *  with reverse and independent yaw / pitch turning radii. Heading change uses
 *  parallel (time-optimal) and sequential yaw/pitch primitives. Position is
 *  matched by searching the intermediate heading; the path is shortest among
 *  those families, not a proven spatial optimum. */
class SpatialReedsShepp
{
public:
  SpatialReedsShepp(double yaw_radius, double pitch_radius);

  double yawRadius() const { return yaw_radius_; }
  double pitchRadius() const { return pitch_radius_; }

  Path3d plan(const Pose3d & start, const Pose3d & goal) const;
  Pose3d sample(const Pose3d & start, const Path3d & path, double s) const;
  std::vector<Pose3d> discretize(const Pose3d & start, const Path3d & path, double ds) const;

  static Eigen::Vector3d heading(double yaw, double pitch);
  static double wrapToPi(double angle);

private:
  double yaw_radius_;
  double pitch_radius_;
};

}  // namespace shortest_curve_path

#endif  // SPATIAL_REEDS_SHEPP_HPP_

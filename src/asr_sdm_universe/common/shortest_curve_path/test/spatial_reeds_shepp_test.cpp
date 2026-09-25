#include "shortest_curve_path/spatial_reeds_shepp.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iostream>

using shortest_curve_path::Path3d;
using shortest_curve_path::Pose3d;
using shortest_curve_path::SpatialReedsShepp;

namespace
{

constexpr double kPi = 3.14159265358979323846;

int failures = 0;

void expect(bool cond, const char * msg)
{
  if (!cond) {
    ++failures;
    std::cerr << "FAIL: " << msg << '\n';
  }
}

void checkCase(
  const char * name, const SpatialReedsShepp & planner, const Pose3d & start, const Pose3d & goal,
  double pos_tol, double heading_tol)
{
  const Path3d path = planner.plan(start, goal);
  const Pose3d end = planner.sample(start, path, path.length);
  const double pos_err = (end.position - goal.position).norm();
  const double yaw_err = std::abs(SpatialReedsShepp::wrapToPi(end.yaw - goal.yaw));
  const double pitch_err = std::abs(end.pitch - goal.pitch);

  std::cout << name << ": length=" << path.length << " pos_err=" << pos_err
            << " reported=" << path.position_error << " yaw_err=" << yaw_err
            << " pitch_err=" << pitch_err << " segs=" << path.segments.size() << '\n';

  expect(path.feasible(pos_tol) || pos_err <= pos_tol, name);
  expect(pos_err <= pos_tol, name);
  expect(yaw_err <= heading_tol, name);
  expect(pitch_err <= heading_tol, name);

  for (const auto & segment : path.segments) {
    expect(segment.length >= -1.0e-12, name);
    expect(std::abs(segment.gear) == 1.0, name);
    expect(std::abs(segment.kappa_yaw) <= 1.0 / planner.yawRadius() + 1.0e-9, name);
    expect(std::abs(segment.kappa_pitch) <= 1.0 / planner.pitchRadius() + 1.0e-9, name);
  }
}

}  // namespace

int main()
{
  const SpatialReedsShepp planner(2.0, 1.5);

  Pose3d start;
  Pose3d goal;

  goal.position = Eigen::Vector3d(4.0, 0.0, 0.0);
  checkCase("straight", planner, start, goal, 1.0e-3, 1.0e-3);

  goal = {};
  goal.position = Eigen::Vector3d(-3.0, 0.0, 0.0);
  checkCase("reverse", planner, start, goal, 2.0e-3, 1.0e-3);

  goal = {};
  goal.position = Eigen::Vector3d(0.0, 4.0, 0.0);
  goal.yaw = 0.5 * kPi;
  checkCase("yaw_left", planner, start, goal, 2.0e-2, 2.0e-3);

  goal = {};
  goal.position = Eigen::Vector3d(1.5, 0.0, -1.5);
  goal.pitch = 0.5 * kPi - 1.0e-3;
  checkCase("pitch_down", planner, start, goal, 3.0e-2, 5.0e-3);

  goal = {};
  goal.position = Eigen::Vector3d(5.0, 3.0, -1.2);
  goal.yaw = 0.7;
  goal.pitch = 0.25;
  checkCase("spatial_forward", planner, start, goal, 5.0e-2, 5.0e-3);

  start.yaw = 0.4;
  start.pitch = -0.15;
  start.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  goal.position = Eigen::Vector3d(4.5, 2.5, -0.8);
  goal.yaw = -0.9;
  goal.pitch = 0.35;
  checkCase("spatial_offset", planner, start, goal, 8.0e-2, 8.0e-3);

  if (failures > 0) {
    std::cerr << failures << " check(s) failed\n";
    return 1;
  }
  std::cout << "all checks passed\n";
  return 0;
}

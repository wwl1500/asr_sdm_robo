#include "asr_sdm_kinematic_dynamic_model/underwater_visualization.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace
{

bool check(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

}  // namespace

int main()
{
  using namespace asr_sdm_kinematic_dynamic_model;
  bool passed = true;
  const Eigen::Matrix3d mapped = displayRotation(true);
  passed &= check((mapped * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitX()),
    "z to x mapping");
  passed &= check(std::abs(mapped.determinant() - 1.0) < 1.0e-12, "mapping orientation");
  passed &= check(displayRotation(false).isApprox(Eigen::Matrix3d::Identity()), "identity mapping");

  const auto quaternion = toRosQuaternion(
    Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix());
  passed &= check(std::abs(quaternion.w - std::sqrt(0.5)) < 1.0e-12, "quaternion conversion");
  passed &= check(std::abs(quaternion.y - std::sqrt(0.5)) < 1.0e-12, "quaternion axis");

  builtin_interfaces::msg::Time stamp;
  const auto arrow = makeForceMarker(
    "world", "fluid", 3, stamp, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
    0.5, std::array<float, 4>{1.0F, 0.0F, 0.0F, 1.0F});
  passed &= check(arrow.action == visualization_msgs::msg::Marker::ADD, "force marker add");
  passed &= check(arrow.id == 3 && arrow.ns == "fluid", "force marker identity");
  const auto deleted = makeForceMarker(
    "world", "fluid", 3, stamp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    0.5, std::array<float, 4>{1.0F, 0.0F, 0.0F, 1.0F});
  passed &= check(deleted.action == visualization_msgs::msg::Marker::DELETE, "zero force delete");

  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  const auto trajectory = makeTrajectoryMarker(
    "world", stamp, points, 0.01, std::array<float, 4>{1.0F, 1.0F, 1.0F, 1.0F});
  passed &= check(trajectory.type == visualization_msgs::msg::Marker::LINE_STRIP,
    "trajectory marker type");
  passed &= check(trajectory.points.size() == 2, "trajectory points");

  std::cout << (passed ? "Underwater visualization regression passed" :
  "Underwater visualization regression failed") << std::endl;
  return passed ? 0 : 1;
}

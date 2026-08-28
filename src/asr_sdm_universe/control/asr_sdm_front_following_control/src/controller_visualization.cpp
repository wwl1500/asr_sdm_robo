#include "asr_sdm_front_following_control/controller_visualization.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace asr_sdm_controller
{
namespace
{

constexpr double kArrowLengthScale = 0.01;
constexpr double kArrowMinLength = 0.05;
constexpr double kArrowMaxLength = 0.60;
constexpr double kHalfPi = 1.5707963267948966;
constexpr double kRadiansToDegrees = 57.29577951308232;

geometry_msgs::msg::Point makePoint(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std::string formatTorquePair(const double pitch_torque, const double yaw_torque)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "tau=[" << pitch_torque << ", " << yaw_torque << "]";
  return stream.str();
}

geometry_msgs::msg::Quaternion normalizeQuaternion(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  Eigen::Quaterniond eigen_quaternion(
    quaternion.w,
    quaternion.x,
    quaternion.y,
    quaternion.z);

  if (eigen_quaternion.norm() < 1.0e-9) {
    geometry_msgs::msg::Quaternion identity;
    identity.w = 1.0;
    return identity;
  }

  eigen_quaternion.normalize();

  geometry_msgs::msg::Quaternion normalized;
  normalized.x = eigen_quaternion.x();
  normalized.y = eigen_quaternion.y();
  normalized.z = eigen_quaternion.z();
  normalized.w = eigen_quaternion.w();
  return normalized;
}

std::array<double, 3> computeRollPitchYawDegrees(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  const auto normalized = normalizeQuaternion(quaternion);
  const double x = normalized.x;
  const double y = normalized.y;
  const double z = normalized.z;
  const double w = normalized.w;

  const double sin_roll_cos_pitch = 2.0 * (w * x + y * z);
  const double cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

  const double sin_pitch = 2.0 * (w * y - z * x);
  const double pitch =
    std::abs(sin_pitch) >= 1.0 ? std::copysign(kHalfPi, sin_pitch) : std::asin(sin_pitch);

  const double sin_yaw_cos_pitch = 2.0 * (w * z + x * y);
  const double cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

  const auto sanitize_degrees = [](const double radians) {
      const double degrees = radians * kRadiansToDegrees;
      return std::abs(degrees) < 0.05 ? 0.0 : degrees;
    };

  return {
    sanitize_degrees(roll),
    sanitize_degrees(pitch),
    sanitize_degrees(yaw)};
}

std::string formatOrientationSummary(const asr_sdm_control_msgs::msg::RobotState & state)
{
  const auto rpy_degrees = computeRollPitchYawDegrees(state.orientation);

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1);
  stream << "rpy_deg=["
         << rpy_degrees[0] << ", "
         << rpy_degrees[1] << ", "
         << rpy_degrees[2] << "]";
  return stream.str();
}

}  // namespace

std::array<std::string, 6> controllerJointNames()
{
  return {
    "joint1_pitch", "joint1_yaw",
    "joint2_pitch", "joint2_yaw",
    "joint3_pitch", "joint3_yaw"};
}

std::array<std::string, 4> propulsionFrameNames()
{
  return {"base_link", "link_2", "link_3", "link_4"};
}

std::array<std::string, 3> intermediateJointFrameNames()
{
  return {"joint1_intermediate", "joint2_intermediate", "joint3_intermediate"};
}

sensor_msgs::msg::JointState buildJointStateMessage(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;

  const auto joint_names = controllerJointNames();
  message.name.assign(joint_names.begin(), joint_names.end());
  message.position.assign(state.joint_positions.begin(), state.joint_positions.end());
  message.velocity.assign(state.joint_velocities.begin(), state.joint_velocities.end());
  return message;
}

geometry_msgs::msg::TransformStamped buildBaseTransformMessage(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const std::string & world_frame,
  const std::string & base_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = world_frame;
  message.child_frame_id = base_frame;
  message.transform.translation.x = state.position.x;
  message.transform.translation.y = state.position.y;
  message.transform.translation.z = state.position.z;
  message.transform.rotation = state.orientation;
  return message;
}

bool extractPlanarPose(
  const std_msgs::msg::Float64MultiArray & message,
  std::array<double, 3> & planar_pose)
{
  if (message.data.size() < 3) {
    return false;
  }

  if (!std::isfinite(message.data[0]) ||
    !std::isfinite(message.data[1]) ||
    !std::isfinite(message.data[2]))
  {
    return false;
  }

  planar_pose[0] = message.data[0];
  planar_pose[1] = message.data[1];
  planar_pose[2] = message.data[2];
  return true;
}

geometry_msgs::msg::PoseStamped buildPlanarPoseStamped(
  const std::array<double, 3> & planar_pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.pose.position = makePoint(planar_pose[0], planar_pose[1], 0.0);

  const Eigen::Quaterniond quaternion(
    Eigen::AngleAxisd(planar_pose[2], Eigen::Vector3d::UnitZ()));
  message.pose.orientation.x = quaternion.x();
  message.pose.orientation.y = quaternion.y();
  message.pose.orientation.z = quaternion.z();
  message.pose.orientation.w = quaternion.w();
  return message;
}

double computeEuclideanNorm(const std_msgs::msg::Float64MultiArray & message)
{
  double squared_norm = 0.0;
  for (const double value : message.data) {
    if (!std::isfinite(value)) {
      return std::numeric_limits<double>::infinity();
    }
    squared_norm += value * value;
  }
  return std::sqrt(squared_norm);
}

visualization_msgs::msg::Marker buildDeleteAllMarker(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

visualization_msgs::msg::Marker buildFrontPointMarker(
  const std::array<double, 3> & planar_pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const int marker_id,
  const bool desired)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = desired ? "front_point_desired" : "front_point_actual";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = makePoint(planar_pose[0], planar_pose[1], 0.0);
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.08;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  if (desired) {
    marker.color.r = 0.95F;
    marker.color.g = 0.55F;
    marker.color.b = 0.15F;
  } else {
    marker.color.r = 0.10F;
    marker.color.g = 0.80F;
    marker.color.b = 0.95F;
  }
  marker.color.a = 1.0F;
  return marker;
}

visualization_msgs::msg::Marker buildBodyHeadingMarker(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const int marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "body_heading";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = state.position;
  marker.pose.position.z += 0.10;
  marker.pose.orientation = normalizeQuaternion(state.orientation);
  marker.scale.x = 0.22;
  marker.scale.y = 0.035;
  marker.scale.z = 0.050;
  marker.color.r = 0.98F;
  marker.color.g = 0.86F;
  marker.color.b = 0.20F;
  marker.color.a = 0.75F;
  return marker;
}

visualization_msgs::msg::Marker buildBodyOrientationTextMarker(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const int marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "body_orientation";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = makePoint(
    state.position.x + 0.45,
    state.position.y + 0.28,
    state.position.z + 0.52);
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.075;
  marker.color.r = 0.75F;
  marker.color.g = 0.92F;
  marker.color.b = 0.98F;
  marker.color.a = 0.92F;
  marker.text = formatOrientationSummary(state);
  return marker;
}

visualization_msgs::msg::Marker buildPropulsionForceMarker(
  const std::string & frame_id,
  const int marker_id,
  const double propulsion_force,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "propulsion_forces";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.025;
  marker.scale.y = 0.050;
  marker.scale.z = 0.080;

  const double sign = propulsion_force >= 0.0 ? 1.0 : -1.0;
  const double arrow_length = std::clamp(
    std::abs(propulsion_force) * kArrowLengthScale,
    kArrowMinLength,
    kArrowMaxLength);
  marker.points.push_back(makePoint(0.0, 0.0, 0.0));
  marker.points.push_back(makePoint(sign * arrow_length, 0.0, 0.0));

  if (propulsion_force >= 0.0) {
    marker.color.r = 0.15F;
    marker.color.g = 0.85F;
    marker.color.b = 0.25F;
  } else {
    marker.color.r = 0.90F;
    marker.color.g = 0.20F;
    marker.color.b = 0.20F;
  }
  marker.color.a = 1.0F;
  return marker;
}

visualization_msgs::msg::Marker buildJointTorqueMarker(
  const std::string & frame_id,
  const int marker_id,
  const double pitch_torque,
  const double yaw_torque,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "joint_torques";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = makePoint(0.0, 0.0, 0.12);
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.09;
  marker.color.r = 1.0F;
  marker.color.g = 1.0F;
  marker.color.b = 1.0F;
  marker.color.a = 1.0F;
  marker.text = formatTorquePair(pitch_torque, yaw_torque);
  return marker;
}

visualization_msgs::msg::Marker buildStatusTextMarker(
  const std::string & frame_id,
  const int marker_id,
  const double x,
  const double y,
  const double z,
  const std::string & text,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = "controller_status";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = makePoint(x, y, z);
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.08;
  marker.color.r = 0.98F;
  marker.color.g = 0.95F;
  marker.color.b = 0.60F;
  marker.color.a = 0.88F;
  marker.text = text;
  return marker;
}

}  // namespace asr_sdm_controller

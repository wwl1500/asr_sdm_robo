#pragma once

#include "asr_sdm_control_msgs/msg/actuator_force_cmd.hpp"
#include "asr_sdm_control_msgs/msg/robot_state.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <array>
#include <string>

namespace asr_sdm_controller
{

std::array<std::string, 6> controllerJointNames();
std::array<std::string, 4> propulsionFrameNames();
std::array<std::string, 3> intermediateJointFrameNames();

sensor_msgs::msg::JointState buildJointStateMessage(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const builtin_interfaces::msg::Time & stamp);

geometry_msgs::msg::TransformStamped buildBaseTransformMessage(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const std::string & world_frame,
  const std::string & base_frame,
  const builtin_interfaces::msg::Time & stamp);

bool extractPlanarPose(
  const std_msgs::msg::Float64MultiArray & message,
  std::array<double, 3> & planar_pose);

geometry_msgs::msg::PoseStamped buildPlanarPoseStamped(
  const std::array<double, 3> & planar_pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id);

double computeEuclideanNorm(const std_msgs::msg::Float64MultiArray & message);

visualization_msgs::msg::Marker buildDeleteAllMarker(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id);

visualization_msgs::msg::Marker buildFrontPointMarker(
  const std::array<double, 3> & planar_pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  int marker_id,
  bool desired);

visualization_msgs::msg::Marker buildBodyHeadingMarker(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  int marker_id);

visualization_msgs::msg::Marker buildBodyOrientationTextMarker(
  const asr_sdm_control_msgs::msg::RobotState & state,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  int marker_id);

visualization_msgs::msg::Marker buildPropulsionForceMarker(
  const std::string & frame_id,
  int marker_id,
  double propulsion_force,
  const builtin_interfaces::msg::Time & stamp);

visualization_msgs::msg::Marker buildJointTorqueMarker(
  const std::string & frame_id,
  int marker_id,
  double pitch_torque,
  double yaw_torque,
  const builtin_interfaces::msg::Time & stamp);

visualization_msgs::msg::Marker buildStatusTextMarker(
  const std::string & frame_id,
  int marker_id,
  double x,
  double y,
  double z,
  const std::string & text,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace asr_sdm_controller

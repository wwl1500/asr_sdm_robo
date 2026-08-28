#include "asr_sdm_front_following_control/controller_visualization.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <string>

namespace asr_sdm_controller
{
namespace
{

constexpr double kHalfPi = 1.5707963267948966;

asr_sdm_control_msgs::msg::RobotState makeRobotState()
{
  asr_sdm_control_msgs::msg::RobotState state;
  state.position.x = 1.2;
  state.position.y = -0.5;
  state.position.z = 0.3;
  state.orientation.x = 0.1;
  state.orientation.y = -0.2;
  state.orientation.z = 0.3;
  state.orientation.w = 0.9;
  state.linear_velocity_body.x = 0.4;
  state.linear_velocity_body.y = -0.1;
  state.linear_velocity_body.z = 0.2;
  state.angular_velocity_body.x = 0.05;
  state.angular_velocity_body.y = -0.03;
  state.angular_velocity_body.z = 0.07;

  for (size_t index = 0; index < state.joint_positions.size(); ++index) {
    state.joint_positions[index] = 0.1 * static_cast<double>(index + 1);
    state.joint_velocities[index] = -0.2 * static_cast<double>(index + 1);
  }

  return state;
}

builtin_interfaces::msg::Time makeStamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;
  return stamp;
}

geometry_msgs::msg::Quaternion makeQuaternionFromYaw(const double yaw)
{
  const Eigen::Quaterniond quaternion(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

Eigen::Vector3d extractForwardDirection(const geometry_msgs::msg::Quaternion & quaternion)
{
  const Eigen::Quaterniond eigen_quaternion(
    quaternion.w,
    quaternion.x,
    quaternion.y,
    quaternion.z);
  return eigen_quaternion * Eigen::Vector3d::UnitX();
}

}  // namespace

TEST(ControllerVisualizationTest, JointStateAndBaseTransformMatchRobotState)
{
  const auto state = makeRobotState();
  const auto stamp = makeStamp();

  const auto joint_state = buildJointStateMessage(state, stamp);
  ASSERT_EQ(joint_state.name.size(), 6U);
  ASSERT_EQ(joint_state.position.size(), 6U);
  ASSERT_EQ(joint_state.velocity.size(), 6U);
  EXPECT_EQ(joint_state.name[0], "joint1_pitch");
  EXPECT_EQ(joint_state.name[5], "joint3_yaw");
  EXPECT_DOUBLE_EQ(joint_state.position[2], state.joint_positions[2]);
  EXPECT_DOUBLE_EQ(joint_state.velocity[4], state.joint_velocities[4]);

  const auto transform = buildBaseTransformMessage(state, "world", "base_link", stamp);
  EXPECT_EQ(transform.header.frame_id, "world");
  EXPECT_EQ(transform.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, state.position.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, state.position.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, state.position.z);
  EXPECT_DOUBLE_EQ(transform.transform.rotation.x, state.orientation.x);
  EXPECT_DOUBLE_EQ(transform.transform.rotation.w, state.orientation.w);
}

TEST(ControllerVisualizationTest, PlanarPoseAndMarkersAreFinite)
{
  std_msgs::msg::Float64MultiArray message;
  message.data = {0.8, -0.2, 1.1};

  std::array<double, 3> planar_pose{};
  ASSERT_TRUE(extractPlanarPose(message, planar_pose));
  EXPECT_DOUBLE_EQ(planar_pose[0], 0.8);
  EXPECT_DOUBLE_EQ(planar_pose[1], -0.2);
  EXPECT_DOUBLE_EQ(planar_pose[2], 1.1);

  const auto stamp = makeStamp();
  const auto pose = buildPlanarPoseStamped(planar_pose, stamp, "world");
  EXPECT_EQ(pose.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 0.8);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, -0.2);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.0);
  EXPECT_NEAR(pose.pose.orientation.w, 0.8525245, 1.0e-6);

  const auto propulsion_marker =
    buildPropulsionForceMarker("link_2", 3, -12.0, stamp);
  ASSERT_EQ(propulsion_marker.points.size(), 2U);
  EXPECT_EQ(propulsion_marker.header.frame_id, "link_2");
  EXPECT_LT(propulsion_marker.points[1].x, 0.0);

  const auto joint_marker =
    buildJointTorqueMarker("joint1_intermediate", 7, 1.25, -0.75, stamp);
  EXPECT_EQ(joint_marker.header.frame_id, "joint1_intermediate");
  EXPECT_NE(joint_marker.text.find("1.25"), std::string::npos);
  EXPECT_NE(joint_marker.text.find("-0.75"), std::string::npos);

  const auto status_marker =
    buildStatusTextMarker("world", 9, 0.0, 0.0, 0.5, "status: ok", stamp);
  EXPECT_EQ(status_marker.header.frame_id, "world");
  EXPECT_EQ(status_marker.text, "status: ok");
}

TEST(ControllerVisualizationTest, BodyHeadingMarkerTracksRobotForwardDirection)
{
  auto state = makeRobotState();
  const auto stamp = makeStamp();

  state.orientation = makeQuaternionFromYaw(0.0);
  const auto marker_yaw_zero = buildBodyHeadingMarker(state, stamp, "world", 15);
  EXPECT_EQ(marker_yaw_zero.header.frame_id, "world");
  EXPECT_EQ(marker_yaw_zero.ns, "body_heading");
  EXPECT_EQ(marker_yaw_zero.type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_DOUBLE_EQ(marker_yaw_zero.pose.position.x, state.position.x);
  EXPECT_DOUBLE_EQ(marker_yaw_zero.pose.position.y, state.position.y);
  EXPECT_DOUBLE_EQ(marker_yaw_zero.pose.position.z, state.position.z + 0.10);

  const auto forward_zero = extractForwardDirection(marker_yaw_zero.pose.orientation);
  EXPECT_NEAR(forward_zero.x(), 1.0, 1.0e-6);
  EXPECT_NEAR(forward_zero.y(), 0.0, 1.0e-6);
  EXPECT_NEAR(forward_zero.z(), 0.0, 1.0e-6);

  state.orientation = makeQuaternionFromYaw(kHalfPi);
  const auto marker_yaw_ninety = buildBodyHeadingMarker(state, stamp, "world", 16);
  const auto forward_ninety = extractForwardDirection(marker_yaw_ninety.pose.orientation);
  EXPECT_NEAR(forward_ninety.x(), 0.0, 1.0e-6);
  EXPECT_NEAR(forward_ninety.y(), 1.0, 1.0e-6);
  EXPECT_NEAR(forward_ninety.z(), 0.0, 1.0e-6);
}

TEST(ControllerVisualizationTest, BodyOrientationTextMarkerFormatsDegrees)
{
  auto state = makeRobotState();
  state.orientation = makeQuaternionFromYaw(kHalfPi);

  const auto marker = buildBodyOrientationTextMarker(state, makeStamp(), "world", 17);
  EXPECT_EQ(marker.header.frame_id, "world");
  EXPECT_EQ(marker.ns, "body_orientation");
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(marker.text, "rpy_deg=[0.0, 0.0, 90.0]");
  EXPECT_DOUBLE_EQ(marker.pose.position.x, state.position.x + 0.45);
  EXPECT_DOUBLE_EQ(marker.pose.position.y, state.position.y + 0.28);
  EXPECT_DOUBLE_EQ(marker.pose.position.z, state.position.z + 0.52);
}

}  // namespace asr_sdm_controller

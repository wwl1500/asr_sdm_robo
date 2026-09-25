#include "asr_sdm_head_following_control/front_unit_following_controller_3d.hpp"
#include "asr_sdm_head_following_control/head_command_mpc.hpp"
#include "asr_sdm_kinematic_dynamic_model/asr_sdm_kinematic_model.hpp"

#include "asr_sdm_control_msgs/msg/actuator_cmd.hpp"
#include "asr_sdm_control_msgs/msg/robot_command.hpp"
#include "asr_sdm_control_msgs/msg/unit_cmd.hpp"

#include <asr_sdm_log_collector/log_client.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr double kUrdfJointLimit = 1.5707963267948966;
constexpr size_t kStateSize3D = 43;

const std::array<std::string, 6> kArticulationJointNames = {
  "joint_joint_unit_a__joint_unit_cross__0",
  "joint_joint_unit_cross__joint_unit_b__0",
  "joint_joint_unit_a__joint_unit_cross__1",
  "joint_joint_unit_cross__joint_unit_b__1",
  "joint_joint_unit_a__joint_unit_cross__2",
  "joint_joint_unit_cross__joint_unit_b__2",
};

const std::array<std::string, 8> kRotorJointNames = {
  "joint_screwdrive_segment_0__screw_rotor_left_0__0",
  "joint_screwdrive_segment_0__screw_rotor_right_0__0",
  "joint_screwdrive_segment_0__screw_rotor_left_0__1",
  "joint_screwdrive_segment_0__screw_rotor_right_0__1",
  "joint_screwdrive_segment_0__screw_rotor_left_0__2",
  "joint_screwdrive_segment_0__screw_rotor_right_0__2",
  "joint_screwdrive_segment_0__screw_rotor_left_0__3",
  "joint_screwdrive_segment_0__screw_rotor_right_0__3",
};

int32_t scaleToInt32(double value, double scale, int32_t min_value, int32_t max_value)
{
  const double scaled = value * scale;
  const double clamped = std::clamp(
    scaled, static_cast<double>(min_value), static_cast<double>(max_value));
  return static_cast<int32_t>(std::lround(clamped));
}

geometry_msgs::msg::Quaternion quaternionFromFrame(const asr::Mat3 & frame)
{
  const double trace = frame.v[0][0] + frame.v[1][1] + frame.v[2][2];
  geometry_msgs::msg::Quaternion quaternion;
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    quaternion.w = 0.25 * scale;
    quaternion.x = (frame.v[2][1] - frame.v[1][2]) / scale;
    quaternion.y = (frame.v[0][2] - frame.v[2][0]) / scale;
    quaternion.z = (frame.v[1][0] - frame.v[0][1]) / scale;
  } else if (frame.v[0][0] > frame.v[1][1] && frame.v[0][0] > frame.v[2][2]) {
    const double scale = 2.0 * std::sqrt(1.0 + frame.v[0][0] - frame.v[1][1] - frame.v[2][2]);
    quaternion.w = (frame.v[2][1] - frame.v[1][2]) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (frame.v[0][1] + frame.v[1][0]) / scale;
    quaternion.z = (frame.v[0][2] + frame.v[2][0]) / scale;
  } else if (frame.v[1][1] > frame.v[2][2]) {
    const double scale = 2.0 * std::sqrt(1.0 + frame.v[1][1] - frame.v[0][0] - frame.v[2][2]);
    quaternion.w = (frame.v[0][2] - frame.v[2][0]) / scale;
    quaternion.x = (frame.v[0][1] + frame.v[1][0]) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (frame.v[1][2] + frame.v[2][1]) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + frame.v[2][2] - frame.v[0][0] - frame.v[1][1]);
    quaternion.w = (frame.v[1][0] - frame.v[0][1]) / scale;
    quaternion.x = (frame.v[0][2] + frame.v[2][0]) / scale;
    quaternion.y = (frame.v[1][2] + frame.v[2][1]) / scale;
    quaternion.z = 0.25 * scale;
  }

  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

}  // namespace

class AsrSdmControlManager
{
public:
  void init(const rclcpp::Node::SharedPtr & nh)
  {
    node_ = nh;
    loadParameters();

    model_ = std::make_unique<asr::AsrSdmKinematicModel>(model_params_);
    controller_ = std::make_unique<asr::FrontUnitFollowingController3D>(controller_params_);
    if (enable_grampc_compensation_) {
      mpc_ = std::make_unique<asr::HeadCommandMPC>(mpc_params_);
    }

    state_ = controller_->makeInitialState();
    resetState(initial_x_, initial_y_, initial_z_, initial_yaw_);
    last_cmd_time_ = node_->now();
    last_control_time_ = node_->now();

    sub_robot_cmd_ = node_->create_subscription<asr_sdm_control_msgs::msg::RobotCommand>(
      robot_cmd_topic_, rclcpp::QoS(10),
      std::bind(&AsrSdmControlManager::onRobotCmd, this, std::placeholders::_1));
    sub_initialpose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10),
      std::bind(&AsrSdmControlManager::onInitialPose, this, std::placeholders::_1));
    pub_controller_state_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      controller_state_topic_, rclcpp::QoS(10));
    pub_joint_state_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(10));
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    if (publish_control_cmd_) {
      pub_control_cmd_ = node_->create_publisher<asr_sdm_control_msgs::msg::ActuatorCmd>(
        control_cmd_topic_, rclcpp::QoS(1));
    }

    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&AsrSdmControlManager::onControlTimer, this));

    SPDLOG_INFO(
      "ASR SDM control manager started: model=URDF-free Pinocchio, robot_cmd={}, state={}, grampc={}",
      robot_cmd_topic_, controller_state_topic_, enable_grampc_compensation_ ? "on" : "off");
  }

private:
  void loadParameters()
  {
    model_params_.link_length = node_->declare_parameter<double>(
      "kinematic_controller.link_length", 0.25);
    model_params_.link_mass = node_->declare_parameter<double>(
      "kinematic_controller.link_mass", 1.5);
    model_params_.link_radius = node_->declare_parameter<double>(
      "kinematic_controller.link_radius", 0.05);
    model_params_.joint_limit = node_->declare_parameter<double>(
      "kinematic_controller.joint_limit", kUrdfJointLimit);
    model_params_.joint_velocity_limit = node_->declare_parameter<double>(
      "kinematic_controller.joint_velocity_limit", 2.0);
    model_params_.joint_effort_limit = node_->declare_parameter<double>(
      "kinematic_controller.joint_effort_limit", 50.0);
    controller_params_.link_length = model_params_.link_length;
    controller_params_.joint_rate_limit = node_->declare_parameter<double>(
      "kinematic_controller.joint_rate_limit", 2.0);
    controller_params_.joint_limit = model_params_.joint_limit;
    controller_params_.max_curvature = node_->declare_parameter<double>(
      "kinematic_controller.max_curvature", 1.2);
    controller_params_.curvature_velocity_epsilon = node_->declare_parameter<double>(
      "kinematic_controller.curvature_velocity_epsilon", 1.0e-6);
    controller_params_.damping = node_->declare_parameter<double>(
      "kinematic_controller.damping", 0.02);
    controller_params_.min_linear_velocity = node_->declare_parameter<double>(
      "min_linear_velocity", 0.0);
    controller_params_.max_linear_velocity = node_->declare_parameter<double>(
      "max_linear_velocity", 0.12);
    controller_params_.min_pitch_rate = node_->declare_parameter<double>(
      "min_pitch_rate", -0.35);
    controller_params_.max_pitch_rate = node_->declare_parameter<double>(
      "max_pitch_rate", 0.35);
    controller_params_.min_yaw_rate = node_->declare_parameter<double>(
      "min_yaw_rate", -0.35);
    controller_params_.max_yaw_rate = node_->declare_parameter<double>(
      "max_yaw_rate", 0.35);

    enable_grampc_compensation_ = node_->declare_parameter<bool>(
      "grampc_compensation.enable", true);
    mpc_params_.Thor = node_->declare_parameter<double>(
      "grampc_compensation.Thor", 0.4);
    mpc_params_.Nhor = node_->declare_parameter<int>(
      "grampc_compensation.Nhor", 20);
    mpc_params_.max_grad_iter = node_->declare_parameter<int>(
      "grampc_compensation.max_grad_iter", 4);
    mpc_params_.q_position = node_->declare_parameter<double>(
      "grampc_compensation.q_position", 8.0);
    mpc_params_.q_heading = node_->declare_parameter<double>(
      "grampc_compensation.q_heading", 1.5);
    mpc_params_.r_linear = node_->declare_parameter<double>(
      "grampc_compensation.r_linear", 0.2);
    mpc_params_.r_angular = node_->declare_parameter<double>(
      "grampc_compensation.r_angular", 0.05);
    mpc_params_.terminal_position_scale = node_->declare_parameter<double>(
      "grampc_compensation.terminal_position_scale", 4.0);
    mpc_params_.min_linear_velocity = controller_params_.min_linear_velocity;
    mpc_params_.max_linear_velocity = controller_params_.max_linear_velocity;
    mpc_params_.min_pitch_rate = controller_params_.min_pitch_rate;
    mpc_params_.max_pitch_rate = controller_params_.max_pitch_rate;
    mpc_params_.min_yaw_rate = controller_params_.min_yaw_rate;
    mpc_params_.max_yaw_rate = controller_params_.max_yaw_rate;

    robot_cmd_topic_ = node_->declare_parameter<std::string>(
      "robot_cmd_topic", "/control/asr_sdm/robot_cmd");
    controller_state_topic_ = node_->declare_parameter<std::string>(
      "controller_state_topic", "/control/asr_sdm/controller_state_3d");
    control_cmd_topic_ = node_->declare_parameter<std::string>(
      "control_cmd_topic", "/control/asr_sdm/control_cmd_3d");
    initialpose_topic_ = node_->declare_parameter<std::string>(
      "initialpose_topic", "/control/initial_pose");
    odom_topic_ = node_->declare_parameter<std::string>(
      "odom_topic", "/control/asr_sdm/odom");
    joint_state_topic_ = node_->declare_parameter<std::string>(
      "joint_state_topic", "/control/joint_states");
    world_frame_ = node_->declare_parameter<std::string>("world_frame", "world");
    controller_base_frame_ = node_->declare_parameter<std::string>("controller_base_frame", "base");

    initial_x_ = node_->declare_parameter<double>("initial_x", -5.0);
    initial_y_ = node_->declare_parameter<double>("initial_y", 0.0);
    initial_z_ = node_->declare_parameter<double>("initial_z", 0.0);
    initial_yaw_ = node_->declare_parameter<double>("initial_yaw", 0.0);
    joint_source_indices_ = node_->declare_parameter<std::vector<int64_t>>(
      "joint_source_indices", {0, 1, 2, 3, 4, 5});
    joint_signs_ = node_->declare_parameter<std::vector<double>>(
      "joint_signs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    joint_offsets_rad_ = node_->declare_parameter<std::vector<double>>(
      "joint_offsets_rad", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    clip_joint_positions_ = node_->declare_parameter<bool>("clip_joint_positions", true);
    joint_position_limit_rad_ = node_->declare_parameter<double>(
      "joint_position_limit_rad", model_params_.joint_limit);
    control_period_ms_ = node_->declare_parameter<int>("control_period_ms", 20);
    mpc_params_.dt = static_cast<double>(control_period_ms_) / 1000.0;
    cmd_timeout_sec_ = node_->declare_parameter<double>("cmd_timeout_sec", 0.3);
    screw_velocity_scale_ = node_->declare_parameter<double>(
      "screw_velocity_scale", 21.277);
    publish_control_cmd_ = node_->declare_parameter<bool>("publish_control_cmd", false);
    joint_angle_scale_ = node_->declare_parameter<double>("joint_angle_scale", 1.0);
    joint_angle_limit_ = node_->declare_parameter<int>("joint_angle_limit", 2147483647);

    if (
      joint_source_indices_.size() != kArticulationJointNames.size() ||
      joint_signs_.size() != kArticulationJointNames.size() ||
      joint_offsets_rad_.size() != kArticulationJointNames.size())
    {
      throw std::runtime_error(
              "joint_source_indices, joint_signs, and joint_offsets_rad must each contain six values");
    }
    for (const auto index : joint_source_indices_) {
      if (index < 0 || index >= static_cast<int64_t>(asr::kNum3dJointDofs)) {
        throw std::runtime_error("joint_source_indices values must be in [0, 5]");
      }
    }
  }

  void onRobotCmd(const asr_sdm_control_msgs::msg::RobotCommand::SharedPtr msg)
  {
    if (node_->now() < ignore_robot_cmd_until_) {
      return;
    }
    cmd_raw_ = *msg;
    cmd_cur_ = controller_->toHeadFollowingCommand(*msg);
    last_cmd_time_ = node_->now();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_robot_cmd_log_ >= std::chrono::seconds(1)) {
      last_robot_cmd_log_ = now;
      SPDLOG_INFO(
        "robot_cmd limited -> v={:.3f} pitch={:.3f} yaw={:.3f} pos=({:.3f},{:.3f},{:.3f}) flag={}",
        cmd_cur_.vel.linear.x, cmd_cur_.vel.angular.y, cmd_cur_.vel.angular.z,
        cmd_raw_.position.x, cmd_raw_.position.y, cmd_raw_.position.z,
        static_cast<unsigned>(cmd_raw_.trajectory_flag));
    }
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto & orientation = msg->pose.pose.orientation;
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (norm < 1.0e-6 || !std::isfinite(norm)) {
      SPDLOG_WARN("Ignoring initial pose with an invalid quaternion");
      return;
    }

    // Arm the ignore window before clearing state so a concurrent robot_cmd
    // from traj_server cannot overwrite the reset.
    ignore_robot_cmd_until_ = node_->now() + rclcpp::Duration::from_seconds(0.15);
    resetState(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
      orientation.x / norm, orientation.y / norm,
      orientation.z / norm, orientation.w / norm);
    last_cmd_time_ = node_->now();
    last_control_time_ = node_->now();
    publishControllerState(asr_sdm_control_msgs::msg::RobotCommand{}, {});
    publishRobotState(asr_sdm_control_msgs::msg::RobotCommand{}, {});
    SPDLOG_INFO(
      "pose reset to ({:.3f},{:.3f},{:.3f})",
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void resetState(double x, double y, double z, double yaw)
  {
    resetState(x, y, z, 0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0));
  }

  void resetState(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    state_ = controller_->makeInitialState();
    state_.head_position = {x, y, z};
    state_.head_frame = {{
      {1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qz + qy * qw)},
      {2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz - qx * qw)},
      {2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy)}}};
    cmd_raw_ = asr_sdm_control_msgs::msg::RobotCommand();
    cmd_cur_ = asr_sdm_control_msgs::msg::RobotCommand();
    latest_joint_velocity_ = {};
    rotor_positions_.fill(0.0);
    refreshModelState(cmd_cur_, latest_joint_velocity_);
  }

  Eigen::VectorXd configurationFromState() const
  {
    Eigen::Vector3d head_position(
      state_.head_position.x, state_.head_position.y, state_.head_position.z);
    Eigen::Matrix3d head_frame;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        head_frame(row, col) = state_.head_frame.v[row][col];
      }
    }

    asr::AsrSdmKinematicModel::JointVector theta;
    for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
      theta[static_cast<int>(i)] = state_.joints.theta[i];
    }
    return model_->toConfiguration(head_position, head_frame, theta);
  }

  Eigen::VectorXd velocityFromState(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd,
    const asr::JointVelocity3D & joint_velocity) const
  {
    asr::AsrSdmKinematicModel::JointVector theta_dot;
    for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
      theta_dot[static_cast<int>(i)] = joint_velocity.theta_dot[i];
    }
    return model_->toVelocity(
      cmd.vel.linear.x, cmd.vel.angular.y, cmd.vel.angular.z, theta_dot);
  }

  void copyModelGeometryToState()
  {
    const auto points = model_->computeBodyPoints();
    const auto frames = model_->computeLinkFrames();
    for (size_t point = 0; point < asr::kNum3dPoints; ++point) {
      state_.body_points[point] = {points[point].x(), points[point].y(), points[point].z()};
    }
    for (size_t link = 0; link < asr::kNum3dLinks; ++link) {
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          state_.link_frames[link].v[row][col] = frames[link](row, col);
        }
      }
      state_.link_axes[link] = {
        frames[link](0, 0), frames[link](1, 0), frames[link](2, 0)};
    }
  }

  void refreshModelState(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd, const asr::JointVelocity3D & joint_velocity)
  {
    const Eigen::VectorXd q = configurationFromState();
    const Eigen::VectorXd v = velocityFromState(cmd, joint_velocity);
    model_->updateKinematics(q, v);
    copyModelGeometryToState();
  }

  void advanceModelState(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd, const asr::JointVelocity3D & joint_velocity,
    double dt)
  {
    const Eigen::VectorXd q = configurationFromState();
    const Eigen::VectorXd v = velocityFromState(cmd, joint_velocity);
    const Eigen::VectorXd q_next = model_->integrateConfiguration(q, v, dt);

    Eigen::Vector3d head_position;
    Eigen::Matrix3d head_frame;
    asr::AsrSdmKinematicModel::JointVector theta;
    model_->fromConfiguration(q_next, head_position, head_frame, theta);
    state_.head_position = {head_position.x(), head_position.y(), head_position.z()};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        state_.head_frame.v[row][col] = head_frame(row, col);
      }
    }
    for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
      state_.joints.theta[i] = theta[static_cast<int>(i)];
    }
    state_.time += dt;
    model_->updateKinematics(q_next, v);
    copyModelGeometryToState();
  }

  bool isZeroCommand(const asr_sdm_control_msgs::msg::RobotCommand & cmd) const
  {
    return std::abs(cmd.vel.linear.x) < 1.0e-6 &&
           std::abs(cmd.vel.angular.y) < 1.0e-6 &&
           std::abs(cmd.vel.angular.z) < 1.0e-6;
  }

  void onControlTimer()
  {
    const rclcpp::Time current_time = node_->now();
    double dt = (current_time - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {
      dt = static_cast<double>(control_period_ms_) / 1000.0;
    }
    last_control_time_ = current_time;

    asr_sdm_control_msgs::msg::RobotCommand robot_cmd = cmd_cur_;
    bool timed_out = false;
    if (current_time < ignore_robot_cmd_until_ ||
      (current_time - last_cmd_time_).seconds() > cmd_timeout_sec_)
    {
      robot_cmd = asr_sdm_control_msgs::msg::RobotCommand();
      timed_out = true;
    }
    if (!timed_out && mpc_ && asr::HeadCommandMPC::hasPositionReference(cmd_raw_)) {
      const auto compensated = mpc_->compensate(cmd_raw_, robot_cmd, state_);
      robot_cmd = compensated.command;
      const auto now = std::chrono::steady_clock::now();
      if (compensated.applied && now - last_compensation_log_ >= std::chrono::seconds(1)) {
        last_compensation_log_ = now;
        SPDLOG_INFO(
          "grampc compensation e=({:.3f},{:.3f},{:.3f}) v={:.3f} wy={:.3f} wz={:.3f}",
          compensated.position_error.x, compensated.position_error.y, compensated.position_error.z,
          robot_cmd.vel.linear.x, robot_cmd.vel.angular.y, robot_cmd.vel.angular.z);
      }
    }
    robot_cmd = controller_->limitCommand(robot_cmd);

    latest_joint_velocity_ = controller_->computeJointVelocity(robot_cmd, state_);
    if (isZeroCommand(robot_cmd)) {
      latest_joint_velocity_ = {};
    }
    advanceModelState(robot_cmd, latest_joint_velocity_, dt);
    publishControllerState(robot_cmd, latest_joint_velocity_);
    publishRobotState(robot_cmd, latest_joint_velocity_, dt);
    if (publish_control_cmd_ && pub_control_cmd_) {
      publishActuatorCmd(robot_cmd);
    }
  }

  void publishControllerState(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd, const asr::JointVelocity3D & joint_velocity)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(kStateSize3D, 0.0);
    msg.data[0] = state_.time;
    msg.data[1] = state_.head_position.x;
    msg.data[2] = state_.head_position.y;
    msg.data[3] = state_.head_position.z;

    size_t index = 4;
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        msg.data[index++] = state_.head_frame.v[row][col];
      }
    }
    msg.data[13] = cmd.vel.linear.x;
    msg.data[14] = cmd.vel.angular.y;
    msg.data[15] = cmd.vel.angular.z;
    for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
      msg.data[16 + i] = state_.joints.theta[i];
      msg.data[22 + i] = joint_velocity.theta_dot[i];
    }
    for (size_t point = 0; point < asr::kNum3dPoints; ++point) {
      msg.data[28 + 3 * point] = state_.body_points[point].x;
      msg.data[28 + 3 * point + 1] = state_.body_points[point].y;
      msg.data[28 + 3 * point + 2] = state_.body_points[point].z;
    }
    pub_controller_state_->publish(msg);
  }

  void publishRobotState(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd, const asr::JointVelocity3D & joint_velocity,
    double dt = 0.0)
  {
    const auto stamp = node_->now();
    const auto odom_orientation = quaternionFromFrame(state_.head_frame);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name.reserve(kArticulationJointNames.size() + kRotorJointNames.size());
    joint_state.position.reserve(kArticulationJointNames.size() + kRotorJointNames.size());
    joint_state.velocity.reserve(kArticulationJointNames.size() + kRotorJointNames.size());
    for (size_t urdf_index = 0; urdf_index < kArticulationJointNames.size(); ++urdf_index) {
      const size_t source_index = static_cast<size_t>(joint_source_indices_[urdf_index]);
      double position = joint_signs_[urdf_index] * state_.joints.theta[source_index] +
        joint_offsets_rad_[urdf_index];
      if (clip_joint_positions_) {
        position = std::clamp(
          position, -std::abs(joint_position_limit_rad_), std::abs(joint_position_limit_rad_));
      }
      joint_state.name.push_back(kArticulationJointNames[urdf_index]);
      joint_state.position.push_back(position);
      joint_state.velocity.push_back(
        joint_signs_[urdf_index] * joint_velocity.theta_dot[source_index]);
    }
    const double omega = cmd.vel.linear.x * screw_velocity_scale_;
    for (size_t i = 0; i < kRotorJointNames.size(); ++i) {
      const double rotor_omega = ((i % 2) == 0 ? 1.0 : -1.0) * omega;
      rotor_positions_[i] += rotor_omega * dt;
      joint_state.name.push_back(kRotorJointNames[i]);
      joint_state.position.push_back(rotor_positions_[i]);
      joint_state.velocity.push_back(rotor_omega);
    }
    pub_joint_state_->publish(joint_state);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = controller_base_frame_;
    odometry.pose.pose.position.x = state_.head_position.x;
    odometry.pose.pose.position.y = state_.head_position.y;
    odometry.pose.pose.position.z = state_.head_position.z;
    odometry.pose.pose.orientation = odom_orientation;
    const auto forward = asr::column(state_.head_frame, 0);
    const auto pitch_axis = asr::column(state_.head_frame, 1);
    const auto yaw_axis = asr::column(state_.head_frame, 2);
    odometry.twist.twist.linear.x = cmd.vel.linear.x * forward.x;
    odometry.twist.twist.linear.y = cmd.vel.linear.x * forward.y;
    odometry.twist.twist.linear.z = cmd.vel.linear.x * forward.z;
    odometry.twist.twist.angular.x =
      cmd.vel.angular.y * pitch_axis.x + cmd.vel.angular.z * yaw_axis.x;
    odometry.twist.twist.angular.y =
      cmd.vel.angular.y * pitch_axis.y + cmd.vel.angular.z * yaw_axis.y;
    odometry.twist.twist.angular.z =
      cmd.vel.angular.y * pitch_axis.z + cmd.vel.angular.z * yaw_axis.z;
    pub_odom_->publish(odometry);
  }

  void publishActuatorCmd(const asr_sdm_control_msgs::msg::RobotCommand & cmd)
  {
    asr_sdm_control_msgs::msg::ActuatorCmd msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "front_unit_following_controller_3d";
    const int32_t screw_vel = scaleToInt32(
      cmd.vel.linear.x * screw_velocity_scale_, 1.0, -joint_angle_limit_, joint_angle_limit_);
    msg.units_cmd.resize(asr::kNum3dJoints);
    for (size_t joint = 0; joint < asr::kNum3dJoints; ++joint) {
      auto & unit = msg.units_cmd[joint];
      unit.unit_id = static_cast<int32_t>(joint);
      unit.screw1_vel = screw_vel;
      unit.screw2_vel = -screw_vel;
      unit.joint1_angle = scaledJointAngle(asr::yawIndex(joint));
      unit.joint2_angle = scaledJointAngle(asr::pitchIndex(joint));
    }
    pub_control_cmd_->publish(msg);
  }

  int32_t scaledJointAngle(size_t index) const
  {
    const double value = index < state_.joints.theta.size() ? state_.joints.theta[index] : 0.0;
    return scaleToInt32(value, joint_angle_scale_, -joint_angle_limit_, joint_angle_limit_);
  }

  rclcpp::Node::SharedPtr node_;
  asr::AsrSdmKinematicModelParameters model_params_;
  asr::FrontUnitController3DParameters controller_params_;
  asr::HeadCommandMPCParameters mpc_params_;
  std::unique_ptr<asr::AsrSdmKinematicModel> model_;
  std::unique_ptr<asr::FrontUnitFollowingController3D> controller_;
  std::unique_ptr<asr::HeadCommandMPC> mpc_;
  asr::SimulationState3D state_;
  asr_sdm_control_msgs::msg::RobotCommand cmd_raw_{};
  asr_sdm_control_msgs::msg::RobotCommand cmd_cur_{};
  asr::JointVelocity3D latest_joint_velocity_{};
  std::array<double, kRotorJointNames.size()> rotor_positions_{};

  std::string robot_cmd_topic_;
  std::string controller_state_topic_;
  std::string control_cmd_topic_;
  std::string initialpose_topic_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string world_frame_;
  std::string controller_base_frame_;
  std::vector<int64_t> joint_source_indices_;
  std::vector<double> joint_signs_;
  std::vector<double> joint_offsets_rad_;
  bool clip_joint_positions_{true};
  double joint_position_limit_rad_{kUrdfJointLimit};
  double initial_x_{-5.0};
  double initial_y_{0.0};
  double initial_z_{0.0};
  double initial_yaw_{0.0};
  int control_period_ms_{20};
  double cmd_timeout_sec_{0.3};
  bool enable_grampc_compensation_{true};
  double screw_velocity_scale_{21.277};
  bool publish_control_cmd_{false};
  double joint_angle_scale_{1.0};
  int joint_angle_limit_{2147483647};

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ignore_robot_cmd_until_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_robot_cmd_log_{};
  std::chrono::steady_clock::time_point last_compensation_log_{};
  rclcpp::Subscription<asr_sdm_control_msgs::msg::RobotCommand>::SharedPtr sub_robot_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_controller_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ActuatorCmd>::SharedPtr pub_control_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("control_manager_node");
  asr_sdm::log::initialize("asr_sdm_control_manager");

  AsrSdmControlManager control_manager;
  control_manager.init(nh);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  executor.spin();

  asr_sdm::log::shutdown();
  rclcpp::shutdown();
  return 0;
}

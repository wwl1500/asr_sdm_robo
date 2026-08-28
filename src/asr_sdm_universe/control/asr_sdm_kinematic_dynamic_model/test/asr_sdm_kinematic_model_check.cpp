#include "asr_sdm_kinematic_dynamic_model/asr_sdm_kinematic_model.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

namespace
{

using BodyModel = asr::AsrSdmKinematicModel;

static_assert(BodyModel::kNumJoints == asr::kNum3dJoints, "joint count mismatch");
static_assert(BodyModel::kNumLinks == asr::kNum3dLinks, "link count mismatch");
static_assert(BodyModel::kNumPoints == asr::kNum3dPoints, "body point count mismatch");

constexpr double kTolerance = 1.0e-6;
constexpr double kVelocityTolerance = 1.0e-6;

asr_sdm_control_msgs::msg::RobotCommand makeRobotCommand(
  double linear_velocity, double pitch_rate, double yaw_rate)
{
  asr_sdm_control_msgs::msg::RobotCommand cmd;
  cmd.vel.linear.x = linear_velocity;
  cmd.vel.angular.y = pitch_rate;
  cmd.vel.angular.z = yaw_rate;
  return cmd;
}

Eigen::Vector3d toEigen(const asr::Vec3 & v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

Eigen::Matrix3d toEigen(const asr::Mat3 & m)
{
  Eigen::Matrix3d out;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out(row, col) = m.v[static_cast<size_t>(row)][static_cast<size_t>(col)];
    }
  }
  return out;
}

BodyModel::JointVector toEigen(const std::array<double, asr::kNum3dJointDofs> & theta)
{
  BodyModel::JointVector out;
  for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
    out[static_cast<int>(i)] = theta[i];
  }
  return out;
}

asr::AsrSdmKinematicModelParameters makeModelParameters()
{
  asr::AsrSdmKinematicModelParameters params;
  params.link_length = 0.25;
  params.link_mass = 1.5;
  params.link_radius = 0.05;
  params.joint_limit = 1.5707963267948966;
  params.joint_velocity_limit = 2.0;
  return params;
}

}

Eigen::VectorXd configurationFor(const BodyModel & body_model, const asr::SimulationState3D & state)
{
  return body_model.toConfiguration(
    toEigen(state.head_position), toEigen(state.head_frame), toEigen(state.joints.theta));
}

double maxPointError(
  const std::array<asr::Vec3, asr::kNum3dPoints> & expected,
  const std::array<Eigen::Vector3d, BodyModel::kNumPoints> & actual)
{
  double error = 0.0;
  for (size_t point = 0; point < asr::kNum3dPoints; ++point) {
    error = std::max(error, (toEigen(expected[point]) - actual[point]).norm());
  }
  return error;
}

double maxFrameError(
  const std::array<asr::Mat3, asr::kNum3dLinks> & expected,
  const std::array<Eigen::Matrix3d, BodyModel::kNumLinks> & actual)
{
  double error = 0.0;
  for (size_t link = 0; link < asr::kNum3dLinks; ++link) {
    error = std::max(error, (toEigen(expected[link]) - actual[link]).cwiseAbs().maxCoeff());
  }
  return error;
}

asr::Mat3 randomFrame(std::mt19937 & rng)
{
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Quaterniond orientation(normal(rng), normal(rng), normal(rng), normal(rng));
  if (orientation.norm() < 1.0e-6) {
    orientation = Eigen::Quaterniond::Identity();
  }
  orientation.normalize();

  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  asr::Mat3 out{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out.v[static_cast<size_t>(row)][static_cast<size_t>(col)] = rotation(row, col);
    }
  }
  return out;
}

void printModelSummary(const BodyModel & body_model)
{
  const auto & model = body_model.model();
  std::printf(
    "model \"%s\": njoints=%d nq=%d nv=%d nframes=%d\n", model.name.c_str(), model.njoints,
    model.nq, model.nv, static_cast<int>(model.nframes));
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(model.njoints); ++id) {
    std::printf(
      "  joint %2zu %-16s parent=%-2zu nq=%d nv=%d idx_q=%d idx_v=%d\n", static_cast<size_t>(id),
      model.names[id].c_str(), static_cast<size_t>(model.parents[id]), model.joints[id].nq(),
      model.joints[id].nv(), model.joints[id].idx_q(), model.joints[id].idx_v());
  }
  double total_mass = 0.0;
  for (const auto & inertia : model.inertias) {
    total_mass += inertia.mass();
  }
  std::printf("  total mass = %.4f kg\n", total_mass);
}

bool checkRandomConfigurations(BodyModel & body_model)
{
  std::mt19937 rng(20260810);
  std::uniform_real_distribution<double> position(-3.0, 3.0);
  std::uniform_real_distribution<double> angle(-1.5, 1.5);

  double point_error = 0.0;
  double frame_error = 0.0;

  for (int trial = 0; trial < 5000; ++trial) {
    const asr::Vec3 head_position{position(rng), position(rng), position(rng)};
    const asr::Mat3 head_frame = randomFrame(rng);
    std::array<double, asr::kNum3dJointDofs> theta{};
    for (auto & value : theta) {
      value = angle(rng);
    }

    const auto expected_frames = asr::linkFrames(head_frame, theta);
    const auto expected_axes = asr::linkAxes(expected_frames);
    const auto expected_points =
      asr::bodyPoints(head_position, expected_axes, body_model.parameters().link_length);

    body_model.updateKinematics(
      body_model.toConfiguration(toEigen(head_position), toEigen(head_frame), toEigen(theta)));

    point_error =
      std::max(point_error, maxPointError(expected_points, body_model.computeBodyPoints()));
    frame_error =
      std::max(frame_error, maxFrameError(expected_frames, body_model.computeLinkFrames()));
  }

  std::printf(
    "random configurations : max body-point error = %.3e m, max link-frame error = %.3e\n",
    point_error, frame_error);
  return point_error < kTolerance && frame_error < kTolerance;
}

bool checkControllerRollout(BodyModel & body_model)
{
  const asr::FrontUnitFollowingController3D controller(
    makeControllerParameters(body_model.parameters().link_length));
  asr::SimulationState3D state = controller.makeInitialState();

  const double dt = 0.02;
  double point_error = 0.0;
  double frame_error = 0.0;

  for (int step = 0; step < 3000; ++step) {
    const double t = step * dt;
    const auto cmd = makeRobotCommand(0.12, 0.30 * std::sin(0.7 * t), 0.30 * std::cos(0.4 * t));
    controller.step(cmd, dt, state);

    body_model.updateKinematics(configurationFor(body_model, state));
    point_error =
      std::max(point_error, maxPointError(state.body_points, body_model.computeBodyPoints()));
    frame_error =
      std::max(frame_error, maxFrameError(state.link_frames, body_model.computeLinkFrames()));
  }

  std::printf(
    "controller rollout    : max body-point error = %.3e m, max link-frame error = %.3e\n",
    point_error, frame_error);
  return point_error < kTolerance && frame_error < kTolerance;
}

bool checkVelocityMapping(BodyModel & body_model)
{
  const asr::FrontUnitFollowingController3D controller(
    makeControllerParameters(body_model.parameters().link_length));
  asr::SimulationState3D state = controller.makeInitialState();

  const double dt = 0.01;
  const double finite_difference_step = 1.0e-6;
  double jacobian_error = 0.0;
  double propagation_error = 0.0;

  for (int step = 0; step < 200; ++step) {
    const double t = step * dt;
    const auto cmd = makeRobotCommand(0.12, 0.25 * std::sin(0.9 * t), 0.25 * std::cos(0.5 * t));
    const asr::JointVelocity3D joint_velocity = controller.step(cmd, dt, state);
    const auto limited_cmd = controller.limitCommand(cmd);

    const Eigen::VectorXd q = configurationFor(body_model, state);
    const Eigen::VectorXd v = body_model.toVelocity(
      limited_cmd.vel.linear.x, limited_cmd.vel.angular.y, limited_cmd.vel.angular.z,
      toEigen(joint_velocity.theta_dot));

    body_model.updateKinematics(q);
    const auto points_at_q = body_model.computeBodyPoints();
    const Eigen::VectorXd q_next =
      pinocchio::integrate(body_model.model(), q, Eigen::VectorXd(finite_difference_step * v));
    body_model.updateKinematics(q_next);
    const auto points_at_q_next = body_model.computeBodyPoints();

    body_model.updateKinematics(q, v);
    for (size_t point = 0; point < BodyModel::kNumPoints; ++point) {
      const Eigen::Vector3d numerical =
        (points_at_q_next[point] - points_at_q[point]) / finite_difference_step;

      propagation_error = std::max(
        propagation_error, (body_model.bodyPointVelocity(point) - numerical).norm());

      const Eigen::Vector3d from_jacobian =
        (body_model.bodyPointJacobian(q, point) * v).head<3>();
      jacobian_error = std::max(jacobian_error, (from_jacobian - numerical).norm());
    }
  }

  std::printf(
    "velocity mapping      : max forward-kinematics error = %.3e m/s, max Jacobian error = %.3e m/s\n",
    propagation_error, jacobian_error);
  return propagation_error < kVelocityTolerance && jacobian_error < kVelocityTolerance;
}

bool checkJointLimitIntegration()
{
  auto params = makeModelParameters();
  params.joint_limit = 0.1;
  BodyModel body_model(params);

  Eigen::VectorXd q = body_model.neutralConfiguration();
  BodyModel::JointVector theta_dot = BodyModel::JointVector::Constant(1.0);
  const Eigen::VectorXd v = body_model.toVelocity(0.0, 0.0, 0.0, theta_dot);
  for (int step = 0; step < 100; ++step) {
    q = body_model.integrateConfiguration(q, v, 0.02);
  }

  Eigen::Vector3d head_position;
  Eigen::Matrix3d head_frame;
  BodyModel::JointVector theta;
  body_model.fromConfiguration(q, head_position, head_frame, theta);

  const double max_angle = theta.cwiseAbs().maxCoeff();
  std::printf(
    "joint limit integration: max articulation angle = %.6f rad, limit = %.6f rad\n",
    max_angle, params.joint_limit);
  return max_angle <= params.joint_limit + kTolerance;
}

void printDynamicsSample(BodyModel & body_model)
{
  const Eigen::VectorXd q = body_model.neutralConfiguration();
  const Eigen::MatrixXd mass_matrix = body_model.massMatrix(q);
  const Eigen::VectorXd gravity = body_model.gravityTorque(q);
  std::printf(
    "dynamics at neutral   : M is %ldx%ld, trace = %.4f, |g| = %.4f\n", mass_matrix.rows(),
    mass_matrix.cols(), mass_matrix.trace(), gravity.norm());
}

}  // namespace

int main()
{
  BodyModel body_model(makeModelParameters());

  printModelSummary(body_model);

  bool ok = true;
  ok = checkRandomConfigurations(body_model) && ok;
  ok = checkControllerRollout(body_model) && ok;
  ok = checkVelocityMapping(body_model) && ok;
  ok = checkJointLimitIntegration() && ok;
  printDynamicsSample(body_model);

  std::printf("%s\n", ok ? "ALL CHECKS PASSED" : "CHECKS FAILED");
  return ok ? 0 : 1;
}

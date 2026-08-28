#include "asr_sdm_front_following_control/front_following_controller.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace asr_sdm_controller
{
namespace
{

constexpr double kQuaternionEpsilon = 1.0e-12;
constexpr double kSmallDt = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

std::array<std::string, 4> propulsionLinkNames()
{
  return {"base_link", "link_2", "link_3", "link_4"};
}

template<typename Derived>
bool isFinite(const Eigen::MatrixBase<Derived> & value)
{
  return value.array().isFinite().all();
}

Eigen::VectorXd pseudoInverseSolve(
  const Eigen::MatrixXd & matrix,
  const Eigen::VectorXd & rhs,
  const double tolerance,
  double * condition_number = nullptr)
{
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();

  if (singular_values.size() == 0) {
    if (condition_number != nullptr) {
      *condition_number = std::numeric_limits<double>::infinity();
    }
    return Eigen::VectorXd::Zero(matrix.cols());
  }

  const double largest = singular_values.maxCoeff();
  const double smallest = singular_values.minCoeff();
  if (condition_number != nullptr) {
    *condition_number =
      smallest > tolerance ? largest / smallest : std::numeric_limits<double>::infinity();
  }

  Eigen::VectorXd inverse_singular_values = Eigen::VectorXd::Zero(singular_values.size());
  const double threshold = tolerance * std::max(1.0, largest);
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    if (singular_values(index) > threshold) {
      inverse_singular_values(index) = 1.0 / singular_values(index);
    }
  }

  return svd.matrixV() * inverse_singular_values.asDiagonal() * svd.matrixU().transpose() * rhs;
}

Eigen::MatrixXd pseudoInverse(
  const Eigen::MatrixXd & matrix,
  const double tolerance)
{
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();

  if (singular_values.size() == 0) {
    return Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
  }

  const double largest = singular_values.maxCoeff();
  const double threshold = tolerance * std::max(1.0, largest);

  Eigen::VectorXd inverse_singular_values = Eigen::VectorXd::Zero(singular_values.size());
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    if (singular_values(index) > threshold) {
      inverse_singular_values(index) = 1.0 / singular_values(index);
    }
  }

  return svd.matrixV() * inverse_singular_values.asDiagonal() * svd.matrixU().transpose();
}

}  // namespace

FrontFollowingController::FrontFollowingController(
  std::shared_ptr<pinocchio::Model> model,
  std::shared_ptr<pinocchio::Data> data,
  const asr_sdm_kinematic_dynamic_model::HydrodynamicParameters & hydrodynamic_parameters,
  const ControllerParameters & parameters)
: model_(std::move(model)),
  data_(std::move(data)),
  parameters_(parameters),
  dynamics_(std::make_unique<asr_sdm_kinematic_dynamic_model::UnderwaterDynamics>(
      model_, data_, hydrodynamic_parameters))
{
  if (!model_ || !data_) {
    throw std::invalid_argument("FrontFollowingController requires a valid Pinocchio model/data.");
  }

  if (model_->nq != 13 || model_->nv != 12) {
    throw std::invalid_argument(
            "FrontFollowingController expects the nominal ASR-SDM free-flyer model (nq=13, nv=12).");
  }

  for (const auto & link_name : propulsionLinkNames()) {
    if (!model_->existFrame(link_name, pinocchio::BODY)) {
      throw std::invalid_argument("Propulsion frame not found in model: " + link_name);
    }
    propulsion_frame_ids_.push_back(model_->getFrameId(link_name, pinocchio::BODY));
  }
}

void FrontFollowingController::reset()
{
  reference_state_ = ReferenceGeneratorState{};
}

Eigen::VectorXd FrontFollowingController::buildConfiguration(
  const RobotKinematicState & state) const
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
  const Eigen::Quaterniond normalized = normalizeQuaternion(state.orientation);

  q.segment<3>(0) = state.position;
  q(3) = normalized.x();
  q(4) = normalized.y();
  q(5) = normalized.z();
  q(6) = normalized.w();
  q.segment<6>(7) = state.joint_positions;

  return q;
}

Eigen::VectorXd FrontFollowingController::buildVelocity(const RobotKinematicState & state) const
{
  Eigen::VectorXd zeta = Eigen::VectorXd::Zero(model_->nv);
  zeta.segment<3>(0) = state.linear_velocity_body;
  zeta.segment<3>(3) = state.angular_velocity_body;
  zeta.segment<6>(6) = state.joint_velocities;
  return zeta;
}

OutputKinematics FrontFollowingController::computeOutputKinematics(
  const RobotKinematicState & state) const
{
  return computeOutputKinematics(buildConfiguration(state), buildVelocity(state));
}

OutputKinematics FrontFollowingController::computeOutputKinematics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & zeta) const
{
  OutputKinematics output;

  const Eigen::Vector3d position = q.segment<3>(0);
  const Eigen::Quaterniond orientation(q(6), q(3), q(4), q(5));
  const Eigen::Matrix3d rotation = normalizeQuaternion(orientation).toRotationMatrix();
  const Vector6d joint_positions = q.segment<6>(7);

  const Eigen::Vector3d body_ex = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d body_r_h(-parameters_.front_half_length, 0.0, 0.0);
  const Eigen::Vector3d world_r_h = rotation * body_r_h;
  const Eigen::Vector3d world_t_h = rotation * body_ex;

  const double heading_projection_norm_sq =
    world_t_h.x() * world_t_h.x() + world_t_h.y() * world_t_h.y();
  const double psi_p = std::atan2(world_t_h.y(), world_t_h.x());
  const double pitch_argument = std::clamp(-rotation(2, 0), -1.0, 1.0);
  const double pitch = std::asin(pitch_argument);
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch_cos = std::cos(pitch);
  const double clamped_pitch_cos =
    std::copysign(std::max(std::abs(pitch_cos), parameters_.pitch_cos_min), pitch_cos);

  Eigen::Matrix<double, 3, 12> j_h_p = Eigen::Matrix<double, 3, 12>::Zero();
  j_h_p.block<3, 3>(0, 0) = rotation;
  j_h_p.block<3, 3>(0, 3) = -rotation * skew(body_r_h);

  Eigen::Matrix<double, 3, 12> j_h_t = Eigen::Matrix<double, 3, 12>::Zero();
  j_h_t.block<3, 3>(0, 3) = -rotation * skew(body_ex);

  Eigen::RowVector3d heading_row(0.0, 0.0, 0.0);
  if (heading_projection_norm_sq > parameters_.heading_projection_min) {
    heading_row <<
      -world_t_h.y() / heading_projection_norm_sq,
      world_t_h.x() / heading_projection_norm_sq,
      0.0;
  }

  Eigen::Matrix<double, 3, 12> j_p = Eigen::Matrix<double, 3, 12>::Zero();
  j_p.row(0) = j_h_p.row(0);
  j_p.row(1) = j_h_p.row(1);
  j_p.row(2) = heading_row * j_h_t;

  Eigen::Matrix<double, 3, 12> s_y = Eigen::Matrix<double, 3, 12>::Zero();
  s_y(0, 7) = 1.0;
  s_y(1, 9) = 1.0;
  s_y(2, 11) = 1.0;

  Eigen::Matrix<double, 6, 12> t_eta = Eigen::Matrix<double, 6, 12>::Zero();
  t_eta.block<1, 3>(0, 0) = rotation.row(2);
  const double tan_pitch = std::sin(pitch) / clamped_pitch_cos;
  Eigen::Matrix<double, 2, 3> t_rp = Eigen::Matrix<double, 2, 3>::Zero();
  t_rp <<
    1.0, std::sin(roll) * tan_pitch, std::cos(roll) * tan_pitch,
    0.0, std::cos(roll), -std::sin(roll);
  t_eta.block<2, 3>(1, 3) = t_rp;
  t_eta(3, 6) = 1.0;
  t_eta(4, 8) = 1.0;
  t_eta(5, 10) = 1.0;

  output.a_ff.block<3, 12>(0, 0) = j_p;
  output.a_ff.block<3, 12>(3, 0) = s_y;
  output.a_ff.block<6, 12>(6, 0) = t_eta;

  const Eigen::Vector3d p_h = position + world_r_h;
  output.y <<
    p_h.x(),
    p_h.y(),
    psi_p,
    joint_positions(1),
    joint_positions(3),
    joint_positions(5),
    position.z(),
    roll,
    pitch,
    joint_positions(0),
    joint_positions(2),
    joint_positions(4);
  output.y_dot = output.a_ff * zeta;
  output.heading_projection_norm_sq = heading_projection_norm_sq;
  output.pitch_cos = pitch_cos;

  return output;
}

FrontFollowingController::DesiredTrajectory FrontFollowingController::updateDesiredTrajectory(
  const OutputKinematics & output,
  const MotionCommand & command,
  const double dt_seconds)
{
  DesiredTrajectory desired;
  const double dt = std::max(dt_seconds, kSmallDt);

  if (!reference_state_.initialized) {
    reference_state_.initialized = true;
    reference_state_.xi_desired = output.y.segment<3>(0);
    reference_state_.phi_desired = output.y.segment<3>(3);
    reference_state_.previous_command = command;
  }

  const double phi1 = output.y(3);
  const double phi2 = output.y(4);
  const double phi3 = output.y(5);
  const double psi_p = output.y(2);
  const double psi_p_dot = output.y_dot(2);

  const double dot_v1 = (command.v1 - reference_state_.previous_command.v1) / dt;
  const double dot_omega1 = (command.omega1 - reference_state_.previous_command.omega1) / dt;

  const double bar_v2 =
    command.v1 * std::cos(phi1) - parameters_.link_length * command.omega1 * std::sin(phi1);
  const double phi1_dot_desired =
    -(2.0 / parameters_.link_length) * command.v1 * std::sin(phi1) -
    command.omega1 * (2.0 * std::cos(phi1) + 1.0);
  const double bar_v3 =
    bar_v2 * std::cos(phi2) -
    0.5 * parameters_.link_length * (command.omega1 + phi1_dot_desired) * std::sin(phi2);
  const double phi2_dot_desired =
    -(2.0 / parameters_.link_length) * bar_v2 * std::sin(phi2) -
    (command.omega1 + phi1_dot_desired) * (1.0 + std::cos(phi2));
  const double phi3_dot_desired =
    -(2.0 / parameters_.link_length) * bar_v3 * std::sin(phi3) -
    (command.omega1 + phi1_dot_desired + phi2_dot_desired) * (1.0 + std::cos(phi3));

  const double dot_bar_v2 =
    dot_v1 * std::cos(phi1) -
    command.v1 * std::sin(phi1) * phi1_dot_desired -
    parameters_.link_length * dot_omega1 * std::sin(phi1) -
    parameters_.link_length * command.omega1 * std::cos(phi1) * phi1_dot_desired;
  const double phi1_ddot_desired =
    -(2.0 / parameters_.link_length) *
    (dot_v1 * std::sin(phi1) + command.v1 * std::cos(phi1) * phi1_dot_desired) -
    dot_omega1 * (2.0 * std::cos(phi1) + 1.0) +
    2.0 * command.omega1 * std::sin(phi1) * phi1_dot_desired;
  const double dot_bar_v3 =
    dot_bar_v2 * std::cos(phi2) -
    bar_v2 * std::sin(phi2) * phi2_dot_desired -
    0.5 * parameters_.link_length * (dot_omega1 + phi1_ddot_desired) * std::sin(phi2) -
    0.5 * parameters_.link_length *
    (command.omega1 + phi1_dot_desired) * std::cos(phi2) * phi2_dot_desired;
  const double phi2_ddot_desired =
    -(2.0 / parameters_.link_length) *
    (dot_bar_v2 * std::sin(phi2) + bar_v2 * std::cos(phi2) * phi2_dot_desired) -
    (dot_omega1 + phi1_ddot_desired) * (1.0 + std::cos(phi2)) +
    (command.omega1 + phi1_dot_desired) * std::sin(phi2) * phi2_dot_desired;
  const double phi3_ddot_desired =
    -(2.0 / parameters_.link_length) *
    (dot_bar_v3 * std::sin(phi3) + bar_v3 * std::cos(phi3) * phi3_dot_desired) -
    (dot_omega1 + phi1_ddot_desired + phi2_ddot_desired) * (1.0 + std::cos(phi3)) +
    (command.omega1 + phi1_dot_desired + phi2_dot_desired) *
    std::sin(phi3) * phi3_dot_desired;

  const Eigen::Vector3d xi_dot_desired(
    -command.v1 * std::cos(psi_p),
    -command.v1 * std::sin(psi_p),
    command.omega1);
  const Eigen::Vector3d xi_ddot_desired(
    -dot_v1 * std::cos(psi_p) + command.v1 * psi_p_dot * std::sin(psi_p),
    -dot_v1 * std::sin(psi_p) - command.v1 * psi_p_dot * std::cos(psi_p),
    dot_omega1);
  const Eigen::Vector3d phi_dot_desired(
    phi1_dot_desired,
    phi2_dot_desired,
    phi3_dot_desired);
  const Eigen::Vector3d phi_ddot_desired(
    phi1_ddot_desired,
    phi2_ddot_desired,
    phi3_ddot_desired);

  reference_state_.xi_desired += xi_dot_desired * dt;
  reference_state_.phi_desired += phi_dot_desired * dt;
  reference_state_.xi_desired(2) = wrapAngle(reference_state_.xi_desired(2));
  for (Eigen::Index index = 0; index < reference_state_.phi_desired.size(); ++index) {
    reference_state_.phi_desired(index) = wrapAngle(reference_state_.phi_desired(index));
  }
  reference_state_.previous_command = command;

  const Vector6d eta = output.y.segment<6>(6);
  const Vector6d eta_dot = output.y_dot.segment<6>(6);
  const Vector6d eta_ddot_desired =
    -(parameters_.k_eta_d.asDiagonal() * eta_dot) -
    parameters_.k_eta_p.asDiagonal() * (eta - parameters_.eta_reference);

  desired.y_d.segment<3>(0) = reference_state_.xi_desired;
  desired.y_d.segment<3>(3) = reference_state_.phi_desired;
  desired.y_d.segment<6>(6) = parameters_.eta_reference;

  desired.y_dot_d.segment<3>(0) = xi_dot_desired;
  desired.y_dot_d.segment<3>(3) = phi_dot_desired;
  desired.y_dot_d.segment<6>(6) = Vector6d::Zero();

  desired.y_ddot_d.segment<3>(0) = xi_ddot_desired;
  desired.y_ddot_d.segment<3>(3) = phi_ddot_desired;
  desired.y_ddot_d.segment<6>(6) = eta_ddot_desired;

  return desired;
}

Matrix12x10d FrontFollowingController::buildActuationMatrix()
{
  Matrix12x10d b_act = Matrix12x10d::Zero();
  const Eigen::Matrix<double, 6, 1> thrust_wrench(
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  for (size_t index = 0; index < propulsion_frame_ids_.size(); ++index) {
    Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::getFrameJacobian(
      *model_, *data_, propulsion_frame_ids_[index], pinocchio::LOCAL, local_jacobian);
    b_act.col(static_cast<Eigen::Index>(index)) = local_jacobian.transpose() * thrust_wrench;
  }

  b_act.block<6, 6>(6, 4) = Eigen::Matrix<double, 6, 6>::Identity();
  return b_act;
}

Vector12d FrontFollowingController::computeDotAffTimesZeta(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & zeta,
  const Matrix12d & a_ff) const
{
  const double dt = std::max(parameters_.finite_difference_dt, kSmallDt);
  const Eigen::VectorXd q_next = pinocchio::integrate(*model_, q, dt * zeta);
  const OutputKinematics next_output = computeOutputKinematics(q_next, zeta);
  return ((next_output.a_ff - a_ff) / dt) * zeta;
}

ControlComputationResult FrontFollowingController::compute(
  const RobotKinematicState & state,
  const MotionCommand & command,
  const double dt_seconds)
{
  ControlComputationResult result;

  if (state.orientation.norm() < kQuaternionEpsilon) {
    result.error_message = "RobotState orientation quaternion has near-zero norm.";
    return result;
  }

  const Eigen::VectorXd q = buildConfiguration(state);
  const Eigen::VectorXd zeta = buildVelocity(state);
  if (!isFinite(q) || !isFinite(zeta)) {
    result.error_message = "RobotState contains non-finite values.";
    return result;
  }

  const OutputKinematics output = computeOutputKinematics(q, zeta);
  result.y = output.y;
  result.y_dot = output.y_dot;
  result.a_ff = output.a_ff;

  if (output.heading_projection_norm_sq <= parameters_.heading_projection_min) {
    result.error_message = "Projected front-link heading is singular.";
    return result;
  }

  if (std::abs(output.pitch_cos) <= parameters_.pitch_cos_min) {
    result.error_message = "Base pitch is too close to the ZYX singularity.";
    return result;
  }

  const DesiredTrajectory desired = updateDesiredTrajectory(output, command, dt_seconds);
  result.y_d = desired.y_d;
  result.y_dot_d = desired.y_dot_d;
  result.y_ddot_d = desired.y_ddot_d;

  Vector12d e_y = output.y - desired.y_d;
  wrapTrackingError(e_y);
  const Vector12d e_dot_y = output.y_dot - desired.y_dot_d;
  const Vector12d dot_aff_times_zeta = computeDotAffTimesZeta(q, zeta, output.a_ff);

  const Eigen::VectorXd zeta_dot_command = pseudoInverseSolve(
    output.a_ff,
    desired.y_ddot_d -
    dot_aff_times_zeta -
    parameters_.k_d.asDiagonal() * e_dot_y -
    parameters_.k_p.asDiagonal() * e_y,
    parameters_.pinv_tolerance,
    &result.aff_condition_number);

  if (!std::isfinite(result.aff_condition_number) ||
    result.aff_condition_number > parameters_.aff_condition_number_limit)
  {
    result.error_message = "A_ff is ill-conditioned in the current configuration.";
    return result;
  }

  if (!isFinite(zeta_dot_command)) {
    result.error_message = "Dynamic inversion produced non-finite generalized accelerations.";
    return result;
  }

  dynamics_->computeDynamics(q, zeta);
  result.b_act = buildActuationMatrix();

  const Vector12d u_command =
    dynamics_->getMassMatrix() * zeta_dot_command +
    dynamics_->getCoriolisMatrix() * zeta +
    dynamics_->getDampingMatrix() * zeta +
    dynamics_->getRestoringForces();
  result.u_cmd = u_command;

  if (!isFinite(u_command) || !isFinite(result.b_act)) {
    result.error_message = "Inverse dynamics or actuation mapping produced non-finite values.";
    return result;
  }

  Vector10d actuator_command = pseudoInverse(result.b_act, parameters_.pinv_tolerance) * u_command;
  if (!isFinite(actuator_command)) {
    result.error_message = "Actuator allocation produced non-finite commands.";
    return result;
  }

  for (Eigen::Index index = 0; index < 4; ++index) {
    actuator_command(index) = std::clamp(
      actuator_command(index),
      -std::abs(parameters_.max_propulsion_force(index)),
      std::abs(parameters_.max_propulsion_force(index)));
  }
  for (Eigen::Index index = 0; index < 6; ++index) {
    actuator_command(index + 4) = std::clamp(
      actuator_command(index + 4),
      -std::abs(parameters_.max_joint_torque(index)),
      std::abs(parameters_.max_joint_torque(index)));
  }

  result.propulsion_forces = actuator_command.head<4>();
  result.joint_torques = actuator_command.tail<6>();
  result.allocation_residual = result.b_act * actuator_command - u_command;

  if (!isFinite(result.allocation_residual)) {
    result.error_message = "Actuator allocation residual is non-finite.";
    return result;
  }

  result.ok = true;
  return result;
}

Eigen::Matrix3d FrontFollowingController::skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return matrix;
}

double FrontFollowingController::wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void FrontFollowingController::wrapTrackingError(Vector12d & error)
{
  constexpr std::array<int, 9> kAngularIndices = {2, 3, 4, 5, 7, 8, 9, 10, 11};
  for (const int index : kAngularIndices) {
    error(index) = wrapAngle(error(index));
  }
}

Eigen::Quaterniond FrontFollowingController::normalizeQuaternion(
  const Eigen::Quaterniond & quaternion)
{
  if (quaternion.norm() < kQuaternionEpsilon) {
    return Eigen::Quaterniond::Identity();
  }
  return quaternion.normalized();
}

}  // namespace asr_sdm_controller

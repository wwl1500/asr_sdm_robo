#pragma once

#include "asr_sdm_kinematic_dynamic_model/underwater_dynamics.hpp"

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <memory>
#include <string>
#include <vector>

namespace asr_sdm_controller
{

using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector10d = Eigen::Matrix<double, 10, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix12x10d = Eigen::Matrix<double, 12, 10>;

struct MotionCommand
{
  double v1 = 0.0;
  double omega1 = 0.0;
};

struct RobotKinematicState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_body = Eigen::Vector3d::Zero();
  Vector6d joint_positions = Vector6d::Zero();
  Vector6d joint_velocities = Vector6d::Zero();
};

struct ControllerParameters
{
  double link_length = 0.30;
  double front_half_length = 0.15;
  double finite_difference_dt = 1.0e-4;
  double pinv_tolerance = 1.0e-6;
  double heading_projection_min = 1.0e-6;
  double pitch_cos_min = 1.0e-3;
  double aff_condition_number_limit = 1.0e6;

  Vector6d eta_reference = Vector6d::Zero();
  Vector6d k_eta_p = Vector6d::Constant(4.0);
  Vector6d k_eta_d = Vector6d::Constant(2.0);
  Vector12d k_p = Vector12d::Constant(8.0);
  Vector12d k_d = Vector12d::Constant(4.0);

  Vector4d max_propulsion_force = Vector4d::Constant(50.0);
  Vector6d max_joint_torque = Vector6d::Constant(100.0);
};

struct OutputKinematics
{
  Vector12d y = Vector12d::Zero();
  Vector12d y_dot = Vector12d::Zero();
  Matrix12d a_ff = Matrix12d::Zero();
  double heading_projection_norm_sq = 0.0;
  double pitch_cos = 1.0;
};

struct ControlComputationResult
{
  bool ok = false;
  std::string error_message;

  Vector4d propulsion_forces = Vector4d::Zero();
  Vector6d joint_torques = Vector6d::Zero();
  Vector12d y = Vector12d::Zero();
  Vector12d y_d = Vector12d::Zero();
  Vector12d y_dot = Vector12d::Zero();
  Vector12d y_dot_d = Vector12d::Zero();
  Vector12d y_ddot_d = Vector12d::Zero();
  Vector12d u_cmd = Vector12d::Zero();
  Vector12d allocation_residual = Vector12d::Zero();
  Matrix12d a_ff = Matrix12d::Zero();
  Matrix12x10d b_act = Matrix12x10d::Zero();
  double aff_condition_number = 0.0;
};

class FrontFollowingController
{
public:
  FrontFollowingController(
    std::shared_ptr<pinocchio::Model> model,
    std::shared_ptr<pinocchio::Data> data,
    const asr_sdm_kinematic_dynamic_model::HydrodynamicParameters & hydrodynamic_parameters,
    const ControllerParameters & parameters);

  void reset();

  OutputKinematics computeOutputKinematics(const RobotKinematicState & state) const;

  ControlComputationResult compute(
    const RobotKinematicState & state,
    const MotionCommand & command,
    double dt_seconds);

private:
  struct DesiredTrajectory
  {
    Vector12d y_d = Vector12d::Zero();
    Vector12d y_dot_d = Vector12d::Zero();
    Vector12d y_ddot_d = Vector12d::Zero();
  };

  struct ReferenceGeneratorState
  {
    bool initialized = false;
    Eigen::Vector3d xi_desired = Eigen::Vector3d::Zero();
    Eigen::Vector3d phi_desired = Eigen::Vector3d::Zero();
    MotionCommand previous_command;
  };

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  ControllerParameters parameters_;
  std::unique_ptr<asr_sdm_kinematic_dynamic_model::UnderwaterDynamics> dynamics_;
  std::vector<pinocchio::FrameIndex> propulsion_frame_ids_;
  ReferenceGeneratorState reference_state_;

  Eigen::VectorXd buildConfiguration(const RobotKinematicState & state) const;
  Eigen::VectorXd buildVelocity(const RobotKinematicState & state) const;

  OutputKinematics computeOutputKinematics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & zeta) const;

  DesiredTrajectory updateDesiredTrajectory(
    const OutputKinematics & output,
    const MotionCommand & command,
    double dt_seconds);

  Matrix12x10d buildActuationMatrix();
  Vector12d computeDotAffTimesZeta(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & zeta,
    const Matrix12d & a_ff) const;

  static Eigen::Matrix3d skew(const Eigen::Vector3d & vector);
  static double wrapAngle(double angle);
  static void wrapTrackingError(Vector12d & error);
  static Eigen::Quaterniond normalizeQuaternion(const Eigen::Quaterniond & quaternion);
};

}  // namespace asr_sdm_controller

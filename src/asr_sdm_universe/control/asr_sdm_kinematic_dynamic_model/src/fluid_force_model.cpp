#include "asr_sdm_kinematic_dynamic_model/fluid_force_model.hpp"

#include "asr_sdm_kinematic_dynamic_model/jacobian_3d.hpp"
#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

Eigen::Vector3d toEigen(const Vec3 & value)
{
  return Eigen::Vector3d(value.x, value.y, value.z);
}

Eigen::Matrix3d toEigen(const Mat3 & value)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result(row, col) = value.v[row][col];
    }
  }
  return result;
}

Eigen::Matrix3d skew(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return result;
}

SpatialVector spatialCrossStar(
  const SpatialVector & twist, const SpatialVector & wrench)
{
  SpatialVector result;
  const Eigen::Vector3d linear = twist.head<3>();
  const Eigen::Vector3d angular = twist.tail<3>();
  const Eigen::Vector3d force = wrench.head<3>();
  const Eigen::Vector3d moment = wrench.tail<3>();
  result.head<3>() = angular.cross(force);
  result.tail<3>() = linear.cross(force) + angular.cross(moment);
  return result;
}

bool finite(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all();
}

bool finite(const Eigen::Matrix3d & value)
{
  return value.array().isFinite().all();
}

bool finite(const SpatialVector & value)
{
  return value.array().isFinite().all();
}

bool finite(const SpatialMatrix & value)
{
  return value.array().isFinite().all();
}

}  // namespace

FluidForceModel::FluidForceModel(const HydrodynamicModelParameters & params)
: params_(params)
{
  const double tolerance = std::max(0.0, params_.validation_tolerance);
  if (!std::isfinite(params_.fluid_density) || params_.fluid_density < 0.0) {
    error_ = "Fluid density must be finite and nonnegative";
    return;
  }
  if (!finite(params_.gravity_world)) {
    error_ = "Gravity must contain only finite values";
    return;
  }
  if (!std::isfinite(params_.link_length) || params_.link_length <= 0.0) {
    error_ = "Link length must be finite and positive";
    return;
  }
  if (!std::isfinite(params_.validation_tolerance) || params_.validation_tolerance < 0.0) {
    error_ = "Validation tolerance must be finite and nonnegative";
    return;
  }

  for (std::size_t link = 0; link < kNumLinks; ++link) {
    const auto & link_params = params_.links[link];
    if (!std::isfinite(link_params.mass) || link_params.mass < 0.0) {
      std::ostringstream stream;
      stream << "Link " << link << " mass must be finite and nonnegative";
      error_ = stream.str();
      return;
    }
    if (!finite(link_params.center_of_mass) ||
      !finite(link_params.inertia_about_center_of_mass) ||
      !finite(link_params.added_mass) || !finite(link_params.linear_damping) ||
      !finite(link_params.quadratic_damping) || !finite(link_params.center_of_buoyancy) ||
      !std::isfinite(link_params.displaced_volume) || link_params.displaced_volume < 0.0)
    {
      std::ostringstream stream;
      stream << "Link " << link << " contains a non-finite or invalid parameter";
      error_ = stream.str();
      return;
    }
    if ((link_params.added_mass - link_params.added_mass.transpose()).cwiseAbs().maxCoeff() >
      tolerance)
    {
      std::ostringstream stream;
      stream << "Link " << link << " added-mass matrix is not symmetric";
      error_ = stream.str();
      return;
    }
    const Eigen::SelfAdjointEigenSolver<SpatialMatrix> solver(
      (link_params.added_mass + link_params.added_mass.transpose()) * 0.5);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -tolerance) {
      std::ostringstream stream;
      stream << "Link " << link << " added-mass matrix is not positive semidefinite";
      error_ = stream.str();
      return;
    }
    if (link_params.inertia_about_center_of_mass.array().isFinite().all() &&
      (link_params.inertia_about_center_of_mass -
      link_params.inertia_about_center_of_mass.transpose()).cwiseAbs().maxCoeff() > tolerance)
    {
      std::ostringstream stream;
      stream << "Link " << link << " inertia matrix is not symmetric";
      error_ = stream.str();
      return;
    }
    if ((link_params.linear_damping.array() < -tolerance).any() ||
      (link_params.quadratic_damping.array() < -tolerance).any())
    {
      std::ostringstream stream;
      stream << "Link " << link << " damping coefficients must be nonnegative";
      error_ = stream.str();
      return;
    }
  }
}

bool FluidForceModel::isValid() const
{
  return error_.empty();
}

const std::string & FluidForceModel::error() const
{
  return error_;
}

const HydrodynamicModelParameters & FluidForceModel::parameters() const
{
  return params_;
}

SpatialMatrix FluidForceModel::effectiveSpatialInertia(std::size_t link) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot compute effective inertia for an invalid fluid model: " +
        error_);
  }
  if (link >= kNumLinks) {
    throw std::out_of_range("Fluid link index is out of range");
  }

  const auto & link_params = params_.links[link];
  const Eigen::Matrix3d com_skew = skew(link_params.center_of_mass);
  SpatialMatrix rigid = SpatialMatrix::Zero();
  rigid.topLeftCorner<3, 3>() = link_params.mass * Eigen::Matrix3d::Identity();
  rigid.topRightCorner<3, 3>() = -link_params.mass * com_skew;
  rigid.bottomLeftCorner<3, 3>() = link_params.mass * com_skew;
  rigid.bottomRightCorner<3, 3>() = link_params.inertia_about_center_of_mass +
    link_params.mass * com_skew.transpose() * com_skew;
  return rigid + link_params.added_mass;
}

HydrodynamicEvaluation FluidForceModel::evaluate(
  const KinematicState3D & state,
  const BaseTwist3D & base_twist,
  const JointVelocity3D & joint_velocity,
  const Eigen::Vector3d & fluid_current_world,
  const BaseAcceleration3D & base_acceleration,
  const JointAcceleration3D & joint_acceleration,
  const std::array<SpatialVector, kNumLinks> * explicit_local_accelerations) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid fluid model: " + error_);
  }
  if (!finite(fluid_current_world) || !finite(base_twist.linear_velocity_world) ||
    !finite(base_twist.angular_velocity_world) ||
    !finite(base_acceleration.linear_acceleration_world) ||
    !finite(base_acceleration.angular_acceleration_world))
  {
    throw std::invalid_argument("Fluid model inputs must contain only finite values");
  }
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    if (!std::isfinite(joint_velocity.theta_dot[dof]) ||
      !std::isfinite(joint_acceleration.theta_ddot[dof]))
    {
      throw std::invalid_argument("Joint velocities and accelerations must be finite");
    }
  }
  if (explicit_local_accelerations != nullptr) {
    for (const auto & acceleration : *explicit_local_accelerations) {
      if (!finite(acceleration)) {
        throw std::invalid_argument("Explicit local accelerations must be finite");
      }
    }
  }

  GeneralizedVector generalized_velocity = GeneralizedVector::Zero();
  generalized_velocity.head<3>() = base_twist.linear_velocity_world;
  generalized_velocity.segment<3>(3) = base_twist.angular_velocity_world;
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    generalized_velocity(6 + static_cast<Eigen::Index>(dof)) = joint_velocity.theta_dot[dof];
  }

  GeneralizedVector generalized_acceleration = GeneralizedVector::Zero();
  generalized_acceleration.head<3>() = base_acceleration.linear_acceleration_world;
  generalized_acceleration.segment<3>(3) = base_acceleration.angular_acceleration_world;
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    generalized_acceleration(6 + static_cast<Eigen::Index>(dof)) =
      joint_acceleration.theta_ddot[dof];
  }

  HydrodynamicEvaluation output;
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    output.local_jacobians[link].setZero();
    output.local_twists[link].setZero();
    output.relative_twists[link].setZero();
    output.local_accelerations[link].setZero();
    output.added_mass_wrenches[link].setZero();
    output.linear_damping_wrenches[link].setZero();
    output.quadratic_damping_wrenches[link].setZero();
    output.gravity_wrenches[link].setZero();
    output.buoyancy_wrenches[link].setZero();
    output.total_link_wrenches[link].setZero();
    output.generalized_link_forces[link].setZero();

    const Eigen::Matrix3d frame = toEigen(state.link_frames[link]);
    const Eigen::Vector3d origin = toEigen(state.body_points[link]);
    const Eigen::Vector3d axis = toEigen(state.link_axes[link]);
    const Eigen::Vector3d center = origin - 0.5 * params_.link_length * axis;
    const Eigen::Vector3d center_offset = center - toEigen(state.head_position);

    Eigen::Matrix<double, kSpatialDofs, kGeneralizedDofs> world_jacobian;
    world_jacobian.setZero();
    world_jacobian.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
    world_jacobian.topRightCorner<3, 3>() = -skew(center_offset);
    world_jacobian.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

    for (std::size_t joint = 0; joint < link; ++joint) {
      Vec3 pitch_column;
      Vec3 yaw_column;
      Jacobian3D::pitchYawColumns(
        state.link_frames[joint + 1], state.joints.theta[pitchIndex(joint)],
        pitch_column, yaw_column);
      const Eigen::Vector3d joint_position = toEigen(state.body_points[joint + 1]);
      const Eigen::Vector3d lever = center - joint_position;
      const Eigen::Vector3d yaw_omega = -toEigen(yaw_column);
      const Eigen::Vector3d pitch_omega = -toEigen(pitch_column);

      world_jacobian.block<3, 1>(3, 6 + static_cast<Eigen::Index>(yawIndex(joint))) = yaw_omega;
      world_jacobian.block<3, 1>(0, 6 + static_cast<Eigen::Index>(yawIndex(joint))) =
        yaw_omega.cross(lever);
      world_jacobian.block<3, 1>(3, 6 + static_cast<Eigen::Index>(pitchIndex(joint))) =
        pitch_omega;
      world_jacobian.block<3, 1>(0, 6 + static_cast<Eigen::Index>(pitchIndex(joint))) =
        pitch_omega.cross(lever);
    }

    output.local_jacobians[link].topRows<3>() = frame.transpose() * world_jacobian.topRows<3>();
    output.local_jacobians[link].bottomRows<3>() = frame.transpose() *
      world_jacobian.bottomRows<3>();
    output.local_twists[link] = output.local_jacobians[link] * generalized_velocity;
    output.relative_twists[link] = output.local_twists[link];
    output.relative_twists[link].head<3>() -= frame.transpose() * fluid_current_world;

    if (explicit_local_accelerations != nullptr) {
      output.local_accelerations[link] = (*explicit_local_accelerations)[link];
    } else {
      output.local_accelerations[link] = output.local_jacobians[link] * generalized_acceleration;
    }

    const auto & link_params = params_.links[link];
    const SpatialVector added_mass_momentum = link_params.added_mass * output.relative_twists[link];
    output.added_mass_wrenches[link] = -(
      link_params.added_mass * output.local_accelerations[link] +
      spatialCrossStar(output.relative_twists[link], added_mass_momentum));
    output.linear_damping_wrenches[link] =
      -link_params.linear_damping.cwiseProduct(output.relative_twists[link]);
    output.quadratic_damping_wrenches[link] =
      -link_params.quadratic_damping.cwiseProduct(
      output.relative_twists[link].cwiseAbs().cwiseProduct(output.relative_twists[link]));

    const Eigen::Vector3d gravity_force_local =
      link_params.mass * frame.transpose() * params_.gravity_world;
    output.gravity_wrenches[link].head<3>() = gravity_force_local;
    output.gravity_wrenches[link].tail<3>() =
      link_params.center_of_mass.cross(gravity_force_local);

    const Eigen::Vector3d buoyancy_force_world =
      -params_.fluid_density * link_params.displaced_volume * params_.gravity_world;
    const Eigen::Vector3d buoyancy_force_local = frame.transpose() * buoyancy_force_world;
    output.buoyancy_wrenches[link].head<3>() = buoyancy_force_local;
    output.buoyancy_wrenches[link].tail<3>() =
      link_params.center_of_buoyancy.cross(buoyancy_force_local);

    output.total_link_wrenches[link] = output.added_mass_wrenches[link] +
      output.linear_damping_wrenches[link] + output.quadratic_damping_wrenches[link] +
      output.gravity_wrenches[link] + output.buoyancy_wrenches[link];
    output.generalized_link_forces[link] =
      output.local_jacobians[link].transpose() * output.total_link_wrenches[link];

    output.added_mass_force += output.local_jacobians[link].transpose() *
      output.added_mass_wrenches[link];
    output.damping_force += output.local_jacobians[link].transpose() *
      (output.linear_damping_wrenches[link] + output.quadratic_damping_wrenches[link]);
    output.gravity_buoyancy_force += output.local_jacobians[link].transpose() *
      (output.gravity_wrenches[link] + output.buoyancy_wrenches[link]);
    output.total_force += output.generalized_link_forces[link];
  }
  return output;
}

PinocchioHydrodynamicEvaluation FluidForceModel::evaluatePinocchio(
  const PinocchioKinematicsState & kinematics,
  const GeneralizedVector & velocity,
  const GeneralizedVector & acceleration,
  const Eigen::Vector3d & fluid_current_world) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid fluid model: " + error_);
  }
  if (!fluid_current_world.array().isFinite().all() ||
    !velocity.array().isFinite().all() || !acceleration.array().isFinite().all())
  {
    throw std::invalid_argument("Pinocchio fluid inputs must contain only finite values");
  }

  PinocchioHydrodynamicEvaluation output;
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    const auto & jacobian = kinematics.segment_jacobians[link];
    const auto & jacobian_dot = kinematics.segment_jacobian_time_variations[link];
    const Eigen::Matrix3d rotation = kinematics.segment_placements[link].rotation();
    const SpatialVector local_velocity = jacobian * velocity;
    const SpatialVector local_acceleration = jacobian * acceleration + jacobian_dot * velocity;
    SpatialVector relative_velocity = local_velocity;
    relative_velocity.head<3>() -= rotation.transpose() * fluid_current_world;
    output.relative_twists[link] = relative_velocity;
    output.local_accelerations[link] = local_acceleration;

    const auto & link_params = params_.links[link];
    const SpatialVector momentum = link_params.added_mass * relative_velocity;
    const SpatialVector acceleration_bias =
      link_params.added_mass * (jacobian_dot * velocity) +
      spatialCrossStar(relative_velocity, momentum);
    const SpatialVector added_wrench = -(
      link_params.added_mass * jacobian * acceleration + acceleration_bias);
    const SpatialVector damping_wrench =
      -link_params.linear_damping.cwiseProduct(relative_velocity) -
      link_params.quadratic_damping.cwiseProduct(
      relative_velocity.cwiseAbs().cwiseProduct(relative_velocity));

    const Eigen::Vector3d buoyancy_world =
      -params_.fluid_density * link_params.displaced_volume * params_.gravity_world;
    const Eigen::Vector3d buoyancy_force = rotation.transpose() * buoyancy_world;
    SpatialVector buoyancy_wrench = SpatialVector::Zero();
    buoyancy_wrench.head<3>() = buoyancy_force;
    buoyancy_wrench.tail<3>() = link_params.center_of_buoyancy.cross(buoyancy_force);

    output.added_mass_wrenches[link] = added_wrench;
    output.damping_wrenches[link] = damping_wrench;
    output.buoyancy_wrenches[link] = buoyancy_wrench;
    output.total_wrenches[link] = damping_wrench + buoyancy_wrench;
    output.added_mass_matrix += jacobian.transpose() * link_params.added_mass * jacobian;
    output.added_mass_bias_force += jacobian.transpose() * acceleration_bias;
    output.added_mass_force += jacobian.transpose() * added_wrench;
    output.damping_force += jacobian.transpose() * damping_wrench;
    output.buoyancy_force += jacobian.transpose() * buoyancy_wrench;
    output.total_force += jacobian.transpose() * output.total_wrenches[link];
  }
  return output;
}

}  // namespace asr_sdm_kinematic_dynamic_model

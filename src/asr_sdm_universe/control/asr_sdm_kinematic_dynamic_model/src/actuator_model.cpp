#include "asr_sdm_kinematic_dynamic_model/actuator_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{

ActuatorModel::ActuatorModel(const ActuatorModelParameters & params)
: params_(params), screw_model_(params.screw)
{
  if (!params_.maximum_segment_thrust.array().isFinite().all() ||
    (params_.maximum_segment_thrust.array() < 0.0).any())
  {
    error_ = "Maximum segment thrust must be finite and nonnegative";
    return;
  }
  if (!params_.rotor_force_axis_local.array().isFinite().all() ||
    params_.rotor_force_axis_local.norm() < 1.0e-12)
  {
    error_ = "Rotor force axis must be finite and nonzero";
    return;
  }
  if (!params_.thrust_linear.array().isFinite().all() ||
    !params_.thrust_quadratic.array().isFinite().all())
  {
    error_ = "Rotor thrust coefficients must be finite";
    return;
  }
  if (!params_.maximum_rotor_rate.array().isFinite().all() ||
    (params_.maximum_rotor_rate.array() < 0.0).any())
  {
    error_ = "Maximum rotor rate must be finite and nonnegative";
    return;
  }
  if (!std::isfinite(params_.thrust_time_constant) || params_.thrust_time_constant < 0.0) {
    error_ = "Thrust time constant must be finite and nonnegative";
    return;
  }
  if (!screw_model_.isValid()) {
    error_ = "Invalid screw-drive geometry: " + screw_model_.error();
    return;
  }
  params_.rotor_force_axis_local.normalize();
}

bool ActuatorModel::isValid() const
{
  return error_.empty();
}

const std::string & ActuatorModel::error() const
{
  return error_;
}

const ActuatorModelParameters & ActuatorModel::parameters() const
{
  return params_;
}

const ScrewDriveModel & ActuatorModel::screwDriveModel() const
{
  return screw_model_;
}

RotorVector ActuatorModel::commandedRotorThrust(
  const SegmentThrustVector & segment_thrust, const RotorVector & rotor_rate) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot resolve thrust for an invalid actuator model: " + error_);
  }
  if (!segment_thrust.array().isFinite().all() || !rotor_rate.array().isFinite().all()) {
    throw std::invalid_argument("Actuator inputs must contain only finite values");
  }

  RotorVector thrust = RotorVector::Zero();
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    const auto link_index = static_cast<Eigen::Index>(link);
    if (params_.command_mode == ActuatorCommandMode::AggregateThrust) {
      const double segment = std::clamp(
        segment_thrust(link_index), 0.0, params_.maximum_segment_thrust(link_index));
      for (std::size_t rotor = 0; rotor < 2; ++rotor) {
        thrust(static_cast<Eigen::Index>(rotorIndex(link, rotor))) = 0.5 * segment;
      }
      continue;
    }

    for (std::size_t rotor = 0; rotor < 2; ++rotor) {
      const auto index = static_cast<Eigen::Index>(rotorIndex(link, rotor));
      const double limit = params_.maximum_rotor_rate(index);
      const double rate = std::clamp(rotor_rate(index), -limit, limit);
      thrust(index) = params_.thrust_linear(index) * rate +
        params_.thrust_quadratic(index) * rate * std::abs(rate);
    }
    // Saturate the pair together so the segment budget still holds in this mode.
    const double pair_sum =
      thrust(static_cast<Eigen::Index>(rotorIndex(link, 0))) +
      thrust(static_cast<Eigen::Index>(rotorIndex(link, 1)));
    const double maximum = params_.maximum_segment_thrust(link_index);
    if (std::abs(pair_sum) > maximum && std::abs(pair_sum) > 0.0) {
      const double scale = maximum / std::abs(pair_sum);
      thrust(static_cast<Eigen::Index>(rotorIndex(link, 0))) *= scale;
      thrust(static_cast<Eigen::Index>(rotorIndex(link, 1))) *= scale;
    }
  }
  return thrust;
}

RotorVector ActuatorModel::advanceRotorThrust(
  const RotorVector & rotor_thrust, const RotorVector & commanded_rotor_thrust, double dt) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot advance thrust for an invalid actuator model: " + error_);
  }
  if (!rotor_thrust.array().isFinite().all() ||
    !commanded_rotor_thrust.array().isFinite().all())
  {
    throw std::invalid_argument("Actuator thrust state must contain only finite values");
  }
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("Actuator thrust step must be finite and nonnegative");
  }
  if (params_.thrust_time_constant <= 0.0) {
    return commanded_rotor_thrust;
  }
  // Exact solution of tau * f_dot = f_cmd - f over the step, so any dt stays stable.
  const double blend = 1.0 - std::exp(-dt / params_.thrust_time_constant);
  return rotor_thrust + blend * (commanded_rotor_thrust - rotor_thrust);
}

ActuatorEvaluation ActuatorModel::evaluate(
  const PinocchioKinematicsState & kinematics,
  const RotorVector & rotor_thrust,
  const JointTorqueVector & joint_torque) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid actuator model: " + error_);
  }
  if (!rotor_thrust.array().isFinite().all() || !joint_torque.array().isFinite().all()) {
    throw std::invalid_argument("Actuator inputs must contain only finite values");
  }

  ActuatorEvaluation output;
  output.rotor_thrust = rotor_thrust;
  output.generalized_force.segment<kNumJointDofs>(6) = joint_torque;
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    for (std::size_t rotor = 0; rotor < 2; ++rotor) {
      const auto index = static_cast<Eigen::Index>(rotorIndex(link, rotor));
      const double thrust = rotor_thrust(index);
      output.actual_segment_thrust(static_cast<Eigen::Index>(link)) += thrust;

      SpatialVector wrench = SpatialVector::Zero();
      wrench.head<3>() = thrust * params_.rotor_force_axis_local;
      if (params_.enable_reaction_torque) {
        // Ideal-screw power balance: driving torque tau = f * r * tan(alpha). The
        // segment feels the opposite sign, scaled by the screw handedness.
        output.rotor_reaction_torque(index) = -screw_model_.handedness(rotorIndex(link, rotor)) *
          thrust * screw_model_.reactionTorqueLever(rotorIndex(link, rotor));
        wrench.tail<3>() =
          output.rotor_reaction_torque(index) * params_.rotor_force_axis_local;
      }

      output.rotor_wrenches[link][rotor] = wrench;
      output.generalized_force += kinematics.rotor_jacobians[link][rotor].transpose() * wrench;
    }
  }
  return output;
}

ActuatorEvaluation ActuatorModel::evaluate(
  const PinocchioKinematicsState & kinematics,
  const SegmentThrustVector & segment_thrust,
  const JointTorqueVector & joint_torque,
  const RotorVector & rotor_rate) const
{
  return evaluate(kinematics, commandedRotorThrust(segment_thrust, rotor_rate), joint_torque);
}

}  // namespace asr_sdm_kinematic_dynamic_model

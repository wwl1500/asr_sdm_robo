#include "asr_sdm_kinematic_dynamic_model/actuator_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{

ActuatorModel::ActuatorModel(const ActuatorModelParameters & params)
: params_(params)
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

ActuatorEvaluation ActuatorModel::evaluate(
  const PinocchioKinematicsState & kinematics,
  const SegmentThrustVector & segment_thrust,
  const JointTorqueVector & joint_torque) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid actuator model: " + error_);
  }
  if (!segment_thrust.array().isFinite().all() || !joint_torque.array().isFinite().all()) {
    throw std::invalid_argument("Actuator inputs must contain only finite values");
  }

  ActuatorEvaluation output;
  output.generalized_force.segment<kNumJointDofs>(6) = joint_torque;
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    output.actual_segment_thrust(static_cast<Eigen::Index>(link)) = std::max(
      0.0, std::min(segment_thrust(static_cast<Eigen::Index>(link)),
      params_.maximum_segment_thrust(static_cast<Eigen::Index>(link))));
    for (std::size_t rotor = 0; rotor < 2; ++rotor) {
      SpatialVector wrench = SpatialVector::Zero();
      wrench.head<3>() = 0.5 * output.actual_segment_thrust(
        static_cast<Eigen::Index>(link)) * params_.rotor_force_axis_local;
      output.rotor_wrenches[link][rotor] = wrench;
      output.generalized_force += kinematics.rotor_jacobians[link][rotor].transpose() * wrench;
    }
  }
  return output;
}

}  // namespace asr_sdm_kinematic_dynamic_model

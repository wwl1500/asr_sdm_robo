#include "asr_sdm_kinematic_dynamic_model/joint_dynamics_model.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{

JointDynamicsModel::JointDynamicsModel(const JointDynamicsParameters & params)
: params_(params)
{
  if (!params_.viscous_damping.array().isFinite().all() ||
    (params_.viscous_damping.array() < 0.0).any())
  {
    error_ = "Joint viscous damping must be finite and nonnegative";
    return;
  }
  if (!params_.coulomb_friction.array().isFinite().all() ||
    (params_.coulomb_friction.array() < 0.0).any())
  {
    error_ = "Joint Coulomb friction must be finite and nonnegative";
    return;
  }
  if (!std::isfinite(params_.friction_velocity_scale) ||
    params_.friction_velocity_scale <= 0.0)
  {
    error_ = "Joint friction velocity scale must be finite and positive";
    return;
  }
  if (!params_.lower_limit.array().isFinite().all() ||
    !params_.upper_limit.array().isFinite().all())
  {
    error_ = "Joint limits must contain only finite values";
    return;
  }
  if ((params_.upper_limit.array() < params_.lower_limit.array()).any()) {
    error_ = "Joint upper limits must not fall below the lower limits";
    return;
  }
  if (!std::isfinite(params_.limit_stiffness) || params_.limit_stiffness < 0.0 ||
    !std::isfinite(params_.limit_damping) || params_.limit_damping < 0.0)
  {
    error_ = "Joint limit stiffness and damping must be finite and nonnegative";
    return;
  }
}

bool JointDynamicsModel::isValid() const
{
  return error_.empty();
}

const std::string & JointDynamicsModel::error() const
{
  return error_;
}

const JointDynamicsParameters & JointDynamicsModel::parameters() const
{
  return params_;
}

JointDynamicsEvaluation JointDynamicsModel::evaluate(
  const JointTorqueVector & joint_position, const JointTorqueVector & joint_velocity) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid joint dynamics model: " + error_);
  }
  if (!joint_position.array().isFinite().all() || !joint_velocity.array().isFinite().all()) {
    throw std::invalid_argument("Joint dynamics inputs must contain only finite values");
  }

  JointDynamicsEvaluation output;
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    const auto index = static_cast<Eigen::Index>(dof);
    const double position = joint_position(index);
    const double velocity = joint_velocity(index);

    output.viscous_torque(index) = -params_.viscous_damping(index) * velocity;
    output.friction_torque(index) = -params_.coulomb_friction(index) *
      std::tanh(velocity / params_.friction_velocity_scale);

    if (params_.limit_stiffness > 0.0) {
      // Only the component of the velocity that digs further into the stop is damped,
      // so the joint is never pulled back towards the limit while it is escaping.
      if (position > params_.upper_limit(index)) {
        const double penetration = position - params_.upper_limit(index);
        output.limit_torque(index) = -params_.limit_stiffness * penetration -
          params_.limit_damping * std::max(0.0, velocity);
      } else if (position < params_.lower_limit(index)) {
        const double penetration = params_.lower_limit(index) - position;
        output.limit_torque(index) = params_.limit_stiffness * penetration +
          params_.limit_damping * std::max(0.0, -velocity);
      }
    }
  }
  output.total_torque =
    output.viscous_torque + output.friction_torque + output.limit_torque;
  return output;
}

}  // namespace asr_sdm_kinematic_dynamic_model

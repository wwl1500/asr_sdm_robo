// Copyright (c) 2026.
// MpcSafetyMapper implementation.

#include "asr_sdm_kinematic_dynamic_model/mpc_safety_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace asr_sdm_kinematic_dynamic_model
{

namespace
{

bool isFinite(double x) {return std::isfinite(x);}

bool isFiniteState(const UnderwaterSimulatorState & state)
{
  for (int i = 0; i < state.configuration.size(); ++i) {
    if (!isFinite(state.configuration(i))) {return false;}
  }
  for (int i = 0; i < state.velocity.size(); ++i) {
    if (!isFinite(state.velocity(i))) {return false;}
  }
  return true;
}

bool isFiniteInput(const UnderwaterSimulatorInput & input)
{
  for (int i = 0; i < input.segment_thrust.size(); ++i) {
    if (!isFinite(input.segment_thrust(i))) {return false;}
  }
  for (int i = 0; i < input.joint_torque.size(); ++i) {
    if (!isFinite(input.joint_torque(i))) {return false;}
  }
  return true;
}

/// Clamp a value into [-limit, +limit].  Returns true if clamping happened.
bool clampSymmetric(double & value, double limit)
{
  const double original = value;
  value = std::clamp(value, -limit, limit);
  return value != original;
}

/// Clamp a value into [0, +limit].  Returns true if clamping happened.
bool clampNonNegative(double & value, double limit)
{
  const double original = value;
  value = std::clamp(value, 0.0, limit);
  return value != original;
}

}  // namespace

MpcSafetyMapper::MpcSafetyMapper(const MpcSafetyParameters & params)
: params_(params)
{
}

void MpcSafetyMapper::updateParameters(const MpcSafetyParameters & params)
{
  params_ = params;
}

const MpcSafetyParameters & MpcSafetyMapper::parameters() const
{
  return params_;
}

void MpcSafetyMapper::reset()
{
  has_last_valid_command_ = false;
  last_valid_command_ = UnderwaterSimulatorInput{};
  status_ = MpcSafetyStatus{};
}

const MpcSafetyStatus & MpcSafetyMapper::lastStatus() const
{
  return status_;
}

bool MpcSafetyMapper::isSafeModeActive() const
{
  return status_.safe_mode_active;
}

UnderwaterSimulatorInput MpcSafetyMapper::map(
  const UnderwaterSimulatorInput & mpc_command,
  const UnderwaterSimulatorState & measured_state,
  const ReducedConfiguration & initial_state,
  double dt)
{
  // Reset status.
  status_ = MpcSafetyStatus{};

  // ---- 1. Fault detection on measured state. ----
  if (!isFiniteState(measured_state)) {
    status_.state_was_invalid = true;
    status_.fault_message = "measured state contains NaN/Inf";
    status_.safe_mode_active = true;
  } else {
    const double lin_speed = measured_state.velocity.head<3>().norm();
    const double ang_speed = measured_state.velocity.segment<3>(3).norm();
    if (lin_speed > params_.max_linear_velocity_fault) {
      status_.state_exceeded_fault_threshold = true;
      std::ostringstream oss;
      oss << "linear speed " << lin_speed << " m/s > threshold "
          << params_.max_linear_velocity_fault << " m/s";
      status_.fault_message = oss.str();
      status_.safe_mode_active = true;
    } else if (ang_speed > params_.max_angular_velocity_fault) {
      status_.state_exceeded_fault_threshold = true;
      std::ostringstream oss;
      oss << "angular speed " << ang_speed << " rad/s > threshold "
          << params_.max_angular_velocity_fault << " rad/s";
      status_.fault_message = oss.str();
      status_.safe_mode_active = true;
    } else {
      const Eigen::Vector3d pos_drift = measured_state.configuration.head<3>() -
        initial_state.head<3>();
      if (pos_drift.norm() > params_.max_position_deviation_fault) {
        status_.state_exceeded_fault_threshold = true;
        std::ostringstream oss;
        oss << "position drift " << pos_drift.norm() << " m > threshold "
            << params_.max_position_deviation_fault << " m";
        status_.fault_message = oss.str();
        status_.safe_mode_active = true;
      }
    }
  }

  // ---- 2. Fault detection on MPC output. ----
  if (!isFiniteInput(mpc_command)) {
    status_.mpc_output_was_invalid = true;
    status_.safe_mode_active = true;
    if (!status_.fault_message.empty()) {status_.fault_message += "; ";}
    status_.fault_message += "MPC output contains NaN/Inf";
  }

  // ---- 3. Build the safe command. ----
  UnderwaterSimulatorInput safe = mpc_command;

  if (status_.safe_mode_active) {
    // Substitute safe-mode scaled version of last valid command (or zero).
    if (has_last_valid_command_) {
      safe.segment_thrust = last_valid_command_.segment_thrust *
        params_.safe_mode_thrust_scale;
      safe.joint_torque = last_valid_command_.joint_torque *
        params_.safe_mode_torque_scale;
    } else {
      safe.segment_thrust.setZero();
      safe.joint_torque.setZero();
    }
    status_.input_was_clamped = true;
    status_.slew_rate_was_clamped = false;
  } else {
    // ---- 4. Apply actuator saturation. ----
    bool any_clamp = false;
    for (Eigen::Index i = 0; i < safe.segment_thrust.size(); ++i) {
      if (clampNonNegative(safe.segment_thrust(i), params_.max_segment_thrust(i))) {
        any_clamp = true;
      }
    }
    for (Eigen::Index i = 0; i < safe.joint_torque.size(); ++i) {
      if (clampSymmetric(safe.joint_torque(i), params_.max_joint_torque(i))) {
        any_clamp = true;
      }
    }
    status_.input_was_clamped = any_clamp;

    // ---- 5. Apply slew-rate limits. ----
    if (has_last_valid_command_ && dt > 0.0) {
      const double max_dthrust = params_.max_segment_thrust_rate * dt;
      const double max_dtorque = params_.max_joint_torque_rate * dt;
      bool any_rate_clamp = false;
      const auto & last_thrust = last_valid_command_.segment_thrust;
      const auto & last_torque = last_valid_command_.joint_torque;
      for (Eigen::Index i = 0; i < safe.segment_thrust.size(); ++i) {
        double delta = safe.segment_thrust(i) - last_thrust(i);
        if (clampSymmetric(delta, max_dthrust)) {
          any_rate_clamp = true;
        }
        safe.segment_thrust(i) = last_thrust(i) + delta;
      }
      for (Eigen::Index i = 0; i < safe.joint_torque.size(); ++i) {
        double delta = safe.joint_torque(i) - last_torque(i);
        if (clampSymmetric(delta, max_dtorque)) {
          any_rate_clamp = true;
        }
        safe.joint_torque(i) = last_torque(i) + delta;
      }
      status_.slew_rate_was_clamped = any_rate_clamp;
    }
  }

  // ---- 6. Update last-valid-command cache. ----
  // Only update if the safe command is finite (always true after clamping)
  // and we are NOT in safe mode (so that a hard fault doesn't get cached
  // and re-scaled forever).
  if (!status_.safe_mode_active) {
    last_valid_command_ = safe;
    has_last_valid_command_ = true;
  }

  return safe;
}

}  // namespace asr_sdm_kinematic_dynamic_model

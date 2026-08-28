// Copyright (c) 2026.
// Safety mapping layer between MPC solver output and hardware command.
//
// Sits between the MPC controller (which produces optimal but unconstrained
// inputs) and the simulator / hardware bus (which requires physically safe
// commands).  Responsibilities:
//   - Enforce actuator saturation limits (segment thrust, joint torque)
//   - Enforce actuator slew rate limits (prevent step commands that exceed
//     what the motor drivers can deliver)
//   - Detect faults in the MPC output (NaN/Inf) and substitute a safe
//     zero-output or the previous valid command
//   - Detect faults in the measured state (excessive velocity / position)
//     and gracefully reduce control authority
//   - Provide a "safe-mode" pass-through flag for emergency stop

#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_MPC_SAFETY_MAPPER_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_MPC_SAFETY_MAPPER_HPP_

#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <Eigen/Dense>

#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

/// Tunable parameters for MpcSafetyMapper.
struct MpcSafetyParameters
{
  // ---- Actuator saturation (hard limits).  These are HARD limits that
  // cannot be exceeded under any circumstance.  Set to per-link from URDF. ----
  Eigen::Vector4d max_segment_thrust{Eigen::Vector4d::Constant(10.0)};   // [N]
  Eigen::Matrix<double, kNumJointDofs, 1> max_joint_torque{
    Eigen::Matrix<double, kNumJointDofs, 1>::Constant(10.0)};           // [Nm]

  // ---- Actuator slew rate limits (soft limits to prevent step commands). ----
  double max_segment_thrust_rate{50.0};     // [N/s]   per segment per second
  double max_joint_torque_rate{30.0};       // [Nm/s]  per joint per second

  // ---- Fault detection thresholds. ----
  // Measured-state faults trigger safe-mode.
  double max_linear_velocity_fault{5.0};     // [m/s]   above -> safe-mode
  double max_angular_velocity_fault{5.0};    // [rad/s] above -> safe-mode
  double max_position_deviation_fault{10.0}; // [m]     from initial pose -> safe-mode

  // ---- Safe-mode output.  When true, the mapper outputs this scaled
  // version of the MPC command instead of the raw command. ----
  double safe_mode_thrust_scale{0.0};     // 0.0 = full stop
  double safe_mode_torque_scale{0.0};
};

/// Status of the most recent safety mapping.
struct MpcSafetyStatus
{
  bool input_was_clamped{false};        // any actuator limit clamped the output
  bool slew_rate_was_clamped{false};    // any actuator rate limit clamped the output
  bool safe_mode_active{false};         // mapper fell back to safe output
  bool mpc_output_was_invalid{false};   // NaN/Inf detected in MPC output
  bool state_was_invalid{false};        // NaN/Inf detected in measured state
  bool state_exceeded_fault_threshold{false};  // velocity/position above fault threshold

  std::string fault_message;            // human-readable reason if any fault
};

/// Safety mapping layer.
///
/// Call `map(...)` once per control cycle, AFTER `controller.computeControl(...)`
/// and BEFORE passing the result to the simulator or hardware bus.  Holds
/// the previous valid command internally for slew-rate limiting and fault
/// recovery.
class MpcSafetyMapper
{
public:
  explicit MpcSafetyMapper(const MpcSafetyParameters & params);

  /// Update tunable parameters at runtime.
  void updateParameters(const MpcSafetyParameters & params);

  const MpcSafetyParameters & parameters() const;

  /// Map an MPC command into a safe hardware command.
  ///
  /// @param mpc_command       Raw MPC output from the controller
  /// @param measured_state    Current measured state for fault detection
  /// @param initial_state     Reference pose for "drifted too far" detection
  /// @param dt                Control period in seconds (for slew rate)
  /// @return Safe command (always finite, within all limits)
  UnderwaterSimulatorInput map(
    const UnderwaterSimulatorInput & mpc_command,
    const UnderwaterSimulatorState & measured_state,
    const ReducedConfiguration & initial_state,
    double dt);

  /// Get the status of the most recent `map()` call.
  const MpcSafetyStatus & lastStatus() const;

  /// True if the most recent call was in safe mode.
  bool isSafeModeActive() const;

  /// Reset internal state (clears last valid command).  Call when the
  /// controller is re-initialized.
  void reset();

private:
  MpcSafetyParameters params_;
  MpcSafetyStatus status_;

  UnderwaterSimulatorInput last_valid_command_{};
  bool has_last_valid_command_{false};
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_MPC_SAFETY_MAPPER_HPP_

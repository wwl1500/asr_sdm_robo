// Copyright (c) 2025.
// High-level MPC controller for underwater robot with full dynamics and constraints.

#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_MPC_CONTROLLER_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_MPC_CONTROLLER_HPP_

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace asr_sdm_kinematic_dynamic_model
{

/// MPC controller parameters
struct UnderwaterMpcParameters
{
  // Prediction horizon
  double Thor = 1.0;                    // Prediction time horizon [s]
  int Nhor = 50;                        // Number of prediction steps
  double dt = 0.02;                     // Discretization time step [s]

  // Solver options
  int max_grad_iter = 5;                // Maximum gradient iterations per step
  int max_mult_iter = 1;                // Maximum multiplier iterations
  std::string integrator = "erk2";      // Integration method: erk2, heun, ruku45
  double penalty_min = 1e3;             // Minimum penalty for constraints

  // Cost weights
  double q_position = 10.0;             // Position tracking weight
  double q_orientation = 5.0;           // Orientation tracking weight
  double q_linear_velocity = 2.0;       // Linear velocity tracking weight
  double q_angular_velocity = 2.0;      // Angular velocity tracking weight
  double q_joint_position = 5.0;        // Joint position tracking weight
  double q_joint_velocity = 2.0;        // Joint velocity tracking weight

  double r_thrust = 0.1;                // Thrust control cost
  double r_joint_torque = 0.01;         // Joint torque control cost

  double terminal_position_scale = 20.0; // Terminal cost scale for position
  double terminal_velocity_scale = 10.0; // Terminal cost scale for velocity

  // Control limits
  double max_linear_velocity = 2.0;     // Maximum linear velocity [m/s]
  double max_angular_velocity = 1.0;    // Maximum angular velocity [rad/s]
  double max_joint_velocity = 2.0;      // Maximum joint velocity [rad/s]

  // Thrust limits are read from ActuatorModel
  double min_joint_torque = -10.0;      // Minimum joint torque [Nm]
  double max_joint_torque = 10.0;       // Maximum joint torque [Nm]

  // Options
  bool enable_terminal_cost = true;     // Use terminal cost
  bool enable_velocity_constraints = true;  // Enable velocity limit constraints
  /// When true (default), the controller warm-starts the next MPC solve with
  /// the previously applied control (u0 = previous_unext).  GRAMPC also does
  /// an internal shift of the prediction sequence, so this gives the solver a
  /// good initial guess for the first node.  Disable to force a cold start
  /// every period (useful for benchmarking and unit tests).
  bool enable_warm_start = true;
  bool verbose = false;                 // Print diagnostic info
  double log_throttle_sec = 1.0;        // Throttle logging to this interval
};

/// MPC solution result
struct UnderwaterMpcResult
{
  UnderwaterSimulatorInput control{};   // Optimal control at current time
  ReducedConfiguration predicted_config{}; // Predicted configuration at Thor
  ReducedVelocity predicted_velocity{}; // Predicted velocity at Thor

  double solve_time_ms{0.0};            // Solution time in milliseconds
  int iterations{0};                    // Number of iterations taken
  double cost{0.0};                     // Optimal cost

  bool success{false};                  // Whether solve succeeded
  std::string error_message{};          // Error message if failed

  // Tracking errors
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_error{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, kReducedNv, 1> velocity_error{
    Eigen::Matrix<double, kReducedNv, 1>::Zero()};
};

/// A single snapshot of a time-varying reference trajectory.
struct MpcTrajectoryPoint
{
  double time{0.0};                                       // [s]
  ReducedConfiguration configuration{ReducedConfiguration::Zero()};
  ReducedVelocity velocity{ReducedVelocity::Zero()};
  ReducedAcceleration acceleration{ReducedAcceleration::Zero()};
  Eigen::Vector3d fluid_current_world{Eigen::Vector3d::Zero()};
};

/// Reference trajectory for MPC tracking.
///
/// Supports two modes:
///   1. Static (backward-compatible): set `has_valid_target` = true and fill
///      the flat fields directly.  The controller uses them as the constant
///      reference for the entire prediction horizon.
///   2. Time-varying: call `setTrajectory(points)` to install a list of
///      timestamped waypoints.  The controller then calls `evaluate(t)` to
///      sample the trajectory at any time within [t0, tend] using linear
///      interpolation between adjacent waypoints (clamped at boundaries).
///
/// Example (time-varying):
///   MpcReferenceTrajectory ref;
///   ref.setTrajectory({
///       {0.0, q0, v0, a0, {}},
///       {1.0, q1, v1, a1, {}},
///       {2.0, q2, v2, a2, {}},
///   });
struct MpcReferenceTrajectory
{
  /// Build a trajectory from an ordered list of waypoints.
  /// Linear interpolation is used between consecutive points.
  /// Times must be strictly increasing.
  void setTrajectory(const std::vector<MpcTrajectoryPoint> & points);

  /// Sample the trajectory at absolute time `t`.
  /// Returns the interpolated point at `t`, clamped to [t_start, t_end].
  /// If no trajectory is installed, returns the static snapshot fields.
  MpcTrajectoryPoint evaluate(double t) const;

  // ---- Static snapshot (backward-compatible) ----
  ReducedConfiguration target_configuration{ReducedConfiguration::Zero()};
  ReducedVelocity target_velocity{ReducedVelocity::Zero()};
  ReducedAcceleration target_acceleration{ReducedAcceleration::Zero()};

  // Fluid current for prediction
  Eigen::Vector3d fluid_current_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d fluid_current_acceleration_world{Eigen::Vector3d::Zero()};

  bool has_valid_target{false};

private:
  std::vector<MpcTrajectoryPoint> trajectory_points_;
};

/// High-level MPC controller that wraps GrampcUnderwaterDynamics
/// Similar to HeadCommandMPC but for full underwater robot dynamics
class UnderwaterMpcController
{
public:
  explicit UnderwaterMpcController(
    const UnderwaterSimulatorParameters & sim_params,
    const UnderwaterMpcParameters & mpc_params = UnderwaterMpcParameters{});

  ~UnderwaterMpcController();

  // Non-copyable
  UnderwaterMpcController(const UnderwaterMpcController &) = delete;
  UnderwaterMpcController & operator=(const UnderwaterMpcController &) = delete;

  /// Check if controller is ready to use
  bool isValid() const;
  const std::string & error() const;

  /// Compute optimal control for current state
  /// @param current_config Current robot configuration
  /// @param current_velocity Current robot velocity
  /// @param reference Reference trajectory to track
  /// @return MPC solution with optimal control
  UnderwaterMpcResult computeControl(
    const ReducedConfiguration & current_config,
    const ReducedVelocity & current_velocity,
    const MpcReferenceTrajectory & reference,
    double current_time = 0.0);

  /// Update MPC parameters at runtime
  void updateParameters(const UnderwaterMpcParameters & params);

  /// Get current MPC parameters
  const UnderwaterMpcParameters & parameters() const;

  /// Access underlying simulator (for inspection)
  const UnderwaterSimulator & simulator() const;

  /// Get last solution
  const UnderwaterMpcResult & lastResult() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_MPC_CONTROLLER_HPP_

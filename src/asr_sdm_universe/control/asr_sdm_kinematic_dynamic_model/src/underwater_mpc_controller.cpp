// Copyright (c) 2025.
// Underwater MPC controller implementation.

#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"

#include <grampc_s/grampc_s.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace asr_sdm_kinematic_dynamic_model
{

void MpcReferenceTrajectory::setTrajectory(const std::vector<MpcTrajectoryPoint> & points)
{
  trajectory_points_ = points;
}

MpcTrajectoryPoint MpcReferenceTrajectory::evaluate(double t) const
{
  if (trajectory_points_.empty()) {
    MpcTrajectoryPoint out;
    out.time = t;
    out.configuration = target_configuration;
    out.velocity = target_velocity;
    out.acceleration = target_acceleration;
    out.fluid_current_world = fluid_current_world;
    return out;
  }

  const double t_start = trajectory_points_.front().time;
  const double t_end = trajectory_points_.back().time;

  if (t <= t_start) {
    return trajectory_points_.front();
  }
  if (t >= t_end) {
    return trajectory_points_.back();
  }

  // Binary search for the interval [i, i+1] containing t.
  std::size_t left = 0;
  std::size_t right = trajectory_points_.size() - 1;
  while (left + 1 < right) {
    std::size_t mid = left + (right - left) / 2;
    if (trajectory_points_[mid].time <= t) {
      left = mid;
    } else {
      right = mid;
    }
  }

  const MpcTrajectoryPoint & p0 = trajectory_points_[left];
  const MpcTrajectoryPoint & p1 = trajectory_points_[right];
  const double dt = p1.time - p0.time;
  const double alpha = (dt > 0.0) ? (t - p0.time) / dt : 0.0;

  MpcTrajectoryPoint out;
  out.time = t;
  out.configuration = p0.configuration + alpha * (p1.configuration - p0.configuration);
  out.velocity = p0.velocity + alpha * (p1.velocity - p0.velocity);
  out.acceleration = p0.acceleration + alpha * (p1.acceleration - p0.acceleration);
  out.fluid_current_world = p0.fluid_current_world +
    alpha * (p1.fluid_current_world - p0.fluid_current_world);
  return out;
}

namespace
{

/// Build cost weights from MPC parameters (full-state [q(13), v(12)])
GrampcCostWeights buildCostWeights(const UnderwaterMpcParameters & params)
{
  GrampcCostWeights weights;

  // Configuration weights (first 13 of x):
  //   q = [base_position(3), base_quat(4), joint_positions(6)]
  // Quaternion weights should be small (e.g. 0.1) compared to position to
  // avoid dominating the cost; orientation error is dominated by the
  // position and velocity components in practice.
  weights.state_weight.segment<3>(0).setConstant(params.q_position);     // base position
  weights.state_weight.segment<4>(3).setConstant(0.1 * params.q_orientation); // quat (small)
  weights.state_weight.segment<6>(7).setConstant(params.q_joint_position); // joint positions

  // Velocity weights (last 12 of x):
  //   v = [base_linear_vel(3), base_angular_vel(3), joint_velocities(6)]
  weights.state_weight.segment<3>(kMpcConfigurationTangentDim).setConstant(
    params.q_linear_velocity);
  weights.state_weight.segment<3>(kMpcConfigurationTangentDim + 3).setConstant(
    params.q_angular_velocity);
  weights.state_weight.segment<6>(kMpcConfigurationTangentDim + 6).setConstant(
    params.q_joint_velocity);

  // Control weights: [segment_thrust(4), joint_torque(6)]
  weights.control_weight.segment<kNumLinks>(0).setConstant(params.r_thrust);
  weights.control_weight.segment<kNumJointDofs>(kNumLinks).setConstant(params.r_joint_torque);

  // Terminal weights
  weights.terminal_weight.segment<3>(0).setConstant(
    params.q_position * params.terminal_position_scale);
  weights.terminal_weight.segment<4>(3).setConstant(
    0.1 * params.q_orientation * params.terminal_position_scale);
  weights.terminal_weight.segment<6>(7).setConstant(
    params.q_joint_position * params.terminal_position_scale);

  weights.terminal_weight.segment<3>(kMpcConfigurationTangentDim).setConstant(
    params.q_linear_velocity * params.terminal_velocity_scale);
  weights.terminal_weight.segment<3>(kMpcConfigurationTangentDim + 3).setConstant(
    params.q_angular_velocity * params.terminal_velocity_scale);
  weights.terminal_weight.segment<6>(kMpcConfigurationTangentDim + 6).setConstant(
    params.q_joint_velocity * params.terminal_velocity_scale);

  return weights;
}

/// Extract position and orientation errors for diagnostics
void extractSpatialErrors(
  const Eigen::Matrix<double, kMpcConfigurationTangentDim, 1> & config_error,
  Eigen::Vector3d & position_error,
  Eigen::Vector3d & orientation_error)
{
  position_error = config_error.segment<3>(0);
  orientation_error = config_error.segment<3>(3);
}

}  // namespace

class UnderwaterMpcController::Impl
{
public:
  explicit Impl(
    const UnderwaterSimulatorParameters & sim_params,
    const UnderwaterMpcParameters & mpc_params)
  : params_(mpc_params)
  {
    // Build cost weights
    GrampcCostWeights weights = buildCostWeights(mpc_params);

    // Create dynamics interface
    GrampcDimensionConfig dimensions;  // Use default: 24 state, 10 control
    dynamics_ = std::make_shared<GrampcUnderwaterDynamics>(
      sim_params, dimensions, weights);

    if (!dynamics_->isValid()) {
      error_ = "Failed to create dynamics interface: " + dynamics_->error();
      return;
    }

    // Create GRAMPC solver
    try {
      solver_ = std::make_unique<grampc::Grampc>(dynamics_);
    } catch (const std::exception & e) {
      error_ = std::string("Failed to create GRAMPC solver: ") + e.what();
      dynamics_.reset();
      return;
    }

    // Configure solver
    configureSolver();

    // Initialize control limits
    initializeControlLimits();

    // Allocate workspace
    x0_.resize(kMpcStateDim);
    xdes_.resize(kMpcStateDim);
    u0_.resize(kMpcControlDim);
    udes_.resize(kMpcControlDim);

    last_log_time_ = std::chrono::steady_clock::now();
  }

  void configureSolver()
  {
    solver_->setparam_real("Thor", static_cast<typeRNum>(params_.Thor));
    solver_->setparam_real("dt", static_cast<typeRNum>(params_.dt));
    solver_->setparam_real("t0", 0.0);

    solver_->setopt_int("Nhor", params_.Nhor);
    solver_->setparam_real("Thor", static_cast<typeRNum>(params_.Thor));  // Reset after Nhor change

    solver_->setopt_int("MaxGradIter", params_.max_grad_iter);
    solver_->setopt_int("MaxMultIter", params_.max_mult_iter);
    solver_->setopt_string("Integrator", params_.integrator.c_str());
    solver_->setopt_real("PenaltyMin", static_cast<typeRNum>(params_.penalty_min));

    if (params_.enable_terminal_cost) {
      solver_->setopt_string("TerminalCost", "on");
    } else {
      solver_->setopt_string("TerminalCost", "off");
    }

    // Configure velocity constraints in dynamics interface
    if (params_.enable_velocity_constraints) {
      dynamics_->setVelocityLimits(
        params_.max_linear_velocity,
        params_.max_angular_velocity,
        params_.max_joint_velocity);
      solver_->setopt_string("InequalityConstraints", "on");
    } else {
      dynamics_->disableVelocityLimits();
      solver_->setopt_string("InequalityConstraints", "off");
    }
  }

  void initializeControlLimits()
  {
    umin_.resize(kMpcControlDim);
    umax_.resize(kMpcControlDim);

    // Thrust limits from actuator model
    const auto & maximum_thrust =
      dynamics_->simulator().actuatorModel().parameters().maximum_segment_thrust;

    for (std::size_t i = 0; i < kNumLinks; ++i) {
      umin_[i] = 0.0;  // Non-negative thrust in aggregate mode
      umax_[i] = static_cast<typeRNum>(maximum_thrust(i));
    }

    // Joint torque limits
    for (std::size_t i = kNumLinks; i < kMpcControlDim; ++i) {
      umin_[i] = static_cast<typeRNum>(params_.min_joint_torque);
      umax_[i] = static_cast<typeRNum>(params_.max_joint_torque);
    }

    solver_->setparam_real_vector("umin", umin_.data());
    solver_->setparam_real_vector("umax", umax_.data());
  }

  UnderwaterMpcResult computeControl(
    const ReducedConfiguration & current_config,
    const ReducedVelocity & current_velocity,
    const MpcReferenceTrajectory & reference,
    double current_time)
  {
    UnderwaterMpcResult result;

    if (!dynamics_ || !solver_) {
      result.error_message = error_;
      return result;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    // Sample the reference trajectory at the current time.
    const MpcTrajectoryPoint ref_now = reference.evaluate(current_time);
    // Update fluid current (acceleration not stored per-point; use zero).
    dynamics_->updateFluidCurrent(ref_now.fluid_current_world, Eigen::Vector3d::Zero());

    // Build initial state in FULL-STATE form: x0 = [q_measured, v_measured].
    // The first 13 entries ARE the configuration; the last 12 are the body
    // velocity.  We re-normalize the quaternion defensively so GramPC's
    // internal RK never sees a non-unit quaternion.
    std::fill(x0_.begin(), x0_.end(), 0.0);
    {
      ReducedConfiguration q_norm = current_config;
      q_norm.segment<4>(3).normalize();
      for (int i = 0; i < kReducedNq; ++i) {
        x0_[i] = static_cast<typeRNum>(q_norm(i));
      }
    }
    for (int i = 0; i < kReducedNv; ++i) {
      x0_[kMpcConfigurationTangentDim + i] = static_cast<typeRNum>(current_velocity(i));
    }

    // Build desired state: xdes = [q_des, v_des].
    // Use ref_now (sampled at current_time).  For static trajectory (no
    // trajectory installed via setTrajectory), evaluate(t) returns the static
    // snapshot fields, so this is fully backward-compatible.
    if (reference.has_valid_target) {
      // Use ref_now (sampled at current_time).  For static trajectory,
      // ref_now == static snapshot fields via evaluate(0.0).
      ReducedConfiguration q_des_norm = ref_now.configuration;
      q_des_norm.segment<4>(3).normalize();
      for (int i = 0; i < kReducedNq; ++i) {
        xdes_[i] = static_cast<typeRNum>(q_des_norm(i));
      }

      for (int i = 0; i < kReducedNv; ++i) {
        xdes_[kMpcConfigurationTangentDim + i] =
          static_cast<typeRNum>(ref_now.velocity(i));
      }

      // Store simple errors for diagnostics (position / orientation).
      // Note: these are naive Euclidean differences; quaternion wrap-around
      // makes orientation errors ambiguous at large angles and is only used
      // for log output.
      Eigen::Vector3d raw_pos_err = current_config.head<3>() - q_des_norm.head<3>();
      result.position_error = raw_pos_err;
      result.velocity_error = current_velocity - ref_now.velocity;
      result.orientation_error = Eigen::Vector3d::Zero();
    } else {
      // No target: regulate to a "stay-in-place" reference using the
      // current configuration (zero velocity desired).
      ReducedConfiguration q_norm = current_config;
      q_norm.segment<4>(3).normalize();
      std::fill(xdes_.begin(), xdes_.end(), 0.0);
      for (int i = 0; i < kReducedNq; ++i) {
        xdes_[i] = static_cast<typeRNum>(q_norm(i));
      }
    }

    // Incremental warm-start: use the previous control as the first node
    // of the new prediction horizon if available and valid.  GRAMPC itself
    // shifts the prediction by one step internally; this just gives the
    // solver a better initial guess for the first node u0.  Falls back to
    // zero if no previous solution exists or if warm-start is disabled.
    if (params_.enable_warm_start && last_result_.success) {
      // Shift the previous sequence forward by one step: the new u0 is the
      // previous u1 (i.e. last_result_.control), but we don't have the full
      // sequence cached.  Use the previously applied control as the seed and
      // let GRAMPC's internal shift populate the remaining nodes.
      u0_[0] = static_cast<typeRNum>(last_result_.control.segment_thrust(0));
      for (std::size_t i = 0; i < kNumLinks; ++i) {
        u0_[i] = static_cast<typeRNum>(last_result_.control.segment_thrust(i));
      }
      for (std::size_t i = 0; i < kNumJointDofs; ++i) {
        u0_[kNumLinks + i] = static_cast<typeRNum>(last_result_.control.joint_torque(i));
      }
    } else {
      std::fill(u0_.begin(), u0_.end(), 0.0);
    }
    std::fill(udes_.begin(), udes_.end(), 0.0);

    // Set GRAMPC parameters
    solver_->setparam_real_vector("x0", x0_.data());
    solver_->setparam_real_vector("xdes", xdes_.data());
    solver_->setparam_real_vector("u0", u0_.data());
    solver_->setparam_real_vector("udes", udes_.data());
    solver_->setparam_real("t0", 0.0);

    // Solve MPC
    try {
      solver_->run();
    } catch (const std::exception & e) {
      result.error_message = std::string("GRAMPC solve failed: ") + e.what();
      return result;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.solve_time_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time).count();

    // Extract solution
    const typeGRAMPCsol * sol = solver_->getSolution();
    if (!sol || !sol->unext) {
      result.error_message = "GRAMPC returned null solution";
      return result;
    }

    // Check for NaN/Inf
    bool control_valid = true;
    for (int i = 0; i < kMpcControlDim; ++i) {
      if (!std::isfinite(sol->unext[i])) {
        control_valid = false;
        break;
      }
    }

    if (!control_valid) {
      result.error_message = "GRAMPC returned non-finite control";
      return result;
    }

    // Extract and clamp control
    for (std::size_t i = 0; i < kNumLinks; ++i) {
      result.control.segment_thrust(i) = std::clamp(
        static_cast<double>(sol->unext[i]),
        static_cast<double>(umin_[i]),
        static_cast<double>(umax_[i]));
    }

    for (std::size_t i = 0; i < kNumJointDofs; ++i) {
      result.control.joint_torque(i) = std::clamp(
        static_cast<double>(sol->unext[kNumLinks + i]),
        static_cast<double>(umin_[kNumLinks + i]),
        static_cast<double>(umax_[kNumLinks + i]));
    }

    // Extract cost and iteration info
    result.cost = static_cast<double>(sol->J[0]);
    result.iterations = static_cast<int>(sol->iter[1]);  // Gradient iterations
    result.success = true;

    // Log diagnostics
    logDiagnostics(result, reference);

    last_result_ = result;
    return result;
  }

  void logDiagnostics(
    const UnderwaterMpcResult & result,
    const MpcReferenceTrajectory & reference)
  {
    if (!params_.verbose) {
      return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_log_time_).count();

    if (elapsed < params_.log_throttle_sec) {
      return;
    }

    last_log_time_ = now;

    std::cout << "[UnderwaterMPC] "
              << "solve=" << std::fixed << std::setprecision(2) << result.solve_time_ms << "ms "
              << "iter=" << result.iterations << " "
              << "cost=" << std::scientific << std::setprecision(2) << result.cost << " ";

    if (reference.has_valid_target) {
      std::cout << "e_pos=[" << std::fixed << std::setprecision(3)
                << result.position_error.x() << ","
                << result.position_error.y() << ","
                << result.position_error.z() << "] "
                << "e_vel=[" << result.velocity_error(0) << ","
                << result.velocity_error(1) << ","
                << result.velocity_error(2) << "] ";
    }

    std::cout << "thrust=[" << std::fixed << std::setprecision(2)
              << result.control.segment_thrust(0) << ","
              << result.control.segment_thrust(1) << ","
              << result.control.segment_thrust(2) << ","
              << result.control.segment_thrust(3) << "] "
              << "torque=[" << result.control.joint_torque(0) << ","
              << result.control.joint_torque(1) << ",...]\n";
  }

  void updateParameters(const UnderwaterMpcParameters & params)
  {
    params_ = params;

    // Update cost weights
    GrampcCostWeights weights = buildCostWeights(params);
    dynamics_->setCostWeights(weights);

    // Reconfigure solver
    configureSolver();

    // Update control limits
    initializeControlLimits();
  }

  const UnderwaterSimulator & simulator() const
  {
    return dynamics_->simulator();
  }

  bool isValid() const
  {
    return dynamics_ && solver_ && dynamics_->isValid();
  }

  const std::string & error() const
  {
    return error_;
  }

  const UnderwaterMpcParameters & parameters() const
  {
    return params_;
  }

  const UnderwaterMpcResult & lastResult() const
  {
    return last_result_;
  }

private:
  UnderwaterMpcParameters params_;
  std::shared_ptr<GrampcUnderwaterDynamics> dynamics_;
  std::unique_ptr<grampc::Grampc> solver_;

  std::vector<typeRNum> x0_;
  std::vector<typeRNum> xdes_;
  std::vector<typeRNum> u0_;
  std::vector<typeRNum> udes_;
  std::vector<typeRNum> umin_;
  std::vector<typeRNum> umax_;

  UnderwaterMpcResult last_result_;
  std::string error_;

  std::chrono::steady_clock::time_point last_log_time_;
};

// Public interface implementation

UnderwaterMpcController::UnderwaterMpcController(
  const UnderwaterSimulatorParameters & sim_params,
  const UnderwaterMpcParameters & mpc_params)
: impl_(std::make_unique<Impl>(sim_params, mpc_params))
{
}

UnderwaterMpcController::~UnderwaterMpcController() = default;

bool UnderwaterMpcController::isValid() const
{
  return impl_->isValid();
}

const std::string & UnderwaterMpcController::error() const
{
  return impl_->error();
}

UnderwaterMpcResult UnderwaterMpcController::computeControl(
  const ReducedConfiguration & current_config,
  const ReducedVelocity & current_velocity,
  const MpcReferenceTrajectory & reference,
  const double current_time)
{
  return impl_->computeControl(current_config, current_velocity, reference, current_time);
}

void UnderwaterMpcController::updateParameters(const UnderwaterMpcParameters & params)
{
  impl_->updateParameters(params);
}

const UnderwaterMpcParameters & UnderwaterMpcController::parameters() const
{
  return impl_->parameters();
}

const UnderwaterSimulator & UnderwaterMpcController::simulator() const
{
  return impl_->simulator();
}

const UnderwaterMpcResult & UnderwaterMpcController::lastResult() const
{
  return impl_->lastResult();
}

}  // namespace asr_sdm_kinematic_dynamic_model

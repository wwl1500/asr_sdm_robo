// Copyright (c) 2025.
// GRAMPC dynamics bridge implementation.

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"

#include <cmath>
#include <sstream>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

template<typename Derived>
std::string sizeMismatch(
  const char * callback, const char * value, const Derived & actual,
  int expected)
{
  std::ostringstream stream;
  stream << callback << ": " << value << " size must be " << expected << ", got " << actual.size();
  return stream.str();
}

}  // namespace

GrampcUnderwaterDynamics::GrampcUnderwaterDynamics(
  const UnderwaterSimulatorParameters & sim_params,
  const GrampcDimensionConfig & dim_config,
  const GrampcCostWeights & cost_weights,
  std::optional<ReducedConfiguration> initial_configuration)
: ProblemDescription(
    dim_config.nx, dim_config.nu, dim_config.np, dim_config.ng, dim_config.nh,
    dim_config.ng_terminal, dim_config.nh_terminal),
  dim_config_(dim_config),
  cost_weights_(cost_weights)
{
  if (!dim_config_.isValid()) {
    error_ = "GrampcDimensionConfig only supports nx=24, nu=10, np=ng=nh=ng_terminal=nh_terminal=0";
    return;
  }
  if (!cost_weights_.isValid()) {
    error_ = "GrampcCostWeights contains NaN, infinity, or a negative entry";
    return;
  }

  // P0-2: Reject RotorVelocity mode - MPC only supports AggregateThrust
  if (sim_params.actuators.command_mode == ActuatorCommandMode::RotorVelocity) {
    error_ = "RotorVelocity mode is not supported by this MPC interface; "
      "use ActuatorCommandMode::AggregateThrust";
    return;
  }

  // P1-1: Reject thrust lag - MPC assumes lag-free actuators
  if (sim_params.actuators.thrust_time_constant > 0.0) {
    error_ = "Non-zero thrust_time_constant is not supported by this MPC interface; "
      "the bridge assumes lag-free actuators (thrust_time_constant must be 0)";
    return;
  }

  simulator_ = std::make_unique<UnderwaterSimulator>(sim_params);
  if (!simulator_->isValid()) {
    error_ = "Failed to initialize underwater simulator: " + simulator_->error();
    simulator_.reset();
    return;
  }

  reference_configuration_ = initial_configuration.has_value() ? *initial_configuration :
    simulator_->pinocchioModel().configuration();
  if (!isValidConfiguration(reference_configuration_)) {
    error_ =
      "Initial MPC reference configuration must be finite and contain a normalized quaternion";
    simulator_.reset();
  }
}

bool GrampcUnderwaterDynamics::isValidConfiguration(const ReducedConfiguration & configuration)
{
  const double quaternion_norm = configuration.segment<4>(3).norm();
  return configuration.array().isFinite().all() && std::isfinite(quaternion_norm) &&
         quaternion_norm >= 1.0e-12 && std::abs(quaternion_norm - 1.0) <= 1.0e-5;
}

void GrampcUnderwaterDynamics::validateStateAndControl(
  const grampc::VectorConstRef & x, const grampc::VectorConstRef & u,
  const char * callback) const
{
  if (!simulator_) {
    throw std::runtime_error(std::string(callback) + ": simulator is not initialized");
  }
  if (x.size() != kMpcStateDim) {
    throw std::invalid_argument(sizeMismatch(callback, "x", x, kMpcStateDim));
  }
  if (u.size() != kMpcControlDim) {
    throw std::invalid_argument(sizeMismatch(callback, "u", u, kMpcControlDim));
  }
  if (!x.allFinite()) {
    std::ostringstream stream;
    stream << callback << ": x must be finite; first non-finite index=";
    for (Eigen::Index index = 0; index < x.size(); ++index) {
      if (!std::isfinite(x(index))) {
        stream << index << " (" << x(index) << ')';
        break;
      }
    }
    throw std::invalid_argument(stream.str());
  }
  if (!u.allFinite()) {
    throw std::invalid_argument(std::string(callback) + ": u must be finite");
  }
}

ReducedConfiguration GrampcUnderwaterDynamics::configurationFromState(const MpcState & state) const
{
  const ReducedVelocity delta_configuration = state.head<kMpcConfigurationTangentDim>();
  return simulator_->pinocchioModel().integrate(reference_configuration_, delta_configuration);
}

ReducedAcceleration GrampcUnderwaterDynamics::computeAcceleration(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity,
  const SegmentThrustVector & segment_thrust, const JointTorqueVector & joint_torque) const
{
  UnderwaterSimulatorState state;
  state.configuration = configuration;
  state.velocity = velocity;

  UnderwaterSimulatorInput input;
  input.segment_thrust = segment_thrust;
  input.joint_torque = joint_torque;
  // P1-2: Use frozen fluid current for MPC prediction
  input.fluid_current_world = fluid_current_;
  input.fluid_current_acceleration_world = fluid_current_acceleration_;
  return simulator_->evaluate(state, input).acceleration;
}

MpcState GrampcUnderwaterDynamics::computeStateDerivative(
  const MpcState & state, const Eigen::Matrix<double, kMpcControlDim, 1> & control) const
{
  const ReducedConfiguration configuration = configurationFromState(state);
  const ReducedVelocity velocity = state.tail<kReducedNv>();
  const SegmentThrustVector segment_thrust = control.head<kNumLinks>();
  const JointTorqueVector joint_torque = control.tail<kNumJointDofs>();

  MpcState derivative;
  derivative.head<kMpcConfigurationTangentDim>() =
    simulator_->pinocchioModel().configurationTangentDerivative(
      reference_configuration_, state.head<kMpcConfigurationTangentDim>(), velocity);
  derivative.tail<kReducedNv>() =
    computeAcceleration(configuration, velocity, segment_thrust, joint_torque);
  return derivative;
}

MpcStateJacobian GrampcUnderwaterDynamics::computeJacobianX(
  const MpcState & state, const Eigen::Matrix<double, kMpcControlDim, 1> & control) const
{
  MpcStateJacobian jacobian;
  for (int i = 0; i < kMpcStateDim; ++i) {
    MpcState plus = state;
    MpcState minus = state;
    plus(i) += kJacobianEpsilon;
    minus(i) -= kJacobianEpsilon;
    jacobian.col(i) =
      (computeStateDerivative(plus, control) - computeStateDerivative(minus, control)) /
      (2.0 * kJacobianEpsilon);
  }
  return jacobian;
}

MpcControlJacobian GrampcUnderwaterDynamics::computeJacobianU(
  const MpcState & state, const Eigen::Matrix<double, kMpcControlDim, 1> & control) const
{
  MpcControlJacobian jacobian;
  for (int i = 0; i < kMpcControlDim; ++i) {
    Eigen::Matrix<double, kMpcControlDim, 1> plus = control;
    Eigen::Matrix<double, kMpcControlDim, 1> minus = control;
    plus(i) += kJacobianEpsilon;
    minus(i) -= kJacobianEpsilon;
    jacobian.col(i) =
      (computeStateDerivative(state, plus) - computeStateDerivative(state, minus)) /
      (2.0 * kJacobianEpsilon);
  }
  return jacobian;
}

void GrampcUnderwaterDynamics::ffct(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef u, grampc::VectorConstRef, const grampc::GrampcParam &)
{
  validateStateAndControl(x, u, "ffct");
  if (out.size() != kMpcStateDim) {
    throw std::invalid_argument(sizeMismatch("ffct", "out", out, kMpcStateDim));
  }

  const MpcState state = Eigen::Map<const MpcState>(x.data());
  const Eigen::Matrix<double, kMpcControlDim, 1> control =
    Eigen::Map<const Eigen::Matrix<double, kMpcControlDim, 1>>(u.data());
  Eigen::Map<MpcState>(out.data()) = computeStateDerivative(state, control);
}

void GrampcUnderwaterDynamics::dfdx_vec(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef u, grampc::VectorConstRef, grampc::VectorConstRef vec,
  const grampc::GrampcParam &)
{
  validateStateAndControl(x, u, "dfdx_vec");
  if (vec.size() != kMpcStateDim) {
    throw std::invalid_argument(sizeMismatch("dfdx_vec", "vec", vec, kMpcStateDim));
  }
  if (out.size() != kMpcStateDim) {
    throw std::invalid_argument(sizeMismatch("dfdx_vec", "out", out, kMpcStateDim));
  }

  const MpcState state = Eigen::Map<const MpcState>(x.data());
  const Eigen::Matrix<double, kMpcControlDim, 1> control =
    Eigen::Map<const Eigen::Matrix<double, kMpcControlDim, 1>>(u.data());
  Eigen::Map<MpcState>(out.data()) = computeJacobianX(state, control).transpose() * vec;
}

void GrampcUnderwaterDynamics::dfdu_vec(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef u, grampc::VectorConstRef, grampc::VectorConstRef vec,
  const grampc::GrampcParam &)
{
  validateStateAndControl(x, u, "dfdu_vec");
  if (vec.size() != kMpcStateDim) {
    throw std::invalid_argument(sizeMismatch("dfdu_vec", "vec", vec, kMpcStateDim));
  }
  if (out.size() != kMpcControlDim) {
    throw std::invalid_argument(sizeMismatch("dfdu_vec", "out", out, kMpcControlDim));
  }

  const MpcState state = Eigen::Map<const MpcState>(x.data());
  const Eigen::Matrix<double, kMpcControlDim, 1> control =
    Eigen::Map<const Eigen::Matrix<double, kMpcControlDim, 1>>(u.data());
  Eigen::Map<Eigen::Matrix<double, kMpcControlDim, 1>>(out.data()) =
    computeJacobianU(state, control).transpose() * vec;
}

void GrampcUnderwaterDynamics::dfdp_vec(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef, grampc::VectorConstRef,
  grampc::VectorConstRef p, grampc::VectorConstRef, const grampc::GrampcParam &)
{
  if (p.size() != 0 || out.size() != 0) {
    throw std::invalid_argument("dfdp_vec: this problem has no parameters");
  }
}

void GrampcUnderwaterDynamics::lfct(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef u, grampc::VectorConstRef, const grampc::GrampcParam & param)
{
  validateStateAndControl(x, u, "lfct");
  if (out.size() != 1) {
    throw std::invalid_argument(sizeMismatch("lfct", "out", out, 1));
  }
  // P1-3: Use param.xdes as the single source of truth for desired state
  // If xdes is not bound (default constructor), use zero vector
  MpcState xdes = MpcState::Zero();
  if (param.xdes.size() == kMpcStateDim) {
    xdes = Eigen::Map<const MpcState>(param.xdes.data());
  }
  const MpcState error = Eigen::Map<const MpcState>(x.data()) - xdes;
  const Eigen::Matrix<double, kMpcControlDim, 1> control =
    Eigen::Map<const Eigen::Matrix<double, kMpcControlDim, 1>>(u.data());
  out(0) = 0.5 * (cost_weights_.state_weight.array() * error.array().square()).sum() +
    0.5 * (cost_weights_.control_weight.array() * control.array().square()).sum();
}

void GrampcUnderwaterDynamics::dldx(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef u, grampc::VectorConstRef, const grampc::GrampcParam & param)
{
  validateStateAndControl(x, u, "dldx");
  if (out.size() != kMpcStateDim) {
    throw std::invalid_argument(sizeMismatch("dldx", "out", out, kMpcStateDim));
  }
  // P1-3: Use param.xdes as the single source of truth for desired state
  MpcState xdes = MpcState::Zero();
  if (param.xdes.size() == kMpcStateDim) {
    xdes = Eigen::Map<const MpcState>(param.xdes.data());
  }
  Eigen::Map<MpcState>(out.data()) = cost_weights_.state_weight.array() *
    (Eigen::Map<const MpcState>(x.data()) - xdes).array();
}

void GrampcUnderwaterDynamics::dldu(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef u, grampc::VectorConstRef, const grampc::GrampcParam &)
{
  validateStateAndControl(x, u, "dldu");
  if (out.size() != kMpcControlDim) {
    throw std::invalid_argument(sizeMismatch("dldu", "out", out, kMpcControlDim));
  }
  Eigen::Map<Eigen::Matrix<double, kMpcControlDim, 1>>(out.data()) =
    cost_weights_.control_weight.array() *
    Eigen::Map<const Eigen::Matrix<double, kMpcControlDim, 1>>(u.data()).array();
}

void GrampcUnderwaterDynamics::Vfct(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef p, const grampc::GrampcParam & param)
{
  if (x.size() != kMpcStateDim || p.size() != 0 || out.size() != 1) {
    throw std::invalid_argument("Vfct: invalid state, parameter, or output dimension");
  }
  // P1-3: Use param.xdes as the single source of truth for desired state
  MpcState xdes = MpcState::Zero();
  if (param.xdes.size() == kMpcStateDim) {
    xdes = Eigen::Map<const MpcState>(param.xdes.data());
  }
  const MpcState error = Eigen::Map<const MpcState>(x.data()) - xdes;
  out(0) = 0.5 * (cost_weights_.terminal_weight.array() * error.array().square()).sum();
}

void GrampcUnderwaterDynamics::dVdx(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef x,
  grampc::VectorConstRef p, const grampc::GrampcParam & param)
{
  if (x.size() != kMpcStateDim || p.size() != 0 || out.size() != kMpcStateDim) {
    throw std::invalid_argument("dVdx: invalid state, parameter, or output dimension");
  }
  // P1-3: Use param.xdes as the single source of truth for desired state
  MpcState xdes = MpcState::Zero();
  if (param.xdes.size() == kMpcStateDim) {
    xdes = Eigen::Map<const MpcState>(param.xdes.data());
  }
  Eigen::Map<MpcState>(out.data()) = cost_weights_.terminal_weight.array() *
    (Eigen::Map<const MpcState>(x.data()) - xdes).array();
}

void GrampcUnderwaterDynamics::dVdT(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef, grampc::VectorConstRef,
  const grampc::GrampcParam &)
{
  if (out.size() != 1) {
    throw std::invalid_argument(sizeMismatch("dVdT", "out", out, 1));
  }
  out(0) = 0.0;
}

void GrampcUnderwaterDynamics::hfct(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef, grampc::VectorConstRef,
  grampc::VectorConstRef, const grampc::GrampcParam &)
{
  if (out.size() != 0) {
    throw std::invalid_argument("hfct: this problem has no inequality constraints");
  }
}

void GrampcUnderwaterDynamics::dhdx_vec(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef, grampc::VectorConstRef,
  grampc::VectorConstRef, grampc::VectorConstRef, const grampc::GrampcParam &)
{
  if (out.size() != 0) {
    throw std::invalid_argument("dhdx_vec: this problem has no inequality constraints");
  }
}

void GrampcUnderwaterDynamics::dhdu_vec(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef, grampc::VectorConstRef,
  grampc::VectorConstRef, grampc::VectorConstRef, const grampc::GrampcParam &)
{
  if (out.size() != 0) {
    throw std::invalid_argument("dhdu_vec: this problem has no inequality constraints");
  }
}

void GrampcUnderwaterDynamics::dhdp_vec(
  grampc::VectorRef out, ctypeRNum, grampc::VectorConstRef, grampc::VectorConstRef,
  grampc::VectorConstRef, grampc::VectorConstRef, const grampc::GrampcParam &)
{
  if (out.size() != 0) {
    throw std::invalid_argument("dhdp_vec: this problem has no inequality constraints");
  }
}

void GrampcUnderwaterDynamics::updateReferenceConfiguration(
  const ReducedConfiguration & configuration)
{
  if (!simulator_) {
    throw std::runtime_error(
        "Cannot update reference configuration before simulator initialization");
  }
  if (!isValidConfiguration(configuration)) {
    throw std::invalid_argument(
      "MPC reference configuration must be finite and contain a normalized quaternion");
  }
  reference_configuration_ = configuration;
}

void GrampcUnderwaterDynamics::setCostWeights(const GrampcCostWeights & weights)
{
  if (!weights.isValid()) {
    throw std::invalid_argument("GrampcCostWeights contains NaN, infinity, or a negative entry");
  }
  cost_weights_ = weights;
}

void GrampcUnderwaterDynamics::updateFluidCurrent(
  const Eigen::Vector3d & current_world,
  const Eigen::Vector3d & current_acceleration_world)
{
  if (!current_world.array().isFinite().all()) {
    throw std::invalid_argument("Fluid current must be finite");
  }
  if (!current_acceleration_world.array().isFinite().all()) {
    throw std::invalid_argument("Fluid current acceleration must be finite");
  }
  fluid_current_ = current_world;
  fluid_current_acceleration_ = current_acceleration_world;
}

}  // namespace asr_sdm_kinematic_dynamic_model

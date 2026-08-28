#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace asr_sdm_kinematic_dynamic_model
{

UnderwaterSimulator::UnderwaterSimulator(const UnderwaterSimulatorParameters & params)
: pinocchio_model_(params.pinocchio),
  fluid_model_(params.hydrodynamics),
  actuator_model_(params.actuators)
{
  if (!pinocchio_model_.isValid()) {
    error_ = "Invalid Pinocchio model: " + pinocchio_model_.error();
  } else if (!fluid_model_.isValid()) {
    error_ = "Invalid hydrodynamic model: " + fluid_model_.error();
  } else if (!actuator_model_.isValid()) {
    error_ = "Invalid actuator model: " + actuator_model_.error();
  }
}

bool UnderwaterSimulator::isValid() const
{
  return error_.empty();
}

const std::string & UnderwaterSimulator::error() const
{
  return error_;
}

const PinocchioModel & UnderwaterSimulator::pinocchioModel() const
{
  return pinocchio_model_;
}

UnderwaterSimulatorState UnderwaterSimulator::makeInitialState() const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot create state for an invalid simulator: " + error_);
  }
  UnderwaterSimulatorState state;
  state.configuration = pinocchio_model_.neutralConfiguration();
  state.velocity = pinocchio_model_.zeroVelocity();
  return state;
}

UnderwaterDynamicsOutput UnderwaterSimulator::evaluate(
  const UnderwaterSimulatorState & state,
  const UnderwaterSimulatorInput & input) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid simulator: " + error_);
  }
  if (!std::isfinite(state.time) || !state.configuration.array().isFinite().all() ||
    !state.velocity.array().isFinite().all())
  {
    throw std::invalid_argument("Simulator state must contain only finite values");
  }
  if (!input.fluid_current_world.array().isFinite().all() ||
    !input.segment_thrust.array().isFinite().all() ||
    !input.joint_torque.array().isFinite().all())
  {
    throw std::invalid_argument("Simulator input must contain only finite values");
  }
  const double quaternion_norm = state.configuration.segment<4>(3).norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-12 ||
    std::abs(quaternion_norm - 1.0) > 1.0e-5)
  {
    throw std::invalid_argument("Simulator quaternion is not normalized");
  }

  UnderwaterDynamicsOutput output;
  output.kinematics = pinocchio_model_.computeKinematics(state.configuration, state.velocity);
  const auto rigid = pinocchio_model_.computeDynamics(state.configuration, state.velocity);
  output.hydrodynamics = fluid_model_.evaluatePinocchio(
    output.kinematics, state.velocity, ReducedAcceleration::Zero(), input.fluid_current_world);
  output.actuators = actuator_model_.evaluate(
    output.kinematics, input.segment_thrust, input.joint_torque);

  output.effective_mass = rigid.mass_matrix + output.hydrodynamics.added_mass_matrix;
  output.effective_mass = 0.5 * (output.effective_mass + output.effective_mass.transpose());
  output.rigid_nonlinear_effects = rigid.nonlinear_effects;
  output.actuator_force = output.actuators.generalized_force;
  output.added_mass_bias_force = output.hydrodynamics.added_mass_bias_force;
  output.damping_force = output.hydrodynamics.damping_force;
  output.buoyancy_force = output.hydrodynamics.buoyancy_force;
  output.right_hand_side = output.actuator_force + output.damping_force +
    output.buoyancy_force - output.rigid_nonlinear_effects - output.added_mass_bias_force;

  const Eigen::LDLT<Eigen::Matrix<double, kReducedNv, kReducedNv>> solver(
    output.effective_mass);
  if (solver.info() != Eigen::Success ||
    !solver.vectorD().array().isFinite().all() ||
    (solver.vectorD().array() <= 0.0).any())
  {
    throw std::runtime_error("Effective simulator mass matrix is not positive definite");
  }
  output.acceleration = solver.solve(output.right_hand_side);
  output.balance_residual = output.effective_mass * output.acceleration - output.right_hand_side;
  if (!output.acceleration.array().isFinite().all() ||
    !output.balance_residual.array().isFinite().all())
  {
    throw std::runtime_error("Simulator produced a non-finite acceleration");
  }
  return output;
}

UnderwaterDynamicsOutput UnderwaterSimulator::step(
  UnderwaterSimulatorState & state,
  double dt,
  const UnderwaterSimulatorInput & input) const
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Simulator step must be finite and positive");
  }
  const UnderwaterDynamicsOutput output = evaluate(state, input);
  const ReducedVelocity next_velocity = state.velocity + dt * output.acceleration;
  const ReducedConfiguration next_configuration = pinocchio_model_.integrate(
    state.configuration, dt * next_velocity);
  if (!next_configuration.array().isFinite().all() || !next_velocity.array().isFinite().all()) {
    throw std::runtime_error("Simulator integration produced a non-finite state");
  }
  state.velocity = next_velocity;
  state.configuration = next_configuration;
  state.time += dt;
  return output;
}

bool UnderwaterSimulator::replaceHydrodynamicParameters(
  const HydrodynamicModelParameters & parameters, std::string * error)
{
  FluidForceModel candidate(parameters);
  if (!candidate.isValid()) {
    if (error != nullptr) {
      *error = candidate.error();
    }
    return false;
  }
  fluid_model_ = std::move(candidate);
  return true;
}

const HydrodynamicModelParameters & UnderwaterSimulator::hydrodynamicParameters() const
{
  return fluid_model_.parameters();
}

}  // namespace asr_sdm_kinematic_dynamic_model

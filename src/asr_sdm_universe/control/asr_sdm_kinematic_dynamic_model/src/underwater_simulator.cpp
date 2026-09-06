#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

/// Gravity lives on UnderwaterSimulatorParameters, so both sub-models are forced onto
/// the same vector before they are constructed.
PinocchioModelParameters withGravity(const UnderwaterSimulatorParameters & params)
{
  PinocchioModelParameters output = params.pinocchio;
  output.gravity_world = params.gravity_world;
  return output;
}

HydrodynamicModelParameters withGravity(
  const HydrodynamicModelParameters & hydrodynamics, const Eigen::Vector3d & gravity_world)
{
  HydrodynamicModelParameters output = hydrodynamics;
  output.gravity_world = gravity_world;
  return output;
}

}  // namespace

UnderwaterSimulator::UnderwaterSimulator(const UnderwaterSimulatorParameters & params)
: gravity_world_(params.gravity_world),
  integration_method_(params.integration_method),
  pinocchio_model_(withGravity(params)),
  fluid_model_(withGravity(params.hydrodynamics, params.gravity_world)),
  actuator_model_(params.actuators),
  joint_model_(params.joints)
{
  if (!params.gravity_world.array().isFinite().all()) {
    error_ = "Gravity must contain only finite values";
  } else if (!pinocchio_model_.isValid()) {
    error_ = "Invalid Pinocchio model: " + pinocchio_model_.error();
  } else if (!fluid_model_.isValid()) {
    error_ = "Invalid hydrodynamic model: " + fluid_model_.error();
  } else if (!actuator_model_.isValid()) {
    error_ = "Invalid actuator model: " + actuator_model_.error();
  } else if (!joint_model_.isValid()) {
    error_ = "Invalid joint dynamics model: " + joint_model_.error();
  } else {
    joint_mapping_ = pinocchio_model_.controllerDofMapping();
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

const ActuatorModel & UnderwaterSimulator::actuatorModel() const
{
  return actuator_model_;
}

const JointDynamicsModel & UnderwaterSimulator::jointDynamicsModel() const
{
  return joint_model_;
}

IntegrationMethod UnderwaterSimulator::integrationMethod() const
{
  return integration_method_;
}

const Eigen::Vector3d & UnderwaterSimulator::gravityWorld() const
{
  return gravity_world_;
}

UnderwaterSimulatorState UnderwaterSimulator::makeInitialState() const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot create state for an invalid simulator: " + error_);
  }
  UnderwaterSimulatorState state;
  state.configuration = pinocchio_model_.neutralConfiguration();
  state.velocity = pinocchio_model_.zeroVelocity();
  state.rotor_thrust.setZero();
  return state;
}

JointTorqueVector UnderwaterSimulator::jointPosition(
  const UnderwaterSimulatorState & state) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot read joints from an invalid simulator: " + error_);
  }
  JointTorqueVector position = JointTorqueVector::Zero();
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    position(static_cast<Eigen::Index>(dof)) = state.configuration(joint_mapping_[dof].q_index);
  }
  return position;
}

JointTorqueVector UnderwaterSimulator::jointVelocity(
  const UnderwaterSimulatorState & state) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot read joints from an invalid simulator: " + error_);
  }
  JointTorqueVector velocity = JointTorqueVector::Zero();
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    velocity(static_cast<Eigen::Index>(dof)) = state.velocity(joint_mapping_[dof].v_index);
  }
  return velocity;
}

UnderwaterDynamicsOutput UnderwaterSimulator::evaluate(
  const UnderwaterSimulatorState & state,
  const UnderwaterSimulatorInput & input) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot evaluate an invalid simulator: " + error_);
  }
  if (!std::isfinite(state.time) || !state.configuration.array().isFinite().all() ||
    !state.velocity.array().isFinite().all() || !state.rotor_thrust.array().isFinite().all())
  {
    throw std::invalid_argument("Simulator state must contain only finite values");
  }
  if (!input.fluid_current_world.array().isFinite().all() ||
    !input.fluid_current_acceleration_world.array().isFinite().all() ||
    !input.segment_thrust.array().isFinite().all() ||
    !input.rotor_rate.array().isFinite().all() ||
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
    output.kinematics, state.velocity, ReducedAcceleration::Zero(), input.fluid_current_world,
    input.fluid_current_acceleration_world);

  output.commanded_rotor_thrust =
    actuator_model_.commandedRotorThrust(input.segment_thrust, input.rotor_rate);
  // A lag-free actuator applies its command within the same step, so the stored
  // thrust is bypassed entirely and the default configuration stays memoryless.
  const RotorVector applied_rotor_thrust =
    actuator_model_.parameters().thrust_time_constant > 0.0 ?
    state.rotor_thrust : output.commanded_rotor_thrust;
  output.actuators = actuator_model_.evaluate(
    output.kinematics, applied_rotor_thrust, input.joint_torque);
  output.joints = joint_model_.evaluate(jointPosition(state), jointVelocity(state));
  for (std::size_t dof = 0; dof < kNumJointDofs; ++dof) {
    output.joint_passive_force(joint_mapping_[dof].v_index) =
      output.joints.total_torque(static_cast<Eigen::Index>(dof));
  }

  output.effective_mass = rigid.mass_matrix + output.hydrodynamics.added_mass_matrix;
  output.effective_mass = 0.5 * (output.effective_mass + output.effective_mass.transpose());
  output.rigid_nonlinear_effects = rigid.nonlinear_effects;
  output.rigid_coriolis_force = rigid.coriolis_force;
  output.rigid_gravity_force = rigid.gravity;
  output.actuator_force = output.actuators.generalized_force;
  output.added_mass_bias_force = output.hydrodynamics.added_mass_bias_force;
  output.damping_force = output.hydrodynamics.damping_force;
  output.buoyancy_force = output.hydrodynamics.buoyancy_force;
  output.froude_krylov_force = output.hydrodynamics.froude_krylov_force;
  output.right_hand_side = output.actuator_force + output.damping_force +
    output.buoyancy_force + output.froude_krylov_force + output.joint_passive_force -
    output.rigid_nonlinear_effects - output.added_mass_bias_force;

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

ReducedAcceleration UnderwaterSimulator::accelerationOnly(
  const UnderwaterSimulatorState & state, const UnderwaterSimulatorInput & input) const
{
  return evaluate(state, input).acceleration;
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

  ReducedVelocity next_velocity = ReducedVelocity::Zero();
  ReducedConfiguration next_configuration = ReducedConfiguration::Zero();

  // Both integrators are explicit and this model is stiff: the articulation rows of
  // the mass matrix are three orders of magnitude lighter than the base rows. Once a
  // step leaves the stability region the state grows without bound, and the quaternion
  // exponential of an absurd increment silently stops being a unit quaternion and
  // trips an assertion deep inside Pinocchio. Bounding the increment turns that abort
  // into an error the caller can act on.
  const auto guard = [](const ReducedVelocity & increment) -> const ReducedVelocity & {
      constexpr double kMaxIncrementNorm = 1.0e3;
      if (!increment.array().isFinite().all() || increment.norm() > kMaxIncrementNorm) {
        throw std::runtime_error(
          "Simulator integration diverged; reduce dt, add joint damping, or select "
          "IntegrationMethod::SemiImplicitEuler");
      }
      return increment;
    };

  if (integration_method_ == IntegrationMethod::SemiImplicitEuler) {
    next_velocity = state.velocity + dt * output.acceleration;
    const ReducedVelocity increment = dt * next_velocity;
    next_configuration = pinocchio_model_.integrate(state.configuration, guard(increment));
  } else {
    // RK4 in the tangent space at the start of the step. The configuration stages are
    // reached with pinocchio::integrate so the free-flyer quaternion stays on the
    // manifold instead of being renormalized after the fact.
    const auto stage = [&](const ReducedVelocity & velocity_offset,
      const ReducedVelocity & configuration_offset) {
        UnderwaterSimulatorState probe = state;
        probe.configuration =
          pinocchio_model_.integrate(state.configuration, guard(configuration_offset));
        probe.velocity = state.velocity + guard(velocity_offset);
        return accelerationOnly(probe, input);
      };

    const ReducedVelocity k1_position = state.velocity;
    const ReducedAcceleration k1_velocity = output.acceleration;

    const ReducedVelocity k2_position = state.velocity + (0.5 * dt) * k1_velocity;
    const ReducedAcceleration k2_velocity =
      stage((0.5 * dt) * k1_velocity, (0.5 * dt) * k1_position);

    const ReducedVelocity k3_position = state.velocity + (0.5 * dt) * k2_velocity;
    const ReducedAcceleration k3_velocity =
      stage((0.5 * dt) * k2_velocity, (0.5 * dt) * k2_position);

    const ReducedVelocity k4_position = state.velocity + dt * k3_velocity;
    const ReducedAcceleration k4_velocity = stage(dt * k3_velocity, dt * k3_position);

    next_velocity = state.velocity +
      (dt / 6.0) * (k1_velocity + 2.0 * k2_velocity + 2.0 * k3_velocity + k4_velocity);
    const ReducedVelocity increment =
      (dt / 6.0) * (k1_position + 2.0 * k2_position + 2.0 * k3_position + k4_position);
    next_configuration = pinocchio_model_.integrate(state.configuration, guard(increment));
  }

  const RotorVector next_rotor_thrust = actuator_model_.advanceRotorThrust(
    state.rotor_thrust, output.commanded_rotor_thrust, dt);

  if (!next_configuration.array().isFinite().all() || !next_velocity.array().isFinite().all() ||
    !next_rotor_thrust.array().isFinite().all())
  {
    throw std::runtime_error("Simulator integration produced a non-finite state");
  }
  state.velocity = next_velocity;
  state.configuration = next_configuration;
  state.rotor_thrust = next_rotor_thrust;
  state.time += dt;
  return output;
}

bool UnderwaterSimulator::replaceHydrodynamicParameters(
  const HydrodynamicModelParameters & parameters, std::string * error)
{
  FluidForceModel candidate(withGravity(parameters, gravity_world_));
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

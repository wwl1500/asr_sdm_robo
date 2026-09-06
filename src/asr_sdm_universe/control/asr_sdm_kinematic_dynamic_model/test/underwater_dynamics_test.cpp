#include "asr_sdm_kinematic_dynamic_model/joint_dynamics_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <string>

namespace
{
namespace model = asr_sdm_kinematic_dynamic_model;

bool check(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

model::UnderwaterSimulatorParameters makeParameters(const char * urdf_path)
{
  model::UnderwaterSimulatorParameters params;
  params.pinocchio.urdf_path = urdf_path;
  params.pinocchio.use_free_flyer = true;
  params.pinocchio.lock_rotor_joints = true;
  for (auto & link : params.hydrodynamics.links) {
    link.linear_damping.setConstant(0.1);
    link.quadratic_damping.setConstant(0.01);
    link.displaced_volume = 5.81e-4;
  }
  params.actuators.maximum_segment_thrust.setConstant(1.0);
  return params;
}

/// A state that exercises every coupling: off-axis attitude, base twist, and
/// articulation rates.
model::UnderwaterSimulatorState makeExcitedState(const model::UnderwaterSimulator & simulator)
{
  auto state = simulator.makeInitialState();
  const Eigen::Quaterniond attitude =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.3, -0.7, 0.5).normalized()));
  state.configuration(3) = attitude.x();
  state.configuration(4) = attitude.y();
  state.configuration(5) = attitude.z();
  state.configuration(6) = attitude.w();
  for (int i = 0; i < model::kReducedNv; ++i) {
    state.velocity(i) = 0.15 * std::sin(0.7 * i + 0.3);
  }
  for (std::size_t dof = 0; dof < model::kNumJointDofs; ++dof) {
    state.configuration(7 + static_cast<Eigen::Index>(dof)) = 0.2 * std::cos(1.1 * dof);
  }
  return state;
}

bool checkRigidBodyIdentities(const model::UnderwaterSimulator & simulator)
{
  const auto & pinocchio = simulator.pinocchioModel();
  const auto state = makeExcitedState(simulator);
  const auto dynamics = pinocchio.computeDynamics(state.configuration, state.velocity);

  bool passed = true;

  // C(q, v) v + g(q) is exactly what RNEA reports as the nonlinear effects.
  const double coriolis_error =
    (dynamics.coriolis_force + dynamics.gravity - dynamics.nonlinear_effects).norm();
  passed &= check(coriolis_error < 1.0e-9, "Coriolis matrix matches RNEA nonlinear effects");

  // RNEA and ABA must invert each other.
  model::ReducedAcceleration acceleration;
  for (int i = 0; i < model::kReducedNv; ++i) {
    acceleration(i) = 0.3 * std::cos(0.9 * i);
  }
  const auto torque = pinocchio.inverseDynamics(
    state.configuration, state.velocity, acceleration);
  const auto recovered = pinocchio.forwardDynamics(
    state.configuration, state.velocity, torque);
  const double round_trip_error = (recovered - acceleration).norm();
  passed &= check(round_trip_error < 1.0e-8, "RNEA and ABA round trip");

  passed &= check(dynamics.total_mass > 0.0, "positive total mass");
  passed &= check(dynamics.kinetic_energy > 0.0, "positive kinetic energy at nonzero velocity");

  std::cout << "  rigid identities   : |Cv + g - nle| = " << coriolis_error <<
    ", |aba(rnea(a)) - a| = " << round_trip_error << std::endl;
  return passed;
}

/// The simulator solves (M_rb + M_added) a = rhs. With zero added mass this must agree
/// with Pinocchio's articulated-body algorithm driven by the same external forces,
/// which is an independent check on the whole right-hand-side assembly.
bool checkAgainstArticulatedBodyAlgorithm(const char * urdf_path)
{
  auto params = makeParameters(urdf_path);
  for (auto & link : params.hydrodynamics.links) {
    link.added_mass.setZero();
  }
  params.joints.viscous_damping.setConstant(0.05);
  params.joints.coulomb_friction.setConstant(0.01);

  const model::UnderwaterSimulator simulator(params);
  if (!check(simulator.isValid(), "zero-added-mass simulator construction")) {
    return false;
  }

  const auto state = makeExcitedState(simulator);
  model::UnderwaterSimulatorInput input;
  input.segment_thrust << 0.4, 0.1, 0.0, 0.2;
  input.joint_torque.setConstant(0.05);
  input.fluid_current_world = Eigen::Vector3d(0.2, -0.1, 0.05);

  const auto output = simulator.evaluate(state, input);
  const model::GeneralizedVector external = output.actuator_force + output.damping_force +
    output.buoyancy_force + output.froude_krylov_force + output.joint_passive_force;
  const auto aba_acceleration = simulator.pinocchioModel().forwardDynamics(
    state.configuration, state.velocity, external);

  const double error = (aba_acceleration - output.acceleration).norm();
  std::cout << "  aba cross-check    : |a_aba - a_sim| = " << error << std::endl;
  return check(error < 1.0e-8, "simulator acceleration matches ABA with zero added mass");
}

/// Validates that local_accelerations really carries J-dot * v, and that
/// relative_accelerations additionally accounts for the current rotating in the link
/// frame. Both are compared against finite differences of the link twists.
bool checkJacobianTimeVariation(const char * urdf_path)
{
  const auto params = makeParameters(urdf_path);
  const model::PinocchioModel pinocchio(params.pinocchio);
  const model::FluidForceModel fluid(params.hydrodynamics);
  if (!check(pinocchio.isValid() && fluid.isValid(), "J-dot fixture construction")) {
    return false;
  }

  const model::UnderwaterSimulator simulator(params);
  const auto state = makeExcitedState(simulator);
  const Eigen::Vector3d current(0.3, -0.2, 0.1);

  const auto kinematics = pinocchio.computeKinematics(state.configuration, state.velocity);
  const auto evaluation = fluid.evaluatePinocchio(
    kinematics, state.velocity, model::ReducedAcceleration::Zero(), current);

  // Hold the velocity fixed and advance only the configuration, so the numerical
  // derivative of J(q) v isolates J-dot v.
  const double h = 1.0e-7;
  const auto configuration_plus = pinocchio.integrate(state.configuration, h * state.velocity);
  const auto configuration_minus = pinocchio.integrate(state.configuration, -h * state.velocity);
  const auto kinematics_plus = pinocchio.computeKinematics(configuration_plus, state.velocity);
  const auto kinematics_minus = pinocchio.computeKinematics(configuration_minus, state.velocity);
  const auto evaluation_plus = fluid.evaluatePinocchio(
    kinematics_plus, state.velocity, model::ReducedAcceleration::Zero(), current);
  const auto evaluation_minus = fluid.evaluatePinocchio(
    kinematics_minus, state.velocity, model::ReducedAcceleration::Zero(), current);

  double absolute_error = 0.0;
  double relative_error = 0.0;
  double current_term = 0.0;
  for (std::size_t link = 0; link < model::kNumLinks; ++link) {
    const model::SpatialVector numerical_absolute =
      (kinematics_plus.segment_jacobians[link] * state.velocity -
      kinematics_minus.segment_jacobians[link] * state.velocity) / (2.0 * h);
    const model::SpatialVector numerical_relative =
      (evaluation_plus.relative_twists[link] - evaluation_minus.relative_twists[link]) /
      (2.0 * h);
    absolute_error = std::max(
      absolute_error, (numerical_absolute - evaluation.local_accelerations[link]).norm());
    relative_error = std::max(
      relative_error, (numerical_relative - evaluation.relative_accelerations[link]).norm());
    current_term = std::max(
      current_term,
      (evaluation.relative_accelerations[link] - evaluation.local_accelerations[link]).norm());
  }

  std::cout << "  jacobian time var  : |Jdot v - numerical| = " << absolute_error <<
    ", |a_rel - numerical| = " << relative_error <<
    ", current contribution = " << current_term << std::endl;

  bool passed = true;
  passed &= check(absolute_error < 1.0e-5, "local acceleration matches finite-difference J-dot v");
  passed &= check(
    relative_error < 1.0e-5, "relative acceleration matches finite-difference relative twist");
  // Without the rotating-current correction the two would be identical, so a nonzero
  // gap is what proves the term is actually applied.
  passed &= check(current_term > 1.0e-3, "rotating current contributes to relative acceleration");
  return passed;
}

bool checkGravityUnification(const char * urdf_path)
{
  bool passed = true;

  auto params = makeParameters(urdf_path);
  params.gravity_world.setZero();
  // A stale gravity on the sub-parameters must not survive; the simulator overrides it.
  params.hydrodynamics.gravity_world = Eigen::Vector3d(0.0, 0.0, -9.81);
  const model::UnderwaterSimulator weightless(params);
  passed &= check(weightless.isValid(), "weightless simulator construction");
  if (weightless.isValid()) {
    const auto state = weightless.makeInitialState();
    const auto output = weightless.evaluate(state, model::UnderwaterSimulatorInput{});
    passed &= check(
      output.rigid_gravity_force.norm() < 1.0e-12, "zero gravity removes rigid weight");
    passed &= check(
      output.buoyancy_force.norm() < 1.0e-12, "zero gravity removes buoyancy");
    passed &= check(
      weightless.hydrodynamicParameters().gravity_world.norm() < 1.0e-12,
      "hydrodynamic gravity follows the simulator gravity");
  }

  // Size the displaced volume so the buoyant weight equals the rigid weight, then the
  // total world-frame buoyancy must cancel the total weight. That only holds if both
  // sides are driven by the same gravity vector, which is the property under test.
  // The two do not cancel per DOF, because weight acts at each body centre of mass
  // while buoyancy acts at the segment frames.
  auto neutral_params = makeParameters(urdf_path);
  const model::UnderwaterSimulator probe(neutral_params);
  passed &= check(probe.isValid(), "probe simulator construction");
  if (probe.isValid()) {
    const auto state = probe.makeInitialState();
    const double total_mass =
      probe.pinocchioModel().computeDynamics(state.configuration, state.velocity).total_mass;
    const double volume_per_link =
      total_mass / (neutral_params.hydrodynamics.fluid_density * model::kNumLinks);
    for (auto & link : neutral_params.hydrodynamics.links) {
      link.displaced_volume = volume_per_link;
    }
    const model::UnderwaterSimulator neutral(neutral_params);
    passed &= check(neutral.isValid(), "neutrally buoyant simulator construction");
    if (neutral.isValid()) {
      const auto excited = makeExcitedState(neutral);
      const auto output = neutral.evaluate(excited, model::UnderwaterSimulatorInput{});
      Eigen::Vector3d buoyancy_world = Eigen::Vector3d::Zero();
      for (std::size_t link = 0; link < model::kNumLinks; ++link) {
        buoyancy_world += output.kinematics.segment_placements[link].rotation() *
          output.hydrodynamics.buoyancy_wrenches[link].head<3>();
      }
      const Eigen::Vector3d weight_world = total_mass * neutral.gravityWorld();
      const double residual = (buoyancy_world + weight_world).norm();
      std::cout << "  gravity unification: |total buoyancy + total weight| = " << residual <<
        " N" << std::endl;
      passed &= check(residual < 1.0e-9, "neutral buoyancy cancels the total weight");
    }
  }

  // Scaling gravity has to scale weight and buoyancy by the same factor.
  auto scaled_params = makeParameters(urdf_path);
  scaled_params.gravity_world *= 2.0;
  const model::UnderwaterSimulator base(makeParameters(urdf_path));
  const model::UnderwaterSimulator scaled(scaled_params);
  if (base.isValid() && scaled.isValid()) {
    const auto state = makeExcitedState(base);
    const model::UnderwaterSimulatorInput idle;
    const auto base_output = base.evaluate(state, idle);
    const auto scaled_output = scaled.evaluate(state, idle);
    passed &= check(
      (scaled_output.rigid_gravity_force - 2.0 * base_output.rigid_gravity_force).norm() < 1.0e-9,
      "doubling gravity doubles the rigid weight");
    passed &= check(
      (scaled_output.buoyancy_force - 2.0 * base_output.buoyancy_force).norm() < 1.0e-9,
      "doubling gravity doubles the buoyancy");
  }
  return passed;
}

bool checkDampingDissipates(const char * urdf_path)
{
  auto params = makeParameters(urdf_path);
  // Cross-coupled damping on top of the diagonal coefficients.
  model::SpatialMatrix coupling = model::SpatialMatrix::Zero();
  coupling(0, 2) = 0.05;
  coupling(2, 0) = 0.05;
  coupling(1, 5) = 0.02;
  coupling(5, 1) = 0.02;
  coupling.diagonal().setConstant(0.2);
  for (auto & link : params.hydrodynamics.links) {
    link.linear_damping_matrix = coupling;
    link.quadratic_damping_matrix = 0.5 * coupling;
  }

  const model::UnderwaterSimulator simulator(params);
  bool passed = check(simulator.isValid(), "cross-coupled damping accepted");
  if (!simulator.isValid()) {
    std::cerr << "  reason: " << simulator.error() << std::endl;
    return false;
  }

  std::mt19937 rng(20260904);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  double worst_power = -1.0;
  for (int trial = 0; trial < 200; ++trial) {
    auto state = makeExcitedState(simulator);
    for (int i = 0; i < model::kReducedNv; ++i) {
      state.velocity(i) = uniform(rng);
    }
    const auto output = simulator.evaluate(state, model::UnderwaterSimulatorInput{});
    worst_power = std::max(worst_power, state.velocity.dot(output.damping_force));
  }
  std::cout << "  damping            : worst v . f_damping = " << worst_power << " W" << std::endl;
  passed &= check(worst_power <= 1.0e-12, "cross-coupled damping never injects energy");

  // A damping matrix whose symmetric part is indefinite would pump energy in.
  auto bad_params = params;
  bad_params.hydrodynamics.links[0].linear_damping_matrix(0, 0) = -1.0;
  const model::UnderwaterSimulator bad(bad_params);
  passed &= check(!bad.isValid(), "non-dissipative damping matrix rejected");

  const model::SpatialVector morison =
    model::makeCylinderMorisonDamping(1000.0, 0.1, 0.25, 1.2, 0.3);
  passed &= check(
    (morison.array() >= 0.0).all() && morison(0) > 0.0 && morison(2) > 0.0 && morison(3) > 0.0,
    "Morison helper produces nonnegative coefficients");
  passed &= check(
    morison(0) > morison(2), "transverse Morison drag exceeds axial drag for a slender cylinder");
  return passed;
}

bool checkScrewReactionTorque(const char * urdf_path)
{
  auto params = makeParameters(urdf_path);
  const model::UnderwaterSimulator simulator(params);
  bool passed = check(simulator.isValid(), "screw simulator construction");
  if (!simulator.isValid()) {
    return false;
  }

  const auto state = makeExcitedState(simulator);
  const auto kinematics =
    simulator.pinocchioModel().computeKinematics(state.configuration, state.velocity);
  const auto & actuator = simulator.actuatorModel();

  model::SegmentThrustVector symmetric = model::SegmentThrustVector::Zero();
  symmetric(0) = 0.6;
  const auto symmetric_output = actuator.evaluate(kinematics, symmetric);
  const double paired_torque = symmetric_output.rotor_reaction_torque(0) +
    symmetric_output.rotor_reaction_torque(1);
  passed &= check(
    std::abs(symmetric_output.rotor_reaction_torque(0)) > 1.0e-6,
    "each screw produces a reaction torque");
  passed &= check(
    std::abs(paired_torque) < 1.0e-12,
    "counter-rotating screws cancel their reaction torque under symmetric thrust");

  // Differential thrust is what leaves a net torque behind.
  model::RotorVector differential = model::RotorVector::Zero();
  differential(0) = 0.4;
  const auto differential_output = actuator.evaluate(kinematics, differential);
  passed &= check(
    std::abs(
      differential_output.rotor_reaction_torque(0) +
      differential_output.rotor_reaction_torque(1)) > 1.0e-6,
    "differential thrust leaves a net reaction torque");

  // Aggregate thrust must still be split evenly and saturated as before.
  passed &= check(
    std::abs(symmetric_output.actual_segment_thrust(0) - 0.6) < 1.0e-12 &&
    std::abs(symmetric_output.rotor_thrust(0) - 0.3) < 1.0e-12,
    "aggregate thrust splits evenly across the screw pair");
  model::SegmentThrustVector saturated = model::SegmentThrustVector::Constant(5.0);
  passed &= check(
    std::abs(actuator.evaluate(kinematics, saturated).actual_segment_thrust(0) - 1.0) < 1.0e-12,
    "aggregate thrust saturates at the segment maximum");

  // Rotor-velocity mode maps rates through the thrust coefficients.
  auto velocity_params = params;
  velocity_params.actuators.command_mode = model::ActuatorCommandMode::RotorVelocity;
  velocity_params.actuators.thrust_quadratic.setConstant(1.0e-3);
  velocity_params.actuators.maximum_segment_thrust.setConstant(10.0);
  const model::ActuatorModel velocity_actuator(velocity_params.actuators);
  passed &= check(velocity_actuator.isValid(), "rotor-velocity actuator construction");
  if (velocity_actuator.isValid()) {
    model::RotorVector rate = model::RotorVector::Zero();
    rate(0) = 10.0;
    const auto resolved = velocity_actuator.commandedRotorThrust(
      model::SegmentThrustVector::Zero(), rate);
    passed &= check(
      std::abs(resolved(0) - 1.0e-3 * 100.0) < 1.0e-12,
      "quadratic rotor thrust map");
    passed &= check(std::abs(resolved(1)) < 1.0e-12, "idle screw produces no thrust");
  }
  return passed;
}

bool checkThrusterLag(const char * urdf_path)
{
  auto params = makeParameters(urdf_path);
  params.actuators.thrust_time_constant = 0.2;
  const model::UnderwaterSimulator simulator(params);
  bool passed = check(simulator.isValid(), "lagged simulator construction");
  if (!simulator.isValid()) {
    return false;
  }

  auto state = simulator.makeInitialState();
  model::UnderwaterSimulatorInput input;
  input.segment_thrust.setConstant(0.8);

  const auto first = simulator.evaluate(state, input);
  passed &= check(
    first.actuators.rotor_thrust.norm() < 1.0e-12,
    "lagged thruster starts from zero delivered thrust");
  passed &= check(
    first.commanded_rotor_thrust.norm() > 0.0, "lagged thruster still reports its command");

  const double dt = 0.001;
  double previous = 0.0;
  bool monotonic = true;
  const int steps = static_cast<int>(std::lround(params.actuators.thrust_time_constant / dt));
  for (int step = 0; step < steps; ++step) {
    simulator.step(state, dt, input);
    monotonic = monotonic && state.rotor_thrust(0) >= previous;
    previous = state.rotor_thrust(0);
  }
  const double target = 0.5 * 0.8;
  const double reached = state.rotor_thrust(0) / target;
  std::cout << "  thruster lag       : fraction of command after one tau = " << reached <<
    std::endl;
  passed &= check(monotonic, "lagged thrust rises monotonically");
  passed &= check(std::abs(reached - (1.0 - std::exp(-1.0))) < 5.0e-3,
    "lagged thrust reaches 1 - 1/e of its command after one time constant");

  // The default configuration has no lag and must stay memoryless.
  const model::UnderwaterSimulator instant(makeParameters(urdf_path));
  auto instant_state = instant.makeInitialState();
  const auto instant_output = instant.evaluate(instant_state, input);
  passed &= check(
    (instant_output.actuators.rotor_thrust - instant_output.commanded_rotor_thrust).norm() <
    1.0e-12,
    "lag-free thruster applies its command immediately");
  return passed;
}

bool checkJointDynamics()
{
  bool passed = true;

  const model::JointDynamicsModel ideal;
  passed &= check(ideal.isValid(), "default joint dynamics model is valid");
  passed &= check(
    ideal.evaluate(
      model::JointTorqueVector::Constant(0.5),
      model::JointTorqueVector::Constant(1.0)).total_torque.norm() < 1.0e-12,
    "unconfigured joint dynamics contributes nothing");

  model::JointDynamicsParameters params;
  params.viscous_damping.setConstant(0.2);
  params.coulomb_friction.setConstant(0.05);
  params.limit_stiffness = 50.0;
  params.limit_damping = 1.0;
  params.lower_limit.setConstant(-0.5);
  params.upper_limit.setConstant(0.5);
  const model::JointDynamicsModel joints(params);
  passed &= check(joints.isValid(), "configured joint dynamics model is valid");

  const auto moving = joints.evaluate(
    model::JointTorqueVector::Zero(), model::JointTorqueVector::Constant(1.0));
  passed &= check(
    (moving.viscous_torque.array() < 0.0).all() && (moving.friction_torque.array() < 0.0).all(),
    "viscous and Coulomb torques oppose positive motion");
  passed &= check(
    moving.limit_torque.norm() < 1.0e-12, "no limit torque strictly inside the travel range");

  // Friction has to be continuous through zero velocity, not a jump.
  const double tiny = 0.1 * params.friction_velocity_scale;
  const auto creeping = joints.evaluate(
    model::JointTorqueVector::Zero(), model::JointTorqueVector::Constant(tiny));
  passed &= check(
    std::abs(creeping.friction_torque(0)) < params.coulomb_friction(0),
    "Coulomb friction is regularized near zero velocity");

  const auto beyond_upper = joints.evaluate(
    model::JointTorqueVector::Constant(0.6), model::JointTorqueVector::Constant(0.1));
  passed &= check(
    (beyond_upper.limit_torque.array() < 0.0).all(),
    "limit torque pushes back below the upper stop");
  const auto beyond_lower = joints.evaluate(
    model::JointTorqueVector::Constant(-0.6), model::JointTorqueVector::Constant(-0.1));
  passed &= check(
    (beyond_lower.limit_torque.array() > 0.0).all(),
    "limit torque pushes back above the lower stop");

  // Escaping a stop must not be damped back into it.
  const auto escaping = joints.evaluate(
    model::JointTorqueVector::Constant(0.6), model::JointTorqueVector::Constant(-1.0));
  passed &= check(
    std::abs(escaping.limit_torque(0) + params.limit_stiffness * 0.1) < 1.0e-12,
    "limit damping only resists deeper penetration");

  model::JointDynamicsParameters invalid;
  invalid.viscous_damping(0) = -1.0;
  passed &= check(
    !model::JointDynamicsModel(invalid).isValid(), "negative joint damping rejected");
  return passed;
}

/// Drives the simulator with both integrators and measures each one against a
/// finely stepped RK4 reference.
bool checkIntegratorAccuracy(const char * urdf_path)
{
  auto euler_params = makeParameters(urdf_path);
  euler_params.integration_method = model::IntegrationMethod::SemiImplicitEuler;
  auto rk4_params = makeParameters(urdf_path);
  rk4_params.integration_method = model::IntegrationMethod::RungeKutta4;

  const model::UnderwaterSimulator euler(euler_params);
  const model::UnderwaterSimulator rk4(rk4_params);
  bool passed = check(euler.isValid() && rk4.isValid(), "integrator fixtures construct");
  if (!passed) {
    return false;
  }

  model::UnderwaterSimulatorInput input;
  input.segment_thrust << 0.5, 0.3, 0.0, 0.1;
  input.fluid_current_world = Eigen::Vector3d(0.1, 0.05, 0.0);

  // Both integrators are explicit and this model is stiff, so dt has to stay at or
  // below the 0.005 s the simulator is documented for.
  const double duration = 0.4;
  const double dt = 0.005;
  const int refinement = 16;

  const auto roll_out = [&](const model::UnderwaterSimulator & simulator, double step) {
      auto state = makeExcitedState(simulator);
      const int steps = static_cast<int>(std::lround(duration / step));
      for (int i = 0; i < steps; ++i) {
        simulator.step(state, step, input);
      }
      return state;
    };

  const auto reference = roll_out(rk4, dt / refinement);
  const auto euler_state = roll_out(euler, dt);
  const auto rk4_state = roll_out(rk4, dt);

  const auto error = [&reference](const model::UnderwaterSimulatorState & state) {
      return (state.configuration - reference.configuration).norm() +
             (state.velocity - reference.velocity).norm();
    };
  const double euler_error = error(euler_state);
  const double rk4_error = error(rk4_state);

  std::cout << "  integrators        : euler error = " << euler_error <<
    ", rk4 error = " << rk4_error << std::endl;

  passed &= check(rk4_error < euler_error, "RK4 beats semi-implicit Euler at the same dt");
  passed &= check(rk4_error < 0.05 * euler_error, "RK4 is substantially more accurate");
  passed &= check(
    std::abs(rk4_state.configuration.segment<4>(3).norm() - 1.0) < 1.0e-10,
    "RK4 keeps the free-flyer quaternion normalized");

  // Too large a step diverges. That has to surface as an exception, because an
  // unchecked increment reaches Pinocchio's quaternion exponential and aborts.
  for (const auto & simulator : {std::cref(euler), std::cref(rk4)}) {
    bool threw = false;
    auto state = makeExcitedState(simulator.get());
    try {
      for (int i = 0; i < 200; ++i) {
        simulator.get().step(state, 0.05, input);
      }
    } catch (const std::runtime_error &) {
      threw = true;
    }
    passed &= check(threw, "divergence at an oversized dt is reported, not aborted");
  }
  return passed;
}

}  // namespace

int main()
{
#ifndef ASR_SDM_GENERATED_URDF
  std::cerr << "FAILED: generated URDF path is not configured" << std::endl;
  return 1;
#else
  const char * urdf_path = ASR_SDM_GENERATED_URDF;
  bool passed = true;

  const model::UnderwaterSimulator simulator(makeParameters(urdf_path));
  if (!check(simulator.isValid(), "baseline simulator construction")) {
    std::cerr << "  reason: " << simulator.error() << std::endl;
    return 1;
  }

  passed &= checkRigidBodyIdentities(simulator);
  passed &= checkAgainstArticulatedBodyAlgorithm(urdf_path);
  passed &= checkJacobianTimeVariation(urdf_path);
  passed &= checkGravityUnification(urdf_path);
  passed &= checkDampingDissipates(urdf_path);
  passed &= checkScrewReactionTorque(urdf_path);
  passed &= checkThrusterLag(urdf_path);
  passed &= checkJointDynamics();
  passed &= checkIntegratorAccuracy(urdf_path);

  std::cout << (passed ? "Underwater dynamics regression passed" :
  "Underwater dynamics regression failed") << std::endl;
  return passed ? 0 : 1;
#endif
}

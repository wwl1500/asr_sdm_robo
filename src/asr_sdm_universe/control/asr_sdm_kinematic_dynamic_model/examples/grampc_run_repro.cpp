// Closed-loop GRAMPC + UnderwaterSimulator run.
//
// Runs a real closed-loop MPC over the full rigid-body simulator.
// Each iteration:
//   1. Updates the dynamics' tangent-space reference to the current measured
//      configuration.
//   2. Calls solver.run() with the current state as x0.
//   3. Applies the optimal control (segment_thrust, joint_torque) to the
//      simulator with sub-step integration.
//   4. Logs the simulated state vs the desired target.
//
// Goal: verify that Grampc::run() never produces NaN/Inf and that the
// closed-loop controller actually drives the robot towards the desired
// forward velocity.

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <grampc_s/grampc_s.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

namespace
{

bool isFiniteVec(const typeRNum * v, int n, const char * name)
{
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(v[i])) {
      std::cerr << "    " << name << "[" << i << "] = " << v[i] << " (NON-FINITE)\n";
      return false;
    }
  }
  return true;
}

}  // namespace

int main()
{
  using namespace asr_sdm_kinematic_dynamic_model;

  try {
    UnderwaterSimulatorParameters sim_params;
    sim_params.gravity_world = Eigen::Vector3d(0.0, 0.0, -9.81);
    sim_params.integration_method = IntegrationMethod::RungeKutta4;
    sim_params.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
    sim_params.pinocchio.use_free_flyer = true;
    sim_params.pinocchio.lock_rotor_joints = true;

    // Damping and buoyancy tuned for the URDF total mass (~1.0 kg across 4
    // links). Matches the verified parameters in dynamics_validation_test.
    for (auto & link : sim_params.hydrodynamics.links) {
      link.mass = 0.25;
      link.displaced_volume = 5.0e-4;  // 0.5 L per link, total 2 L = 2 kg of buoyancy
      link.added_mass.setIdentity();
      link.added_mass *= 0.1;
      link.linear_damping.setConstant(2.0);
      link.quadratic_damping.setConstant(0.1);
    }
    sim_params.actuators.maximum_segment_thrust.setConstant(10.0);

    GrampcDimensionConfig dimensions;
    GrampcCostWeights weights;
    // Config error weights (delta_q in tangent chart)
    weights.state_weight.head<kMpcConfigurationTangentDim>().setConstant(2.0);
    // Base linear / angular velocity weights
    weights.state_weight.segment<6>(kMpcConfigurationTangentDim).setConstant(5.0);
    // Joint velocity weights
    weights.state_weight.tail<kNumJointDofs>().setConstant(5.0);
    weights.terminal_weight = weights.state_weight * 10.0;
    // Heavier thrust weight so the controller prefers low thrust when not needed
    weights.control_weight.head<kNumLinks>().setConstant(2.0);
    weights.control_weight.tail<kNumJointDofs>().setConstant(0.5);

    auto dynamics = std::make_shared<GrampcUnderwaterDynamics>(sim_params, dimensions, weights);
    if (!dynamics->isValid()) {
      std::cerr << "Failed to initialize dynamics: " << dynamics->error() << std::endl;
      return 1;
    }

    grampc::Grampc solver(dynamics);
    solver.setparam_real("Thor", 0.5);
    solver.setparam_real("Tmax", 1.0);
    solver.setparam_real("Tmin", 0.05);
    solver.setparam_real("dt", 0.02);
    solver.setparam_real("t0", 0.0);
    solver.setopt_int("Nhor", 25);
    solver.setparam_real("Thor", 0.5);  // Nhor realloc resets Thor
    solver.setopt_int("MaxGradIter", 5);
    solver.setopt_int("MaxMultIter", 1);
    solver.setopt_string("Integrator", "erk2");
    solver.setopt_real("PenaltyMin", 1.0e3);

    std::vector<typeRNum> x0(kMpcStateDim, 0.0);
    std::vector<typeRNum> xdes(kMpcStateDim, 0.0);
    std::vector<typeRNum> umin(kMpcControlDim);
    std::vector<typeRNum> umax(kMpcControlDim);
    // Target: base linear vx = 1.0 m/s, zero everything else.
    // In full-state mode, xdes = [q_target, v_target].  We use the
    // initial (zero-velocity) configuration as the target q.
    const auto & init_q = dynamics->simulator().pinocchioModel().configuration();
    for (int i = 0; i < kReducedNq; ++i) {
      xdes[i] = static_cast<typeRNum>(init_q(i));
    }
    xdes[kMpcConfigurationTangentDim + 0] = 1.0;

    const auto & max_thrust =
      dynamics->simulator().actuatorModel().parameters().maximum_segment_thrust;
    for (std::size_t i = 0; i < kNumLinks; ++i) {
      umin[i] = 0.0;
      umax[i] = max_thrust(i);  // 10 N per segment
    }
    for (int i = kNumLinks; i < kMpcControlDim; ++i) {
      umin[i] = -10.0;
      umax[i] = 10.0;
    }
    std::vector<typeRNum> u0(kMpcControlDim, 0.0);

    solver.setparam_real_vector("xdes", xdes.data());
    solver.setparam_real_vector("u0", u0.data());
    solver.setparam_real_vector("udes", u0.data());
    solver.setparam_real_vector("umin", umin.data());
    solver.setparam_real_vector("umax", umax.data());

    // Closed-loop state - the simulator's full state.
    auto state = dynamics->simulator().makeInitialState();

    const double control_period = 0.02;
    constexpr int kSubSteps = 5;
    const double dt = control_period / kSubSteps;
    constexpr int kSteps = 100;  // 2 seconds

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "=== Closed-loop GRAMPC + UnderwaterSimulator ===" << std::endl;
    std::cout << "Target: base linear vx = 1.0 m/s (rest zero)" << std::endl;
    std::cout << "Solver horizon: Thor=0.5s, dt=0.02s, Nhor=25" << std::endl;
    std::cout << "Plant:  RK4 sub-stepped at " << dt << "s x" << kSubSteps << std::endl;
    std::cout << "Steps: " << kSteps << " (=" << kSteps * control_period << " s)\n" << std::endl;
    std::cout <<
      " step |  time  |   vx   |   vy   |   vz   |  u0   |  u1   |  u2   |  u3   | J    \n";
    std::cout <<
      "------+--------+--------+--------+--------+-------+-------+-------+-------+-------\n";

    bool any_nan = false;
    for (int step = 0; step < kSteps; ++step) {
      // Full-state mode: x0 = [q_measured, v_measured] — no tangent reference needed.
      {
        ReducedConfiguration q_norm = state.configuration;
        q_norm.segment<4>(3).normalize();
        for (int i = 0; i < kReducedNq; ++i) {
          x0[i] = static_cast<typeRNum>(q_norm(i));
        }
      }
      for (int i = 0; i < kReducedNv; ++i) {
        x0[kMpcConfigurationTangentDim + i] = static_cast<typeRNum>(state.velocity(i));
      }
      solver.setparam_real_vector("x0", x0.data());
      solver.setparam_real("t0", static_cast<typeRNum>(state.time));

      solver.run();

      const typeGRAMPCsol * sol = solver.getSolution();
      if (sol == nullptr || !isFiniteVec(sol->unext, kMpcControlDim, "u") ||
        !isFiniteVec(sol->J, 1, "J"))
      {
        std::cerr << "NaN at step " << step << std::endl;
        any_nan = true;
        break;
      }

      UnderwaterSimulatorInput input;
      for (std::size_t i = 0; i < kNumLinks; ++i) {
        input.segment_thrust(i) = sol->unext[i];
      }
      for (std::size_t i = 0; i < kNumJointDofs; ++i) {
        input.joint_torque(i) = sol->unext[kNumLinks + i];
      }

      for (int substep = 0; substep < kSubSteps; ++substep) {
        dynamics->simulator().step(state, dt, input);
      }

      if (step % 10 == 0) {
        std::cout << std::setw(5) << step << " | "
                  << std::setw(6) << state.time << " | "
                  << std::setw(6) << state.velocity(0) << " | "
                  << std::setw(6) << state.velocity(1) << " | "
                  << std::setw(6) << state.velocity(2) << " | "
                  << std::setw(5) << input.segment_thrust(0) << " | "
                  << std::setw(5) << input.segment_thrust(1) << " | "
                  << std::setw(5) << input.segment_thrust(2) << " | "
                  << std::setw(5) << input.segment_thrust(3) << " | "
                  << std::setw(5) << sol->J[0] << "\n";
      }
    }

    std::cout << "\n=== Final state ===" << std::endl;
    std::cout << "  vx = " << state.velocity(0) << " (target 1.0)\n";
    std::cout << "  vy = " << state.velocity(1) << "\n";
    std::cout << "  vz = " << state.velocity(2) << "\n";
    std::cout << "  All finite? " << (!any_nan ? "YES" : "NO") << std::endl;

    return any_nan ? 1 : 0;
  } catch (const std::exception & e) {
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return 1;
  }
}

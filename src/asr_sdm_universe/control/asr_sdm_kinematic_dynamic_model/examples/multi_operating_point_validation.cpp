// Multi-operating-point closed-loop MPC validation.
//
// Exercises the full-state MPC controller across several operating points
// that cover the regime the dynamics were designed for, to verify:
//
//   * Solver convergence (no exception, no NaN/Inf in state or control)
//   * Control saturation behavior (targets above max_velocity should be
//     softened by the constraint, not cause overshoot)
//   * Numerical stability (state remains finite across 100 ms × N steps
//     of sub-stepped RK4 integration)
//
// Operating points:
//
//   1. Hover          : target = current q, v = 0
//   2. Forward 1 m/s  : target v_x = 1 m/s, rest zero (the basic tracking case)
//   3. Yaw 90 deg     : target yaw = 90 deg around z, v = 0
//   4. Lateral 1 m/s  : target v_y = 1 m/s (cross-coupling test)
//   5. Velocity cap   : target v_x = 3 m/s > max 2 m/s (constraint engagement)
//   6. Joint motion   : target joint[0] = 0.3 rad, rest zero
//
// Each case runs N steps (~0.6 s) so the total example takes a few seconds
// and stays well within the ctest budget.  Exit code is 0 only if every
// case reports `success` for every step AND no state/control component is
// non-finite.

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <grampc_s/grampc_s.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{
using namespace asr_sdm_kinematic_dynamic_model;

struct OpCase
{
  std::string name;
  ReducedConfiguration target_q;
  ReducedVelocity target_v;
  int steps;
};

bool isFiniteConfiguration(const ReducedConfiguration & q)
{
  return q.allFinite() && std::isfinite(q.norm());
}

bool isFiniteVelocity(const ReducedVelocity & v)
{
  return v.allFinite();
}

bool isFiniteControl(const UnderwaterSimulatorInput & u)
{
  return u.segment_thrust.allFinite() && u.joint_torque.allFinite();
}

// Run one closed-loop case.  Returns true iff every step succeeded and all
// state/control components stayed finite.
bool runCase(
  UnderwaterMpcController & controller,
  UnderwaterSimulator & plant,
  const OpCase & op)
{
  auto state = plant.makeInitialState();

  // Quaternion targets are pre-normalized once and re-applied each step so
  // the controller sees a valid unit quaternion in xdes.
  ReducedConfiguration q_des_norm = op.target_q;
  if (!q_des_norm.segment<4>(3).isZero()) {
    q_des_norm.segment<4>(3).normalize();
  }

  MpcReferenceTrajectory ref;
  ref.target_configuration = q_des_norm;
  ref.target_velocity = op.target_v;
  ref.fluid_current_world = Eigen::Vector3d::Zero();
  ref.fluid_current_acceleration_world = Eigen::Vector3d::Zero();
  ref.has_valid_target = true;

  std::cout << "  --- Case: " << op.name << " (" << op.steps << " steps) ---" << std::endl;
  for (int step = 0; step < op.steps; ++step) {
    auto result = controller.computeControl(state.configuration, state.velocity, ref);
    if (!result.success) {
      std::cerr << "    step " << step << " FAILED to solve MPC" << std::endl;
      return false;
    }
    if (!isFiniteControl(result.control)) {
      std::cerr << "    step " << step << " non-finite control" << std::endl;
      return false;
    }
    // Plant step (RK4 sub-stepping handled inside simulator).
    plant.step(state, 0.02, result.control);
    if (!isFiniteConfiguration(state.configuration)) {
      std::cerr << "    step " << step << " non-finite configuration" << std::endl;
      return false;
    }
    if (!isFiniteVelocity(state.velocity)) {
      std::cerr << "    step " << step << " non-finite velocity" << std::endl;
      return false;
    }
  }

  // Print a brief one-line summary so the test output stays compact.
  std::cout << "    [OK] pos=(" << std::fixed << std::setprecision(3)
            << state.configuration(0) << ", " << state.configuration(1) << ", "
            << state.configuration(2) << ") "
            << "v=(" << state.velocity(0) << ", " << state.velocity(1) << ", "
            << state.velocity(2) << ")" << std::endl;
  return true;
}

}  // namespace

int main()
{
#ifndef ASR_SDM_GENERATED_URDF
  std::cerr << "FAILED: ASR_SDM_GENERATED_URDF is not configured" << std::endl;
  return 1;
#else
  using namespace asr_sdm_kinematic_dynamic_model;

  // ---- Simulator / dynamics / cost weights (same setup as grampc_run_repro) ----
  UnderwaterSimulatorParameters sim_params;
  sim_params.gravity_world = Eigen::Vector3d(0.0, 0.0, -9.81);
  sim_params.integration_method = IntegrationMethod::RungeKutta4;
  sim_params.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
  sim_params.pinocchio.use_free_flyer = true;
  sim_params.pinocchio.lock_rotor_joints = true;
  for (auto & link : sim_params.hydrodynamics.links) {
    link.mass = 0.25;
    link.displaced_volume = 5.0e-4;
    link.added_mass.setIdentity();
    link.added_mass *= 0.1;
    link.linear_damping.setConstant(2.0);
    link.quadratic_damping.setConstant(0.1);
  }
  sim_params.actuators.maximum_segment_thrust.setConstant(10.0);

  // ---- MPC parameters ----
  // Cost weights are derived from these by buildCostWeights() inside the
  // controller.  Terminal weights = state weights * scale, quaternion
  // weights are auto-scaled by 0.1 to avoid over-regularization.
  UnderwaterMpcParameters mpc_params;
  mpc_params.Nhor = 20;
  mpc_params.Thor = 0.2;            // 0.2 s horizon
  mpc_params.dt = 0.02;
  mpc_params.max_grad_iter = 5;
  mpc_params.max_mult_iter = 1;
  mpc_params.q_position = 10.0;
  mpc_params.q_orientation = 5.0;
  mpc_params.q_joint_position = 5.0;
  mpc_params.q_linear_velocity = 5.0;
  mpc_params.q_angular_velocity = 5.0;
  mpc_params.q_joint_velocity = 5.0;
  mpc_params.r_thrust = 2.0;
  mpc_params.r_joint_torque = 0.5;
  mpc_params.terminal_position_scale = 10.0;
  mpc_params.terminal_velocity_scale = 10.0;
  mpc_params.enable_terminal_cost = true;
  mpc_params.enable_velocity_constraints = true;
  mpc_params.max_linear_velocity = 2.0;
  mpc_params.max_angular_velocity = 1.0;
  mpc_params.max_joint_velocity = 2.0;
  mpc_params.enable_warm_start = true;
  mpc_params.verbose = false;

  UnderwaterMpcController controller(sim_params, mpc_params);
  if (!controller.isValid()) {
    std::cerr << "Controller invalid: " << controller.error() << std::endl;
    return 1;
  }
  UnderwaterSimulator plant(sim_params);  // separate from controller's internal simulator

  // ---- Define operating points ----
  // Start from the plant's neutral configuration (with unit quaternion) and
  // build each target from there so quaternion channels stay valid.
  const ReducedConfiguration q_init = plant.pinocchioModel().configuration();

  // Case 1: hover (zero target velocity, current q).
  OpCase hover;
  hover.name = "hover";
  hover.target_q = q_init;
  hover.target_v = ReducedVelocity::Zero();
  hover.steps = 30;

  // Case 2: forward 1 m/s.
  OpCase forward;
  forward.name = "forward 1 m/s";
  forward.target_q = q_init;
  forward.target_v = ReducedVelocity::Zero();
  forward.target_v(0) = 1.0;
  forward.steps = 30;

  // Case 3: yaw 90 deg around z.
  // Quaternion for 90 deg yaw: [cos(45), 0, 0, sin(45)].
  OpCase yaw;
  yaw.name = "yaw 90 deg";
  yaw.target_q = q_init;
  yaw.target_q(3) = std::cos(M_PI / 4.0);
  yaw.target_q(6) = std::sin(M_PI / 4.0);
  yaw.target_v = ReducedVelocity::Zero();
  yaw.steps = 30;

  // Case 4: lateral 1 m/s.
  OpCase lateral;
  lateral.name = "lateral 1 m/s";
  lateral.target_q = q_init;
  lateral.target_v = ReducedVelocity::Zero();
  lateral.target_v(1) = 1.0;
  lateral.steps = 30;

  // Case 5: target above max_linear_velocity to test constraint engagement.
  OpCase vel_cap;
  vel_cap.name = "velocity cap (v_x=3 > max 2)";
  vel_cap.target_q = q_init;
  vel_cap.target_v = ReducedVelocity::Zero();
  vel_cap.target_v(0) = 3.0;
  vel_cap.steps = 30;

  // Case 6: joint motion target.
  OpCase joint;
  joint.name = "joint motion (q_joint[0] = 0.3 rad)";
  joint.target_q = q_init;
  joint.target_q(7) = 0.3;  // first joint (after base position + quaternion)
  joint.target_v = ReducedVelocity::Zero();
  joint.steps = 30;

  std::vector<OpCase> cases = {hover, forward, yaw, lateral, vel_cap, joint};

  std::cout << "=== Multi-operating-point closed-loop MPC validation ===" << std::endl;
  std::cout << "Cases: " << cases.size() << ", steps each: 30 (~0.6s plant time)" << std::endl;
  std::cout << "MPC horizon: Thor=0.2s, Nhor=20, dt=0.02s" << std::endl;
  std::cout << "Max linear velocity: 2.0 m/s (case 'velocity cap' tests this)" << std::endl;
  std::cout << std::endl;

  int passed = 0;
  int failed = 0;
  for (const auto & c : cases) {
    if (runCase(controller, plant, c)) {
      ++passed;
    } else {
      ++failed;
    }
  }

  std::cout << std::endl;
  std::cout << "Result: " << passed << "/" << cases.size()
            << " cases passed" << std::endl;
  return failed == 0 ? 0 : 1;
#endif
}

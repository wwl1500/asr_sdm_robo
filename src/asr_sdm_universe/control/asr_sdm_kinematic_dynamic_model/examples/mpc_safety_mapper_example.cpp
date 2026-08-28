// Closed-loop MPC + Safety Mapper integration demo.
//
// Pipeline:
//   MPC controller.computeControl() -> MpcSafetyMapper.map()
//   -> UnderwaterSimulator.step()
//
// Three scenarios:
//   1. Normal operation: no fault, no clamping (baseline)
//   2. Slew rate limit engaged: tight thrust rate, large step command
//   3. Fault injection: NaN in MPC output -> safe-mode engages

#include "asr_sdm_kinematic_dynamic_model/mpc_safety_mapper.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace mpc = asr_sdm_kinematic_dynamic_model;

namespace
{

void buildSimParams(mpc::UnderwaterSimulatorParameters & p)
{
  p.gravity_world = Eigen::Vector3d(0.0, 0.0, -9.81);
  p.integration_method = mpc::IntegrationMethod::RungeKutta4;
  p.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
  p.pinocchio.use_free_flyer = true;
  p.pinocchio.lock_rotor_joints = true;
  for (auto & link : p.hydrodynamics.links) {
    link.mass = 0.25;
    link.displaced_volume = 5.0e-4;
    link.added_mass.setIdentity();
    link.added_mass *= 0.1;
    link.linear_damping.setConstant(2.0);
    link.quadratic_damping.setConstant(0.1);
  }
  p.actuators.maximum_segment_thrust.setConstant(10.0);
}

void buildMpcParams(mpc::UnderwaterMpcParameters & p)
{
  p.Thor = 0.5;
  p.Nhor = 25;
  p.dt = 0.02;
  p.max_grad_iter = 5;
  p.max_mult_iter = 1;
  p.enable_warm_start = true;
  p.verbose = false;
  p.q_position = 10.0;
  p.q_orientation = 5.0;
  p.q_linear_velocity = 5.0;
  p.q_angular_velocity = 2.0;
  p.q_joint_position = 5.0;
  p.q_joint_velocity = 2.0;
  p.r_thrust = 0.1;
  p.r_joint_torque = 0.01;
  p.enable_terminal_cost = true;
  p.enable_velocity_constraints = true;
  p.max_linear_velocity = 2.0;
  p.max_angular_velocity = 1.0;
  p.max_joint_velocity = 2.0;
}

struct RunResult
{
  int n_clamp_events{0};
  int n_slew_events{0};
  int n_safe_mode_events{0};
  int total_steps{0};
  bool ran_to_completion{false};
};

RunResult runScenario(
  const std::string & label,
  mpc::MpcSafetyParameters safety_params,
  bool inject_fault_at_step_50)
{
  std::cout << "--- " << label << " ---\n";
  mpc::UnderwaterSimulatorParameters sim_params;
  buildSimParams(sim_params);
  mpc::UnderwaterMpcParameters mpc_params;
  buildMpcParams(mpc_params);

  mpc::UnderwaterMpcController controller(sim_params, mpc_params);
  if (!controller.isValid()) {
    std::cerr << "  Controller invalid: " << controller.error() << "\n";
    return {};
  }
  mpc::MpcSafetyMapper safety_mapper(safety_params);

  mpc::MpcReferenceTrajectory ref;
  ref.has_valid_target = true;
  ref.target_velocity.head<3>() = Eigen::Vector3d(0.5, 0.0, 0.0);

  auto state = controller.simulator().makeInitialState();
  const auto initial_pose = state.configuration;

  constexpr int kSteps = 80;
  constexpr double kDt = 0.02;
  constexpr int kSubSteps = 5;
  constexpr double kSubDt = kDt / kSubSteps;

  RunResult r;
  r.total_steps = kSteps;

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  step |   t   |  vx   | thrust0 | status\n";
  std::cout << "  -----+-------+-------+---------+----------------\n";

  for (int step = 0; step < kSteps; ++step) {
    const double t = step * kDt;
    auto ctrl_result = controller.computeControl(
      state.configuration, state.velocity, ref, t);

    mpc::UnderwaterSimulatorInput mpc_cmd = ctrl_result.control;

    // Optional fault injection.
    if (inject_fault_at_step_50 && step == 50) {
      mpc_cmd.segment_thrust(0) = std::numeric_limits<double>::quiet_NaN();
      std::cout << "  [fault injection: NaN at step 50]\n";
    }

    auto safe_cmd = safety_mapper.map(mpc_cmd, state, initial_pose, kDt);

    const auto & st = safety_mapper.lastStatus();
    if (st.input_was_clamped) {++r.n_clamp_events;}
    if (st.slew_rate_was_clamped) {++r.n_slew_events;}
    if (st.safe_mode_active) {++r.n_safe_mode_events;}

    for (int s = 0; s < kSubSteps; ++s) {
      controller.simulator().step(state, kSubDt, safe_cmd);
    }

    if (step % 20 == 0 || step == 50 || step == 51) {
      std::cout << "  " << std::setw(4) << step << " | " << std::setw(5) << t << " | "
                << std::setw(5) << state.velocity(0) << " | "
                << std::setw(7) << safe_cmd.segment_thrust(0) << " | "
                << (st.safe_mode_active ? "SAFE_MODE" :
      st.slew_rate_was_clamped ? "slew" :
      st.input_was_clamped ? "clamp" : "ok") << "\n";
    }
  }

  r.ran_to_completion = true;
  std::cout << "  clamp_events=" << r.n_clamp_events
            << " slew_events=" << r.n_slew_events
            << " safe_mode_events=" << r.n_safe_mode_events << "\n\n";
  return r;
}

}  // namespace

int main()
{
  std::cout << "=== MPC + Safety Mapper Integration Demo ===\n\n";

  mpc::MpcSafetyParameters safety_normal;
  safety_normal.max_segment_thrust.setConstant(10.0);
  safety_normal.max_joint_torque.setConstant(8.0);
  safety_normal.max_segment_thrust_rate = 1000.0;  // permissive
  safety_normal.max_joint_torque_rate = 1000.0;
  safety_normal.safe_mode_thrust_scale = 0.0;
  safety_normal.safe_mode_torque_scale = 0.0;
  auto r1 = runScenario("Scenario 1: Normal operation", safety_normal, false);

  mpc::MpcSafetyParameters safety_tight_slew;
  safety_tight_slew.max_segment_thrust.setConstant(10.0);
  safety_tight_slew.max_joint_torque.setConstant(8.0);
  safety_tight_slew.max_segment_thrust_rate = 20.0;  // 20 N/s -> 0.4 N/step
  safety_tight_slew.max_joint_torque_rate = 10.0;
  safety_tight_slew.safe_mode_thrust_scale = 0.0;
  safety_tight_slew.safe_mode_torque_scale = 0.0;
  auto r2 = runScenario("Scenario 2: Tight slew rate (20 N/s)", safety_tight_slew, false);

  mpc::MpcSafetyParameters safety_fault;
  safety_fault.max_segment_thrust.setConstant(10.0);
  safety_fault.max_joint_torque.setConstant(8.0);
  safety_fault.max_segment_thrust_rate = 1000.0;
  safety_fault.max_joint_torque_rate = 1000.0;
  safety_fault.safe_mode_thrust_scale = 0.0;
  safety_fault.safe_mode_torque_scale = 0.0;
  auto r3 = runScenario("Scenario 3: NaN fault at step 50", safety_fault, true);

  std::cout << "=== Summary ===\n";
  std::cout << "  S1 (normal):    clamp=" << r1.n_clamp_events
            << " slew=" << r1.n_slew_events << " safe=" << r1.n_safe_mode_events << "\n";
  std::cout << "  S2 (tight slew): clamp=" << r2.n_clamp_events
            << " slew=" << r2.n_slew_events << " safe=" << r2.n_safe_mode_events << "\n";
  std::cout << "  S3 (fault):      clamp=" << r3.n_clamp_events
            << " slew=" << r3.n_slew_events << " safe=" << r3.n_safe_mode_events << "\n";

  const bool ok = r1.ran_to_completion && r2.ran_to_completion && r3.ran_to_completion &&
    r1.n_safe_mode_events == 0 && r3.n_safe_mode_events > 0;
  std::cout << "\n" << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}

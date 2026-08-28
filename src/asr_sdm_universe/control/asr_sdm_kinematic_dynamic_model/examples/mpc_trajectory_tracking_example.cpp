// Time-varying trajectory MPC tracking example.
//
// Demonstrates the MpcReferenceTrajectory::setTrajectory() / evaluate(t)
// API for closed-loop tracking of a moving target.
//
// Scenario:
//   - T=0: robot starts at rest
//   - T=0..1s: target ramps up to vx=0.5 m/s
//   - T=1..2s: target maintains vx=0.5 m/s
//   - T=2..3s: target decelerates to vx=0
//   - Throughout: lateral vy oscillates sinusoidally (figure-Z pattern)
//
// For comparison, a static-baseline run tracks a fixed target (vx=0.5, vy=0).
// Both runs log position, velocity, and control effort.

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace mpc = asr_sdm_kinematic_dynamic_model;

constexpr int kSteps = 150;         // 3 seconds at 20ms
constexpr double kDt = 0.02;
constexpr int kSubSteps = 5;
constexpr double kSubDt = kDt / kSubSteps;

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
  p.dt = kDt;
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

// Velocity profile: ramp up (0→0.5 m/s in 1s), hold, ramp down.
double vxProfile(double t)
{
  if (t < 1.0) {return 0.5 * t;}
  if (t < 2.0) {return 0.5;}
  if (t < 3.0) {return 0.5 * (3.0 - t);}
  return 0.0;
}

// Sinusoidal lateral velocity: vy = 0.2 * sin(2π * t / 3).
double vyProfile(double t)
{
  return 0.2 * std::sin(2.0 * M_PI * t / 3.0);
}

// Integral of vyProfile → lateral position y(t).
double yPositionProfile(double t)
{
  return 0.2 * (-3.0 / (2.0 * M_PI)) * std::cos(2.0 * M_PI * t / 3.0) +
         0.2 * (-3.0 / (2.0 * M_PI));
}

// Integral of vxProfile → forward position x(t).
double xPositionProfile(double t)
{
  if (t < 1.0) {return 0.5 * 0.5 * t * t;}
  if (t < 2.0) {return 0.25 + 0.5 * (t - 1.0);}
  if (t < 3.0) {return 0.75 + 0.5 * (t - 2.0) - 0.25 * (t - 2.0) * (t - 2.0);}
  return 1.0;
}

// Build the time-varying reference trajectory from analytic profiles.
mpc::MpcReferenceTrajectory buildTrajectory()
{
  mpc::MpcReferenceTrajectory ref;
  std::vector<mpc::MpcTrajectoryPoint> points;
  constexpr int kNumPoints = 16;
  for (int i = 0; i <= kNumPoints; ++i) {
    const double t = i * 3.0 / kNumPoints;
    mpc::MpcTrajectoryPoint p;
    p.time = t;
    p.configuration.head<3>() = Eigen::Vector3d(xPositionProfile(t), yPositionProfile(t), 0.0);
    p.configuration.segment<4>(3) = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    p.configuration.tail<6>().setZero();
    p.velocity.head<3>() = Eigen::Vector3d(vxProfile(t), vyProfile(t), 0.0);
    p.velocity.tail<9>().setZero();
    points.push_back(p);
  }
  ref.setTrajectory(points);
  return ref;
}

// Run one closed-loop simulation and return tracking error stats.
struct TrackingResult
{
  double mean_vx_error = 0.0;
  double mean_vy_error = 0.0;
  double max_vx_error = 0.0;
  double max_vy_error = 0.0;
  double rms_vx_error = 0.0;
  double rms_vy_error = 0.0;
  bool success = true;
};

TrackingResult runTracking(
  mpc::UnderwaterSimulatorParameters & sim_params,
  mpc::UnderwaterMpcParameters & mpc_params,
  const mpc::MpcReferenceTrajectory & ref,
  bool is_static_baseline,
  const std::string & label)
{
  TrackingResult result;
  mpc::UnderwaterMpcController controller(sim_params, mpc_params);
  if (!controller.isValid()) {
    std::cerr << "  Controller invalid: " << controller.error() << "\n";
    result.success = false;
    return result;
  }

  auto state = controller.simulator().makeInitialState();
  double sum_vx = 0.0, sum_vy = 0.0;
  double sum_vx2 = 0.0, sum_vy2 = 0.0;
  double max_vx = 0.0, max_vy = 0.0;

  for (int step = 0; step < kSteps; ++step) {
    const double t = step * kDt;

    mpc::MpcReferenceTrajectory ref_use = ref;
    if (is_static_baseline) {
      // Static: set fixed target (no trajectory).
      ref_use.has_valid_target = true;
      ref_use.target_velocity.head<3>() = Eigen::Vector3d(0.5, 0.0, 0.0);
    }

    auto ctrl_result = controller.computeControl(
      state.configuration, state.velocity, ref_use, t);

    if (!ctrl_result.success) {
      result.success = false;
      break;
    }

    mpc::UnderwaterSimulatorInput input;
    input.segment_thrust = ctrl_result.control.segment_thrust;
    input.joint_torque = ctrl_result.control.joint_torque;

    for (int s = 0; s < kSubSteps; ++s) {
      controller.simulator().step(state, kSubDt, input);
    }

    const double target_vx = is_static_baseline ? 0.5 : vxProfile(t);
    const double target_vy = is_static_baseline ? 0.0 : vyProfile(t);
    const double err_vx = state.velocity(0) - target_vx;
    const double err_vy = state.velocity(1) - target_vy;
    sum_vx += err_vx;
    sum_vy += err_vy;
    sum_vx2 += err_vx * err_vx;
    sum_vy2 += err_vy * err_vy;
    max_vx = std::max(max_vx, std::abs(err_vx));
    max_vy = std::max(max_vy, std::abs(err_vy));
  }

  const double n = static_cast<double>(kSteps);
  result.mean_vx_error = sum_vx / n;
  result.mean_vy_error = sum_vy / n;
  result.max_vx_error = max_vx;
  result.max_vy_error = max_vy;
  result.rms_vx_error = std::sqrt(sum_vx2 / n);
  result.rms_vy_error = std::sqrt(sum_vy2 / n);
  return result;
}

int main()
{
  std::cout << "=== Time-Varying Trajectory MPC Tracking ===" << std::endl;
  std::cout << "Steps: " << kSteps << " (" << kSteps * kDt << "s)\n";
  std::cout << "Control period: " << kDt << "s, sub-steps: " << kSubSteps << "\n\n";

  mpc::UnderwaterSimulatorParameters sim_params;
  buildSimParams(sim_params);

  mpc::UnderwaterMpcParameters mpc_params;
  buildMpcParams(mpc_params);

  const mpc::MpcReferenceTrajectory ref = buildTrajectory();

  std::cout << "  Trajectory: vx ramp(0→0.5)→hold→decel, vy=0.2*sin(2πt/3)\n\n";

  // Run 1: Time-varying (full tracking).
  std::cout << "--- Run 1: Time-varying trajectory tracking ---\n";
  auto res_tv = runTracking(sim_params, mpc_params, ref, false, "TV");
  if (!res_tv.success) {
    std::cerr << "FAILED: time-varying run crashed\n";
    return 1;
  }
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  mean_vx_err=" << res_tv.mean_vx_error
            << "  max_vx_err=" << res_tv.max_vx_error
            << "  rms_vx_err=" << res_tv.rms_vx_error << "\n";
  std::cout << "  mean_vy_err=" << res_tv.mean_vy_error
            << "  max_vy_err=" << res_tv.max_vy_error
            << "  rms_vy_err=" << res_tv.rms_vy_error << "\n";

  // Run 2: Static baseline (target = 0.5 m/s constant, ignores trajectory).
  std::cout << "\n--- Run 2: Static baseline (vx=0.5 constant, ignores trajectory) ---\n";
  auto res_static = runTracking(sim_params, mpc_params, ref, true, "STATIC");
  if (!res_static.success) {
    std::cerr << "FAILED: static baseline crashed\n";
    return 1;
  }
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  mean_vx_err=" << res_static.mean_vx_error
            << "  max_vx_err=" << res_static.max_vx_error
            << "  rms_vx_err=" << res_static.rms_vx_error << "\n";
  std::cout << "  mean_vy_err=" << res_static.mean_vy_error
            << "  max_vy_err=" << res_static.max_vy_error
            << "  rms_vy_err=" << res_static.rms_vy_error << "\n";

  // Comparison.
  std::cout << "\n--- Comparison ---\n";
  std::cout << "  TV RMS vx error:  " << res_tv.rms_vx_error
            << "  vs  STATIC RMS vx error:  " << res_static.rms_vx_error << "\n";
  std::cout << "  TV RMS vy error:  " << res_tv.rms_vy_error
            << "  vs  STATIC RMS vy error:  " << res_static.rms_vy_error << "\n";
  std::cout << "  (TV should have lower vy error since it tracks the moving target)\n";

  const bool tv_better_vy = res_tv.rms_vy_error < res_static.rms_vy_error * 0.8;
  std::cout << "\n" << (tv_better_vy ? "PASS" : "NOTE")
            << ": Time-varying tracking shows "
            << (tv_better_vy ? "significantly" : "not a clear")
            << " improvement on lateral tracking.\n";

  return 0;
}

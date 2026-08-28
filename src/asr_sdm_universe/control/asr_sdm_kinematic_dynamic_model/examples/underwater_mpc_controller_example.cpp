// Copyright (c) 2025.
// Complete MPC controller example with constraints and full state tracking.

#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iomanip>
#include <iostream>

namespace
{

std::string generatedUrdfPath()
{
  return ament_index_cpp::get_package_share_directory("asr_sdm") +
         "/urdf/generated/asr_sdm_segments_4.urdf";
}

}  // namespace

int main()
{
  using namespace asr_sdm_kinematic_dynamic_model;

  try {
    // Configure simulator
    UnderwaterSimulatorParameters sim_params;
    sim_params.gravity_world = Eigen::Vector3d(0.0, 0.0, -9.81);
    sim_params.integration_method = IntegrationMethod::RungeKutta4;
    sim_params.pinocchio.urdf_path = generatedUrdfPath();
    sim_params.pinocchio.use_free_flyer = true;
    sim_params.pinocchio.lock_rotor_joints = true;

    // Configure MPC controller
    UnderwaterMpcParameters mpc_params;
    mpc_params.Thor = 1.0;                    // 1 second prediction horizon
    mpc_params.Nhor = 50;                     // 50 steps
    mpc_params.dt = 0.02;                     // 20ms discretization

    // Cost weights
    mpc_params.q_position = 10.0;
    mpc_params.q_orientation = 5.0;
    mpc_params.q_linear_velocity = 2.0;
    mpc_params.q_angular_velocity = 2.0;
    mpc_params.q_joint_position = 5.0;
    mpc_params.q_joint_velocity = 2.0;

    mpc_params.r_thrust = 0.1;
    mpc_params.r_joint_torque = 0.01;

    // Terminal cost
    mpc_params.terminal_position_scale = 20.0;
    mpc_params.terminal_velocity_scale = 10.0;
    mpc_params.enable_terminal_cost = true;

    // Constraints
    mpc_params.enable_velocity_constraints = true;
    mpc_params.max_linear_velocity = 2.0;     // 2 m/s
    mpc_params.max_angular_velocity = 1.0;    // 1 rad/s
    mpc_params.max_joint_velocity = 2.0;      // 2 rad/s
    mpc_params.min_joint_torque = -10.0;
    mpc_params.max_joint_torque = 10.0;

    // Solver options
    mpc_params.max_grad_iter = 5;
    mpc_params.max_mult_iter = 1;
    mpc_params.integrator = "erk2";
    mpc_params.penalty_min = 1e3;

    // Logging
    mpc_params.verbose = true;
    mpc_params.log_throttle_sec = 0.5;

    // Create MPC controller
    UnderwaterMpcController controller(sim_params, mpc_params);

    if (!controller.isValid()) {
      std::cerr << "Failed to initialize MPC controller: " << controller.error() << std::endl;
      return 1;
    }

    std::cout << "MPC controller initialized successfully\n";
    std::cout << "Prediction horizon: " << mpc_params.Thor << " s (" << mpc_params.Nhor <<
      " steps)\n";
    std::cout << "Velocity constraints: v_lin=" << mpc_params.max_linear_velocity
              << " m/s, v_ang=" << mpc_params.max_angular_velocity << " rad/s\n\n";

    // Create plant simulator (separate from MPC's internal model)
    auto plant = std::make_unique<UnderwaterSimulator>(sim_params);
    if (!plant->isValid()) {
      std::cerr << "Failed to create plant simulator: " << plant->error() << std::endl;
      return 1;
    }

    auto plant_state = plant->makeInitialState();

    // Set up reference trajectory: move forward 2m, rotate 90 degrees
    MpcReferenceTrajectory reference;
    reference.has_valid_target = true;
    reference.target_configuration = ReducedConfiguration::Zero();
    reference.target_configuration(0) = 2.0;  // Move 2m in x
    reference.target_configuration(1) = 0.0;
    reference.target_configuration(2) = 0.0;
    // Rotate 90 degrees around z-axis: quaternion for 90° z-rotation
    // q = [cos(45°), 0, 0, sin(45°)] = [0.7071, 0, 0, 0.7071]
    reference.target_configuration(3) = 0.7071;  // qw
    reference.target_configuration(4) = 0.0;     // qx
    reference.target_configuration(5) = 0.0;     // qy
    reference.target_configuration(6) = 0.7071;  // qz

    // Keep joints at zero
    reference.target_configuration.tail<6>().setZero();

    // Zero target velocity (regulation)
    reference.target_velocity.setZero();
    reference.target_acceleration.setZero();

    // No fluid current
    reference.fluid_current_world.setZero();
    reference.fluid_current_acceleration_world.setZero();

    // Simulation parameters
    const double control_period = 0.02;  // 20ms control loop
    constexpr int kSubSteps = 5;         // 5 sub-steps for plant integration
    const double dt = control_period / kSubSteps;
    constexpr int kSimulationSteps = 300;  // 6 seconds simulation

    std::cout << "Starting closed-loop simulation...\n";
    std::cout << "Control period: " << control_period << " s\n";
    std::cout << "Plant integration dt: " << dt << " s (" << kSubSteps << " sub-steps)\n";
    std::cout << "Target: position=[2.0, 0.0, 0.0] m, yaw=90°\n\n";

    std::cout << std::fixed << std::setprecision(3)
              << "time[s]   pos_x   pos_y   pos_z    vx      vy      vz   "
              << "thrust0  torque0  cost      solve[ms]\n";
    std::cout << std::string(95, '-') << '\n';

    for (int step = 0; step < kSimulationSteps; ++step) {
      // Compute optimal control using MPC
      auto mpc_result = controller.computeControl(
        plant_state.configuration,
        plant_state.velocity,
        reference);

      if (!mpc_result.success) {
        std::cerr << "\nMPC failed at step " << step << ": "
                  << mpc_result.error_message << std::endl;
        return 1;
      }

      // Apply control to plant with sub-stepping
      for (int substep = 0; substep < kSubSteps; ++substep) {
        plant->step(plant_state, dt, mpc_result.control);
      }

      // Log every 10 steps (0.2s)
      if (step % 10 == 0) {
        std::cout << std::setw(7) << plant_state.time
                  << std::setw(8) << plant_state.configuration(0)
                  << std::setw(8) << plant_state.configuration(1)
                  << std::setw(8) << plant_state.configuration(2)
                  << std::setw(8) << plant_state.velocity(0)
                  << std::setw(8) << plant_state.velocity(1)
                  << std::setw(8) << plant_state.velocity(2)
                  << std::setw(9) << mpc_result.control.segment_thrust(0)
                  << std::setw(9) << mpc_result.control.joint_torque(0)
                  << std::scientific << std::setprecision(2)
                  << std::setw(10) << mpc_result.cost
                  << std::fixed << std::setprecision(2)
                  << std::setw(11) << mpc_result.solve_time_ms
                  << '\n';
      }

      // Check for convergence
      if (step > 100) {  // After 2 seconds
        double pos_error = (plant_state.configuration.head<3>() -
          reference.target_configuration.head<3>()).norm();
        double vel_norm = plant_state.velocity.head<3>().norm();

        if (pos_error < 0.05 && vel_norm < 0.05) {
          std::cout << "\nConverged at t=" << plant_state.time << " s\n";
          std::cout << "Final position error: " << std::setprecision(4)
                    << pos_error << " m\n";
          std::cout << "Final velocity: " << vel_norm << " m/s\n";
          break;
        }
      }
    }

    std::cout << "\nSimulation completed successfully\n";
    std::cout << "Final state:\n";
    std::cout << "  Position: [" << std::setprecision(3)
              << plant_state.configuration(0) << ", "
              << plant_state.configuration(1) << ", "
              << plant_state.configuration(2) << "] m\n";
    std::cout << "  Velocity: ["
              << plant_state.velocity(0) << ", "
              << plant_state.velocity(1) << ", "
              << plant_state.velocity(2) << "] m/s\n";

    // Compute final tracking error
    Eigen::Vector3d final_pos_error =
      plant_state.configuration.head<3>() - reference.target_configuration.head<3>();
    Eigen::Vector3d final_vel_error =
      plant_state.velocity.head<3>() - reference.target_velocity.head<3>();

    std::cout << "  Position error: " << final_pos_error.norm() << " m\n";
    std::cout << "  Velocity error: " << final_vel_error.norm() << " m/s\n";

  } catch (const std::exception & error) {
    std::cerr << "MPC controller example failed: " << error.what() << std::endl;
    return 1;
  }

  return 0;
}

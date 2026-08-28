// Copyright (c) 2025.
// GRAMPC MPC example using the production simulator as the closed-loop plant.

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <grampc_s/grampc_s.hpp>

#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

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
    UnderwaterSimulatorParameters sim_params;
    sim_params.gravity_world = Eigen::Vector3d(0.0, 0.0, -9.81);
    sim_params.integration_method = IntegrationMethod::RungeKutta4;
    sim_params.pinocchio.urdf_path = generatedUrdfPath();
    sim_params.pinocchio.use_free_flyer = true;
    sim_params.pinocchio.lock_rotor_joints = true;

    GrampcDimensionConfig dimensions;
    GrampcCostWeights weights;
    weights.state_weight.head<kMpcConfigurationTangentDim>().setConstant(2.0);
    weights.state_weight.segment<6>(kMpcConfigurationTangentDim).setConstant(10.0);
    weights.state_weight.tail<kNumJointDofs>().setConstant(5.0);
    weights.terminal_weight = weights.state_weight * 20.0;
    weights.control_weight.head<kNumLinks>().setConstant(0.1);
    weights.control_weight.tail<kNumJointDofs>().setConstant(0.01);

    auto dynamics = std::make_shared<GrampcUnderwaterDynamics>(sim_params, dimensions, weights);
    if (!dynamics->isValid()) {
      std::cerr << "Failed to initialize MPC dynamics: " << dynamics->error() << std::endl;
      return 1;
    }

    grampc::Grampc solver(dynamics);
    solver.setparam_real("Thor", 0.5);
    solver.setparam_real("Tmax", 1.0);
    solver.setparam_real("Tmin", 0.05);
    solver.setparam_real("dt", 0.02);
    solver.setparam_real("t0", 0.0);
    solver.setopt_int("Nhor", 25);
    // Nhor reallocation resets the internal time grid; set Thor once more afterwards.
    solver.setparam_real("Thor", 0.5);
    solver.setopt_int("MaxGradIter", 5);
    solver.setopt_int("MaxMultIter", 1);
    solver.setopt_string("Integrator", "erk2");
    solver.setopt_real("PenaltyMin", 1.0e3);

    std::vector<typeRNum> x0(kMpcStateDim, 0.0);
    std::vector<typeRNum> xdes(kMpcStateDim, 0.0);
    std::vector<typeRNum> umin(kMpcControlDim);
    std::vector<typeRNum> umax(kMpcControlDim);
    xdes[kMpcConfigurationTangentDim] = 1.0;
    const auto & maximum_thrust =
      dynamics->simulator().actuatorModel().parameters().maximum_segment_thrust;
    for (std::size_t i = 0; i < kNumLinks; ++i) {
      umin[i] = 0.0;
      umax[i] = maximum_thrust(i);
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

    auto state = dynamics->simulator().makeInitialState();
    // P2-1: Use sub-steps for stable integration of rigid plant
    const double control_period = 0.02;
    constexpr int kSubSteps = 5;
    const double dt = control_period / kSubSteps;
    constexpr int kSteps = 100;

    std::cout << std::fixed << std::setprecision(3)
              << "time[s]       vx       vy       vz  thrust0  torque0\n";
    for (int step = 0; step < kSteps; ++step) {
      // Anchor the tangent chart at the measured configuration for this solve.
      dynamics->updateReferenceConfiguration(state.configuration);
      std::fill(x0.begin(), x0.end(), 0.0);
      for (int i = 0; i < kReducedNv; ++i) {
        x0[kMpcConfigurationTangentDim + i] = state.velocity(i);
      }
      solver.setparam_real_vector("x0", x0.data());
      solver.setparam_real("t0", state.time);
      solver.run();

      const typeRNum * optimal_control = solver.getSolution()->unext;
      UnderwaterSimulatorInput input;
      for (std::size_t i = 0; i < kNumLinks; ++i) {
        input.segment_thrust(i) = optimal_control[i];
      }
      for (std::size_t i = 0; i < kNumJointDofs; ++i) {
        input.joint_torque(i) = optimal_control[kNumLinks + i];
      }
      // P2-1: Use sub-steps to maintain stability for rigid plant integration
      for (int substep = 0; substep < kSubSteps; ++substep) {
        dynamics->simulator().step(state, dt, input);
      }

      if (step % 10 == 0) {
        std::cout << std::setw(7) << state.time << std::setw(9) << state.velocity(0)
                  << std::setw(9) << state.velocity(1) << std::setw(9) << state.velocity(2)
                  << std::setw(9) << input.segment_thrust(0)
                  << std::setw(9) << input.joint_torque(0) << '\n';
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "MPC example failed: " << error.what() << std::endl;
    return 1;
  }

  return 0;
}

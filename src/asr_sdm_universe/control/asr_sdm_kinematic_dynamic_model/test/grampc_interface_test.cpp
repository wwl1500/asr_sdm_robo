#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"

#include <grampc_s/grampc_s.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

model::UnderwaterSimulatorParameters makeParameters()
{
  model::UnderwaterSimulatorParameters parameters;
  parameters.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
  parameters.pinocchio.use_free_flyer = true;
  parameters.pinocchio.lock_rotor_joints = true;
  parameters.pinocchio.initial_joint_positions = {0.1, -0.1, 0.05, -0.05, 0.02, -0.02};
  for (auto & link : parameters.hydrodynamics.links) {
    link.added_mass.setIdentity();
    link.linear_damping.setConstant(0.1);
    link.quadratic_damping.setConstant(0.01);
  }
  parameters.actuators.maximum_segment_thrust.setConstant(1.0);
  return parameters;
}

model::MpcState evaluate(
  model::GrampcUnderwaterDynamics & dynamics, const model::MpcState & state,
  const Eigen::Matrix<double, model::kMpcControlDim, 1> & control)
{
  grampc::Vector derivative(model::kMpcStateDim);
  grampc::Vector x = state;
  grampc::Vector u = control;
  grampc::Vector p(0);
  grampc::GrampcParam parameters;
  dynamics.ffct(derivative, 0.0, x, u, p, parameters);
  return derivative;
}

bool checkTransposeJacobians(model::GrampcUnderwaterDynamics & dynamics)
{
  model::MpcState state = model::MpcState::Zero();
  Eigen::Matrix<double, model::kMpcControlDim, 1> control =
    Eigen::Matrix<double, model::kMpcControlDim, 1>::Zero();
  model::MpcState adjoint = model::MpcState::LinSpaced(-0.3, 0.4);

  constexpr double epsilon = 1.0e-6;
  model::MpcStateJacobian expected_x;
  for (int i = 0; i < model::kMpcStateDim; ++i) {
    model::MpcState plus = state;
    model::MpcState minus = state;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    expected_x.col(i) = (evaluate(dynamics, plus, control) - evaluate(dynamics, minus, control)) /
      (2.0 * epsilon);
  }

  model::MpcControlJacobian expected_u;
  for (int i = 0; i < model::kMpcControlDim; ++i) {
    auto plus = control;
    auto minus = control;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    expected_u.col(i) = (evaluate(dynamics, state, plus) - evaluate(dynamics, state, minus)) /
      (2.0 * epsilon);
  }

  grampc::Vector x = state;
  grampc::Vector u = control;
  grampc::Vector p(0);
  grampc::Vector vec = adjoint;
  grampc::Vector actual_x(model::kMpcStateDim);
  grampc::Vector actual_u(model::kMpcControlDim);
  grampc::GrampcParam parameters;
  dynamics.dfdx_vec(actual_x, 0.0, x, u, p, vec, parameters);
  dynamics.dfdu_vec(actual_u, 0.0, x, u, p, vec, parameters);

  const double state_error = (actual_x - expected_x.transpose() * adjoint).norm();
  const double state_scale = std::max(1.0, (expected_x.transpose() * adjoint).norm());
  const double control_error = (actual_u - expected_u.transpose() * adjoint).norm();
  const double control_scale = std::max(1.0, (expected_u.transpose() * adjoint).norm());
  return check(state_error / state_scale < 1.0e-5, "dfdx_vec equals J^T vec (relative error=" +
      std::to_string(state_error / state_scale) + ")") &&
         check(control_error / control_scale < 1.0e-5, "dfdu_vec equals J^T vec (relative error=" +
      std::to_string(control_error / control_scale) + ")");
}

}  // namespace

int main()
{
#ifndef ASR_SDM_GENERATED_URDF
  std::cerr << "FAILED: generated URDF path is not configured" << std::endl;
  return 1;
#else
  bool passed = true;
  const auto parameters = makeParameters();

  model::UnderwaterSimulator simulator(parameters);
  passed &= check(simulator.isValid(), "simulator construction");
  if (!simulator.isValid()) {
    return 1;
  }
  const auto initial_state = simulator.makeInitialState();
  const auto joint_mapping = simulator.pinocchioModel().controllerDofMapping();
  for (std::size_t i = 0; i < parameters.pinocchio.initial_joint_positions.size(); ++i) {
    passed &= check(
      std::abs(initial_state.configuration(joint_mapping[i].q_index) -
        parameters.pinocchio.initial_joint_positions[i]) < 1.0e-12,
      "configured initial joint position is retained");
  }

  model::GrampcUnderwaterDynamics dynamics(parameters);
  passed &= check(dynamics.isValid(), "default MPC interface construction");
  if (!dynamics.isValid()) {
    std::cerr << dynamics.error() << std::endl;
    return 1;
  }
  passed &= check(
    dynamics.referenceConfiguration().isApprox(initial_state.configuration),
    "default MPC reference configuration uses configured initial state");

  model::MpcState state = model::MpcState::Zero();
  Eigen::Matrix<double, model::kMpcControlDim, 1> control =
    Eigen::Matrix<double, model::kMpcControlDim, 1>::Zero();
  control(0) = 0.2;
  const model::MpcState derivative = evaluate(dynamics, state, control);
  passed &= check(derivative.allFinite(), "ffct returns a finite full-state derivative");
  passed &= check(
    derivative.head<model::kMpcConfigurationTangentDim>().isApprox(
      state.tail<model::kReducedNv>(), 1.0e-12),
    "zero tangent displacement has q-error derivative equal to velocity");

  model::MpcState displaced = state;
  displaced(6) = 0.2;
  const auto displaced_derivative = evaluate(dynamics, displaced, control);
  passed &= check(
    (displaced_derivative.tail<model::kReducedNv>() - derivative.tail<model::kReducedNv>()).norm() >
      1.0e-10,
    "tangent configuration error changes predicted dynamics");

  passed &= checkTransposeJacobians(dynamics);

  bool invalid_reference_rejected = false;
  auto invalid_reference = dynamics.referenceConfiguration();
  invalid_reference.segment<4>(3).setZero();
  try {
    dynamics.updateReferenceConfiguration(invalid_reference);
  } catch (const std::invalid_argument &) {
    invalid_reference_rejected = true;
  }
  passed &= check(invalid_reference_rejected, "invalid reference quaternion is rejected");

  auto invalid_dimensions = model::GrampcDimensionConfig{};
  invalid_dimensions.nx = model::kMpcStateDim - 1;
  model::GrampcUnderwaterDynamics invalid_dynamics(parameters, invalid_dimensions);
  passed &= check(!invalid_dynamics.isValid(), "unsupported dimensions are rejected");

  try {
    auto problem = std::make_shared<model::GrampcUnderwaterDynamics>(parameters);
    grampc::Grampc solver(problem);
    std::vector<typeRNum> x0(model::kMpcStateDim, 0.0);
    std::vector<typeRNum> xdes(model::kMpcStateDim, 0.0);
    std::vector<typeRNum> u0(model::kMpcControlDim, 0.0);
    std::vector<typeRNum> umin(model::kMpcControlDim, -10.0);
    std::vector<typeRNum> umax(model::kMpcControlDim, 10.0);
    for (std::size_t i = 0; i < model::kNumLinks; ++i) {
      umin[i] = 0.0;
      umax[i] = 1.0;
    }
    solver.setparam_real("Thor", 0.002);
    solver.setparam_real("Tmax", 0.01);
    solver.setparam_real("Tmin", 0.0001);
    solver.setparam_real("dt", 0.0001);
    solver.setparam_real("t0", 0.0);
    solver.setparam_real_vector("x0", x0.data());
    solver.setparam_real_vector("xdes", xdes.data());
    solver.setparam_real_vector("u0", u0.data());
    solver.setparam_real_vector("udes", u0.data());
    solver.setparam_real_vector("umin", umin.data());
    solver.setparam_real_vector("umax", umax.data());
    solver.setopt_int("Nhor", 20);
    // Nhor reallocation resets the internal time grid; set Thor once more afterwards.
    solver.setparam_real("Thor", 0.002);
    solver.setopt_int("MaxGradIter", 1);
    solver.setopt_int("MaxMultIter", 1);
    solver.setopt_string("Integrator", "erk2");
    passed &= check(
      solver.getParameters()->Nx == model::kMpcStateDim &&
      solver.getParameters()->Nu == model::kMpcControlDim,
      "GRAMPC solver accepts the full-state problem dimensions");
  } catch (const std::exception & error) {
    std::cerr << "FAILED: GRAMPC construction/configuration threw: " << error.what() << std::endl;
    passed = false;
  }

  std::cout <<
    (passed ? "GRAMPC interface regression passed" : "GRAMPC interface regression failed")
            << std::endl;
  return passed ? 0 : 1;
#endif
}

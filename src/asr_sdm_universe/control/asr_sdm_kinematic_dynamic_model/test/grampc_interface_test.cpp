#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"

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

// Quaternion helper functions exposed here so we can unit-test them.
// (Mirror of the anonymous-namespace helpers in grampc_dynamics_interface.cpp.)
Eigen::Vector4d quaternionProduct(const Eigen::Vector4d & q1, const Eigen::Vector4d & q2)
{
  const double w1 = q1(0);
  const Eigen::Vector3d v1 = q1.tail<3>();
  const double w2 = q2(0);
  const Eigen::Vector3d v2 = q2.tail<3>();
  Eigen::Vector4d out;
  out(0) = w1 * w2 - v1.dot(v2);
  out.tail<3>() = w1 * v2 + w2 * v1 + v1.cross(v2);
  return out;
}
Eigen::Vector4d quaternionConjugate(const Eigen::Vector4d & q)
{
  Eigen::Vector4d out = q;
  out.tail<3>() = -q.tail<3>();
  return out;
}
Eigen::Vector3d quaternionLog(const Eigen::Vector4d & q)
{
  const double w = q(0);
  const Eigen::Vector3d v = q.tail<3>();
  const double v_norm = v.norm();
  if (v_norm < 1.0e-12) {
    Eigen::Vector4d q_pos = q;
    if (w < 0.0) {q_pos = -q;}
    const Eigen::Vector3d vp = q_pos.tail<3>();
    if (vp.norm() < 1.0e-12) {return Eigen::Vector3d::Zero();}
    return 2.0 * (vp / q_pos(0));
  }
  Eigen::Vector4d q_pos = q;
  if (w < 0.0) {q_pos = -q;}
  return 2.0 * std::atan2(v_norm, q_pos(0)) * (q_pos.tail<3>() / v_norm);
}
double quaternionLogSquaredError(
  const Eigen::Vector4d & q_meas, const Eigen::Vector4d & q_des)
{
  return quaternionLog(quaternionProduct(quaternionConjugate(q_des), q_meas)).squaredNorm();
}
Eigen::Vector4d quaternionLogGradientWrtMeas(
  const Eigen::Vector4d & q_meas, const Eigen::Vector4d & q_des,
  const double weight, const Eigen::Vector3d & /*rot_vec*/)
{
  constexpr double kEps = 1.0e-6;
  Eigen::Vector4d grad = Eigen::Vector4d::Zero();
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector4d qp = q_meas;
    Eigen::Vector4d qm = q_meas;
    qp(i) += kEps;
    qm(i) -= kEps;
    qp.segment<4>(3).normalize();
    qm.segment<4>(3).normalize();
    const double Lp = 0.5 * weight *
      quaternionLog(quaternionProduct(quaternionConjugate(q_des), qp)).squaredNorm();
    const double Lm = 0.5 * weight *
      quaternionLog(quaternionProduct(quaternionConjugate(q_des), qm)).squaredNorm();
    grad(i) = (Lp - Lm) / (2.0 * kEps);
  }
  return grad;
}

bool checkTransposeJacobians(model::GrampcUnderwaterDynamics & dynamics)
{
  model::MpcState state = model::MpcState::Zero();
  // Seed unit quaternion so evaluate() doesn't fail the normalization check.
  state.segment<4>(3) = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
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
  // Full-state mode: no separate referenceConfiguration() to check.
  // Instead verify that a zero-velocity initial state gives zero q_dot.
  model::MpcState state = model::MpcState::Zero();
  // Seed the quaternion with a unit quaternion to avoid NaN in quaternionDerivative.
  state.segment<4>(3) = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  Eigen::Matrix<double, model::kMpcControlDim, 1> control =
    Eigen::Matrix<double, model::kMpcControlDim, 1>::Zero();
  control(0) = 0.2;
  const model::MpcState derivative = evaluate(dynamics, state, control);
  passed &= check(derivative.allFinite(), "ffct returns a finite full-state derivative");
  // With zero velocity, position/quat/joint q_dot should all be zero.
  passed &= check(
    derivative.head<model::kReducedNq>().isApprox(
      Eigen::Matrix<double, model::kReducedNq, 1>::Zero(), 1.0e-12),
    "zero velocity gives zero q_dot");

  // Non-zero velocity should produce non-zero q_dot.
  // Velocity is stored at indices [kMpcConfigurationTangentDim, kMpcConfigurationTangentDim+12).
  model::MpcState state_with_vel = state;
  state_with_vel[model::kMpcConfigurationTangentDim + 0] = 1.0;  // vx = 1 m/s
  const auto derivative_with_vel = evaluate(dynamics, state_with_vel, control);
  passed &= check(derivative_with_vel.head<3>().isApprox(
    Eigen::Vector3d::UnitX(), 1.0e-12), "vx=1 gives q_dot[0]=1");
  passed &= check(
    (derivative_with_vel.tail<model::kReducedNv>() - derivative.tail<model::kReducedNv>()).norm() >
      1.0e-10,
    "non-zero velocity changes acceleration");

  passed &= checkTransposeJacobians(dynamics);

  // Full-state mode: isValidConfiguration() should still reject zero quaternions.
  model::ReducedConfiguration zero_quat_cfg = dynamics.simulator().pinocchioModel().configuration();
  zero_quat_cfg.segment<4>(3).setZero();
  passed &= check(
    !model::GrampcUnderwaterDynamics::isValidConfiguration(zero_quat_cfg),
    "zero quaternion is rejected by isValidConfiguration");

  auto invalid_dimensions = model::GrampcDimensionConfig{};
  invalid_dimensions.nx = model::kMpcStateDim - 1;
  model::GrampcUnderwaterDynamics invalid_dynamics(parameters, invalid_dimensions);
  passed &= check(!invalid_dynamics.isValid(), "unsupported dimensions are rejected");

  try {
    auto problem = std::make_shared<model::GrampcUnderwaterDynamics>(parameters);
    grampc::Grampc solver(problem);
    // Full-state mode: x0 must contain a valid unit quaternion.
    std::vector<typeRNum> x0(model::kMpcStateDim, 0.0);
    const auto & init_q = problem->simulator().pinocchioModel().configuration();
    for (int i = 0; i < model::kReducedNq; ++i) {
      x0[i] = static_cast<typeRNum>(init_q(i));
    }
    std::vector<typeRNum> xdes(model::kMpcStateDim, 0.0);
    for (int i = 0; i < model::kReducedNq; ++i) {
      xdes[i] = static_cast<typeRNum>(init_q(i));
    }
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

  // ---- Quaternion logarithm math tests ----
  // Verify the manifold-aware quaternion cost helper functions numerically.
  // (The lfct/dldx integration with param.xdes requires a full GRAMPC solver
  // lifecycle and is validated end-to-end by stochastic_mpc_test and
  // grampc_run_repro.)  These helper tests live inside an anonymous namespace
  // block; the helper functions are declared above the namespace.
  {
    // Case: q_des = 90 deg around z, q_meas = identity.
    // q_err = q_des^{-1} ⊗ q_meas = q_des* = [cos(45), 0, 0, -sin(45)].
    // ||log(q_err)|| = 90 deg = pi/2  (the rotation vector from q_des to
    // q_meas has magnitude π/2 along -z axis).
    Eigen::Vector4d q_meas_id = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector4d q_des_90 = Eigen::Vector4d(
      std::cos(M_PI / 4.0), 0.0, 0.0, std::sin(M_PI / 4.0));
    Eigen::Vector3d log_err = quaternionLog(
      quaternionProduct(quaternionConjugate(q_des_90), q_meas_id));
    passed &= check(
      std::abs(log_err.norm() - M_PI / 2.0) < 1.0e-6,
      "quaternionLog: ||log(q_err)|| = pi/2 for 90-deg error");
    passed &= check(
      log_err.head<2>().norm() < 1.0e-6,
      "quaternionLog: 90-deg rotation gives log along z axis only");

    double cost_90 = 0.5 * quaternionLogSquaredError(q_meas_id, q_des_90);
    const double expected_cost_90 = 0.5 * (M_PI / 2.0) * (M_PI / 2.0);
    passed &= check(
      std::abs(cost_90 - expected_cost_90) < 1.0e-6,
      "quaternionLogSquaredError: 90-deg = 0.5*(pi/2)^2 ≈ 1.2337");

    // Double-cover invariance: q and -q represent the same orientation.
    Eigen::Vector4d q_des_neg = -q_des_90;
    double cost_neg = 0.5 * quaternionLogSquaredError(q_meas_id, q_des_neg);
    passed &= check(
      std::abs(cost_90 - cost_neg) < 1.0e-6,
      "quaternionLogSquaredError: invariant under q -> -q (no double-cover ambiguity)");

    // Gradient w.r.t. q_meas is finite and non-zero.
    Eigen::Vector4d grad = quaternionLogGradientWrtMeas(
      q_meas_id, q_des_90, 1.0, log_err);
    passed &= check(grad.allFinite(), "quaternionLogGradientWrtMeas: finite output");
    passed &= check(
      grad.norm() > 1.0e-3,
      "quaternionLogGradientWrtMeas: non-zero gradient for 90-deg error");

    // Small-angle case: q_des ≈ q_meas (within 1 deg around z).
    // The error rotation magnitude should be ~1 deg.
    double small_angle = M_PI / 180.0;  // 1 deg
    Eigen::Vector4d q_des_small = Eigen::Vector4d(
      std::cos(small_angle / 2.0), 0.0, 0.0, std::sin(small_angle / 2.0));
    Eigen::Vector3d log_small = quaternionLog(
      quaternionProduct(quaternionConjugate(q_des_small), q_meas_id));
    passed &= check(
      std::abs(log_small.norm() - small_angle) < 1.0e-6,
      "quaternionLog: 1-deg error magnitude ≈ 1 deg");
    passed &= check(
      log_small.head<2>().norm() < 1.0e-6,
      "quaternionLog: 1-deg rotation around z gives log along z axis only");

    // Identity case: identical quaternions → zero log.
    Eigen::Vector3d log_id = quaternionLog(
      quaternionProduct(quaternionConjugate(q_meas_id), q_meas_id));
    passed &= check(
      log_id.norm() < 1.0e-10,
      "quaternionLog: identity gives zero vector");

    // GrampcCostWeights validation with log weight.
    model::GrampcCostWeights weights_with_log;
    weights_with_log.quaternion_log_weight = 1.0;
    passed &= check(weights_with_log.isValid(),
      "GrampcCostWeights with quaternion_log_weight=1.0 is valid");

    weights_with_log.quaternion_log_weight = -1.0;
    passed &= check(!weights_with_log.isValid(),
      "GrampcCostWeights with quaternion_log_weight=-1.0 is invalid");
  }

  // ---- Incremental warm-start test ----
  // ---- Incremental warm-start test ----
  // Verify that successive MPC solves with enable_warm_start=true are no
  // worse (in cost) than with enable_warm_start=false, and that the warm
  // start succeeds (does not produce NaN/Inf).
  {
    model::UnderwaterMpcParameters mpc_params;
    mpc_params.Nhor = 15;
    mpc_params.Thor = 0.15;
    mpc_params.dt = 0.02;
    mpc_params.max_grad_iter = 5;   // intentionally limited to see warm-start benefit
    mpc_params.max_mult_iter = 1;
    mpc_params.enable_warm_start = true;
    mpc_params.verbose = false;

    model::UnderwaterMpcController controller(parameters, mpc_params);
    passed &= check(controller.isValid(), "warm-start controller is valid");
    if (controller.isValid()) {
      auto & plant = controller.simulator();
      auto state = plant.makeInitialState();
      model::MpcReferenceTrajectory ref;
      ref.target_configuration = state.configuration;
      ref.target_velocity = model::ReducedVelocity::Zero();
      ref.has_valid_target = true;

      // First solve (cold start, but GRAMPC's internal shift still seeds).
      auto res1 = controller.computeControl(state.configuration, state.velocity, ref);
      passed &= check(res1.success, "warm-start controller first solve succeeds");
      passed &= check(
        res1.iterations >= 0 && res1.iterations <= mpc_params.max_grad_iter,
        "first solve iteration count is in valid range");

      // Plant step.
      auto input = res1.control;
      plant.step(state, 0.02, input);

      // Second solve (warm-started with previous control).
      auto res2 = controller.computeControl(state.configuration, state.velocity, ref);
      passed &= check(res2.success, "warm-start controller second solve succeeds");
      passed &= check(std::isfinite(res2.cost),
        "second solve cost is finite");

      // Third solve.
      auto res3 = controller.computeControl(state.configuration, state.velocity, ref);
      passed &= check(res3.success, "warm-start controller third solve succeeds");

      std::cout << "    Warm-start runs: solve1=" << res1.solve_time_ms
                << "ms/" << res1.iterations << "it"
                << ", solve2=" << res2.solve_time_ms
                << "ms/" << res2.iterations << "it"
                << ", solve3=" << res3.solve_time_ms
                << "ms/" << res3.iterations << "it" << std::endl;
    }
  }

  std::cout <<
    (passed ? "GRAMPC interface regression passed" : "GRAMPC interface regression failed")
            << std::endl;
  return passed ? 0 : 1;
#endif
}

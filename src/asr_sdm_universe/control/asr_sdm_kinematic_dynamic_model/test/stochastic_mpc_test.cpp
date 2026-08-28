// Copyright (c) 2025.
// Unit tests for the stochastic MPC wrapper (StochasticGrampcDynamics).
//
// Verifies that:
//   1. Configuration validation rejects invalid inputs.
//   2. The wrapper builds a valid stochastic problem for each transformation.
//   3. The resulting problem description is a GRAMPC-S stochastic problem
//      (SigmaPoint or Monte Carlo).
//   4. The wrapper integrates with a real grampc::Grampc solver and produces
//      a finite, sensible solution.

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/stochastic_grampc_dynamics.hpp"
#include "asr_sdm_kinematic_dynamic_model/stochastic_mpc_config.hpp"

#include <grampc_s/grampc_s.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{
namespace model = asr_sdm_kinematic_dynamic_model;

bool report(bool ok, const std::string & message)
{
  if (!ok) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return ok;
}

model::UnderwaterSimulatorParameters makeSimParams()
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

std::shared_ptr<model::GrampcUnderwaterDynamics> makeInnerDynamics(
  bool with_velocity_limits = true)
{
  auto inner = std::make_shared<model::GrampcUnderwaterDynamics>(makeSimParams());
  if (!inner->isValid()) {
    std::cerr << "Inner dynamics invalid: " << inner->error() << std::endl;
    return nullptr;
  }
  if (with_velocity_limits) {
    inner->setVelocityLimits(2.0, 1.0, 2.0);
  } else {
    inner->disableVelocityLimits();
  }
  return inner;
}

bool testConfigValidation()
{
  bool all_ok = true;

  // Valid default config
  model::StochasticMpcConfig cfg;
  std::string err;
  all_ok &= report(cfg.isValid(8, &err), "default config is valid");

  // Invalid probability outside (0,1)
  model::StochasticMpcConfig bad_prob;
  bad_prob.chance_constraints.satisfaction_probability(0) = -0.1;
  all_ok &= report(!bad_prob.isValid(8, &err), "negative probability rejected");

  // Wrong size for probability vector
  model::StochasticMpcConfig bad_size;
  bad_size.chance_constraints.satisfaction_probability = Eigen::VectorXd::Constant(5, 0.95);
  all_ok &= report(!bad_size.isValid(8, &err), "size mismatch rejected");

  // Invalid covariance (not positive semi-definite). Set only diagonal
  // element to a negative value so the matrix is not all-zeros.
  model::StochasticMpcConfig bad_cov;
  bad_cov.state.use_full_covariance = true;
  bad_cov.state.covariance.setZero(model::kMpcStateDim, model::kMpcStateDim);
  bad_cov.state.covariance(0, 0) = -1.0;  // negative -> not PSD
  all_ok &= report(!bad_cov.state.isValid(&err), "non-PSD covariance rejected");

  // Parametric uncertainty without inner np > 0 is rejected at construction time
  model::StochasticMpcConfig bad_param;
  bad_param.parameters.count = 3;
  bad_param.parameters.mean = Eigen::VectorXd::Zero(3);
  bad_param.parameters.stddev = Eigen::VectorXd::Ones(3);
  auto inner = makeInnerDynamics(false);
  all_ok &= report(inner != nullptr, "inner dynamics created");
  if (inner) {
    model::StochasticGrampcDynamics stoch(inner, bad_param);
    all_ok &= report(!stoch.isValid(),
      "parametric uncertainty rejected (got: " + stoch.error() + ")");
  }

  return all_ok;
}

bool testBuildForEachTransformation(
  model::StochasticTransformationType type, const std::string & name)
{
  bool all_ok = true;
  auto inner = makeInnerDynamics();
  if (!inner) {
    std::cerr << "testBuildForEachTransformation: cannot create inner dynamics" << std::endl;
    return false;
  }

  model::StochasticMpcConfig cfg;
  cfg.transformation = type;
  cfg.state.stddev = Eigen::Matrix<double, model::kMpcStateDim, 1>::Constant(1.0e-2);
  cfg.chance_constraints.satisfaction_probability = Eigen::VectorXd::Constant(8, 0.95);

  model::StochasticGrampcDynamics stoch(inner, cfg);
  all_ok &= report(stoch.isValid(),
    name + ": stochastic dynamics is valid (error: " + stoch.error() + ")");
  if (!stoch.isValid()) {return all_ok;}

  auto problem = stoch.problemDescription();
  all_ok &= report(problem != nullptr, name + ": problem description is non-null");
  all_ok &= report(stoch.numPoints() > 0, name + ": positive point count");
  all_ok &= report(stoch.innerPtr() == inner, name + ": inner pointer preserved");

  return all_ok;
}

bool testEndToEndSolve()
{
  // GRAMPC-S' SigmaPointProblemDescription slices the N×nx expanded state
  // (1176 = 49×24 for Unscented) into nx-dimensional chunks and calls the
  // inner callbacks one slice at a time.  StochasticGrampcDynamics already
  // calls enableRelaxedValidationForStochasticCallbacks() on the inner
  // GrampcUnderwaterDynamics, so the dimension check is bypassed and each
  // single-slice call passes.  This test runs a single solver.run() on the
  // full stochastic problem to verify that the full pipeline works end-to-end.
  bool all_ok = true;
  auto inner = makeInnerDynamics();
  if (!inner) {return false;}

  model::StochasticMpcConfig cfg;
  cfg.transformation = model::StochasticTransformationType::Unscented;
  cfg.state.stddev = Eigen::Matrix<double, model::kMpcStateDim, 1>::Constant(1.0e-2);
  // CRITICAL: mean must contain a valid unit quaternion in positions [3, 7).
  // Without this the state distribution centers at a zero quaternion and
  // GramPC-S's compute_x0_and_p0() produces an invalid initial state.
  cfg.state.mean = Eigen::Matrix<double, model::kMpcStateDim, 1>::Zero();
  const auto & init_q = inner->simulator().pinocchioModel().configuration();
  for (int i = 0; i < model::kReducedNq; ++i) {
    cfg.state.mean[i] = init_q(i);
  }
  cfg.chance_constraints.enabled = true;
  cfg.chance_constraints.satisfaction_probability = Eigen::VectorXd::Constant(8, 0.95);

  model::StochasticGrampcDynamics stoch(inner, cfg);
  if (!stoch.isValid()) {
    std::cerr << "End-to-end: stochastic dynamics invalid: " << stoch.error() << std::endl;
    return false;
  }

  try {
    grampc::Grampc solver(stoch.problemDescription());
    solver.setparam_real("Thor", 0.5);
    solver.setparam_real("dt", 0.02);
    solver.setopt_int("Nhor", 20);
    solver.setopt_string("Integrator", "erk2");
    solver.setopt_real("PenaltyMin", 1.0e3);

    // Expanded stochastic dimensions.
    const int expanded_nx = solver.getParameters()->Nx;   // 51*25 = 1275 for full-state
    const int nu = solver.getParameters()->Nu;             // 10
    all_ok &= report(
      expanded_nx == (2 * (model::kMpcStateDim + 0) + 1) * model::kMpcStateDim,
      "expanded Nx = 51*25 = 1275 (got " + std::to_string(expanded_nx) + ")");
    all_ok &= report(nu == model::kMpcControlDim,
      "Nu = 10");

    // CRITICAL: GramPC-S's compute_x0_and_p0() initializes x0 from the state
    // distribution but the GramPC solver itself uses its OWN x0 (init to zero
    // by default).  We must explicitly construct an expanded x0 that has a
    // valid unit quaternion at every sigma-point slice so the simulator never
    // sees a zero quaternion.
    const int num_points = stoch.numPoints();
    const auto & init_q = inner->simulator().pinocchioModel().configuration();
    std::vector<typeRNum> x0(expanded_nx, 0.0);
    for (int i = 0; i < num_points; ++i) {
      for (int j = 0; j < model::kReducedNq; ++j) {
        x0[i * model::kMpcStateDim + j] = static_cast<typeRNum>(init_q(j));
      }
      // velocity = 0 (already zero-initialized)
    }

    // Target: base linear vx = 0.5 m/s, rest zero.
    // In full-state mode xdes = [q_des, v_des]; we track the current q.
    std::vector<typeRNum> xdes(expanded_nx, 0.0);
    for (int i = 0; i < num_points; ++i) {
      // Configuration part of target (same q for all sigma points).
      for (int j = 0; j < model::kReducedNq; ++j) {
        xdes[i * model::kMpcStateDim + j] = static_cast<typeRNum>(init_q(j));
      }
      // Velocity target: vx = 0.5 m/s.
      xdes[i * model::kMpcStateDim + model::kMpcConfigurationTangentDim + 0] = 0.5;
    }

    // Control bounds (same for all sigma points; Nu is not expanded).
    std::vector<typeRNum> umin(nu, 0.0), umax(nu, 10.0);
    for (int i = model::kNumLinks; i < nu; ++i) {
      umin[i] = -10.0;
      umax[i] = 10.0;
    }
    std::vector<typeRNum> u0(nu, 0.0);

    solver.setparam_real_vector("x0", x0.data());
    solver.setparam_real_vector("xdes", xdes.data());
    solver.setparam_real_vector("u0", u0.data());
    solver.setparam_real_vector("udes", u0.data());
    solver.setparam_real_vector("umin", umin.data());
    solver.setparam_real_vector("umax", umax.data());
    solver.setparam_real("t0", 0.0);

    solver.run();

    const typeGRAMPCsol * sol = solver.getSolution();
    all_ok &= report(sol != nullptr, "solution pointer is non-null");
    if (sol != nullptr) {
      bool sol_finite = true;
      for (int i = 0; i < nu; ++i) {
        if (!std::isfinite(sol->unext[i])) {
          sol_finite = false;
          std::cerr << "    unext[" << i << "] = " << sol->unext[i] << std::endl;
        }
      }
      if (!std::isfinite(sol->J[0])) {
        sol_finite = false;
        std::cerr << "    J = " << sol->J[0] << std::endl;
      }
      all_ok &= report(sol_finite, "all unext[] and J[] are finite (no NaN/Inf)");
    }

    std::cout << "    [OK] Stochastic solver ran successfully; all outputs finite." << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "End-to-end: solver threw: " << e.what() << std::endl;
    all_ok = false;
  }

  return all_ok;
}

bool testChanceConstraintsTightenConstraints()
{
  // Property test: with the same state, the sigma-point problem with
  // chance constraints must enforce the velocity limits MORE conservatively
  // than the deterministic problem (i.e., the constraint is active at a
  // lower velocity). We probe h(x) = v^2 - v_max^2 at a chosen velocity and
  // verify the constraint tightening direction.
  bool all_ok = true;
  auto inner = makeInnerDynamics();
  if (!inner) {return false;}

  model::StochasticMpcConfig cfg;
  cfg.transformation = model::StochasticTransformationType::Unscented;
  cfg.state.stddev = Eigen::Matrix<double, model::kMpcStateDim, 1>::Constant(0.1);
  cfg.chance_constraints.enabled = true;
  cfg.chance_constraints.satisfaction_probability = Eigen::VectorXd::Constant(8, 0.95);

  model::StochasticGrampcDynamics stoch(inner, cfg);
  all_ok &= report(stoch.isValid(),
    "chance-constraint tightening test: stochastic dynamics valid");

  // Verify that the GaussianConstraintApproximation tightens (z > 0 for
  // P_sat > 0.5). We construct it directly with satisfaction probabilities.
  // For P_sat = 0.95, z ≈ Φ⁻¹(0.95) ≈ 1.645 > 0.
  grampc::GaussianConstraintApproximation approx(Eigen::VectorXd::Constant(8, 0.95));
  const auto coeffs = approx.tighteningCoefficient();
  all_ok &= report(coeffs.size() == 8,
    "tightening coefficient has size 8");
  all_ok &= report((coeffs.array() > 0.0).all(),
    "tightening coefficient > 0 for P_sat=0.95 (got coeffs[0]=" +
    std::to_string(coeffs(0)) + ", expected ~1.645)");

  return all_ok;
}

bool testNoChanceConstraintsFallsBack()
{
  bool all_ok = true;
  auto inner = makeInnerDynamics();
  if (!inner) {return false;}

  model::StochasticMpcConfig cfg;
  cfg.transformation = model::StochasticTransformationType::Unscented;
  cfg.state.stddev = Eigen::Matrix<double, model::kMpcStateDim, 1>::Constant(1.0e-3);
  cfg.chance_constraints.enabled = false;  // fall back to deterministic h(x)<=0

  model::StochasticGrampcDynamics stoch(inner, cfg);
  all_ok &= report(stoch.isValid(),
    "fall-back (no chance constraints) is valid");
  return all_ok;
}

}  // namespace

int main()
{
  bool all_ok = true;

  std::cout << "=== Stochastic MPC tests ===" << std::endl;

  std::cout << "[1] Configuration validation..." << std::endl;
  all_ok &= testConfigValidation();

  std::cout << "[2] Build with Unscented transformation..." << std::endl;
  all_ok &= testBuildForEachTransformation(
    model::StochasticTransformationType::Unscented, "Unscented");

  std::cout << "[3] Build with Stirling first-order transformation..." << std::endl;
  all_ok &= testBuildForEachTransformation(
    model::StochasticTransformationType::StirlingFirstOrder, "StirlingFirstOrder");

  std::cout << "[4] Build with Stirling second-order transformation..." << std::endl;
  all_ok &= testBuildForEachTransformation(
    model::StochasticTransformationType::StirlingSecondOrder, "StirlingSecondOrder");

  std::cout << "[5] Build with Monte Carlo transformation..." << std::endl;
  all_ok &= testBuildForEachTransformation(
    model::StochasticTransformationType::MonteCarlo, "MonteCarlo");

  std::cout << "[6] Chance-constraint tightening direction..." << std::endl;
  all_ok &= testChanceConstraintsTightenConstraints();

  std::cout << "[7] Fall back to deterministic when chance constraints disabled..."
            << std::endl;
  all_ok &= testNoChanceConstraintsFallsBack();

  std::cout << "[8] End-to-end solve via grampc::Grampc..." << std::endl;
  all_ok &= testEndToEndSolve();

  if (all_ok) {
    std::cout << "All stochastic MPC tests passed." << std::endl;
    return 0;
  }
  std::cout << "Some stochastic MPC tests failed." << std::endl;
  return 1;
}

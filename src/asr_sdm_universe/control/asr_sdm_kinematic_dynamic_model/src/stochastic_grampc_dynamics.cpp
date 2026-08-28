// Copyright (c) 2025.
// Implementation of StochasticGrampcDynamics - the bridge between
// GrampcUnderwaterDynamics and GRAMPC-S' stochastic extensions.

#include "asr_sdm_kinematic_dynamic_model/stochastic_grampc_dynamics.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <sstream>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{

namespace
{

/// Build a `Distribution` from a `StateUncertaintyConfig`.
/// Supports both diagonal (uncorrelated) and full covariance cases.
grampc::DistributionPtr makeStateDistribution(const StateUncertaintyConfig & cfg)
{
  if (cfg.hasFullCovariance()) {
    // Gaussian with full covariance - correlation between channels is preserved.
    return std::make_shared<grampc::GaussianDistribution>(cfg.mean, cfg.covariance);
  }
  // Build a multivariate uncorrelated distribution component-by-component.
  // This is more efficient than a full covariance matrix when the noise is
  // actually independent across axes (e.g. independent IMU / DVL noise).
  std::vector<grampc::DistributionPtr> components;
  components.reserve(kMpcStateDim);
  for (int i = 0; i < kMpcStateDim; ++i) {
    const double var = cfg.stddev(i) * cfg.stddev(i);
    components.push_back(std::make_shared<grampc::GaussianDistribution>(cfg.mean(i), var));
  }
  return std::make_shared<grampc::MultivariateDistribution>(components);
}

/// Build a parameter distribution from `ParametricUncertaintyConfig`.
/// Currently unused because `GrampcUnderwaterDynamics` has np=0 (the inner
/// problem exposes no parameters). The function is kept in the translation
/// unit so it remains in sync with the configuration type when the inner
/// dynamics is extended.
[[maybe_unused]] static grampc::DistributionPtr makeParameterDistribution(
  const ParametricUncertaintyConfig & cfg)
{
  if (cfg.count <= 0) {
    return nullptr;
  }
  std::vector<grampc::DistributionPtr> components;
  components.reserve(cfg.count);
  for (int i = 0; i < cfg.count; ++i) {
    const double var = cfg.stddev(i) * cfg.stddev(i);
    components.push_back(std::make_shared<grampc::GaussianDistribution>(cfg.mean(i), var));
  }
  return std::make_shared<grampc::MultivariateDistribution>(components);
}

/// Build a Gaussian chance-constraint approximation from a satisfaction-probability
/// vector. Each element `P_sat_i` must be in (0, 1).
grampc::ChanceConstraintApproximationPtr makeChanceApproximation(
  const Eigen::VectorXd & satisfaction_probabilities)
{
  return std::make_shared<grampc::GaussianConstraintApproximation>(satisfaction_probabilities);
}

}  // namespace

StochasticGrampcDynamics::StochasticGrampcDynamics(
  std::shared_ptr<GrampcUnderwaterDynamics> inner,
  const StochasticMpcConfig & config)
: inner_(std::move(inner)), config_(config)
{
  if (!inner_) {
    error_ = "StochasticGrampcDynamics: inner GrampcUnderwaterDynamics is null";
    return;
  }
  if (!inner_->isValid()) {
    error_ = "StochasticGrampcDynamics: inner dynamics is invalid: " + inner_->error();
    return;
  }

  const int expected_nh = inner_->hasVelocityLimits() ? 8 : 0;
  std::string config_error;
  if (!config_.isValid(expected_nh, &config_error)) {
    error_ = "StochasticGrampcDynamics: " + config_error;
    return;
  }

  // GRAMPC-S slices the N×nx expanded state into nx-dimensional chunks and
  // calls each inner callback with one chunk.  This means the inner callbacks
  // always receive canonical nx/n u dimensions, but the total x vector is
  // N×nx long and would fail the dimension check in validateStateAndControl.
  // Tell the inner dynamics to skip that check so the SigmaPoint / Monte Carlo
  // wrappers can call it with single-slice input.
  inner_->enableRelaxedValidationForStochasticCallbacks();

  // Parametric uncertainty requires np > 0 in the inner problem. We surface a
  // helpful error if the user requested parametric uncertainty on a model
  // that doesn't expose tunable parameters (current underwater model).
  if (config_.parameters.count > 0) {
    error_ = "StochasticGrampcDynamics: parametric uncertainty requested "
      "(count=" + std::to_string(config_.parameters.count) + ") but the current "
      "GrampcUnderwaterDynamics has np=0. To use parametric uncertainty, "
      "extend GrampcDimensionConfig.np and override dfdp_vec in the inner "
      "dynamics. State-only uncertainty is fully supported today.";
    return;
  }

  switch (config_.transformation) {
    case StochasticTransformationType::Unscented:
    case StochasticTransformationType::StirlingFirstOrder:
    case StochasticTransformationType::StirlingSecondOrder:
      buildSigmaPointProblem();
      break;
    case StochasticTransformationType::MonteCarlo:
      buildMonteCarloProblem();
      break;
    default:
      error_ = "StochasticGrampcDynamics: unknown transformation type";
      return;
  }

  if (!stochastic_problem_) {
    if (error_.empty()) {
      error_ = "StochasticGrampcDynamics: failed to build stochastic problem description";
    }
    return;
  }

  if (config_.verbose) {
    std::ostringstream stream;
    stream << "[StochasticGrampcDynamics] Built stochastic problem with "
           << numPoints() << " points; "
           << (config_.chance_constraints.enabled ? "chance constraints on" :
    "chance constraints off");
    // std::cout is the simplest logging channel; production code should use ROS_LOG.
    std::cout << stream.str() << std::endl;
  }
}

void StochasticGrampcDynamics::buildSigmaPointProblem()
{
  // Build the state distribution. GRAMPC-S' SigmaPointProblemDescription also
  // supports parameter distributions but we currently only use state
  // uncertainty because the inner problem has np=0.
  grampc::DistributionPtr state_dist = makeStateDistribution(config_.state);

  // Choose the point transformation.
  grampc::PointTransformationPtr transformation;
  switch (config_.transformation) {
    case StochasticTransformationType::Unscented:
      // nx=state_dim, ny=nx (SigmaPointProblemDescription does NOT use ny
      // directly for the state-only case; we pass nu=0 to satisfy the
      // constructor of the point transformation, which expects a "second"
      // distribution dimension).
      transformation = std::make_shared<grampc::UnscentedTransformation>(
        kMpcStateDim, kMpcStateDim,
        config_.ut_alpha, config_.ut_beta, config_.ut_kappa);
      break;
    case StochasticTransformationType::StirlingFirstOrder:
      // dimX = state dimension, dimY = same as dimX (no separate output dim),
      // stepSize = scaling factor for the interpolation grid (h = stepSize *
      // sigma). Typical choice: 1.0 for unit-sigma perturbations.
      transformation = std::make_shared<grampc::StirlingInterpolationFirstOrder>(
        kMpcStateDim, kMpcStateDim, /*stepSize=*/1.0);
      break;
    case StochasticTransformationType::StirlingSecondOrder:
      transformation = std::make_shared<grampc::StirlingInterpolationSecondOrder>(
        kMpcStateDim, kMpcStateDim, /*stepSize=*/1.0);
      break;
    case StochasticTransformationType::MonteCarlo:
      // Should not reach here; handled in buildMonteCarloProblem().
      error_ = "buildSigmaPointProblem: called with MonteCarlo type";
      return;
  }

  if (config_.chance_constraints.enabled && inner_->hasVelocityLimits()) {
    grampc::ChanceConstraintApproximationConstPtr approx =
      makeChanceApproximation(config_.chance_constraints.satisfaction_probability);
    stochastic_problem_ = grampc::SigmaPointProblem(
      inner_, approx, transformation);
  } else {
    // No chance-constraint tightening: just propagate uncertainty through
    // the dynamics. Constraints remain deterministic.
    stochastic_problem_ = grampc::SigmaPointProblem(inner_, transformation);
  }

  // Finalise the point representation for x0 (initial state) and p0
  // (parameters). compute_x0_and_p0 is on the SigmaPointProblemDescription
  // subclass, not on the base class, so we must static_pointer_cast.
  auto sigma_p = std::static_pointer_cast<grampc::SigmaPointProblemDescription>(
    stochastic_problem_);
  sigma_p->compute_x0_and_p0(state_dist);
}

void StochasticGrampcDynamics::buildMonteCarloProblem()
{
  // Monte Carlo: same idea but uses sampling rather than deterministic
  // sigma points. Slower per step, but the only transformation supported
  // by GRAMPC-S that doesn't require gradients of the dynamics through the
  // distribution (useful for highly nonlinear or non-Gaussian cases).
  grampc::DistributionPtr state_dist = makeStateDistribution(config_.state);

  // MonteCarloProblem does not accept a separate chance-constraint
  // approximation - it enforces the (deterministic) constraint at every
  // sample point, so the constraint becomes a hard constraint on all
  // samples. This is stronger than the chance constraint produced by
  // SigmaPoint, and therefore more conservative. We report this in the
  // verbose log.
  if (config_.verbose && config_.chance_constraints.enabled) {
    std::cout << "[StochasticGrampcDynamics] Note: Monte Carlo transformation "
      "ignores `chance_constraints.violation_probability` and "
      "enforces h(x) <= 0 at every sample point (hard constraint)."
              << std::endl;
  }

  // Construct the Monte Carlo point transformation with a fixed seed for
  // reproducibility. Production code may want to thread a seed from the
  // caller; for now we use a deterministic default.
  static grampc::RandomNumberGenerator rng{std::random_device{}()};
  grampc::PointTransformationPtr mc = grampc::MonteCarlo(
    kMpcStateDim, kMpcStateDim, static_cast<typeInt>(numPoints()), rng);

  stochastic_problem_ = grampc::MonteCarloProblem(inner_, mc);

  if (!stochastic_problem_) {
    error_ = "StochasticGrampcDynamics: MonteCarloProblem factory returned null";
    return;
  }

  // compute_x0_and_p0 lives on the MonteCarloProblemDescription subclass.
  auto mc_p = std::static_pointer_cast<grampc::MonteCarloProblemDescription>(
    stochastic_problem_);
  mc_p->compute_x0_and_p0(state_dist);
}

int StochasticGrampcDynamics::numPoints() const
{
  // The library doesn't expose a public accessor, so we infer from the
  // chosen transformation type. This is used only for logging / diagnostics;
  // it doesn't affect solver behaviour.
  const int n = kMpcStateDim + config_.parameters.count;
  switch (config_.transformation) {
    case StochasticTransformationType::Unscented:
    case StochasticTransformationType::StirlingFirstOrder:
    case StochasticTransformationType::StirlingSecondOrder:
      return 2 * n + 1;
    case StochasticTransformationType::MonteCarlo:
      // Default Monte Carlo sample count used by GRAMPC-S. The library's
      // MonteCarloProblem uses 1000 by default; we expose that here.
      return 1000;
  }
  return 0;
}

}  // namespace asr_sdm_kinematic_dynamic_model

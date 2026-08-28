// Copyright (c) 2025.
// Stochastic MPC wrapper that augments GrampcUnderwaterDynamics with
// uncertainty propagation and chance-constraint tightening via GRAMPC-S.
//
// This class composes (rather than inherits from) the deterministic
// GrampcUnderwaterDynamics: it constructs a SigmaPointProblemDescription (or
// other point transformation) on top of the deterministic problem, then the
// user creates a `grampc::Grampc` solver with the resulting problem
// description as input. All stochastic computations are performed by
// GRAMPC-S; this class only handles configuration translation.

#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_STOCHASTIC_GRAMPC_DYNAMICS_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_STOCHASTIC_GRAMPC_DYNAMICS_HPP_

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include "asr_sdm_kinematic_dynamic_model/stochastic_mpc_config.hpp"

// GRAMPC-S uncertainty handling headers. These pull in Eigen and the
// grampc-s distribution / point-transformation infrastructure. They are part
// of the same GRAMPC-S installation that provides `ProblemDescription`, so no
// new external dependency is introduced.
#if __has_include(<grampc_s/grampc_s.hpp>)
#include <grampc_s/grampc_s.hpp>
#elif __has_include(<grampc_s/problem_description/sigma_point_problem_description.hpp>)
#include <grampc_s/problem_description/sigma_point_problem_description.hpp>
#include <grampc_s/problem_description/monte_carlo_problem_description.hpp>
#include <grampc_s/point_transformation/unscented_transformation.hpp>
#include <grampc_s/point_transformation/stirling_interpolation_first_order.hpp>
#include <grampc_s/point_transformation/stirling_interpolation_second_order.hpp>
#include <grampc_s/point_transformation/monte_carlo.hpp>
#include <grampc_s/distribution/Gaussian_distribution.hpp>
#include <grampc_s/distribution/multivariate_uncorrelated_distribution.hpp>
#include <grampc_s/constraint_approx/Gaussian_constraint_approximation.hpp>
#else
#error "GRAMPC-S headers not found; cannot enable stochastic MPC."
#endif

#include <memory>
#include <stdexcept>
#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

/// Number of sigma points used by the Unscented Transformation for a problem
/// of dimension n (nx + np). 2n + 1 points.
inline constexpr int kSigmaPointCount(int nx, int np) {return 2 * (nx + np) + 1;}

/// Stochastic MPC problem description wrapper.
///
/// `StochasticGrampcDynamics` owns:
/// 1. A deterministic `GrampcUnderwaterDynamics` (the "inner" problem).
/// 2. A GRAMPC-S point-transformation problem description (`SigmaPoint...` or
///    `MonteCarloProblemDescription`) that wraps the inner problem and adds
///    uncertainty propagation + chance-constraint tightening.
///
/// Usage:
///
///     GrampcDimensionConfig dims;
///     // Enable chance constraints (otherwise the wrapper falls back to the
///     // deterministic constraint h(x)<=0).
///     GrampcCostWeights weights;
///
///     GrampcUnderwaterDynamics inner(sim_params, dims, weights);
///     StochasticMpcConfig cfg;
///     cfg.state.stddev = ...;        // initial-state uncertainty
///     cfg.chance_constraints.violation_probability = ...;
///
///     StochasticGrampcDynamics stochastic(inner, cfg);
///     grampc::Grampc solver(stochastic.problemDescription());
///     // solver.run() now propagates uncertainty via the chosen point
///     // transformation and tightens constraints per the chance constraint.
///
class StochasticGrampcDynamics
{
public:
  /// Construct the stochastic wrapper around a fully-initialised deterministic
  /// `GrampcUnderwaterDynamics`. The inner dynamics must be `isValid()`.
  ///
  /// @param inner  The deterministic MPC problem description.
  /// @param config Uncertainty configuration (state/parameter distributions and
  ///               chance-constraint probabilities).
  StochasticGrampcDynamics(
    std::shared_ptr<GrampcUnderwaterDynamics> inner,
    const StochasticMpcConfig & config);

  /// Non-copyable. The underlying GRAMPC-S problem description is opaque and
  /// contains raw pointers that are not safe to copy.
  StochasticGrampcDynamics(const StochasticGrampcDynamics &) = delete;
  StochasticGrampcDynamics & operator=(const StochasticGrampcDynamics &) = delete;

  /// Move-constructible / movable so callers can return values from factories.
  StochasticGrampcDynamics(StochasticGrampcDynamics &&) = default;
  StochasticGrampcDynamics & operator=(StochasticGrampcDynamics &&) = default;

  /// Get the wrapped deterministic problem description. Useful for inspection
  /// or for re-using the inner dynamics in a non-stochastic solver.
  const GrampcUnderwaterDynamics & inner() const {return *inner_;}

  /// Get the underlying shared pointer (e.g. to keep the inner alive).
  std::shared_ptr<GrampcUnderwaterDynamics> innerPtr() const {return inner_;}

  /// Get the GRAMPC-S problem description that should be passed to
  /// `grampc::Grampc`. The returned shared_ptr is the SigmaPoint or Monte
  /// Carlo problem description that wraps the inner deterministic problem.
  grampc::ProblemDescriptionPtr problemDescription() const
  {
    return stochastic_problem_;
  }

  /// Number of points used by the chosen transformation. Useful for logging
  /// and for an ablation study comparing transformations.
  int numPoints() const;

  /// Whether the construction succeeded and a valid problem description is
  /// available.
  bool isValid() const {return error_.empty();}
  const std::string & error() const {return error_;}

  /// The configuration that was used (for introspection).
  const StochasticMpcConfig & config() const {return config_;}

private:
  void buildSigmaPointProblem();
  void buildMonteCarloProblem();

  std::shared_ptr<GrampcUnderwaterDynamics> inner_;
  StochasticMpcConfig config_;

  grampc::ProblemDescriptionPtr stochastic_problem_;
  std::string error_;
};

/// Factory helper for the common case: a deterministic MPC controller with a
/// stochastic uncertainty layer. Equivalent to constructing
/// `StochasticGrampcDynamics` directly but convenient when the caller only
/// has the inner dynamics on the stack.
inline std::shared_ptr<StochasticGrampcDynamics> makeStochasticDynamics(
  std::shared_ptr<GrampcUnderwaterDynamics> inner,
  const StochasticMpcConfig & config)
{
  return std::make_shared<StochasticGrampcDynamics>(std::move(inner), config);
}

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_STOCHASTIC_GRAMPC_DYNAMICS_HPP_

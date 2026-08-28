// Copyright (c) 2025.
// Uncertainty configuration for stochastic GRAMPC-S MPC.
//
// Provides configuration types for state/process uncertainty distributions,
// parametric uncertainty, and probabilistic (chance) constraints that are
// propagated through the underwater MPC dynamics via GRAMPC-S' point-based
// transformations (Unscented Transform, Monte Carlo, etc.).

#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_STOCHASTIC_MPC_CONFIG_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_STOCHASTIC_MPC_CONFIG_HPP_

#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace asr_sdm_kinematic_dynamic_model
{

/// Type of point-based approximation used to propagate uncertainty through
/// the dynamics and through the chance constraints.
enum class StochasticTransformationType
{
  /// Unscented (sigma-point) transformation - O(n^2) sigma points, fast & deterministic.
  /// Recommended for low-to-moderate state dimensions (our nx=24 is acceptable).
  Unscented,
  /// Stirling first-order interpolation (2n+1 points). Cheaper than UT, slightly less accurate.
  StirlingFirstOrder,
  /// Stirling second-order interpolation (4n+1 points). Better accuracy than first order.
  StirlingSecondOrder,
  /// Monte Carlo sampling. Statistical convergence, no gradient through the distribution.
  /// Higher computational cost, recommended for validation rather than runtime.
  MonteCarlo
};

/// Configuration of the initial-state distribution.
///
/// We support a multivariate Gaussian over the 24-dim tangent state. The
/// covariance can be diagonal (uncorrelated) or fully dense. Quaternions are
/// NOT part of the MPC state (we use a tangent-space delta_q), so the Gaussian
/// distribution is well defined over the full 24-dim state vector.
struct StateUncertaintyConfig
{
  /// Mean is always zero in our reference-frame formulation; the MPC reference
  /// is the actual mean. This is exposed for completeness / future extensions.
  Eigen::Matrix<double, kMpcStateDim, 1> mean =
    Eigen::Matrix<double, kMpcStateDim, 1>::Zero();

  /// Diagonal covariance (uncorrelated). Default = small (1e-4 I).
  Eigen::Matrix<double, kMpcStateDim, 1> stddev =
    Eigen::Matrix<double, kMpcStateDim, 1>::Ones() * 1.0e-2;

  /// Full covariance matrix. Only used when `use_full_covariance = true`.
  /// Useful when sensor noise is correlated across axes.
  Eigen::Matrix<double, kMpcStateDim, kMpcStateDim> covariance;

  /// Must be set to `true` to use the `covariance` matrix above.
  /// When `false` (the default), the diagonal `stddev` vector is used.
  /// This avoids ambiguity with the default-initialized zero matrix.
  bool use_full_covariance{false};

  bool hasFullCovariance() const {return use_full_covariance;}

  bool isValid(std::string * error = nullptr) const
  {
    if (!mean.allFinite()) {
      if (error) {*error = "StateUncertaintyConfig: mean contains non-finite entries";}
      return false;
    }
    if (mean.size() != kMpcStateDim) {
      if (error) {*error = "StateUncertaintyConfig: mean size mismatch";}
      return false;
    }
    if (hasFullCovariance()) {
      if (covariance.rows() != kMpcStateDim || covariance.cols() != kMpcStateDim) {
        if (error) {*error = "StateUncertaintyConfig: covariance size mismatch";}
        return false;
      }
      if (!covariance.allFinite()) {
        if (error) {*error = "StateUncertaintyConfig: covariance contains non-finite entries";}
        return false;
      }
      // Check positive semi-definite via Cholesky (throws on failure).
      // Zero matrix IS PSD but is almost never the user's intent.
      if (covariance.isApprox(Eigen::Matrix<double, kMpcStateDim, kMpcStateDim>::Zero())) {
        if (error) {
          *error = "StateUncertaintyConfig: covariance is all-zero; did you mean to use stddev?";
        }
        return false;
      }
      Eigen::LLT<Eigen::Matrix<double, kMpcStateDim, kMpcStateDim>> llt(covariance);
      if (llt.info() != Eigen::Success) {
        if (error) {*error = "StateUncertaintyConfig: covariance is not positive semi-definite";}
        return false;
      }
    } else {
      if (stddev.size() != kMpcStateDim || !stddev.allFinite() ||
        (stddev.array() < 0.0).any())
      {
        if (error) {*error = "StateUncertaintyConfig: stddev invalid";}
        return false;
      }
    }
    return true;
  }
};

/// Configuration of parametric (process / model) uncertainty.
///
/// Typical uses:
/// - Unknown added mass: scale added-mass coefficient with a multiplicative noise
/// - Unknown fluid drag: scale quadratic drag with a multiplicative noise
/// - Actuator bias: bias on thrust / torque channels
///
/// We expose a generic interface: the user provides nominal values plus a
/// covariance. The MPC propagates this as additive Gaussian process noise on
/// the state derivative via the sigma-point machinery.
struct ParametricUncertaintyConfig
{
  /// Number of uncertain parameters. Must equal the underlying `dim_config.np`.
  /// (In our current underwater model `np = 0`; using parametric uncertainty
  /// therefore requires `dim_config.np = this->count > 0`.)
  int count{0};

  /// Mean of the parameter distribution. Defaults to zero (i.e., the nominal
  /// model; uncertainty is centered).
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(0);

  /// Diagonal std-dev for each parameter.
  Eigen::VectorXd stddev = Eigen::VectorXd::Zero(0);

  bool isValid(std::string * error = nullptr) const
  {
    if (count <= 0) {return true;}
    if (mean.size() != count || stddev.size() != count) {
      if (error) {*error = "ParametricUncertaintyConfig: mean/stddev size mismatch";}
      return false;
    }
    if (!mean.allFinite() || !stddev.allFinite() || (stddev.array() < 0.0).any()) {
      if (error) {*error = "ParametricUncertaintyConfig: mean/stddev invalid";}
      return false;
    }
    return true;
  }
};

/// Configuration of the chance constraints.
///
/// The default `enable_velocity_limits_` of `GrampcUnderwaterDynamics` exposes
/// 8 scalar inequality constraints of the form `h_i(x) <= 0` (linear / angular
/// / joint velocity squared minus squared limit). Under uncertainty each of
/// these becomes a chance constraint:
///     P(h_i(x) <= 0) >= P_sat
/// where `P_sat` is the **satisfaction probability** (e.g. 0.95 = 95%).
/// GRAMPC-S approximates this via `GaussianConstraintApproximation`, which
/// yields a deterministic tightened constraint:
///     E{h_i} + z * sqrt(Var{h_i}) <= 0,   z = Phi^{-1}(P_sat)
/// For `P_sat = 0.95`, z ≈ 1.645, making the constraint more conservative.
struct ChanceConstraintsConfig
{
  /// Satisfaction probability per constraint. Must match `nh` of the
  /// underlying problem. Values in (0, 1). Default: 0.95 (i.e. 95% chance
  /// of satisfying the velocity constraint).
  Eigen::VectorXd satisfaction_probability =
    Eigen::VectorXd::Constant(8, 0.95);

  /// Whether to enforce the chance constraints at all. When false the MPC
  /// solves the nominal (deterministic) constraint problem inside the
  /// stochastic framework - useful for ablation studies.
  bool enabled{true};

  bool isValid(int expected_nh, std::string * error = nullptr) const
  {
    if (!enabled) {return true;}
    if (satisfaction_probability.size() != expected_nh) {
      if (error) {
        *error = "ChanceConstraintsConfig: satisfaction_probability size (" +
          std::to_string(satisfaction_probability.size()) +
          ") != nh (" + std::to_string(expected_nh) + ")";
      }
      return false;
    }
    if (!satisfaction_probability.allFinite() ||
      (satisfaction_probability.array() <= 0.0).any() ||
      (satisfaction_probability.array() >= 1.0).any())
    {
      if (error) {*error = "ChanceConstraintsConfig: probabilities must be in (0, 1)";}
      return false;
    }
    return true;
  }
};

/// Aggregate configuration for stochastic MPC.
struct StochasticMpcConfig
{
  /// Type of point transformation.
  StochasticTransformationType transformation{StochasticTransformationType::Unscented};

  /// Unscented-transformation hyperparameters (only used when
  /// `transformation == Unscented`). Standard recommendations: alpha=1e-3,
  /// beta=2, kappa=0. For our 24-dim state, kappa=0 with alpha=1e-3 works well.
  double ut_alpha{1.0e-3};
  double ut_beta{2.0};
  /// kappa = 3 - nx is the standard for additive Gaussian noise; we use 0
  /// since the user typically wants the standard symmetric UT.
  double ut_kappa{0.0};

  /// Process (state) uncertainty distribution.
  StateUncertaintyConfig state;

  /// Parametric (model) uncertainty distribution.
  ParametricUncertaintyConfig parameters;

  /// Chance-constraint configuration.
  ChanceConstraintsConfig chance_constraints;

  /// When true the MPC will print a single line of diagnostics on each
  /// successful build (point count, etc.). Disable in production.
  bool verbose{false};

  bool isValid(int expected_nh, std::string * error = nullptr) const
  {
    if (!state.isValid(error)) {return false;}
    if (!parameters.isValid(error)) {return false;}
    if (!chance_constraints.isValid(expected_nh, error)) {return false;}
    if (transformation == StochasticTransformationType::Unscented) {
      if (ut_alpha <= 0.0 || ut_beta < 0.0) {
        if (error) {*error = "StochasticMpcConfig: invalid UT hyperparameters";}
        return false;
      }
    }
    return true;
  }
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_STOCHASTIC_MPC_CONFIG_HPP_

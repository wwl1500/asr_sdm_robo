// Copyright (c) 2025.
// GRAMPC 接入层：将水下机器人动力学模型桥接到 GRAMPC MPC 框架

#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_GRAMPC_DYNAMICS_INTERFACE_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_GRAMPC_DYNAMICS_INTERFACE_HPP_

#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <grampc_s/problem_description/problem_description.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

// A Pinocchio configuration contains a quaternion and has nq=13, while its tangent
// space has nv=12. GRAMPC uses Euclidean state vectors, so q is represented as a
// 12-dimensional tangent displacement from referenceConfiguration().
constexpr int kMpcConfigurationTangentDim = kReducedNv;
constexpr int kMpcStateDim = kMpcConfigurationTangentDim + kReducedNv;
constexpr int kMpcControlDim = kNumLinks + kNumJointDofs;
using MpcState = Eigen::Matrix<double, kMpcStateDim, 1>;
using MpcStateJacobian = Eigen::Matrix<double, kMpcStateDim, kMpcStateDim>;
using MpcControlJacobian = Eigen::Matrix<double, kMpcStateDim, kMpcControlDim>;

/// GRAMPC problem dimensions supported by this implementation.
struct GrampcDimensionConfig
{
  typeInt nx{kMpcStateDim};
  typeInt nu{kMpcControlDim};
  typeInt np{0};
  typeInt ng{0};
  typeInt nh{0};
  typeInt ng_terminal{0};
  typeInt nh_terminal{0};

  bool isValid() const
  {
    return nx == kMpcStateDim && nu == kMpcControlDim && np == 0 && ng == 0 && nh == 0 &&
           ng_terminal == 0 && nh_terminal == 0;
  }
};

/// Diagonal quadratic-cost weights for the tangent-configuration and velocity state.
struct GrampcCostWeights
{
  Eigen::Matrix<double, kMpcStateDim, 1> state_weight =
    Eigen::Matrix<double, kMpcStateDim, 1>::Ones();
  Eigen::Matrix<double, kMpcControlDim, 1> control_weight =
    Eigen::Matrix<double, kMpcControlDim, 1>::Ones() * 0.01;
  Eigen::Matrix<double, kMpcStateDim, 1> terminal_weight =
    Eigen::Matrix<double, kMpcStateDim, 1>::Ones() * 10.0;

  bool isValid() const
  {
    return state_weight.allFinite() && control_weight.allFinite() &&
           terminal_weight.allFinite() && !(state_weight.array() < 0.0).any() &&
           !(control_weight.array() < 0.0).any() && !(terminal_weight.array() < 0.0).any();
  }
};

/// Bridges UnderwaterSimulator to GRAMPC.
///
/// The 24-dimensional state is [delta_q(12), v(12)]. delta_q is a tangent-space
/// displacement from referenceConfiguration(), reconstructed with Pinocchio's
/// integrate() before every dynamics evaluation. A controller must update the
/// reference to its measured configuration before starting each MPC solve; x0 then
/// normally has zero delta_q.
class GrampcUnderwaterDynamics : public grampc::ProblemDescription
{
public:
  GrampcUnderwaterDynamics(
    const UnderwaterSimulatorParameters & sim_params,
    const GrampcDimensionConfig & dim_config = GrampcDimensionConfig{},
    const GrampcCostWeights & cost_weights = GrampcCostWeights{},
    std::optional<ReducedConfiguration> initial_configuration = std::nullopt);

  void ffct(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;
  void dfdx_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    grampc::VectorConstRef vec, const grampc::GrampcParam & param) override;
  void dfdu_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    grampc::VectorConstRef vec, const grampc::GrampcParam & param) override;
  void dfdp_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    grampc::VectorConstRef vec, const grampc::GrampcParam & param) override;

  void lfct(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;
  void dldx(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;
  void dldu(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;
  void Vfct(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;
  void dVdx(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;
  void dVdT(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;

  void hfct(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;
  void dhdx_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    grampc::VectorConstRef vec, const grampc::GrampcParam & param) override;
  void dhdu_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    grampc::VectorConstRef vec, const grampc::GrampcParam & param) override;
  void dhdp_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x,
    grampc::VectorConstRef u, grampc::VectorConstRef p,
    grampc::VectorConstRef vec, const grampc::GrampcParam & param) override;

  const UnderwaterSimulator & simulator() const {return *simulator_;}
  UnderwaterSimulator & simulator() {return *simulator_;}
  const ReducedConfiguration & referenceConfiguration() const {return reference_configuration_;}
  void updateReferenceConfiguration(const ReducedConfiguration & configuration);

  // P1-2: Update fluid current for MPC prediction
  void updateFluidCurrent(
    const Eigen::Vector3d & current_world,
    const Eigen::Vector3d & current_acceleration_world = Eigen::Vector3d::Zero());
  const Eigen::Vector3d & fluidCurrent() const {return fluid_current_;}
  const Eigen::Vector3d & fluidCurrentAcceleration() const {return fluid_current_acceleration_;}

  void setCostWeights(const GrampcCostWeights & weights);
  const GrampcCostWeights & costWeights() const {return cost_weights_;}
  bool isValid() const
  {
    return simulator_ && simulator_->isValid() && dim_config_.isValid() &&
           cost_weights_.isValid() &&
           isValidConfiguration(reference_configuration_);
  }
  const std::string & error() const {return error_;}

private:
  static bool isValidConfiguration(const ReducedConfiguration & configuration);
  void validateStateAndControl(
    const grampc::VectorConstRef & x, const grampc::VectorConstRef & u,
    const char * callback) const;
  ReducedConfiguration configurationFromState(const MpcState & state) const;
  MpcState computeStateDerivative(
    const MpcState & state,
    const Eigen::Matrix<double, kMpcControlDim, 1> & control) const;
  ReducedAcceleration computeAcceleration(
    const ReducedConfiguration & configuration, const ReducedVelocity & velocity,
    const SegmentThrustVector & segment_thrust, const JointTorqueVector & joint_torque) const;
  MpcStateJacobian computeJacobianX(
    const MpcState & state, const Eigen::Matrix<double, kMpcControlDim, 1> & control) const;
  MpcControlJacobian computeJacobianU(
    const MpcState & state, const Eigen::Matrix<double, kMpcControlDim, 1> & control) const;

  std::unique_ptr<UnderwaterSimulator> simulator_;
  GrampcDimensionConfig dim_config_;
  GrampcCostWeights cost_weights_;
  ReducedConfiguration reference_configuration_{ReducedConfiguration::Zero()};
  // P1-2: Frozen fluid current for MPC prediction
  Eigen::Vector3d fluid_current_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d fluid_current_acceleration_{Eigen::Vector3d::Zero()};
  std::string error_;

  static constexpr double kJacobianEpsilon = 1e-6;
  static constexpr double kConfigurationDerivativeEpsilon = 1e-6;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_GRAMPC_DYNAMICS_INTERFACE_HPP_

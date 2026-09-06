#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_SIMULATOR_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_SIMULATOR_HPP_

#include "asr_sdm_kinematic_dynamic_model/actuator_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/fluid_force_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/joint_dynamics_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"

#include <Eigen/Core>

#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

struct UnderwaterSimulatorState
{
  double time{0.0};
  ReducedConfiguration configuration = ReducedConfiguration::Zero();
  ReducedVelocity velocity = ReducedVelocity::Zero();
  /// Thrust actually delivered by each screw. Only meaningful when the actuator model
  /// has a nonzero thrust_time_constant; otherwise the command is applied directly.
  RotorVector rotor_thrust = RotorVector::Zero();
};

struct UnderwaterSimulatorInput
{
  SegmentThrustVector segment_thrust = SegmentThrustVector::Zero();
  /// Per-rotor angular-velocity command, used when the actuator model runs in
  /// ActuatorCommandMode::RotorVelocity.
  RotorVector rotor_rate = RotorVector::Zero();
  JointTorqueVector joint_torque = JointTorqueVector::Zero();
  Eigen::Vector3d fluid_current_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d fluid_current_acceleration_world = Eigen::Vector3d::Zero();
};

struct UnderwaterDynamicsOutput
{
  ReducedAcceleration acceleration = ReducedAcceleration::Zero();
  Eigen::Matrix<double, kReducedNv, kReducedNv> effective_mass =
    Eigen::Matrix<double, kReducedNv, kReducedNv>::Zero();
  GeneralizedVector rigid_nonlinear_effects = GeneralizedVector::Zero();
  GeneralizedVector rigid_coriolis_force = GeneralizedVector::Zero();
  GeneralizedVector rigid_gravity_force = GeneralizedVector::Zero();
  GeneralizedVector actuator_force = GeneralizedVector::Zero();
  GeneralizedVector added_mass_bias_force = GeneralizedVector::Zero();
  GeneralizedVector damping_force = GeneralizedVector::Zero();
  GeneralizedVector buoyancy_force = GeneralizedVector::Zero();
  GeneralizedVector froude_krylov_force = GeneralizedVector::Zero();
  GeneralizedVector joint_passive_force = GeneralizedVector::Zero();
  GeneralizedVector right_hand_side = GeneralizedVector::Zero();
  GeneralizedVector balance_residual = GeneralizedVector::Zero();
  RotorVector commanded_rotor_thrust = RotorVector::Zero();
  PinocchioKinematicsState kinematics;
  PinocchioHydrodynamicEvaluation hydrodynamics;
  ActuatorEvaluation actuators;
  JointDynamicsEvaluation joints;
};

enum class IntegrationMethod
{
  /// Semi-implicit Euler: velocity first, then the configuration with the new velocity.
  /// Cheap, one dynamics evaluation per step, and dissipative enough to stay stable.
  SemiImplicitEuler,
  /// Classical RK4 taken in the tangent space at the start of the step. Four dynamics
  /// evaluations per step, and markedly more accurate at the same dt.
  RungeKutta4,
};

struct UnderwaterSimulatorParameters
{
  /// Single source of truth for gravity. The simulator copies it into both the
  /// rigid-body and the hydrodynamic model so weight and buoyancy cannot disagree.
  Eigen::Vector3d gravity_world{0.0, 0.0, -9.81};
  IntegrationMethod integration_method{IntegrationMethod::SemiImplicitEuler};
  PinocchioModelParameters pinocchio;
  HydrodynamicModelParameters hydrodynamics;
  ActuatorModelParameters actuators;
  JointDynamicsParameters joints;
};

class UnderwaterSimulator
{
public:
  explicit UnderwaterSimulator(const UnderwaterSimulatorParameters & params);

  bool isValid() const;
  const std::string & error() const;
  const PinocchioModel & pinocchioModel() const;
  const ActuatorModel & actuatorModel() const;
  const JointDynamicsModel & jointDynamicsModel() const;
  IntegrationMethod integrationMethod() const;
  const Eigen::Vector3d & gravityWorld() const;
  UnderwaterSimulatorState makeInitialState() const;

  /// Articulation angles and rates pulled out of a full reduced state, ordered the
  /// same way as the controller DOF mapping.
  JointTorqueVector jointPosition(const UnderwaterSimulatorState & state) const;
  JointTorqueVector jointVelocity(const UnderwaterSimulatorState & state) const;

  UnderwaterDynamicsOutput evaluate(
    const UnderwaterSimulatorState & state,
    const UnderwaterSimulatorInput & input) const;
  /// Advances the state and returns the dynamics evaluated at its start, so the
  /// returned output always describes the state that was passed in.
  UnderwaterDynamicsOutput step(
    UnderwaterSimulatorState & state,
    double dt,
    const UnderwaterSimulatorInput & input) const;
  /// Swaps the hydrodynamic parameters atomically, keeping the simulator gravity. The
  /// gravity field of the incoming parameters is ignored for that reason.
  bool replaceHydrodynamicParameters(
    const HydrodynamicModelParameters & parameters, std::string * error = nullptr);
  const HydrodynamicModelParameters & hydrodynamicParameters() const;

private:
  ReducedAcceleration accelerationOnly(
    const UnderwaterSimulatorState & state, const UnderwaterSimulatorInput & input) const;

  Eigen::Vector3d gravity_world_{0.0, 0.0, -9.81};
  IntegrationMethod integration_method_{IntegrationMethod::SemiImplicitEuler};
  PinocchioModel pinocchio_model_;
  FluidForceModel fluid_model_;
  ActuatorModel actuator_model_;
  JointDynamicsModel joint_model_;
  std::array<PinocchioControllerDofMapping, kNumJointDofs> joint_mapping_{};
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_SIMULATOR_HPP_

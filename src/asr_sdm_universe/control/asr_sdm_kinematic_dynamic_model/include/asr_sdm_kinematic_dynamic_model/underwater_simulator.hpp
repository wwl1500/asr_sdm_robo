#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_SIMULATOR_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_SIMULATOR_HPP_

#include "asr_sdm_kinematic_dynamic_model/actuator_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/fluid_force_model.hpp"
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
};

struct UnderwaterSimulatorInput
{
  SegmentThrustVector segment_thrust = SegmentThrustVector::Zero();
  JointTorqueVector joint_torque = JointTorqueVector::Zero();
  Eigen::Vector3d fluid_current_world = Eigen::Vector3d::Zero();
};

struct UnderwaterDynamicsOutput
{
  ReducedAcceleration acceleration = ReducedAcceleration::Zero();
  Eigen::Matrix<double, kReducedNv, kReducedNv> effective_mass =
    Eigen::Matrix<double, kReducedNv, kReducedNv>::Zero();
  GeneralizedVector rigid_nonlinear_effects = GeneralizedVector::Zero();
  GeneralizedVector actuator_force = GeneralizedVector::Zero();
  GeneralizedVector added_mass_bias_force = GeneralizedVector::Zero();
  GeneralizedVector damping_force = GeneralizedVector::Zero();
  GeneralizedVector buoyancy_force = GeneralizedVector::Zero();
  GeneralizedVector right_hand_side = GeneralizedVector::Zero();
  GeneralizedVector balance_residual = GeneralizedVector::Zero();
  PinocchioKinematicsState kinematics;
  PinocchioHydrodynamicEvaluation hydrodynamics;
  ActuatorEvaluation actuators;
};

struct UnderwaterSimulatorParameters
{
  PinocchioModelParameters pinocchio;
  HydrodynamicModelParameters hydrodynamics;
  ActuatorModelParameters actuators;
};

class UnderwaterSimulator
{
public:
  explicit UnderwaterSimulator(const UnderwaterSimulatorParameters & params);

  bool isValid() const;
  const std::string & error() const;
  const PinocchioModel & pinocchioModel() const;
  UnderwaterSimulatorState makeInitialState() const;

  UnderwaterDynamicsOutput evaluate(
    const UnderwaterSimulatorState & state,
    const UnderwaterSimulatorInput & input) const;
  UnderwaterDynamicsOutput step(
    UnderwaterSimulatorState & state,
    double dt,
    const UnderwaterSimulatorInput & input) const;
  bool replaceHydrodynamicParameters(
    const HydrodynamicModelParameters & parameters, std::string * error = nullptr);
  const HydrodynamicModelParameters & hydrodynamicParameters() const;

private:
  PinocchioModel pinocchio_model_;
  FluidForceModel fluid_model_;
  ActuatorModel actuator_model_;
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_SIMULATOR_HPP_

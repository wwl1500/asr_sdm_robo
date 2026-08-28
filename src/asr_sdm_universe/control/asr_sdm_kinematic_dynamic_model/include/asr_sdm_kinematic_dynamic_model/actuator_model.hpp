#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ACTUATOR_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ACTUATOR_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/fluid_force_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

using SegmentThrustVector = Eigen::Matrix<double, kNumLinks, 1>;
using JointTorqueVector = Eigen::Matrix<double, kNumJointDofs, 1>;

struct ActuatorModelParameters
{
  SegmentThrustVector maximum_segment_thrust = SegmentThrustVector::Constant(1.0);
  Eigen::Vector3d rotor_force_axis_local{0.0, 0.0, 1.0};
};

struct ActuatorEvaluation
{
  SegmentThrustVector actual_segment_thrust = SegmentThrustVector::Zero();
  GeneralizedVector generalized_force = GeneralizedVector::Zero();
  std::array<std::array<SpatialVector, 2>, kNumLinks> rotor_wrenches{};
};

class ActuatorModel
{
public:
  explicit ActuatorModel(const ActuatorModelParameters & params = ActuatorModelParameters{});

  bool isValid() const;
  const std::string & error() const;
  ActuatorEvaluation evaluate(
    const PinocchioKinematicsState & kinematics,
    const SegmentThrustVector & segment_thrust,
    const JointTorqueVector & joint_torque = JointTorqueVector::Zero()) const;

private:
  ActuatorModelParameters params_;
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ACTUATOR_MODEL_HPP_

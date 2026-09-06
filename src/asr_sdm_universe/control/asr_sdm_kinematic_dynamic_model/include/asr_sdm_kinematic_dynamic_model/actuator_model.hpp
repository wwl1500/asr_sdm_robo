#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ACTUATOR_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ACTUATOR_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/fluid_force_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/screw_drive_model.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

using SegmentThrustVector = Eigen::Matrix<double, kNumLinks, 1>;
using JointTorqueVector = Eigen::Matrix<double, kNumJointDofs, 1>;
using RotorVector = Eigen::Matrix<double, kNumRotors, 1>;

enum class ActuatorCommandMode
{
  /// One aggregate thrust per segment, split equally between its two screws.
  AggregateThrust,
  /// Per-rotor angular-velocity command mapped through the thrust coefficients.
  RotorVelocity,
};

struct ActuatorModelParameters
{
  ActuatorCommandMode command_mode{ActuatorCommandMode::AggregateThrust};
  SegmentThrustVector maximum_segment_thrust = SegmentThrustVector::Constant(1.0);
  // Rotor force axis in the segment's local frame.
  // The URDF applies rpy="0 -pi/2 0" which rotates: CAD +Z → World +X (forward).
  // In the segment's local frame BEFORE this rotation, the螺旋桨 thrust is along -X
  // to produce forward motion in the world frame. This compensates for the URDF rotation.
  Eigen::Vector3d rotor_force_axis_local{-1.0, 0.0, 0.0};

  /// RotorVelocity mode thrust map, f = thrust_linear * w + thrust_quadratic * w * |w|.
  RotorVector thrust_linear = RotorVector::Zero();
  RotorVector thrust_quadratic = RotorVector::Zero();
  RotorVector maximum_rotor_rate = RotorVector::Constant(20.9439510239);

  /// First-order thruster lag, in seconds. Zero makes each screw reach its commanded
  /// thrust within the same step, which is the behaviour of a lag-free thruster.
  double thrust_time_constant{0.0};

  /// Reaction torque that the helix feeds back into its segment. The two screws of a
  /// segment have opposite handedness, so symmetric thrust produces no net torque and
  /// only differential thrust does.
  bool enable_reaction_torque{true};
  ScrewDriveParameters screw;
};

struct ActuatorEvaluation
{
  SegmentThrustVector actual_segment_thrust = SegmentThrustVector::Zero();
  RotorVector rotor_thrust = RotorVector::Zero();
  RotorVector rotor_reaction_torque = RotorVector::Zero();
  GeneralizedVector generalized_force = GeneralizedVector::Zero();
  std::array<std::array<SpatialVector, 2>, kNumLinks> rotor_wrenches{};
};

class ActuatorModel
{
public:
  explicit ActuatorModel(const ActuatorModelParameters & params = ActuatorModelParameters{});

  bool isValid() const;
  const std::string & error() const;
  const ActuatorModelParameters & parameters() const;
  const ScrewDriveModel & screwDriveModel() const;

  /// Saturated per-rotor thrust that the commands ask for, before the thruster lag.
  RotorVector commandedRotorThrust(
    const SegmentThrustVector & segment_thrust, const RotorVector & rotor_rate) const;

  /// Exact first-order lag update over one step. Returns the commanded thrust
  /// unchanged when thrust_time_constant is zero.
  RotorVector advanceRotorThrust(
    const RotorVector & rotor_thrust, const RotorVector & commanded_rotor_thrust,
    double dt) const;

  /// Projects an already-resolved per-rotor thrust into generalized coordinates.
  ActuatorEvaluation evaluate(
    const PinocchioKinematicsState & kinematics,
    const RotorVector & rotor_thrust,
    const JointTorqueVector & joint_torque = JointTorqueVector::Zero()) const;

  /// Lag-free convenience path: resolves the commands and evaluates in one call.
  ActuatorEvaluation evaluate(
    const PinocchioKinematicsState & kinematics,
    const SegmentThrustVector & segment_thrust,
    const JointTorqueVector & joint_torque = JointTorqueVector::Zero(),
    const RotorVector & rotor_rate = RotorVector::Zero()) const;

private:
  ActuatorModelParameters params_;
  ScrewDriveModel screw_model_;
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ACTUATOR_MODEL_HPP_

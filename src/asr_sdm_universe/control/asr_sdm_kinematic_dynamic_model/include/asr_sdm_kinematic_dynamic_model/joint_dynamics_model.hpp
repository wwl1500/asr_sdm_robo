#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_JOINT_DYNAMICS_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_JOINT_DYNAMICS_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/actuator_model.hpp"

#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

/// Passive torques acting on the six articulation DOFs. Every coefficient defaults to
/// zero, so an unconfigured model contributes nothing and the joints stay ideal.
struct JointDynamicsParameters
{
  /// Viscous term -b * theta_dot, in N m s/rad.
  JointTorqueVector viscous_damping = JointTorqueVector::Zero();

  /// Coulomb term -c * tanh(theta_dot / friction_velocity_scale), in N m. The tanh
  /// regularization keeps the right-hand side continuous through zero velocity, which
  /// a plain sign() would not.
  JointTorqueVector coulomb_friction = JointTorqueVector::Zero();
  double friction_velocity_scale{1.0e-3};

  /// One-sided spring-damper that pushes a joint back inside its travel. Inactive
  /// while limit_stiffness is zero.
  JointTorqueVector lower_limit = JointTorqueVector::Constant(-1.5707963267948966);
  JointTorqueVector upper_limit = JointTorqueVector::Constant(1.5707963267948966);
  double limit_stiffness{0.0};
  double limit_damping{0.0};
};

struct JointDynamicsEvaluation
{
  JointTorqueVector viscous_torque = JointTorqueVector::Zero();
  JointTorqueVector friction_torque = JointTorqueVector::Zero();
  JointTorqueVector limit_torque = JointTorqueVector::Zero();
  JointTorqueVector total_torque = JointTorqueVector::Zero();
};

class JointDynamicsModel
{
public:
  explicit JointDynamicsModel(
    const JointDynamicsParameters & params = JointDynamicsParameters{});

  bool isValid() const;
  const std::string & error() const;
  const JointDynamicsParameters & parameters() const;

  JointDynamicsEvaluation evaluate(
    const JointTorqueVector & joint_position,
    const JointTorqueVector & joint_velocity) const;

private:
  JointDynamicsParameters params_;
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_JOINT_DYNAMICS_MODEL_HPP_

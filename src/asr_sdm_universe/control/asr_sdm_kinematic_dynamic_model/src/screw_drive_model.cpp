#include "asr_sdm_kinematic_dynamic_model/screw_drive_model.hpp"

#include <cmath>

namespace asr_sdm_kinematic_dynamic_model
{

ScrewDriveModel::ScrewDriveModel(const ScrewDriveParameters & params)
: params_(params)
{
}

ScrewVelocity3D ScrewDriveModel::computeScrewVelocity(
  const JointVelocity3D & joint_velocity) const
{
  ScrewVelocity3D output;
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    output.theta_dot[i] = joint_velocity.theta_dot[i] * std::sin(params_.alpha[i]);
  }
  return output;
}

}  // namespace asr_sdm_kinematic_dynamic_model

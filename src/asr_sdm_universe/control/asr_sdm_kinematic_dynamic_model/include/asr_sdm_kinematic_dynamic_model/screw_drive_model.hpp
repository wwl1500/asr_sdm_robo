#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_SCREW_DRIVE_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_SCREW_DRIVE_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/kinematic_types.hpp"

#include <array>

namespace asr_sdm_kinematic_dynamic_model
{

struct ScrewDriveParameters
{
  std::array<double, kNumJointDofs> alpha{
    -0.7853981633974483, 0.7853981633974483,
    -0.7853981633974483, 0.7853981633974483,
    -0.7853981633974483, 0.7853981633974483};
};

struct ScrewVelocity3D
{
  std::array<double, kNumJointDofs> theta_dot{};
};

class ScrewDriveModel
{
public:
  explicit ScrewDriveModel(const ScrewDriveParameters & params);

  ScrewVelocity3D computeScrewVelocity(const JointVelocity3D & joint_velocity) const;

private:
  ScrewDriveParameters params_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_SCREW_DRIVE_MODEL_HPP_

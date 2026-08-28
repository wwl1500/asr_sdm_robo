#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_KINEMATIC_TYPES_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_KINEMATIC_TYPES_HPP_

#include "asr_sdm_kinematic_dynamic_model/constants.hpp"
#include "asr_sdm_kinematic_dynamic_model/math_types.hpp"

#include <array>

namespace asr_sdm_kinematic_dynamic_model
{

struct JointConfiguration3D
{
  std::array<double, kNumJointDofs> theta{};
};

struct JointVelocity3D
{
  std::array<double, kNumJointDofs> theta_dot{};
};

struct KinematicState3D
{
  Vec3 head_position{0.0, 0.0, 0.0};
  Mat3 head_frame{};
  JointConfiguration3D joints{};
  std::array<Mat3, kNumLinks> link_frames{};
  std::array<Vec3, kNumLinks> link_axes{};
  std::array<Vec3, kNumBodyPoints> body_points{};
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_KINEMATIC_TYPES_HPP_

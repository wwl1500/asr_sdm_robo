#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_JACOBIAN_3D_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_JACOBIAN_3D_HPP_

#include "asr_sdm_kinematic_dynamic_model/math_types.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

class Jacobian3D
{
public:
  static void pitchYawColumns(
    const Mat3 & downstream_frame, double upstream_pitch, Vec3 & pitch_column,
    Vec3 & yaw_column);
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_JACOBIAN_3D_HPP_

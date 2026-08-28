#include "asr_sdm_kinematic_dynamic_model/jacobian_3d.hpp"

#include <cmath>

namespace asr_sdm_kinematic_dynamic_model
{

void Jacobian3D::pitchYawColumns(
  const Mat3 & downstream_frame, double upstream_pitch, Vec3 & pitch_column,
  Vec3 & yaw_column)
{
  pitch_column = column(downstream_frame, 1);
  yaw_column = std::sin(upstream_pitch) * column(downstream_frame, 0) +
    std::cos(upstream_pitch) * column(downstream_frame, 2);
}

}  // namespace asr_sdm_kinematic_dynamic_model

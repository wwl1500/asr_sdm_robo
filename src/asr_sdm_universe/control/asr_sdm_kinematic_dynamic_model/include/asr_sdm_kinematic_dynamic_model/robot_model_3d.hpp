#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ROBOT_MODEL_3D_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ROBOT_MODEL_3D_HPP_

#include "asr_sdm_kinematic_dynamic_model/kinematic_types.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

struct RobotModel3DParameters
{
  double link_length{0.25};
};

class RobotModel3D
{
public:
  explicit RobotModel3D(const RobotModel3DParameters & params);

  KinematicState3D makeInitialState() const;
  void refreshGeometry(KinematicState3D & state) const;

  std::array<Mat3, kNumLinks> linkFrames(
    const Mat3 & head_frame, const JointConfiguration3D & joints) const;
  std::array<Vec3, kNumLinks> linkAxes(const std::array<Mat3, kNumLinks> & frames) const;
  std::array<Vec3, kNumBodyPoints> bodyPoints(
    const Vec3 & head_point, const std::array<Vec3, kNumLinks> & axes) const;

  double linkLength() const;

private:
  RobotModel3DParameters params_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_ROBOT_MODEL_3D_HPP_

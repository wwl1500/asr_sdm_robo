#include "asr_sdm_kinematic_dynamic_model/robot_model_3d.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

RobotModel3D::RobotModel3D(const RobotModel3DParameters & params)
: params_(params)
{
}

KinematicState3D RobotModel3D::makeInitialState() const
{
  KinematicState3D state;
  state.head_frame = identityFrame();
  state.joints.theta.fill(0.0);
  refreshGeometry(state);
  return state;
}

void RobotModel3D::refreshGeometry(KinematicState3D & state) const
{
  state.link_frames = linkFrames(state.head_frame, state.joints);
  state.link_axes = linkAxes(state.link_frames);
  state.body_points = bodyPoints(state.head_position, state.link_axes);
}

std::array<Mat3, kNumLinks> RobotModel3D::linkFrames(
  const Mat3 & head_frame, const JointConfiguration3D & joints) const
{
  std::array<Mat3, kNumLinks> frames{};
  frames[0] = head_frame;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    frames[i + 1] = multiply(
      multiply(frames[i], rotationZ(-joints.theta[yawIndex(i)])),
      rotationY(-joints.theta[pitchIndex(i)]));
  }
  return frames;
}

std::array<Vec3, kNumLinks> RobotModel3D::linkAxes(
  const std::array<Mat3, kNumLinks> & frames) const
{
  std::array<Vec3, kNumLinks> axes{};
  for (std::size_t i = 0; i < kNumLinks; ++i) {
    axes[i] = column(frames[i], 0);
  }
  return axes;
}

std::array<Vec3, kNumBodyPoints> RobotModel3D::bodyPoints(
  const Vec3 & head_point, const std::array<Vec3, kNumLinks> & axes) const
{
  std::array<Vec3, kNumBodyPoints> points{};
  points[0] = head_point;
  for (std::size_t i = 0; i < kNumLinks; ++i) {
    points[i + 1] = points[i] - params_.link_length * axes[i];
  }
  return points;
}

double RobotModel3D::linkLength() const
{
  return params_.link_length;
}

}  // namespace asr_sdm_kinematic_dynamic_model

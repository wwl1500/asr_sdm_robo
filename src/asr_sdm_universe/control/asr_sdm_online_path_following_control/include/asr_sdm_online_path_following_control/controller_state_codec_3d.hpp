#ifndef CONTROLLER_STATE_CODEC_3D_HPP_
#define CONTROLLER_STATE_CODEC_3D_HPP_

#include "asr_sdm_head_following_control/front_unit_following_controller_3d.hpp"

#include <vector>

namespace asr
{

constexpr size_t kControllerStateSize3D = 43;

struct ControllerStateMessage3D
{
  double time{0.0};
  Vec3 head_position{};
  Mat3 head_frame{};
  HeadCommand3D command{};
  std::array<double, kNum3dJointDofs> theta{};
  std::array<double, kNum3dJointDofs> theta_dot{};
  std::array<Vec3, kNum3dPoints> body_points{};
};

inline void encodeControllerState3D(
  const SimulationState3D & state, const HeadCommand3D & command,
  const JointVelocity3D & joint_velocity, std::vector<double> & data)
{
  data.assign(kControllerStateSize3D, 0.0);
  data[0] = state.time;
  data[1] = state.head_position.x;
  data[2] = state.head_position.y;
  data[3] = state.head_position.z;

  size_t index = 4;
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      data[index++] = state.head_frame.v[row][col];
    }
  }

  data[13] = command.linear_velocity;
  data[14] = command.pitch_rate;
  data[15] = command.yaw_rate;
  for (size_t i = 0; i < kNum3dJointDofs; ++i) {
    data[16 + i] = state.joints.theta[i];
    data[22 + i] = joint_velocity.theta_dot[i];
  }
  for (size_t point = 0; point < kNum3dPoints; ++point) {
    data[28 + 3 * point] = state.body_points[point].x;
    data[28 + 3 * point + 1] = state.body_points[point].y;
    data[28 + 3 * point + 2] = state.body_points[point].z;
  }
}

inline bool decodeControllerState3D(
  const std::vector<double> & data, ControllerStateMessage3D & state)
{
  if (data.size() < kControllerStateSize3D) {
    return false;
  }

  state.time = data[0];
  state.head_position = {data[1], data[2], data[3]};

  size_t index = 4;
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      state.head_frame.v[row][col] = data[index++];
    }
  }

  state.command = {data[13], data[14], data[15]};
  for (size_t i = 0; i < kNum3dJointDofs; ++i) {
    state.theta[i] = data[16 + i];
    state.theta_dot[i] = data[22 + i];
  }
  for (size_t point = 0; point < kNum3dPoints; ++point) {
    state.body_points[point] = {
      data[28 + 3 * point],
      data[28 + 3 * point + 1],
      data[28 + 3 * point + 2]};
  }
  return true;
}

}  // namespace asr

#endif  // CONTROLLER_STATE_CODEC_3D_HPP_

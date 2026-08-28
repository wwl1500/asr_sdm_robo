#include "asr_sdm_kinematic_dynamic_model/jacobian_3d.hpp"
#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/robot_model_3d.hpp"
#include "asr_sdm_kinematic_dynamic_model/screw_drive_model.hpp"

#include <cmath>
#include <iostream>

namespace
{
namespace model = asr_sdm_kinematic_dynamic_model;

bool check(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

}  // namespace

int main()
{
  model::RobotModel3D robot({0.25});
  auto state = robot.makeInitialState();
  bool passed = true;
  passed &= check(state.body_points.size() == model::kNumBodyPoints, "body point count");
  for (std::size_t i = 0; i < model::kNumLinks; ++i) {
    passed &= check(
      std::abs(state.body_points[i + 1].x - state.body_points[i].x + 0.25) < 1.0e-12 &&
      std::abs(state.body_points[i + 1].y - state.body_points[i].y) < 1.0e-12 &&
      std::abs(state.body_points[i + 1].z - state.body_points[i].z) < 1.0e-12,
      "initial body point direction");
  }

  state.joints.theta[model::yawIndex(0)] = 0.4;
  state.joints.theta[model::pitchIndex(0)] = -0.3;
  robot.refreshGeometry(state);
  const auto expected = model::multiply(
    model::rotationZ(-0.4), model::rotationY(0.3));
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      passed &= check(
        std::abs(state.link_frames[1].v[row][col] - expected.v[row][col]) < 1.0e-12,
        "yaw-pitch frame order");
    }
  }

  model::Vec3 pitch_column;
  model::Vec3 yaw_column;
  model::Jacobian3D::pitchYawColumns(
    state.link_frames[1], -0.3, pitch_column, yaw_column);
  passed &= check(
    std::abs(model::dot(pitch_column, model::column(state.link_frames[1], 1)) - 1.0) < 1.0e-12,
    "pitch Jacobian axis");

  model::ScrewDriveModel screw_drive({{0.0, 1.5707963267948966, 0.0,
      1.5707963267948966, 0.0, 1.5707963267948966}});
  model::JointVelocity3D joint_velocity;
  joint_velocity.theta_dot = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const auto screw_velocity = screw_drive.computeScrewVelocity(joint_velocity);
  passed &= check(
    std::abs(screw_velocity.theta_dot[0]) < 1.0e-12 &&
    std::abs(screw_velocity.theta_dot[1] - 2.0) < 1.0e-12 &&
    std::abs(screw_velocity.theta_dot[4]) < 1.0e-12 &&
    std::abs(screw_velocity.theta_dot[5] - 6.0) < 1.0e-12,
    "screw velocity mapping");

#ifdef ASR_SDM_GENERATED_URDF
  model::PinocchioModelParameters pinocchio_params;
  pinocchio_params.urdf_path = ASR_SDM_GENERATED_URDF;
  model::PinocchioModel pinocchio_model(pinocchio_params);
  passed &= check(pinocchio_model.isValid(), "generated URDF loads in Pinocchio");
  if (pinocchio_model.isValid()) {
    passed &= check(pinocchio_model.nq() == model::kReducedNq, "reduced configuration dimension");
    passed &= check(pinocchio_model.nv() == model::kReducedNv, "reduced velocity dimension");
    const auto mapping = pinocchio_model.controllerDofMapping();
    for (std::size_t i = 0; i < mapping.size(); ++i) {
      passed &= check(
        mapping[i].q_index >= 0 && mapping[i].v_index >= 0 &&
        mapping[i].joint_name == pinocchio_params.controller_joint_names[i],
        "controller-to-Pinocchio mapping");
    }
    pinocchio_model.setControllerState(state.joints, joint_velocity);
    const auto dynamics = pinocchio_model.computeDynamics();
    passed &= check(
      dynamics.mass_matrix.rows() == pinocchio_model.nv() &&
      dynamics.mass_matrix.cols() == pinocchio_model.nv() && dynamics.total_mass > 0.0,
      "Pinocchio dynamics state");
    const auto q = pinocchio_model.neutralConfiguration();
    const auto v = pinocchio_model.zeroVelocity();
    const auto kinematics = pinocchio_model.computeKinematics(q, v);
    for (std::size_t link = 0; link < model::kNumLinks; ++link) {
      passed &= check(
        kinematics.segment_jacobians[link].rows() == 6 &&
        kinematics.segment_jacobians[link].cols() == model::kReducedNv &&
        kinematics.segment_jacobians[link].array().isFinite().all(),
        "Pinocchio segment kinematics");
    }
    const auto integrated = pinocchio_model.integrate(q, v);
    passed &= check(std::abs(integrated.segment<4>(3).norm() - 1.0) < 1.0e-12,
      "Pinocchio quaternion integration");
  }
#endif

  std::cout << (passed ? "3D model regression passed" : "3D model regression failed") << std::endl;
  return passed ? 0 : 1;
}

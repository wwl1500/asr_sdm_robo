#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_PINOCCHIO_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_PINOCCHIO_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/kinematic_types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace asr_sdm_kinematic_dynamic_model
{

constexpr int kReducedNq = 13;
constexpr int kReducedNv = 12;
using ReducedConfiguration = Eigen::Matrix<double, kReducedNq, 1>;
using ReducedVelocity = Eigen::Matrix<double, kReducedNv, 1>;
using ReducedAcceleration = Eigen::Matrix<double, kReducedNv, 1>;
using ReducedJacobian = Eigen::Matrix<double, 6, kReducedNv>;

struct PinocchioControllerDofMapping
{
  std::string joint_name;
  int q_index{-1};
  int v_index{-1};
};

struct PinocchioModelParameters
{
  std::string urdf_path;
  bool use_free_flyer{true};
  bool lock_rotor_joints{true};
  std::array<std::string, kNumJointDofs> controller_joint_names{
    "joint_joint_unit_a__joint_unit_cross__0",
    "joint_joint_unit_cross__joint_unit_b__0",
    "joint_joint_unit_a__joint_unit_cross__1",
    "joint_joint_unit_cross__joint_unit_b__1",
    "joint_joint_unit_a__joint_unit_cross__2",
    "joint_joint_unit_cross__joint_unit_b__2"};
  std::array<std::string, kNumLinks> segment_frame_names{
    "screwdrive_segment_0", "screwdrive_segment_1", "screwdrive_segment_2",
    "screwdrive_segment_3"};
  std::array<std::array<std::string, 2>, kNumLinks> rotor_frame_names{{
    {{"screw_rotor_left_0", "screw_rotor_right_0"}},
    {{"screw_rotor_left_1", "screw_rotor_right_1"}},
    {{"screw_rotor_left_2", "screw_rotor_right_2"}},
    {{"screw_rotor_left_3", "screw_rotor_right_3"}}}};
  std::vector<double> initial_joint_positions;
};

struct PinocchioDynamicsState
{
  Eigen::Matrix<double, kReducedNv, kReducedNv> mass_matrix =
    Eigen::Matrix<double, kReducedNv, kReducedNv>::Zero();
  Eigen::Matrix<double, kReducedNv, 1> nonlinear_effects =
    Eigen::Matrix<double, kReducedNv, 1>::Zero();
  Eigen::Matrix<double, kReducedNv, 1> gravity =
    Eigen::Matrix<double, kReducedNv, 1>::Zero();
  Eigen::Vector3d center_of_mass{Eigen::Vector3d::Zero()};
  double total_mass{0.0};
};

struct PinocchioKinematicsState
{
  std::array<Eigen::Isometry3d, kNumLinks> segment_placements{};
  std::array<ReducedJacobian, kNumLinks> segment_jacobians{};
  std::array<ReducedJacobian, kNumLinks> segment_jacobian_time_variations{};
  std::array<std::array<Eigen::Isometry3d, 2>, kNumLinks> rotor_placements{};
  std::array<std::array<ReducedJacobian, 2>, kNumLinks> rotor_jacobians{};
};

class PinocchioModel
{
public:
  explicit PinocchioModel(const PinocchioModelParameters & params);

  bool isValid() const;
  const std::string & error() const;
  int nq() const;
  int nv() const;
  std::array<PinocchioControllerDofMapping, kNumJointDofs> controllerDofMapping() const;

  ReducedConfiguration neutralConfiguration() const;
  ReducedVelocity zeroVelocity() const;
  ReducedConfiguration configuration() const;
  ReducedVelocity velocity() const;
  void setState(const ReducedConfiguration & configuration, const ReducedVelocity & velocity);
  void setControllerState(
    const JointConfiguration3D & configuration, const JointVelocity3D & velocity);

  PinocchioDynamicsState computeDynamics() const;
  PinocchioDynamicsState computeDynamics(
    const ReducedConfiguration & configuration, const ReducedVelocity & velocity) const;
  PinocchioKinematicsState computeKinematics(
    const ReducedConfiguration & configuration, const ReducedVelocity & velocity) const;
  ReducedConfiguration integrate(
    const ReducedConfiguration & configuration, const ReducedVelocity & velocity) const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_PINOCCHIO_MODEL_HPP_

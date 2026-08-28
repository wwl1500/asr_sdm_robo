#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_FLUID_FORCE_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_FLUID_FORCE_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/kinematic_types.hpp"

#include <Eigen/Core>

#include <array>
#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

constexpr int kSpatialDofs = 6;
constexpr int kGeneralizedDofs = 2 * kSpatialDofs;

using SpatialVector = Eigen::Matrix<double, kSpatialDofs, 1>;
using SpatialMatrix = Eigen::Matrix<double, kSpatialDofs, kSpatialDofs>;
using GeneralizedVector = Eigen::Matrix<double, kGeneralizedDofs, 1>;
using LinkJacobian = Eigen::Matrix<double, kSpatialDofs, kGeneralizedDofs>;

// Spatial vectors use [linear; angular] ordering. Forces and moments use
// the same ordering as the corresponding local link twist.
struct BaseTwist3D
{
  Eigen::Vector3d linear_velocity_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_world = Eigen::Vector3d::Zero();
};

struct BaseAcceleration3D
{
  Eigen::Vector3d linear_acceleration_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration_world = Eigen::Vector3d::Zero();
};

struct JointAcceleration3D
{
  std::array<double, kNumJointDofs> theta_ddot{};
};

struct HydrodynamicLinkParameters
{
  // Rigid-body properties are referenced to the link-frame origin. The
  // rotational inertia is specified about the center of mass.
  double mass{0.0};
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia_about_center_of_mass = Eigen::Matrix3d::Zero();

  // Added mass is a local, symmetric positive-semidefinite spatial matrix.
  SpatialMatrix added_mass = SpatialMatrix::Zero();

  // Nonnegative local damping coefficients. The nonlinear term is
  // -coefficient * abs(V[c]) * V[c], component by component.
  SpatialVector linear_damping = SpatialVector::Zero();
  SpatialVector quadratic_damping = SpatialVector::Zero();

  double displaced_volume{0.0};
  Eigen::Vector3d center_of_buoyancy = Eigen::Vector3d::Zero();
};

struct HydrodynamicModelParameters
{
  double fluid_density{1000.0};
  Eigen::Vector3d gravity_world{0.0, 0.0, -9.81};
  double link_length{0.25};
  double validation_tolerance{1.0e-10};
  std::array<HydrodynamicLinkParameters, kNumLinks> links{};
};

struct HydrodynamicEvaluation
{
  std::array<LinkJacobian, kNumLinks> local_jacobians{};
  std::array<SpatialVector, kNumLinks> local_twists{};
  std::array<SpatialVector, kNumLinks> relative_twists{};
  std::array<SpatialVector, kNumLinks> local_accelerations{};

  std::array<SpatialVector, kNumLinks> added_mass_wrenches{};
  std::array<SpatialVector, kNumLinks> linear_damping_wrenches{};
  std::array<SpatialVector, kNumLinks> quadratic_damping_wrenches{};
  std::array<SpatialVector, kNumLinks> gravity_wrenches{};
  std::array<SpatialVector, kNumLinks> buoyancy_wrenches{};
  std::array<SpatialVector, kNumLinks> total_link_wrenches{};
  std::array<GeneralizedVector, kNumLinks> generalized_link_forces{};

  GeneralizedVector added_mass_force = GeneralizedVector::Zero();
  GeneralizedVector damping_force = GeneralizedVector::Zero();
  GeneralizedVector gravity_buoyancy_force = GeneralizedVector::Zero();
  GeneralizedVector total_force = GeneralizedVector::Zero();
};

struct PinocchioKinematicsState;

struct PinocchioHydrodynamicEvaluation
{
  Eigen::Matrix<double, kGeneralizedDofs, kGeneralizedDofs> added_mass_matrix =
    Eigen::Matrix<double, kGeneralizedDofs, kGeneralizedDofs>::Zero();
  GeneralizedVector added_mass_bias_force = GeneralizedVector::Zero();
  GeneralizedVector added_mass_force = GeneralizedVector::Zero();
  GeneralizedVector damping_force = GeneralizedVector::Zero();
  GeneralizedVector buoyancy_force = GeneralizedVector::Zero();
  GeneralizedVector total_force = GeneralizedVector::Zero();

  std::array<SpatialVector, kNumLinks> relative_twists{};
  std::array<SpatialVector, kNumLinks> local_accelerations{};
  std::array<SpatialVector, kNumLinks> added_mass_wrenches{};
  std::array<SpatialVector, kNumLinks> damping_wrenches{};
  std::array<SpatialVector, kNumLinks> buoyancy_wrenches{};
  std::array<SpatialVector, kNumLinks> total_wrenches{};
};

class FluidForceModel
{
public:
  explicit FluidForceModel(const HydrodynamicModelParameters & params);

  bool isValid() const;
  const std::string & error() const;
  const HydrodynamicModelParameters & parameters() const;

  SpatialMatrix effectiveSpatialInertia(std::size_t link) const;

  // Evaluates link-local fluid and restoring wrenches and projects them into
  // [base linear, base angular, six internal joint coordinates]. The base
  // twist and current are expressed in the inertial/world frame. Link-local
  // accelerations use J * generalized_acceleration when explicit local
  // accelerations are not supplied; the J-dot term is intentionally left to
  // the caller because the kinematic package does not expose it yet.
  HydrodynamicEvaluation evaluate(
    const KinematicState3D & state,
    const BaseTwist3D & base_twist,
    const JointVelocity3D & joint_velocity,
    const Eigen::Vector3d & fluid_current_world = Eigen::Vector3d::Zero(),
    const BaseAcceleration3D & base_acceleration = BaseAcceleration3D{},
    const JointAcceleration3D & joint_acceleration = JointAcceleration3D{},
    const std::array<SpatialVector, kNumLinks> * explicit_local_accelerations = nullptr) const;

  PinocchioHydrodynamicEvaluation evaluatePinocchio(
    const PinocchioKinematicsState & kinematics,
    const GeneralizedVector & velocity,
    const GeneralizedVector & acceleration,
    const Eigen::Vector3d & fluid_current_world = Eigen::Vector3d::Zero()) const;

private:
  HydrodynamicModelParameters params_;
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_FLUID_FORCE_MODEL_HPP_

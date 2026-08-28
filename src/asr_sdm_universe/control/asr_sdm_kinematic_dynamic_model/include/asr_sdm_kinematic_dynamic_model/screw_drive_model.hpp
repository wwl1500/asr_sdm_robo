#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_SCREW_DRIVE_MODEL_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_SCREW_DRIVE_MODEL_HPP_

#include "asr_sdm_kinematic_dynamic_model/kinematic_types.hpp"

#include <array>
#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

struct ScrewDriveParameters
{
  /// Articulation-to-screw kinematic coupling, one entry per joint DOF.
  std::array<double, kNumJointDofs> alpha{
    -0.7853981633974483, 0.7853981633974483,
    -0.7853981633974483, 0.7853981633974483,
    -0.7853981633974483, 0.7853981633974483};

  /// Helix (lead) angle of each propulsion screw, measured from the plane normal to
  /// the screw axis. Must stay inside (0, pi/2).
  std::array<double, kNumRotors> helix_angle{
    0.7853981633974483, 0.7853981633974483, 0.7853981633974483, 0.7853981633974483,
    0.7853981633974483, 0.7853981633974483, 0.7853981633974483, 0.7853981633974483};

  /// Effective radius at which the helix acts, in metres.
  std::array<double, kNumRotors> screw_radius{
    0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};

  /// Screw handedness, +1 or -1. The two screws of one segment carry opposite signs,
  /// which is what makes their reaction torques cancel under symmetric thrust.
  std::array<double, kNumRotors> handedness{
    1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
};

struct ScrewVelocity3D
{
  std::array<double, kNumJointDofs> theta_dot{};
};

class ScrewDriveModel
{
public:
  explicit ScrewDriveModel(const ScrewDriveParameters & params = ScrewDriveParameters{});

  bool isValid() const;
  const std::string & error() const;
  const ScrewDriveParameters & parameters() const;

  ScrewVelocity3D computeScrewVelocity(const JointVelocity3D & joint_velocity) const;

  /// Axial advance speed of an ideal screw, u = omega * r * tan(alpha).
  double axialAdvanceSpeed(std::size_t rotor, double rotor_rate) const;

  /// Ideal-screw lever relating axial thrust to driving torque, tau = f * r * tan(alpha).
  /// Power balance tau * omega = f * u gives the same lever, so this is consistent with
  /// axialAdvanceSpeed().
  double reactionTorqueLever(std::size_t rotor) const;

  /// Sign of the torque the screw reacts back onto its segment, +1 or -1.
  double handedness(std::size_t rotor) const;

private:
  ScrewDriveParameters params_;
  std::array<double, kNumRotors> lever_{};
  std::string error_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_SCREW_DRIVE_MODEL_HPP_

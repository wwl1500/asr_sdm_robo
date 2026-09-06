#include "asr_sdm_kinematic_dynamic_model/screw_drive_model.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

constexpr double kHalfPi = 1.5707963267948966;

}  // namespace

ScrewDriveModel::ScrewDriveModel(const ScrewDriveParameters & params)
: params_(params)
{
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    if (!std::isfinite(params_.alpha[i])) {
      error_ = "Screw-drive alpha must be finite";
      return;
    }
  }
  for (std::size_t rotor = 0; rotor < kNumRotors; ++rotor) {
    const double helix_angle = params_.helix_angle[rotor];
    if (!std::isfinite(helix_angle) || helix_angle <= 0.0 || helix_angle >= kHalfPi) {
      std::ostringstream stream;
      stream << "Rotor " << rotor << " helix angle must lie inside (0, pi/2)";
      error_ = stream.str();
      return;
    }
    if (!std::isfinite(params_.screw_radius[rotor]) || params_.screw_radius[rotor] < 0.0) {
      std::ostringstream stream;
      stream << "Rotor " << rotor << " screw radius must be finite and nonnegative";
      error_ = stream.str();
      return;
    }
    if (!std::isfinite(params_.handedness[rotor]) || params_.handedness[rotor] == 0.0) {
      std::ostringstream stream;
      stream << "Rotor " << rotor << " handedness must be nonzero";
      error_ = stream.str();
      return;
    }
    params_.handedness[rotor] = params_.handedness[rotor] > 0.0 ? 1.0 : -1.0;
    lever_[rotor] = params_.screw_radius[rotor] * std::tan(helix_angle);
  }
}

bool ScrewDriveModel::isValid() const
{
  return error_.empty();
}

const std::string & ScrewDriveModel::error() const
{
  return error_;
}

const ScrewDriveParameters & ScrewDriveModel::parameters() const
{
  return params_;
}

ScrewVelocity3D ScrewDriveModel::computeScrewVelocity(
  const JointVelocity3D & joint_velocity) const
{
  ScrewVelocity3D output;
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    output.theta_dot[i] = joint_velocity.theta_dot[i] * std::sin(params_.alpha[i]);
  }
  return output;
}

double ScrewDriveModel::axialAdvanceSpeed(std::size_t rotor, double rotor_rate) const
{
  if (rotor >= kNumRotors) {
    throw std::out_of_range("Screw rotor index is out of range");
  }
  return rotor_rate * lever_[rotor];
}

double ScrewDriveModel::reactionTorqueLever(std::size_t rotor) const
{
  if (rotor >= kNumRotors) {
    throw std::out_of_range("Screw rotor index is out of range");
  }
  return lever_[rotor];
}

double ScrewDriveModel::handedness(std::size_t rotor) const
{
  if (rotor >= kNumRotors) {
    throw std::out_of_range("Screw rotor index is out of range");
  }
  return params_.handedness[rotor];
}

}  // namespace asr_sdm_kinematic_dynamic_model

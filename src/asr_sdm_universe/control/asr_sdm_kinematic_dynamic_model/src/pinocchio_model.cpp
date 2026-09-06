#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace asr_sdm_kinematic_dynamic_model
{

struct PinocchioModel::Impl
{
  std::shared_ptr<pinocchio::Model> model;
  std::shared_ptr<pinocchio::Data> data;
  ReducedConfiguration q{ReducedConfiguration::Zero()};
  ReducedVelocity v{ReducedVelocity::Zero()};
  std::array<PinocchioControllerDofMapping, kNumJointDofs> controller_dof_mapping{};
  std::array<pinocchio::FrameIndex, kNumLinks> segment_frames{};
  std::array<std::array<pinocchio::FrameIndex, 2>, kNumLinks> rotor_frames{};
};

namespace
{

std::string describeMappingError(const std::string & joint_name, const std::string & reason)
{
  return "Invalid controller-to-Pinocchio mapping for joint '" + joint_name + "': " + reason;
}

bool isRotorJoint(const std::string & name)
{
  return name.find("screw_rotor") != std::string::npos;
}

Eigen::Isometry3d toEigen(const pinocchio::SE3 & placement)
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.matrix() = placement.toHomogeneousMatrix();
  return output;
}

}  // namespace

PinocchioModel::PinocchioModel(const PinocchioModelParameters & params)
: impl_(std::make_shared<Impl>())
{
  if (params.urdf_path.empty()) {
    error_ = "URDF path is empty";
    return;
  }
  if (!std::ifstream(params.urdf_path)) {
    error_ = "URDF file not found: " + params.urdf_path;
    return;
  }
  if (!params.gravity_world.array().isFinite().all()) {
    error_ = "Gravity must contain only finite values";
    return;
  }
  gravity_world_ = params.gravity_world;

  auto full_model = std::make_shared<pinocchio::Model>();
  try {
    if (params.use_free_flyer) {
      pinocchio::urdf::buildModel(
        params.urdf_path, pinocchio::JointModelFreeFlyer(), *full_model);
    } else {
      pinocchio::urdf::buildModel(params.urdf_path, *full_model);
    }
  } catch (const std::exception & e) {
    error_ = std::string("Pinocchio failed to build model: ") + e.what();
    return;
  }

  Eigen::VectorXd full_q = pinocchio::neutral(*full_model);
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    const std::string & joint_name = params.controller_joint_names[i];
    if (!full_model->existJointName(joint_name)) {
      error_ = describeMappingError(joint_name, "joint name does not exist in the URDF");
      return;
    }
    const pinocchio::JointIndex joint_id = full_model->getJointId(joint_name);
    const auto & joint_model = full_model->joints[joint_id];
    if (joint_model.nq() != 1 || joint_model.nv() != 1) {
      std::ostringstream stream;
      stream << "expected nq=1 and nv=1, got nq=" << joint_model.nq() <<
        " nv=" << joint_model.nv();
      error_ = describeMappingError(joint_name, stream.str());
      return;
    }
    const std::size_t count = std::min(params.initial_joint_positions.size(), kNumJointDofs);
    if (i < count) {
      full_q[static_cast<Eigen::Index>(joint_model.idx_q())] = params.initial_joint_positions[i];
    }
  }

  try {
    if (params.lock_rotor_joints) {
      std::vector<pinocchio::JointIndex> locked_joints;
      for (pinocchio::JointIndex joint = 1;
        joint < static_cast<pinocchio::JointIndex>(full_model->njoints); ++joint)
      {
        if (isRotorJoint(full_model->names[joint])) {
          locked_joints.push_back(joint);
        }
      }
      if (locked_joints.size() != 2 * kNumLinks) {
        std::ostringstream stream;
        stream << "Expected " << 2 * kNumLinks << " rotor joints, found " <<
          locked_joints.size();
        error_ = stream.str();
        return;
      }
      impl_->model = std::make_shared<pinocchio::Model>(
        pinocchio::buildReducedModel(*full_model, locked_joints, full_q));
    } else {
      impl_->model = full_model;
    }
  } catch (const std::exception & e) {
    error_ = std::string("Pinocchio failed to reduce model: ") + e.what();
    return;
  }

  if (params.use_free_flyer && params.lock_rotor_joints &&
    (impl_->model->nq != kReducedNq || impl_->model->nv != kReducedNv))
  {
    std::ostringstream stream;
    stream << "Reduced model has nq=" << impl_->model->nq << " nv=" << impl_->model->nv <<
      ", expected nq=" << kReducedNq << " nv=" << kReducedNv;
    error_ = stream.str();
    return;
  }
  if (impl_->model->nq != kReducedNq || impl_->model->nv != kReducedNv) {
    error_ = "The simulator requires a free-flyer model with eight locked rotor joints";
    return;
  }

  impl_->model->gravity.linear() = gravity_world_;
  impl_->model->gravity.angular().setZero();

  impl_->data = std::make_shared<pinocchio::Data>(*impl_->model);
  impl_->q = pinocchio::neutral(*impl_->model);
  impl_->v.setZero();

  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    const std::string & joint_name = params.controller_joint_names[i];
    if (!impl_->model->existJointName(joint_name)) {
      error_ = describeMappingError(joint_name, "joint was not retained in the reduced model");
      impl_->data.reset();
      impl_->model.reset();
      return;
    }
    const pinocchio::JointIndex joint_id = impl_->model->getJointId(joint_name);
    const auto & joint_model = impl_->model->joints[joint_id];
    if (joint_model.nq() != 1 || joint_model.nv() != 1) {
      error_ = describeMappingError(joint_name, "reduced joint is not scalar");
      impl_->data.reset();
      impl_->model.reset();
      return;
    }
    impl_->controller_dof_mapping[i] = {
      joint_name, static_cast<int>(joint_model.idx_q()), static_cast<int>(joint_model.idx_v())};
    const std::size_t count = std::min(params.initial_joint_positions.size(), kNumJointDofs);
    if (i < count) {
      impl_->q[impl_->controller_dof_mapping[i].q_index] = params.initial_joint_positions[i];
    }
  }

  for (std::size_t link = 0; link < kNumLinks; ++link) {
    const std::string & segment_name = params.segment_frame_names[link];
    if (!impl_->model->existFrame(segment_name)) {
      error_ = "Segment frame does not exist: " + segment_name;
      impl_->data.reset();
      impl_->model.reset();
      return;
    }
    impl_->segment_frames[link] = impl_->model->getFrameId(segment_name);
    for (std::size_t rotor = 0; rotor < 2; ++rotor) {
      const std::string & rotor_name = params.rotor_frame_names[link][rotor];
      if (!impl_->model->existFrame(rotor_name)) {
        error_ = "Rotor frame does not exist: " + rotor_name;
        impl_->data.reset();
        impl_->model.reset();
        return;
      }
      impl_->rotor_frames[link][rotor] = impl_->model->getFrameId(rotor_name);
    }
  }
}

bool PinocchioModel::isValid() const
{
  return impl_ != nullptr && impl_->model != nullptr && impl_->data != nullptr;
}

const std::string & PinocchioModel::error() const
{
  return error_;
}

int PinocchioModel::nq() const
{
  return isValid() ? impl_->model->nq : 0;
}

int PinocchioModel::nv() const
{
  return isValid() ? impl_->model->nv : 0;
}

const Eigen::Vector3d & PinocchioModel::gravityWorld() const
{
  return gravity_world_;
}

std::array<PinocchioControllerDofMapping, kNumJointDofs>
PinocchioModel::controllerDofMapping() const
{
  return isValid() ? impl_->controller_dof_mapping :
         std::array<PinocchioControllerDofMapping, kNumJointDofs>{};
}

ReducedConfiguration PinocchioModel::neutralConfiguration() const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot get configuration from an invalid Pinocchio model: " + error_);
  }
  return pinocchio::neutral(*impl_->model);
}

ReducedVelocity PinocchioModel::zeroVelocity() const
{
  return ReducedVelocity::Zero();
}

ReducedConfiguration PinocchioModel::configuration() const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot get configuration from an invalid Pinocchio model: " + error_);
  }
  return impl_->q;
}

ReducedVelocity PinocchioModel::velocity() const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot get velocity from an invalid Pinocchio model: " + error_);
  }
  return impl_->v;
}

void PinocchioModel::setState(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity)
{
  if (!isValid()) {
    throw std::runtime_error("Cannot set state on an invalid Pinocchio model: " + error_);
  }
  impl_->q = configuration;
  impl_->v = velocity;
}

void PinocchioModel::setControllerState(
  const JointConfiguration3D & configuration, const JointVelocity3D & velocity)
{
  if (!isValid()) {
    throw std::runtime_error("Cannot set an invalid Pinocchio model state: " + error_);
  }
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    impl_->q[impl_->controller_dof_mapping[i].q_index] = configuration.theta[i];
    impl_->v[impl_->controller_dof_mapping[i].v_index] = velocity.theta_dot[i];
  }
}

PinocchioDynamicsState PinocchioModel::computeDynamics() const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot compute dynamics for an invalid Pinocchio model: " + error_);
  }
  return computeDynamics(impl_->q, impl_->v);
}

PinocchioDynamicsState PinocchioModel::computeDynamics(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot compute dynamics for an invalid Pinocchio model: " + error_);
  }

  PinocchioDynamicsState output;

  // Every algorithm below writes into the shared Data, so each result is copied out
  // before the next call runs.
  pinocchio::crba(*impl_->model, *impl_->data, configuration);
  impl_->data->M.triangularView<Eigen::StrictlyLower>() = impl_->data->M.transpose();
  output.mass_matrix = impl_->data->M;

  output.coriolis_matrix =
    pinocchio::computeCoriolisMatrix(*impl_->model, *impl_->data, configuration, velocity);
  output.coriolis_force = output.coriolis_matrix * velocity;
  output.nonlinear_effects =
    pinocchio::nonLinearEffects(*impl_->model, *impl_->data, configuration, velocity);
  output.gravity =
    pinocchio::computeGeneralizedGravity(*impl_->model, *impl_->data, configuration);
  output.center_of_mass =
    pinocchio::centerOfMass(*impl_->model, *impl_->data, configuration, velocity);
  output.total_mass = pinocchio::computeTotalMass(*impl_->model);
  output.kinetic_energy =
    pinocchio::computeKineticEnergy(*impl_->model, *impl_->data, configuration, velocity);
  output.potential_energy =
    pinocchio::computePotentialEnergy(*impl_->model, *impl_->data, configuration);
  return output;
}

ReducedVelocity PinocchioModel::inverseDynamics(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity,
  const ReducedAcceleration & acceleration) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot run inverse dynamics on an invalid Pinocchio model: " +
        error_);
  }
  if (!configuration.array().isFinite().all() || !velocity.array().isFinite().all() ||
    !acceleration.array().isFinite().all())
  {
    throw std::invalid_argument("Inverse-dynamics inputs must contain only finite values");
  }
  return pinocchio::rnea(
    *impl_->model, *impl_->data, configuration, velocity, acceleration);
}

ReducedAcceleration PinocchioModel::forwardDynamics(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity,
  const ReducedVelocity & torque) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot run forward dynamics on an invalid Pinocchio model: " +
        error_);
  }
  if (!configuration.array().isFinite().all() || !velocity.array().isFinite().all() ||
    !torque.array().isFinite().all())
  {
    throw std::invalid_argument("Forward-dynamics inputs must contain only finite values");
  }
  return pinocchio::aba(
    *impl_->model, *impl_->data, configuration, velocity, torque,
    pinocchio::Convention::WORLD);
}

PinocchioKinematicsState PinocchioModel::computeKinematics(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot compute kinematics for an invalid Pinocchio model: " + error_);
  }

  pinocchio::forwardKinematics(*impl_->model, *impl_->data, configuration, velocity);
  pinocchio::updateFramePlacements(*impl_->model, *impl_->data);
  pinocchio::computeJointJacobians(*impl_->model, *impl_->data, configuration);
  pinocchio::computeJointJacobiansTimeVariation(
    *impl_->model, *impl_->data, configuration, velocity);

  PinocchioKinematicsState output;
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    output.segment_placements[link] = toEigen(impl_->data->oMf[impl_->segment_frames[link]]);
    output.segment_jacobians[link].setZero();
    output.segment_jacobian_time_variations[link].setZero();
    pinocchio::getFrameJacobian(
      *impl_->model, *impl_->data, impl_->segment_frames[link], pinocchio::LOCAL,
      output.segment_jacobians[link]);
    pinocchio::getFrameJacobianTimeVariation(
      *impl_->model, *impl_->data, impl_->segment_frames[link], pinocchio::LOCAL,
      output.segment_jacobian_time_variations[link]);
    for (std::size_t rotor = 0; rotor < 2; ++rotor) {
      output.rotor_placements[link][rotor] =
        toEigen(impl_->data->oMf[impl_->rotor_frames[link][rotor]]);
      output.rotor_jacobians[link][rotor].setZero();
      pinocchio::getFrameJacobian(
        *impl_->model, *impl_->data, impl_->rotor_frames[link][rotor], pinocchio::LOCAL,
        output.rotor_jacobians[link][rotor]);
    }
  }
  return output;
}

ReducedConfiguration PinocchioModel::integrate(
  const ReducedConfiguration & configuration, const ReducedVelocity & velocity) const
{
  if (!isValid()) {
    throw std::runtime_error("Cannot integrate an invalid Pinocchio model: " + error_);
  }
  return pinocchio::integrate(*impl_->model, configuration, velocity);
}

}  // namespace asr_sdm_kinematic_dynamic_model

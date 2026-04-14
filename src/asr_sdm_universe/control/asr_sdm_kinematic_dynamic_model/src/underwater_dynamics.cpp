#include "asr_sdm_kinematic_dynamic_model/underwater_dynamics.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159265358979323846;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

std::string sizeMismatchMessage(
  const char * name, const size_t actual_size, const size_t expected_size)
{
  return std::string(name) + " must have size " + std::to_string(expected_size) +
         ", got " + std::to_string(actual_size);
}

}  // namespace

UnderwaterDynamics::UnderwaterDynamics(
  std::shared_ptr<pinocchio::Model> model,
  std::shared_ptr<pinocchio::Data> data,
  const HydrodynamicParameters & params)
: model_(std::move(model)),
  data_(std::move(data)),
  params_(params)
{
  const int nv = model_->nv;
  M_ = Eigen::MatrixXd::Zero(nv, nv);
  C_ = Eigen::MatrixXd::Zero(nv, nv);
  D_ = Eigen::MatrixXd::Zero(nv, nv);
  N_ = Eigen::VectorXd::Zero(nv);

  initializeHydrodynamicLinks();
  validateParameters();
}

bool UnderwaterDynamics::isNominalHydrodynamicLinkName(const std::string & name)
{
  if (name == "base_link") {
    return true;
  }

  static constexpr char kPrefix[] = "link_";
  if (name.rfind(kPrefix, 0) != 0) {
    return false;
  }

  if (name.size() <= sizeof(kPrefix) - 1) {
    return false;
  }

  return std::all_of(
    name.begin() + static_cast<std::ptrdiff_t>(sizeof(kPrefix) - 1), name.end(),
    [](const unsigned char ch) { return std::isdigit(ch) != 0; });
}

int UnderwaterDynamics::hydrodynamicLinkSortKey(const std::string & name)
{
  if (name == "base_link") {
    return 1;
  }

  return std::stoi(name.substr(5));
}

void UnderwaterDynamics::initializeHydrodynamicLinks()
{
  hydrodynamic_links_.clear();

  if (!params_.hydrodynamic_link_names.empty()) {
    for (const auto & name : params_.hydrodynamic_link_names) {
      const bool exists = model_->existFrame(name, pinocchio::BODY);
      if (!exists) {
        throw std::invalid_argument("Hydrodynamic body frame not found: " + name);
      }

      hydrodynamic_links_.push_back({name, model_->getFrameId(name, pinocchio::BODY)});
    }
    return;
  }

  for (pinocchio::FrameIndex frame_id = 0;
    frame_id < static_cast<pinocchio::FrameIndex>(model_->nframes); ++frame_id)
  {
    const auto & frame = model_->frames[frame_id];
    if (frame.type != pinocchio::BODY) {
      continue;
    }

    if (!isNominalHydrodynamicLinkName(frame.name)) {
      continue;
    }

    hydrodynamic_links_.push_back({frame.name, frame_id});
  }

  std::sort(
    hydrodynamic_links_.begin(), hydrodynamic_links_.end(),
    [](const HydrodynamicLink & lhs, const HydrodynamicLink & rhs) {
      return hydrodynamicLinkSortKey(lhs.name) < hydrodynamicLinkSortKey(rhs.name);
    });

  if (hydrodynamic_links_.empty()) {
    throw std::invalid_argument("No hydrodynamic body links were found in the model.");
  }
}

void UnderwaterDynamics::validateParameters() const
{
  const size_t link_count = hydrodynamic_links_.size();

  auto ensure_size_or_empty =
    [link_count](const std::vector<double> & values, const char * name) {
      if (!values.empty() && values.size() != link_count) {
        throw std::invalid_argument(sizeMismatchMessage(name, values.size(), link_count));
      }
    };

  ensure_size_or_empty(params_.link_volumes, "link_volumes");
  ensure_size_or_empty(params_.link_radii, "link_radii");
  ensure_size_or_empty(params_.link_lengths, "link_lengths");
  ensure_size_or_empty(params_.added_mass_factors, "added_mass_factors");

  if (params_.link_radii.empty() != params_.link_lengths.empty()) {
    throw std::invalid_argument(
      "link_radii and link_lengths must either both be empty or both have one entry per body link.");
  }

  if (params_.link_volumes.empty() && !params_.added_mass_factors.empty() &&
      params_.link_added_mass_diagonal.empty())
  {
    throw std::invalid_argument(
      "added_mass_factors require link_volumes unless link_added_mass_diagonal is provided.");
  }

  const size_t expected_flattened_size = 6 * link_count;
  if (!params_.link_added_mass_diagonal.empty() &&
      params_.link_added_mass_diagonal.size() != expected_flattened_size)
  {
    throw std::invalid_argument(
      sizeMismatchMessage(
        "link_added_mass_diagonal", params_.link_added_mass_diagonal.size(),
        expected_flattened_size));
  }

  if (!params_.link_linear_damping_diagonal.empty() &&
      params_.link_linear_damping_diagonal.size() != expected_flattened_size)
  {
    throw std::invalid_argument(
      sizeMismatchMessage(
        "link_linear_damping_diagonal", params_.link_linear_damping_diagonal.size(),
        expected_flattened_size));
  }
}

Matrix6d UnderwaterDynamics::buildAddedMassMatrix(const size_t link_index) const
{
  Matrix6d added_mass = Matrix6d::Zero();

  if (!params_.link_added_mass_diagonal.empty()) {
    const size_t offset = 6 * link_index;
    for (size_t axis = 0; axis < 6; ++axis) {
      added_mass(static_cast<Eigen::Index>(axis), static_cast<Eigen::Index>(axis)) =
        params_.link_added_mass_diagonal[offset + axis];
    }
    return added_mass;
  }

  if (params_.link_volumes.empty()) {
    return added_mass;
  }

  const double factor =
    params_.added_mass_factors.empty() ? 1.0 : params_.added_mass_factors.at(link_index);
  const double displaced_fluid_mass = params_.rho * params_.link_volumes.at(link_index) * factor;

  // The derivation keeps M_A,i abstract. The default cylindrical realization follows the
  // transverse added-mass estimate and leaves axial/rotational terms at zero unless configured.
  added_mass(1, 1) = displaced_fluid_mass;
  added_mass(2, 2) = displaced_fluid_mass;
  return added_mass;
}

Matrix6d UnderwaterDynamics::buildLinearDampingMatrix(const size_t link_index) const
{
  Matrix6d linear_damping = Matrix6d::Zero();
  if (params_.link_linear_damping_diagonal.empty()) {
    return linear_damping;
  }

  const size_t offset = 6 * link_index;
  for (size_t axis = 0; axis < 6; ++axis) {
    linear_damping(static_cast<Eigen::Index>(axis), static_cast<Eigen::Index>(axis)) =
      params_.link_linear_damping_diagonal[offset + axis];
  }

  return linear_damping;
}

Matrix6d UnderwaterDynamics::buildNonlinearDampingMatrix(
  const size_t link_index,
  const Eigen::Matrix<double, 6, 1> & local_velocity) const
{
  Matrix6d nonlinear_damping = Matrix6d::Zero();
  if (params_.link_radii.empty() || params_.link_lengths.empty()) {
    return nonlinear_damping;
  }

  const double radius = params_.link_radii.at(link_index);
  const double length = params_.link_lengths.at(link_index);
  const double normal_area = 2.0 * radius * length;
  const double tangential_area = kPi * radius * radius;

  const double d_n = 0.5 * params_.rho * params_.Cd_n * normal_area;
  const double d_t = 0.5 * params_.rho * params_.Cd_t * tangential_area;

  nonlinear_damping(0, 0) = d_t * std::abs(local_velocity(0));
  nonlinear_damping(1, 1) = d_n * std::abs(local_velocity(1));
  nonlinear_damping(2, 2) = d_n * std::abs(local_velocity(2));
  return nonlinear_damping;
}

void UnderwaterDynamics::computeDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  M_.setZero();
  C_.setZero();
  D_.setZero();
  N_.setZero();

  pinocchio::computeJointJacobians(*model_, *data_, q);
  pinocchio::forwardKinematics(*model_, *data_, q, v);
  pinocchio::updateFramePlacements(*model_, *data_);
  pinocchio::computeJointJacobiansTimeVariation(*model_, *data_, q, v);

  pinocchio::crba(*model_, *data_, q);
  data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose();
  M_ = data_->M;

  pinocchio::computeGeneralizedGravity(*model_, *data_, q);
  N_ = data_->g;

  pinocchio::computeCoriolisMatrix(*model_, *data_, q, v);
  C_ = data_->C;

  computeAddedMass();
  computeHydrodynamicDamping();
  computeBuoyancyAndGravity();
  computeCoriolis();
}

void UnderwaterDynamics::computeAddedMass()
{
  const int nv = model_->nv;

  for (size_t link_index = 0; link_index < hydrodynamic_links_.size(); ++link_index) {
    const Matrix6d added_mass = buildAddedMassMatrix(link_index);
    if (added_mass.isZero(0.0)) {
      continue;
    }

    Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(6, nv);
    pinocchio::getFrameJacobian(
      *model_, *data_, hydrodynamic_links_[link_index].frame_id, pinocchio::LOCAL, local_jacobian);

    M_.noalias() += local_jacobian.transpose() * added_mass * local_jacobian;
  }
}

void UnderwaterDynamics::computeHydrodynamicDamping()
{
  const int nv = model_->nv;

  for (size_t link_index = 0; link_index < hydrodynamic_links_.size(); ++link_index) {
    const auto frame_id = hydrodynamic_links_[link_index].frame_id;
    const Vector6d local_velocity =
      pinocchio::getFrameVelocity(*model_, *data_, frame_id, pinocchio::LOCAL).toVector();

    Matrix6d damping = buildLinearDampingMatrix(link_index);
    damping += buildNonlinearDampingMatrix(link_index, local_velocity);
    if (damping.isZero(0.0)) {
      continue;
    }

    Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(6, nv);
    pinocchio::getFrameJacobian(*model_, *data_, frame_id, pinocchio::LOCAL, local_jacobian);

    D_.noalias() += local_jacobian.transpose() * damping * local_jacobian;
  }
}

void UnderwaterDynamics::computeBuoyancyAndGravity()
{
  if (params_.link_volumes.empty()) {
    return;
  }

  const int nv = model_->nv;
  const Eigen::Vector3d buoyancy_force_world_unit(0.0, 0.0, 1.0);

  for (size_t link_index = 0; link_index < hydrodynamic_links_.size(); ++link_index) {
    const auto frame_id = hydrodynamic_links_[link_index].frame_id;
    const auto & world_to_frame = data_->oMf[frame_id];
    const double buoyancy_force = params_.rho * params_.link_volumes.at(link_index) * kGravity;

    Vector6d local_wrench = Vector6d::Zero();
    local_wrench.head<3>() =
      world_to_frame.rotation().transpose() * (buoyancy_force * buoyancy_force_world_unit);

    Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(6, nv);
    pinocchio::getFrameJacobian(*model_, *data_, frame_id, pinocchio::LOCAL, local_jacobian);

    N_.noalias() -= local_jacobian.transpose() * local_wrench;
  }
}

void UnderwaterDynamics::computeCoriolis()
{
  const int nv = model_->nv;

  for (size_t link_index = 0; link_index < hydrodynamic_links_.size(); ++link_index) {
    const Matrix6d added_mass = buildAddedMassMatrix(link_index);
    if (added_mass.isZero(0.0)) {
      continue;
    }

    Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(6, nv);
    Eigen::MatrixXd local_jacobian_dot = Eigen::MatrixXd::Zero(6, nv);
    const auto frame_id = hydrodynamic_links_[link_index].frame_id;

    pinocchio::getFrameJacobian(*model_, *data_, frame_id, pinocchio::LOCAL, local_jacobian);
    pinocchio::getFrameJacobianTimeVariation(
      *model_, *data_, frame_id, pinocchio::LOCAL, local_jacobian_dot);

    C_.noalias() += local_jacobian.transpose() * added_mass * local_jacobian_dot;
  }
}

}  // namespace asr_sdm_kinematic_dynamic_model

#include "asr_sdm_kinematic_dynamic_model/underwater_dynamics.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <gtest/gtest.h>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMatrixTol = 1e-9;
constexpr double kVectorTol = 1e-9;
constexpr double kPowerTol = 1e-9;

const std::array<std::string, 4> kHydrodynamicLinkNames = {
  "base_link", "link_2", "link_3", "link_4"};

const std::array<std::array<double, 6>, 4> kAddedMassDiagonalBlocks = {{
  {{0.11, 1.10, 1.20, 0.40, 0.50, 0.60}},
  {{0.21, 2.10, 2.20, 0.70, 0.80, 0.90}},
  {{0.31, 3.10, 3.20, 1.00, 1.10, 1.20}},
  {{0.41, 4.10, 4.20, 1.30, 1.40, 1.50}},
}};

const std::array<std::array<double, 6>, 4> kLinearDampingDiagonalBlocks = {{
  {{5.0, 6.0, 7.0, 0.1, 0.2, 0.3}},
  {{8.0, 9.0, 10.0, 0.4, 0.5, 0.6}},
  {{11.0, 12.0, 13.0, 0.7, 0.8, 0.9}},
  {{14.0, 15.0, 16.0, 1.0, 1.1, 1.2}},
}};

struct StateSample
{
  std::string name;
  Eigen::VectorXd q;
  Eigen::VectorXd v;
};

struct ReferenceTerms
{
  Eigen::MatrixXd rigid_body_mass;
  Eigen::MatrixXd rigid_body_coriolis;
  Eigen::VectorXd rigid_body_restoring;
  Eigen::MatrixXd added_mass;
  Eigen::MatrixXd added_mass_coriolis;
  Eigen::MatrixXd added_mass_mdot;
  Eigen::MatrixXd linear_damping;
  Eigen::MatrixXd nonlinear_damping;
  Eigen::VectorXd buoyancy_generalized;
};

std::shared_ptr<pinocchio::Model> loadModel()
{
  const auto package_share = ament_index_cpp::get_package_share_directory("asr_sdm_description");
  const auto urdf_path = package_share + "/urdf/underwater_snakerobot.urdf";

  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model);
  return model;
}

std::shared_ptr<pinocchio::Data> makeData(const pinocchio::Model & model)
{
  return std::make_shared<pinocchio::Data>(model);
}

std::vector<double> flattenDiagonalBlocks(
  const std::array<std::array<double, 6>, 4> & blocks)
{
  std::vector<double> flattened;
  flattened.reserve(blocks.size() * 6);

  for (const auto & block : blocks) {
    flattened.insert(flattened.end(), block.begin(), block.end());
  }

  return flattened;
}

HydrodynamicParameters makeBaseParameters(const bool explicit_link_names = true)
{
  HydrodynamicParameters params;
  if (explicit_link_names) {
    params.hydrodynamic_link_names.assign(
      kHydrodynamicLinkNames.begin(), kHydrodynamicLinkNames.end());
  }
  return params;
}

HydrodynamicParameters makeExplicitAddedMassParameters(
  const bool explicit_link_names = true)
{
  HydrodynamicParameters params = makeBaseParameters(explicit_link_names);
  params.link_added_mass_diagonal = flattenDiagonalBlocks(kAddedMassDiagonalBlocks);
  return params;
}

HydrodynamicParameters makeExplicitLinearDampingParameters(
  const bool explicit_link_names = true)
{
  HydrodynamicParameters params = makeBaseParameters(explicit_link_names);
  params.link_linear_damping_diagonal = flattenDiagonalBlocks(kLinearDampingDiagonalBlocks);
  return params;
}

HydrodynamicParameters makeQuadraticDampingParameters(
  const bool explicit_link_names = true)
{
  HydrodynamicParameters params = makeBaseParameters(explicit_link_names);
  params.rho = 997.0;
  params.Cd_n = 1.1;
  params.Cd_t = 0.07;
  params.link_radii = {0.051, 0.056, 0.061, 0.066};
  params.link_lengths = {0.28, 0.30, 0.32, 0.34};
  return params;
}

HydrodynamicParameters makeBuoyancyParameters(
  const bool explicit_link_names = true)
{
  HydrodynamicParameters params = makeBaseParameters(explicit_link_names);
  params.rho = 998.0;
  params.link_volumes = {0.0015, 0.0018, 0.0020, 0.0023};
  return params;
}

HydrodynamicParameters makeNeutralBuoyancyParameters(
  const bool explicit_link_names = true)
{
  HydrodynamicParameters params = makeBaseParameters(explicit_link_names);
  params.link_volumes = std::vector<double>(4, 2.0 / params.rho);
  return params;
}

void setJointTangentComponent(
  Eigen::VectorXd & tangent, const pinocchio::Model & model, const std::string & joint_name,
  const double value)
{
  if (!model.existJointName(joint_name)) {
    throw std::runtime_error("Joint not found in test model: " + joint_name);
  }

  const auto joint_id = model.getJointId(joint_name);
  tangent[model.joints[joint_id].idx_v()] = value;
}

StateSample makeStateOne(const pinocchio::Model & model)
{
  Eigen::VectorXd configuration_tangent = Eigen::VectorXd::Zero(model.nv);
  setJointTangentComponent(configuration_tangent, model, "joint1_pitch", 0.2);
  setJointTangentComponent(configuration_tangent, model, "joint1_yaw", -0.15);
  setJointTangentComponent(configuration_tangent, model, "joint2_pitch", 0.1);
  setJointTangentComponent(configuration_tangent, model, "joint2_yaw", 0.05);
  setJointTangentComponent(configuration_tangent, model, "joint3_pitch", -0.1);
  setJointTangentComponent(configuration_tangent, model, "joint3_yaw", 0.12);

  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(model.nv);
  velocity[0] = 0.1;
  velocity[1] = -0.05;
  velocity[2] = 0.02;
  velocity[3] = 0.03;
  velocity[4] = -0.02;
  velocity[5] = 0.04;
  setJointTangentComponent(velocity, model, "joint1_pitch", 0.5);
  setJointTangentComponent(velocity, model, "joint1_yaw", -0.4);
  setJointTangentComponent(velocity, model, "joint2_pitch", 0.3);
  setJointTangentComponent(velocity, model, "joint2_yaw", -0.2);
  setJointTangentComponent(velocity, model, "joint3_pitch", 0.1);
  setJointTangentComponent(velocity, model, "joint3_yaw", -0.05);

  return {"state_one", pinocchio::integrate(model, pinocchio::neutral(model), configuration_tangent), velocity};
}

StateSample makeStateTwo(const pinocchio::Model & model)
{
  Eigen::VectorXd configuration_tangent = Eigen::VectorXd::Zero(model.nv);
  configuration_tangent[0] = 0.12;
  configuration_tangent[1] = -0.08;
  configuration_tangent[2] = 0.05;
  configuration_tangent[3] = -0.04;
  configuration_tangent[4] = 0.03;
  configuration_tangent[5] = 0.02;
  setJointTangentComponent(configuration_tangent, model, "joint1_pitch", -0.22);
  setJointTangentComponent(configuration_tangent, model, "joint1_yaw", 0.18);
  setJointTangentComponent(configuration_tangent, model, "joint2_pitch", -0.14);
  setJointTangentComponent(configuration_tangent, model, "joint2_yaw", 0.09);
  setJointTangentComponent(configuration_tangent, model, "joint3_pitch", 0.16);
  setJointTangentComponent(configuration_tangent, model, "joint3_yaw", -0.11);

  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(model.nv);
  velocity[0] = -0.07;
  velocity[1] = 0.09;
  velocity[2] = -0.03;
  velocity[3] = 0.06;
  velocity[4] = 0.05;
  velocity[5] = -0.04;
  setJointTangentComponent(velocity, model, "joint1_pitch", -0.35);
  setJointTangentComponent(velocity, model, "joint1_yaw", 0.28);
  setJointTangentComponent(velocity, model, "joint2_pitch", -0.24);
  setJointTangentComponent(velocity, model, "joint2_yaw", 0.18);
  setJointTangentComponent(velocity, model, "joint3_pitch", -0.13);
  setJointTangentComponent(velocity, model, "joint3_yaw", 0.07);

  return {"state_two", pinocchio::integrate(model, pinocchio::neutral(model), configuration_tangent), velocity};
}

std::vector<StateSample> makeNonZeroStates(const pinocchio::Model & model)
{
  return {makeStateOne(model), makeStateTwo(model)};
}

StateSample makeZeroState(const pinocchio::Model & model)
{
  return {"zero_state", pinocchio::neutral(model), Eigen::VectorXd::Zero(model.nv)};
}

std::vector<pinocchio::FrameIndex> resolveHydrodynamicFrames(
  const pinocchio::Model & model, const HydrodynamicParameters & params)
{
  std::vector<pinocchio::FrameIndex> frame_ids;

  if (!params.hydrodynamic_link_names.empty()) {
    frame_ids.reserve(params.hydrodynamic_link_names.size());
    for (const auto & name : params.hydrodynamic_link_names) {
      frame_ids.push_back(model.getFrameId(name, pinocchio::BODY));
    }
    return frame_ids;
  }

  for (const auto & name : kHydrodynamicLinkNames) {
    frame_ids.push_back(model.getFrameId(name, pinocchio::BODY));
  }
  return frame_ids;
}

Matrix6d buildReferenceAddedMassMatrix(
  const HydrodynamicParameters & params, const size_t link_index)
{
  Matrix6d added_mass = Matrix6d::Zero();

  if (!params.link_added_mass_diagonal.empty()) {
    const size_t offset = 6 * link_index;
    for (size_t axis = 0; axis < 6; ++axis) {
      added_mass(static_cast<Eigen::Index>(axis), static_cast<Eigen::Index>(axis)) =
        params.link_added_mass_diagonal[offset + axis];
    }
    return added_mass;
  }

  if (params.link_volumes.empty()) {
    return added_mass;
  }

  const double factor =
    params.added_mass_factors.empty() ? 1.0 : params.added_mass_factors.at(link_index);
  const double displaced_fluid_mass = params.rho * params.link_volumes.at(link_index) * factor;
  added_mass(1, 1) = displaced_fluid_mass;
  added_mass(2, 2) = displaced_fluid_mass;
  return added_mass;
}

Matrix6d buildReferenceLinearDampingMatrix(
  const HydrodynamicParameters & params, const size_t link_index)
{
  Matrix6d linear_damping = Matrix6d::Zero();
  if (params.link_linear_damping_diagonal.empty()) {
    return linear_damping;
  }

  const size_t offset = 6 * link_index;
  for (size_t axis = 0; axis < 6; ++axis) {
    linear_damping(static_cast<Eigen::Index>(axis), static_cast<Eigen::Index>(axis)) =
      params.link_linear_damping_diagonal[offset + axis];
  }
  return linear_damping;
}

Matrix6d buildReferenceNonlinearDampingMatrix(
  const HydrodynamicParameters & params, const size_t link_index,
  const Vector6d & local_velocity)
{
  Matrix6d nonlinear_damping = Matrix6d::Zero();
  if (params.link_radii.empty() || params.link_lengths.empty()) {
    return nonlinear_damping;
  }

  const double radius = params.link_radii.at(link_index);
  const double length = params.link_lengths.at(link_index);
  const double normal_area = 2.0 * radius * length;
  const double tangential_area = kPi * radius * radius;
  const double d_n = 0.5 * params.rho * params.Cd_n * normal_area;
  const double d_t = 0.5 * params.rho * params.Cd_t * tangential_area;

  nonlinear_damping(0, 0) = d_t * std::abs(local_velocity(0));
  nonlinear_damping(1, 1) = d_n * std::abs(local_velocity(1));
  nonlinear_damping(2, 2) = d_n * std::abs(local_velocity(2));
  return nonlinear_damping;
}

ReferenceTerms computeReferenceTerms(
  const pinocchio::Model & model, const HydrodynamicParameters & params,
  const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::Data data(model);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);

  ReferenceTerms reference;
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose();
  reference.rigid_body_mass = data.M;

  pinocchio::computeGeneralizedGravity(model, data, q);
  reference.rigid_body_restoring = data.g;

  pinocchio::computeCoriolisMatrix(model, data, q, v);
  reference.rigid_body_coriolis = data.C;

  const int nv = model.nv;
  reference.added_mass = Eigen::MatrixXd::Zero(nv, nv);
  reference.added_mass_coriolis = Eigen::MatrixXd::Zero(nv, nv);
  reference.added_mass_mdot = Eigen::MatrixXd::Zero(nv, nv);
  reference.linear_damping = Eigen::MatrixXd::Zero(nv, nv);
  reference.nonlinear_damping = Eigen::MatrixXd::Zero(nv, nv);
  reference.buoyancy_generalized = Eigen::VectorXd::Zero(nv);

  const auto frame_ids = resolveHydrodynamicFrames(model, params);
  for (size_t link_index = 0; link_index < frame_ids.size(); ++link_index) {
    const auto frame_id = frame_ids[link_index];
    const auto local_jacobian =
      pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL);
    Eigen::MatrixXd local_jacobian_dot = Eigen::MatrixXd::Zero(6, nv);
    pinocchio::getFrameJacobianTimeVariation(
      model, data, frame_id, pinocchio::LOCAL, local_jacobian_dot);
    const Vector6d local_velocity =
      pinocchio::getFrameVelocity(model, data, frame_id, pinocchio::LOCAL).toVector();

    const Matrix6d added_mass = buildReferenceAddedMassMatrix(params, link_index);
    const Matrix6d linear_damping = buildReferenceLinearDampingMatrix(params, link_index);
    const Matrix6d nonlinear_damping =
      buildReferenceNonlinearDampingMatrix(params, link_index, local_velocity);

    reference.added_mass.noalias() += local_jacobian.transpose() * added_mass * local_jacobian;
    reference.added_mass_coriolis.noalias() +=
      local_jacobian.transpose() * added_mass * local_jacobian_dot;
    reference.added_mass_mdot.noalias() +=
      local_jacobian_dot.transpose() * added_mass * local_jacobian +
      local_jacobian.transpose() * added_mass * local_jacobian_dot;
    reference.linear_damping.noalias() +=
      local_jacobian.transpose() * linear_damping * local_jacobian;
    reference.nonlinear_damping.noalias() +=
      local_jacobian.transpose() * nonlinear_damping * local_jacobian;

    if (!params.link_volumes.empty()) {
      const double buoyancy_force = params.rho * params.link_volumes.at(link_index) * kGravity;
      Vector6d local_wrench = Vector6d::Zero();
      local_wrench.head<3>() =
        data.oMf[frame_id].rotation().transpose() * Eigen::Vector3d(0.0, 0.0, buoyancy_force);
      reference.buoyancy_generalized.noalias() += local_jacobian.transpose() * local_wrench;
    }
  }

  return reference;
}

void expectMatrixNear(
  const Eigen::MatrixXd & actual, const Eigen::MatrixXd & expected, const double tolerance,
  const std::string & context)
{
  ASSERT_EQ(actual.rows(), expected.rows()) << context;
  ASSERT_EQ(actual.cols(), expected.cols()) << context;
  EXPECT_LT((actual - expected).norm(), tolerance) << context;
}

void expectVectorNear(
  const Eigen::VectorXd & actual, const Eigen::VectorXd & expected, const double tolerance,
  const std::string & context)
{
  ASSERT_EQ(actual.rows(), expected.rows()) << context;
  EXPECT_LT((actual - expected).norm(), tolerance) << context;
}

}  // namespace

TEST(UnderwaterDynamicsTest, EmptyHydrodynamicParametersMatchRigidBodyTerms)
{
  const auto model = loadModel();
  const auto state = makeStateOne(*model);
  const auto params = makeBaseParameters();
  UnderwaterDynamics dynamics(model, makeData(*model), params);

  dynamics.computeDynamics(state.q, state.v);
  const auto reference = computeReferenceTerms(*model, params, state.q, state.v);

  expectMatrixNear(
    dynamics.getMassMatrix(), reference.rigid_body_mass, kMatrixTol,
    "Rigid-body mass should match CRBA when hydrodynamics are disabled");
  expectMatrixNear(
    dynamics.getCoriolisMatrix(), reference.rigid_body_coriolis, kMatrixTol,
    "Rigid-body Coriolis should match Pinocchio when hydrodynamics are disabled");
  expectVectorNear(
    dynamics.getRestoringForces(), reference.rigid_body_restoring, kVectorTol,
    "Restoring forces should match Pinocchio gravity when buoyancy is disabled");
  expectMatrixNear(
    dynamics.getDampingMatrix(), Eigen::MatrixXd::Zero(model->nv, model->nv), kMatrixTol,
    "Damping should be zero when no damping parameters are supplied");
}

TEST(UnderwaterDynamicsTest, AddedMassAssemblyMatchesExplicitDiagonalReference)
{
  const auto model = loadModel();
  const auto params = makeExplicitAddedMassParameters();
  UnderwaterDynamics dynamics(model, makeData(*model), params);

  for (const auto & state : {makeZeroState(*model), makeStateOne(*model)}) {
    dynamics.computeDynamics(state.q, state.v);
    const auto reference = computeReferenceTerms(*model, params, state.q, state.v);

    expectMatrixNear(
      dynamics.getMassMatrix() - reference.rigid_body_mass, reference.added_mass, kMatrixTol,
      "Added-mass contribution should equal the exact frame-wise assembly for " + state.name);
    expectMatrixNear(
      dynamics.getMassMatrix(),
      reference.rigid_body_mass + reference.added_mass, kMatrixTol,
      "Total mass should equal rigid-body plus added mass for " + state.name);
    expectMatrixNear(
      dynamics.getMassMatrix(), dynamics.getMassMatrix().transpose(), kMatrixTol,
      "Total mass matrix must remain symmetric for " + state.name);
  }
}

TEST(UnderwaterDynamicsTest, AddedMassCoriolisProductMatchesReferenceAndPowerIdentity)
{
  const auto model = loadModel();
  const auto params = makeExplicitAddedMassParameters();
  UnderwaterDynamics dynamics(model, makeData(*model), params);

  for (const auto & state : makeNonZeroStates(*model)) {
    dynamics.computeDynamics(state.q, state.v);
    const auto reference = computeReferenceTerms(*model, params, state.q, state.v);
    const Eigen::MatrixXd added_mass_coriolis =
      dynamics.getCoriolisMatrix() - reference.rigid_body_coriolis;

    expectVectorNear(
      added_mass_coriolis * state.v, reference.added_mass_coriolis * state.v, kVectorTol,
      "Added-mass Coriolis product should match the document assembly for " + state.name);

    const double power_residual =
      state.v.dot((reference.added_mass_mdot - 2.0 * added_mass_coriolis) * state.v);
    EXPECT_NEAR(power_residual, 0.0, kPowerTol)
      << "Added-mass Coriolis realization should satisfy the power identity for " << state.name;
  }
}

TEST(UnderwaterDynamicsTest, LinearDampingAssemblyMatchesExplicitDiagonalReference)
{
  const auto model = loadModel();
  const auto params = makeExplicitLinearDampingParameters();
  UnderwaterDynamics dynamics(model, makeData(*model), params);

  for (const auto & state : {makeZeroState(*model), makeStateOne(*model)}) {
    dynamics.computeDynamics(state.q, state.v);
    const auto reference = computeReferenceTerms(*model, params, state.q, state.v);

    expectMatrixNear(
      dynamics.getDampingMatrix(), reference.linear_damping, kMatrixTol,
      "Linear damping matrix should match the exact generalized assembly for " + state.name);
  }
}

TEST(UnderwaterDynamicsTest, QuadraticDampingAssemblyMatchesReference)
{
  const auto model = loadModel();
  const auto params = makeQuadraticDampingParameters();
  UnderwaterDynamics dynamics(model, makeData(*model), params);

  for (const auto & state : makeNonZeroStates(*model)) {
    dynamics.computeDynamics(state.q, state.v);
    const auto reference = computeReferenceTerms(*model, params, state.q, state.v);

    expectMatrixNear(
      dynamics.getDampingMatrix(), reference.nonlinear_damping, kMatrixTol,
      "Quadratic damping matrix should match the cylindrical-link mapping for " + state.name);
  }
}

TEST(UnderwaterDynamicsTest, BuoyancyAssemblyMatchesReferenceAndNeutralBuoyancyReducesRestoringForce)
{
  const auto model = loadModel();
  const auto buoyancy_params = makeBuoyancyParameters();
  UnderwaterDynamics buoyant_dynamics(model, makeData(*model), buoyancy_params);

  for (const auto & state : {makeZeroState(*model), makeStateTwo(*model)}) {
    buoyant_dynamics.computeDynamics(state.q, state.v);
    const auto reference = computeReferenceTerms(*model, buoyancy_params, state.q, state.v);

    expectVectorNear(
      buoyant_dynamics.getRestoringForces(),
      reference.rigid_body_restoring - reference.buoyancy_generalized,
      kVectorTol,
      "Restoring forces should equal rigid-body gravity minus generalized buoyancy for " + state.name);
  }

  UnderwaterDynamics rigid_body_only(model, makeData(*model), makeBaseParameters());
  UnderwaterDynamics neutral_buoyancy(model, makeData(*model), makeNeutralBuoyancyParameters());
  const auto zero_state = makeZeroState(*model);

  rigid_body_only.computeDynamics(zero_state.q, zero_state.v);
  neutral_buoyancy.computeDynamics(zero_state.q, zero_state.v);

  EXPECT_LT(
    neutral_buoyancy.getRestoringForces().norm(),
    0.01 * rigid_body_only.getRestoringForces().norm())
    << "Near-neutral buoyancy for the four main body links should greatly reduce restoring forces.";
}

TEST(UnderwaterDynamicsTest, ParameterValidationRejectsMismatches)
{
  const auto model = loadModel();

  {
    HydrodynamicParameters params = makeBaseParameters();
    params.link_volumes = {0.001, 0.002, 0.003};
    EXPECT_THROW(
      static_cast<void>(UnderwaterDynamics(model, makeData(*model), params)),
      std::invalid_argument);
  }

  {
    HydrodynamicParameters params = makeBaseParameters();
    params.link_added_mass_diagonal = std::vector<double>(23, 1.0);
    EXPECT_THROW(
      static_cast<void>(UnderwaterDynamics(model, makeData(*model), params)),
      std::invalid_argument);
  }

  {
    HydrodynamicParameters params = makeBaseParameters();
    params.link_radii = {0.05, 0.06, 0.07, 0.08};
    EXPECT_THROW(
      static_cast<void>(UnderwaterDynamics(model, makeData(*model), params)),
      std::invalid_argument);
  }

  {
    HydrodynamicParameters params = makeBaseParameters();
    params.hydrodynamic_link_names = {"base_link", "link_2", "link_3", "missing_link"};
    EXPECT_THROW(
      static_cast<void>(UnderwaterDynamics(model, makeData(*model), params)),
      std::invalid_argument);
  }
}

TEST(UnderwaterDynamicsTest, BodySelectionAutoDiscoveryMatchesExplicitOrdering)
{
  const auto model = loadModel();
  auto explicit_params = makeQuadraticDampingParameters(true);
  explicit_params.link_volumes = {0.0015, 0.0018, 0.0021, 0.0024};
  explicit_params.link_added_mass_diagonal = flattenDiagonalBlocks(kAddedMassDiagonalBlocks);
  explicit_params.link_linear_damping_diagonal = flattenDiagonalBlocks(kLinearDampingDiagonalBlocks);

  auto autodiscovery_params = explicit_params;
  autodiscovery_params.hydrodynamic_link_names.clear();

  UnderwaterDynamics explicit_dynamics(model, makeData(*model), explicit_params);
  UnderwaterDynamics autodiscovery_dynamics(model, makeData(*model), autodiscovery_params);
  const auto state = makeStateOne(*model);

  explicit_dynamics.computeDynamics(state.q, state.v);
  autodiscovery_dynamics.computeDynamics(state.q, state.v);

  expectMatrixNear(
    autodiscovery_dynamics.getMassMatrix(), explicit_dynamics.getMassMatrix(), kMatrixTol,
    "Auto-discovered body ordering should match the explicit body ordering for mass");
  expectMatrixNear(
    autodiscovery_dynamics.getCoriolisMatrix(), explicit_dynamics.getCoriolisMatrix(), kMatrixTol,
    "Auto-discovered body ordering should match the explicit body ordering for Coriolis");
  expectMatrixNear(
    autodiscovery_dynamics.getDampingMatrix(), explicit_dynamics.getDampingMatrix(), kMatrixTol,
    "Auto-discovered body ordering should match the explicit body ordering for damping");
  expectVectorNear(
    autodiscovery_dynamics.getRestoringForces(), explicit_dynamics.getRestoringForces(), kVectorTol,
    "Auto-discovered body ordering should match the explicit body ordering for restoring forces");
}

}  // namespace asr_sdm_kinematic_dynamic_model

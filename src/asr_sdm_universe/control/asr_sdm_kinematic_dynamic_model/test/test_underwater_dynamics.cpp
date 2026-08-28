#include "asr_sdm_kinematic_dynamic_model/underwater_dynamics.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <gtest/gtest.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <memory>

namespace asr_sdm_kinematic_dynamic_model
{
namespace
{

std::shared_ptr<pinocchio::Model> loadModel()
{
  const auto package_share = ament_index_cpp::get_package_share_directory("asr_sdm_description");
  const auto urdf_path = package_share + "/urdf/deprecated/underwater_snakerobot.urdf";

  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *model);
  return model;
}

}  // namespace

TEST(UnderwaterDynamicsTest, ComputesFiniteDynamicsForNominalState)
{
  auto model = loadModel();
  auto data = std::make_shared<pinocchio::Data>(*model);
  HydrodynamicParameters parameters;
  UnderwaterDynamics dynamics(model, data, parameters);

  const Eigen::VectorXd q = pinocchio::neutral(*model);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model->nv);
  dynamics.computeDynamics(q, v);

  EXPECT_EQ(dynamics.getMassMatrix().rows(), model->nv);
  EXPECT_EQ(dynamics.getMassMatrix().cols(), model->nv);
  EXPECT_EQ(dynamics.getCoriolisMatrix().rows(), model->nv);
  EXPECT_EQ(dynamics.getDampingMatrix().cols(), model->nv);
  EXPECT_EQ(dynamics.getRestoringForces().size(), model->nv);
  EXPECT_TRUE(dynamics.getMassMatrix().allFinite());
  EXPECT_TRUE(dynamics.getCoriolisMatrix().allFinite());
  EXPECT_TRUE(dynamics.getDampingMatrix().allFinite());
  EXPECT_TRUE(dynamics.getRestoringForces().allFinite());
}

}  // namespace asr_sdm_kinematic_dynamic_model

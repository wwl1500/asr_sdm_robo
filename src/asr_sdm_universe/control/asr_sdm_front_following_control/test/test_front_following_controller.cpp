#include "asr_sdm_front_following_control/front_following_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <gtest/gtest.h>

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <memory>
#include <string>

namespace asr_sdm_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::shared_ptr<pinocchio::Model> loadModel()
{
  const auto package_share = ament_index_cpp::get_package_share_directory("asr_sdm_description");
  const auto urdf_path = package_share + "/urdf/deprecated/underwater_snakerobot.urdf";

  auto full_model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *full_model);

  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (pinocchio::JointIndex joint_id = 1;
    joint_id < static_cast<pinocchio::JointIndex>(full_model->njoints); ++joint_id)
  {
    if (full_model->names[joint_id].find("screw") != std::string::npos) {
      joints_to_lock.push_back(joint_id);
    }
  }

  return std::make_shared<pinocchio::Model>(
    pinocchio::buildReducedModel(
      *full_model, joints_to_lock, pinocchio::neutral(*full_model)));
}

std::unique_ptr<FrontFollowingController> makeController()
{
  auto model = loadModel();
  auto data = std::make_shared<pinocchio::Data>(*model);

  ControllerParameters parameters;
  parameters.max_propulsion_force = Vector4d::Constant(500.0);
  parameters.max_joint_torque = Vector6d::Constant(500.0);

  asr_sdm_kinematic_dynamic_model::HydrodynamicParameters hydrodynamic_parameters;
  return std::make_unique<FrontFollowingController>(
    model, data, hydrodynamic_parameters, parameters);
}

RobotKinematicState makeNominalState()
{
  RobotKinematicState state;
  state.position << 0.3, -0.2, 0.1;
  state.orientation =
    Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  state.linear_velocity_body << 0.1, -0.02, 0.03;
  state.angular_velocity_body << 0.01, -0.02, 0.04;
  state.joint_positions << 0.05, -0.10, 0.03, 0.08, -0.04, 0.06;
  state.joint_velocities << 0.02, -0.01, 0.03, -0.02, 0.01, 0.04;
  return state;
}

}  // namespace

TEST(FrontFollowingControllerTest, NominalStateProducesFiniteMappings)
{
  auto controller = makeController();
  const auto result = controller->compute(makeNominalState(), MotionCommand{}, 0.05);

  ASSERT_TRUE(result.ok) << result.error_message;
  EXPECT_EQ(result.a_ff.rows(), 12);
  EXPECT_EQ(result.a_ff.cols(), 12);
  EXPECT_EQ(result.b_act.rows(), 12);
  EXPECT_EQ(result.b_act.cols(), 10);
  EXPECT_TRUE(result.a_ff.allFinite());
  EXPECT_TRUE(result.b_act.allFinite());
  EXPECT_TRUE(result.u_cmd.allFinite());
  EXPECT_TRUE(result.allocation_residual.allFinite());
  EXPECT_GT(result.aff_condition_number, 0.0);
}

TEST(FrontFollowingControllerTest, NearPitchSingularityTriggersSafetyFallback)
{
  auto controller = makeController();
  auto state = makeNominalState();
  state.orientation =
    Eigen::AngleAxisd((kPi / 2.0) - 5.0e-4, Eigen::Vector3d::UnitY());

  const auto result = controller->compute(state, MotionCommand{}, 0.05);

  EXPECT_FALSE(result.ok);
  const bool reported_singularity =
    result.error_message.find("singularity") != std::string::npos ||
    result.error_message.find("singular") != std::string::npos;
  const bool reported_conditioning =
    result.error_message.find("ill-conditioned") != std::string::npos;
  EXPECT_TRUE(reported_singularity || reported_conditioning);
}

}  // namespace asr_sdm_controller

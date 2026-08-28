#include "asr_sdm_online_path_following_control/online_path_following_controller_3d.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{

asr::RealizedFrontTwist3D makeSample(
  double stamp, double speed, const asr::Mat3 & frame, bool measured_linear)
{
  asr::RealizedFrontTwist3D sample;
  sample.stamp_sec = stamp;
  sample.linear_velocity_head = {speed, 0.0, 0.0};
  sample.angular_velocity_head = {0.0, 0.0, 0.0};
  sample.tracking_from_head = frame;
  sample.estimator_epoch = 1;
  sample.tracking_frame_epoch = 1;
  sample.twist_valid = true;
  sample.attitude_valid = true;
  sample.linear_velocity_measured = measured_linear;
  return sample;
}

TEST(OnlinePathFollowingController3DTest, AcceptsValidAndStraightLimitParameters)
{
  asr::OnlinePathFollowing3DParameters params;
  EXPECT_NO_THROW(asr::OnlinePathFollowingController3D controller(params));

  params.max_curvature = 0.0;
  params.history_length = 3.0 * params.link_length + 2.0 * params.path_sample_ds;
  EXPECT_NO_THROW(asr::OnlinePathFollowingController3D controller(params));
}

TEST(OnlinePathFollowingController3DTest, RejectsInsufficientHistory)
{
  asr::OnlinePathFollowing3DParameters params;
  params.history_length = 0.01;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

TEST(OnlinePathFollowingController3DTest, RejectsInfeasibleChordCurvature)
{
  asr::OnlinePathFollowing3DParameters params;
  params.link_length = 0.25;
  params.max_curvature = 8.0;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);

  params.max_curvature = 8.1;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

TEST(OnlinePathFollowingController3DTest, RejectsInvalidSamplingAndNonFiniteParameters)
{
  asr::OnlinePathFollowing3DParameters params;
  params.path_sample_ds = 0.0;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);

  params = asr::OnlinePathFollowing3DParameters{};
  params.min_append_distance = params.path_sample_ds * 2.0;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);

  params = asr::OnlinePathFollowing3DParameters{};
  params.link_length = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

TEST(OnlinePathFollowingController3DTest, SyntheticWarmupNeverCountsAsActualCoverage)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::ActualRequired;
  params.use_local_fallback = false;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  controller.step({0.1, 0.0, 0.0}, 0.02, state);
  const auto diag = controller.diagnostics();
  EXPECT_GT(diag.geometry_coverage, 0.0);
  EXPECT_DOUBLE_EQ(diag.history_coverage, 0.0);
  EXPECT_FALSE(diag.buffer_ready);
  EXPECT_FALSE(diag.reference_valid);
}

TEST(OnlinePathFollowingController3DTest, FirstCommandUsesSlewAndLowPassFilter)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  params.linear_accel_limit = 0.5;
  params.angular_rate_slew_limit = 1.0;
  params.command_filter_time_constant = 0.02;
  params.max_curvature = 7.0;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  constexpr double dt = 0.02;
  controller.step({1.0, 1.0, -1.0}, dt, state);
  const auto applied = controller.appliedCommand();
  const double alpha = dt / (params.command_filter_time_constant + dt);
  EXPECT_NEAR(applied.linear_velocity, alpha * params.linear_accel_limit * dt, 1.0e-12);
  EXPECT_NEAR(applied.pitch_rate, alpha * params.angular_rate_slew_limit * dt, 1.0e-12);
  EXPECT_NEAR(applied.yaw_rate, -alpha * params.angular_rate_slew_limit * dt, 1.0e-12);
}

TEST(OnlinePathFollowingController3DTest, RejectsReverseCommand)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();
  controller.step({-0.1, 0.0, 0.0}, 0.02, state);
  EXPECT_DOUBLE_EQ(controller.appliedCommand().linear_velocity, 0.0);
  EXPECT_TRUE(controller.diagnostics().reverse_command_rejected);
}

TEST(OnlinePathFollowingController3DTest, ImuAssistedKeepsCommandVelocityDegraded)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::ImuAssisted;
  params.command_filter_time_constant = 0.0;
  params.linear_accel_limit = 100.0;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), false));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_TRUE(diag.linear_velocity_from_command);
  EXPECT_TRUE(diag.degraded_front_motion);
  EXPECT_TRUE(diag.sensor_valid);
  EXPECT_DOUBLE_EQ(diag.history_coverage, 0.0);
  EXPECT_GT(diag.degraded_coverage, 0.05);
  EXPECT_FALSE(diag.buffer_ready);
}

TEST(OnlinePathFollowingController3DTest, ActualRequiredRejectsCommandLinearVelocity)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::ActualRequired;
  params.command_filter_time_constant = 0.0;
  params.linear_accel_limit = 100.0;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), false));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_FALSE(diag.sensor_valid);
  EXPECT_DOUBLE_EQ(diag.history_coverage, 0.0);
  EXPECT_FALSE(diag.linear_velocity_from_command);
}

TEST(OnlinePathFollowingController3DTest, AttitudeJumpResetsImuAssistedHistory)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::ImuAssisted;
  params.max_attitude_step_rad = 0.2;
  params.command_filter_time_constant = 0.0;
  params.linear_accel_limit = 100.0;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  controller.ingestRealizedFrontTwist(makeSample(0.02, 0.1, asr::identityFrame(), false));
  controller.step({0.1, 0.0, 0.0}, 0.02, 0.02, state);
  controller.ingestRealizedFrontTwist(makeSample(0.04, 0.1, asr::rotationZ(1.0), false));
  controller.step({0.1, 0.0, 0.0}, 0.04, 0.02, state);

  EXPECT_GT(controller.diagnostics().reset_count, 0u);
}

TEST(OnlinePathFollowingController3DTest, SmallDisplacementsAccumulateBeforeAppend)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.command_filter_time_constant = 0.0;
  params.linear_accel_limit = 100.0;
  params.min_append_distance = 0.002;
  params.path_sample_ds = 0.01;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 1000; ++i) {
    t += 0.01;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.01, state);
  }

  const auto diag = controller.diagnostics();
  EXPECT_GT(diag.history_coverage, 0.90);
  EXPECT_LT(std::abs(diag.formal_actual_coverage - diag.history_coverage), 0.01);
}

TEST(OnlinePathFollowingController3DTest, ReadOnlyStepDoesNotModifyMeasuredState)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();
  asr::MeasuredRobotState3D measured = controller.makeMeasuredState(state);
  measured.joints.theta[0] = 0.25;
  const auto before = measured;

  const auto output = controller.step({0.1, 0.1, 0.0}, 0.02, 0.02, measured);
  EXPECT_NE(output.joint_velocity_command.theta_dot[0], 0.0);
  EXPECT_DOUBLE_EQ(measured.stamp_sec, before.stamp_sec);
  EXPECT_DOUBLE_EQ(measured.joints.theta[0], before.joints.theta[0]);
  EXPECT_DOUBLE_EQ(measured.tracking_from_head.v[0][0], before.tracking_from_head.v[0][0]);
}

TEST(OnlinePathFollowingController3DTest, KinematicPlantIntegratesOnce)
{
  asr::FrontUnitController3DParameters params;
  params.link_length = 0.25;
  params.joint_rate_limit = 2.0;
  params.joint_limit = 3.0;
  params.max_curvature = 1.2;
  params.curvature_velocity_epsilon = 1.0e-3;
  asr::KinematicPlant3D plant(params);
  auto state = plant.makeInitialState();
  asr::JointVelocity3D joint_velocity;
  joint_velocity.theta_dot[0] = 1.0;

  plant.apply({0.1, 0.0, 0.0}, joint_velocity, 0.02, state);
  EXPECT_NEAR(state.time, 0.02, 1.0e-12);
  EXPECT_NEAR(state.head_position.x, 0.002, 1.0e-12);
  EXPECT_NEAR(state.joints.theta[0], 0.02, 1.0e-12);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

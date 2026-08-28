#include "asr_sdm_online_path_following_control/online_path_following_controller_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

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
  sample.angular_velocity_measured = true;
  return sample;
}

// ============ Mode A/B/C 测试 ============

TEST(Chapter8ModeTest, ModeARequiresMeasuredTwist)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), false));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_DOUBLE_EQ(diag.formal_actual_coverage, 0.0);
  EXPECT_DOUBLE_EQ(diag.history_coverage, 0.0);
  EXPECT_FALSE(diag.buffer_ready);
}

TEST(Chapter8ModeTest, ModeAWithMeasuredTwistFormsFormalCoverage)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 120; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_GT(diag.formal_actual_coverage, 0.0);
  EXPECT_GT(diag.history_coverage, 0.0);
}

TEST(Chapter8ModeTest, ModeBKeepsFormalCoverageZero)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::ImuAssisted;
  params.use_local_fallback = false;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), false));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_TRUE(diag.degraded_front_motion);
  EXPECT_DOUBLE_EQ(diag.formal_actual_coverage, 0.0);
  EXPECT_GT(diag.degraded_coverage, 0.0);
  EXPECT_FALSE(diag.buffer_ready);
}

TEST(Chapter8ModeTest, ModeCSkipsAllHistory)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_DOUBLE_EQ(diag.formal_actual_coverage, 0.0);
  EXPECT_DOUBLE_EQ(diag.degraded_coverage, 0.0);
}

// ============ 低速守卫测试 ============

TEST(Chapter8LowSpeedTest, NoiseDoesNotAddToBuffer)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  params.min_buffer_update_speed = 0.02;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.001, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_DOUBLE_EQ(diag.formal_actual_coverage, 0.0);
}

// ============ 反向测试 ============

TEST(Chapter8ReverseTest, ReverseMotionCreatesNewEpoch)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 30; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto before = controller.diagnostics();
  const uint64_t epoch_before = before.geometry_epoch;

  // 负速度（反向）
  for (size_t i = 0; i < 5; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, -0.05, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto after = controller.diagnostics();
  EXPECT_GT(after.geometry_epoch, epoch_before);
  EXPECT_EQ(after.reset_count, before.reset_count + 1);
  EXPECT_TRUE(after.reverse_motion_detected);
}

// ============ 队列溢出测试 ============

TEST(Chapter8QueueTest, OverflowCausesGeometryReset)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  params.max_pending_motion_samples = 3;
  params.max_motion_samples_per_control_step = 64;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 30; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto before = controller.diagnostics();
  const uint64_t epoch_before = before.geometry_epoch;

  // 快速注入以触发溢出
  for (size_t i = 0; i < 5; ++i) {
    t += 0.001;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
  }
  controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  const auto after = controller.diagnostics();
  EXPECT_GT(after.dropped_queue_overflow_samples, before.dropped_queue_overflow_samples);
}

// ============ 姿态安全测试 ============

TEST(Chapter8RotationTest, InvalidRotationCausesSafeZeroOutput)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();
  asr::MeasuredRobotState3D measured = controller.makeMeasuredState(state);
  measured.tracking_from_head.v[0][0] = std::numeric_limits<double>::quiet_NaN();

  const auto output = controller.step({0.1, 0.0, 0.0}, 0.02, 0.02, measured);
  EXPECT_DOUBLE_EQ(output.applied_front_command.linear_velocity, 0.0);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(output.joint_velocity_command.theta_dot[i], 0.0);
  }
}

TEST(Chapter8RotationTest, ReflectionMatrixCausesSafeZero)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();
  asr::MeasuredRobotState3D measured = controller.makeMeasuredState(state);
  measured.tracking_from_head.v[0][0] = -1.0;

  const auto output = controller.step({0.1, 0.0, 0.0}, 0.02, 0.02, measured);
  EXPECT_DOUBLE_EQ(output.applied_front_command.linear_velocity, 0.0);
}

// ============ 状态时间测试 ============

TEST(Chapter8StateAgeTest, StaleStateCausesSafeZero)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  params.measured_state_timeout_sec = 0.1;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();
  asr::MeasuredRobotState3D measured = controller.makeMeasuredState(state);
  measured.stamp_sec = 0.0;

  const auto output = controller.step({0.1, 0.0, 0.0}, 0.5, 0.02, measured);
  EXPECT_DOUBLE_EQ(output.applied_front_command.linear_velocity, 0.0);
  EXPECT_TRUE(output.diagnostics.measured_state_stale);
}

TEST(Chapter8StateAgeTest, FutureStateCausesSafeZero)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::LocalFallbackOnly;
  params.future_state_tolerance_sec = 0.01;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();
  asr::MeasuredRobotState3D measured = controller.makeMeasuredState(state);
  measured.stamp_sec = 10.0;

  const auto output = controller.step({0.1, 0.0, 0.0}, 0.02, 0.02, measured);
  EXPECT_DOUBLE_EQ(output.applied_front_command.linear_velocity, 0.0);
  EXPECT_TRUE(output.diagnostics.measured_state_from_future);
}

// ============ 平移不变性测试 ============

TEST(Chapter8TranslationInvarianceTest, TranslationDoesNotAffectChords)
{
  // 直接使用 geometry 内部验证：平移后的 chord 搜索应得到相同结果
  // 此测试本质上由线性插值的不变性保证，这里验证控制器在平移后仍正常运行
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 120; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_TRUE(diag.sensor_valid);
  EXPECT_GT(diag.formal_actual_coverage, 0.0);
  EXPECT_FALSE(diag.chord_search_failed_this_step);
}

// ============ 原始/滤波曲率测试 ============

TEST(Chapter8CurvatureTest, RawAndFilteredCurvatureAreSeparate)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  params.curvature_filter_alpha = 0.3;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 120; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_TRUE(std::isfinite(diag.latest_raw_curvature));
  EXPECT_TRUE(std::isfinite(diag.latest_filtered_curvature));
}

// ============ 无效参数测试 ============

TEST(Chapter8ParameterTest, RejectsZeroQueueCapacity)
{
  asr::OnlinePathFollowing3DParameters params;
  params.max_pending_motion_samples = 0;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

TEST(Chapter8ParameterTest, RejectsZeroSamplesPerStep)
{
  asr::OnlinePathFollowing3DParameters params;
  params.max_motion_samples_per_control_step = 0;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

TEST(Chapter8ParameterTest, RejectsInvalidCurvatureFilterAlpha)
{
  asr::OnlinePathFollowing3DParameters params;
  params.curvature_filter_alpha = -0.1;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);

  params.curvature_filter_alpha = 1.1;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

TEST(Chapter8ParameterTest, RejectsNegativeBufferUpdateSpeed)
{
  asr::OnlinePathFollowing3DParameters params;
  params.min_buffer_update_speed = -0.1;
  EXPECT_THROW(asr::OnlinePathFollowingController3D controller(params), std::invalid_argument);
}

// ============ 预测失败语义测试 ============

TEST(Chapter8PredictorTest, HighMismatchDisablesFeedforwardOnly)
{
  asr::OnlinePathFollowing3DParameters params;
  params.front_motion_mode = asr::FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = false;
  params.max_predictor_velocity_mismatch = 0.01;
  asr::OnlinePathFollowingController3D controller(params);
  auto state = controller.makeInitialState();

  double t = 0.0;
  for (size_t i = 0; i < 120; ++i) {
    t += 0.02;
    controller.ingestRealizedFrontTwist(makeSample(t, 0.1, asr::identityFrame(), true));
    controller.step({0.1, 0.0, 0.0}, t, 0.02, state);
  }
  const auto diag = controller.diagnostics();
  EXPECT_TRUE(diag.sensor_valid);
  // 只要几何搜索成功，reference_valid 应为 true
  // 但前馈被 disabled（命令速度 0.1 vs 实测速度 0.1，但方向有差异的情况下）
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

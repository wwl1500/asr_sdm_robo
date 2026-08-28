// Unit tests for MpcSafetyMapper.

#include "asr_sdm_kinematic_dynamic_model/mpc_safety_mapper.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using asr_sdm_kinematic_dynamic_model::JointTorqueVector;
using asr_sdm_kinematic_dynamic_model::MpcSafetyMapper;
using asr_sdm_kinematic_dynamic_model::MpcSafetyParameters;
using asr_sdm_kinematic_dynamic_model::ReducedConfiguration;
using asr_sdm_kinematic_dynamic_model::ReducedVelocity;
using asr_sdm_kinematic_dynamic_model::SegmentThrustVector;
using asr_sdm_kinematic_dynamic_model::UnderwaterSimulatorInput;
using asr_sdm_kinematic_dynamic_model::UnderwaterSimulatorState;

namespace
{

MpcSafetyParameters defaultParams()
{
  MpcSafetyParameters p;
  p.max_segment_thrust.setConstant(10.0);
  p.max_joint_torque.setConstant(8.0);
  p.max_segment_thrust_rate = 100.0;
  p.max_joint_torque_rate = 50.0;
  p.max_linear_velocity_fault = 5.0;
  p.max_angular_velocity_fault = 5.0;
  p.max_position_deviation_fault = 10.0;
  p.safe_mode_thrust_scale = 0.0;
  p.safe_mode_torque_scale = 0.0;
  return p;
}

UnderwaterSimulatorInput makeInput(double thrust_val, double torque_val)
{
  UnderwaterSimulatorInput u;
  u.segment_thrust.setConstant(thrust_val);
  u.joint_torque.setConstant(torque_val);
  return u;
}

UnderwaterSimulatorState makeState(double vx = 0.0, double x = 0.0)
{
  UnderwaterSimulatorState s;
  s.configuration = ReducedConfiguration::Zero();
  s.configuration(0) = x;
  s.velocity = ReducedVelocity::Zero();
  s.velocity(0) = vx;
  return s;
}

}  // namespace

TEST(MpcSafetyMapperTest, WithinLimitsReturnsInputUnchanged)
{
  MpcSafetyMapper mapper(defaultParams());
  UnderwaterSimulatorInput in = makeInput(3.0, 2.0);
  UnderwaterSimulatorState state = makeState();
  ReducedConfiguration initial = ReducedConfiguration::Zero();

  auto out = mapper.map(in, state, initial, 0.02);

  EXPECT_NEAR(out.segment_thrust(0), 3.0, 1e-9);
  EXPECT_NEAR(out.joint_torque(0), 2.0, 1e-9);
  EXPECT_FALSE(mapper.lastStatus().input_was_clamped);
  EXPECT_FALSE(mapper.lastStatus().slew_rate_was_clamped);
  EXPECT_FALSE(mapper.lastStatus().safe_mode_active);
}

TEST(MpcSafetyMapperTest, ThrustAboveLimitIsClamped)
{
  MpcSafetyMapper mapper(defaultParams());
  UnderwaterSimulatorInput in = makeInput(15.0, 2.0);  // above 10N limit
  UnderwaterSimulatorState state = makeState();

  auto out = mapper.map(in, state, ReducedConfiguration::Zero(), 0.02);

  EXPECT_LE(out.segment_thrust(0), 10.0);
  EXPECT_GE(out.segment_thrust(0), 0.0);
  EXPECT_TRUE(mapper.lastStatus().input_was_clamped);
}

TEST(MpcSafetyMapperTest, NegativeThrustClampedToZero)
{
  MpcSafetyMapper mapper(defaultParams());
  UnderwaterSimulatorInput in = makeInput(-5.0, 0.0);
  UnderwaterSimulatorState state = makeState();

  auto out = mapper.map(in, state, ReducedConfiguration::Zero(), 0.02);

  EXPECT_GE(out.segment_thrust(0), 0.0);
  EXPECT_LE(out.segment_thrust(0), 10.0);
  EXPECT_TRUE(mapper.lastStatus().input_was_clamped);
}

TEST(MpcSafetyMapperTest, TorqueClampedSymmetric)
{
  MpcSafetyMapper mapper(defaultParams());
  UnderwaterSimulatorInput in = makeInput(1.0, 20.0);  // above 8 Nm limit
  UnderwaterSimulatorState state = makeState();

  auto out = mapper.map(in, state, ReducedConfiguration::Zero(), 0.02);

  EXPECT_LE(out.joint_torque(0), 8.0);
  EXPECT_GE(out.joint_torque(0), -8.0);
  EXPECT_TRUE(mapper.lastStatus().input_was_clamped);
}

TEST(MpcSafetyMapperTest, SlewRateLimitSmoothsStep)
{
  MpcSafetyParameters params = defaultParams();
  params.max_segment_thrust_rate = 100.0;  // 100 N/s, dt=0.02 → max_d=2 N
  MpcSafetyMapper mapper(params);

  // First call: warm cache with thrust=1.
  auto first = makeInput(1.0, 0.0);
  auto state = makeState();
  mapper.map(first, state, ReducedConfiguration::Zero(), 0.02);

  // Second call: ask for 5 N (jump of 4).  Should be limited to 1+2=3.
  auto second = makeInput(5.0, 0.0);
  auto out = mapper.map(second, state, ReducedConfiguration::Zero(), 0.02);

  EXPECT_NEAR(out.segment_thrust(0), 3.0, 1e-6);
  EXPECT_TRUE(mapper.lastStatus().slew_rate_was_clamped);
}

TEST(MpcSafetyMapperTest, NaNInMpcOutputTriggersSafeMode)
{
  MpcSafetyMapper mapper(defaultParams());

  // Warm the cache with a valid command first.
  mapper.map(makeInput(2.0, 1.0), makeState(), ReducedConfiguration::Zero(), 0.02);

  // Now feed a corrupted command.
  UnderwaterSimulatorInput bad = makeInput(2.0, 1.0);
  bad.segment_thrust(1) = std::numeric_limits<double>::quiet_NaN();
  auto out = mapper.map(bad, makeState(), ReducedConfiguration::Zero(), 0.02);

  EXPECT_TRUE(mapper.lastStatus().mpc_output_was_invalid);
  EXPECT_TRUE(mapper.lastStatus().safe_mode_active);
  // safe_mode_scale=0 by default → zero output.
  EXPECT_NEAR(out.segment_thrust(0), 0.0, 1e-9);
}

TEST(MpcSafetyMapperTest, InfInStateTriggersSafeMode)
{
  MpcSafetyMapper mapper(defaultParams());

  UnderwaterSimulatorState bad_state = makeState();
  bad_state.velocity(0) = std::numeric_limits<double>::infinity();

  auto out = mapper.map(makeInput(1.0, 0.0), bad_state, ReducedConfiguration::Zero(), 0.02);

  EXPECT_TRUE(mapper.lastStatus().state_was_invalid);
  EXPECT_TRUE(mapper.lastStatus().safe_mode_active);
  EXPECT_NEAR(out.segment_thrust(0), 0.0, 1e-9);
}

TEST(MpcSafetyMapperTest, VelocityAboveFaultThresholdTriggersSafeMode)
{
  MpcSafetyMapper mapper(defaultParams());

  UnderwaterSimulatorState fast = makeState(/*vx=*/10.0);  // > 5 m/s threshold
  auto out = mapper.map(makeInput(1.0, 0.0), fast, ReducedConfiguration::Zero(), 0.02);

  EXPECT_TRUE(mapper.lastStatus().state_exceeded_fault_threshold);
  EXPECT_TRUE(mapper.lastStatus().safe_mode_active);
}

TEST(MpcSafetyMapperTest, SafeModeScaleZeroFiveProducesHalfOutput)
{
  MpcSafetyParameters params = defaultParams();
  params.safe_mode_thrust_scale = 0.5;
  params.safe_mode_torque_scale = 0.5;
  MpcSafetyMapper mapper(params);

  // Warm cache with thrust=4 N.
  mapper.map(makeInput(4.0, 2.0), makeState(), ReducedConfiguration::Zero(), 0.02);

  // Inject NaN to force safe mode.
  UnderwaterSimulatorInput bad = makeInput(4.0, 2.0);
  bad.joint_torque(0) = std::numeric_limits<double>::quiet_NaN();
  auto out = mapper.map(bad, makeState(), ReducedConfiguration::Zero(), 0.02);

  EXPECT_TRUE(mapper.lastStatus().safe_mode_active);
  // Should output 0.5 * last_valid = (2.0, 1.0).
  EXPECT_NEAR(out.segment_thrust(0), 2.0, 1e-9);
  EXPECT_NEAR(out.joint_torque(0), 1.0, 1e-9);
}

TEST(MpcSafetyMapperTest, ResetClearsState)
{
  MpcSafetyMapper mapper(defaultParams());

  mapper.map(makeInput(5.0, 5.0), makeState(), ReducedConfiguration::Zero(), 0.02);
  mapper.reset();
  EXPECT_FALSE(mapper.lastStatus().input_was_clamped);

  // First call after reset: no slew rate limit (no last valid command).
  UnderwaterSimulatorState state = makeState();
  auto out = mapper.map(makeInput(8.0, 7.0), state, ReducedConfiguration::Zero(), 0.02);
  EXPECT_NEAR(out.segment_thrust(0), 8.0, 1e-9);
  EXPECT_NEAR(out.joint_torque(0), 7.0, 1e-9);
  EXPECT_FALSE(mapper.lastStatus().slew_rate_was_clamped);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

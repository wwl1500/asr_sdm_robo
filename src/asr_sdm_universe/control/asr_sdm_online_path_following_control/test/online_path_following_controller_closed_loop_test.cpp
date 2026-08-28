#include "online_path_following_controller_closed_loop.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace
{

std::string metricsMessage(const asr::validation::Result & result)
{
  const auto & m = result.metrics;
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4)
         << "scenario=" << result.scenario.name
         << " score=" << m.score_samples
         << " common=" << m.common_error_samples
         << " online_rms=[" << m.online_follower_error[0].rms << ", "
         << m.online_follower_error[1].rms << ", " << m.online_follower_error[2].rms << "]"
         << " local_rms=[" << m.local_follower_error[0].rms << ", "
         << m.local_follower_error[1].rms << ", " << m.local_follower_error[2].rms << "]"
         << " coverage=" << m.final_actual_coverage << "/" << m.required_history
         << " ready/new/frozen/chord/curvature/axis/continuity=["
         << m.buffer_ready_samples << "/" << m.new_reference_samples << "/" << m.frozen_samples
         << "/" << m.chord_failure_samples << "/" << m.curvature_guard_samples
         << "/" << m.axis_guard_samples << "/" << m.continuity_guard_samples << "]"
         << " max_rate=" << m.maximum_online_joint_rate
         << " max_angle=" << m.maximum_online_joint_angle
         << " max_curvature_ratio=" << m.maximum_curvature_ratio
         << " max_curvature_rate=" << m.maximum_online_curvature_rate
         << " max_chord_residual=" << m.maximum_chord_residual
         << " max_span_curvature=" << m.maximum_reference_span_curvature
         << " max_filtered_curvature=" << m.maximum_filtered_curvature;
  return stream.str();
}

TEST(OnlinePathFollowingClosedLoop, StrictMeasuredTwistScenariosRemainFiniteAndBounded)
{
  const auto online_params = asr::validation::makeOnlineParameters();
  const auto local_params = asr::validation::makeLocalParameters();

  for (const auto & scenario : asr::validation::makeScenarios()) {
    const auto result = asr::validation::runScenario(scenario);
    const std::string message = metricsMessage(result);

    EXPECT_TRUE(result.finite) << message;
    EXPECT_TRUE(result.all_twists_accepted) << message;
    EXPECT_TRUE(result.all_measured_states_unchanged) << message;
    EXPECT_FALSE(result.unexpected_guard) << message;
    EXPECT_EQ(result.metrics.final_reset_count, 0U) << message;
    EXPECT_EQ(result.metrics.final_overflow_count, 0U) << message;
    EXPECT_EQ(result.metrics.final_duplicate_count, 0U) << message;
    EXPECT_EQ(result.metrics.final_out_of_order_count, 0U) << message;
    EXPECT_LE(result.metrics.maximum_online_joint_rate, online_params.joint_rate_limit + 1.0e-9)
      << message;
    EXPECT_LE(result.metrics.maximum_local_joint_rate, local_params.joint_rate_limit + 1.0e-9)
      << message;
    EXPECT_LE(result.metrics.maximum_online_joint_angle, online_params.joint_limit + 1.0e-9)
      << message;
    EXPECT_LE(result.metrics.maximum_local_joint_angle, local_params.joint_limit + 1.0e-9)
      << message;
    EXPECT_GE(result.metrics.final_actual_coverage,
      result.metrics.required_history - online_params.path_sample_ds) << message;

    if (scenario.name == "smooth_3d") {
      EXPECT_EQ(scenario.tape.size(), asr::validation::kWarmupSteps + 1500U) << message;
      // The amplitude-preserving fast profile reaches about 64% of the curvature
      // limit and 0.45 m^-1 s^-1 of applied curvature-vector variation rate.
      EXPECT_GT(result.metrics.maximum_curvature_ratio, 0.60) << message;
      EXPECT_LT(result.metrics.maximum_curvature_ratio, 0.70) << message;
      EXPECT_GT(result.metrics.maximum_online_curvature_rate, 0.42) << message;
      EXPECT_LT(result.metrics.maximum_online_curvature_rate, 0.48) << message;
    }

    if (scenario.is_high_curvature) {
      EXPECT_GT(result.metrics.high_curvature_samples, 0U) << message;
      EXPECT_EQ(result.metrics.high_curvature_limited_samples,
        result.metrics.high_curvature_samples) << message;
      EXPECT_LE(result.metrics.maximum_curvature_ratio, 1.0 + 1.0e-9) << message;
      continue;
    }

    EXPECT_GT(result.metrics.score_samples, 50U) << message;
    EXPECT_GT(result.metrics.common_error_samples, 50U) << message;
    EXPECT_EQ(result.metrics.fallback_samples, 0U) << message;
    EXPECT_EQ(result.metrics.shape_memory_samples, result.metrics.score_samples) << message;
    EXPECT_EQ(result.metrics.reference_valid_samples, result.metrics.score_samples) << message;
    EXPECT_LE(result.metrics.maximum_chord_residual,
      online_params.chord_residual_tolerance * 2.0) << message;

    for (size_t follower = 0; follower < 3; ++follower) {
      const double absolute_ceiling = online_params.link_length * (0.30 + 0.15 * follower);
      EXPECT_LE(result.metrics.online_follower_error[follower].rms, absolute_ceiling) << message;
      // The local controller is an A/B reference, not a physical ground truth. This
      // broad bound catches severe online regressions without claiming superiority.
      const double local_rms = result.metrics.local_follower_error[follower].rms;
      EXPECT_LE(result.metrics.online_follower_error[follower].rms,
        std::max(absolute_ceiling, local_rms * 1.5 + 0.01)) << message;
    }

    if (scenario.is_stop_resume) {
      EXPECT_GT(result.metrics.hold_samples, 0U) << message;
      EXPECT_GE(result.metrics.frozen_hold_samples * 0.90, result.metrics.hold_samples * 0.85)
        << message;
    }
  }
}

}  // namespace

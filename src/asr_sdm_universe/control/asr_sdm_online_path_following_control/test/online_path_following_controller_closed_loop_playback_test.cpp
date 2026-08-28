#include "online_path_following_controller_closed_loop.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace
{

const asr::validation::Scenario & scenarioNamed(
  const std::vector<asr::validation::Scenario> & scenarios, const std::string & name)
{
  const auto match = std::find_if(scenarios.begin(), scenarios.end(), [&](const auto & scenario) {
        return scenario.name == name;
    });
  if (match == scenarios.end()) {
    throw std::runtime_error("Missing test scenario: " + name);
  }
  return *match;
}

TEST(OnlinePathFollowingClosedLoopPlayback, ResolvesWholeTraceAndNamedContiguousSegment)
{
  const auto scenarios = asr::validation::makeScenarios();
  const auto result = asr::validation::runScenario(scenarioNamed(scenarios, "stop_resume"));

  const auto all = asr::validation::resolvePlaybackRange(result, "all");
  EXPECT_EQ(all.first, 0U);
  EXPECT_EQ(all.last, result.trace.size() - 1);
  EXPECT_DOUBLE_EQ(all.start_time, result.trace.front().time);
  EXPECT_DOUBLE_EQ(all.end_time, result.trace.back().time);

  const auto hold = asr::validation::resolvePlaybackRange(result, "stop_hold");
  EXPECT_LT(hold.first, hold.last);
  EXPECT_EQ(result.scenario.tape[hold.first].segment, "stop_hold");
  EXPECT_EQ(result.scenario.tape[hold.last].segment, "stop_hold");
  EXPECT_EQ(result.scenario.tape[hold.first - 1].segment, "stop_settling");
  EXPECT_EQ(result.scenario.tape[hold.last + 1].segment, "resume_settling");
}

TEST(OnlinePathFollowingClosedLoopPlayback, MapsTimesMonotonicallyAndClampsBounds)
{
  const auto scenarios = asr::validation::makeScenarios();
  const auto result = asr::validation::runScenario(scenarioNamed(scenarios, "constant_curvature"));
  const auto range = asr::validation::resolvePlaybackRange(result, "curve_score");

  EXPECT_EQ(asr::validation::playbackIndexAtOrBefore(result, range, range.start_time - 1.0),
    range.first);
  EXPECT_EQ(asr::validation::playbackIndexAtOrBefore(result, range, range.end_time + 1.0),
    range.last);

  const double midpoint = 0.5 * (range.start_time + range.end_time);
  const size_t midpoint_index = asr::validation::playbackIndexAtOrBefore(result, range, midpoint);
  EXPECT_GE(midpoint_index, range.first);
  EXPECT_LE(midpoint_index, range.last);
  EXPECT_LE(result.trace[midpoint_index].time, midpoint);
  if (midpoint_index < range.last) {
    EXPECT_GT(result.trace[midpoint_index + 1].time, midpoint);
  }
}

TEST(OnlinePathFollowingClosedLoopPlayback, SmoothSpatialSegmentRemainsThirtySecondReplay)
{
  const auto scenarios = asr::validation::makeScenarios();
  const auto result = asr::validation::runScenario(scenarioNamed(scenarios, "smooth_3d"));
  const auto range = asr::validation::resolvePlaybackRange(result, "smooth_3d_score");

  EXPECT_EQ(range.last - range.first + 1, 1500U);
  EXPECT_NEAR(range.end_time - range.start_time, 29.98, 1.0e-9);
}

TEST(OnlinePathFollowingClosedLoopPlayback, InterpolatesDisplayBodyAndBoundsTrail)
{
  const auto scenarios = asr::validation::makeScenarios();
  const auto result = asr::validation::runScenario(scenarioNamed(scenarios, "smooth_3d"));
  const auto range = asr::validation::resolvePlaybackRange(result, "smooth_3d_score");
  const size_t lower = range.first + 100;
  const size_t upper = lower + 1;
  const double midpoint = 0.5 * (result.trace[lower].time + result.trace[upper].time);
  const auto cursor = asr::validation::interpolatePlaybackDisplay(result, range, midpoint);

  EXPECT_EQ(cursor.lower_index, lower);
  EXPECT_EQ(cursor.upper_index, upper);
  EXPECT_NEAR(cursor.alpha, 0.5, 1.0e-12);
  for (size_t point = 0; point < asr::kNum3dPoints; ++point) {
    const auto & before = result.trace[lower].online_state.body_points[point];
    const auto & after = result.trace[upper].online_state.body_points[point];
    EXPECT_NEAR(cursor.body_points[point].x, 0.5 * (before.x + after.x), 1.0e-12);
    EXPECT_NEAR(cursor.body_points[point].y, 0.5 * (before.y + after.y), 1.0e-12);
    EXPECT_NEAR(cursor.body_points[point].z, 0.5 * (before.z + after.z), 1.0e-12);
  }

  const auto before_start = asr::validation::interpolatePlaybackDisplay(
    result, range, range.start_time - 1.0);
  const auto after_end = asr::validation::interpolatePlaybackDisplay(
    result, range, range.end_time + 1.0);
  EXPECT_EQ(before_start.lower_index, range.first);
  EXPECT_EQ(before_start.upper_index, range.first + 1);
  EXPECT_DOUBLE_EQ(before_start.alpha, 0.0);
  EXPECT_EQ(after_end.lower_index, range.last);
  EXPECT_EQ(after_end.upper_index, range.last);
  EXPECT_DOUBLE_EQ(after_end.alpha, 0.0);

  EXPECT_EQ(asr::validation::playbackTrailFirstIndex(range, range.first + 400, 360U),
    range.first + 41);
  EXPECT_EQ(asr::validation::playbackTrailFirstIndex(range, range.first + 100, 360U), range.first);
  EXPECT_EQ(asr::validation::playbackTrailFirstIndex(range, range.last, 2000U), range.first);
  EXPECT_THROW(asr::validation::playbackTrailFirstIndex(range, range.first, 0U),
      std::invalid_argument);
}

TEST(OnlinePathFollowingClosedLoopPlayback, RejectsUnknownSegment)
{
  const auto scenarios = asr::validation::makeScenarios();
  const auto result = asr::validation::runScenario(scenarioNamed(scenarios, "straight"));
  EXPECT_THROW(asr::validation::resolvePlaybackRange(result, "does_not_exist"),
      std::invalid_argument);
}

}  // namespace

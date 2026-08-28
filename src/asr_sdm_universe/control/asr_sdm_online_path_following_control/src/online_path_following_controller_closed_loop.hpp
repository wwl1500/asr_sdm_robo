#ifndef ONLINE_PATH_FOLLOWING_CONTROLLER_CLOSED_LOOP_HPP_
#define ONLINE_PATH_FOLLOWING_CONTROLLER_CLOSED_LOOP_HPP_

#include "asr_sdm_head_following_control/front_unit_following_controller_3d.hpp"
#include "asr_sdm_online_path_following_control/online_path_following_controller_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace asr::validation
{

constexpr double kClosedLoopDt = 0.02;
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kWarmupSteps = 420;

enum class Phase : uint8_t
{
  Warmup,
  Settle,
  Score,
  StopSettling,
  StopHold,
  ResumeSettling,
  ResumeScore,
  HighCurvature
};

struct CommandSample
{
  HeadCommand3D requested{};
  Phase phase{Phase::Warmup};
  std::string segment;
};

struct Scenario
{
  std::string name;
  std::vector<CommandSample> tape;
  bool is_stop_resume{false};
  bool is_high_curvature{false};
};

struct ErrorStats
{
  double sum_squared{0.0};
  double rms{0.0};
  double maximum{0.0};
  size_t count{0};

  void add(const Vec3 & error)
  {
    const double magnitude_squared = dot(error, error);
    sum_squared += magnitude_squared;
    maximum = std::max(maximum, std::sqrt(magnitude_squared));
    ++count;
  }

  void finish()
  {
    if (count > 0) {
      rms = std::sqrt(sum_squared / static_cast<double>(count));
    }
  }
};

struct TraceSample
{
  double time{0.0};
  Phase phase{Phase::Warmup};
  HeadCommand3D requested{};
  HeadCommand3D online_applied{};
  HeadCommand3D local_applied{};
  SimulationState3D online_state{};
  SimulationState3D local_state{};
  JointVelocity3D online_joint_velocity{};
  JointVelocity3D local_joint_velocity{};
  OnlinePathFollowing3DDiagnostics diagnostics{};
  bool twist_accepted{false};
  bool measured_state_unchanged{false};
};

struct Metrics
{
  std::array<ErrorStats, 3> online_follower_error{};
  std::array<ErrorStats, 3> local_follower_error{};
  size_t score_samples{0};
  size_t common_error_samples{0};
  size_t shape_memory_samples{0};
  size_t reference_valid_samples{0};
  size_t fallback_samples{0};
  size_t buffer_ready_samples{0};
  size_t new_reference_samples{0};
  size_t frozen_samples{0};
  size_t chord_failure_samples{0};
  size_t curvature_guard_samples{0};
  size_t axis_guard_samples{0};
  size_t continuity_guard_samples{0};
  size_t frozen_hold_samples{0};
  size_t hold_samples{0};
  size_t high_curvature_samples{0};
  size_t high_curvature_limited_samples{0};
  double maximum_online_joint_rate{0.0};
  double maximum_local_joint_rate{0.0};
  double maximum_online_joint_angle{0.0};
  double maximum_local_joint_angle{0.0};
  double maximum_curvature_ratio{0.0};
  double maximum_online_curvature_rate{0.0};
  double maximum_chord_residual{0.0};
  double maximum_reference_span_curvature{0.0};
  double maximum_filtered_curvature{0.0};
  double final_actual_coverage{0.0};
  double required_history{0.0};
  size_t final_reset_count{0};
  size_t final_overflow_count{0};
  size_t final_duplicate_count{0};
  size_t final_out_of_order_count{0};
};

struct Result
{
  Scenario scenario;
  std::vector<TraceSample> trace;
  Metrics metrics{};
  bool finite{true};
  bool all_twists_accepted{true};
  bool all_measured_states_unchanged{true};
  bool unexpected_guard{false};
};

inline void appendSegment(
  Scenario & scenario, const std::string & segment, const HeadCommand3D & command,
  size_t steps, Phase phase)
{
  for (size_t index = 0; index < steps; ++index) {
    scenario.tape.push_back({command, phase, segment});
  }
}

inline void appendWarmup(Scenario & scenario)
{
  appendSegment(scenario, "warmup", {0.12, 0.0, 0.0}, kWarmupSteps, Phase::Warmup);
}

inline std::vector<Scenario> makeScenarios()
{
  std::vector<Scenario> scenarios;

  Scenario straight;
  straight.name = "straight";
  appendWarmup(straight);
  appendSegment(straight, "straight_score", {0.12, 0.0, 0.0}, 300, Phase::Score);
  scenarios.push_back(straight);

  Scenario constant_curvature;
  constant_curvature.name = "constant_curvature";
  appendWarmup(constant_curvature);
  appendSegment(constant_curvature, "curve_settle", {0.12, 0.0, 0.004}, 80, Phase::Settle);
  appendSegment(constant_curvature, "curve_score", {0.12, 0.0, 0.004}, 350, Phase::Score);
  scenarios.push_back(constant_curvature);

  Scenario smooth_3d;
  smooth_3d.name = "smooth_3d";
  appendWarmup(smooth_3d);
  for (size_t index = 0; index < 1500; ++index) {
    const double t = static_cast<double>(index) * kClosedLoopDt;
    // Preserve curvature amplitude but increase temporal frequency so this
    // scenario exercises repeated, rapid 3D turn-direction changes.
    smooth_3d.tape.push_back({
        {0.12, 0.045 * std::sin(0.682 * t), 0.080 * std::sin(0.550 * t)},
        Phase::Score, "smooth_3d_score"});
  }
  scenarios.push_back(smooth_3d);

  Scenario stop_resume;
  stop_resume.name = "stop_resume";
  stop_resume.is_stop_resume = true;
  appendWarmup(stop_resume);
  appendSegment(stop_resume, "pre_stop", {0.10, 0.002, 0.003}, 180, Phase::Score);
  appendSegment(stop_resume, "stop_settling", {0.0, 0.0, 0.0}, 30, Phase::StopSettling);
  appendSegment(stop_resume, "stop_hold", {0.0, 0.0, 0.0}, 120, Phase::StopHold);
  appendSegment(stop_resume, "resume_settling", {0.10, 0.002, 0.003}, 50, Phase::ResumeSettling);
  appendSegment(stop_resume, "resume_score", {0.10, 0.002, 0.003}, 300, Phase::ResumeScore);
  scenarios.push_back(stop_resume);

  Scenario high_curvature;
  high_curvature.name = "high_curvature_limited";
  high_curvature.is_high_curvature = true;
  appendWarmup(high_curvature);
  appendSegment(high_curvature, "high_curvature_request", {0.12, 0.30, 0.50}, 250,
    Phase::HighCurvature);
  scenarios.push_back(high_curvature);

  return scenarios;
}

inline FrontUnitController3DParameters makeLocalParameters()
{
  FrontUnitController3DParameters params;
  params.link_length = 0.25;
  params.joint_rate_limit = 2.0;
  params.joint_limit = 0.85 * kPi;
  params.max_curvature = 1.2;
  params.curvature_velocity_epsilon = 1.0e-3;
  params.damping = 0.02;
  return params;
}

inline OnlinePathFollowing3DParameters makeOnlineParameters()
{
  OnlinePathFollowing3DParameters params;
  params.link_length = 0.25;
  params.joint_rate_limit = 2.0;
  params.joint_limit = 0.85 * kPi;
  params.max_curvature = 1.2;
  params.curvature_velocity_epsilon = 1.0e-3;
  params.damping = 0.02;
  params.path_sample_ds = 0.01;
  params.min_append_distance = 0.002;
  params.history_length = 1.30;
  params.min_shape_speed = 0.015;
  params.min_history_coverage = 0.0;
  params.joint_tracking_gain = 8.0;
  params.feedforward_ds = 0.02;
  params.feedforward_gain = 1.0;
  params.use_command_predictor_feedforward = true;
  // The benchmark models ideal measurements from applied commands. Removing the
  // command-conditioning transient keeps the sensor contract deterministic.
  params.command_filter_time_constant = 0.0;
  params.angular_rate_slew_limit = 100.0;
  params.linear_accel_limit = 100.0;
  params.front_motion_mode = FrontMotionMode::MeasuredTwistRequired;
  params.use_local_fallback = true;
  params.freeze_reference_when_stopped = true;
  return params;
}

inline std::vector<double> onlineCurvatureRateHistory(const Result & result)
{
  const OnlinePathFollowing3DParameters params = makeOnlineParameters();
  std::vector<double> values(result.trace.size(), 0.0);
  if (result.trace.size() < 2) {
    return values;
  }

  const auto curvatureVector = [&params](const HeadCommand3D & command) {
      const double speed = std::max(command.linear_velocity, params.curvature_velocity_epsilon);
      return std::array<double, 2>{command.pitch_rate / speed, command.yaw_rate / speed};
    };
  for (size_t index = 1; index < result.trace.size(); ++index) {
    const TraceSample & previous = result.trace[index - 1];
    const TraceSample & current = result.trace[index];
    const double dt = current.time - previous.time;
    if (!std::isfinite(dt) || dt <= 0.0 ||
      previous.online_applied.linear_velocity <= params.curvature_velocity_epsilon ||
      current.online_applied.linear_velocity <= params.curvature_velocity_epsilon)
    {
      continue;
    }
    const auto previous_curvature = curvatureVector(previous.online_applied);
    const auto current_curvature = curvatureVector(current.online_applied);
    values[index] = std::hypot(
      current_curvature[0] - previous_curvature[0],
      current_curvature[1] - previous_curvature[1]) / dt;
  }
  return values;
}

inline bool finiteVec(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline bool finiteFrame(const Mat3 & frame)
{
  for (size_t row = 0; row < 3; ++row) {
    for (size_t column_index = 0; column_index < 3; ++column_index) {
      if (!std::isfinite(frame.v[row][column_index])) {
        return false;
      }
    }
  }
  return true;
}

inline bool finiteState(const SimulationState3D & state)
{
  if (!std::isfinite(state.time) || !finiteVec(state.head_position) ||
    !finiteFrame(state.head_frame))
  {
    return false;
  }
  for (const double theta : state.joints.theta) {
    if (!std::isfinite(theta)) {
      return false;
    }
  }
  for (const Vec3 & point : state.body_points) {
    if (!finiteVec(point)) {
      return false;
    }
  }
  return true;
}

inline bool finiteJointVelocity(const JointVelocity3D & velocity)
{
  return std::all_of(velocity.theta_dot.begin(), velocity.theta_dot.end(),
           [](double value) {return std::isfinite(value);});
}

inline bool sameMeasuredState(const MeasuredRobotState3D & lhs, const MeasuredRobotState3D & rhs)
{
  if (lhs.stamp_sec != rhs.stamp_sec || lhs.tracking_frame_epoch != rhs.tracking_frame_epoch ||
    lhs.attitude_valid != rhs.attitude_valid || lhs.joints_valid != rhs.joints_valid ||
    lhs.joints.theta != rhs.joints.theta)
  {
    return false;
  }
  for (size_t row = 0; row < 3; ++row) {
    for (size_t column_index = 0; column_index < 3; ++column_index) {
      if (lhs.tracking_from_head.v[row][column_index] !=
        rhs.tracking_from_head.v[row][column_index])
      {
        return false;
      }
    }
  }
  return true;
}

inline RealizedFrontTwist3D makeIdealTwist(
  double stamp, const HeadCommand3D & previous_applied, const Mat3 & tracking_from_head)
{
  RealizedFrontTwist3D sample;
  sample.stamp_sec = stamp;
  sample.linear_velocity_head = {previous_applied.linear_velocity, 0.0, 0.0};
  sample.angular_velocity_head = {0.0, previous_applied.pitch_rate, previous_applied.yaw_rate};
  sample.tracking_from_head = tracking_from_head;
  sample.estimator_epoch = 1;
  sample.tracking_frame_epoch = 1;
  sample.twist_valid = true;
  sample.attitude_valid = true;
  sample.linear_velocity_measured = true;
  sample.angular_velocity_measured = true;
  return sample;
}

inline std::vector<double> cumulativeJoint1Distance(
  const std::vector<TraceSample> & trace,
  bool online)
{
  std::vector<double> distance(trace.size(), 0.0);
  for (size_t index = 1; index < trace.size(); ++index) {
    const auto & before = online ? trace[index - 1].online_state : trace[index - 1].local_state;
    const auto & after = online ? trace[index].online_state : trace[index].local_state;
    distance[index] = distance[index - 1] +
      norm(after.body_points[1] - before.body_points[1]);
  }
  return distance;
}

inline Vec3 sampleJoint1AtDistance(
  const std::vector<TraceSample> & trace, const std::vector<double> & distance,
  bool online, double s)
{
  const auto pointAt = [&](size_t index) -> const Vec3 & {
      return online ? trace[index].online_state.body_points[1] :
             trace[index].local_state.body_points[1];
    };
  if (s <= 0.0) {
    return pointAt(0);
  }
  if (s >= distance.back()) {
    return pointAt(distance.size() - 1);
  }
  const auto upper = std::upper_bound(distance.begin(), distance.end(), s);
  const size_t high = std::min<size_t>(
    static_cast<size_t>(std::distance(distance.begin(), upper)), distance.size() - 1);
  const size_t low = high == 0 ? 0 : high - 1;
  const double span = distance[high] - distance[low];
  if (span <= 1.0e-12) {
    return pointAt(high);
  }
  const double alpha = std::clamp((s - distance[low]) / span, 0.0, 1.0);
  return (1.0 - alpha) * pointAt(low) + alpha * pointAt(high);
}

inline bool findPreviousChordPoint(
  const std::vector<TraceSample> & trace, const std::vector<double> & distance, bool online,
  double front_s, const Vec3 & front_point, double link_length, double & back_s, Vec3 & back_point)
{
  constexpr double scan_step = 0.0025;
  if (front_s <= 0.0) {
    return false;
  }
  double near_s = front_s;
  for (double candidate = front_s - scan_step; candidate >= -1.0e-12; candidate -= scan_step) {
    const double far_s = std::max(0.0, candidate);
    const Vec3 far_point = sampleJoint1AtDistance(trace, distance, online, far_s);
    if (norm(front_point - far_point) >= link_length) {
      double far = far_s;
      double near = near_s;
      Vec3 root = far_point;
      for (size_t iteration = 0; iteration < 36; ++iteration) {
        const double mid = 0.5 * (far + near);
        const Vec3 point = sampleJoint1AtDistance(trace, distance, online, mid);
        if (norm(front_point - point) >= link_length) {
          far = mid;
          root = point;
        } else {
          near = mid;
        }
      }
      back_s = far;
      back_point = root;
      return std::abs(norm(front_point - back_point) - link_length) <= 1.0e-5;
    }
    near_s = far_s;
    if (far_s <= 1.0e-12) {
      break;
    }
  }
  return false;
}

inline bool followerTargets(
  const std::vector<TraceSample> & trace, const std::vector<double> & distance, bool online,
  size_t index, double link_length, std::array<Vec3, 3> & targets)
{
  if (index >= trace.size()) {
    return false;
  }
  double front_s = distance[index];
  Vec3 front_point = sampleJoint1AtDistance(trace, distance, online, front_s);
  for (size_t follower = 0; follower < targets.size(); ++follower) {
    double back_s = 0.0;
    if (!findPreviousChordPoint(
        trace, distance, online, front_s, front_point, link_length, back_s, targets[follower]))
    {
      return false;
    }
    front_s = back_s;
    front_point = targets[follower];
  }
  return true;
}

inline bool isScorePhase(Phase phase)
{
  return phase == Phase::Score || phase == Phase::ResumeScore;
}

inline bool isUnexpectedGuard(const OnlinePathFollowing3DDiagnostics & d)
{
  return d.sensor_queue_overflow || d.invalid_rotation_matrix || d.measured_state_stale ||
         d.measured_state_from_future || d.reverse_motion_detected ||
         d.control_time_jump_detected ||
         d.tracking_frame_redefined || d.buffer_capacity_overflow || d.joint_command_infeasible;
}

inline void updateExtrema(Metrics & metrics, const TraceSample & sample)
{
  for (size_t index = 0; index < kNum3dJointDofs; ++index) {
    metrics.maximum_online_joint_rate = std::max(
      metrics.maximum_online_joint_rate, std::abs(sample.online_joint_velocity.theta_dot[index]));
    metrics.maximum_local_joint_rate = std::max(
      metrics.maximum_local_joint_rate, std::abs(sample.local_joint_velocity.theta_dot[index]));
    metrics.maximum_online_joint_angle = std::max(
      metrics.maximum_online_joint_angle, std::abs(sample.online_state.joints.theta[index]));
    metrics.maximum_local_joint_angle = std::max(
      metrics.maximum_local_joint_angle, std::abs(sample.local_state.joints.theta[index]));
  }
  const auto & d = sample.diagnostics;
  const double denominator = std::max(
    1.0e-12, makeOnlineParameters().max_curvature * std::max(
      sample.online_applied.linear_velocity, makeOnlineParameters().curvature_velocity_epsilon));
  metrics.maximum_curvature_ratio = std::max(metrics.maximum_curvature_ratio,
    std::hypot(sample.online_applied.pitch_rate, sample.online_applied.yaw_rate) / denominator);
  metrics.maximum_chord_residual = std::max(metrics.maximum_chord_residual, d.max_chord_residual);
  metrics.maximum_reference_span_curvature = std::max(
    metrics.maximum_reference_span_curvature, d.max_curvature_on_reference_span);
  metrics.maximum_filtered_curvature = std::max(
    metrics.maximum_filtered_curvature, d.latest_filtered_curvature);
}

inline Result runScenario(const Scenario & scenario)
{
  const FrontUnitController3DParameters local_params = makeLocalParameters();
  const OnlinePathFollowing3DParameters online_params = makeOnlineParameters();
  FrontUnitFollowingController3D local_controller(local_params);
  OnlinePathFollowingController3D online_controller(online_params);
  KinematicPlant3D online_plant(online_controller.frontUnitParameters());
  SimulationState3D local_state = local_controller.makeInitialState();
  SimulationState3D online_state = online_plant.makeInitialState();
  HeadCommand3D previous_online_applied{0.0, 0.0, 0.0};

  Result result;
  result.scenario = scenario;
  result.trace.reserve(scenario.tape.size());

  for (const CommandSample & command : scenario.tape) {
    const MeasuredRobotState3D measured_before = online_plant.measuredState(online_state);
    const MeasuredRobotState3D measured = measured_before;
    const bool twist_accepted = online_controller.ingestRealizedFrontTwist(
      makeIdealTwist(online_state.time, previous_online_applied, online_state.head_frame));
    const OnlineFollowerOutput3D output = online_controller.step(
      command.requested, online_state.time, kClosedLoopDt, measured);
    const bool measured_unchanged = sameMeasuredState(measured_before, measured);
    online_plant.apply(
      output.applied_front_command, output.joint_velocity_command, kClosedLoopDt, online_state);
    const JointVelocity3D local_joint_velocity = local_controller.step(
      command.requested, kClosedLoopDt, local_state);

    TraceSample sample;
    sample.time = online_state.time;
    sample.phase = command.phase;
    sample.requested = command.requested;
    sample.online_applied = output.applied_front_command;
    sample.local_applied = local_controller.limitCommand(command.requested);
    sample.online_state = online_state;
    sample.local_state = local_state;
    sample.online_joint_velocity = output.joint_velocity_command;
    sample.local_joint_velocity = local_joint_velocity;
    sample.diagnostics = output.diagnostics;
    sample.twist_accepted = twist_accepted;
    sample.measured_state_unchanged = measured_unchanged;
    result.trace.push_back(sample);

    result.all_twists_accepted = result.all_twists_accepted && twist_accepted;
    result.all_measured_states_unchanged = result.all_measured_states_unchanged &&
      measured_unchanged;
    result.unexpected_guard = result.unexpected_guard || isUnexpectedGuard(output.diagnostics);
    result.finite = result.finite && finiteState(online_state) && finiteState(local_state) &&
      finiteJointVelocity(output.joint_velocity_command) &&
      finiteJointVelocity(local_joint_velocity);
    updateExtrema(result.metrics, sample);
    previous_online_applied = output.applied_front_command;
  }

  if (result.trace.empty()) {
    result.finite = false;
    return result;
  }

  const std::vector<double> curvature_rates = onlineCurvatureRateHistory(result);
  for (const double value : curvature_rates) {
    result.metrics.maximum_online_curvature_rate = std::max(
      result.metrics.maximum_online_curvature_rate, value);
  }

  const std::vector<double> online_distance = cumulativeJoint1Distance(result.trace, true);
  const std::vector<double> local_distance = cumulativeJoint1Distance(result.trace, false);
  for (size_t index = 0; index < result.trace.size(); ++index) {
    const TraceSample & sample = result.trace[index];
    const auto & d = sample.diagnostics;
    result.metrics.buffer_ready_samples += d.buffer_ready ? 1U : 0U;
    result.metrics.new_reference_samples += d.new_reference_valid ? 1U : 0U;
    result.metrics.frozen_samples += d.reference_frozen ? 1U : 0U;
    result.metrics.chord_failure_samples += d.chord_search_failed_this_step ? 1U : 0U;
    result.metrics.curvature_guard_samples += d.curvature_guard_triggered ? 1U : 0U;
    result.metrics.axis_guard_samples += d.axis_singularity_guard_triggered ? 1U : 0U;
    result.metrics.continuity_guard_samples += d.chord_continuity_guard_triggered ? 1U : 0U;
    if (sample.phase == Phase::StopHold) {
      ++result.metrics.hold_samples;
      result.metrics.frozen_hold_samples += d.reference_frozen ? 1U : 0U;
    }
    if (sample.phase == Phase::HighCurvature) {
      ++result.metrics.high_curvature_samples;
      const double limit = online_params.max_curvature * std::max(
        sample.online_applied.linear_velocity, online_params.curvature_velocity_epsilon);
      result.metrics.high_curvature_limited_samples +=
        std::hypot(sample.online_applied.pitch_rate,
          sample.online_applied.yaw_rate) <= limit + 1.0e-9 ? 1U : 0U;
    }
    if (!isScorePhase(sample.phase) || !d.buffer_ready || !d.using_shape_memory ||
      !d.reference_valid || d.reference_frozen ||
      d.joint1_path_speed < online_params.min_shape_speed)
    {
      continue;
    }
    ++result.metrics.score_samples;
    result.metrics.shape_memory_samples += d.using_shape_memory ? 1U : 0U;
    result.metrics.reference_valid_samples += d.reference_valid ? 1U : 0U;
    result.metrics.fallback_samples += d.using_local_fallback ? 1U : 0U;

    std::array<Vec3, 3> online_targets{};
    std::array<Vec3, 3> local_targets{};
    if (!followerTargets(result.trace, online_distance, true, index, online_params.link_length,
        online_targets) ||
      !followerTargets(result.trace, local_distance, false, index, local_params.link_length,
        local_targets))
    {
      continue;
    }
    ++result.metrics.common_error_samples;
    for (size_t follower = 0; follower < 3; ++follower) {
      result.metrics.online_follower_error[follower].add(
        sample.online_state.body_points[follower + 2] - online_targets[follower]);
      result.metrics.local_follower_error[follower].add(
        sample.local_state.body_points[follower + 2] - local_targets[follower]);
    }
  }

  for (ErrorStats & stats : result.metrics.online_follower_error) {
    stats.finish();
  }
  for (ErrorStats & stats : result.metrics.local_follower_error) {
    stats.finish();
  }
  const OnlinePathFollowing3DDiagnostics & final_diag = result.trace.back().diagnostics;
  result.metrics.final_actual_coverage = final_diag.formal_actual_coverage;
  result.metrics.required_history = final_diag.required_history;
  result.metrics.final_reset_count = final_diag.reset_count;
  result.metrics.final_overflow_count = final_diag.dropped_queue_overflow_samples;
  result.metrics.final_duplicate_count = final_diag.dropped_duplicate_samples;
  result.metrics.final_out_of_order_count = final_diag.dropped_out_of_order_samples;
  return result;
}

inline std::vector<double> times(const Result & result)
{
  std::vector<double> values;
  values.reserve(result.trace.size());
  for (const TraceSample & sample : result.trace) {
    values.push_back(sample.time);
  }
  return values;
}

inline std::vector<double> pointValues(const Result & result, bool online, size_t point, char axis)
{
  std::vector<double> values;
  values.reserve(result.trace.size());
  for (const TraceSample & sample : result.trace) {
    const Vec3 & value =
      online ? sample.online_state.body_points[point] : sample.local_state.body_points[point];
    values.push_back(axis == 'x' ? value.x : (axis == 'y' ? value.y : value.z));
  }
  return values;
}

inline std::array<std::vector<double>, 3> followerErrorHistory(const Result & result, bool online)
{
  std::array<std::vector<double>, 3> errors;
  for (std::vector<double> & values : errors) {
    values.assign(result.trace.size(), std::numeric_limits<double>::quiet_NaN());
  }
  const std::vector<double> distance = cumulativeJoint1Distance(result.trace, online);
  const double link_length =
    online ? makeOnlineParameters().link_length : makeLocalParameters().link_length;
  for (size_t index = 0; index < result.trace.size(); ++index) {
    std::array<Vec3, 3> targets{};
    if (!followerTargets(result.trace, distance, online, index, link_length, targets)) {
      continue;
    }
    for (size_t follower = 0; follower < 3; ++follower) {
      const auto & state =
        online ? result.trace[index].online_state : result.trace[index].local_state;
      errors[follower][index] = norm(state.body_points[follower + 2] - targets[follower]);
    }
  }
  return errors;
}

// ===================== 回放（playback）辅助 =====================

struct PlaybackRange
{
  size_t first{0};
  size_t last{0};
  double start_time{0.0};
  double end_time{0.0};
};

inline void validatePlaybackTrace(const Result & result)
{
  if (result.trace.empty() || result.trace.size() != result.scenario.tape.size()) {
    throw std::invalid_argument(
        "Playback requires a nonempty trace aligned with its scenario tape");
  }
  if (!std::isfinite(result.trace.front().time) || !std::isfinite(result.trace.back().time) ||
    result.trace.back().time < result.trace.front().time)
  {
    throw std::invalid_argument("Playback trace has invalid time bounds");
  }
}

inline PlaybackRange resolvePlaybackRange(const Result & result, const std::string & segment)
{
  validatePlaybackTrace(result);
  if (segment == "all") {
    return {0, result.trace.size() - 1, result.trace.front().time, result.trace.back().time};
  }

  size_t first = result.trace.size();
  size_t last = 0;
  for (size_t index = 0; index < result.scenario.tape.size(); ++index) {
    if (result.scenario.tape[index].segment != segment) {
      continue;
    }
    first = std::min(first, index);
    last = std::max(last, index);
  }
  if (first == result.trace.size()) {
    throw std::invalid_argument("Unknown playback segment: " + segment);
  }
  for (size_t index = first; index <= last; ++index) {
    if (result.scenario.tape[index].segment != segment) {
      throw std::invalid_argument("Playback segment is not contiguous: " + segment);
    }
  }
  return {first, last, result.trace[first].time, result.trace[last].time};
}

inline size_t playbackIndexAtOrBefore(
  const Result & result, const PlaybackRange & range, double desired_time)
{
  validatePlaybackTrace(result);
  if (range.first > range.last || range.last >= result.trace.size()) {
    throw std::invalid_argument("Playback range is outside the trace");
  }
  if (!std::isfinite(desired_time) || desired_time <= range.start_time) {
    return range.first;
  }
  if (desired_time >= range.end_time) {
    return range.last;
  }
  const auto first = result.trace.begin() + static_cast<std::ptrdiff_t>(range.first);
  const auto last = result.trace.begin() + static_cast<std::ptrdiff_t>(range.last + 1);
  const auto upper = std::upper_bound(first, last, desired_time,
      [](double time, const TraceSample & sample) {return time < sample.time;});
  if (upper == first) {
    return range.first;
  }
  return static_cast<size_t>(std::distance(result.trace.begin(), upper - 1));
}

struct PlaybackDisplayCursor
{
  double time{0.0};
  size_t lower_index{0};
  size_t upper_index{0};
  double alpha{0.0};
  std::array<Vec3, kNum3dPoints> body_points{};
};

inline PlaybackDisplayCursor interpolatePlaybackDisplay(
  const Result & result, const PlaybackRange & range, double desired_time)
{
  const size_t lower = playbackIndexAtOrBefore(result, range, desired_time);
  const double clamped_time = std::clamp(desired_time, range.start_time, range.end_time);
  size_t upper = lower;
  if (lower < range.last) {
    upper = lower + 1;
  }
  const double lower_time = result.trace[lower].time;
  const double upper_time = result.trace[upper].time;
  const double alpha = upper_time > lower_time ?
    std::clamp((clamped_time - lower_time) / (upper_time - lower_time), 0.0, 1.0) : 0.0;

  PlaybackDisplayCursor cursor;
  cursor.time = clamped_time;
  cursor.lower_index = lower;
  cursor.upper_index = upper;
  cursor.alpha = alpha;
  for (size_t point = 0; point < cursor.body_points.size(); ++point) {
    const Vec3 & before = result.trace[lower].online_state.body_points[point];
    const Vec3 & after = result.trace[upper].online_state.body_points[point];
    cursor.body_points[point] = (1.0 - alpha) * before + alpha * after;
  }
  return cursor;
}

inline size_t playbackTrailFirstIndex(
  const PlaybackRange & range, size_t latest_index, size_t max_samples)
{
  if (max_samples == 0) {
    throw std::invalid_argument("Playback trail sample count must be positive");
  }
  if (range.first > range.last || latest_index < range.first || latest_index > range.last) {
    throw std::invalid_argument("Playback trail index is outside the selected range");
  }
  const size_t available = latest_index - range.first + 1;
  return available <= max_samples ? range.first : latest_index - max_samples + 1;
}

}  // namespace asr::validation

#endif  // ONLINE_PATH_FOLLOWING_CONTROLLER_CLOSED_LOOP_HPP_

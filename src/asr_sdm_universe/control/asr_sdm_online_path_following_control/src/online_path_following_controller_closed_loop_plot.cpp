#include "online_path_following_controller_closed_loop.hpp"

#include <matplot/matplot.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace plt = matplot;
namespace fs = std::filesystem;

namespace
{

constexpr std::array<float, 3> kOnlineColor{0.0f, 0.447f, 0.698f};
constexpr std::array<float, 3> kLocalColor{0.835f, 0.329f, 0.0f};
constexpr std::array<float, 3> kHistoryColor{0.0f, 0.620f, 0.451f};
constexpr std::array<float, 3> kStateColor{0.800f, 0.475f, 0.655f};
constexpr std::array<float, 3> kThresholdColor{0.30f, 0.30f, 0.30f};

struct Options
{
  std::string scenario{"all"};
  std::string playback_segment{"all"};
  fs::path output_directory;
  double playback_speed{1.0};
  int refresh_ms{200};
  size_t trail_samples{360};
  bool show{false};
  bool animate{false};
  bool hold_final{true};
};

void printUsage(const char * executable)
{
  std::cout << "Usage: " << executable
            << " --output-dir <directory> [--scenario <name|all>] [--show]\n"
            << "       " << executable
            << " --output-dir <directory> --scenario <name> --animate"
            << " [--playback-segment <name|all>] [--playback-speed <positive>]"
            << " [--refresh-ms <positive>] [--trail-samples <positive>] [--no-hold-final]\n"
            << "Runs deterministic ideal-kinematic A/B controller validation.\n";
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--scenario" && index + 1 < argc) {
      options.scenario = argv[++index];
    } else if (argument == "--output-dir" && index + 1 < argc) {
      options.output_directory = argv[++index];
    } else if (argument == "--animate") {
      options.animate = true;
    } else if (argument == "--playback-segment" && index + 1 < argc) {
      options.playback_segment = argv[++index];
    } else if (argument == "--playback-speed" && index + 1 < argc) {
      options.playback_speed = std::stod(argv[++index]);
    } else if (argument == "--refresh-ms" && index + 1 < argc) {
      options.refresh_ms = std::stoi(argv[++index]);
    } else if (argument == "--trail-samples" && index + 1 < argc) {
      options.trail_samples = static_cast<size_t>(std::stoull(argv[++index]));
    } else if (argument == "--no-hold-final") {
      options.hold_final = false;
    } else if (argument == "--show") {
      options.show = true;
    } else if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown or incomplete argument: " + argument);
    }
  }
  if (options.output_directory.empty()) {
    throw std::invalid_argument("--output-dir is required");
  }
  if (!(options.playback_speed > 0.0) || !std::isfinite(options.playback_speed)) {
    throw std::invalid_argument("--playback-speed must be positive and finite");
  }
  if (options.refresh_ms <= 0) {
    throw std::invalid_argument("--refresh-ms must be positive");
  }
  if (options.trail_samples == 0) {
    throw std::invalid_argument("--trail-samples must be positive");
  }
  if (options.animate && options.scenario == "all") {
    throw std::invalid_argument("--animate requires one concrete --scenario");
  }
  if (options.animate && options.show) {
    throw std::invalid_argument("--animate and --show cannot be used together");
  }
  return options;
}

std::vector<asr::validation::Scenario> selectScenarios(const Options & options)
{
  const std::vector<asr::validation::Scenario> all = asr::validation::makeScenarios();
  if (options.scenario == "all") {
    return all;
  }
  const auto match = std::find_if(all.begin(), all.end(), [&](const auto & scenario) {
        return scenario.name == options.scenario;
    });
  if (match == all.end()) {
    throw std::invalid_argument("Unknown scenario: " + options.scenario);
  }
  return {*match};
}

std::vector<double> diagnosticValues(
  const asr::validation::Result & result, double asr::OnlinePathFollowing3DDiagnostics::* member)
{
  std::vector<double> values;
  values.reserve(result.trace.size());
  for (const auto & sample : result.trace) {
    values.push_back(sample.diagnostics.*member);
  }
  return values;
}

std::vector<double> boolDiagnosticValues(
  const asr::validation::Result & result, bool asr::OnlinePathFollowing3DDiagnostics::* member)
{
  std::vector<double> values;
  values.reserve(result.trace.size());
  for (const auto & sample : result.trace) {
    values.push_back(sample.diagnostics.*member ? 1.0 : 0.0);
  }
  return values;
}

std::vector<double> jointRateMagnitude(const asr::validation::Result & result, bool online)
{
  std::vector<double> values;
  values.reserve(result.trace.size());
  for (const auto & sample : result.trace) {
    const auto & velocity = online ? sample.online_joint_velocity : sample.local_joint_velocity;
    double squared_sum = 0.0;
    for (const double rate : velocity.theta_dot) {
      squared_sum += rate * rate;
    }
    values.push_back(std::sqrt(squared_sum));
  }
  return values;
}

std::vector<double> curvatureRatio(const asr::validation::Result & result)
{
  const auto params = asr::validation::makeOnlineParameters();
  std::vector<double> values;
  values.reserve(result.trace.size());
  for (const auto & sample : result.trace) {
    const double limit = params.max_curvature * std::max(
      sample.online_applied.linear_velocity, params.curvature_velocity_epsilon);
    values.push_back(std::hypot(sample.online_applied.pitch_rate, sample.online_applied.yaw_rate) /
      std::max(limit, 1.0e-12));
  }
  return values;
}

std::vector<double> finiteForPlot(const std::vector<double> & values)
{
  std::vector<double> result;
  result.reserve(values.size());
  for (const double value : values) {
    result.push_back(std::isfinite(value) ? value : 0.0);
  }
  return result;
}

void writeCsv(const asr::validation::Result & result, const fs::path & path)
{
  const auto online_errors = asr::validation::followerErrorHistory(result, true);
  const auto local_errors = asr::validation::followerErrorHistory(result, false);
  std::ofstream output(path);
  output << "# Deterministic ideal-kinematic simulation; not hardware/dynamics validation.\n";
  output <<
    "time_sec,phase,requested_v,requested_pitch,requested_yaw,online_v,online_pitch,online_yaw,"
         << "online_j1_x,online_j1_y,online_j1_z,online_tail_x,online_tail_y,online_tail_z,"
         << "local_j1_x,local_j1_y,local_j1_z,local_tail_x,local_tail_y,local_tail_z,"
         <<
    "online_error_j2,online_error_j3,online_error_tail,local_error_j2,local_error_j3,local_error_tail,"
         <<
    "history_m,required_history_m,shape_memory,reference_valid,reference_frozen,local_fallback,"
         << "online_joint_rate_norm,local_joint_rate_norm,curvature_ratio,"
         << "online_curvature_rate_m_inv_s\n";
  output << std::setprecision(12);
  const auto online_rates = jointRateMagnitude(result, true);
  const auto local_rates = jointRateMagnitude(result, false);
  const auto ratios = curvatureRatio(result);
  const auto curvature_rates = asr::validation::onlineCurvatureRateHistory(result);
  for (size_t index = 0; index < result.trace.size(); ++index) {
    const auto & sample = result.trace[index];
    const auto & online_j1 = sample.online_state.body_points[1];
    const auto & online_tail = sample.online_state.body_points[4];
    const auto & local_j1 = sample.local_state.body_points[1];
    const auto & local_tail = sample.local_state.body_points[4];
    const auto & d = sample.diagnostics;
    output << sample.time << ',' << static_cast<int>(sample.phase) << ','
           << sample.requested.linear_velocity << ',' << sample.requested.pitch_rate << ','
           << sample.requested.yaw_rate << ',' << sample.online_applied.linear_velocity << ','
           << sample.online_applied.pitch_rate << ',' << sample.online_applied.yaw_rate << ','
           << online_j1.x << ',' << online_j1.y << ',' << online_j1.z << ','
           << online_tail.x << ',' << online_tail.y << ',' << online_tail.z << ','
           << local_j1.x << ',' << local_j1.y << ',' << local_j1.z << ','
           << local_tail.x << ',' << local_tail.y << ',' << local_tail.z << ','
           << online_errors[0][index] << ',' << online_errors[1][index] << ','
           << online_errors[2][index] << ',' << local_errors[0][index] << ','
           << local_errors[1][index] << ',' << local_errors[2][index] << ','
           << d.formal_actual_coverage << ',' << d.required_history << ','
           << d.using_shape_memory << ',' << d.reference_valid << ',' << d.reference_frozen << ','
           << d.using_local_fallback << ',' << online_rates[index] << ',' << local_rates[index] <<
      ','
           << ratios[index] << ',' << curvature_rates[index] << '\n';
  }
}

void writeSummary(const std::vector<asr::validation::Result> & results, const fs::path & path)
{
  std::ofstream output(path);
  output <<
    "{\n  \"simulation\": \"deterministic ideal kinematic plant; not hardware or dynamics validation\",\n"
         << "  \"dt_sec\": " << asr::validation::kClosedLoopDt << ",\n  \"scenarios\": [\n";
  output << std::setprecision(8);
  for (size_t index = 0; index < results.size(); ++index) {
    const auto & result = results[index];
    const auto & metrics = result.metrics;
    output << "    {\"name\": \"" << result.scenario.name << "\", \"finite\": "
           << (result.finite ? "true" : "false") << ", \"score_samples\": "
           << metrics.score_samples << ", \"online_rms_m\": ["
           << metrics.online_follower_error[0].rms << ", "
           << metrics.online_follower_error[1].rms << ", "
           << metrics.online_follower_error[2].rms << "], \"local_rms_m\": ["
           << metrics.local_follower_error[0].rms << ", "
           << metrics.local_follower_error[1].rms << ", "
           << metrics.local_follower_error[2].rms << "], \"actual_coverage_m\": "
           << metrics.final_actual_coverage << ", \"required_history_m\": "
           << metrics.required_history << ", \"max_curvature_ratio\": "
           << metrics.maximum_curvature_ratio << ", \"max_online_curvature_rate_m_inv_s\": "
           << metrics.maximum_online_curvature_rate << "}";
    output << (index + 1 == results.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
}

void saveFigure(const std::shared_ptr<matplot::figure_type> & figure, const fs::path & stem)
{
  figure->save(stem.string() + ".svg");
  figure->save(stem.string() + ".png");
}

void drawTrajectory(const asr::validation::Result & result, const fs::path & output_directory)
{
  const auto figure = plt::figure(true);
  figure->width(1400);
  figure->height(950);
  plt::tiledlayout(2, 2);
  const auto three_d = plt::nexttile();
  plt::hold(three_d, true);
  auto online_joint1 = plt::plot3(three_d,
    asr::validation::pointValues(result, true, 1, 'x'),
    asr::validation::pointValues(result, true, 1, 'y'),
    asr::validation::pointValues(result, true, 1, 'z'));
  online_joint1->color(kOnlineColor);
  online_joint1->line_width(2.0);
  auto online_tail = plt::plot3(three_d,
    asr::validation::pointValues(result, true, 4, 'x'),
    asr::validation::pointValues(result, true, 4, 'y'),
    asr::validation::pointValues(result, true, 4, 'z'), "--");
  online_tail->color(kOnlineColor);
  auto local_joint1 = plt::plot3(three_d,
    asr::validation::pointValues(result, false, 1, 'x'),
    asr::validation::pointValues(result, false, 1, 'y'),
    asr::validation::pointValues(result, false, 1, 'z'));
  local_joint1->color(kLocalColor);
  local_joint1->line_width(2.0);
  auto local_tail = plt::plot3(three_d,
    asr::validation::pointValues(result, false, 4, 'x'),
    asr::validation::pointValues(result, false, 4, 'y'),
    asr::validation::pointValues(result, false, 4, 'z'), "--");
  local_tail->color(kLocalColor);
  plt::title(three_d, result.scenario.name + " — body trajectories");
  plt::xlabel(three_d, "x [m]");
  plt::ylabel(three_d, "y [m]");
  plt::zlabel(three_d, "z [m]");
  plt::grid(three_d, true);
  plt::legend(three_d, {"online joint1", "online tail", "local joint1", "local tail"});

  const auto xy = plt::nexttile();
  plt::hold(xy, true);
  auto online_xy = plt::plot(xy, asr::validation::pointValues(result, true, 1, 'x'),
    asr::validation::pointValues(result, true, 1, 'y'));
  online_xy->color(kOnlineColor);
  online_xy->line_width(2.0);
  auto local_xy = plt::plot(xy, asr::validation::pointValues(result, false, 1, 'x'),
    asr::validation::pointValues(result, false, 1, 'y'));
  local_xy->color(kLocalColor);
  local_xy->line_width(2.0);
  plt::title(xy, "joint1 XY projection");
  plt::xlabel(xy, "x [m]");
  plt::ylabel(xy, "y [m]");
  plt::grid(xy, true);
  plt::legend(xy, {"online", "local"});

  const auto draw_projection = [&](char horizontal_axis, char vertical_axis,
    const std::string & title) {
      const auto axes = plt::nexttile();
      plt::hold(axes, true);
      auto online = plt::plot(axes,
        asr::validation::pointValues(result, true, 1, horizontal_axis),
        asr::validation::pointValues(result, true, 1, vertical_axis));
      online->color(kOnlineColor);
      online->line_width(2.0);
      auto local = plt::plot(axes,
        asr::validation::pointValues(result, false, 1, horizontal_axis),
        asr::validation::pointValues(result, false, 1, vertical_axis));
      local->color(kLocalColor);
      local->line_width(2.0);
      plt::title(axes, title);
      plt::xlabel(axes, std::string(1, horizontal_axis) + " [m]");
      plt::ylabel(axes, std::string(1, vertical_axis) + " [m]");
      plt::grid(axes, true);
      plt::legend(axes, {"online", "local"});
    };
  draw_projection('x', 'z', "joint1 XZ projection");
  draw_projection('y', 'z', "joint1 YZ projection");
  saveFigure(figure, output_directory / (result.scenario.name + "_trajectory"));
}

void drawErrors(const asr::validation::Result & result, const fs::path & output_directory)
{
  const auto times = asr::validation::times(result);
  const auto online_errors = asr::validation::followerErrorHistory(result, true);
  const auto local_errors = asr::validation::followerErrorHistory(result, false);
  const auto figure = plt::figure(true);
  figure->width(1400);
  figure->height(900);
  plt::tiledlayout(3, 1);
  const std::array<std::string, 3> labels{"joint2", "joint3", "tail"};
  for (size_t follower = 0; follower < labels.size(); ++follower) {
    const auto axes = plt::nexttile();
    plt::hold(axes, true);
    auto online_line = plt::plot(axes, times, finiteForPlot(online_errors[follower]));
    online_line->color(kOnlineColor);
    online_line->line_width(1.5);
    auto local_line = plt::plot(axes, times, finiteForPlot(local_errors[follower]));
    local_line->color(kLocalColor);
    local_line->line_width(1.5);
    plt::title(axes, labels[follower] + " trailing chord error");
    plt::xlabel(axes, "time [s]");
    plt::ylabel(axes, "error [m]");
    plt::grid(axes, true);
    plt::legend(axes, {"online shape-memory", "local-DLS baseline"});
  }
  saveFigure(figure, output_directory / (result.scenario.name + "_errors"));
}

void drawDiagnostics(const asr::validation::Result & result, const fs::path & output_directory)
{
  const auto times = asr::validation::times(result);
  const auto figure = plt::figure(true);
  figure->width(1400);
  figure->height(1120);
  plt::tiledlayout(5, 1);

  const auto history = plt::nexttile();
  plt::hold(history, true);
  auto coverage = plt::plot(history, times,
    diagnosticValues(result, &asr::OnlinePathFollowing3DDiagnostics::formal_actual_coverage));
  coverage->color(kHistoryColor);
  coverage->line_width(1.8);
  auto required = plt::plot(history, times,
    diagnosticValues(result, &asr::OnlinePathFollowing3DDiagnostics::required_history), "--");
  required->color(kThresholdColor);
  plt::title(history, "formal actual history coverage");
  plt::ylabel(history, "length [m]");
  plt::grid(history, true);
  plt::legend(history, {"actual history", "required history"});

  const auto modes = plt::nexttile();
  plt::hold(modes, true);
  auto shape = plt::plot(modes, times,
    boolDiagnosticValues(result, &asr::OnlinePathFollowing3DDiagnostics::using_shape_memory));
  shape->color(kOnlineColor);
  auto valid = plt::plot(modes, times,
    boolDiagnosticValues(result, &asr::OnlinePathFollowing3DDiagnostics::reference_valid));
  valid->color(kHistoryColor);
  auto frozen = plt::plot(modes, times,
    boolDiagnosticValues(result, &asr::OnlinePathFollowing3DDiagnostics::reference_frozen));
  frozen->color(kStateColor);
  auto fallback = plt::plot(modes, times,
    boolDiagnosticValues(result, &asr::OnlinePathFollowing3DDiagnostics::using_local_fallback));
  fallback->color(kLocalColor);
  plt::title(modes, "online controller mode state");
  plt::ylabel(modes, "state [0/1]");
  plt::yrange(modes, {-0.05, 1.05});
  plt::grid(modes, true);
  plt::legend(modes, {"shape memory", "reference valid", "frozen", "local fallback"});

  const auto rates = plt::nexttile();
  plt::hold(rates, true);
  auto online_rate = plt::plot(rates, times, jointRateMagnitude(result, true));
  online_rate->color(kOnlineColor);
  auto local_rate = plt::plot(rates, times, jointRateMagnitude(result, false));
  local_rate->color(kLocalColor);
  plt::title(rates, "joint-command Euclidean norm");
  plt::ylabel(rates, "norm [rad/s]");
  plt::grid(rates, true);
  plt::legend(rates, {"online", "local"});

  const auto curvature = plt::nexttile();
  plt::hold(curvature, true);
  auto ratio = plt::plot(curvature, times, curvatureRatio(result));
  ratio->color(kStateColor);
  ratio->line_width(1.8);
  auto bound = plt::plot(curvature, times, std::vector<double>(times.size(), 1.0), "--");
  bound->color(kThresholdColor);
  plt::title(curvature, "applied front curvature ratio");
  plt::ylabel(curvature, "ratio [-]");
  plt::grid(curvature, true);
  plt::legend(curvature, {"applied / allowed", "allowed limit"});

  const auto curvature_rate = plt::nexttile();
  auto rate = plt::plot(curvature_rate, times,
    asr::validation::onlineCurvatureRateHistory(result));
  rate->color(kStateColor);
  rate->line_width(1.8);
  plt::title(curvature_rate, "applied curvature-vector variation rate");
  plt::xlabel(curvature_rate, "time [s]");
  plt::ylabel(curvature_rate, "rate [m^-1 s^-1]");
  plt::grid(curvature_rate, true);
  plt::legend(curvature_rate, std::vector<std::string>{"online applied"});
  saveFigure(figure, output_directory / (result.scenario.name + "_diagnostics"));
}

struct SpatialLimits
{
  std::array<std::array<double, 2>, 3> axes{};
};

std::vector<double> tracePointValues(
  const asr::validation::Result & result, size_t first, size_t last, size_t point, char axis)
{
  std::vector<double> values;
  values.reserve(last - first + 1);
  for (size_t index = first; index <= last; ++index) {
    const auto & value = result.trace[index].online_state.body_points[point];
    values.push_back(axis == 'x' ? value.x : (axis == 'y' ? value.y : value.z));
  }
  return values;
}

std::vector<double> currentBodyValues(const asr::SimulationState3D & state, char axis)
{
  std::vector<double> values;
  values.reserve(state.body_points.size());
  for (const auto & value : state.body_points) {
    values.push_back(axis == 'x' ? value.x : (axis == 'y' ? value.y : value.z));
  }
  return values;
}

SpatialLimits spatialLimits(
  const asr::validation::Result & result, const asr::validation::PlaybackRange & range)
{
  asr::Vec3 minimum = result.trace[range.first].online_state.body_points.front();
  asr::Vec3 maximum = minimum;
  for (size_t index = range.first; index <= range.last; ++index) {
    for (const auto & point : result.trace[index].online_state.body_points) {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
    }
  }
  const auto bounds = [](double low, double high) {
      const double span = std::max(high - low, 0.10);
      const double margin = std::max(0.05, 0.10 * span);
      return std::array<double, 2>{low - margin, high + margin};
    };
  return {{{bounds(minimum.x, maximum.x), bounds(minimum.y, maximum.y),
    bounds(minimum.z, maximum.z)}}};
}

class AnimatedTrajectoryFigure
{
public:
  AnimatedTrajectoryFigure(
    const asr::validation::Result & result, const asr::validation::PlaybackRange & range,
    size_t trail_samples)
  : result_(result), range_(range), trail_samples_(trail_samples),
    spatial_limits_(spatialLimits(result, range))
  {
    trajectory_figure_ = makeFigure(40, 40);
    xy_figure_ = makeFigure(752, 40);
    xz_figure_ = makeFigure(40, 530);
    yz_figure_ = makeFigure(752, 530);

    plt::figure(trajectory_figure_);
    trajectory_axes_ = plt::gca();
    plt::figure(xy_figure_);
    xy_panel_.axes = plt::gca();
    plt::figure(xz_figure_);
    xz_panel_.axes = plt::gca();
    plt::figure(yz_figure_);
    yz_panel_.axes = plt::gca();
    initialize();
  }

  bool shouldClose() const
  {
    return trajectory_figure_->should_close() || xy_figure_->should_close() ||
           xz_figure_->should_close() || yz_figure_->should_close();
  }

  void show()
  {
    trajectory_figure_->show();
    xy_figure_->show();
    xz_figure_->show();
    yz_figure_->show();
  }

  void draw(const asr::validation::PlaybackDisplayCursor & cursor)
  {
    const size_t trail_first = asr::validation::playbackTrailFirstIndex(
      range_, cursor.lower_index, trail_samples_);
    std::vector<double> trail_x = tracePointValues(
      result_, trail_first, cursor.lower_index, 1, 'x');
    std::vector<double> trail_y = tracePointValues(
      result_, trail_first, cursor.lower_index, 1, 'y');
    std::vector<double> trail_z = tracePointValues(
      result_, trail_first, cursor.lower_index, 1, 'z');
    trail_x.push_back(cursor.body_points[1].x);
    trail_y.push_back(cursor.body_points[1].y);
    trail_z.push_back(cursor.body_points[1].z);

    updateLine3(path_3d_, trail_x, trail_y, trail_z);
    updateLine3(current_body_3d_, bodyValues(cursor.body_points, 'x'),
      bodyValues(cursor.body_points, 'y'), bodyValues(cursor.body_points, 'z'));
    updateProjection(xy_panel_, trail_x, trail_y, cursor.body_points, 'x', 'y');
    updateProjection(xz_panel_, trail_x, trail_z, cursor.body_points, 'x', 'z');
    updateProjection(yz_panel_, trail_y, trail_z, cursor.body_points, 'y', 'z');

    const size_t snapshot_first = asr::validation::playbackTrailFirstIndex(
      range_, cursor.lower_index, trail_samples_);
    size_t snapshot_handle = 0;
    for (size_t sample = snapshot_first;
      sample <= cursor.lower_index && snapshot_handle < snapshot_handles_.size();
      sample += kBodySnapshotStride, ++snapshot_handle)
    {
      const auto & body = result_.trace[sample].online_state.body_points;
      updateLine3(snapshot_handles_[snapshot_handle], bodyValues(body, 'x'), bodyValues(body, 'y'),
        bodyValues(body, 'z'));
      snapshot_handles_[snapshot_handle]->visible(true);
    }
    for (; snapshot_handle < snapshot_handles_.size(); ++snapshot_handle) {
      snapshot_handles_[snapshot_handle]->visible(false);
    }

    plt::title(trajectory_axes_, result_.scenario.name + " — online shape-memory body, t=" +
      std::to_string(cursor.time) + " s");
    trajectory_figure_->draw();
    xy_figure_->draw();
    xz_figure_->draw();
    yz_figure_->draw();
  }

private:
  static constexpr size_t kBodySnapshotStride = 80;
  static constexpr int kWindowWidth = 700;
  static constexpr int kWindowHeight = 450;

  struct ProjectionPanel
  {
    matplot::axes_handle axes;
    matplot::line_handle path;
    matplot::line_handle body;
  };

  std::shared_ptr<matplot::figure_type> makeFigure(int x, int y) const
  {
    const auto figure = plt::figure(true);
    figure->position(x, y, kWindowWidth, kWindowHeight);
    return figure;
  }

  void updateLine3(
    const matplot::line_handle & handle, const std::vector<double> & x,
    const std::vector<double> & y, const std::vector<double> & z) const
  {
    handle->x_data(x);
    handle->y_data(y);
    handle->z_data(z);
  }

  std::vector<double> bodyValues(
    const std::array<asr::Vec3, asr::kNum3dPoints> & body, char axis) const
  {
    std::vector<double> values;
    values.reserve(body.size());
    for (const auto & point : body) {
      values.push_back(axis == 'x' ? point.x : (axis == 'y' ? point.y : point.z));
    }
    return values;
  }

  void updateProjection(
    const ProjectionPanel & panel, const std::vector<double> & horizontal_trail,
    const std::vector<double> & vertical_trail,
    const std::array<asr::Vec3, asr::kNum3dPoints> & body_points,
    char horizontal_axis, char vertical_axis) const
  {
    panel.path->x_data(horizontal_trail);
    panel.path->y_data(vertical_trail);
    panel.body->x_data(bodyValues(body_points, horizontal_axis));
    panel.body->y_data(bodyValues(body_points, vertical_axis));
  }

  size_t spatialAxisIndex(char axis) const
  {
    return axis == 'x' ? 0U : (axis == 'y' ? 1U : 2U);
  }

  void initializeProjection(
    ProjectionPanel & panel, const asr::SimulationState3D & first_state,
    char horizontal_axis, char vertical_axis, const std::string & title)
  {
    plt::hold(panel.axes, true);
    panel.path = plt::plot(panel.axes, std::vector<double>{}, std::vector<double>{});
    panel.path->color(kOnlineColor);
    panel.path->line_width(2.0);
    panel.body = plt::plot(panel.axes, currentBodyValues(first_state, horizontal_axis),
      currentBodyValues(first_state, vertical_axis), "-o");
    panel.body->color(kOnlineColor);
    panel.body->line_width(2.5);
    panel.body->marker_size(5.0);
    plt::title(panel.axes, title);
    plt::xlabel(panel.axes, std::string(1, horizontal_axis) + " [m]");
    plt::ylabel(panel.axes, std::string(1, vertical_axis) + " [m]");
    plt::grid(panel.axes, true);
    plt::xrange(panel.axes, spatial_limits_.axes[spatialAxisIndex(horizontal_axis)]);
    plt::yrange(panel.axes, spatial_limits_.axes[spatialAxisIndex(vertical_axis)]);
    plt::legend(panel.axes, {"joint1 path", "current body"});
  }

  void initialize()
  {
    const auto & first_state = result_.trace[range_.first].online_state;
    plt::hold(trajectory_axes_, true);
    path_3d_ = plt::plot3(trajectory_axes_, std::vector<double>{}, std::vector<double>{},
      std::vector<double>{});
    path_3d_->color(kOnlineColor);
    path_3d_->line_width(2.0);
    current_body_3d_ = plt::plot3(trajectory_axes_, currentBodyValues(first_state, 'x'),
      currentBodyValues(first_state, 'y'), currentBodyValues(first_state, 'z'), "-o");
    current_body_3d_->color(kOnlineColor);
    current_body_3d_->line_width(2.5);
    current_body_3d_->marker_size(5.0);
    plt::xlabel(trajectory_axes_, "x [m]");
    plt::ylabel(trajectory_axes_, "y [m]");
    plt::zlabel(trajectory_axes_, "z [m]");
    plt::view(trajectory_axes_, 45, 28);
    plt::grid(trajectory_axes_, true);
    plt::xrange(trajectory_axes_, spatial_limits_.axes[0]);
    plt::yrange(trajectory_axes_, spatial_limits_.axes[1]);
    trajectory_axes_->z_axis().limits(spatial_limits_.axes[2]);
    trajectory_axes_->z_axis().limits_mode_auto(false);

    const auto initial_body = currentBodyValues(first_state, 'x');
    initial_body_ = plt::plot3(trajectory_axes_, initial_body,
      currentBodyValues(first_state, 'y'), currentBodyValues(first_state, 'z'), "--s");
    initial_body_->color(kThresholdColor);
    initial_body_->line_width(1.0);
    initial_body_->marker_size(4.0);
    const size_t snapshot_count = std::max<size_t>(1, trail_samples_ / kBodySnapshotStride + 1);
    snapshot_handles_.reserve(snapshot_count);
    for (size_t index = 0; index < snapshot_count; ++index) {
      auto snapshot = plt::plot3(trajectory_axes_, std::vector<double>{}, std::vector<double>{},
        std::vector<double>{}, ".-");
      snapshot->color(kThresholdColor);
      snapshot->line_width(0.5);
      snapshot->marker_size(2.0);
      snapshot_handles_.push_back(snapshot);
    }
    plt::legend(trajectory_axes_, {"joint1 path", "current body", "initial body"});
    initializeProjection(xy_panel_, first_state, 'x', 'y', "joint1 XY tracking");
    initializeProjection(xz_panel_, first_state, 'x', 'z', "joint1 XZ tracking");
    initializeProjection(yz_panel_, first_state, 'y', 'z', "joint1 YZ tracking");
  }

  const asr::validation::Result & result_;
  asr::validation::PlaybackRange range_;
  size_t trail_samples_{0};
  SpatialLimits spatial_limits_;
  std::shared_ptr<matplot::figure_type> trajectory_figure_;
  std::shared_ptr<matplot::figure_type> xy_figure_;
  std::shared_ptr<matplot::figure_type> xz_figure_;
  std::shared_ptr<matplot::figure_type> yz_figure_;
  matplot::axes_handle trajectory_axes_;
  ProjectionPanel xy_panel_;
  ProjectionPanel xz_panel_;
  ProjectionPanel yz_panel_;
  matplot::line_handle path_3d_;
  matplot::line_handle current_body_3d_;
  matplot::line_handle initial_body_;
  std::vector<matplot::line_handle> snapshot_handles_;
};

void animateTrajectory(
  const asr::validation::Result & result, const asr::validation::PlaybackRange & range,
  const Options & options)
{
  AnimatedTrajectoryFigure figure(result, range, options.trail_samples);
  std::cout << "Animating " << result.scenario.name << " segment=" << options.playback_segment
            << " over [" << range.start_time << ", " << range.end_time << "] s at "
            << options.playback_speed << "x, motion refresh=" << options.refresh_ms
            << " ms, rolling trail=" << options.trail_samples << " samples\n";
  const auto first_cursor = asr::validation::interpolatePlaybackDisplay(
    result, range, range.start_time);
  figure.draw(first_cursor);
  const auto start = std::chrono::steady_clock::now();
  auto next_frame = start + std::chrono::milliseconds(options.refresh_ms);
  size_t frames_rendered = 1;
  size_t interpolated_frames = 0;
  size_t skipped_source_samples = 0;
  size_t previous_lower = first_cursor.lower_index;
  std::chrono::nanoseconds total_draw_time{0};
  std::chrono::nanoseconds max_draw_time{0};
  while (!figure.shouldClose()) {
    const auto now = std::chrono::steady_clock::now();
    const double desired_time = range.start_time +
      std::chrono::duration<double>(now - start).count() * options.playback_speed;
    const auto cursor = asr::validation::interpolatePlaybackDisplay(result, range, desired_time);
    const auto draw_start = std::chrono::steady_clock::now();
    figure.draw(cursor);
    const auto draw_time = std::chrono::steady_clock::now() - draw_start;
    total_draw_time += draw_time;
    max_draw_time = std::max(max_draw_time, draw_time);
    ++frames_rendered;
    interpolated_frames += cursor.alpha > 1.0e-12 ? 1U : 0U;
    skipped_source_samples += cursor.lower_index > previous_lower ?
      cursor.lower_index - previous_lower - 1 : 0;
    previous_lower = cursor.lower_index;
    if (cursor.lower_index == range.last) {
      break;
    }
    next_frame = std::chrono::steady_clock::now() + std::chrono::milliseconds(options.refresh_ms);
    std::this_thread::sleep_until(next_frame);
  }
  const auto final_cursor = asr::validation::interpolatePlaybackDisplay(result, range,
      range.end_time);
  if (!figure.shouldClose() && previous_lower != range.last) {
    figure.draw(final_cursor);
    ++frames_rendered;
  }
  const double observed_duration = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();
  const double mean_draw_ms = frames_rendered > 1 ?
    std::chrono::duration<double, std::milli>(total_draw_time).count() /
    static_cast<double>(frames_rendered - 1) : 0.0;
  const double max_draw_ms = std::chrono::duration<double, std::milli>(max_draw_time).count();
  std::cout << "Animation render: frames=" << frames_rendered << ", interpolated=" <<
    interpolated_frames << ", skipped_source_samples=" << skipped_source_samples <<
    ", observed_duration=" << observed_duration << " s, mean/max draw=" << mean_draw_ms << "/" <<
    max_draw_ms << " ms\n";
  std::ofstream rendering_report(options.output_directory / "animation_rendering.json");
  rendering_report << std::setprecision(12)
                   << "{\n  \"scenario\": \"" << result.scenario.name << "\",\n"
                   << "  \"segment\": \"" << options.playback_segment << "\",\n"
                   << "  \"target_duration_sec\": "
                   << (range.end_time - range.start_time) / options.playback_speed << ",\n"
                   << "  \"observed_duration_sec\": " << observed_duration << ",\n"
                   << "  \"refresh_ms\": " << options.refresh_ms << ",\n"
                   << "  \"trail_samples\": " << options.trail_samples << ",\n"
                   << "  \"rendered_frames\": " << frames_rendered << ",\n"
                   << "  \"interpolated_frames\": " << interpolated_frames << ",\n"
                   << "  \"skipped_source_samples\": " << skipped_source_samples << ",\n"
                   << "  \"mean_draw_ms\": " << mean_draw_ms << ",\n"
                   << "  \"max_draw_ms\": " << max_draw_ms << "\n}\n";
  if (!figure.shouldClose() && options.hold_final) {
    std::cout << "Animation complete. Press Enter in this terminal to close the final frame.\n";
    figure.show();
  }
}

void drawSummary(
  const std::vector<asr::validation::Result> & results,
  const fs::path & output_directory)
{
  std::vector<double> index;
  std::vector<double> online_tail_rms;
  std::vector<double> local_tail_rms;
  std::vector<std::string> names;
  for (size_t scenario_index = 0; scenario_index < results.size(); ++scenario_index) {
    index.push_back(static_cast<double>(scenario_index + 1));
    online_tail_rms.push_back(results[scenario_index].metrics.online_follower_error[2].rms);
    local_tail_rms.push_back(results[scenario_index].metrics.local_follower_error[2].rms);
    names.push_back(results[scenario_index].scenario.name);
  }
  const auto figure = plt::figure(true);
  figure->width(1300);
  figure->height(650);
  const auto axes = plt::gca();
  plt::hold(axes, true);
  auto online = plt::plot(axes, index, online_tail_rms, "o-");
  online->color(kOnlineColor);
  online->line_width(2.0);
  online->marker_size(7.0);
  auto local = plt::plot(axes, index, local_tail_rms, "s-");
  local->color(kLocalColor);
  local->line_width(2.0);
  local->marker_size(7.0);
  plt::title(axes, "A/B tail trailing-chord RMS (mature valid windows only)");
  plt::xlabel(axes, "scenario index (see CSV / summary.json)");
  plt::ylabel(axes, "RMS error [m]");
  plt::grid(axes, true);
  plt::legend(axes, {"online shape-memory", "local-DLS baseline"});
  saveFigure(figure, output_directory / "ab_summary");
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    fs::create_directories(options.output_directory);
    std::vector<asr::validation::Result> results;
    for (const auto & scenario : selectScenarios(options)) {
      auto result = asr::validation::runScenario(scenario);
      writeCsv(result, options.output_directory / (scenario.name + ".csv"));
      if (!options.animate) {
        drawTrajectory(result, options.output_directory);
        drawErrors(result, options.output_directory);
        drawDiagnostics(result, options.output_directory);
      }
      std::cout << scenario.name << ": finite=" << std::boolalpha << result.finite
                << ", score_samples=" << result.metrics.score_samples
                << ", tail_rms_online=" << result.metrics.online_follower_error[2].rms
                << ", tail_rms_local=" << result.metrics.local_follower_error[2].rms << '\n';
      results.push_back(std::move(result));
    }
    writeSummary(results, options.output_directory / "summary.json");
    if (!options.animate) {
      drawSummary(results, options.output_directory);
    }
    if (options.animate) {
      const auto range = asr::validation::resolvePlaybackRange(
        results.front(), options.playback_segment);
      animateTrajectory(results.front(), range, options);
    }
    std::cout << "Artifacts written to " << options.output_directory << '\n';
    std::cout << "Caveat: results use an ideal deterministic kinematic plant; they do not validate "
              << "hardware, actuators, estimator noise, delays, or vehicle dynamics.\n";
    if (options.show) {
      plt::show();
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Validation plotting failed: " << error.what() << '\n';
    printUsage(argv[0]);
    return 2;
  }
}

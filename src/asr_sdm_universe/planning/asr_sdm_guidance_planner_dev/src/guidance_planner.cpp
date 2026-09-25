// Copyright (c) Amphibious Robotics.
// Guidance path planner pipeline implementation.

#include <guidance_planner.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>

namespace amprobo
{

GuidancePlanner::GuidancePlanner(const GuidancePlannerOptions & options)
{
  setOptions(options);
}

void GuidancePlanner::initGuidancePlanner(const std::shared_ptr<rclcpp::Node> & nh)
{
  GuidancePlannerOptions options;

  /* get parameter — ROS 2 dotted names match astar_lbfgs_planner.yaml */
  nh->declare_parameter("selection.use_optimized_only_if_safe", true);
  nh->declare_parameter("selection.project_start_goal_to_safe", true);
  nh->declare_parameter("selection.safe_point_search_radius", 1.5);
  options.use_optimized_only_if_safe =
    nh->get_parameter("selection.use_optimized_only_if_safe").as_bool();
  options.project_start_goal_to_safe =
    nh->get_parameter("selection.project_start_goal_to_safe").as_bool();
  options.safe_point_search_radius =
    nh->get_parameter("selection.safe_point_search_radius").as_double();

  nh->declare_parameter("astar.heuristic_weight", 1.0);
  nh->declare_parameter("astar.extra_clearance", 0.0);
  nh->declare_parameter("astar.max_expansions", 300000);
  nh->declare_parameter("astar.nearest_free_search_radius", 10);
  nh->declare_parameter("astar.allow_diagonal", true);
  options.astar.heuristic_weight = nh->get_parameter("astar.heuristic_weight").as_double();
  options.astar.extra_clearance = nh->get_parameter("astar.extra_clearance").as_double();
  options.astar.max_expansions =
    static_cast<int>(nh->get_parameter("astar.max_expansions").as_int());
  options.astar.nearest_free_search_radius =
    static_cast<int>(nh->get_parameter("astar.nearest_free_search_radius").as_int());
  options.astar.allow_diagonal = nh->get_parameter("astar.allow_diagonal").as_bool();

  nh->declare_parameter("corridor.enabled", true);
  nh->declare_parameter("corridor.batch_sample_count", 80);
  nh->declare_parameter("corridor.max_spheres", 80);
  nh->declare_parameter("corridor.drone_radius", 0.0);
  nh->declare_parameter("corridor.safety_margin", 0.10);
  nh->declare_parameter("corridor.min_radius", 0.10);
  nh->declare_parameter("corridor.max_radius", 6.0);
  nh->declare_parameter("corridor.min_overlap_volume", 1.0e-5);
  nh->declare_parameter("corridor.radius_weight", 1.0);
  nh->declare_parameter("corridor.overlap_weight", 4.0);
  nh->declare_parameter("corridor.sample_axis_scale", 0.3333333333333333);
  nh->declare_parameter("corridor.sample_lateral_scale", 2.0);
  nh->declare_parameter("corridor.random_seed", 7);
  nh->declare_parameter("corridor.deterministic_sampling", true);
  options.corridor.enabled = nh->get_parameter("corridor.enabled").as_bool();
  options.corridor.batch_sample_count =
    static_cast<int>(nh->get_parameter("corridor.batch_sample_count").as_int());
  options.corridor.max_spheres =
    static_cast<int>(nh->get_parameter("corridor.max_spheres").as_int());
  options.corridor.drone_radius = nh->get_parameter("corridor.drone_radius").as_double();
  options.corridor.safety_margin = nh->get_parameter("corridor.safety_margin").as_double();
  options.corridor.min_radius = nh->get_parameter("corridor.min_radius").as_double();
  options.corridor.max_radius = nh->get_parameter("corridor.max_radius").as_double();
  options.corridor.min_overlap_volume =
    nh->get_parameter("corridor.min_overlap_volume").as_double();
  options.corridor.radius_weight = nh->get_parameter("corridor.radius_weight").as_double();
  options.corridor.overlap_weight = nh->get_parameter("corridor.overlap_weight").as_double();
  options.corridor.sample_axis_scale =
    nh->get_parameter("corridor.sample_axis_scale").as_double();
  options.corridor.sample_lateral_scale =
    nh->get_parameter("corridor.sample_lateral_scale").as_double();
  options.corridor.random_seed =
    static_cast<uint32_t>(nh->get_parameter("corridor.random_seed").as_int());
  options.corridor.deterministic_sampling =
    nh->get_parameter("corridor.deterministic_sampling").as_bool();

  nh->declare_parameter("optimizer.enabled", true);
  nh->declare_parameter("optimizer.max_iterations", 120);
  nh->declare_parameter("optimizer.max_control_points", 80);
  nh->declare_parameter("optimizer.epsilon", 1.0e-4);
  nh->declare_parameter("optimizer.smooth_weight", 1.0);
  nh->declare_parameter("optimizer.length_weight", 0.05);
  nh->declare_parameter("optimizer.obstacle_weight", 12.0);
  nh->declare_parameter("optimizer.guide_weight", 0.08);
  nh->declare_parameter("optimizer.safe_distance", 0.20);
  nh->declare_parameter("optimizer.validity_check_step", 0.05);
  nh->declare_parameter("optimizer.extra_clearance", 0.0);
  nh->declare_parameter("optimizer.corridor_weight", 60.0);
  options.optimizer.enabled = nh->get_parameter("optimizer.enabled").as_bool();
  options.optimizer.max_iterations =
    static_cast<int>(nh->get_parameter("optimizer.max_iterations").as_int());
  options.optimizer.max_control_points =
    static_cast<int>(nh->get_parameter("optimizer.max_control_points").as_int());
  options.optimizer.epsilon = nh->get_parameter("optimizer.epsilon").as_double();
  options.optimizer.smooth_weight = nh->get_parameter("optimizer.smooth_weight").as_double();
  options.optimizer.length_weight = nh->get_parameter("optimizer.length_weight").as_double();
  options.optimizer.obstacle_weight = nh->get_parameter("optimizer.obstacle_weight").as_double();
  options.optimizer.guide_weight = nh->get_parameter("optimizer.guide_weight").as_double();
  options.optimizer.safe_distance = nh->get_parameter("optimizer.safe_distance").as_double();
  options.optimizer.validity_check_step =
    nh->get_parameter("optimizer.validity_check_step").as_double();
  options.optimizer.extra_clearance = nh->get_parameter("optimizer.extra_clearance").as_double();
  options.optimizer.corridor_weight = nh->get_parameter("optimizer.corridor_weight").as_double();

  setOptions(options);
}

void GuidancePlanner::setOptions(const GuidancePlannerOptions & options)
{
  options_ = options;
  options_.safe_point_search_radius = std::max(0.0, options_.safe_point_search_radius);
  astar_planner_.setOptions(options_.astar);
  corridor_generator_.setOptions(options_.corridor);
  optimizer_.setOptions(options_.optimizer);
}

GuidancePlannerResult GuidancePlanner::plan(
  const MapQueryInterface & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  GuidancePlannerResult result;
  result.requested_start = start;
  result.requested_goal = goal;
  result.planning_start = start;
  result.planning_goal = goal;

  if (!map.isReady()) {
    result.message =
      "Map is not ready. Load occupancy + ESDF binaries or wait for both map topics first.";
    return result;
  }

  const auto cycle_start_time = std::chrono::steady_clock::now();
  double start_projection_time_ms = 0.0;
  double goal_projection_time_ms = 0.0;
  double astar_time_ms = 0.0;
  double corridor_time_ms = 0.0;
  double optimizer_time_ms = 0.0;

  const auto makeTimingSummary = [&](const double total_time_ms) {
    const double projection_time_ms = start_projection_time_ms + goal_projection_time_ms;
    const double accounted_time_ms =
      projection_time_ms + astar_time_ms + corridor_time_ms + optimizer_time_ms;
    const double other_time_ms = std::max(0.0, total_time_ms - accounted_time_ms);

    std::ostringstream timing;
    timing << "Guidance planner cycle time: total=" << total_time_ms << " ms"
           << ", projection=" << projection_time_ms << " ms"
           << " (start=" << start_projection_time_ms << " ms"
           << ", goal=" << goal_projection_time_ms << " ms)"
           << ", astar=" << astar_time_ms << " ms"
           << ", corridor=" << corridor_time_ms << " ms"
           << ", optimizer=" << optimizer_time_ms << " ms"
           << ", other=" << other_time_ms << " ms";
    return timing.str();
  };

  if (options_.project_start_goal_to_safe && corridor_generator_.options().enabled) {
    const auto start_projection_start_time = std::chrono::steady_clock::now();
    const bool start_projection_success = findNearestSafePlanningPoint(
      map, start, "start", result.planning_start, result.start_projection_status);
    const auto start_projection_end_time = std::chrono::steady_clock::now();
    start_projection_time_ms = std::chrono::duration<double, std::milli>(
                                 start_projection_end_time - start_projection_start_time)
                                 .count();

    if (!start_projection_success) {
      const auto cycle_end_time = std::chrono::steady_clock::now();
      const double total_time_ms =
        std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();
      result.timing_summary = makeTimingSummary(total_time_ms);
      result.message = result.start_projection_status + " | " + result.timing_summary;
      return result;
    }

    const auto goal_projection_start_time = std::chrono::steady_clock::now();
    const bool goal_projection_success = findNearestSafePlanningPoint(
      map, goal, "goal", result.planning_goal, result.goal_projection_status);
    const auto goal_projection_end_time = std::chrono::steady_clock::now();
    goal_projection_time_ms = std::chrono::duration<double, std::milli>(
                                goal_projection_end_time - goal_projection_start_time)
                                .count();

    if (!goal_projection_success) {
      const auto cycle_end_time = std::chrono::steady_clock::now();
      const double total_time_ms =
        std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();
      result.timing_summary = makeTimingSummary(total_time_ms);
      result.message = result.goal_projection_status + " | " + result.timing_summary;
      return result;
    }

    result.start_projected = (result.planning_start - start).norm() > 0.5 * map.resolution();
    result.goal_projected = (result.planning_goal - goal).norm() > 0.5 * map.resolution();
  }

  const auto astar_start_time = std::chrono::steady_clock::now();
  PlanResult plan = astar_planner_.plan(map, result.planning_start, result.planning_goal);
  const auto astar_end_time = std::chrono::steady_clock::now();
  astar_time_ms =
    std::chrono::duration<double, std::milli>(astar_end_time - astar_start_time).count();
  result.raw_path = plan.raw_path;

  if (!plan.success) {
    const auto cycle_end_time = std::chrono::steady_clock::now();
    const double total_time_ms =
      std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();
    result.timing_summary = makeTimingSummary(total_time_ms);
    result.message = plan.message + " | " + result.timing_summary;
    return result;
  }

  SphereCorridorResult corridor;
  std::vector<Eigen::Vector3d> optimizer_input = plan.raw_path;
  const std::vector<CorridorSphere> * corridor_ptr = nullptr;

  if (corridor_generator_.options().enabled) {
    const auto corridor_start_time = std::chrono::steady_clock::now();
    corridor =
      corridor_generator_.generate(map, plan.raw_path, result.planning_start, result.planning_goal);
    const auto corridor_end_time = std::chrono::steady_clock::now();
    corridor_time_ms =
      std::chrono::duration<double, std::milli>(corridor_end_time - corridor_start_time).count();

    if (!corridor.success) {
      const auto cycle_end_time = std::chrono::steady_clock::now();
      const double total_time_ms =
        std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();
      result.timing_summary = makeTimingSummary(total_time_ms);
      result.message = plan.message + " | " + corridor.message + " | " + result.timing_summary;
      return result;
    }

    optimizer_input = corridor.waypoints;
    corridor_ptr = &corridor.spheres;
    result.corridor_waypoints = corridor.waypoints;
    result.corridor_spheres = corridor.spheres;
  } else {
    result.corridor_waypoints = plan.raw_path;
  }

  const auto optimizer_start_time = std::chrono::steady_clock::now();
  OptimizerResult opt = optimizer_.optimize(optimizer_input, map, corridor_ptr);
  const auto optimizer_end_time = std::chrono::steady_clock::now();
  optimizer_time_ms =
    std::chrono::duration<double, std::milli>(optimizer_end_time - optimizer_start_time).count();

  const auto cycle_end_time = std::chrono::steady_clock::now();
  const double total_time_ms =
    std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();

  result.final_waypoints = opt.path;
  if (!opt.success || (options_.use_optimized_only_if_safe && !opt.path_safe)) {
    result.used_fallback = true;
    result.final_waypoints = optimizer_input;
  }

  result.timing_summary = makeTimingSummary(total_time_ms);

  std::ostringstream status;
  if (result.start_projected) {
    status << result.start_projection_status << " | ";
  }
  if (result.goal_projected) {
    status << result.goal_projection_status << " | ";
  }
  status << plan.message;
  if (corridor_generator_.options().enabled) {
    status << " | " << corridor.message;
  }
  status << " | " << opt.message << " | " << result.timing_summary;
  if (result.used_fallback) {
    status << " | fallback: corridor/default waypoints published";
  }

  result.success = true;
  result.message = status.str();
  return result;
}

double GuidancePlanner::corridorRequiredClearance() const
{
  const auto & options = corridor_generator_.options();
  return std::max(0.0, options.drone_radius + options.safety_margin + options.min_radius);
}

bool GuidancePlanner::findNearestSafePlanningPoint(
  const MapQueryInterface & map, const Eigen::Vector3d & seed, const std::string & label,
  Eigen::Vector3d & safe_point, std::string & status) const
{
  if (!map.hasDistanceField()) {
    status = "Cannot project " + label + " point: ESDF is not ready.";
    return false;
  }

  Eigen::Vector3d query = seed;
  const double eps = 0.5 * map.resolution();
  const Eigen::Vector3d lower = map.origin() + Eigen::Vector3d::Constant(eps);
  const Eigen::Vector3d upper = map.origin() + map.size() - Eigen::Vector3d::Constant(eps);
  for (int axis = 0; axis < 3; ++axis) {
    query(axis) = std::clamp(query(axis), lower(axis), upper(axis));
  }

  GridIndex current_index;
  if (!map.worldToIndex(query, current_index)) {
    std::ostringstream oss;
    oss << "Cannot project " << label << " point: point is outside map bounds.";
    status = oss.str();
    return false;
  }

  const double required_clearance = corridorRequiredClearance();
  const double step_size = std::max(map.resolution(), 1.0e-3);
  const int max_steps =
    std::max(1, static_cast<int>(std::ceil(options_.safe_point_search_radius / step_size)));
  const double max_distance = options_.safe_point_search_radius + 0.5 * map.resolution();
  const double min_progress = 1.0e-6;

  int inspected_points = 0;
  int gradient_steps = 0;
  double best_seen_clearance = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_seen_point = map.indexToWorld(current_index);

  const auto updateBestSeen = [&](const GridIndex & index, const double clearance) {
    if (std::isfinite(clearance) && clearance > best_seen_clearance) {
      best_seen_clearance = clearance;
      best_seen_point = map.indexToWorld(index);
    }
  };

  const auto isSafeIndex = [&](const GridIndex & index, double & clearance) -> bool {
    ++inspected_points;
    if (!map.isInMap(index) || !map.isFree(index)) {
      clearance = 0.0;
      return false;
    }

    clearance = map.distance(index);
    updateBestSeen(index, clearance);
    return std::isfinite(clearance) && clearance >= required_clearance;
  };

  const auto isUsableGradientIndex = [&](const GridIndex & index, double & clearance) -> bool {
    ++inspected_points;
    if (!map.isInMap(index)) {
      clearance = 0.0;
      return false;
    }

    clearance = map.distance(index);
    updateBestSeen(index, clearance);
    return std::isfinite(clearance);
  };

  const auto chooseDiscreteGradientNeighbor =
    [&](
      const GridIndex & center, const double center_clearance, GridIndex & next_index,
      double & next_clearance) -> bool {
    bool found = false;
    double best_score = -std::numeric_limits<double>::infinity();
    GridIndex best_index = center;
    double best_clearance = center_clearance;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }

          const GridIndex candidate{center.x + dx, center.y + dy, center.z + dz};
          const Eigen::Vector3d candidate_point = map.indexToWorld(candidate);
          if ((candidate_point - seed).norm() > max_distance) {
            continue;
          }

          double clearance = 0.0;
          if (!isUsableGradientIndex(candidate, clearance)) {
            continue;
          }

          // This is a local discrete ESDF-gradient ascent step, not the old global radius search.
          // The distance term dominates. The tiny shift penalty keeps the projected point near
          // the original click when several neighbors have nearly identical ESDF values.
          const double shift_penalty = 1.0e-4 * (candidate_point - seed).squaredNorm();
          const double score = clearance - shift_penalty;
          if (clearance > center_clearance + min_progress && score > best_score) {
            found = true;
            best_score = score;
            best_index = candidate;
            best_clearance = clearance;
          }
        }
      }
    }

    if (!found) {
      return false;
    }

    next_index = best_index;
    next_clearance = best_clearance;
    return true;
  };

  for (int iter = 0; iter <= max_steps; ++iter) {
    double current_clearance = 0.0;
    if (isSafeIndex(current_index, current_clearance)) {
      safe_point = map.indexToWorld(current_index);
      std::ostringstream oss;
      oss << "gradient-projected " << label << " from [" << seed.x() << ", " << seed.y() << ", "
          << seed.z() << "] to [" << safe_point.x() << ", " << safe_point.y() << ", "
          << safe_point.z() << "], shift=" << (safe_point - seed).norm()
          << " m, clearance=" << current_clearance << " m, required>=" << required_clearance
          << ", gradient_steps=" << gradient_steps << ", inspected_points=" << inspected_points;
      status = oss.str();
      return true;
    }

    double usable_current_clearance = 0.0;
    const bool current_has_finite_esdf =
      isUsableGradientIndex(current_index, usable_current_clearance);
    if (!current_has_finite_esdf) {
      usable_current_clearance = -std::numeric_limits<double>::infinity();
    }

    GridIndex next_index = current_index;
    double next_clearance = usable_current_clearance;
    bool have_next = false;

    const Eigen::Vector3d current_point = map.indexToWorld(current_index);
    const Eigen::Vector3d gradient = map.gradient(current_point);
    if (
      std::isfinite(gradient.x()) && std::isfinite(gradient.y()) && std::isfinite(gradient.z()) &&
      gradient.norm() > 1.0e-6) {
      Eigen::Vector3d proposal = current_point + step_size * gradient.normalized();
      for (int axis = 0; axis < 3; ++axis) {
        proposal(axis) = std::clamp(proposal(axis), lower(axis), upper(axis));
      }

      GridIndex proposal_index;
      if (
        (proposal - seed).norm() <= max_distance && map.worldToIndex(proposal, proposal_index) &&
        !(proposal_index == current_index)) {
        double proposal_clearance = 0.0;
        if (
          isUsableGradientIndex(proposal_index, proposal_clearance) &&
          proposal_clearance > usable_current_clearance + min_progress) {
          next_index = proposal_index;
          next_clearance = proposal_clearance;
          have_next = true;
        }
      }
    }

    if (!have_next) {
      have_next = chooseDiscreteGradientNeighbor(
        current_index, usable_current_clearance, next_index, next_clearance);
    }

    if (!have_next || next_index == current_index) {
      break;
    }

    current_index = next_index;
    ++gradient_steps;

    (void)next_clearance;
  }

  std::ostringstream oss;
  oss << "Cannot gradient-project " << label << " point to a safe corridor point within "
      << options_.safe_point_search_radius
      << " m: required ESDF clearance >= " << required_clearance;
  if (std::isfinite(best_seen_clearance)) {
    oss << ", best reached clearance=" << best_seen_clearance
        << " m at shift=" << (best_seen_point - seed).norm() << " m";
  }
  oss << ". Try selecting a point farther from obstacles, increasing "
      << "selection.safe_point_search_radius, or reducing corridor.safety_margin/min_radius.";
  status = oss.str();
  return false;
}

}  // namespace amprobo

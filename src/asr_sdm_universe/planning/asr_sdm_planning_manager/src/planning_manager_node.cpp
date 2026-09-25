// Copyright (c) Amphibious Robotics.
// ROS 2 planning manager node.

#include <asr_sdm_planning_manager/backward.hpp>
#include <asr_sdm_log_collector/log_client.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <asr_sdm_planning_manager/planner_manager.h>
#include <asr_sdm_planning_manager/topo_replan_fsm.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
namespace backward
{
backward::SignalHandling sh;
}

namespace amprobo
{

namespace
{

Eigen::Vector3d headingBodyX(double yaw, double pitch)
{
  return Eigen::Vector3d(
    std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), -std::sin(pitch));
}

constexpr double MIN_HEADING_SPEED = 0.02;

double wrapToPi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool headingFromVelocity(const Eigen::Vector3d & v, double & yaw, double & pitch)
{
  const double sxy2 = v(0) * v(0) + v(1) * v(1);
  const double v2 = sxy2 + v(2) * v(2);
  const double min_speed2 = MIN_HEADING_SPEED * MIN_HEADING_SPEED;
  if (sxy2 < min_speed2 || v2 < min_speed2) return false;

  yaw = std::atan2(v(1), v(0));
  pitch = std::atan2(-v(2), std::sqrt(sxy2));
  return true;
}

bool headingFromBodyX(const Eigen::Vector3d & dir, double & yaw, double & pitch)
{
  if (dir.squaredNorm() < 1.0e-12) return false;
  yaw = std::atan2(dir.y(), dir.x());
  pitch = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));
  return true;
}

vector<Eigen::Vector3d> downsamplePolyline(const vector<Eigen::Vector3d> & path, double ds)
{
  if (path.size() <= 2 || ds <= 1.0e-6) return path;

  vector<Eigen::Vector3d> out;
  out.reserve(path.size());
  out.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    if ((path[i] - out.back()).norm() >= ds) out.push_back(path[i]);
  }
  if ((path.back() - out.back()).squaredNorm() > 1.0e-12) {
    out.push_back(path.back());
  } else {
    out.back() = path.back();
  }
  return out;
}

}  // namespace

// SECTION interfaces for setup and query

PlanningManager::PlanningManager()
{
}

PlanningManager::~PlanningManager()
{
  SPDLOG_INFO("des manager");
}

void PlanningManager::initPlanModules(const std::shared_ptr<rclcpp::Node> & nh)
{
  node_ = nh;

  /* read algorithm parameters */
  node_->declare_parameter("manager.max_vel", -1.0);
  node_->declare_parameter("manager.max_acc", -1.0);
  node_->declare_parameter("manager.max_jerk", -1.0);
  node_->declare_parameter("manager.dynamic_environment", -1);
  node_->declare_parameter("manager.clearance_threshold", -1.0);
  node_->declare_parameter("manager.local_segment_length", -1.0);
  node_->declare_parameter("manager.control_points_distance", -1.0);
  node_->declare_parameter("manager.nonholonomic", false);
  node_->declare_parameter("manager.max_yaw_rate", 0.35);
  node_->declare_parameter("manager.max_pitch_rate", 0.35);
  node_->declare_parameter("manager.min_vel", 0.0);
  node_->declare_parameter("manager.max_time_lengthen_ratio", 1.5);
  pp_.max_vel_ = node_->get_parameter("manager.max_vel").as_double();
  pp_.max_acc_ = node_->get_parameter("manager.max_acc").as_double();
  pp_.max_jerk_ = node_->get_parameter("manager.max_jerk").as_double();
  pp_.dynamic_ = node_->get_parameter("manager.dynamic_environment").as_int();
  pp_.clearance_ = node_->get_parameter("manager.clearance_threshold").as_double();
  pp_.local_traj_len_ = node_->get_parameter("manager.local_segment_length").as_double();
  pp_.ctrl_pt_dist = node_->get_parameter("manager.control_points_distance").as_double();
  pp_.nonholonomic_ = node_->get_parameter("manager.nonholonomic").as_bool();
  pp_.max_yaw_rate_ = node_->get_parameter("manager.max_yaw_rate").as_double();
  pp_.max_pitch_rate_ = node_->get_parameter("manager.max_pitch_rate").as_double();
  pp_.min_vel_ = node_->get_parameter("manager.min_vel").as_double();
  pp_.max_time_lengthen_ratio_ =
    std::max(1.0, node_->get_parameter("manager.max_time_lengthen_ratio").as_double());

  node_->declare_parameter("manager.use_guidance_planner", false);
  node_->declare_parameter("manager.use_topo_path", false);
  node_->declare_parameter("manager.use_optimization", false);
  bool use_guidance_planner = node_->get_parameter("manager.use_guidance_planner").as_bool();
  bool use_topo_path = node_->get_parameter("manager.use_topo_path").as_bool();
  bool use_optimization = node_->get_parameter("manager.use_optimization").as_bool();

  local_data_.traj_id_ = 0;
  esdf_map_.reset(new ESDFMap);
  esdf_map_->initMap(node_);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(esdf_map_);

  if (use_guidance_planner) {
    guidance_planner_.reset(new GuidancePlanner);
    guidance_planner_->setParam(node_);
    guidance_planner_->setEnvironment(edt_environment_);
    guidance_planner_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(node_);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(node_);
  }
}

void PlanningManager::setGlobalWaypoints(vector<Eigen::Vector3d> & waypoints)
{
  plan_data_.global_waypoints_ = waypoints;
}

void PlanningManager::resetPlan()
{
  plan_data_.clearTopoPaths();
  plan_data_.global_waypoints_.clear();
  plan_data_.path_yaw_.clear();
  plan_data_.path_pitch_.clear();
  global_data_.local_traj_.clear();
  global_data_.global_duration_ = 0.0;
  global_data_.local_start_time_ = -1.0;
  global_data_.local_end_time_ = -1.0;
  local_data_.duration_ = 0.0;
  goal_heading_.setZero();
  start_vel_plan_.setZero();
  start_acc_plan_.setZero();
  start_yaw_.setZero();
  start_pitch_.setZero();
}

void PlanningManager::setGoalHeading(const Eigen::Vector3d & heading)
{
  goal_heading_ = heading.squaredNorm() > 1.0e-12 ? heading.normalized() : Eigen::Vector3d::Zero();
}

void PlanningManager::setStartMotion(
  const Eigen::Vector3d & start_vel, const Eigen::Vector3d & start_acc,
  const Eigen::Vector3d & start_yaw, const Eigen::Vector3d & start_pitch)
{
  start_yaw_ = start_yaw;
  start_pitch_ = start_pitch;

  const Eigen::Vector3d dir = headingBodyX(start_yaw(0), start_pitch(0));
  if (pp_.nonholonomic_ && dir.squaredNorm() > 1.0e-12) {
    const Eigen::Vector3d heading = dir.normalized();
    double speed = start_vel.norm();
    if (speed < pp_.min_vel_) speed = pp_.min_vel_;
    start_vel_plan_ = speed * heading;
    start_acc_plan_ = start_acc.dot(heading) * heading;
    SPDLOG_INFO(
      "nonholonomic start: yaw={:.3f} pitch={:.3f} speed={:.3f}", start_yaw(0), start_pitch(0),
      start_vel_plan_.norm());
  } else {
    start_vel_plan_ = start_vel;
    start_acc_plan_ = start_acc;
  }
}

bool PlanningManager::checkTrajCollision(double & distance)
{
  double t_now = (node_->now() - local_data_.start_time_).seconds();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION topological replanning

bool PlanningManager::planGlobalTraj(const Eigen::Vector3d & start_pos)
{
  // Clear any previous topological search results before building a new global reference.
  plan_data_.clearTopoPaths();

  // Guidance planner (3D Dubins) supplies the global polyline when it is enabled
  // and finds a feasible curve; otherwise densify the configured waypoints.
  // Min-snap then fits that polyline and the first local segment is truncated
  // from it.
  vector<Eigen::Vector3d> points;
  if (!buildGuidanceGlobalWaypoints(start_pos, points)) {
    points = buildGlobalWaypoints(start_pos);
  }
  PolynomialTraj global_traj = fitGlobalMinSnapTraj(points);
  auto time_now = node_->now();
  global_data_.setGlobalTraj(global_traj, time_now);

  initLocalTrajFromGlobal(time_now);
  updateTrajInfo();
  return true;
}

bool PlanningManager::buildGuidanceGlobalWaypoints(
  const Eigen::Vector3d & start_pos, vector<Eigen::Vector3d> & points)
{
  points.clear();
  if (!guidance_planner_) return false;

  const vector<Eigen::Vector3d> & anchors = plan_data_.global_waypoints_;
  if (anchors.empty()) {
    SPDLOG_WARN("no global waypoints!");
    return false;
  }

  double yaw = start_yaw_(0);
  double pitch = start_pitch_(0);
  Eigen::Vector3d curr = start_pos;
  // sample_ds is for collision checking; min-snap needs a coarser polyline so
  // segment times stay around dist / max_vel instead of hitting the 1 s floor.
  const double ds = std::max(1.0, pp_.ctrl_pt_dist);
  int sample_count = 1;
  double path_length = 0.0;
  points.push_back(curr);

  for (size_t i = 0; i < anchors.size(); ++i) {
    const Eigen::Vector3d & goal = anchors[i];
    if ((goal - curr).norm() < 1.0e-3) continue;

    double end_yaw = yaw;
    double end_pitch = pitch;
    const bool last = (i + 1 == anchors.size());
    if (last) {
      if (!headingFromBodyX(goal_heading_, end_yaw, end_pitch)) {
        headingFromBodyX(goal - curr, end_yaw, end_pitch);
      }
    } else if (!headingFromBodyX(anchors[i + 1] - goal, end_yaw, end_pitch)) {
      headingFromBodyX(goal - curr, end_yaw, end_pitch);
    }

    const int status = guidance_planner_->search(curr, yaw, pitch, goal, end_yaw, end_pitch);
    if (status != GuidancePlanner::REACH_END) {
      SPDLOG_WARN(
        "guidance planner failed between ({:.2f},{:.2f},{:.2f}) and ({:.2f},{:.2f},{:.2f})",
        curr.x(), curr.y(), curr.z(), goal.x(), goal.y(), goal.z());
      points.clear();
      return false;
    }

    const vector<Eigen::Vector3d> seg = guidance_planner_->getPath();
    const vector<double> yaw_seg = guidance_planner_->getYawPath();
    const vector<double> pitch_seg = guidance_planner_->getPitchPath();
    if (seg.size() < 2 || yaw_seg.size() != seg.size() || pitch_seg.size() != seg.size()) {
      SPDLOG_WARN("guidance planner returned an empty curve");
      points.clear();
      return false;
    }

    const vector<Eigen::Vector3d> sparse = downsamplePolyline(seg, ds);
    const size_t before = points.size();
    if ((sparse.front() - points.back()).squaredNorm() < 1.0e-12) {
      points.insert(points.end(), sparse.begin() + 1, sparse.end());
    } else {
      points.insert(points.end(), sparse.begin(), sparse.end());
    }
    if (points.size() == before) {
      points.push_back(goal);
    } else {
      points.back() = goal;
    }

    curr = goal;
    yaw = yaw_seg.back();
    pitch = pitch_seg.back();
    sample_count += static_cast<int>(seg.size()) - 1;
    path_length += guidance_planner_->getPathLength();
  }

  if (points.size() == 2) {
    points.insert(points.begin() + 1, 0.5 * (points[0] + points[1]));
  }
  if (points.size() < 3) {
    points.clear();
    return false;
  }

  SPDLOG_INFO(
    "guidance global path: {} samples -> {} waypoints, length {:.2f} m", sample_count,
    static_cast<int>(points.size()), path_length);
  return true;
}

vector<Eigen::Vector3d> PlanningManager::buildGlobalWaypoints(const Eigen::Vector3d & start_pos)
{
  // Start from configured global waypoints and prepend the current start position.
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) SPDLOG_WARN("no global waypoints!");

  points.insert(points.begin(), start_pos);

  if (pp_.nonholonomic_) insertNonholonomicStartArc(points);

  // Insert intermediate points when consecutive waypoints are farther than dist_thresh.
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
          points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  // minSnapTraj needs at least 3 waypoints; insert a midpoint if only two remain.
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  return inter_points;
}

void PlanningManager::insertNonholonomicStartArc(vector<Eigen::Vector3d> & points)
{
  if (points.size() < 2) return;

  const Eigen::Vector3d start = points.front();
  const Eigen::Vector3d goal = points[1];
  Eigen::Vector3d heading = headingBodyX(start_yaw_(0), start_pitch_(0));
  if (heading.squaredNorm() < 1.0e-12) return;
  heading.normalize();

  const Eigen::Vector3d to_goal = goal - start;
  const double dist = to_goal.norm();
  if (dist < 1.0e-3) return;
  const Eigen::Vector3d goal_dir = to_goal / dist;

  const double ang = std::acos(std::max(-1.0, std::min(1.0, heading.dot(goal_dir))));
  if (ang < 0.15) return;

  const double omega = std::max(1.0e-3, std::min(pp_.max_yaw_rate_, pp_.max_pitch_rate_));
  const double radius = std::max(pp_.max_vel_, pp_.min_vel_) / omega;
  const double arc_len = std::min(ang * radius, 0.5 * dist);
  if (arc_len < 0.5) return;
  const double alpha = arc_len / radius;

  Eigen::Vector3d axis = heading.cross(goal_dir);
  if (axis.squaredNorm() < 1.0e-12) {
    axis = heading.cross(Eigen::Vector3d::UnitZ());
    if (axis.squaredNorm() < 1.0e-12) axis = heading.cross(Eigen::Vector3d::UnitY());
  }
  axis.normalize();
  const Eigen::Vector3d radial = axis.cross(heading);

  const int n = std::max(1, static_cast<int>(std::lround(arc_len / 4.0)));
  vector<Eigen::Vector3d> arc;
  arc.reserve(static_cast<size_t>(n));
  for (int k = 1; k <= n; ++k) {
    const double theta = alpha * static_cast<double>(k) / static_cast<double>(n);
    const Eigen::Vector3d pt =
      start + radius * (heading * std::sin(theta) + radial * (1.0 - std::cos(theta)));
    if ((pt - goal).norm() < 0.5) break;
    arc.push_back(pt);
  }
  if (arc.empty()) return;

  points.insert(points.begin() + 1, arc.begin(), arc.end());
  SPDLOG_INFO(
    "nonholonomic start arc: heading error {:.1f} deg, {} seed waypoints, radius {:.2f} m",
    ang * 180.0 / M_PI, static_cast<int>(arc.size()), radius);
}

PolynomialTraj PlanningManager::fitGlobalMinSnapTraj(const vector<Eigen::Vector3d> & points)
{
  // Convert waypoints to a position matrix and allocate segment times by distance / max_vel.
  int pt_num = static_cast<int>(points.size());
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = points[i];

  Eigen::Vector3d zero(0, 0, 0);
  const double max_vel = std::max(pp_.max_vel_, 1.0e-6);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / max_vel;
  }

  // Slow down the first and last segments for smoother start/stop.
  time(0) *= 2.0;
  time(time.rows() - 1) *= 2.0;

  // Floor every segment, not only the first and last. Quintic min-snap
  // coefficients scale as 1/T^k; a near-zero middle duration produces
  // km/s velocities that fold a local radius window into tens of km of
  // arc and OOM the dense B-spline fit.
  for (int i = 0; i < time.rows(); ++i) {
    time(i) = std::max(1.0, time(i));
  }

  const Eigen::Vector3d start_vel = pp_.nonholonomic_ ? start_vel_plan_ : zero;
  const Eigen::Vector3d start_acc = pp_.nonholonomic_ ? start_acc_plan_ : zero;

  // A zero terminal velocity leaves the tangent at the goal with no direction,
  // and the heading is read straight off that tangent, so the robot arrives
  // facing whichever way the fit happened to curl in from. minSnapTraj imposes
  // the terminal velocity as an equality constraint, so any nonzero speed pins
  // the direction exactly; keep it small enough that the robot still stops.
  constexpr double MIN_ARRIVAL_SPEED = 1.0e-2;
  const Eigen::Vector3d end_vel = goal_heading_.squaredNorm() > 1.0e-12
                                    ? std::max(pp_.min_vel_, MIN_ARRIVAL_SPEED) * goal_heading_
                                    : zero;

  return minSnapTraj(pos, start_vel, end_vel, start_acc, zero, time);
}

void PlanningManager::initLocalTrajFromGlobal(const rclcpp::Time & time_now)
{
  // Truncate a local B-spline segment from the global polynomial for immediate execution.
  double dt, duration;
  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  fast_planner::NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_ = time_now;
  SPDLOG_INFO("global trajectory generated.");
}

bool PlanningManager::topoReplan(bool collide)
{
  rclcpp::Time t1, t2;

  /* truncate a new local segment for replanning */
  rclcpp::Time time_now = node_->now();
  double t_now = (time_now - global_data_.global_start_time_).seconds();
  double local_traj_dt, local_traj_duration;
  double time_inc = 0.0;

  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  fast_planner::NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    // refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      SPDLOG_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      fast_planner::NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      SPDLOG_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr> graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(
        colli_start.front(), colli_end.back(), start_pts, end_pts, graph, raw_paths, filtered_paths,
        select_paths);

      if (select_paths.size() == 0) {
        SPDLOG_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      SPDLOG_INFO("[Optimize]: ---------");
      t1 = node_->now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (size_t i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(
          &PlanningManager::optimizeTopoBspline, this, t_now, local_traj_duration, select_paths[i],
          int(i));
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (size_t i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (node_->now() - t1).seconds();
      SPDLOG_INFO("[planner]: optimization time: {}", t_opt);
      const int best_id = selectBestTraj(best_traj);
      SPDLOG_INFO(
        "[planner]: candidate {} of {} selected", best_id, static_cast<int>(select_paths.size()));
      // refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(
        local_data_.position_traj_, t_now, local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

/* Pick the candidate of least jerk, scaled by how far its tangent exceeds the
 * yaw / pitch rate limits. A taut detour can have a smaller jerk and still be
 * untrackable; the ratio keeps those knife-edge corners from winning. The
 * candidates share their index with plan_data_.topo_select_paths_ and are
 * searched in place rather than sorted. */
int PlanningManager::selectBestTraj(fast_planner::NonUniformBspline & traj)
{
  vector<fast_planner::NonUniformBspline> & trajs = plan_data_.topo_traj_pos2_;

  int best = 0;
  double best_score = trajs[0].getJerk() * headingRateRatio(trajs[0]);
  for (size_t i = 1; i < trajs.size(); ++i) {
    const double score = trajs[i].getJerk() * headingRateRatio(trajs[i]);
    if (score < best_score) {
      best_score = score;
      best = static_cast<int>(i);
    }
  }

  traj = trajs[best];
  plan_data_.best_topo_idx_ = best;
  return best;
}

/* On a nonholonomic robot the heading is not an independent degree of freedom,
 * so the yaw and pitch limits are optimized together with the position. */
int PlanningManager::localCostFunction() const
{
  return pp_.nonholonomic_ ? BsplineOptimizer::NONHOLONOMIC_PHASE : BsplineOptimizer::NORMAL_PHASE;
}

int PlanningManager::topoGuideCostFunction() const
{
  return pp_.nonholonomic_ ? BsplineOptimizer::GUIDE_NONHOLONOMIC_PHASE
                           : BsplineOptimizer::GUIDE_PHASE;
}

double PlanningManager::headingRateRatio(fast_planner::NonUniformBspline & pos) const
{
  if (!pp_.nonholonomic_) return 1.0;

  fast_planner::NonUniformBspline vel = pos.getDerivative();
  double tm = 0.0, tmp = 0.0;
  vel.getTimeSpan(tm, tmp);
  const double dt = std::max(1.0e-3, pos.getInterval());

  double last_yaw = 0.0, last_pitch = 0.0;
  bool have_heading = false;
  double ratio = 1.0;

  for (double t = tm; t <= tmp + 1.0e-9; t += dt) {
    const Eigen::VectorXd v = vel.evaluateDeBoor(t);
    if (v.size() < 3) continue;
    double yaw = 0.0, pitch = 0.0;
    if (!headingFromVelocity(Eigen::Vector3d(v.head<3>()), yaw, pitch)) continue;

    if (have_heading) {
      const double yaw_rate = std::fabs(wrapToPi(yaw - last_yaw)) / dt;
      const double pitch_rate = std::fabs(pitch - last_pitch) / dt;
      if (pp_.max_yaw_rate_ > 0.0) ratio = std::max(ratio, yaw_rate / pp_.max_yaw_rate_);
      if (pp_.max_pitch_rate_ > 0.0) ratio = std::max(ratio, pitch_rate / pp_.max_pitch_rate_);
    }

    last_yaw = yaw;
    last_pitch = pitch;
    have_heading = true;
  }

  return ratio;
}

void PlanningManager::refineTraj(fast_planner::NonUniformBspline & best_traj, double & time_inc)
{
  rclcpp::Time t1 = node_->now();
  time_inc = 0.0;
  double dt, t_inc;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  if (pp_.nonholonomic_) ratio = std::max(ratio, headingRateRatio(best_traj));
  SPDLOG_INFO("ratio: {}", ratio);

  Eigen::MatrixXd ctrl_pts;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  /* Refinement reallocates time (vel / acc / heading rate) and trims the
   * clearance; which way to go around an obstacle was already decided by the
   * topological candidate. */
  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, localCostFunction(), 1, 1);
  best_traj = fast_planner::NonUniformBspline(ctrl_pts, 3, dt);
  SPDLOG_WARN(
    "[Refine]: cost {} seconds, time change is: {}", (node_->now() - t1).seconds(), time_inc);
}

void PlanningManager::updateTrajInfo()
{
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void PlanningManager::reparamBspline(
  fast_planner::NonUniformBspline & bspline, double ratio, Eigen::MatrixXd & ctrl_pts, double & dt,
  double & time_inc)
{
  double time_origin = bspline.getTimeSum();
  int seg_num = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  /* checkRatio() / headingRateRatio() report how far the segment exceeds the
   * velocity, acceleration and heading-rate limits, so only lengthening
   * restores feasibility. A ratio below one says the segment is already
   * feasible, and shrinking it there would push the robot past the speed the
   * global trajectory asked for and undo the deliberately slow first and last
   * global segments. The upper bound matters because lengthenTime() leaves the
   * leading and trailing knot spans untouched, so the larger the stretch the
   * less a single-interval refit can represent the result. */
  ratio = std::max(1.0, std::min(pp_.max_time_lengthen_ratio_, ratio));
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  point_set.reserve(static_cast<size_t>(seg_num) + 1);
  for (int i = 0; i <= seg_num; ++i) {
    point_set.push_back(bspline.evaluateDeBoorT(static_cast<double>(i) * dt));
  }
  fast_planner::NonUniformBspline::parameterizeToBspline(
    dt, point_set, plan_data_.local_start_end_derivative_, ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void PlanningManager::optimizeTopoBspline(
  double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id)
{
  rclcpp::Time t1;
  double tm1, tm2, tm3;

  t1 = node_->now();

  // parameterize B-spline according to the length of guide path
  const double guide_len = topo_prm_->pathLength(guide_path);
  int seg_num = guide_len / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // A detour is longer than the blocked window it replaces but inherits that
  // window's duration, so heading rates start far above the hinge saturation
  // knee. Stretch the knot span (capped) so the yaw / pitch terms can still
  // shape the corner instead of sitting at cost ≈ 1 with a vanishing gradient.
  if (pp_.nonholonomic_ && duration > 1.0e-6 && pp_.max_vel_ > 1.0e-6) {
    const double stretch =
      std::min(pp_.max_time_lengthen_ratio_, std::max(1.0, guide_len / (pp_.max_vel_ * duration)));
    dt *= stretch;
  }
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (int(guide_pt.size()) != int(ctrl_pts.rows()) - 6) {
    SPDLOG_WARN("what guide");
  }

  tm1 = (node_->now() - t1).seconds();
  t1 = node_->now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
    ctrl_pts, dt, topoGuideCostFunction(), 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = fast_planner::NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (node_->now() - t1).seconds();
  t1 = node_->now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 =
    bspline_optimizers_[traj_id]->BsplineOptimizeTraj(opt_ctrl_pts1, dt, localCostFunction(), 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = fast_planner::NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (node_->now() - t1).seconds();
  SPDLOG_INFO("optimization {} cost {}, {}, {} seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd PlanningManager::reparamLocalTraj(double start_t, double & dt, double & duration)
{
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(
    start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set, start_end_derivative, dt, duration);

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  fast_planner::NonUniformBspline::parameterizeToBspline(
    dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

/* Unlike the overload above, this one runs on one thread per topological
 * candidate, so it must not publish into plan_data_. It would be writing the
 * boundary derivatives of the very window the caller already recorded through
 * the other overload, since only seg_num differs between the candidates. */
Eigen::MatrixXd PlanningManager::reparamLocalTraj(
  double start_t, double duration, int seg_num, double & dt)
{
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  fast_planner::NonUniformBspline::parameterizeToBspline(
    dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void PlanningManager::findCollisionRange(
  vector<Eigen::Vector3d> & colli_start, vector<Eigen::Vector3d> & colli_end,
  vector<Eigen::Vector3d> & start_pts, vector<Eigen::Vector3d> & end_pts)
{
  bool last_safe = true, safe;
  double t_m, t_mp;
  fast_planner::NonUniformBspline * initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e = t_mp;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {
    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int sn = ceil((t_s - t_m) / dt);
  dt = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

bool PlanningManager::tangentAtTime(double t, double dt, Eigen::Vector3d & dir)
{
  constexpr double MIN_TANGENT = 1.0e-4;

  dir = local_data_.velocity_traj_.evaluateDeBoorT(t);
  if (dir.norm() > MIN_TANGENT) return true;

  // The segment starts and ends at rest, where the velocity carries no
  // direction; fall back to the chord spanning one heading sample.
  auto & pos = local_data_.position_traj_;
  const double duration = pos.getTimeSum();
  const double t0 = max(0.0, min(t, duration - dt));
  dir = pos.evaluateDeBoorT(min(duration, t0 + dt)) - pos.evaluateDeBoorT(t0);

  return dir.norm() > MIN_TANGENT;
}

fast_planner::NonUniformBspline PlanningManager::fitAngleBspline(
  const vector<Eigen::Vector3d> & waypts, const vector<int> & waypt_idx,
  const Eigen::Vector3d & start_state, const Eigen::Vector3d & end_state, double dt,
  int optimizer_id)
{
  const int seg_num = static_cast<int>(waypts.size());

  Eigen::MatrixXd ctrl_pts(seg_num + 3, 1);
  ctrl_pts.setZero();

  // Boundary states become the three leading and trailing control points, which
  // the solver holds fixed.
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt, (1 / 3.0) * dt * dt, 1.0, 0.0, -(1 / 6.0) * dt * dt, 1.0, dt,
    (1 / 3.0) * dt * dt;
  ctrl_pts.block(0, 0, 3, 1) = states2pts * start_state;
  ctrl_pts.block(seg_num, 0, 3, 1) = states2pts * end_state;

  bspline_optimizers_[optimizer_id]->setWaypoints(waypts, waypt_idx);
  const int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  ctrl_pts = bspline_optimizers_[optimizer_id]->BsplineOptimizeTraj(ctrl_pts, dt, cost_func, 1, 1);

  fast_planner::NonUniformBspline traj;
  traj.setUniformBspline(ctrl_pts, 3, dt);
  return traj;
}

/* Yaw and pitch are no longer planned as free degrees of freedom: a
 * nonholonomic robot points where it moves, so both angles are read off the
 * tangent of the position trajectory. How fast that tangent may turn is bounded
 * by the nonholonomic terms of the position cost function, which is what makes
 * the resulting heading trackable. */
void PlanningManager::planHeading(
  const Eigen::Vector3d & start_yaw, const Eigen::Vector3d & start_pitch)
{
  auto t1 = node_->now();

  const double duration = local_data_.position_traj_.getTimeSum();
  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  vector<Eigen::Vector3d> yaw_waypts, pitch_waypts;
  vector<int> waypt_idx;
  double last_yaw = start_yaw(0);
  double last_pitch = start_pitch(0);

  for (int i = 0; i < seg_num; ++i) {
    double yaw = last_yaw;
    double pitch = last_pitch;

    Eigen::Vector3d dir;
    if (tangentAtTime(i * dt_yaw, dt_yaw, dir)) {
      yaw = atan2(dir(1), dir(0));
      calcNextYaw(last_yaw, yaw);
      pitch = atan2(-dir(2), dir.head<2>().norm());
    }

    yaw_waypts.push_back(Eigen::Vector3d(yaw, 0.0, 0.0));
    pitch_waypts.push_back(Eigen::Vector3d(pitch, 0.0, 0.0));
    waypt_idx.push_back(i);

    last_yaw = yaw;
    last_pitch = pitch;
  }

  // The robot comes to rest at the end of the segment, so hold the last heading.
  const Eigen::Vector3d end_yaw(last_yaw, 0.0, 0.0);
  const Eigen::Vector3d end_pitch(last_pitch, 0.0, 0.0);

  local_data_.yaw_traj_ = fitAngleBspline(yaw_waypts, waypt_idx, start_yaw, end_yaw, dt_yaw, 1);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  local_data_.pitch_traj_ =
    fitAngleBspline(pitch_waypts, waypt_idx, start_pitch, end_pitch, dt_yaw, 2);
  local_data_.pitchdot_traj_ = local_data_.pitch_traj_.getDerivative();
  local_data_.pitchdotdot_traj_ = local_data_.pitchdot_traj_.getDerivative();

  plan_data_.path_yaw_.clear();
  plan_data_.path_pitch_.clear();
  for (int i = 0; i < seg_num; ++i) {
    plan_data_.path_yaw_.push_back(yaw_waypts[i](0));
    plan_data_.path_pitch_.push_back(pitch_waypts[i](0));
  }
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  SPDLOG_INFO("plan heading: {}", (node_->now() - t1).seconds());
}

void PlanningManager::calcNextYaw(const double & last_yaw, double & yaw)
{
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace amprobo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("planning_manager_node");
  asr_sdm::log::initialize("asr_sdm_planning_manager");

  {
    amprobo::TopoReplanFSM topo_replan;
    topo_replan.init(nh);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Use a multi-threaded executor so the FSM timers and subscription callbacks
    // can run concurrently across multiple threads.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();
  }

  asr_sdm::log::shutdown();
  rclcpp::shutdown();

  return 0;
}

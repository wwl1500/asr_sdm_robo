// Copyright (c) Amphibious Robotics.
// Shared planning data containers.

#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <asr_sdm_local_path_modifier/topo_prm.h>
#include <asr_sdm_trajectory_generator/mini_snap_traj.h>
#include <bspline/non_uniform_bspline.h>

#include <algorithm>
#include <cmath>
#include <vector>

using std::vector;

namespace amprobo
{

class GlobalTrajData
{
private:
public:
  PolynomialTraj global_traj_;
  vector<fast_planner::NonUniformBspline> local_traj_;

  double global_duration_;
  rclcpp::Time global_start_time_;
  double local_start_time_, local_end_time_;
  double time_increase_;
  double last_time_inc_;

  GlobalTrajData(/* args */) {}

  ~GlobalTrajData() {}

  bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

  void setGlobalTraj(const PolynomialTraj & traj, const rclcpp::Time & time)
  {
    global_traj_ = traj;
    global_traj_.init();
    global_duration_ = global_traj_.getTimeSum();
    global_start_time_ = time;

    local_traj_.clear();
    local_start_time_ = -1;
    local_end_time_ = -1;
    time_increase_ = 0.0;
    last_time_inc_ = 0.0;
  }

  void setLocalTraj(fast_planner::NonUniformBspline traj, double local_ts, double local_te, double time_inc)
  {
    local_traj_.resize(3);
    local_traj_[0] = traj;
    local_traj_[1] = local_traj_[0].getDerivative();
    local_traj_[2] = local_traj_[1].getDerivative();

    local_start_time_ = local_ts;
    local_end_time_ = local_te;
    global_duration_ += time_inc;
    time_increase_ += time_inc;
    last_time_inc_ = time_inc;
  }

  Eigen::Vector3d getPosition(double t)
  {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluate(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  Eigen::Vector3d getVelocity(double t)
  {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateVel(t);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateVel(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  Eigen::Vector3d getAcceleration(double t)
  {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateAcc(t);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateAcc(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  // get Bspline paramterization data of a local trajectory within a sphere
  // start_t: start time of the trajectory
  // dist_pt: distance between the discretized points
  void getTrajByRadius(
    const double & start_t, const double & des_radius, const double & dist_pt,
    vector<Eigen::Vector3d> & point_set, vector<Eigen::Vector3d> & start_end_derivative,
    double & dt, double & seg_duration)
  {
    double seg_length = 0.0;  // length of the truncated segment
    double seg_time = 0.0;    // duration of the truncated segment
    double radius = 0.0;      // distance to the first point of the segment

    double delt = 0.2;
    Eigen::Vector3d first_pt = getPosition(start_t);  // first point of the segment
    Eigen::Vector3d prev_pt = first_pt;               // previous point
    Eigen::Vector3d cur_pt;                           // current point

    // go forward until the traj exceed radius or global time

    while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3) {
      seg_time += delt;
      seg_time = min(seg_time, global_duration_ - start_t);

      cur_pt = getPosition(start_t + seg_time);
      seg_length += (cur_pt - prev_pt).norm();
      prev_pt = cur_pt;
      radius = (cur_pt - first_pt).norm();
    }

    // One sample per dist_pt, but never more than a few times the samples
    // a straight crossing of the window would need. A folded min-snap
    // curve can report tens of km of chord in the first 0.2 s step; that
    // must not become a K² dense matrix in parameterizeToBspline.
    const double sample_span = std::max(dist_pt, 1.0e-6);
    const int max_seg_num =
      std::max(1, static_cast<int>(std::ceil(4.0 * des_radius / sample_span)));
    int seg_num = static_cast<int>(std::floor(seg_length / sample_span));
    if (seg_num > max_seg_num) {
      RCLCPP_WARN(
        rclcpp::get_logger("plan_container"),
        "getTrajByRadius: arc length %.1f m in a %.1f m window, capping samples %d -> %d",
        seg_length, des_radius, seg_num, max_seg_num);
      seg_num = max_seg_num;
    }
    seg_num = std::max(1, seg_num);

    // get outputs

    seg_duration = seg_time;  // duration of the truncated segment
    dt = seg_time / static_cast<double>(seg_num);  // time difference between to points

    for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt) {
      cur_pt = getPosition(start_t + tp);
      point_set.push_back(cur_pt);
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + seg_time));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + seg_time));
  }

  // get Bspline paramterization data of a fixed duration local trajectory
  // start_t: start time of the trajectory
  // duration: time length of the segment
  // seg_num: discretized the segment into *seg_num* parts
  void getTrajByDuration(
    double start_t, double duration, int seg_num, vector<Eigen::Vector3d> & point_set,
    vector<Eigen::Vector3d> & start_end_derivative, double & dt)
  {
    dt = duration / seg_num;
    Eigen::Vector3d cur_pt;
    for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
      cur_pt = getPosition(start_t + tp);
      point_set.push_back(cur_pt);
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + duration));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + duration));
  }
};

struct PlanParameters
{
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double local_traj_len_;                // local replanning trajectory length
  double ctrl_pt_dist;                   // distance between adjacient B-spline
                                         // control points
  double clearance_;
  int dynamic_;
  // Largest factor by which refinement may stretch the duration of a local
  // segment to bring it back inside the velocity, acceleration and heading-rate
  // limits.
  double max_time_lengthen_ratio_;
  /* nonholonomic robot: the body axis follows the trajectory tangent, so the
   * heading limits below are enforced by the position optimization itself */
  bool nonholonomic_;
  double max_yaw_rate_, max_pitch_rate_, min_vel_;
  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;
};

struct LocalTrajData
{
  /* info of generated traj */

  int traj_id_;
  double duration_;
  rclcpp::Time start_time_;
  Eigen::Vector3d start_pos_;
  fast_planner::NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
    yawdotdot_traj_;
  fast_planner::NonUniformBspline pitch_traj_, pitchdot_traj_, pitchdotdot_traj_;
};

class MidPlanData
{
public:
  MidPlanData(/* args */) {}
  ~MidPlanData() {}

  vector<Eigen::Vector3d> global_waypoints_;

  // initial trajectory segment
  fast_planner::NonUniformBspline initial_local_segment_;
  vector<Eigen::Vector3d> local_start_end_derivative_;

  // topological paths
  list<GraphNode::Ptr> topo_graph_;
  vector<vector<Eigen::Vector3d>> topo_paths_;
  vector<vector<Eigen::Vector3d>> topo_filtered_paths_;
  vector<vector<Eigen::Vector3d>> topo_select_paths_;

  // multiple topological trajectories, indexed like topo_select_paths_
  vector<fast_planner::NonUniformBspline> topo_traj_pos1_;
  vector<fast_planner::NonUniformBspline> topo_traj_pos2_;
  vector<fast_planner::NonUniformBspline> refines_;
  // Candidate the refinement was run on, or -1 when no candidate was selected.
  int best_topo_idx_ = -1;

  // visibility constraint
  vector<Eigen::Vector3d> block_pts_;
  Eigen::MatrixXd ctrl_pts_;

  // heading planning
  vector<double> path_yaw_;
  vector<double> path_pitch_;
  double dt_yaw_;
  double dt_yaw_path_;

  void clearTopoPaths()
  {
    topo_traj_pos1_.clear();
    topo_traj_pos2_.clear();
    topo_graph_.clear();
    topo_paths_.clear();
    topo_filtered_paths_.clear();
    topo_select_paths_.clear();
    best_topo_idx_ = -1;
  }

  void addTopoPaths(
    list<GraphNode::Ptr> & graph, vector<vector<Eigen::Vector3d>> & paths,
    vector<vector<Eigen::Vector3d>> & filtered_paths,
    vector<vector<Eigen::Vector3d>> & selected_paths)
  {
    topo_graph_ = graph;
    topo_paths_ = paths;
    topo_filtered_paths_ = filtered_paths;
    topo_select_paths_ = selected_paths;
  }
};

}  // namespace amprobo

#endif
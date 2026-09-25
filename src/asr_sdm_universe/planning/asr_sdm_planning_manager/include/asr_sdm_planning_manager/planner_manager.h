// Copyright (c) Amphibious Robotics.
// High-level planning manager interface.

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <asr_sdm_esdf_map/edt_environment.hpp>
#include <asr_sdm_planning_manager/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>

#include <asr_sdm_guidance_planner/guidance_planner.h>
#include <asr_sdm_local_path_modifier/topo_prm.h>
#include <asr_sdm_trajectory_optimizer/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

namespace amprobo
{

// Planning Manager
// Key algorithms of mapping and planning are called

class PlanningManager
{
  // SECTION stable
public:
  PlanningManager();
  ~PlanningManager();

  /* main planning interface */
  bool planGlobalTraj(const Eigen::Vector3d & start_pos);
  bool topoReplan(bool collide);

  void planHeading(const Eigen::Vector3d & start_yaw, const Eigen::Vector3d & start_pitch);
  void setStartMotion(
    const Eigen::Vector3d & start_vel, const Eigen::Vector3d & start_acc,
    const Eigen::Vector3d & start_yaw, const Eigen::Vector3d & start_pitch);

  void initPlanModules(const std::shared_ptr<rclcpp::Node> & nh);
  void resetPlan();
  void setGlobalWaypoints(vector<Eigen::Vector3d> & waypoints);
  // Body axis the robot should hold on arrival. A zero vector means no heading
  // was requested and the trajectory ends at rest, as it did before.
  void setGoalHeading(const Eigen::Vector3d & heading);

  bool checkTrajCollision(double & distance);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;

private:
  /* ROS node handle (for time and logging) */
  std::shared_ptr<rclcpp::Node> node_;

  /* main planning algorithms & modules */
  ESDFMap::Ptr esdf_map_;

  unique_ptr<GuidancePlanner> guidance_planner_;
  unique_ptr<TopologyPRM> topo_prm_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  Eigen::Vector3d start_yaw_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_pitch_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_vel_plan_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_acc_plan_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal_heading_{Eigen::Vector3d::Zero()};

  void updateTrajInfo();

  /* global trajectory helpers */
  vector<Eigen::Vector3d> buildGlobalWaypoints(const Eigen::Vector3d & start_pos);
  bool buildGuidanceGlobalWaypoints(
    const Eigen::Vector3d & start_pos, vector<Eigen::Vector3d> & points);
  void insertNonholonomicStartArc(vector<Eigen::Vector3d> & points);
  PolynomialTraj fitGlobalMinSnapTraj(const vector<Eigen::Vector3d> & points);
  void initLocalTrajFromGlobal(const rclcpp::Time & time_now);

  // topology guided optimization

  void findCollisionRange(
    vector<Eigen::Vector3d> & colli_start, vector<Eigen::Vector3d> & colli_end,
    vector<Eigen::Vector3d> & start_pts, vector<Eigen::Vector3d> & end_pts);

  void optimizeTopoBspline(
    double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id);
  int localCostFunction() const;
  int topoGuideCostFunction() const;
  // Peak |yaw|/|pitch| rate of the position spline, divided by the matching
  // manager limit. 1.0 means the heading already stays inside the limits.
  double headingRateRatio(fast_planner::NonUniformBspline & pos) const;
  Eigen::MatrixXd reparamLocalTraj(double start_t, double & dt, double & duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double & dt);

  // Returns the index of the chosen candidate in plan_data_.topo_traj_pos2_.
  int selectBestTraj(fast_planner::NonUniformBspline & traj);
  void refineTraj(fast_planner::NonUniformBspline & best_traj, double & time_inc);
  void reparamBspline(
    fast_planner::NonUniformBspline & bspline, double ratio, Eigen::MatrixXd & ctrl_pts, double & dt,
    double & time_inc);

  // heading planning
  void calcNextYaw(const double & last_yaw, double & yaw);
  // Direction the body axis must take at time t, read off the position traj.
  bool tangentAtTime(double t, double dt, Eigen::Vector3d & dir);
  // Fit a 1-D angle B-spline through sampled angles with fixed boundary states.
  fast_planner::NonUniformBspline fitAngleBspline(
    const vector<Eigen::Vector3d> & waypts, const vector<int> & waypt_idx,
    const Eigen::Vector3d & start_state, const Eigen::Vector3d & end_state, double dt,
    int optimizer_id);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<PlanningManager> Ptr;

  // !SECTION
};
}  // namespace amprobo

#endif
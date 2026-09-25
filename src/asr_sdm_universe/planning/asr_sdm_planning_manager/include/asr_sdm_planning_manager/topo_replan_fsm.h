// Copyright (c) Amphibious Robotics.
// Topological replanning finite state machine.

#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <asr_sdm_esdf_map/edt_environment.hpp>
#include <asr_sdm_esdf_map/esdf_map.hpp>
#include <asr_sdm_esdf_map/obj_predictor.hpp>
#include <asr_sdm_planning_manager/msg/bspline.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <asr_sdm_planning_manager/planner_manager.h>
#include <asr_sdm_trajectory_optimizer/bspline_optimizer.h>
#include <asr_sdm_trajectory_visualizer/planning_visualization.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using std::vector;

namespace amprobo
{

class TopoReplanFSM
{
private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };

  /* planning utils */
  PlanningManager::Ptr planning_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  std::string flight_type_;  // MANUAL_TARGET, PRESET_TARGET, REFERENCE_PATH
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;

  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;  // start state
  Eigen::Vector3d start_yaw_, start_pitch_;           // heading state (angle, rate, acc)
  Eigen::Vector3d target_point_, end_vel_;            // target state
  Eigen::Vector3d end_heading_;  // body axis to hold on arrival, zero if none requested
  int current_wp_;

  /* ROS utils */
  std::shared_ptr<rclcpp::Node> node_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_, stop_pub_;
  rclcpp::Publisher<asr_sdm_planning_manager::msg::Bspline>::SharedPtr bspline_pub_;

  /* helper functions */
  bool callSearchAndOptimization();    // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();
  void setHeadingStateFromOdom();
  void setHeadingStateFromTraj(double t_cur);
  // Drop the current target and trajectory. Caller must hold mutex_.
  void resetPlanning(const std::string & pos_call);

  /* ROS functions */
  void execFSMCallback();
  void checkCollisionCallback();
  // Take a new target. arrival_heading is the body axis to hold once there, or
  // zero when the caller has no orientation to offer.
  void acceptTarget(const nav_msgs::msg::Path & path, const Eigen::Vector3d & arrival_heading);
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void goalposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
  TopoReplanFSM(/* args */) {}
  ~TopoReplanFSM() {}

  void init(const std::shared_ptr<rclcpp::Node> & nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace amprobo

#endif
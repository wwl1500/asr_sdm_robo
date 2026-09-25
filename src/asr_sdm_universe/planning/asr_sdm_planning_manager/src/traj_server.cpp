// Copyright (c) Amphibious Robotics.
// Trajectory server node for plan execution.

#include "asr_sdm_planning_manager/msg/bspline.hpp"
#include "bspline/non_uniform_bspline.h"

#include <asr_sdm_log_collector/log_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "asr_sdm_control_msgs/msg/robot_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

std::shared_ptr<rclcpp::Node> g_node;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub, traj_pub;
rclcpp::Publisher<asr_sdm_control_msgs::msg::RobotCommand>::SharedPtr pos_cmd_pub;

nav_msgs::msg::Odometry odom;

asr_sdm_control_msgs::msg::RobotCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = {5.7, 5.7, 6.2};
double vel_gain[3] = {3.4, 3.4, 4.0};

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
// Set by /control/initial_pose so a B-spline that was already in flight is not
// started after the robot has been teleported. Cleared by /planning/stop once
// the planner has dropped that plan.
bool reject_bspline_ = false;
std::vector<fast_planner::NonUniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// heading control
double last_yaw_;
double last_pitch_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(
  vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;

  traj_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub->publish(mk);
}

void drawCmd(
  const Eigen::Vector3d & pos, const Eigen::Vector3d & vec, const int & id,
  const Eigen::Vector4d & color)
{
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = g_node->now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::msg::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub->publish(mk_state);
}

void clearTravelVis()
{
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->now();
  mk.action = visualization_msgs::msg::Marker::DELETEALL;
  cmd_vis_pub->publish(mk);
  traj_pub->publish(mk);
}

void stopExecution()
{
  receive_traj_ = false;
  traj_.clear();
  traj_cmd_.clear();
  traj_real_.clear();
  clearTravelVis();
}

void bsplineCallback(const asr_sdm_planning_manager::msg::Bspline::SharedPtr msg)
{
  if (reject_bspline_) {
    SPDLOG_INFO("[Traj server]: ignoring B-spline published before pose reset");
    return;
  }

  // parse pos traj

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  fast_planner::NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (size_t i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  fast_planner::NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = rclcpp::Time(msg->start_time);
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  // parse pitch traj, planned jointly with the position for a nonholonomic head

  Eigen::MatrixXd pitch_pts(msg->pitch_pts.size(), 1);
  for (size_t i = 0; i < msg->pitch_pts.size(); ++i) {
    pitch_pts(i, 0) = msg->pitch_pts[i];
  }

  fast_planner::NonUniformBspline pitch_traj(pitch_pts, msg->order, msg->pitch_dt);
  traj_.push_back(pitch_traj);
  traj_.push_back(pitch_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void replanCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  /* reset duration */
  const double time_out = 0.01;
  rclcpp::Time time_now = g_node->now();
  double t_stop = (time_now - start_time_).seconds() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  traj_cmd_.clear();
  traj_real_.clear();
}

void stopCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  stopExecution();
  reject_bspline_ = false;
}

void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  (void)msg;
  stopExecution();
  reject_bspline_ = true;
  SPDLOG_INFO("[Traj server]: stopped for 2D Pose Estimate");
}

void odomCallbck(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;

  odom = *msg;

  traj_real_.push_back(
    Eigen::Vector3d(
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void visCallback()
{
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_) return;

  rclcpp::Time time_now = g_node->now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_f = Eigen::Vector3d::Zero();
  double yaw = last_yaw_;
  double yawdot = 0.0;
  double pitch = last_pitch_;
  double pitchdot = 0.0;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    pitch = traj_[5].evaluateDeBoorT(t_cur)[0];
    pitchdot = traj_[6].evaluateDeBoorT(t_cur)[0];

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];
    pitch = traj_[5].evaluateDeBoorT(traj_duration_)[0];
    pitchdot = traj_[6].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    SPDLOG_WARN("[Traj server]: invalid time.");
  }

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = asr_sdm_control_msgs::msg::RobotCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  cmd.pos.position.x = pos(0);
  cmd.pos.position.y = pos(1);
  cmd.pos.position.z = pos(2);
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  cmd.pos.orientation.x = q.x();
  cmd.pos.orientation.y = q.y();
  cmd.pos.orientation.z = q.z();
  cmd.pos.orientation.w = q.w();

  cmd.vel.linear.x = vel(0);
  cmd.vel.linear.y = vel(1);
  cmd.vel.linear.z = vel(2);
  cmd.vel.angular.x = 0.0;
  cmd.vel.angular.y = pitchdot;
  cmd.vel.angular.z = yawdot;

  auto pos_err = pos_f - pos;
  // if (pos_err.norm() > 1e-3) {
  //   cmd.yaw = atan2(pos_err(1), pos_err(0));
  // } else {
  //   cmd.yaw = last_yaw_;
  // }
  // cmd.yaw_dot = 1.0;

  last_yaw_ = yaw;
  last_pitch_ = pitch;

  pos_cmd_pub->publish(cmd);

  // draw cmd

  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  Eigen::Vector3d dir(cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch));
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("traj_server");
  g_node = node;
  asr_sdm::log::initialize("asr_sdm_planning_manager");

  node->declare_parameter("traj_server.initialpose_topic", std::string("/control/initial_pose"));
  const std::string initialpose_topic =
    node->get_parameter("traj_server.initialpose_topic").as_string();

  auto bspline_sub = node->create_subscription<asr_sdm_planning_manager::msg::Bspline>(
    "planning/bspline", 10, bsplineCallback);
  auto replan_sub =
    node->create_subscription<std_msgs::msg::Empty>("planning/replan", 10, replanCallback);
  auto new_sub = node->create_subscription<std_msgs::msg::Empty>("planning/new", 10, newCallback);
  auto stop_sub = node->create_subscription<std_msgs::msg::Empty>("planning/stop", 10, stopCallback);
  auto initialpose_sub =
    node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic, 10, initialPoseCallback);
  auto odom_sub =
    node->create_subscription<nav_msgs::msg::Odometry>("odom", 50, odomCallbck);

  cmd_vis_pub =
    node->create_publisher<visualization_msgs::msg::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node->create_publisher<asr_sdm_control_msgs::msg::RobotCommand>("/position_cmd", 50);
  traj_pub = node->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);

  auto cmd_timer = node->create_wall_timer(std::chrono::duration<double>(0.01), cmdCallback);
  auto vis_timer = node->create_wall_timer(std::chrono::duration<double>(0.25), visCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server.time_forward", -1.0);
  time_forward_ = node->get_parameter("traj_server.time_forward").as_double();
  last_yaw_ = 0.0;
  last_pitch_ = 0.0;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  SPDLOG_WARN("[Traj server]: ready.");

  rclcpp::spin(node);
  asr_sdm::log::shutdown();
  rclcpp::shutdown();

  return 0;
}

// Copyright (c) Amphibious Robotics.
// Topological replanning FSM implementation.

#include <asr_sdm_planning_manager/topo_replan_fsm.h>
#include <asr_sdm_log_collector/log_client.hpp>

#include <chrono>
#include <functional>
#include <mutex>
#include <string>

namespace amprobo
{

void TopoReplanFSM::init(const std::shared_ptr<rclcpp::Node> & nh)
{
  node_ = nh;
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_ = false;
  trigger_ = false;
  collide_ = false;
  start_yaw_.setZero();
  start_pitch_.setZero();
  end_heading_.setZero();

  /*  fsm param  */
  node_->declare_parameter("fsm.flight_type", std::string(""));
  node_->declare_parameter("fsm.thresh_replan", -1.0);
  node_->declare_parameter("fsm.thresh_no_replan", -1.0);
  node_->declare_parameter("fsm.waypoint_num", -1);
  node_->declare_parameter("fsm.act_map", false);
  node_->declare_parameter("fsm.initialpose_topic", std::string("/control/initial_pose"));
  flight_type_ = node_->get_parameter("fsm.flight_type").as_string();
  replan_time_threshold_ = node_->get_parameter("fsm.thresh_replan").as_double();
  replan_distance_threshold_ = node_->get_parameter("fsm.thresh_no_replan").as_double();
  waypoint_num_ = node_->get_parameter("fsm.waypoint_num").as_int();
  act_map_ = node_->get_parameter("fsm.act_map").as_bool();
  const std::string initialpose_topic =
    node_->get_parameter("fsm.initialpose_topic").as_string();

  for (int i = 0; i < waypoint_num_; i++) {
    node_->declare_parameter("fsm.waypoint" + to_string(i) + "_x", -1.0);
    node_->declare_parameter("fsm.waypoint" + to_string(i) + "_y", -1.0);
    node_->declare_parameter("fsm.waypoint" + to_string(i) + "_z", -1.0);
    waypoints_[i][0] = node_->get_parameter("fsm.waypoint" + to_string(i) + "_x").as_double();
    waypoints_[i][1] = node_->get_parameter("fsm.waypoint" + to_string(i) + "_y").as_double();
    waypoints_[i][2] = node_->get_parameter("fsm.waypoint" + to_string(i) + "_z").as_double();
  }

  /* initialize main modules */
  planning_manager_.reset(new PlanningManager);
  planning_manager_->initPlanModules(node_);
  visualization_.reset(new PlanningVisualization(node_));

  /* callback */
  exec_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(0.01), std::bind(&TopoReplanFSM::execFSMCallback, this));
  safety_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(0.05), std::bind(&TopoReplanFSM::checkCollisionCallback, this));

  waypoint_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/waypoint_generator/waypoints", 1,
    std::bind(&TopoReplanFSM::waypointCallback, this, std::placeholders::_1));
  goalpose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1, std::bind(&TopoReplanFSM::goalposeCallback, this, std::placeholders::_1));
  initialpose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initialpose_topic, 10,
    std::bind(&TopoReplanFSM::initialposeCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1, std::bind(&TopoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/planning/replan", 20);
  new_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/planning/new", 20);
  stop_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/planning/stop", 20);
  bspline_pub_ =
    node_->create_publisher<asr_sdm_planning_manager::msg::Bspline>("/planning/bspline", 20);
}

void TopoReplanFSM::resetPlanning(const std::string & pos_call)
{
  have_target_ = false;
  trigger_ = false;
  collide_ = false;
  current_wp_ = 0;
  end_heading_.setZero();
  end_vel_.setZero();
  planning_manager_->resetPlan();
  visualization_->clearAll();
  new_pub_->publish(std_msgs::msg::Empty());
  stop_pub_->publish(std_msgs::msg::Empty());
  changeFSMExecState(have_odom_ ? WAIT_TARGET : INIT, pos_call);
}

void TopoReplanFSM::acceptTarget(
  const nav_msgs::msg::Path & path, const Eigen::Vector3d & arrival_heading)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path.poses[0].pose.position.z < -0.1) return;
  SPDLOG_INFO("Triggered!");

  end_heading_ = arrival_heading;

  vector<Eigen::Vector3d> global_wp;
  if (flight_type_ == "REFERENCE_PATH") {
    for (int i = 0; i < waypoint_num_; ++i) {
      Eigen::Vector3d pt;
      pt(0) = waypoints_[i][0];
      pt(1) = waypoints_[i][1];
      pt(2) = waypoints_[i][2];
      global_wp.push_back(pt);
    }
  } else {
    if (flight_type_ == "MANUAL_TARGET") {
      target_point_(0) = path.poses[0].pose.position.x;
      target_point_(1) = path.poses[0].pose.position.y;
      target_point_(2) = 1.0;
      const bool has_heading = end_heading_.squaredNorm() > 1.0e-12;
      SPDLOG_INFO(
        "manual: {} {} {}, arrival heading {:.1f} deg (requested: {})", target_point_(0),
        target_point_(1), target_point_(2),
        has_heading ? atan2(end_heading_(1), end_heading_(0)) * 180.0 / M_PI : 0.0, has_heading);

    } else if (flight_type_ == "PRESET_TARGET") {
      target_point_(0) = waypoints_[current_wp_][0];
      target_point_(1) = waypoints_[current_wp_][1];
      target_point_(2) = waypoints_[current_wp_][2];

      current_wp_ = (current_wp_ + 1) % waypoint_num_;
      SPDLOG_INFO("preset: {} {} {}", target_point_(0), target_point_(1), target_point_(2));
    }

    global_wp.push_back(target_point_);
    visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  }

  planning_manager_->setGlobalWaypoints(global_wp);
  planning_manager_->setGoalHeading(end_heading_);
  end_vel_.setZero();
  have_target_ = true;
  trigger_ = true;

  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }
}

void TopoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  // A waypoint path is a list of positions; nothing in it asks for a particular
  // heading on arrival, and its poses carry an identity orientation that would
  // otherwise read as a request to face +x.
  acceptTarget(*msg, Eigen::Vector3d::Zero());
}

void TopoReplanFSM::goalposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  nav_msgs::msg::Path path;
  path.header = msg->header;
  path.poses.push_back(*msg);

  // A goal pose, unlike a waypoint path, states which way the operator wants the
  // robot to face once it gets there.
  const auto & q = msg->pose.orientation;
  const Eigen::Quaterniond orient(q.w, q.x, q.y, q.z);
  Eigen::Vector3d heading = Eigen::Vector3d::Zero();
  if (orient.norm() > 1.0e-6) heading = orient.normalized().toRotationMatrix().col(0);

  acceptTarget(path, heading);
}

void TopoReplanFSM::initialposeCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  (void)msg;
  // Wait for any in-flight plan to finish publishing, then drop it. traj_server
  // already stopped on the same /control/initial_pose; /planning/stop lets it
  // accept the next goal's B-spline and reject the one that just finished.
  std::lock_guard<std::mutex> lock(mutex_);
  resetPlanning("POSE");
  SPDLOG_INFO("planning reset from 2D Pose Estimate");
}

void TopoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
  string state_str[6] = {
    "INIT",
    "WAIT_TARGET",
    "GEN_NEW_TRAJ",
    "REPLAN_TRAJ",
    "EXEC_TRAJ",
    "REPLAN_"
    "NEW"};
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  SPDLOG_INFO("[{}]: from {} to {}", pos_call, state_str[pre_s], state_str[int(new_state)]);
}

// Heading convention R = Rz(yaw) * Ry(pitch): a positive pitch points the body
// axis downwards, matching the tangent-derived heading used by the planner.
void TopoReplanFSM::setHeadingStateFromOdom()
{
  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);

  start_yaw_(0) = atan2(rot_x(1), rot_x(0));
  start_yaw_(1) = start_yaw_(2) = 0.0;

  start_pitch_(0) = atan2(-rot_x(2), rot_x.head<2>().norm());
  start_pitch_(1) = start_pitch_(2) = 0.0;
}

void TopoReplanFSM::setHeadingStateFromTraj(double t_cur)
{
  LocalTrajData * info = &planning_manager_->local_data_;

  start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
  start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
  start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

  start_pitch_(0) = info->pitch_traj_.evaluateDeBoorT(t_cur)[0];
  start_pitch_(1) = info->pitchdot_traj_.evaluateDeBoorT(t_cur)[0];
  start_pitch_(2) = info->pitchdotdot_traj_.evaluateDeBoorT(t_cur)[0];
}

void TopoReplanFSM::printFSMExecState()
{
  string state_str[6] = {
    "INIT",
    "WAIT_TARGET",
    "GEN_NEW_TRAJ",
    "REPLAN_TRAJ",
    "EXEC_TRAJ",
    "REPLAN_"
    "NEW"};
  SPDLOG_INFO("state: {}", state_str[int(exec_state_)]);
}

void TopoReplanFSM::execFSMCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) SPDLOG_WARN("no odom.");
    if (!trigger_) SPDLOG_WARN("no trigger_.");
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");

      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      setHeadingStateFromOdom();

      new_pub_->publish(std_msgs::msg::Empty());
      /* topo path finding and optimization */
      bool success = callTopologicalTraj(1);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      GlobalTrajData * global_data = &planning_manager_->global_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - global_data->global_start_time_).seconds();

      if (t_cur > global_data->global_duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else {
        LocalTrajData * info = &planning_manager_->local_data_;
        Eigen::Vector3d start_pos = info->start_pos_;
        t_cur = (time_now - info->start_time_).seconds();

        if (t_cur > replan_time_threshold_) {
          if (!global_data->localTrajReachTarget()) {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          } else {
            Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
            Eigen::Vector3d end_pos = info->position_traj_.evaluateDeBoorT(info->duration_);
            if ((cur_pos - end_pos).norm() > replan_distance_threshold_)
              changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData * info = &planning_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
      setHeadingStateFromTraj(t_cur);

      bool success = callTopologicalTraj(2);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        SPDLOG_WARN("Replan fail, retrying...");
      }

      break;
    }
    case REPLAN_NEW: {
      LocalTrajData * info = &planning_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
      setHeadingStateFromTraj(t_cur);

      /* inform server */
      new_pub_->publish(std_msgs::msg::Empty());

      // bool success = callSearchAndOptimization();
      bool success = callTopologicalTraj(1);
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }
  }
}

void TopoReplanFSM::checkCollisionCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  LocalTrajData * info = &planning_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  if (false) {
    auto edt_env = planning_manager_->edt_environment_;

    double dist =
      planning_manager_->pp_.dynamic_
        ? edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ info->duration_)
        : edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {
            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist =
              planning_manager_->pp_.dynamic_
                ? edt_env->evaluateCoarseEDT(new_pt, /* time to program start */ info->duration_)
                : edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset target_point_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        SPDLOG_INFO("change goal, replan.");
        target_point_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        SPDLOG_INFO("goal near collision, keep retry");
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
    double dist;
    bool safe = planning_manager_->checkTrajCollision(dist);
    if (!safe) {
      if (dist > 0.5) {
        SPDLOG_WARN("current traj {} m to collision", dist);
        collide_ = true;
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      } else {
        SPDLOG_ERROR("current traj {} m to collision, emergency stop!", dist);
        replan_pub_->publish(std_msgs::msg::Empty());
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "SAFETY");
      }
    } else {
      collide_ = false;
    }
  }
}

bool TopoReplanFSM::callSearchAndOptimization()
{
  return false;
}

bool TopoReplanFSM::callTopologicalTraj(int step)
{
  bool plan_success;

  planning_manager_->setStartMotion(start_vel_, start_acc_, start_yaw_, start_pitch_);

  if (step == 1) {
    plan_success = planning_manager_->planGlobalTraj(start_pt_);
  } else {
    plan_success = planning_manager_->topoReplan(collide_);
  }

  if (plan_success) {
    planning_manager_->planHeading(start_yaw_, start_pitch_);

    LocalTrajData * locdat = &planning_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    asr_sdm_planning_manager::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = locdat->start_time_;
    bspline.traj_id = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getInterval();

    Eigen::MatrixXd pitch_pts = locdat->pitch_traj_.getControlPoint();
    for (int i = 0; i < pitch_pts.rows(); ++i) {
      double pitch = pitch_pts(i, 0);
      bspline.pitch_pts.push_back(pitch);
    }
    bspline.pitch_dt = locdat->pitch_traj_.getInterval();

    bspline_pub_->publish(bspline);

    /* visualize new trajectories */
    MidPlanData * plan_data = &planning_manager_->plan_data_;
    visualization_->drawPolynomialTraj(
      planning_manager_->global_data_.global_traj_, 0.05, Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(
      locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
      Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos2_, 0.075);

    if (step == 2 && collide_) {
      visualization_->drawTopoPathsPhase1(plan_data->topo_filtered_paths_, 0.05);
      visualization_->drawTopoPathsPhase2(plan_data->topo_select_paths_, 0.075);
    } else {
      vector<vector<Eigen::Vector3d>> empty_paths;
      visualization_->drawTopoPathsPhase1(empty_paths, 0.05);
      visualization_->drawTopoPathsPhase2(empty_paths, 0.075);
    }

    visualization_->drawHeadingTraj(
      locdat->position_traj_, locdat->yaw_traj_, locdat->pitch_traj_, plan_data->dt_yaw_);

    return true;
  } else {
    return false;
  }
}
// TopoReplanFSM::
}  // namespace amprobo

// Copyright (c) Amphibious Robotics.
// Planning debug visualization helpers.

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <asr_sdm_esdf_map/obj_predictor.hpp>
#include <asr_sdm_local_path_modifier/topo_path_modifier.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <asr_sdm_trajectory_generator/mini_snap_traj.h>
#include <bspline/non_uniform_bspline.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

using std::vector;
namespace amprobo
{
class PlanningVisualization
{
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,
    GRAPH_EDGE = 100,
    RAW_PATH = 200,
    FILTERED_PATH = 300,
    SELECT_PATH = 400
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;     // 0
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_pub_;     // 1
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predict_pub_;  // 2
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    visib_pub_;  // 3, visibility constraints
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    frontier_pub_;                                                         // 4, frontier searching
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_pub_;  // 5, yaw trajectory
  vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> pubs_;  //

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  explicit PlanningVisualization(const std::shared_ptr<rclcpp::Node> & nh);

  // Remove every marker this helper has published, including goals and heading.
  void clearAll();

  // draw basic shapes
  void displaySphereList(
    const vector<Eigen::Vector3d> & list, double resolution, const Eigen::Vector4d & color, int id,
    int pub_id = 0);
  void displayCubeList(
    const vector<Eigen::Vector3d> & list, double resolution, const Eigen::Vector4d & color, int id,
    int pub_id = 0);
  void displayLineList(
    const vector<Eigen::Vector3d> & list1, const vector<Eigen::Vector3d> & list2, double line_width,
    const Eigen::Vector4d & color, int id, int pub_id = 0);

  // draw a piece-wise straight line path
  void drawGeometricPath(
    const vector<Eigen::Vector3d> & path, double resolution, const Eigen::Vector4d & color,
    int id = 0);

  // draw a polynomial trajectory
  void drawPolynomialTraj(
    PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d & color, int id = 0);

  // draw a bspline trajectory
  void drawBspline(
    fast_planner::NonUniformBspline & bspline, double size, const Eigen::Vector4d & color,
    bool show_ctrl_pts = false, double size2 = 0.1,
    const Eigen::Vector4d & color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0, int id2 = 0);

  // draw a set of bspline trajectories generated in different phases
  void drawBsplinesPhase1(vector<fast_planner::NonUniformBspline> & bsplines, double size);
  void drawBsplinesPhase2(vector<fast_planner::NonUniformBspline> & bsplines, double size);

  // // draw topological graph and paths
  // void drawTopoGraph(
  //   list<GraphNode::Ptr> & graph, double point_size, double line_width,
  //   const Eigen::Vector4d & color1, const Eigen::Vector4d & color2, const Eigen::Vector4d &
  //   color3, int id = 0);

  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>> & paths, double line_width);
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>> & paths, double line_width);

  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d & color, int id = 0);
  void drawPrediction(
    ObjPrediction pred, double resolution, const Eigen::Vector4d & color, int id = 0);

  Eigen::Vector4d getColor(double h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // SECTION developing
  void drawYawTraj(fast_planner::NonUniformBspline & pos, fast_planner::NonUniformBspline & yaw, const double & dt);
  void drawHeadingTraj(
    fast_planner::NonUniformBspline & pos, fast_planner::NonUniformBspline & yaw,
    fast_planner::NonUniformBspline & pitch, const double & dt);
  void drawYawPath(fast_planner::NonUniformBspline & pos, const vector<double> & yaw, const double & dt);
};
}  // namespace amprobo
#endif

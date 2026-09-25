// Copyright (c) Amphibious Robotics.
// RViz test node for guidance planner visualization.

#ifndef ASR_SDM_GUIDANCE_PLANNER_NODE_RVIZ_PLANNING_NODE_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_NODE_RVIZ_PLANNING_NODE_HPP_

#include <astar_3d_planner.hpp>
#include <guidance_planner.hpp>
#include <lbfgs_path_optimizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sphere_corridor.hpp>
#include <voxel_esdf_map.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace amprobo
{

class RvizPlanningNode final : public rclcpp::Node
{
public:
  explicit RvizPlanningNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();
  void occupancyCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void esdfCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  bool loadBinaryMap();
  bool loadOccupancyBinary(
    const std::string & path, pcl::PointCloud<pcl::PointXYZ> & cloud, std::string & status) const;
  bool loadEsdfBinary(
    const std::string & path, pcl::PointCloud<pcl::PointXYZI> & cloud, std::string & status) const;
  bool updateBoundsFromBinaryFile(
    const std::string & path, const char * expected_magic, uint32_t expected_map_type,
    uint32_t expected_floats_per_record, Eigen::Vector3d & min_bound,
    Eigen::Vector3d & max_bound) const;
  bool resetMapBoundsFromBinaryData(
    const pcl::PointCloud<pcl::PointXYZ> & occupancy_cloud, const std::string & esdf_path);
  std::string expandUserPath(const std::string & path) const;
  std::string resolvePackageRelativePath(const std::string & path) const;
  std::string joinPath(const std::string & directory, const std::string & filename) const;
  void publishOccupiedMapMarker() const;
  void clickedPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);

  void setStart(const Eigen::Vector3d & start);
  void setGoalAndPlan(const Eigen::Vector3d & goal);
  void runPlanning();
  double corridorRequiredClearance() const;
  bool findNearestSafePlanningPoint(
    const Eigen::Vector3d & seed, const std::string & label, Eigen::Vector3d & safe_point,
    std::string & status) const;

  Eigen::Vector3d pointMsgToEigen(const geometry_msgs::msg::Point & point) const;
  void publishPath(
    const std::vector<Eigen::Vector3d> & path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub) const;
  void publishLineMarker(
    const std::vector<Eigen::Vector3d> & path,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub, int id, double width,
    float r, float g, float b, float a) const;
  void publishStartGoalMarker() const;
  void publishCorridorMarkers(const std::vector<CorridorSphere> & spheres);
  void clearCorridorMarkers();
  void publishStatusText(const std::string & text) const;

  std::string frame_id_ = "world";
  std::string occupancy_map_topic_ = "/esdf_map/occupancy_inflate";
  std::string esdf_map_topic_ = "/esdf_map/esdf_distance";
  std::string map_source_ = "binary";
  std::string binary_map_directory_ = "maps";
  std::string occupancy_binary_filename_ = "occupancy.bin";
  std::string esdf_binary_filename_ = "esdf.bin";
  bool binary_fallback_to_topic_ = true;
  bool binary_auto_bounds_ = true;
  double binary_auto_bounds_padding_ = 1.0;
  bool publish_occupied_map_ = true;
  std::string occupied_map_topic_ = "/esdf_map/occupied_map";
  double occupied_map_alpha_ = 0.85;
  int occupied_map_stride_ = 1;
  double occupied_map_mesh_resolution_ = 0.30;
  double occupied_map_mesh_max_height_gap_ = 0.60;
  double occupied_map_ground_height_ = -1.0;
  double occupied_map_visualization_truncate_height_ = 3.0;
  double occupied_map_publish_period_sec_ = 2.0;
  bool clicked_point_use_msg_z_ = true;
  double default_planning_z_ = 0.5;
  bool use_optimized_only_if_safe_ = true;
  bool project_start_goal_to_safe_ = true;
  double safe_point_search_radius_ = 1.5;
  bool skip_repeated_occupancy_with_same_point_count_ = true;
  bool skip_repeated_esdf_with_same_point_count_ = true;
  std::string waypoints_topic_ = "/planning/waypoints";
  std::string corridor_marker_topic_ = "/planning/safe_corridor";
  double corridor_marker_alpha_ = 0.22;
  int last_corridor_marker_count_ = 0;
  int last_occupancy_point_count_ = -1;
  int last_esdf_point_count_ = -1;

  VoxelEsdfMap map_;
  VoxelMapOptions map_options_;
  Astar3dPlanner astar_planner_;
  SphereCorridorGenerator corridor_generator_;
  LbfgsPathOptimizer optimizer_;
  GuidancePlanner guidance_planner_;

  bool have_start_ = false;
  bool have_goal_ = false;
  bool waiting_for_goal_click_ = false;
  Eigen::Vector3d start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr astar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_map_pub_;
  rclcpp::TimerBase::SharedPtr occupied_map_timer_;
};

}  // namespace amprobo

#endif  // ASR_SDM_GUIDANCE_PLANNER_NODE_RVIZ_PLANNING_NODE_HPP_

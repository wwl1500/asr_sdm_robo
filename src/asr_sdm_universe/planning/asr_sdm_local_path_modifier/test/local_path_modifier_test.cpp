// Copyright (c) Amphibious Robotics.
// Interactive test node for the local path modifier.

#include <Eigen/Core>
#include <asr_sdm_local_path_modifier/topo_path_modifier.hpp>
#include <asr_sdm_log_collector/log_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace amprobo
{

class LocalPathModifierTestNode final : public rclcpp::Node
{
public:
  explicit LocalPathModifierTestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("local_path_modifier_test", options)
  {
    loadParameters();

    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.reliable();
    latched_qos.transient_local();

    rclcpp::QoS click_qos(rclcpp::KeepLast(10));
    click_qos.reliable();

    waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      input_waypoints_topic_, latched_qos,
      std::bind(&LocalPathModifierTestNode::waypointsCallback, this, std::placeholders::_1));

    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      add_virtual_obstacle_topic_, click_qos,
      std::bind(&LocalPathModifierTestNode::virtualObstacleCallback, this, std::placeholders::_1));

    clear_obstacles_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      clear_virtual_obstacles_topic_, click_qos,
      std::bind(
        &LocalPathModifierTestNode::clearVirtualObstaclesCallback, this, std::placeholders::_1));

    virtual_obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      virtual_obstacles_marker_topic_, latched_qos);
    topo_candidate_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      topo_candidate_marker_topic_, latched_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "Local path modifier test started. Subscribed latest guidance waypoints: %s. "
      "Click RViz Publish Point on %s to add a virtual obstacle. Output selected topo candidates: "
      "%s.",
      input_waypoints_topic_.c_str(), add_virtual_obstacle_topic_.c_str(),
      topo_candidate_marker_topic_.c_str());
  }

private:
  void loadParameters()
  {
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("topics.input_waypoints", "/planning/waypoints");
    this->declare_parameter("topics.add_virtual_obstacle", "/planning/add_virtual_obstacle");
    this->declare_parameter("topics.clear_virtual_obstacles", "/planning/clear_virtual_obstacles");
    this->declare_parameter("topics.virtual_obstacles_marker", "/planning/virtual_obstacles");
    this->declare_parameter("topics.topo_candidate_paths_marker", "/planning/topo_candidate_paths");

    this->declare_parameter("obstacle.radius", 0.35);
    this->declare_parameter("obstacle.use_clicked_z", true);
    this->declare_parameter("obstacle.default_z", 0.5);

    this->declare_parameter("topo.collision_clearance", 0.10);
    this->declare_parameter("topo.collision_check_step", 0.05);
    this->declare_parameter("topo.local_window_padding", 1.20);
    this->declare_parameter("topo.detour_margin", 0.35);
    this->declare_parameter("topo.max_connection_length", 8.0);
    this->declare_parameter("topo.waypoint_spacing", 0.20);
    this->declare_parameter("topo.max_sample_num", 160);
    this->declare_parameter("topo.random_seed", 7);
    this->declare_parameter("topo.shortcut_iterations", 4);
    this->declare_parameter("topo.block_extend_segments", 2);
    this->declare_parameter("topo.deterministic_sampling", true);
    this->declare_parameter("topo.max_raw_paths", 32);
    this->declare_parameter("topo.reserve_num", 6);
    this->declare_parameter("topo.ratio_to_short", 1.45);
    this->declare_parameter("topo.topo_equiv_sample_num", 48);
    this->declare_parameter("topo.topo_equiv_triangle_step", 0.25);
    this->declare_parameter("topo.smoothness_weight", 0.20);
    this->declare_parameter("topo.clearance_weight", 0.08);
    this->declare_parameter("topo.use_guard_connector_graph", true);
    this->declare_parameter("topo.allow_dense_graph_fallback", true);
    this->declare_parameter("topo.allow_direct_start_goal_edge", false);
    this->declare_parameter("topo.use_triangle_topo_check", false);

    frame_id_ = this->get_parameter("frame_id").as_string();
    input_waypoints_topic_ = this->get_parameter("topics.input_waypoints").as_string();
    add_virtual_obstacle_topic_ = this->get_parameter("topics.add_virtual_obstacle").as_string();
    clear_virtual_obstacles_topic_ =
      this->get_parameter("topics.clear_virtual_obstacles").as_string();
    virtual_obstacles_marker_topic_ =
      this->get_parameter("topics.virtual_obstacles_marker").as_string();
    topo_candidate_marker_topic_ =
      this->get_parameter("topics.topo_candidate_paths_marker").as_string();

    obstacle_radius_ = this->get_parameter("obstacle.radius").as_double();
    obstacle_use_clicked_z_ = this->get_parameter("obstacle.use_clicked_z").as_bool();
    obstacle_default_z_ = this->get_parameter("obstacle.default_z").as_double();

    TopoModifierOptions topo_options;
    topo_options.obstacle_radius = obstacle_radius_;
    topo_options.collision_clearance = this->get_parameter("topo.collision_clearance").as_double();
    topo_options.collision_check_step =
      this->get_parameter("topo.collision_check_step").as_double();
    topo_options.local_window_padding =
      this->get_parameter("topo.local_window_padding").as_double();
    topo_options.detour_margin = this->get_parameter("topo.detour_margin").as_double();
    topo_options.max_connection_length =
      this->get_parameter("topo.max_connection_length").as_double();
    topo_options.waypoint_spacing = this->get_parameter("topo.waypoint_spacing").as_double();
    topo_options.max_sample_num =
      static_cast<int>(this->get_parameter("topo.max_sample_num").as_int());
    topo_options.random_seed = static_cast<int>(this->get_parameter("topo.random_seed").as_int());
    topo_options.shortcut_iterations =
      static_cast<int>(this->get_parameter("topo.shortcut_iterations").as_int());
    topo_options.block_extend_segments =
      static_cast<int>(this->get_parameter("topo.block_extend_segments").as_int());
    topo_options.deterministic_sampling =
      this->get_parameter("topo.deterministic_sampling").as_bool();
    topo_options.max_raw_paths =
      static_cast<int>(this->get_parameter("topo.max_raw_paths").as_int());
    topo_options.reserve_num = static_cast<int>(this->get_parameter("topo.reserve_num").as_int());
    topo_options.ratio_to_short = this->get_parameter("topo.ratio_to_short").as_double();
    topo_options.topo_equiv_sample_num =
      static_cast<int>(this->get_parameter("topo.topo_equiv_sample_num").as_int());
    topo_options.topo_equiv_triangle_step =
      this->get_parameter("topo.topo_equiv_triangle_step").as_double();
    topo_options.smoothness_weight = this->get_parameter("topo.smoothness_weight").as_double();
    topo_options.clearance_weight = this->get_parameter("topo.clearance_weight").as_double();
    topo_options.use_guard_connector_graph =
      this->get_parameter("topo.use_guard_connector_graph").as_bool();
    topo_options.allow_dense_graph_fallback =
      this->get_parameter("topo.allow_dense_graph_fallback").as_bool();
    topo_options.allow_direct_start_goal_edge =
      this->get_parameter("topo.allow_direct_start_goal_edge").as_bool();
    topo_options.use_triangle_topo_check =
      this->get_parameter("topo.use_triangle_topo_check").as_bool();
    modifier_.setOptions(topo_options);
  }

  void waypointsCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    latest_waypoints_.clear();
    latest_waypoints_.reserve(msg->poses.size());
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    for (const auto & pose : msg->poses) {
      latest_waypoints_.push_back(
        Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    }

    have_waypoints_ = latest_waypoints_.size() >= 2U;
    if (!have_waypoints_) {
      RCLCPP_WARN(
        this->get_logger(), "Received /planning/waypoints, but it has fewer than 2 poses.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received latest guidance waypoints: %zu points. Rechecking virtual obstacles.",
      latest_waypoints_.size());
    recomputeAndPublish();
  }

  void virtualObstacleCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
  {
    VirtualObstacle obstacle;
    obstacle.center = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    if (!obstacle_use_clicked_z_) {
      obstacle.center.z() = obstacle_default_z_;
    }
    obstacle.radius = obstacle_radius_;
    virtual_obstacles_.push_back(obstacle);

    RCLCPP_INFO(
      this->get_logger(), "Added virtual obstacle #%zu at [%.3f, %.3f, %.3f], radius=%.3f.",
      virtual_obstacles_.size(), obstacle.center.x(), obstacle.center.y(), obstacle.center.z(),
      obstacle.radius);

    publishVirtualObstaclesMarker();
    recomputeAndPublish();
  }

  void clearVirtualObstaclesCallback(std_msgs::msg::Empty::ConstSharedPtr)
  {
    virtual_obstacles_.clear();
    RCLCPP_INFO(this->get_logger(), "Cleared all virtual obstacles.");
    publishVirtualObstaclesMarker();
    recomputeAndPublish();
  }

  void recomputeAndPublish()
  {
    if (!have_waypoints_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No guidance waypoints yet. Start guidance planner and select Start/Goal first.");
      publishVirtualObstaclesMarker();
      return;
    }

    TopoModifierResult result = modifier_.modify(latest_waypoints_, virtual_obstacles_);
    if (!result.success) {
      RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "%s", result.message.c_str());
    }

    std::vector<std::vector<Eigen::Vector3d>> candidate_paths = result.candidate_paths;
    if (candidate_paths.empty() && !latest_waypoints_.empty()) {
      candidate_paths.push_back(latest_waypoints_);
    }

    publishVirtualObstaclesMarker();
    publishTopoCandidateMarkers(candidate_paths);
  }

  void publishVirtualObstaclesMarker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = frame_id_;
    marker.ns = "virtual_obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * obstacle_radius_;
    marker.scale.y = 2.0 * obstacle_radius_;
    marker.scale.z = 2.0 * obstacle_radius_;
    marker.color.r = 0.95f;
    marker.color.g = 0.05f;
    marker.color.b = 0.05f;
    marker.color.a = 0.75f;
    marker.points.reserve(virtual_obstacles_.size());

    for (const auto & obstacle : virtual_obstacles_) {
      geometry_msgs::msg::Point p;
      p.x = obstacle.center.x();
      p.y = obstacle.center.y();
      p.z = obstacle.center.z();
      marker.points.push_back(p);
    }

    virtual_obstacle_marker_pub_->publish(marker);
  }

  void publishTopoCandidateMarkers(
    const std::vector<std::vector<Eigen::Vector3d>> & candidate_paths) const
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    for (int id = 0; id < last_candidate_marker_count_; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.ns = "topo_candidate_paths";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(marker);
    }

    int id = 0;
    for (const auto & path : candidate_paths) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.ns = "topo_candidate_paths";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.075;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.95f;
      for (const auto & point : path) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
      }
      array.markers.push_back(marker);
    }

    last_candidate_marker_count_ = id;
    topo_candidate_marker_pub_->publish(array);
  }

  std::string frame_id_ = "world";
  std::string input_waypoints_topic_ = "/planning/waypoints";
  std::string add_virtual_obstacle_topic_ = "/planning/add_virtual_obstacle";
  std::string clear_virtual_obstacles_topic_ = "/planning/clear_virtual_obstacles";
  std::string virtual_obstacles_marker_topic_ = "/planning/virtual_obstacles";
  std::string topo_candidate_marker_topic_ = "/planning/topo_candidate_paths";
  double obstacle_radius_ = 0.35;
  bool obstacle_use_clicked_z_ = true;
  double obstacle_default_z_ = 0.5;
  bool have_waypoints_ = false;
  mutable int last_candidate_marker_count_ = 0;

  TopoPathModifier modifier_;
  std::vector<Eigen::Vector3d> latest_waypoints_;
  std::vector<VirtualObstacle> virtual_obstacles_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_obstacles_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr virtual_obstacle_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topo_candidate_marker_pub_;
};

}  // namespace amprobo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  asr_sdm::log::initialize("local_path_modifier_test");
  rclcpp::spin(std::make_shared<amprobo::LocalPathModifierTestNode>());
  asr_sdm::log::shutdown();
  rclcpp::shutdown();
  return 0;
}

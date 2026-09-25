// Copyright (c) Amphibious Robotics.
// RViz test node implementation for guidance planner.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_planning_node.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace amprobo
{

namespace
{

std_msgs::msg::ColorRGBA heightColor(double value, double alpha)
{
  value = std::max(0.0, std::min(1.0, value));

  std_msgs::msg::ColorRGBA color;
  color.a = static_cast<float>(alpha);
  color.r = static_cast<float>(std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * value - 3.0))));
  color.g = static_cast<float>(std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * value - 2.0))));
  color.b = static_cast<float>(std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * value - 1.0))));
  return color;
}

}  // namespace

RvizPlanningNode::RvizPlanningNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("astar_lbfgs_path_planner", options)
{
  loadParameters();
  map_.reset(map_options_);

  astar_planner_.setOptions(
    Astar3dOptions{
      this->get_parameter("astar.heuristic_weight").as_double(),
      this->get_parameter("astar.extra_clearance").as_double(),
      static_cast<int>(this->get_parameter("astar.max_expansions").as_int()),
      static_cast<int>(this->get_parameter("astar.nearest_free_search_radius").as_int()),
      this->get_parameter("astar.allow_diagonal").as_bool()});

  corridor_generator_.setOptions(
    SphereCorridorOptions{
      this->get_parameter("corridor.enabled").as_bool(),
      static_cast<int>(this->get_parameter("corridor.batch_sample_count").as_int()),
      static_cast<int>(this->get_parameter("corridor.max_spheres").as_int()),
      this->get_parameter("corridor.drone_radius").as_double(),
      this->get_parameter("corridor.safety_margin").as_double(),
      this->get_parameter("corridor.min_radius").as_double(),
      this->get_parameter("corridor.max_radius").as_double(),
      this->get_parameter("corridor.min_overlap_volume").as_double(),
      this->get_parameter("corridor.radius_weight").as_double(),
      this->get_parameter("corridor.overlap_weight").as_double(),
      this->get_parameter("corridor.sample_axis_scale").as_double(),
      this->get_parameter("corridor.sample_lateral_scale").as_double(),
      static_cast<uint32_t>(this->get_parameter("corridor.random_seed").as_int()),
      this->get_parameter("corridor.deterministic_sampling").as_bool()});

  optimizer_.setOptions(
    LbfgsPathOptimizerOptions{
      this->get_parameter("optimizer.enabled").as_bool(),
      static_cast<int>(this->get_parameter("optimizer.max_iterations").as_int()),
      static_cast<int>(this->get_parameter("optimizer.max_control_points").as_int()),
      this->get_parameter("optimizer.epsilon").as_double(),
      this->get_parameter("optimizer.smooth_weight").as_double(),
      this->get_parameter("optimizer.length_weight").as_double(),
      this->get_parameter("optimizer.obstacle_weight").as_double(),
      this->get_parameter("optimizer.guide_weight").as_double(),
      this->get_parameter("optimizer.safe_distance").as_double(),
      this->get_parameter("optimizer.validity_check_step").as_double(),
      this->get_parameter("optimizer.extra_clearance").as_double(),
      this->get_parameter("optimizer.corridor_weight").as_double()});

  GuidancePlannerOptions guidance_options;
  guidance_options.astar = astar_planner_.options();
  guidance_options.corridor = corridor_generator_.options();
  guidance_options.optimizer = optimizer_.options();
  guidance_options.use_optimized_only_if_safe = use_optimized_only_if_safe_;
  guidance_options.project_start_goal_to_safe = project_start_goal_to_safe_;
  guidance_options.safe_point_search_radius = safe_point_search_radius_;
  guidance_planner_.setOptions(guidance_options);

  if (map_source_ != "binary") {
    occupancy_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      occupancy_map_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RvizPlanningNode::occupancyCallback, this, std::placeholders::_1));
    esdf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      esdf_map_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RvizPlanningNode::esdfCallback, this, std::placeholders::_1));
  }

  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    this->get_parameter("topics.clicked_point").as_string(), 10,
    std::bind(&RvizPlanningNode::clickedPointCallback, this, std::placeholders::_1));

  rclcpp::QoS waypoints_qos(rclcpp::KeepLast(1));
  waypoints_qos.reliable();
  waypoints_qos.transient_local();
  waypoints_pub_ = this->create_publisher<nav_msgs::msg::Path>(waypoints_topic_, waypoints_qos);
  astar_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/planning/astar_path_marker", waypoints_qos);
  waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/planning/waypoints_marker", waypoints_qos);
  start_goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/planning/start_goal_marker", waypoints_qos);
  corridor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    corridor_marker_topic_, waypoints_qos);
  rclcpp::QoS occupied_map_qos(1);
  occupied_map_qos.reliable();
  occupied_map_qos.transient_local();
  occupied_map_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>(occupied_map_topic_, occupied_map_qos);

  if (publish_occupied_map_ && occupied_map_publish_period_sec_ > 0.0) {
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(occupied_map_publish_period_sec_));
    occupied_map_timer_ =
      this->create_wall_timer(period, std::bind(&RvizPlanningNode::publishOccupiedMapMarker, this));
  }

  if (map_source_ == "binary") {
    if (!loadBinaryMap() && binary_fallback_to_topic_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Binary map load failed. Falling back to occupancy topic: %s and ESDF topic: %s",
        occupancy_map_topic_.c_str(), esdf_map_topic_.c_str());
      map_source_ = "topic";
      occupancy_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        occupancy_map_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RvizPlanningNode::occupancyCallback, this, std::placeholders::_1));
      esdf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        esdf_map_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RvizPlanningNode::esdfCallback, this, std::placeholders::_1));
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "3D A* + sphere corridor + L-BFGS planner started. Map source: %s. Occupancy topic: %s. ESDF "
    "topic: %s. Published path topic: %s. Corridor marker: %s. Use RViz Publish Point "
    "(/clicked_point) twice: start then goal.",
    map_source_.c_str(), occupancy_map_topic_.c_str(), esdf_map_topic_.c_str(),
    waypoints_topic_.c_str(), corridor_marker_topic_.c_str());
}

void RvizPlanningNode::loadParameters()
{
  this->declare_parameter("frame_id", "world");
  this->declare_parameter("topics.occupancy_map", "/esdf_map/occupancy_inflate");
  this->declare_parameter("topics.esdf_map", "/esdf_map/esdf_distance");
  this->declare_parameter("topics.clicked_point", "/clicked_point");
  this->declare_parameter("topics.waypoints", "/planning/waypoints");

  this->declare_parameter("map.source", "binary");
  this->declare_parameter("binary_map.directory", "maps");
  this->declare_parameter("binary_map.occupancy_filename", "occupancy.bin");
  this->declare_parameter("binary_map.esdf_filename", "esdf.bin");
  this->declare_parameter("binary_map.fallback_to_topic", true);
  this->declare_parameter("binary_map.auto_bounds", true);
  this->declare_parameter("binary_map.auto_bounds_padding", 1.0);

  this->declare_parameter("visualization.publish_occupied_map", true);
  this->declare_parameter("visualization.occupied_map_topic", "/esdf_map/occupied_map");
  this->declare_parameter("visualization.occupied_map_alpha", 0.85);
  this->declare_parameter("visualization.occupied_map_stride", 1);
  this->declare_parameter("visualization.occupied_map_mesh_resolution", 0.30);
  this->declare_parameter("visualization.occupied_map_mesh_max_height_gap", 0.60);
  this->declare_parameter("visualization.ground_height", -1.0);
  this->declare_parameter("visualization.visualization_truncate_height", 3.0);
  this->declare_parameter("visualization.occupied_map_publish_period_sec", 2.0);
  this->declare_parameter("visualization.corridor_marker_topic", "/planning/safe_corridor");
  this->declare_parameter("visualization.corridor_marker_alpha", 0.22);

  this->declare_parameter("map.resolution", 0.15);
  this->declare_parameter("map.origin_x", -10.0);
  this->declare_parameter("map.origin_y", -10.0);
  this->declare_parameter("map.origin_z", -1.0);
  this->declare_parameter("map.size_x", 20.0);
  this->declare_parameter("map.size_y", 20.0);
  this->declare_parameter("map.size_z", 5.0);
  this->declare_parameter("map.unknown_as_occupied", false);
  this->declare_parameter("map.clear_before_integrate", true);
  this->declare_parameter("map.skip_repeated_occupancy_with_same_point_count", true);
  this->declare_parameter("map.skip_repeated_esdf_with_same_point_count", true);

  this->declare_parameter("selection.clicked_point_use_msg_z", true);
  this->declare_parameter("selection.default_planning_z", 0.5);
  this->declare_parameter("selection.use_optimized_only_if_safe", true);
  this->declare_parameter("selection.project_start_goal_to_safe", true);
  this->declare_parameter("selection.safe_point_search_radius", 1.5);

  this->declare_parameter("astar.heuristic_weight", 1.0);
  this->declare_parameter("astar.extra_clearance", 0.0);
  this->declare_parameter("astar.max_expansions", 300000);
  this->declare_parameter("astar.nearest_free_search_radius", 10);
  this->declare_parameter("astar.allow_diagonal", true);

  this->declare_parameter("corridor.enabled", true);
  this->declare_parameter("corridor.batch_sample_count", 80);
  this->declare_parameter("corridor.max_spheres", 80);
  this->declare_parameter("corridor.drone_radius", 0.0);
  this->declare_parameter("corridor.safety_margin", 0.10);
  this->declare_parameter("corridor.min_radius", 0.10);
  this->declare_parameter("corridor.max_radius", 6.0);
  this->declare_parameter("corridor.min_overlap_volume", 1.0e-5);
  this->declare_parameter("corridor.radius_weight", 1.0);
  this->declare_parameter("corridor.overlap_weight", 4.0);
  this->declare_parameter("corridor.sample_axis_scale", 0.3333333333333333);
  this->declare_parameter("corridor.sample_lateral_scale", 2.0);
  this->declare_parameter("corridor.random_seed", 7);
  this->declare_parameter("corridor.deterministic_sampling", true);

  this->declare_parameter("optimizer.enabled", true);
  this->declare_parameter("optimizer.max_iterations", 120);
  this->declare_parameter("optimizer.max_control_points", 80);
  this->declare_parameter("optimizer.epsilon", 1.0e-4);
  this->declare_parameter("optimizer.smooth_weight", 1.0);
  this->declare_parameter("optimizer.length_weight", 0.05);
  this->declare_parameter("optimizer.obstacle_weight", 12.0);
  this->declare_parameter("optimizer.guide_weight", 0.08);
  this->declare_parameter("optimizer.safe_distance", 0.20);
  this->declare_parameter("optimizer.validity_check_step", 0.05);
  this->declare_parameter("optimizer.extra_clearance", 0.0);
  this->declare_parameter("optimizer.corridor_weight", 60.0);

  frame_id_ = this->get_parameter("frame_id").as_string();
  occupancy_map_topic_ = this->get_parameter("topics.occupancy_map").as_string();
  esdf_map_topic_ = this->get_parameter("topics.esdf_map").as_string();
  waypoints_topic_ = this->get_parameter("topics.waypoints").as_string();
  map_source_ = this->get_parameter("map.source").as_string();
  binary_map_directory_ =
    resolvePackageRelativePath(this->get_parameter("binary_map.directory").as_string());
  occupancy_binary_filename_ = this->get_parameter("binary_map.occupancy_filename").as_string();
  esdf_binary_filename_ = this->get_parameter("binary_map.esdf_filename").as_string();
  binary_fallback_to_topic_ = this->get_parameter("binary_map.fallback_to_topic").as_bool();
  binary_auto_bounds_ = this->get_parameter("binary_map.auto_bounds").as_bool();
  binary_auto_bounds_padding_ =
    std::max(0.0, this->get_parameter("binary_map.auto_bounds_padding").as_double());
  publish_occupied_map_ = this->get_parameter("visualization.publish_occupied_map").as_bool();
  occupied_map_topic_ = this->get_parameter("visualization.occupied_map_topic").as_string();
  occupied_map_alpha_ =
    std::clamp(this->get_parameter("visualization.occupied_map_alpha").as_double(), 0.0, 1.0);
  occupied_map_stride_ = std::max(
    1, static_cast<int>(this->get_parameter("visualization.occupied_map_stride").as_int()));
  occupied_map_mesh_resolution_ =
    this->get_parameter("visualization.occupied_map_mesh_resolution").as_double();
  occupied_map_mesh_max_height_gap_ =
    this->get_parameter("visualization.occupied_map_mesh_max_height_gap").as_double();
  occupied_map_ground_height_ = this->get_parameter("visualization.ground_height").as_double();
  occupied_map_visualization_truncate_height_ =
    this->get_parameter("visualization.visualization_truncate_height").as_double();
  occupied_map_publish_period_sec_ =
    this->get_parameter("visualization.occupied_map_publish_period_sec").as_double();
  corridor_marker_topic_ = this->get_parameter("visualization.corridor_marker_topic").as_string();
  corridor_marker_alpha_ =
    std::clamp(this->get_parameter("visualization.corridor_marker_alpha").as_double(), 0.01, 1.0);
  clicked_point_use_msg_z_ = this->get_parameter("selection.clicked_point_use_msg_z").as_bool();
  default_planning_z_ = this->get_parameter("selection.default_planning_z").as_double();
  use_optimized_only_if_safe_ =
    this->get_parameter("selection.use_optimized_only_if_safe").as_bool();
  project_start_goal_to_safe_ =
    this->get_parameter("selection.project_start_goal_to_safe").as_bool();
  safe_point_search_radius_ =
    std::max(0.0, this->get_parameter("selection.safe_point_search_radius").as_double());
  skip_repeated_occupancy_with_same_point_count_ =
    this->get_parameter("map.skip_repeated_occupancy_with_same_point_count").as_bool();
  skip_repeated_esdf_with_same_point_count_ =
    this->get_parameter("map.skip_repeated_esdf_with_same_point_count").as_bool();

  map_options_.resolution = this->get_parameter("map.resolution").as_double();
  map_options_.origin = Eigen::Vector3d(
    this->get_parameter("map.origin_x").as_double(),
    this->get_parameter("map.origin_y").as_double(),
    this->get_parameter("map.origin_z").as_double());
  map_options_.size = Eigen::Vector3d(
    this->get_parameter("map.size_x").as_double(), this->get_parameter("map.size_y").as_double(),
    this->get_parameter("map.size_z").as_double());
  map_options_.unknown_as_occupied = this->get_parameter("map.unknown_as_occupied").as_bool();
  map_options_.clear_before_integrate = this->get_parameter("map.clear_before_integrate").as_bool();
  safe_point_search_radius_ = std::max(map_options_.resolution, safe_point_search_radius_);
  occupied_map_mesh_resolution_ = std::max(map_options_.resolution, occupied_map_mesh_resolution_);
  occupied_map_mesh_max_height_gap_ =
    std::max(map_options_.resolution, occupied_map_mesh_max_height_gap_);
  occupied_map_visualization_truncate_height_ =
    std::max(occupied_map_ground_height_ + 1.0e-3, occupied_map_visualization_truncate_height_);
}

void RvizPlanningNode::occupancyCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  if (
    skip_repeated_occupancy_with_same_point_count_ && map_.hasOccupancy() &&
    static_cast<int>(cloud.points.size()) == last_occupancy_point_count_) {
    return;
  }

  last_occupancy_point_count_ = static_cast<int>(cloud.points.size());
  map_.integrateOccupiedCloud(cloud);
  publishOccupiedMapMarker();

  RCLCPP_INFO(
    this->get_logger(),
    "Occupancy updated: input_points=%d, occupied_voxels=%d, voxels=%d, esdf=%s, planner_ready=%s",
    map_.inputPointCount(), map_.occupiedCount(), map_.voxelCount(),
    map_.hasEsdf() ? "ready" : "not_ready", map_.isReady() ? "true" : "false");
}

void RvizPlanningNode::esdfCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);

  if (
    skip_repeated_esdf_with_same_point_count_ && map_.hasEsdf() &&
    static_cast<int>(cloud.points.size()) == last_esdf_point_count_) {
    return;
  }

  last_esdf_point_count_ = static_cast<int>(cloud.points.size());
  map_.integrateEsdfCloud(cloud);

  RCLCPP_INFO(
    this->get_logger(),
    "External ESDF updated: input_points=%d, esdf_voxels=%d, occupancy=%s, planner_ready=%s",
    map_.esdfInputPointCount(), map_.esdfVoxelCount(), map_.hasOccupancy() ? "ready" : "not_ready",
    map_.isReady() ? "true" : "false");
}

bool RvizPlanningNode::loadBinaryMap()
{
  const std::string occupancy_path = joinPath(binary_map_directory_, occupancy_binary_filename_);
  const std::string esdf_path = joinPath(binary_map_directory_, esdf_binary_filename_);

  pcl::PointCloud<pcl::PointXYZ> occupancy_cloud;
  std::string occupancy_status;
  if (!loadOccupancyBinary(occupancy_path, occupancy_cloud, occupancy_status)) {
    RCLCPP_ERROR(this->get_logger(), "%s", occupancy_status.c_str());
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI> esdf_cloud;
  std::string esdf_status;
  if (!loadEsdfBinary(esdf_path, esdf_cloud, esdf_status)) {
    RCLCPP_ERROR(this->get_logger(), "%s", esdf_status.c_str());
    return false;
  }

  if (binary_auto_bounds_) {
    resetMapBoundsFromBinaryData(occupancy_cloud, esdf_path);
  }

  map_.integrateOccupiedCloud(occupancy_cloud);
  map_.integrateEsdfCloud(esdf_cloud);
  last_occupancy_point_count_ = static_cast<int>(occupancy_cloud.points.size());
  last_esdf_point_count_ = static_cast<int>(esdf_cloud.points.size());

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded binary map: occupancy=%s, esdf=%s, occupied_voxels=%d, esdf_voxels=%d, voxels=%d, "
    "planner_ready=%s, frame_id=%s",
    occupancy_path.c_str(), esdf_path.c_str(), map_.occupiedCount(), map_.esdfVoxelCount(),
    map_.voxelCount(), map_.isReady() ? "true" : "false", frame_id_.c_str());

  if (map_.occupiedCount() == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Binary occupancy map was opened but no points fell inside the configured map bounds. "
      "Check map.origin_* / map.size_* / map.resolution in the YAML file.");
  }

  if (!map_.hasEsdf()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Binary ESDF map was opened but no finite distance points fell inside the configured map "
      "bounds. "
      "Check esdf.bin and map origin/size/resolution.");
  }

  publishOccupiedMapMarker();
  return map_.isReady();
}

namespace
{
struct BinaryCloudHeader
{
  char magic[16];
  uint32_t version{0};
  uint32_t map_type{0};
  int64_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
  uint32_t floats_per_record{0};
  uint64_t point_count{0};
  uint32_t frame_id_length{0};
};

bool magicMatches(const char magic[16], const char * expected)
{
  char expected_magic[16] = {};
  const auto copy_len = std::min(std::strlen(expected), sizeof(expected_magic));
  std::memcpy(expected_magic, expected, copy_len);
  return std::memcmp(magic, expected_magic, sizeof(expected_magic)) == 0;
}
}  // namespace

bool RvizPlanningNode::loadOccupancyBinary(
  const std::string & path, pcl::PointCloud<pcl::PointXYZ> & cloud, std::string & status) const
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    status = "Failed to open occupancy binary file: " + path;
    return false;
  }

  BinaryCloudHeader header;
  ifs.read(header.magic, sizeof(header.magic));
  ifs.read(reinterpret_cast<char *>(&header.version), sizeof(header.version));
  ifs.read(reinterpret_cast<char *>(&header.map_type), sizeof(header.map_type));
  ifs.read(reinterpret_cast<char *>(&header.stamp_sec), sizeof(header.stamp_sec));
  ifs.read(reinterpret_cast<char *>(&header.stamp_nanosec), sizeof(header.stamp_nanosec));
  ifs.read(reinterpret_cast<char *>(&header.floats_per_record), sizeof(header.floats_per_record));
  ifs.read(reinterpret_cast<char *>(&header.point_count), sizeof(header.point_count));
  ifs.read(reinterpret_cast<char *>(&header.frame_id_length), sizeof(header.frame_id_length));

  if (!ifs.good()) {
    status = "Invalid occupancy binary header: " + path;
    return false;
  }

  if (
    !magicMatches(header.magic, "ASR_OCC_BIN_V1") || header.version != 1U ||
    header.map_type != 1U || header.floats_per_record != 3U) {
    status = "Unsupported occupancy binary format: " + path;
    return false;
  }

  if (header.frame_id_length > 4096U) {
    status = "Invalid frame_id length in occupancy binary: " + path;
    return false;
  }

  std::string file_frame(header.frame_id_length, '\0');
  if (header.frame_id_length > 0U) {
    ifs.read(file_frame.data(), static_cast<std::streamsize>(header.frame_id_length));
  }

  constexpr uint64_t kMaxReasonablePoints = 20000000ULL;
  if (header.point_count > kMaxReasonablePoints) {
    status = "Too many points in occupancy binary: " + std::to_string(header.point_count);
    return false;
  }

  cloud.clear();
  cloud.reserve(static_cast<size_t>(header.point_count));
  for (uint64_t i = 0; i < header.point_count; ++i) {
    float xyz[3];
    ifs.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
    if (!ifs.good()) {
      status = "Unexpected EOF while reading occupancy binary: " + path;
      return false;
    }
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
      continue;
    }
    cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
  }

  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  status =
    "Loaded occupancy binary frame=" + file_frame + ", points=" + std::to_string(cloud.size());
  return !cloud.empty();
}

bool RvizPlanningNode::loadEsdfBinary(
  const std::string & path, pcl::PointCloud<pcl::PointXYZI> & cloud, std::string & status) const
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    status = "Failed to open ESDF binary file: " + path;
    return false;
  }

  BinaryCloudHeader header;
  ifs.read(header.magic, sizeof(header.magic));
  ifs.read(reinterpret_cast<char *>(&header.version), sizeof(header.version));
  ifs.read(reinterpret_cast<char *>(&header.map_type), sizeof(header.map_type));
  ifs.read(reinterpret_cast<char *>(&header.stamp_sec), sizeof(header.stamp_sec));
  ifs.read(reinterpret_cast<char *>(&header.stamp_nanosec), sizeof(header.stamp_nanosec));
  ifs.read(reinterpret_cast<char *>(&header.floats_per_record), sizeof(header.floats_per_record));
  ifs.read(reinterpret_cast<char *>(&header.point_count), sizeof(header.point_count));
  ifs.read(reinterpret_cast<char *>(&header.frame_id_length), sizeof(header.frame_id_length));

  if (!ifs.good()) {
    status = "Invalid ESDF binary header: " + path;
    return false;
  }

  if (
    !magicMatches(header.magic, "ASR_ESDF_BIN_V1") || header.version != 1U ||
    header.map_type != 2U || header.floats_per_record != 4U) {
    status = "Unsupported ESDF binary format: " + path;
    return false;
  }

  if (header.frame_id_length > 4096U) {
    status = "Invalid frame_id length in ESDF binary: " + path;
    return false;
  }

  std::string file_frame(header.frame_id_length, '\0');
  if (header.frame_id_length > 0U) {
    ifs.read(file_frame.data(), static_cast<std::streamsize>(header.frame_id_length));
  }

  constexpr uint64_t kMaxReasonablePoints = 200000000ULL;
  if (header.point_count > kMaxReasonablePoints) {
    status = "Too many points in ESDF binary: " + std::to_string(header.point_count);
    return false;
  }

  cloud.clear();
  cloud.reserve(static_cast<size_t>(header.point_count));
  for (uint64_t i = 0; i < header.point_count; ++i) {
    float xyzi[4];
    ifs.read(reinterpret_cast<char *>(xyzi), sizeof(xyzi));
    if (!ifs.good()) {
      status = "Unexpected EOF while reading ESDF binary: " + path;
      return false;
    }
    if (
      !std::isfinite(xyzi[0]) || !std::isfinite(xyzi[1]) || !std::isfinite(xyzi[2]) ||
      !std::isfinite(xyzi[3])) {
      continue;
    }
    pcl::PointXYZI point;
    point.x = xyzi[0];
    point.y = xyzi[1];
    point.z = xyzi[2];
    point.intensity = xyzi[3];
    cloud.push_back(point);
  }

  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  status = "Loaded ESDF binary frame=" + file_frame + ", points=" + std::to_string(cloud.size());
  return !cloud.empty();
}

bool RvizPlanningNode::updateBoundsFromBinaryFile(
  const std::string & path, const char * expected_magic, const uint32_t expected_map_type,
  const uint32_t expected_floats_per_record, Eigen::Vector3d & min_bound,
  Eigen::Vector3d & max_bound) const
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    return false;
  }

  BinaryCloudHeader header;
  ifs.read(header.magic, sizeof(header.magic));
  ifs.read(reinterpret_cast<char *>(&header.version), sizeof(header.version));
  ifs.read(reinterpret_cast<char *>(&header.map_type), sizeof(header.map_type));
  ifs.read(reinterpret_cast<char *>(&header.stamp_sec), sizeof(header.stamp_sec));
  ifs.read(reinterpret_cast<char *>(&header.stamp_nanosec), sizeof(header.stamp_nanosec));
  ifs.read(reinterpret_cast<char *>(&header.floats_per_record), sizeof(header.floats_per_record));
  ifs.read(reinterpret_cast<char *>(&header.point_count), sizeof(header.point_count));
  ifs.read(reinterpret_cast<char *>(&header.frame_id_length), sizeof(header.frame_id_length));

  if (
    !ifs.good() || !magicMatches(header.magic, expected_magic) || header.version != 1U ||
    header.map_type != expected_map_type ||
    header.floats_per_record != expected_floats_per_record || header.frame_id_length > 4096U) {
    return false;
  }

  ifs.ignore(static_cast<std::streamsize>(header.frame_id_length));
  if (!ifs.good()) {
    return false;
  }

  bool updated = false;
  std::vector<float> record(header.floats_per_record, 0.0F);
  for (uint64_t i = 0; i < header.point_count; ++i) {
    ifs.read(
      reinterpret_cast<char *>(record.data()),
      static_cast<std::streamsize>(record.size() * sizeof(float)));
    if (!ifs.good()) {
      break;
    }

    const double x = static_cast<double>(record[0]);
    const double y = static_cast<double>(record[1]);
    const double z = static_cast<double>(record[2]);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const Eigen::Vector3d p(x, y, z);
    min_bound = min_bound.cwiseMin(p);
    max_bound = max_bound.cwiseMax(p);
    updated = true;
  }

  return updated;
}

bool RvizPlanningNode::resetMapBoundsFromBinaryData(
  const pcl::PointCloud<pcl::PointXYZ> & occupancy_cloud, const std::string & esdf_path)
{
  Eigen::Vector3d min_bound = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_bound = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  bool has_bound = false;

  for (const auto & point : occupancy_cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3d p(point.x, point.y, point.z);
    min_bound = min_bound.cwiseMin(p);
    max_bound = max_bound.cwiseMax(p);
    has_bound = true;
  }

  if (updateBoundsFromBinaryFile(esdf_path, "ASR_ESDF_BIN_V1", 2U, 4U, min_bound, max_bound)) {
    has_bound = true;
  }

  if (!has_bound) {
    RCLCPP_WARN(
      this->get_logger(),
      "binary_map.auto_bounds is enabled, but no finite point bounds were found. Using configured "
      "map bounds.");
    return false;
  }

  const double resolution = std::max(1.0e-3, map_options_.resolution);
  const double padding = std::max(binary_auto_bounds_padding_, resolution);
  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  for (int axis = 0; axis < 3; ++axis) {
    const double min_padded = min_bound[axis] - padding;
    const double max_padded = max_bound[axis] + padding;
    origin[axis] = std::floor(min_padded / resolution) * resolution;
    const double max_aligned = std::ceil(max_padded / resolution) * resolution;
    size[axis] = std::max(resolution, max_aligned - origin[axis]);
  }

  map_options_.origin = origin;
  map_options_.size = size;
  map_.reset(map_options_);

  RCLCPP_INFO(
    this->get_logger(),
    "Auto map bounds from binary data: origin=[%.3f, %.3f, %.3f], size=[%.3f, %.3f, %.3f], "
    "resolution=%.3f",
    map_options_.origin.x(), map_options_.origin.y(), map_options_.origin.z(),
    map_options_.size.x(), map_options_.size.y(), map_options_.size.z(), map_options_.resolution);
  return true;
}

std::string RvizPlanningNode::expandUserPath(const std::string & path) const
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty()) {
    return path;
  }

  if (path.size() == 1) {
    return std::string(home);
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::string RvizPlanningNode::resolvePackageRelativePath(const std::string & path) const
{
  const std::string expanded = expandUserPath(path);
  if (expanded.empty()) {
    return expanded;
  }

  const std::filesystem::path fs_path(expanded);
  if (fs_path.is_absolute()) {
    return fs_path.string();
  }

  try {
    return (std::filesystem::path(
              ament_index_cpp::get_package_share_directory("asr_sdm_guidance_planner_dev")) /
            fs_path)
      .string();
  } catch (const std::exception &) {
    return fs_path.string();
  }
}

std::string RvizPlanningNode::joinPath(
  const std::string & directory, const std::string & filename) const
{
  return (std::filesystem::path(directory) / filename).string();
}

void RvizPlanningNode::publishOccupiedMapMarker() const
{
  if (!publish_occupied_map_ || !occupied_map_pub_ || !map_.hasOccupancy()) {
    return;
  }

  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = this->now();
  mk.ns = "occupied_map";
  mk.id = 0;
  mk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 1.0;
  mk.scale.y = 1.0;
  mk.scale.z = 1.0;

  // Keep the same mesh Marker conventions as asr_sdm_esdf_map:/esdf_map/occupied_map.
  // RViz still needs marker.color.a to be nonzero even when per-vertex colors are used.
  mk.color.r = 1.0F;
  mk.color.g = 1.0F;
  mk.color.b = 1.0F;
  mk.color.a = static_cast<float>(std::max(0.05, occupied_map_alpha_));

  const int mesh_step = std::max(
    occupied_map_stride_,
    static_cast<int>(std::ceil(occupied_map_mesh_resolution_ / map_.resolution())));
  const double mesh_resolution = mesh_step * map_.resolution();
  const double half = 0.5 * mesh_resolution;
  const double z_min = occupied_map_ground_height_;
  const double z_max =
    std::max(occupied_map_ground_height_ + 1.0e-3, occupied_map_visualization_truncate_height_);

  auto block_key = [](int x, int y, int z) -> long long {
    return (static_cast<long long>(x) << 42) ^ (static_cast<long long>(y) << 21) ^
           static_cast<long long>(z);
  };

  auto add_vertex = [&](const Eigen::Vector3d & v, const std_msgs::msg::ColorRGBA & color) {
    geometry_msgs::msg::Point point;
    point.x = v.x();
    point.y = v.y();
    point.z = v.z();
    mk.points.push_back(point);
    mk.colors.push_back(color);
  };

  auto add_tri = [&](
                   const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
                   const std_msgs::msg::ColorRGBA & color) {
    add_vertex(a, color);
    add_vertex(b, color);
    add_vertex(c, color);
  };

  auto add_quad = [&](
                    const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
                    const Eigen::Vector3d & d, const std_msgs::msg::ColorRGBA & color) {
    add_tri(a, b, c, color);
    add_tri(a, c, d, color);
  };

  std::unordered_map<long long, Eigen::Vector3i> blocks;
  blocks.reserve(static_cast<size_t>(std::max(4096, map_.occupiedCount())));

  // This mirrors SDFMap::publishOccupiedMap(): build coarse occupied blocks from
  // inflated occupied voxel centers, crop by visualization_truncate_height, then
  // publish only exposed block faces as TRIANGLE_LIST.
  const Eigen::Vector3i dims = map_.dims();
  for (int x = 0; x < dims.x(); ++x) {
    for (int y = 0; y < dims.y(); ++y) {
      for (int z = 0; z < dims.z(); ++z) {
        const GridIndex index{x, y, z};
        if (!map_.isOccupied(index)) {
          continue;
        }

        const Eigen::Vector3d pos = map_.indexToWorld(index);
        if (pos.z() > occupied_map_visualization_truncate_height_) {
          continue;
        }

        const int gx = x / mesh_step;
        const int gy = y / mesh_step;
        const int gz = z / mesh_step;
        const long long key = block_key(gx, gy, gz);
        if (blocks.find(key) == blocks.end()) {
          blocks.emplace(key, Eigen::Vector3i(gx, gy, gz));
        }
      }
    }
  }

  auto has_block = [&](int gx, int gy, int gz) -> bool {
    return blocks.find(block_key(gx, gy, gz)) != blocks.end();
  };

  for (const auto & item : blocks) {
    const Eigen::Vector3i & id = item.second;
    const int gx = id(0);
    const int gy = id(1);
    const int gz = id(2);

    const Eigen::Vector3d center(
      map_.origin().x() + (gx + 0.5) * mesh_resolution,
      map_.origin().y() + (gy + 0.5) * mesh_resolution,
      map_.origin().z() + (gz + 0.5) * mesh_resolution);

    const double height_ratio = (center.z() - z_min) / (z_max - z_min);
    const auto color = heightColor(height_ratio, occupied_map_alpha_);

    const Eigen::Vector3d v000(center.x() - half, center.y() - half, center.z() - half);
    const Eigen::Vector3d v100(center.x() + half, center.y() - half, center.z() - half);
    const Eigen::Vector3d v010(center.x() - half, center.y() + half, center.z() - half);
    const Eigen::Vector3d v110(center.x() + half, center.y() + half, center.z() - half);
    const Eigen::Vector3d v001(center.x() - half, center.y() - half, center.z() + half);
    const Eigen::Vector3d v101(center.x() + half, center.y() - half, center.z() + half);
    const Eigen::Vector3d v011(center.x() - half, center.y() + half, center.z() + half);
    const Eigen::Vector3d v111(center.x() + half, center.y() + half, center.z() + half);

    if (!has_block(gx - 1, gy, gz)) add_quad(v000, v001, v011, v010, color);  // -X
    if (!has_block(gx + 1, gy, gz)) add_quad(v100, v110, v111, v101, color);  // +X
    if (!has_block(gx, gy - 1, gz)) add_quad(v000, v100, v101, v001, color);  // -Y
    if (!has_block(gx, gy + 1, gz)) add_quad(v010, v011, v111, v110, color);  // +Y
    if (!has_block(gx, gy, gz - 1)) add_quad(v000, v010, v110, v100, color);  // -Z
    if (!has_block(gx, gy, gz + 1)) add_quad(v001, v101, v111, v011, color);  // +Z
  }

  if (mk.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Occupied map marker is empty. The binary file may be empty or outside the configured map "
      "bounds/truncate height.");
  }

  occupied_map_pub_->publish(mk);
  RCLCPP_DEBUG(
    this->get_logger(), "Published occupied map mesh marker: topic=%s, triangles=%zu, frame_id=%s",
    occupied_map_topic_.c_str(), mk.points.size() / 3U, frame_id_.c_str());
}

void RvizPlanningNode::clickedPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  Eigen::Vector3d point = pointMsgToEigen(msg->point);
  if (!clicked_point_use_msg_z_) {
    point.z() = default_planning_z_;
  }

  if (!waiting_for_goal_click_) {
    setStart(point);
    waiting_for_goal_click_ = true;
    publishStatusText("Start selected. Click goal point.");
    RCLCPP_INFO(
      this->get_logger(), "Start selected: [%.3f, %.3f, %.3f]", point.x(), point.y(), point.z());
    return;
  }

  waiting_for_goal_click_ = false;
  setGoalAndPlan(point);
}

void RvizPlanningNode::setStart(const Eigen::Vector3d & start)
{
  start_ = start;
  have_start_ = true;
  publishStartGoalMarker();
}

void RvizPlanningNode::setGoalAndPlan(const Eigen::Vector3d & goal)
{
  goal_ = goal;
  have_goal_ = true;
  publishStartGoalMarker();

  if (!have_start_) {
    RCLCPP_WARN(this->get_logger(), "Goal selected but start is not set. Select start first.");
    publishStatusText("Goal selected, but start is not set.");
    return;
  }

  runPlanning();
}

void RvizPlanningNode::runPlanning()
{
  if (!map_.isReady()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Map is not ready. Load occupancy + ESDF binaries or wait for both map topics first.");
    publishStatusText("Map is not ready.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Planning from [%.2f %.2f %.2f] to [%.2f %.2f %.2f]", start_.x(),
    start_.y(), start_.z(), goal_.x(), goal_.y(), goal_.z());

  GuidancePlannerResult result = guidance_planner_.plan(map_, start_, goal_);

  if (result.start_projected || result.goal_projected) {
    start_ = result.planning_start;
    goal_ = result.planning_goal;
    publishStartGoalMarker();
    RCLCPP_INFO(
      this->get_logger(), "Projected RViz raw points to safe corridor points: %s%s%s",
      result.start_projected ? result.start_projection_status.c_str() : "",
      (result.start_projected && result.goal_projected) ? " | " : "",
      result.goal_projected ? result.goal_projection_status.c_str() : "");
  }

  if (!result.raw_path.empty()) {
    publishLineMarker(result.raw_path, astar_marker_pub_, 0, 0.05, 0.1f, 0.4f, 1.0f, 0.95f);
  }

  if (!result.corridor_spheres.empty()) {
    publishCorridorMarkers(result.corridor_spheres);
  } else {
    clearCorridorMarkers();
  }

  if (!result.success) {
    RCLCPP_WARN(this->get_logger(), "%s", result.message.c_str());
    publishStatusText(result.message);
    return;
  }

  publishPath(result.final_waypoints, waypoints_pub_);
  publishLineMarker(
    result.final_waypoints, waypoints_marker_pub_, 1, 0.08, 1.0f, 0.15f, 0.05f, 1.0f);

  publishStatusText(result.message);
  RCLCPP_INFO(this->get_logger(), "%s", result.message.c_str());
}

double RvizPlanningNode::corridorRequiredClearance() const
{
  const auto & options = corridor_generator_.options();
  return std::max(0.0, options.drone_radius + options.safety_margin + options.min_radius);
}

bool RvizPlanningNode::findNearestSafePlanningPoint(
  const Eigen::Vector3d & seed, const std::string & label, Eigen::Vector3d & safe_point,
  std::string & status) const
{
  if (!map_.hasEsdf()) {
    status = "Cannot project " + label + " point: ESDF is not ready.";
    return false;
  }

  Eigen::Vector3d query = seed;
  const double eps = 0.5 * map_.resolution();
  const Eigen::Vector3d lower = map_.origin() + Eigen::Vector3d::Constant(eps);
  const Eigen::Vector3d upper = map_.origin() + map_.size() - Eigen::Vector3d::Constant(eps);
  for (int axis = 0; axis < 3; ++axis) {
    query(axis) = std::clamp(query(axis), lower(axis), upper(axis));
  }

  GridIndex current_index;
  if (!map_.worldToIndex(query, current_index)) {
    std::ostringstream oss;
    oss << "Cannot project " << label << " point: point is outside map bounds.";
    status = oss.str();
    return false;
  }

  const double required_clearance = corridorRequiredClearance();
  const double step_size = std::max(map_.resolution(), 1.0e-3);
  const int max_steps =
    std::max(1, static_cast<int>(std::ceil(safe_point_search_radius_ / step_size)));
  const double max_distance = safe_point_search_radius_ + 0.5 * map_.resolution();
  const double min_progress = 1.0e-6;

  int inspected_points = 0;
  int gradient_steps = 0;
  double best_seen_clearance = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_seen_point = map_.indexToWorld(current_index);

  const auto updateBestSeen = [&](const GridIndex & index, const double clearance) {
    if (std::isfinite(clearance) && clearance > best_seen_clearance) {
      best_seen_clearance = clearance;
      best_seen_point = map_.indexToWorld(index);
    }
  };

  const auto isSafeIndex = [&](const GridIndex & index, double & clearance) -> bool {
    ++inspected_points;
    if (!map_.isInMap(index) || !map_.isFree(index)) {
      clearance = 0.0;
      return false;
    }

    clearance = map_.distance(index);
    updateBestSeen(index, clearance);
    return std::isfinite(clearance) && clearance >= required_clearance;
  };

  const auto isUsableGradientIndex = [&](const GridIndex & index, double & clearance) -> bool {
    ++inspected_points;
    if (!map_.isInMap(index)) {
      clearance = 0.0;
      return false;
    }

    clearance = map_.distance(index);
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
          const Eigen::Vector3d candidate_point = map_.indexToWorld(candidate);
          if ((candidate_point - seed).norm() > max_distance) {
            continue;
          }

          double clearance = 0.0;
          if (!isUsableGradientIndex(candidate, clearance)) {
            continue;
          }

          // This is a local discrete ESDF-gradient ascent step, not the old global radius search.
          // The distance term dominates. The tiny shift penalty keeps the projected point near
          // the original RViz click when several neighbors have nearly identical ESDF values.
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
      safe_point = map_.indexToWorld(current_index);
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

    const Eigen::Vector3d current_point = map_.indexToWorld(current_index);
    const Eigen::Vector3d gradient = map_.gradient(current_point);
    if (
      std::isfinite(gradient.x()) && std::isfinite(gradient.y()) && std::isfinite(gradient.z()) &&
      gradient.norm() > 1.0e-6) {
      Eigen::Vector3d proposal = current_point + step_size * gradient.normalized();
      for (int axis = 0; axis < 3; ++axis) {
        proposal(axis) = std::clamp(proposal(axis), lower(axis), upper(axis));
      }

      GridIndex proposal_index;
      if (
        (proposal - seed).norm() <= max_distance && map_.worldToIndex(proposal, proposal_index) &&
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
      << safe_point_search_radius_ << " m: required ESDF clearance >= " << required_clearance;
  if (std::isfinite(best_seen_clearance)) {
    oss << ", best reached clearance=" << best_seen_clearance
        << " m at shift=" << (best_seen_point - seed).norm() << " m";
  }
  oss << ". Try selecting a point farther from obstacles, increasing "
      << "selection.safe_point_search_radius, or reducing corridor.safety_margin/min_radius.";
  status = oss.str();
  return false;
}

Eigen::Vector3d RvizPlanningNode::pointMsgToEigen(const geometry_msgs::msg::Point & point) const
{
  Eigen::Vector3d out(point.x, point.y, point.z);
  if (!clicked_point_use_msg_z_) {
    out.z() = default_planning_z_;
  }
  return out;
}

void RvizPlanningNode::publishPath(
  const std::vector<Eigen::Vector3d> & path,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub) const
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = this->now();
  msg.poses.reserve(path.size());

  for (const auto & point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }

  pub->publish(msg);
}

void RvizPlanningNode::publishLineMarker(
  const std::vector<Eigen::Vector3d> & path,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub, int id, double width,
  float r, float g, float b, float a) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "planning_path";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.points.reserve(path.size());

  for (const auto & point : path) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    marker.points.push_back(p);
  }

  pub->publish(marker);
}

void RvizPlanningNode::publishStartGoalMarker() const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "planning_start_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  if (have_start_) {
    geometry_msgs::msg::Point p;
    p.x = start_.x();
    p.y = start_.y();
    p.z = start_.z();
    marker.points.push_back(p);

    std_msgs::msg::ColorRGBA c;
    c.r = 0.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    marker.colors.push_back(c);
  }

  if (have_goal_) {
    geometry_msgs::msg::Point p;
    p.x = goal_.x();
    p.y = goal_.y();
    p.z = goal_.z();
    marker.points.push_back(p);

    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = 0.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    marker.colors.push_back(c);
  }

  start_goal_marker_pub_->publish(marker);
}

void RvizPlanningNode::publishCorridorMarkers(const std::vector<CorridorSphere> & spheres)
{
  if (!corridor_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  const auto stamp = this->now();

  for (std::size_t i = 0; i < spheres.size(); ++i) {
    const auto & sphere = spheres[i];
    if (!sphere.valid) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "safe_corridor";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = sphere.center.x();
    marker.pose.position.y = sphere.center.y();
    marker.pose.position.z = sphere.center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * sphere.radius;
    marker.scale.y = 2.0 * sphere.radius;
    marker.scale.z = 2.0 * sphere.radius;
    marker.color.r = 0.55f;
    marker.color.g = 1.0f;
    marker.color.b = 0.55f;
    marker.color.a = static_cast<float>(corridor_marker_alpha_);
    array.markers.push_back(marker);
  }

  for (int i = static_cast<int>(array.markers.size()); i < last_corridor_marker_count_; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "safe_corridor";
    marker.id = i;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(marker);
  }

  last_corridor_marker_count_ = static_cast<int>(spheres.size());
  corridor_marker_pub_->publish(array);
}

void RvizPlanningNode::clearCorridorMarkers()
{
  if (!corridor_marker_pub_ || last_corridor_marker_count_ <= 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  const auto stamp = this->now();
  for (int i = 0; i < last_corridor_marker_count_; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "safe_corridor";
    marker.id = i;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(marker);
  }

  last_corridor_marker_count_ = 0;
  corridor_marker_pub_->publish(array);
}

void RvizPlanningNode::publishStatusText(const std::string &) const
{
  // Intentionally empty: RViz status text is disabled so no white text is shown after selecting
  // points.
}

}  // namespace amprobo

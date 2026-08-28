#include "asr_sdm_kinematic_dynamic_model/underwater_visualization.hpp"

#include <cmath>

namespace asr_sdm_kinematic_dynamic_model
{

Eigen::Matrix3d displayRotation(bool z_to_x_map)
{
  if (!z_to_x_map) {
    return Eigen::Matrix3d::Identity();
  }
  Eigen::Matrix3d rotation;
  rotation << 0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    -1.0, 0.0, 0.0;
  return rotation;
}

Eigen::Vector3d mapPosition(const Eigen::Vector3d & position, bool z_to_x_map)
{
  return displayRotation(z_to_x_map) * position;
}

Eigen::Matrix3d mapRotation(const Eigen::Matrix3d & rotation, bool z_to_x_map)
{
  return displayRotation(z_to_x_map) * rotation;
}

geometry_msgs::msg::Quaternion toRosQuaternion(const Eigen::Matrix3d & rotation)
{
  const Eigen::Quaterniond quaternion(rotation);
  geometry_msgs::msg::Quaternion output;
  output.x = quaternion.x();
  output.y = quaternion.y();
  output.z = quaternion.z();
  output.w = quaternion.w();
  return output;
}

visualization_msgs::msg::Marker makeForceMarker(
  const std::string & frame_id,
  const std::string & name_space,
  int id,
  const builtin_interfaces::msg::Time & stamp,
  const Eigen::Vector3d & origin,
  const Eigen::Vector3d & vector,
  double scale,
  const std::array<float, 4> & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = name_space;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.04;
  marker.points.resize(2);
  marker.points[0].x = origin.x();
  marker.points[0].y = origin.y();
  marker.points[0].z = origin.z();
  const Eigen::Vector3d endpoint = origin + scale * vector;
  marker.points[1].x = endpoint.x();
  marker.points[1].y = endpoint.y();
  marker.points[1].z = endpoint.z();
  if (!vector.array().isFinite().all() || vector.norm() < 1.0e-12 || scale <= 0.0) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.points.clear();
  }
  return marker;
}

visualization_msgs::msg::Marker makeTrajectoryMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const std::vector<Eigen::Vector3d> & points,
  double width,
  const std::array<float, 4> & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = width;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  for (const auto & point : points) {
    geometry_msgs::msg::Point message_point;
    message_point.x = point.x();
    message_point.y = point.y();
    message_point.z = point.z();
    marker.points.push_back(message_point);
  }
  return marker;
}

}  // namespace asr_sdm_kinematic_dynamic_model

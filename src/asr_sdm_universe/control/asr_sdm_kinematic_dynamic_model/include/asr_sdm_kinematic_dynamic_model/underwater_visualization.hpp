#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_VISUALIZATION_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_VISUALIZATION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

Eigen::Matrix3d displayRotation(bool z_to_x_map);
Eigen::Vector3d mapPosition(const Eigen::Vector3d & position, bool z_to_x_map);
Eigen::Matrix3d mapRotation(const Eigen::Matrix3d & rotation, bool z_to_x_map);
geometry_msgs::msg::Quaternion toRosQuaternion(const Eigen::Matrix3d & rotation);

visualization_msgs::msg::Marker makeForceMarker(
  const std::string & frame_id,
  const std::string & name_space,
  int id,
  const builtin_interfaces::msg::Time & stamp,
  const Eigen::Vector3d & origin,
  const Eigen::Vector3d & vector,
  double scale,
  const std::array<float, 4> & color);

visualization_msgs::msg::Marker makeTrajectoryMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const std::vector<Eigen::Vector3d> & points,
  double width,
  const std::array<float, 4> & color);

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_UNDERWATER_VISUALIZATION_HPP_

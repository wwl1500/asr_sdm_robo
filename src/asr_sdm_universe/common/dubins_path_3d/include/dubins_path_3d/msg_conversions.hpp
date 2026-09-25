// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef DUBINS_PATH_3D__MSG_CONVERSIONS_HPP_
#define DUBINS_PATH_3D__MSG_CONVERSIONS_HPP_

#include <string>
#include <utility>

#include "asr_sdm_control_msgs/msg/configuration.hpp"
#include "asr_sdm_control_msgs/msg/dubins_path.hpp"
#include "asr_sdm_control_msgs/msg/path_sample.hpp"
#include "asr_sdm_control_msgs/srv/plan_path.hpp"
#include "dubins_path_3d/types.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace dubins_path_3d
{

inline geometry_msgs::msg::Point toPointMsg(const Vec3 & value)
{
  geometry_msgs::msg::Point msg;
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
  return msg;
}

inline geometry_msgs::msg::Vector3 toVector3Msg(const Vec3 & value)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
  return msg;
}

inline Vec3 fromMsg(const geometry_msgs::msg::Point & msg)
{
  return Vec3(msg.x, msg.y, msg.z);
}

inline Vec3 fromMsg(const geometry_msgs::msg::Vector3 & msg)
{
  return Vec3(msg.x, msg.y, msg.z);
}

inline asr_sdm_control_msgs::msg::Configuration toMsg(const Configuration & config)
{
  asr_sdm_control_msgs::msg::Configuration msg;
  msg.position = toPointMsg(config.position);
  msg.tangent = toVector3Msg(config.tangent);
  msg.tangent_normal = toVector3Msg(config.tangent_normal);
  msg.surface_normal = toVector3Msg(config.surface_normal);
  return msg;
}

inline Configuration fromMsg(const asr_sdm_control_msgs::msg::Configuration & msg)
{
  Configuration config;
  config.position = fromMsg(msg.position);
  config.tangent = fromMsg(msg.tangent);
  config.tangent_normal = fromMsg(msg.tangent_normal);
  config.surface_normal = fromMsg(msg.surface_normal);
  return config;
}

inline asr_sdm_control_msgs::msg::PathSample toMsg(const PathSample & sample)
{
  asr_sdm_control_msgs::msg::PathSample msg;
  msg.position = toPointMsg(sample.position);
  msg.tangent = toVector3Msg(sample.tangent);
  msg.tangent_normal = toVector3Msg(sample.tangent_normal);
  msg.surface_normal = toVector3Msg(sample.surface_normal);
  return msg;
}

inline PathSample fromMsg(const asr_sdm_control_msgs::msg::PathSample & msg)
{
  PathSample sample;
  sample.position = fromMsg(msg.position);
  sample.tangent = fromMsg(msg.tangent);
  sample.tangent_normal = fromMsg(msg.tangent_normal);
  sample.surface_normal = fromMsg(msg.surface_normal);
  return sample;
}

inline asr_sdm_control_msgs::msg::DubinsPath toMsg(const Path3D & path)
{
  asr_sdm_control_msgs::msg::DubinsPath msg;
  msg.path_type = path.type;
  msg.length = path.length;
  msg.samples.reserve(path.samples.size());
  for (const PathSample & sample : path.samples) {
    msg.samples.push_back(toMsg(sample));
  }
  return msg;
}

inline Path3D fromMsg(const asr_sdm_control_msgs::msg::DubinsPath & msg)
{
  Path3D path;
  path.type = msg.path_type;
  path.length = msg.length;
  path.samples.reserve(msg.samples.size());
  for (const auto & sample : msg.samples) {
    path.samples.push_back(fromMsg(sample));
  }
  return path;
}

/// Applies per-call overrides from a PlanPath request. Non-positive fields keep
/// the values already present in `options`.
inline PlannerOptions applyRequestOverrides(
  const asr_sdm_control_msgs::srv::PlanPath::Request & request,
  PlannerOptions options = {})
{
  if (request.pitch_radius > 0.0) {
    options.pitch_radius = request.pitch_radius;
  }
  if (request.yaw_radius > 0.0) {
    options.yaw_radius = request.yaw_radius;
  }
  if (request.tight_turn_radius > 0.0) {
    options.tight_turn_radius = request.tight_turn_radius;
  }
  if (request.location_samples > 0) {
    options.location_samples = request.location_samples;
  }
  if (request.heading_samples > 0) {
    options.heading_samples = request.heading_samples;
  }
  if (request.sample_spacing > 0.0) {
    options.sample_spacing = request.sample_spacing;
  }
  return options;
}

inline void toMsg(
  const PlanningResult & result,
  asr_sdm_control_msgs::srv::PlanPath::Response & response)
{
  response.success = result.success();
  response.message = result.success() ? "ok" : "no feasible path";
  response.path = toMsg(result.best);
  response.planning_time = result.planning_time;
  response.candidate_types.clear();
  response.candidate_lengths.clear();
  response.candidate_types.reserve(result.candidates.size());
  response.candidate_lengths.reserve(result.candidates.size());
  for (const std::pair<std::string, double> & candidate : result.candidates) {
    response.candidate_types.push_back(candidate.first);
    response.candidate_lengths.push_back(candidate.second);
  }
}

}  // namespace dubins_path_3d

#endif  // DUBINS_PATH_3D__MSG_CONVERSIONS_HPP_

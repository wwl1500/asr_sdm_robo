// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "dubins_path_3d/msg_conversions.hpp"

namespace
{

using dubins_path_3d::Configuration;
using dubins_path_3d::Path3D;
using dubins_path_3d::PathSample;
using dubins_path_3d::PlannerOptions;
using dubins_path_3d::PlanningResult;
using dubins_path_3d::Vec3;

Configuration sampleConfig()
{
  Configuration config;
  config.position = Vec3(1.0, 2.0, 3.0);
  config.tangent = Vec3(0.0, 1.0, 0.0);
  config.tangent_normal = Vec3(-1.0, 0.0, 0.0);
  config.surface_normal = Vec3(0.0, 0.0, 1.0);
  return config;
}

TEST(MsgConversions, ConfigurationRoundTrip)
{
  const Configuration original = sampleConfig();
  const Configuration restored = dubins_path_3d::fromMsg(dubins_path_3d::toMsg(original));
  EXPECT_TRUE(original.position.isApprox(restored.position));
  EXPECT_TRUE(original.tangent.isApprox(restored.tangent));
  EXPECT_TRUE(original.tangent_normal.isApprox(restored.tangent_normal));
  EXPECT_TRUE(original.surface_normal.isApprox(restored.surface_normal));
}

TEST(MsgConversions, PathRoundTrip)
{
  Path3D path;
  path.type = "cyc_inner";
  path.length = 12.5;
  PathSample sample;
  sample.position = Vec3(4.0, 5.0, 6.0);
  sample.tangent = Vec3(1.0, 0.0, 0.0);
  sample.tangent_normal = Vec3(0.0, 1.0, 0.0);
  sample.surface_normal = Vec3(0.0, 0.0, 1.0);
  path.samples.push_back(sample);

  const Path3D restored = dubins_path_3d::fromMsg(dubins_path_3d::toMsg(path));
  EXPECT_EQ(path.type, restored.type);
  EXPECT_DOUBLE_EQ(path.length, restored.length);
  ASSERT_EQ(restored.samples.size(), 1u);
  EXPECT_TRUE(sample.position.isApprox(restored.samples.front().position));
}

TEST(MsgConversions, PlanPathOverridesAndResponse)
{
  asr_sdm_control_msgs::srv::PlanPath::Request request;
  request.pitch_radius = 30.0;
  request.yaw_radius = -1.0;
  request.location_samples = 9;
  request.heading_samples = 0;
  request.sample_spacing = 1.5;

  PlannerOptions options;
  options.pitch_radius = 40.0;
  options.yaw_radius = 50.0;
  options.location_samples = 15;
  options.heading_samples = 11;
  options.sample_spacing = 2.0;

  const PlannerOptions merged = dubins_path_3d::applyRequestOverrides(request, options);
  EXPECT_DOUBLE_EQ(merged.pitch_radius, 30.0);
  EXPECT_DOUBLE_EQ(merged.yaw_radius, 50.0);
  EXPECT_EQ(merged.location_samples, 9);
  EXPECT_EQ(merged.heading_samples, 11);
  EXPECT_DOUBLE_EQ(merged.sample_spacing, 1.5);

  PlanningResult result;
  result.best.type = "plane_left_right";
  result.best.length = 8.0;
  result.best.samples.push_back(PathSample{});
  result.candidates.emplace_back("cyc_inner", 10.0);
  result.candidates.emplace_back("sphere_outer", std::numeric_limits<double>::infinity());
  result.planning_time = 0.25;

  asr_sdm_control_msgs::srv::PlanPath::Response response;
  dubins_path_3d::toMsg(result, response);
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.path.path_type, "plane_left_right");
  EXPECT_DOUBLE_EQ(response.planning_time, 0.25);
  ASSERT_EQ(response.candidate_types.size(), 2u);
  EXPECT_EQ(response.candidate_types[0], "cyc_inner");
  EXPECT_TRUE(std::isinf(response.candidate_lengths[1]));
}

}  // namespace

// Copyright (c) Amphibious Robotics.
// Shared guidance planner types.

#ifndef ASR_SDM_GUIDANCE_PLANNER_COMMON_TYPES_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_COMMON_TYPES_HPP_

#include <Eigen/Core>
#include <map_query_interface.hpp>

#include <string>
#include <vector>

namespace amprobo
{

struct PlanResult
{
  bool success = false;
  std::string message;
  std::vector<Eigen::Vector3d> raw_path;
};

}  // namespace amprobo

#endif  // ASR_SDM_GUIDANCE_PLANNER_COMMON_TYPES_HPP_

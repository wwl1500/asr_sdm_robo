// Copyright (c) Amphibious Robotics.
// Base planner interface for guidance planning.

#ifndef ASR_SDM_GUIDANCE_PLANNER_PLANNER_PLANNER_BASE_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_PLANNER_PLANNER_BASE_HPP_

#include <Eigen/Core>
#include <types.hpp>

namespace amprobo
{

class PlannerBase
{
public:
  virtual ~PlannerBase() = default;

  virtual PlanResult plan(
    const MapQueryInterface & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) = 0;
};

}  // namespace amprobo

#endif  // ASR_SDM_GUIDANCE_PLANNER_PLANNER_PLANNER_BASE_HPP_

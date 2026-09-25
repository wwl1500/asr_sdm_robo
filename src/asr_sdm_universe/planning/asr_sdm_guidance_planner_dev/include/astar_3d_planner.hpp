// Copyright (c) Amphibious Robotics.
// 3D A* planner for guidance paths.

#ifndef ASR_SDM_GUIDANCE_PLANNER_PLANNER_ASTAR_3D_PLANNER_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_PLANNER_ASTAR_3D_PLANNER_HPP_

#include <planner_base.hpp>

#include <vector>

namespace amprobo
{

struct Astar3dOptions
{
  double heuristic_weight = 1.0;
  double extra_clearance = 0.0;
  int max_expansions = 300000;
  int nearest_free_search_radius = 10;
  bool allow_diagonal = true;
};

class Astar3dPlanner final : public PlannerBase
{
public:
  explicit Astar3dPlanner(const Astar3dOptions & options = Astar3dOptions());

  void setOptions(const Astar3dOptions & options) { options_ = options; }
  const Astar3dOptions & options() const { return options_; }

  PlanResult plan(
    const MapQueryInterface & map, const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal) override;

private:
  Astar3dOptions options_;
  std::vector<GridIndex> neighborOffsets() const;
};

}  // namespace amprobo

#endif  // ASR_SDM_GUIDANCE_PLANNER_PLANNER_ASTAR_3D_PLANNER_HPP_

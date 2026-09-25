// Copyright (c) Amphibious Robotics.
// 3D A* planner implementation.

#include <astar_3d_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>

namespace amprobo
{

namespace
{
struct QueueNode
{
  int address = -1;
  double f = 0.0;

  bool operator>(const QueueNode & other) const { return f > other.f; }
};

constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

Astar3dPlanner::Astar3dPlanner(const Astar3dOptions & options) : options_(options)
{
}

std::vector<GridIndex> Astar3dPlanner::neighborOffsets() const
{
  std::vector<GridIndex> offsets;
  offsets.reserve(options_.allow_diagonal ? 26 : 6);

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }

        const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
        if (!options_.allow_diagonal && manhattan != 1) {
          continue;
        }

        offsets.push_back(GridIndex{dx, dy, dz});
      }
    }
  }

  return offsets;
}

PlanResult Astar3dPlanner::plan(
  const MapQueryInterface & map, const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  PlanResult result;

  if (!map.isReady()) {
    result.message = "map is not ready";
    return result;
  }

  GridIndex start_index;
  GridIndex goal_index;
  if (!map.worldToIndex(start, start_index)) {
    result.message = "start is outside map";
    return result;
  }
  if (!map.worldToIndex(goal, goal_index)) {
    result.message = "goal is outside map";
    return result;
  }

  GridIndex start_free;
  GridIndex goal_free;
  if (!map.findNearestFree(start_index, options_.nearest_free_search_radius, start_free)) {
    result.message = "cannot find a free start voxel";
    return result;
  }
  if (!map.findNearestFree(goal_index, options_.nearest_free_search_radius, goal_free)) {
    result.message = "cannot find a free goal voxel";
    return result;
  }

  const int start_addr = map.toAddress(start_free);
  const int goal_addr = map.toAddress(goal_free);
  const int n = map.voxelCount();

  std::vector<double> g_score(n, kInf);
  std::vector<int> parent(n, -1);
  std::vector<unsigned char> closed(n, 0);
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  const auto heuristic = [&](const GridIndex & a, const GridIndex & b) {
    const Eigen::Vector3d pa = map.indexToWorld(a);
    const Eigen::Vector3d pb = map.indexToWorld(b);
    return options_.heuristic_weight * (pa - pb).norm();
  };

  g_score[start_addr] = 0.0;
  open.push(QueueNode{start_addr, heuristic(start_free, goal_free)});

  const auto offsets = neighborOffsets();
  int expansions = 0;
  bool found = false;

  while (!open.empty()) {
    const QueueNode current_node = open.top();
    open.pop();

    const int current_addr = current_node.address;
    if (closed[current_addr]) {
      continue;
    }
    closed[current_addr] = 1;
    ++expansions;

    if (current_addr == goal_addr) {
      found = true;
      break;
    }

    if (options_.max_expansions > 0 && expansions > options_.max_expansions) {
      break;
    }

    GridIndex current;
    map.addressToIndex(current_addr, current);

    for (const auto & offset : offsets) {
      GridIndex next{current.x + offset.x, current.y + offset.y, current.z + offset.z};
      if (!map.isInMap(next) || !map.isFree(next, options_.extra_clearance)) {
        continue;
      }

      const int next_addr = map.toAddress(next);
      if (closed[next_addr]) {
        continue;
      }

      const double step_cost =
        map.resolution() *
        std::sqrt(
          static_cast<double>(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z));
      const double tentative_g = g_score[current_addr] + step_cost;

      if (tentative_g + 1.0e-12 < g_score[next_addr]) {
        g_score[next_addr] = tentative_g;
        parent[next_addr] = current_addr;
        const double f = tentative_g + heuristic(next, goal_free);
        open.push(QueueNode{next_addr, f});
      }
    }
  }

  if (!found) {
    std::ostringstream oss;
    oss << "A* failed after " << expansions << " expansions";
    result.message = oss.str();
    return result;
  }

  std::vector<int> reversed;
  int current = goal_addr;
  while (current >= 0) {
    reversed.push_back(current);
    if (current == start_addr) {
      break;
    }
    current = parent[current];
  }

  if (reversed.empty() || reversed.back() != start_addr) {
    result.message = "A* parent chain is incomplete";
    return result;
  }

  std::reverse(reversed.begin(), reversed.end());
  result.raw_path.reserve(reversed.size());
  for (const int address : reversed) {
    GridIndex index;
    map.addressToIndex(address, index);
    result.raw_path.push_back(map.indexToWorld(index));
  }

  if (!result.raw_path.empty()) {
    result.raw_path.front() = start;
    result.raw_path.back() = goal;
  }

  result.success = true;
  std::ostringstream oss;
  oss << "A* success: " << result.raw_path.size() << " points, " << expansions << " expansions";
  result.message = oss.str();
  return result;
}

}  // namespace amprobo

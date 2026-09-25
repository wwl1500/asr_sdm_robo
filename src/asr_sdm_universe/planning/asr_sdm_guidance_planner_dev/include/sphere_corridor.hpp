// Copyright (c) Amphibious Robotics.
// Spherical corridor generation interface.

#ifndef ASR_SDM_GUIDANCE_PLANNER_PLANNER_SPHERE_CORRIDOR_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_PLANNER_SPHERE_CORRIDOR_HPP_

#include <Eigen/Core>
#include <types.hpp>

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace amprobo
{

struct CorridorSphere
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d nearest_obstacle = Eigen::Vector3d::Zero();
  double radius = 0.0;
  double raw_distance = 0.0;
  double score = 0.0;
  bool valid = false;

  bool contains(const Eigen::Vector3d & point, double eps = 1.0e-6) const
  {
    return valid && (point - center).norm() <= radius + eps;
  }
};

struct SphereCorridorOptions
{
  bool enabled = true;
  int batch_sample_count = 80;
  int max_spheres = 80;
  double drone_radius = 0.0;
  double safety_margin = 0.25;
  double min_radius = 0.18;
  double max_radius = 6.0;
  double min_overlap_volume = 1.0e-5;
  double radius_weight = 1.0;
  double overlap_weight = 4.0;
  double sample_axis_scale = 1.0 / 3.0;
  double sample_lateral_scale = 2.0;
  uint32_t random_seed = 7U;
  bool deterministic_sampling = true;
};

struct SphereCorridorResult
{
  bool success = false;
  std::string message;
  std::vector<CorridorSphere> spheres;
  std::vector<Eigen::Vector3d> waypoints;
};

class SphereCorridorGenerator
{
public:
  explicit SphereCorridorGenerator(const SphereCorridorOptions & options = SphereCorridorOptions());

  void setOptions(const SphereCorridorOptions & options) { options_ = options; }
  const SphereCorridorOptions & options() const { return options_; }

  SphereCorridorResult generate(
    const MapQueryInterface & map, const std::vector<Eigen::Vector3d> & guide_path,
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal);

  CorridorSphere generateOneSphere(
    const MapQueryInterface & map, const Eigen::Vector3d & center) const;
  static double sphereVolume(double radius);
  static double overlapVolume(const CorridorSphere & a, const CorridorSphere & b);

private:
  SphereCorridorOptions options_;
  std::mt19937 rng_;

  CorridorSphere batchSample(
    const MapQueryInterface & map, const Eigen::Vector3d & guide_point,
    const CorridorSphere & last_sphere);

  std::size_t getForwardPointOnPath(
    const std::vector<Eigen::Vector3d> & guide_path, const CorridorSphere & current_sphere,
    std::size_t start_index) const;

  std::vector<Eigen::Vector3d> initializeWaypoints(
    const std::vector<CorridorSphere> & spheres, const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal) const;

  Eigen::Vector3d overlapCenter(const CorridorSphere & a, const CorridorSphere & b) const;
};

}  // namespace amprobo

#endif  // ASR_SDM_GUIDANCE_PLANNER_PLANNER_SPHERE_CORRIDOR_HPP_

// Copyright (c) Amphibious Robotics.
// Spherical corridor generation implementation.

#include <Eigen/Geometry>
#include <sphere_corridor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace amprobo
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d normalizedOr(const Eigen::Vector3d & v, const Eigen::Vector3d & fallback)
{
  const double n = v.norm();
  if (n > 1.0e-6) {
    return v / n;
  }
  return fallback;
}

void buildOrthonormalBasis(
  const Eigen::Vector3d & axis, Eigen::Vector3d & e1, Eigen::Vector3d & e2, Eigen::Vector3d & e3)
{
  e1 = normalizedOr(axis, Eigen::Vector3d::UnitX());
  Eigen::Vector3d helper = std::abs(e1.dot(Eigen::Vector3d::UnitZ())) < 0.9
                             ? Eigen::Vector3d::UnitZ()
                             : Eigen::Vector3d::UnitY();
  e2 = e1.cross(helper);
  if (e2.norm() < 1.0e-6) {
    helper = Eigen::Vector3d::UnitY();
    e2 = e1.cross(helper);
  }
  e2.normalize();
  e3 = e1.cross(e2).normalized();
}
}  // namespace

SphereCorridorGenerator::SphereCorridorGenerator(const SphereCorridorOptions & options)
: options_(options), rng_(options.random_seed)
{
}

double SphereCorridorGenerator::sphereVolume(const double radius)
{
  if (radius <= 0.0 || !std::isfinite(radius)) {
    return 0.0;
  }
  return 4.0 * kPi * radius * radius * radius / 3.0;
}

double SphereCorridorGenerator::overlapVolume(const CorridorSphere & a, const CorridorSphere & b)
{
  if (!a.valid || !b.valid || a.radius <= 0.0 || b.radius <= 0.0) {
    return 0.0;
  }

  const double r1 = a.radius;
  const double r2 = b.radius;
  const double d = (a.center - b.center).norm();

  if (d >= r1 + r2) {
    return 0.0;
  }

  if (d <= std::abs(r1 - r2)) {
    return sphereVolume(std::min(r1, r2));
  }

  if (d < 1.0e-6) {
    return sphereVolume(std::min(r1, r2));
  }

  const double sum = r1 + r2;
  const double diff = r1 - r2;
  return kPi * (sum - d) * (sum - d) * (d * d + 2.0 * d * sum - 3.0 * diff * diff) / (12.0 * d);
}

CorridorSphere SphereCorridorGenerator::generateOneSphere(
  const MapQueryInterface & map, const Eigen::Vector3d & center) const
{
  CorridorSphere sphere;
  sphere.center = center;

  if (!map.isInMap(center)) {
    return sphere;
  }

  const double raw_distance = map.distance(center);
  if (!std::isfinite(raw_distance)) {
    return sphere;
  }

  const double clearance = std::max(0.0, options_.drone_radius + options_.safety_margin);
  sphere.raw_distance = raw_distance;
  sphere.radius = std::min(
    std::max(0.0, raw_distance - clearance), std::max(options_.min_radius, options_.max_radius));

  const Eigen::Vector3d grad = map.gradient(center);
  if (grad.norm() > 1.0e-6) {
    sphere.nearest_obstacle = center - grad.normalized() * raw_distance;
  } else {
    sphere.nearest_obstacle = center;
  }

  sphere.valid = sphere.radius >= options_.min_radius;
  return sphere;
}

CorridorSphere SphereCorridorGenerator::batchSample(
  const MapQueryInterface & map, const Eigen::Vector3d & guide_point,
  const CorridorSphere & last_sphere)
{
  CorridorSphere best;
  best.score = -std::numeric_limits<double>::infinity();

  const double dist_to_last = (last_sphere.center - guide_point).norm();
  const double sigma_axis = std::max(map.resolution(), options_.sample_axis_scale * dist_to_last);
  const double sigma_lateral =
    std::max(map.resolution(), options_.sample_lateral_scale * sigma_axis);

  Eigen::Vector3d e1, e2, e3;
  buildOrthonormalBasis(last_sphere.center - guide_point, e1, e2, e3);

  if (!options_.deterministic_sampling) {
    const auto seed =
      static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    rng_.seed(seed ^ options_.random_seed);
  } else {
    rng_.seed(options_.random_seed);
  }

  std::normal_distribution<double> standard_normal(0.0, 1.0);
  const int sample_count = std::max(1, options_.batch_sample_count);

  for (int k = 0; k < sample_count; ++k) {
    Eigen::Vector3d candidate_center = guide_point;
    if (k > 0) {
      candidate_center += e1 * (sigma_axis * standard_normal(rng_));
      candidate_center += e2 * (sigma_lateral * standard_normal(rng_));
      candidate_center += e3 * (sigma_lateral * standard_normal(rng_));
    }

    CorridorSphere candidate = generateOneSphere(map, candidate_center);
    if (!candidate.valid) {
      continue;
    }

    // Keep the corridor connected and force this sphere to cover the selected A* guide point.
    if (!candidate.contains(guide_point, map.resolution())) {
      continue;
    }

    const double overlap = overlapVolume(last_sphere, candidate);
    if (overlap < options_.min_overlap_volume) {
      continue;
    }

    const double volume = sphereVolume(candidate.radius);
    candidate.score = options_.radius_weight * volume + options_.overlap_weight * overlap;
    if (!best.valid || candidate.score > best.score) {
      best = candidate;
    }
  }

  return best;
}

std::size_t SphereCorridorGenerator::getForwardPointOnPath(
  const std::vector<Eigen::Vector3d> & guide_path, const CorridorSphere & current_sphere,
  const std::size_t start_index) const
{
  if (guide_path.empty()) {
    return 0U;
  }

  const std::size_t begin = std::min(start_index, guide_path.size() - 1U);
  for (std::size_t i = begin; i < guide_path.size(); ++i) {
    if (!current_sphere.contains(guide_path[i])) {
      return i;
    }
  }

  return guide_path.size() - 1U;
}

Eigen::Vector3d SphereCorridorGenerator::overlapCenter(
  const CorridorSphere & a, const CorridorSphere & b) const
{
  const Eigen::Vector3d ab = b.center - a.center;
  const double d = ab.norm();
  if (d < 1.0e-6) {
    return 0.5 * (a.center + b.center);
  }

  const Eigen::Vector3d dir = ab / d;
  const double lower = std::max(-a.radius, d - b.radius);
  const double upper = std::min(a.radius, d + b.radius);
  const double x = 0.5 * (lower + upper);
  return a.center + x * dir;
}

std::vector<Eigen::Vector3d> SphereCorridorGenerator::initializeWaypoints(
  const std::vector<CorridorSphere> & spheres, const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal) const
{
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.reserve(spheres.size() + 1U);
  waypoints.push_back(start);

  for (std::size_t i = 1; i < spheres.size(); ++i) {
    waypoints.push_back(overlapCenter(spheres[i - 1U], spheres[i]));
  }

  waypoints.push_back(goal);
  return waypoints;
}

SphereCorridorResult SphereCorridorGenerator::generate(
  const MapQueryInterface & map, const std::vector<Eigen::Vector3d> & guide_path,
  const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  SphereCorridorResult result;

  if (!options_.enabled) {
    result.success = true;
    result.message = "sphere corridor disabled";
    result.waypoints = guide_path;
    return result;
  }

  if (!map.hasDistanceField()) {
    result.message = "ESDF is not ready; cannot generate sphere corridor";
    return result;
  }

  if (guide_path.size() < 2U) {
    result.message = "A* guide path is too short for sphere corridor generation";
    return result;
  }

  CorridorSphere current = generateOneSphere(map, start);
  if (!current.valid) {
    std::ostringstream oss;
    oss << "start point has insufficient ESDF clearance for corridor: distance="
        << current.raw_distance
        << ", required>=" << (options_.drone_radius + options_.safety_margin + options_.min_radius);
    result.message = oss.str();
    return result;
  }

  result.spheres.push_back(current);
  std::size_t guide_index = 0U;

  int stuck_count = 0;
  while (!current.contains(goal) &&
         static_cast<int>(result.spheres.size()) < std::max(1, options_.max_spheres)) {
    const std::size_t next_index = getForwardPointOnPath(guide_path, current, guide_index);
    const Eigen::Vector3d guide_point = guide_path[next_index];

    CorridorSphere next = batchSample(map, guide_point, current);
    if (!next.valid) {
      next = generateOneSphere(map, guide_point);
      if (next.valid) {
        const double overlap = overlapVolume(current, next);
        next.score =
          options_.radius_weight * sphereVolume(next.radius) + options_.overlap_weight * overlap;
        if (overlap < options_.min_overlap_volume) {
          next.valid = false;
        }
      }
    }

    if (!next.valid) {
      std::ostringstream oss;
      oss << "BatchSample failed near guide point [" << guide_point.x() << ", " << guide_point.y()
          << ", " << guide_point.z() << "]";
      result.message = oss.str();
      return result;
    }

    if ((next.center - current.center).norm() < 0.25 * map.resolution() && !next.contains(goal)) {
      ++stuck_count;
    } else {
      stuck_count = 0;
    }
    if (stuck_count > 3) {
      result.message =
        "sphere corridor generation stopped because sampled centers stopped advancing";
      return result;
    }

    result.spheres.push_back(next);
    current = next;
    guide_index = std::max(guide_index, next_index);
  }

  if (!current.contains(goal)) {
    std::ostringstream oss;
    oss << "sphere corridor generation exceeded max_spheres=" << options_.max_spheres;
    result.message = oss.str();
    return result;
  }

  result.waypoints = initializeWaypoints(result.spheres, start, goal);
  result.success = true;

  double min_radius = std::numeric_limits<double>::infinity();
  double max_radius = 0.0;
  double min_overlap = std::numeric_limits<double>::infinity();
  for (const auto & sphere : result.spheres) {
    min_radius = std::min(min_radius, sphere.radius);
    max_radius = std::max(max_radius, sphere.radius);
  }
  for (std::size_t i = 1; i < result.spheres.size(); ++i) {
    min_overlap = std::min(min_overlap, overlapVolume(result.spheres[i - 1U], result.spheres[i]));
  }
  if (!std::isfinite(min_overlap)) {
    min_overlap = 0.0;
  }

  std::ostringstream oss;
  oss << "sphere corridor success: spheres=" << result.spheres.size()
      << ", waypoints=" << result.waypoints.size() << ", radius=[" << min_radius << ", "
      << max_radius << "]"
      << ", min_overlap=" << min_overlap;
  result.message = oss.str();
  return result;
}

}  // namespace amprobo

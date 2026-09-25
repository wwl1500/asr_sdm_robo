// Copyright (c) Amphibious Robotics.
// LBFGS path optimizer implementation.

#include <asr_sdm_lbfgs_solver/lbfgs.hpp>
#include <lbfgs_path_optimizer.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace amprobo
{

namespace
{
std::vector<Eigen::Vector3d> unpackPath(
  const std::vector<double> & x, const std::vector<Eigen::Vector3d> & fixed_path)
{
  std::vector<Eigen::Vector3d> path = fixed_path;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const std::size_t base = 3 * (i - 1);
    path[i].x() = x[base + 0];
    path[i].y() = x[base + 1];
    path[i].z() = x[base + 2];
  }
  return path;
}

void packGradient(const std::vector<Eigen::Vector3d> & grad_points, double * g)
{
  for (std::size_t i = 1; i + 1 < grad_points.size(); ++i) {
    const std::size_t base = 3 * (i - 1);
    g[base + 0] = grad_points[i].x();
    g[base + 1] = grad_points[i].y();
    g[base + 2] = grad_points[i].z();
  }
}
}  // namespace

LbfgsPathOptimizer::LbfgsPathOptimizer(const LbfgsPathOptimizerOptions & options)
: options_(options)
{
}

std::vector<Eigen::Vector3d> LbfgsPathOptimizer::selectControlPoints(
  const std::vector<Eigen::Vector3d> & raw_path) const
{
  if (raw_path.size() <= static_cast<std::size_t>(std::max(3, options_.max_control_points))) {
    return raw_path;
  }

  const int keep = std::max(3, options_.max_control_points);
  std::vector<Eigen::Vector3d> reduced;
  reduced.reserve(keep);
  for (int i = 0; i < keep; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(keep - 1);
    const std::size_t index =
      static_cast<std::size_t>(std::round(ratio * static_cast<double>(raw_path.size() - 1)));
    reduced.push_back(raw_path[std::min(index, raw_path.size() - 1)]);
  }
  reduced.front() = raw_path.front();
  reduced.back() = raw_path.back();
  return reduced;
}

OptimizerResult LbfgsPathOptimizer::optimize(
  const std::vector<Eigen::Vector3d> & raw_path, const MapQueryInterface & map,
  const std::vector<CorridorSphere> * corridor) const
{
  OptimizerResult result;

  if (!options_.enabled) {
    result.success = true;
    result.path = raw_path;
    result.path_safe =
      map.pathIsFree(raw_path, options_.validity_check_step, options_.extra_clearance);
    result.message = "L-BFGS disabled";
    return result;
  }

  if (raw_path.size() < 3) {
    result.success = true;
    result.path = raw_path;
    result.path_safe =
      map.pathIsFree(raw_path, options_.validity_check_step, options_.extra_clearance);
    result.message = "path has fewer than 3 points; skip optimization";
    return result;
  }

  if (!map.hasDistanceField()) {
    result.success = false;
    result.path = raw_path;
    result.path_safe =
      map.pathIsFree(raw_path, options_.validity_check_step, options_.extra_clearance);
    result.message = "ESDF is not ready; skip optimization";
    return result;
  }

  const std::vector<Eigen::Vector3d> guide_path =
    (corridor != nullptr) ? raw_path : selectControlPoints(raw_path);
  const int variable_count = static_cast<int>(3 * (guide_path.size() - 2));
  std::vector<double> x(variable_count, 0.0);

  for (std::size_t i = 1; i + 1 < guide_path.size(); ++i) {
    const std::size_t base = 3 * (i - 1);
    x[base + 0] = guide_path[i].x();
    x[base + 1] = guide_path[i].y();
    x[base + 2] = guide_path[i].z();
  }

  const auto evaluate = [&](const double * x_raw, double * g_raw, int n, double /*step*/) {
    std::vector<double> xv(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      xv[static_cast<std::size_t>(i)] = x_raw[i];
    }

    std::vector<Eigen::Vector3d> path = unpackPath(xv, guide_path);
    std::vector<Eigen::Vector3d> grad_points(path.size(), Eigen::Vector3d::Zero());
    double fx = 0.0;

    const double smooth_w = options_.smooth_weight;
    const double length_w = options_.length_weight;
    const double obstacle_w = options_.obstacle_weight;
    const double guide_w = options_.guide_weight;
    const double safe_distance = options_.safe_distance;
    const double corridor_w = options_.corridor_weight;

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const Eigen::Vector3d second_diff = path[i - 1] - 2.0 * path[i] + path[i + 1];
      fx += smooth_w * second_diff.squaredNorm();
      const Eigen::Vector3d common = 2.0 * smooth_w * second_diff;
      grad_points[i - 1] += common;
      grad_points[i] += -2.0 * common;
      grad_points[i + 1] += common;
    }

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const Eigen::Vector3d diff = path[i + 1] - path[i];
      fx += length_w * diff.squaredNorm();
      const Eigen::Vector3d common = 2.0 * length_w * diff;
      grad_points[i] -= common;
      grad_points[i + 1] += common;
    }

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const Eigen::Vector3d diff = path[i] - guide_path[i];
      fx += guide_w * diff.squaredNorm();
      grad_points[i] += 2.0 * guide_w * diff;
    }

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const Eigen::Vector3d p = path[i];
      if (!map.isInMap(p)) {
        const Eigen::Vector3d map_center = map.origin() + 0.5 * map.size();
        const Eigen::Vector3d diff = p - map_center;
        fx += obstacle_w * 100.0 * diff.squaredNorm();
        grad_points[i] += 2.0 * obstacle_w * 100.0 * diff;
        continue;
      }

      const double dist = map.distance(p);
      if (!std::isfinite(dist)) {
        continue;
      }

      if (dist < safe_distance) {
        const double violation = safe_distance - dist;
        fx += obstacle_w * violation * violation;
        const Eigen::Vector3d grad_dist = map.gradient(p);
        grad_points[i] += -2.0 * obstacle_w * violation * grad_dist;
      }
    }

    if (corridor != nullptr && corridor->size() + 1U == path.size() && corridor_w > 0.0) {
      const auto add_corridor_penalty =
        [&, corridor_w](const std::size_t point_index, const CorridorSphere & sphere) {
          if (!sphere.valid) {
            return;
          }
          const Eigen::Vector3d diff = path[point_index] - sphere.center;
          const double norm = diff.norm();
          const double violation = norm - sphere.radius;
          if (violation <= 0.0) {
            return;
          }
          fx += corridor_w * violation * violation;
          if (norm > 1.0e-6) {
            grad_points[point_index] += 2.0 * corridor_w * violation * diff / norm;
          }
        };

      for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        add_corridor_penalty(i, (*corridor)[i - 1U]);
        add_corridor_penalty(i, (*corridor)[i]);
      }
    }

    packGradient(grad_points, g_raw);
    return fx;
  };

  lbfgs::Parameters<double> params;
  params.max_iterations = options_.max_iterations;
  params.epsilon = options_.epsilon;
  params.max_linesearch = 60;
  params.linesearch = lbfgs::LineSearch::BacktrackingStrongWolfe;

  const auto lbfgs_result = lbfgs::minimize(x, evaluate, nullptr, params);
  result.path = unpackPath(x, guide_path);
  result.path_safe =
    map.pathIsFree(result.path, options_.validity_check_step, options_.extra_clearance);
  result.success = static_cast<bool>(lbfgs_result);

  std::ostringstream oss;
  oss << "L-BFGS status=" << static_cast<int>(lbfgs_result.status) << " ("
      << lbfgs::strerror(lbfgs_result.status) << ")"
      << ", fx=" << lbfgs_result.fx << ", iters=" << lbfgs_result.iterations
      << ", safe=" << (result.path_safe ? "true" : "false");
  result.message = oss.str();
  return result;
}

}  // namespace amprobo

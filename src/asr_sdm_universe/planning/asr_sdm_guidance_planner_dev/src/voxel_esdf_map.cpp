// Copyright (c) Amphibious Robotics.
// Voxel ESDF map implementation.

#include <voxel_esdf_map.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace amprobo
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}

void VoxelEsdfMap::reset(const VoxelMapOptions & options)
{
  options_ = options;
  options_.resolution = std::max(1.0e-3, options_.resolution);

  dims_.x() = std::max(1, static_cast<int>(std::ceil(options_.size.x() / options_.resolution)));
  dims_.y() = std::max(1, static_cast<int>(std::ceil(options_.size.y() / options_.resolution)));
  dims_.z() = std::max(1, static_cast<int>(std::ceil(options_.size.z() / options_.resolution)));
  voxel_count_ = dims_.x() * dims_.y() * dims_.z();

  occupied_.assign(voxel_count_, 0);
  distance_.assign(voxel_count_, kInf);

  occupied_count_ = 0;
  input_point_count_ = 0;
  esdf_input_point_count_ = 0;
  esdf_voxel_count_ = 0;
  initialized_ = true;
  has_occupancy_input_ = false;
  has_esdf_ = false;
}

void VoxelEsdfMap::clearOccupancy()
{
  std::fill(occupied_.begin(), occupied_.end(), 0);
  occupied_count_ = 0;
  input_point_count_ = 0;
  has_occupancy_input_ = false;
}

void VoxelEsdfMap::clearEsdf()
{
  std::fill(distance_.begin(), distance_.end(), kInf);
  esdf_input_point_count_ = 0;
  esdf_voxel_count_ = 0;
  has_esdf_ = false;
}

void VoxelEsdfMap::integrateOccupiedCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (!initialized_) {
    reset(options_);
  }

  if (options_.clear_before_integrate) {
    clearOccupancy();
  }

  input_point_count_ = static_cast<int>(cloud.points.size());

  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    GridIndex index;
    if (!worldToIndex(Eigen::Vector3d(point.x, point.y, point.z), index)) {
      continue;
    }

    const int address = toAddress(index);
    if (occupied_[address] == 0) {
      occupied_[address] = 1;
      ++occupied_count_;
    }
  }

  has_occupancy_input_ = true;
}

void VoxelEsdfMap::integrateEsdfCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  if (!initialized_) {
    reset(options_);
  }

  if (options_.clear_before_integrate) {
    clearEsdf();
  }

  esdf_input_point_count_ = static_cast<int>(cloud.points.size());

  for (const auto & point : cloud.points) {
    if (
      !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      !std::isfinite(point.intensity)) {
      continue;
    }

    GridIndex index;
    if (!worldToIndex(Eigen::Vector3d(point.x, point.y, point.z), index)) {
      continue;
    }

    const int address = toAddress(index);
    if (!std::isfinite(distance_[address])) {
      ++esdf_voxel_count_;
    }
    distance_[address] = static_cast<double>(point.intensity);
  }

  has_esdf_ = esdf_voxel_count_ > 0;
}

bool VoxelEsdfMap::isInMap(const Eigen::Vector3d & position) const
{
  return position.x() >= options_.origin.x() && position.y() >= options_.origin.y() &&
         position.z() >= options_.origin.z() &&
         position.x() < options_.origin.x() + options_.size.x() &&
         position.y() < options_.origin.y() + options_.size.y() &&
         position.z() < options_.origin.z() + options_.size.z();
}

bool VoxelEsdfMap::isInMap(const GridIndex & index) const
{
  return index.x >= 0 && index.x < dims_.x() && index.y >= 0 && index.y < dims_.y() &&
         index.z >= 0 && index.z < dims_.z();
}

bool VoxelEsdfMap::worldToIndex(const Eigen::Vector3d & position, GridIndex & index) const
{
  if (!isInMap(position)) {
    return false;
  }

  const Eigen::Vector3d rel = (position - options_.origin) / options_.resolution;
  index.x = static_cast<int>(std::floor(rel.x()));
  index.y = static_cast<int>(std::floor(rel.y()));
  index.z = static_cast<int>(std::floor(rel.z()));
  return isInMap(index);
}

Eigen::Vector3d VoxelEsdfMap::indexToWorld(const GridIndex & index) const
{
  return options_.origin + options_.resolution * (Eigen::Vector3d(index.x, index.y, index.z) +
                                                  Eigen::Vector3d::Constant(0.5));
}

int VoxelEsdfMap::toAddress(const GridIndex & index) const
{
  return index.x * dims_.y() * dims_.z() + index.y * dims_.z() + index.z;
}

bool VoxelEsdfMap::addressToIndex(int address, GridIndex & index) const
{
  if (address < 0 || address >= voxel_count_) {
    return false;
  }

  index.x = address / (dims_.y() * dims_.z());
  const int rem = address - index.x * dims_.y() * dims_.z();
  index.y = rem / dims_.z();
  index.z = rem - index.y * dims_.z();
  return true;
}

bool VoxelEsdfMap::isOccupied(const GridIndex & index) const
{
  if (!isInMap(index)) {
    return options_.unknown_as_occupied;
  }
  return occupied_[toAddress(index)] != 0;
}

bool VoxelEsdfMap::isOccupied(const Eigen::Vector3d & position) const
{
  GridIndex index;
  if (!worldToIndex(position, index)) {
    return options_.unknown_as_occupied;
  }
  return isOccupied(index);
}

bool VoxelEsdfMap::isFree(const GridIndex & index, double extra_clearance) const
{
  if (!isInMap(index)) {
    return false;
  }

  if (occupied_[toAddress(index)] != 0) {
    return false;
  }

  if (extra_clearance <= 1.0e-6) {
    return true;
  }

  const int radius = static_cast<int>(std::ceil(extra_clearance / options_.resolution));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (
          std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * options_.resolution >
          extra_clearance) {
          continue;
        }
        GridIndex near{index.x + dx, index.y + dy, index.z + dz};
        if (!isInMap(near)) {
          if (options_.unknown_as_occupied) {
            return false;
          }
          continue;
        }
        if (occupied_[toAddress(near)] != 0) {
          return false;
        }
      }
    }
  }

  return true;
}

bool VoxelEsdfMap::isFree(const Eigen::Vector3d & position, double extra_clearance) const
{
  GridIndex index;
  if (!worldToIndex(position, index)) {
    return false;
  }
  return isFree(index, extra_clearance);
}

bool VoxelEsdfMap::findNearestFree(
  const GridIndex & seed, int max_radius_voxels, GridIndex & free_index) const
{
  if (isFree(seed)) {
    free_index = seed;
    return true;
  }

  for (int radius = 1; radius <= max_radius_voxels; ++radius) {
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
            continue;
          }
          GridIndex candidate{seed.x + dx, seed.y + dy, seed.z + dz};
          if (isInMap(candidate) && isFree(candidate)) {
            free_index = candidate;
            return true;
          }
        }
      }
    }
  }

  return false;
}

bool VoxelEsdfMap::segmentIsFree(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step, double extra_clearance) const
{
  const double length = (b - a).norm();
  const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(step, 1.0e-3))));
  for (int i = 0; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector3d p = (1.0 - ratio) * a + ratio * b;
    if (!isFree(p, extra_clearance)) {
      return false;
    }
  }
  return true;
}

bool VoxelEsdfMap::pathIsFree(
  const std::vector<Eigen::Vector3d> & path, double step, double extra_clearance) const
{
  if (path.empty()) {
    return false;
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!segmentIsFree(path[i - 1], path[i], step, extra_clearance)) {
      return false;
    }
  }
  return true;
}

double VoxelEsdfMap::distance(const GridIndex & index) const
{
  if (!isInMap(index)) {
    return options_.unknown_as_occupied ? 0.0 : kInf;
  }
  return distance_[toAddress(index)];
}

double VoxelEsdfMap::distance(const Eigen::Vector3d & position) const
{
  GridIndex index;
  if (!worldToIndex(position, index)) {
    return options_.unknown_as_occupied ? 0.0 : kInf;
  }
  return distance(index);
}

Eigen::Vector3d VoxelEsdfMap::gradient(const Eigen::Vector3d & position) const
{
  GridIndex center;
  if (!worldToIndex(position, center)) {
    const Eigen::Vector3d map_center = options_.origin + 0.5 * options_.size;
    Eigen::Vector3d direction = map_center - position;
    if (direction.norm() < 1.0e-6) {
      return Eigen::Vector3d::Zero();
    }
    return direction.normalized();
  }

  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    GridIndex minus = center;
    GridIndex plus = center;
    if (axis == 0) {
      minus.x -= 1;
      plus.x += 1;
    } else if (axis == 1) {
      minus.y -= 1;
      plus.y += 1;
    } else {
      minus.z -= 1;
      plus.z += 1;
    }

    const double dm = distance(minus);
    const double dp = distance(plus);
    if (std::isfinite(dm) && std::isfinite(dp)) {
      grad(axis) = (dp - dm) / (2.0 * options_.resolution);
    }
  }

  if (grad.norm() < 1.0e-6) {
    return Eigen::Vector3d::Zero();
  }
  return grad.normalized();
}

}  // namespace amprobo

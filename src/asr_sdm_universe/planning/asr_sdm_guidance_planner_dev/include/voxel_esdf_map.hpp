// Copyright (c) Amphibious Robotics.
// Voxel ESDF map query interface.

#ifndef ASR_SDM_GUIDANCE_PLANNER_MAP_VOXEL_ESDF_MAP_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_MAP_VOXEL_ESDF_MAP_HPP_

#include <Eigen/Core>
#include <types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace amprobo
{

struct VoxelMapOptions
{
  double resolution = 0.15;
  Eigen::Vector3d origin = Eigen::Vector3d(-10.0, -10.0, -1.0);
  Eigen::Vector3d size = Eigen::Vector3d(20.0, 20.0, 5.0);
  bool unknown_as_occupied = false;
  bool clear_before_integrate = true;
};

class VoxelEsdfMap final : public MapQueryInterface
{
public:
  VoxelEsdfMap() = default;

  void reset(const VoxelMapOptions & options);
  void clearOccupancy();
  void clearEsdf();
  void integrateOccupiedCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud);
  void integrateEsdfCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud);

  bool isReady() const override { return initialized_ && has_occupancy_input_ && has_esdf_; }
  bool hasOccupancy() const { return initialized_ && has_occupancy_input_; }
  bool hasEsdf() const { return initialized_ && has_esdf_; }
  bool hasOccupancyLayer() const override { return hasOccupancy(); }
  bool hasDistanceField() const override { return hasEsdf(); }

  bool isInMap(const Eigen::Vector3d & position) const override;
  bool isInMap(const GridIndex & index) const override;
  bool worldToIndex(const Eigen::Vector3d & position, GridIndex & index) const override;
  Eigen::Vector3d indexToWorld(const GridIndex & index) const override;

  int toAddress(const GridIndex & index) const override;
  bool addressToIndex(int address, GridIndex & index) const override;

  bool isOccupied(const GridIndex & index) const override;
  bool isOccupied(const Eigen::Vector3d & position) const override;
  bool isFree(const GridIndex & index, double extra_clearance = 0.0) const override;
  bool isFree(const Eigen::Vector3d & position, double extra_clearance = 0.0) const override;
  bool findNearestFree(
    const GridIndex & seed, int max_radius_voxels, GridIndex & free_index) const override;

  double distance(const GridIndex & index) const override;
  double distance(const Eigen::Vector3d & position) const override;
  Eigen::Vector3d gradient(const Eigen::Vector3d & position) const override;

  bool segmentIsFree(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step,
    double extra_clearance) const override;
  bool pathIsFree(
    const std::vector<Eigen::Vector3d> & path, double step, double extra_clearance) const override;

  const Eigen::Vector3i & dims() const { return dims_; }
  const Eigen::Vector3d & origin() const override { return options_.origin; }
  const Eigen::Vector3d & size() const override { return options_.size; }
  double resolution() const override { return options_.resolution; }
  int voxelCount() const override { return voxel_count_; }
  int occupiedCount() const { return occupied_count_; }
  int inputPointCount() const { return input_point_count_; }
  int esdfInputPointCount() const { return esdf_input_point_count_; }
  int esdfVoxelCount() const { return esdf_voxel_count_; }

private:
  VoxelMapOptions options_;
  Eigen::Vector3i dims_ = Eigen::Vector3i::Zero();
  int voxel_count_ = 0;
  int occupied_count_ = 0;
  int input_point_count_ = 0;
  int esdf_input_point_count_ = 0;
  int esdf_voxel_count_ = 0;
  bool initialized_ = false;
  bool has_occupancy_input_ = false;
  bool has_esdf_ = false;

  std::vector<unsigned char> occupied_;
  std::vector<double> distance_;
};

}  // namespace amprobo

#endif  // ASR_SDM_GUIDANCE_PLANNER_MAP_VOXEL_ESDF_MAP_HPP_

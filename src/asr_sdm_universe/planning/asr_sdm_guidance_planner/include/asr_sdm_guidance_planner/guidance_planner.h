// Copyright (c) Amphibious Robotics.
// Guidance path planner interface.

#ifndef _GUIDANCE_PLANNER_H
#define _GUIDANCE_PLANNER_H

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_esdf_map/edt_environment.hpp"

#include <memory>
#include <vector>

namespace amprobo
{

class GuidancePlanner
{
private:
  /* ---------- record data ---------- */
  EDTEnvironment::Ptr edt_environment_;
  std::shared_ptr<rclcpp::Node> node_;
  bool has_path_ = false;
  std::vector<Eigen::Vector3d> path_;
  std::vector<double> yaw_path_;
  std::vector<double> pitch_path_;
  double path_length_ = 0.0;
  double position_error_ = 0.0;

  /* ---------- parameter ---------- */
  double yaw_radius_;
  double pitch_radius_;
  double sample_ds_;
  double margin_;
  double position_tol_;
  double tight_turn_radius_;
  int location_samples_;
  int heading_samples_;
  int num_threads_;

  /* map */
  Eigen::Vector3d origin_, map_size_3d_, map_max_;
  bool has_map_ = false;

  /* helper */
  bool headingFromDelta(const Eigen::Vector3d & delta, double & yaw, double & pitch) const;
  bool inMap(const Eigen::Vector3d & pt) const;
  bool collisionFree(const Eigen::Vector3d & pt) const;

public:
  GuidancePlanner() {};
  ~GuidancePlanner() {};

  enum { REACH_END = 1, NO_PATH = 2 };

  /* main API */
  void setParam(const std::shared_ptr<rclcpp::Node> & nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
  int search(
    Eigen::Vector3d start_pt, double start_yaw, double start_pitch, Eigen::Vector3d end_pt,
    double end_yaw, double end_pitch);

  void setEnvironment(const EDTEnvironment::Ptr & env);
  std::vector<Eigen::Vector3d> getPath();
  std::vector<double> getYawPath();
  std::vector<double> getPitchPath();
  double getPathLength() const { return path_length_; }
  double getPositionError() const { return position_error_; }

  typedef std::shared_ptr<GuidancePlanner> Ptr;
};

}  // namespace amprobo

#endif

// Copyright (c) Amphibious Robotics.
// Guidance path planner implementation.

#include <asr_sdm_guidance_planner/guidance_planner.h>

#include <dubins_path_3d/planner.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace std;
using namespace Eigen;

namespace amprobo
{
namespace
{

// ASR heading is R = Rz(yaw) * Ry(pitch), so body +x has z = -sin(pitch).
// dubins_path_3d treats pitch as positive nose-up (tangent.z = +sin(pitch)).
dubins_path_3d::Configuration poseToConfig(
  const Eigen::Vector3d & pt, double yaw, double pitch)
{
  return dubins_path_3d::configurationFromEuler(pt, yaw, -pitch, 0.0);
}

}  // namespace

void GuidancePlanner::setParam(const std::shared_ptr<rclcpp::Node> & nh)
{
  node_ = nh;
  const std::string p = "guidance_planner.dubins_path_3d.";
  node_->declare_parameter(p + "yaw_radius", 1.0);
  node_->declare_parameter(p + "pitch_radius", 1.0);
  node_->declare_parameter(p + "sample_ds", 0.1);
  node_->declare_parameter(p + "margin", 0.2);
  node_->declare_parameter(p + "position_tol", 0.05);
  node_->declare_parameter(p + "tight_turn_radius", -1.0);
  node_->declare_parameter(p + "location_samples", 15);
  node_->declare_parameter(p + "heading_samples", 15);
  node_->declare_parameter(p + "num_threads", 0);
  yaw_radius_ = node_->get_parameter(p + "yaw_radius").as_double();
  pitch_radius_ = node_->get_parameter(p + "pitch_radius").as_double();
  sample_ds_ = node_->get_parameter(p + "sample_ds").as_double();
  margin_ = node_->get_parameter(p + "margin").as_double();
  position_tol_ = node_->get_parameter(p + "position_tol").as_double();
  tight_turn_radius_ = node_->get_parameter(p + "tight_turn_radius").as_double();
  location_samples_ = static_cast<int>(node_->get_parameter(p + "location_samples").as_int());
  heading_samples_ = static_cast<int>(node_->get_parameter(p + "heading_samples").as_int());
  num_threads_ = static_cast<int>(node_->get_parameter(p + "num_threads").as_int());

  cout << "3d dubins yaw radius:" << yaw_radius_ << endl;
  cout << "3d dubins pitch radius:" << pitch_radius_ << endl;
}

void GuidancePlanner::init()
{
  yaw_radius_ = std::max(yaw_radius_, 1.0e-6);
  pitch_radius_ = std::max(pitch_radius_, 1.0e-6);
  sample_ds_ = std::max(sample_ds_, 1.0e-3);
  location_samples_ = std::max(location_samples_, 1);
  heading_samples_ = std::max(heading_samples_, 1);

  if (edt_environment_) {
    edt_environment_->getMapRegion(origin_, map_size_3d_);
    map_max_ = origin_ + map_size_3d_;
    has_map_ = true;
    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;
    cout << "map max: " << map_max_.transpose() << endl;
  }
}

void GuidancePlanner::setEnvironment(const EDTEnvironment::Ptr & env)
{
  this->edt_environment_ = env;
}

void GuidancePlanner::reset()
{
  path_.clear();
  yaw_path_.clear();
  pitch_path_.clear();
  path_length_ = 0.0;
  position_error_ = 0.0;
  has_path_ = false;
}

int GuidancePlanner::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
  double start_yaw = 0.0;
  double start_pitch = 0.0;
  headingFromDelta(end_pt - start_pt, start_yaw, start_pitch);
  return search(start_pt, start_yaw, start_pitch, end_pt, start_yaw, start_pitch);
}

int GuidancePlanner::search(
  Eigen::Vector3d start_pt, double start_yaw, double start_pitch, Eigen::Vector3d end_pt,
  double end_yaw, double end_pitch)
{
  reset();

  dubins_path_3d::PlannerOptions options;
  options.yaw_radius = yaw_radius_;
  options.pitch_radius = pitch_radius_;
  options.sample_spacing = sample_ds_;
  options.tight_turn_radius = tight_turn_radius_;
  options.location_samples = location_samples_;
  options.heading_samples = heading_samples_;
  options.num_threads = num_threads_;

  dubins_path_3d::PlanningResult result;
  try {
    const dubins_path_3d::DubinsPath3D planner(options);
    result = planner.plan(poseToConfig(start_pt, start_yaw, start_pitch),
                          poseToConfig(end_pt, end_yaw, end_pitch));
  } catch (const std::exception & e) {
    cout << "3d dubins: " << e.what() << endl;
    return NO_PATH;
  }

  path_length_ = result.best.length;
  if (!result.success()) {
    cout << "3d dubins: no feasible curve" << endl;
    return NO_PATH;
  }

  const auto & samples = result.best.samples;
  position_error_ = (samples.back().position - end_pt).norm();
  if (position_error_ > std::max(position_tol_, sample_ds_)) {
    cout << "3d dubins: endpoint error " << position_error_ << endl;
    return NO_PATH;
  }

  path_.reserve(samples.size());
  yaw_path_.reserve(samples.size());
  pitch_path_.reserve(samples.size());

  for (const auto & sample : samples) {
    if (!collisionFree(sample.position)) {
      cout << "3d dubins: collision on sampled curve" << endl;
      reset();
      return NO_PATH;
    }
    double yaw = 0.0;
    double pitch = 0.0;
    headingFromDelta(sample.tangent, yaw, pitch);
    path_.push_back(sample.position);
    yaw_path_.push_back(yaw);
    pitch_path_.push_back(pitch);
  }

  has_path_ = true;
  return REACH_END;
}

std::vector<Eigen::Vector3d> GuidancePlanner::getPath()
{
  return path_;
}

std::vector<double> GuidancePlanner::getYawPath()
{
  return yaw_path_;
}

std::vector<double> GuidancePlanner::getPitchPath()
{
  return pitch_path_;
}

bool GuidancePlanner::headingFromDelta(
  const Eigen::Vector3d & delta, double & yaw, double & pitch) const
{
  const double dist_xy = std::hypot(delta.x(), delta.y());
  const double dist = delta.norm();
  if (dist < 1.0e-6) {
    yaw = 0.0;
    pitch = 0.0;
    return false;
  }
  yaw = std::atan2(delta.y(), delta.x());
  pitch = std::atan2(-delta.z(), std::max(dist_xy, 1.0e-9));
  return true;
}

bool GuidancePlanner::inMap(const Eigen::Vector3d & pt) const
{
  if (!has_map_) {
    return true;
  }
  return (pt.array() > origin_.array()).all() && (pt.array() < map_max_.array()).all();
}

bool GuidancePlanner::collisionFree(const Eigen::Vector3d & pt) const
{
  if (!inMap(pt)) {
    return false;
  }
  if (!edt_environment_) {
    return true;
  }
  Eigen::Vector3d query = pt;
  return edt_environment_->evaluateCoarseEDT(query, -1.0) > margin_;
}

}  // namespace amprobo

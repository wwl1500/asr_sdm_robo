// Copyright (c) Amphibious Robotics.
// B-spline trajectory optimizer implementation.

#include "asr_sdm_trajectory_optimizer/bspline_optimizer.h"

#include <asr_sdm_lbfgs_solver/lbfgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
// using namespace std;

namespace amprobo
{

const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT = (1 << 3);
const int BsplineOptimizer::GUIDE = (1 << 4);
const int BsplineOptimizer::WAYPOINTS = (1 << 6);
const int BsplineOptimizer::NONHOLONOMIC = (1 << 7);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::GUIDE_NONHOLONOMIC_PHASE =
  BsplineOptimizer::GUIDE_PHASE | BsplineOptimizer::NONHOLONOMIC;
const int BsplineOptimizer::NORMAL_PHASE =
  BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;
const int BsplineOptimizer::NONHOLONOMIC_PHASE =
  BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::NONHOLONOMIC;

namespace
{

double declareGetDouble(
  const std::shared_ptr<rclcpp::Node> & nh, const std::string & name, double default_value)
{
  if (!nh->has_parameter(name)) nh->declare_parameter(name, default_value);
  return nh->get_parameter(name).as_double();
}

int declareGetInt(
  const std::shared_ptr<rclcpp::Node> & nh, const std::string & name, int default_value)
{
  if (!nh->has_parameter(name)) nh->declare_parameter(name, default_value);
  return nh->get_parameter(name).as_int();
}

/* Speed below which the trajectory tangent, and therefore the heading of a
 * nonholonomic robot, carries no usable direction information. This is kept
 * well above machine epsilon so the atan2 Jacobians stay bounded. */
constexpr double kTangentEps = 1.0e-4;
constexpr double kMinHeadingSpeed = 0.02;

double wrapToPi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

enum class HeadingAxis { kYaw, kPitch };

/* Heading of a robot whose body x-axis follows its velocity, together with the
 * Jacobian of that heading with respect to the velocity. The convention matches
 * R = Rz(yaw) * Ry(pitch) used by asr_sdm_control_manager, so a positive pitch
 * points the body axis downwards. */
struct Heading
{
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Vector3d dyaw_dv = Eigen::Vector3d::Zero();
  Eigen::Vector3d dpitch_dv = Eigen::Vector3d::Zero();
  bool valid = false;

  double angle(HeadingAxis axis) const { return axis == HeadingAxis::kYaw ? yaw : pitch; }

  const Eigen::Vector3d & jacobian(HeadingAxis axis) const
  {
    return axis == HeadingAxis::kYaw ? dyaw_dv : dpitch_dv;
  }
};

/* One-sided hinges on a quantity normalized by its own limit. The quadratic is
 * mapped through x^2 / (1 + x^2) so a large initial violation saturates instead
 * of producing ruinous solver steps that scribble the control polygon. */
bool overLimitPenalty(double value, double limit, double & cost, double & dcost_dvalue)
{
  const double magnitude = std::fabs(value);
  if (limit <= 0.0 || magnitude <= limit) return false;

  const double excess = magnitude / limit - 1.0;
  const double e2 = excess * excess;
  const double den = 1.0 + e2;
  cost = e2 / den;
  dcost_dvalue = 2.0 * excess / (den * den * limit) * (value < 0.0 ? -1.0 : 1.0);

  return true;
}

bool underLimitPenalty(double value, double limit, double & cost, double & dcost_dvalue)
{
  if (limit <= 0.0 || value >= limit || value < kTangentEps) return false;

  const double deficit = 1.0 - value / limit;
  const double e2 = deficit * deficit;
  const double den = 1.0 + e2;
  cost = e2 / den;
  dcost_dvalue = -2.0 * deficit / (den * den * limit);

  return true;
}

Heading headingFromVelocity(const Eigen::Vector3d & v)
{
  Heading h;

  const double sxy2 = v(0) * v(0) + v(1) * v(1);
  const double v2 = sxy2 + v(2) * v(2);
  const double min_speed2 = kMinHeadingSpeed * kMinHeadingSpeed;
  if (sxy2 < min_speed2 || v2 < min_speed2) return h;

  const double sxy = std::sqrt(sxy2);
  h.yaw = std::atan2(v(1), v(0));
  h.pitch = std::atan2(-v(2), sxy);
  h.dyaw_dv << -v(1) / sxy2, v(0) / sxy2, 0.0;
  h.dpitch_dv << v(2) * v(0) / (sxy * v2), v(2) * v(1) / (sxy * v2), -sxy / v2;
  h.valid = true;

  return h;
}

/* Yaw and pitch are two angles of the same tangent direction and the robot
 * bounds the rate of both, so they are evaluated by one implementation that only
 * differs in which angle it reads out of the heading. Yaw needs wrapping because
 * it is periodic; pitch stays inside (-pi/2, pi/2), where wrapping is a no-op.
 *
 * A rate spans two consecutive velocity control points, v_i = (q_{i+1} - q_i)/dt,
 * so each penalty couples three consecutive position control points. */
void headingRateCost(
  const vector<Eigen::Vector3d> & q, double ts, HeadingAxis axis, double max_rate, double & cost,
  vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  const int vel_num = static_cast<int>(q.size()) - 1;

  for (int i = 0; i + 1 < vel_num; i++) {
    const Heading h0 = headingFromVelocity((q[i + 1] - q[i]) / ts);
    const Heading h1 = headingFromVelocity((q[i + 2] - q[i + 1]) / ts);
    if (!h0.valid || !h1.valid) continue;

    const double rate = wrapToPi(h1.angle(axis) - h0.angle(axis)) / ts;

    double term = 0.0, dterm_drate = 0.0;
    if (!overLimitPenalty(rate, max_rate, term, dterm_drate)) continue;

    cost += term;

    const double dcost_dangle = dterm_drate / ts;
    const Eigen::Vector3d g1 = dcost_dangle * h1.jacobian(axis) / ts;
    const Eigen::Vector3d g0 = -dcost_dangle * h0.jacobian(axis) / ts;

    gradient[i + 2] += g1;
    gradient[i + 1] += g0 - g1;
    gradient[i + 0] += -g0;
  }
}

}  // namespace

void BsplineOptimizer::setParam(const std::shared_ptr<rclcpp::Node> & nh)
{
  lambda1_ = declareGetDouble(nh, "optimization.lambda1", -1.0);
  lambda2_ = declareGetDouble(nh, "optimization.lambda2", -1.0);
  lambda3_ = declareGetDouble(nh, "optimization.lambda3", -1.0);
  lambda4_ = declareGetDouble(nh, "optimization.lambda4", -1.0);
  lambda5_ = declareGetDouble(nh, "optimization.lambda5", -1.0);
  lambda6_ = declareGetDouble(nh, "optimization.lambda6", -1.0);
  lambda7_ = declareGetDouble(nh, "optimization.lambda7", -1.0);
  lambda8_ = declareGetDouble(nh, "optimization.lambda8", -1.0);
  lambda_yaw_rate_ = declareGetDouble(nh, "optimization.lambda_yaw_rate", 0.0);
  lambda_pitch_rate_ = declareGetDouble(nh, "optimization.lambda_pitch_rate", 0.0);
  lambda_min_vel_ = declareGetDouble(nh, "optimization.lambda_min_vel", 0.0);

  dist0_ = declareGetDouble(nh, "optimization.dist0", -1.0);
  max_vel_ = declareGetDouble(nh, "manager.max_vel", -1.0);
  max_acc_ = declareGetDouble(nh, "manager.max_acc", -1.0);
  max_yaw_rate_ = declareGetDouble(nh, "manager.max_yaw_rate", 0.35);
  max_pitch_rate_ = declareGetDouble(nh, "manager.max_pitch_rate", 0.35);
  min_vel_ = declareGetDouble(nh, "manager.min_vel", 0.0);
  visib_min_ = declareGetDouble(nh, "optimization.visib_min", -1.0);
  dlmin_ = declareGetDouble(nh, "optimization.dlmin", -1.0);
  wnl_ = declareGetDouble(nh, "optimization.wnl", -1.0);

  max_iteration_num_[0] = declareGetInt(nh, "optimization.max_iteration_num1", -1);
  max_iteration_num_[1] = declareGetInt(nh, "optimization.max_iteration_num2", -1);
  max_iteration_num_[2] = declareGetInt(nh, "optimization.max_iteration_num3", -1);
  max_iteration_num_[3] = declareGetInt(nh, "optimization.max_iteration_num4", -1);
  max_iteration_time_[0] = declareGetDouble(nh, "optimization.max_iteration_time1", -1.0);
  max_iteration_time_[1] = declareGetDouble(nh, "optimization.max_iteration_time2", -1.0);
  max_iteration_time_[2] = declareGetDouble(nh, "optimization.max_iteration_time3", -1.0);
  max_iteration_time_[3] = declareGetDouble(nh, "optimization.max_iteration_time4", -1.0);

  order_ = declareGetInt(nh, "optimization.order", -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr & env)
{
  this->edt_environment_ = env;
}

void BsplineOptimizer::setMapQuery(const MapQueryInterface * map)
{
  map_query_holder_.reset();
  map_query_ = map;
}

void BsplineOptimizer::setMapQuery(
  const std::shared_ptr<const MapQueryInterface> & map)
{
  map_query_holder_ = map;
  map_query_ = map_query_holder_.get();
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd & points)
{
  control_points_ = points;
  dim_ = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double & ts)
{
  bspline_interval_ = ts;
}

void BsplineOptimizer::setTerminateCond(const int & max_num_id, const int & max_time_id)
{
  max_num_id_ = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int & cost_code)
{
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
  if (cost_function_ & NONHOLONOMIC) cost_str += " nonho |";

  RCLCPP_INFO(
    rclcpp::get_logger("asr_sdm_trajectory_optimizerimizer"), "cost func: %s", cost_str.c_str());
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d> & guide_pt)
{
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(
  const vector<Eigen::Vector3d> & waypts, const vector<int> & waypt_idx)
{
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(
  const Eigen::MatrixXd & points, const double & ts, const int & cost_function, int max_num_id,
  int max_time_id)
{
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize()
{
  /* initialize solver */
  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);
  g_yaw_rate_.resize(pt_num);
  g_pitch_rate_.resize(pt_num);
  g_min_vel_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) * (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
                           control_points_.row(pt_num - 1));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_));
  }

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  /* Seed the result with the initial guess so an aborted solve leaves the
   * control points untouched rather than reading a stale best_variable_. */
  best_variable_ = q;
  if (variable_num_ <= 0) return;

  /* do optimization using the L-BFGS solver */
  opt_start_time_ = std::chrono::steady_clock::now();

  lbfgs::Parameters<double> params;
  params.epsilon = 1.0e-5;
  /* Stop once the objective stagnates over three iterations, the closest
   * analogue of the relative step tolerance the previous solver used. */
  params.past = 3;
  params.delta = 1.0e-5;
  params.max_iterations = max(0, max_iteration_num_[max_num_id_]);

  const lbfgs::Result<double> result = lbfgs::minimize<double>(
    variable_num_, q.data(),
    [this](const double * x, double * grad, int n, double) { return evaluateCost(x, grad, n); },
    [this](const lbfgs::Progress<double> &) { return budgetExhausted() ? 1 : 0; }, params);

  /* Running out of iterations, line-search steps or wall-clock time is the
   * normal way a real-time solve ends, so only genuine failures are reported. */
  switch (result.status) {
    case lbfgs::Status::Canceled:
    case lbfgs::Status::MaximumIteration:
    case lbfgs::Status::MaximumLineSearch:
      break;
    default:
      if (!result.ok()) {
        RCLCPP_WARN(
          rclcpp::get_logger("asr_sdm_trajectory_optimizerimizer"), "[Optimization]: lbfgs: %s",
          std::string(lbfgs::strerror(result.status)).c_str());
      }
      break;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE))
    RCLCPP_INFO(
      rclcpp::get_logger("asr_sdm_trajectory_optimizerimizer"), "iter num: %d", iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    bool has_distance = false;
    dist = std::numeric_limits<double>::infinity();
    dist_grad = Eigen::Vector3d::Zero();

    if (map_query_ != nullptr) {
      if (!map_query_->isReady() || !map_query_->isInMap(q[i]) || !map_query_->hasDistanceField()) {
        continue;
      }
      dist = map_query_->distance(q[i]);
      dist_grad = map_query_->gradient(q[i]);
      has_distance = std::isfinite(dist);
    } else if (edt_environment_ != nullptr) {
      edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
      has_distance = std::isfinite(dist);
    }

    if (!has_distance) {
      continue;
    }

    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    // The same optimizer instance serves splines of different lengths, so a set
    // of waypoints left over from an earlier solve may index past this one.
    if (idx < 0 || idx + 2 >= static_cast<int>(q.size())) continue;

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

/* The heading of a nonholonomic robot cannot be chosen freely: its body x-axis
 * is the trajectory tangent. The costs below therefore express the yaw and pitch
 * rate limits of the robot directly on the position control points.
 *
 * Heading angles are evaluated on the velocity control points of the position
 * B-spline, v_i = (q_{i+1} - q_i) / dt, which is the same discretization the
 * velocity feasibility cost uses. */
void BsplineOptimizer::calcYawRateCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  headingRateCost(q, bspline_interval_, HeadingAxis::kYaw, max_yaw_rate_, cost, gradient);
}

void BsplineOptimizer::calcPitchRateCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  headingRateCost(q, bspline_interval_, HeadingAxis::kPitch, max_pitch_rate_, cost, gradient);
}

/* A screw-driven robot cannot turn on the spot, and a vanishing velocity leaves
 * the commanded heading undefined. Keeping the forward speed above min_vel_
 * keeps the tangent, and therefore the heading, well posed. */
void BsplineOptimizer::calcForwardSpeedCost(
  const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient)
{
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  const double ts = bspline_interval_;
  const int vel_num = static_cast<int>(q.size()) - 1;
  const int first = order_;
  const int last = vel_num - order_;

  for (int i = first; i < last; i++) {
    const Eigen::Vector3d v = (q[i + 1] - q[i]) / ts;
    const double speed = v.norm();

    double term = 0.0, dterm_dspeed = 0.0;
    if (!underLimitPenalty(speed, min_vel_, term, dterm_dspeed)) continue;

    cost += term;

    const Eigen::Vector3d g = (dterm_dspeed / (speed * ts)) * v;
    gradient[i + 1] += g;
    gradient[i + 0] += -g;
  }
}

bool BsplineOptimizer::useNonholonomicCost() const
{
  // Heading is only meaningful for a 3D position spline; the 1D heading splines
  // reuse this solver with a quadratic cost and must not enter here.
  return (cost_function_ & NONHOLONOMIC) && dim_ == 3;
}

void BsplineOptimizer::combineCost(
  const std::vector<double> & x, std::vector<double> & grad, double & f_combine)
{
  /* convert the flat solver vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control
  // point For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {
      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
          control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;
  double f_yaw_rate, f_pitch_rate, f_min_vel;
  f_yaw_rate = f_pitch_rate = f_min_vel = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  if (useNonholonomicCost()) {
    calcYawRateCost(g_q_, f_yaw_rate, g_yaw_rate_);
    f_combine += lambda_yaw_rate_ * f_yaw_rate;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda_yaw_rate_ * g_yaw_rate_[i + order_](j);

    calcPitchRateCost(g_q_, f_pitch_rate, g_pitch_rate_);
    f_combine += lambda_pitch_rate_ * f_pitch_rate;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda_pitch_rate_ * g_pitch_rate_[i + order_](j);

    calcForwardSpeedCost(g_q_, f_min_vel, g_min_vel_);
    f_combine += lambda_min_vel_ * f_min_vel;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda_min_vel_ * g_min_vel_[i + order_](j);
  }
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::evaluateCost(const double * x, double * grad, int n)
{
  double cost = 0.0;
  std::vector<double> x_vec(x, x + n);
  std::vector<double> grad_vec;
  combineCost(x_vec, grad_vec, cost);
  if (grad) {
    for (int i = 0; i < n; ++i) grad[i] = grad_vec[i];
  }
  iter_num_++;

  /* save the min cost result */
  if (cost < min_cost_) {
    min_cost_ = cost;
    best_variable_.assign(x, x + n);
  }
  return cost;
}

bool BsplineOptimizer::budgetExhausted() const
{
  const int max_eval = max_iteration_num_[max_num_id_];
  if (max_eval > 0 && iter_num_ >= max_eval) return true;

  const double max_time = max_iteration_time_[max_time_id_];
  if (max_time > 0.0) {
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - opt_start_time_;
    if (elapsed.count() >= max_time) return true;
  }

  return false;
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd & ctrl_pts)
{
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints()
{
  return this->control_points_;
}

}  // namespace amprobo
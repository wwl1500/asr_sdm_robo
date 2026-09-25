// Copyright (c) Amphibious Robotics.
// B-spline trajectory optimizer interface.

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <asr_sdm_esdf_map/edt_environment.hpp>
#include <map_query_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace amprobo
{
class BsplineOptimizer
{
public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;
  static const int NONHOLONOMIC;

  static const int GUIDE_PHASE;
  static const int GUIDE_NONHOLONOMIC_PHASE;
  static const int NORMAL_PHASE;
  static const int NONHOLONOMIC_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /* main API */
  void setEnvironment(const EDTEnvironment::Ptr & env);
  void setMapQuery(const MapQueryInterface * map);
  void setMapQuery(const std::shared_ptr<const MapQueryInterface> & map);
  void setParam(const std::shared_ptr<rclcpp::Node> & nh);
  Eigen::MatrixXd BsplineOptimizeTraj(
    const Eigen::MatrixXd & points, const double & ts, const int & cost_function, int max_num_id,
    int max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd & points);
  void setBsplineInterval(const double & ts);
  void setCostFunction(const int & cost_function);
  void setTerminateCond(const int & max_num_id, const int & max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector3d> & guide_pt);
  // Positions the trajectory should pass through, one index per position: the
  // index selects the knot, so waypts[i] is the position at knot waypt_idx[i].
  void setWaypoints(
    const vector<Eigen::Vector3d> & waypts,
    const vector<int> & waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd & ctrl_pts);

private:
  EDTEnvironment::Ptr edt_environment_;
  const MapQueryInterface * map_query_ = nullptr;
  std::shared_ptr<const MapQueryInterface> map_query_holder_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double bspline_interval_;            // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int dim_;                            // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int> waypt_idx_;              // waypts constraints index
                                       //
  int max_num_id_, max_time_id_;       // stopping criteria
  int cost_function_;                  // used to determine objective function
  bool dynamic_;                       // moving obstacles ?
  double start_time_;                  // global time for moving obstacles

  /* optimization parameters */
  int order_;                     // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // acc smoothness
  double lambda_yaw_rate_;        // yaw rate feasibility weight
  double lambda_pitch_rate_;      // pitch rate feasibility weight
  double lambda_min_vel_;         // minimum forward speed weight
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double max_yaw_rate_;           // heading rate limit [rad/s]
  double max_pitch_rate_;         // pitch rate limit [rad/s]
  double min_vel_;                // forward speed below which the tangent is ill-defined
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int max_iteration_num_[4];      // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_waypoints_;
  vector<Eigen::Vector3d> g_yaw_rate_;
  vector<Eigen::Vector3d> g_pitch_rate_;
  vector<Eigen::Vector3d> g_min_vel_;

  int variable_num_;                   // optimization variables
  int iter_num_;                       // cost evaluations performed by the solver
  std::vector<double> best_variable_;  //
  double min_cost_;                    //

  // Wall-clock origin of the running solve, used to honour max_iteration_time_.
  std::chrono::steady_clock::time_point opt_start_time_;

  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  /* Objective handed to the L-BFGS solver: fills grad with the gradient at x and
   * returns the cost. Also tracks the best iterate seen so far. */
  double evaluateCost(const double * x, double * grad, int n);

  /* True once the evaluation or wall-clock budget of the current solve is spent.
   * The L-BFGS solver only offers cancellation once per iteration, so the
   * budgets are honoured at iteration granularity. */
  bool budgetExhausted() const;

  void combineCost(const std::vector<double> & x, vector<double> & grad, double & cost);

  // q contains all control points
  void calcSmoothnessCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcDistanceCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcFeasibilityCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcEndpointCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcGuideCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcVisibilityCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcWaypointsCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcViewCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);

  /* nonholonomic heading costs: the body axis of the robot is slaved to the
   * trajectory tangent, so yaw and pitch are functions of the position control
   * points and their limits become constraints on the position trajectory.
   * The two rate costs are evaluated equivalently, differing only in which
   * angle of the tangent they bound. */
  void calcYawRateCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcPitchRateCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);
  void calcForwardSpeedCost(
    const vector<Eigen::Vector3d> & q, double & cost, vector<Eigen::Vector3d> & gradient);

  bool useNonholonomicCost() const;

  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  rclcpp::Time time_start_;

  void getCostCurve(vector<double> & cost, vector<double> & time)
  {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace amprobo
#endif
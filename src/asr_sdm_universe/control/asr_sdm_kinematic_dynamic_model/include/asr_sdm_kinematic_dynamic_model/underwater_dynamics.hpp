#pragma once

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <memory>
#include <string>
#include <vector>

namespace asr_sdm_kinematic_dynamic_model
{

struct HydrodynamicParameters {
  double rho = 1000.0;     // Fluid density (kg/m^3)
  double Cd_n = 1.0;       // Normal drag coefficient
  double Cd_t = 0.05;      // Tangential drag coefficient

  // Ordered list of body-frame names used for hydrodynamic assembly.
  // When empty, the implementation falls back to the nominal robot body links.
  std::vector<std::string> hydrodynamic_link_names;
  
  // Basic geometry for cylinder links
  std::vector<double> link_volumes;
  std::vector<double> link_radii;
  std::vector<double> link_lengths;
  
  // Added mass factors (typically 1.0 for cylinder cross-section)
  std::vector<double> added_mass_factors;

  // Optional exact diagonal matrices, flattened as [link0_6d, link1_6d, ...].
  std::vector<double> link_added_mass_diagonal;
  std::vector<double> link_linear_damping_diagonal;
};

class UnderwaterDynamics
{
public:
  UnderwaterDynamics(std::shared_ptr<pinocchio::Model> model,
                     std::shared_ptr<pinocchio::Data> data,
                     const HydrodynamicParameters& params);

  ~UnderwaterDynamics() = default;

  // Compute all hydrodynamic matrices for the given state (q, v)
  // These modify internal states.
  void computeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  // Getters for the system-level matrices
  const Eigen::MatrixXd& getMassMatrix() const { return M_; }
  const Eigen::MatrixXd& getCoriolisMatrix() const { return C_; }
  const Eigen::MatrixXd& getDampingMatrix() const { return D_; }
  const Eigen::VectorXd& getRestoringForces() const { return N_; }

private:
  struct HydrodynamicLink
  {
    std::string name;
    pinocchio::FrameIndex frame_id;
  };

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  HydrodynamicParameters params_;
  std::vector<HydrodynamicLink> hydrodynamic_links_;

  // System-level matrices
  Eigen::MatrixXd M_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd D_;
  Eigen::VectorXd N_;

  void initializeHydrodynamicLinks();
  void validateParameters() const;

  static bool isNominalHydrodynamicLinkName(const std::string& name);
  static int hydrodynamicLinkSortKey(const std::string& name);

  Eigen::Matrix<double, 6, 6> buildAddedMassMatrix(size_t link_index) const;
  Eigen::Matrix<double, 6, 6> buildLinearDampingMatrix(size_t link_index) const;
  Eigen::Matrix<double, 6, 6> buildNonlinearDampingMatrix(
    size_t link_index, const Eigen::Matrix<double, 6, 1>& local_velocity) const;

  void computeAddedMass();
  void computeHydrodynamicDamping();
  void computeBuoyancyAndGravity();
  void computeCoriolis();
};

} // namespace asr_sdm_kinematic_dynamic_model

#pragma once

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <memory>
#include <string>
#include <vector>

namespace asr_sdm_kinematic_dynamic_model
{

struct HydrodynamicParameters
{
  double rho = 1000.0;
  double Cd_n = 1.0;
  double Cd_t = 0.05;

  std::vector<std::string> hydrodynamic_link_names;
  std::vector<double> link_volumes;
  std::vector<double> link_radii;
  std::vector<double> link_lengths;
  std::vector<double> added_mass_factors;
  std::vector<double> link_added_mass_diagonal;
  std::vector<double> link_linear_damping_diagonal;
};

class UnderwaterDynamics
{
public:
  UnderwaterDynamics(
    std::shared_ptr<pinocchio::Model> model,
    std::shared_ptr<pinocchio::Data> data,
    const HydrodynamicParameters & params);

  ~UnderwaterDynamics() = default;

  void computeDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & v);

  const Eigen::MatrixXd & getMassMatrix() const {return M_;}
  const Eigen::MatrixXd & getCoriolisMatrix() const {return C_;}
  const Eigen::MatrixXd & getDampingMatrix() const {return D_;}
  const Eigen::VectorXd & getRestoringForces() const {return N_;}

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

  Eigen::MatrixXd M_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd D_;
  Eigen::VectorXd N_;

  void initializeHydrodynamicLinks();
  void validateParameters() const;

  static bool isNominalHydrodynamicLinkName(const std::string & name);
  static int hydrodynamicLinkSortKey(const std::string & name);

  Eigen::Matrix<double, 6, 6> buildAddedMassMatrix(size_t link_index) const;
  Eigen::Matrix<double, 6, 6> buildLinearDampingMatrix(size_t link_index) const;
  Eigen::Matrix<double, 6, 6> buildNonlinearDampingMatrix(
    size_t link_index,
    const Eigen::Matrix<double, 6, 1> & local_velocity) const;

  void computeAddedMass();
  void computeHydrodynamicDamping();
  void computeBuoyancyAndGravity();
  void computeCoriolis();
};

}  // namespace asr_sdm_kinematic_dynamic_model

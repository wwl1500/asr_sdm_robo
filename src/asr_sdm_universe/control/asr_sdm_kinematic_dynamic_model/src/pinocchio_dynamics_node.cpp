// Copyright (c) 2025.
// This ROS 2 node exposes Pinocchio-based rigid-body dynamics quantities
// (mass matrix, center of mass, total mass) so downstream components
// such as controllers or visualization tools can consume analytical data
// without re-implementing the math.

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "asr_sdm_kinematic_dynamic_model/underwater_dynamics.hpp"

namespace asr_sdm_kinematic_dynamic_model
{
class PinocchioDynamicsNode : public rclcpp::Node
{
public:
  PinocchioDynamicsNode() : Node("pinocchio_dynamics_node")
  {
    // Parameters allow the node to target different robots / publication rates.
    declare_parameter<std::string>("robot_description_path", "");
    declare_parameter<std::string>("robot_description_package", "asr_sdm_description");
    declare_parameter<std::string>("robot_description_file", "urdf/underwater_snakerobot.urdf");
    declare_parameter<bool>("use_free_flyer", true);
    declare_parameter<bool>("log_model_summary", true);
    declare_parameter<int>("publish_period_ms", 200);
    declare_parameter<std::vector<double>>("initial_joint_positions", std::vector<double>{});
    
    // Hydrodynamic parameters
    declare_parameter<double>("fluid_density", 1000.0);
    declare_parameter<double>("cd_n", 1.0);
    declare_parameter<double>("cd_t", 0.05);
    declare_parameter<std::vector<std::string>>("hydrodynamic_link_names", std::vector<std::string>{});
    declare_parameter<std::vector<double>>("link_volumes", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_radii", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});
    declare_parameter<std::vector<double>>("added_mass_factors", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_added_mass_diagonal", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_linear_damping_diagonal", std::vector<double>{});

    load_robot_model();
    if (!model_) {
      throw std::runtime_error("Failed to create Pinocchio model.");
    }

    create_publishers();
    configure_timer();
  }

private:
  // Resolve the URDF path (explicit string or via package share), build the
  // Pinocchio model/data objects and optionally seed q with custom joints.
  void load_robot_model()
  {
    const auto explicit_path = get_parameter("robot_description_path").as_string();
    const auto package_name = get_parameter("robot_description_package").as_string();
    const auto relative_path = get_parameter("robot_description_file").as_string();
    const bool use_free_flyer = get_parameter("use_free_flyer").as_bool();

    std::string resolved_urdf;
    if (!explicit_path.empty()) {
      resolved_urdf = explicit_path;
    } else {
      try {
        const auto package_share = ament_index_cpp::get_package_share_directory(package_name);
        resolved_urdf = package_share + "/" + relative_path;
      } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), "Failed to locate package '%s': %s", package_name.c_str(), e.what());
        return;
      }
    }

    if (!std::ifstream(resolved_urdf)) {
      RCLCPP_FATAL(get_logger(), "URDF file not found: %s", resolved_urdf.c_str());
      return;
    }

    model_ = std::make_shared<pinocchio::Model>();
    try {
      if (use_free_flyer) {
        pinocchio::urdf::buildModel(resolved_urdf, pinocchio::JointModelFreeFlyer(), *model_);
      } else {
        pinocchio::urdf::buildModel(resolved_urdf, *model_);
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio failed to build model: %s", e.what());
      model_.reset();
      return;
    }

    data_ = std::make_shared<pinocchio::Data>(*model_);
    // Start from the neutral configuration and zero joint velocities so the
    // published dynamics are always well-defined even without live inputs.
    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(model_->nv);

    std::vector<double> initial_positions;
    get_parameter("initial_joint_positions", initial_positions);
    if (!initial_positions.empty()) {
      const auto length = static_cast<int>(
        std::min(initial_positions.size(), static_cast<size_t>(q_.size())));
      for (int i = 0; i < length; ++i) {
        q_[i] = initial_positions[i];
      }
      if (static_cast<size_t>(q_.size()) != initial_positions.size()) {
        RCLCPP_WARN(
          get_logger(),
          "Initial joint positions size (%zu) does not match model nq (%ld). Remaining joints keep neutral configuration.",
          initial_positions.size(), static_cast<long>(q_.size()));
      }
    }

    HydrodynamicParameters params;
    params.rho = get_parameter("fluid_density").as_double();
    params.Cd_n = get_parameter("cd_n").as_double();
    params.Cd_t = get_parameter("cd_t").as_double();
    get_parameter("hydrodynamic_link_names", params.hydrodynamic_link_names);
    get_parameter("link_volumes", params.link_volumes);
    get_parameter("link_radii", params.link_radii);
    get_parameter("link_lengths", params.link_lengths);
    get_parameter("added_mass_factors", params.added_mass_factors);
    get_parameter("link_added_mass_diagonal", params.link_added_mass_diagonal);
    get_parameter("link_linear_damping_diagonal", params.link_linear_damping_diagonal);

    ud_model_ = std::make_shared<UnderwaterDynamics>(model_, data_, params);

    if (get_parameter("log_model_summary").as_bool()) {
      RCLCPP_INFO(
        get_logger(),
        "Loaded Pinocchio model: %s | joints: %d | velocity dof: %d",
        model_->name.c_str(), model_->nq, model_->nv);
    }
  }

  void create_publishers()
  {
    mass_matrix_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/mass_matrix", 10);
    coriolis_matrix_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/coriolis_matrix", 10);
    damping_matrix_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/damping_matrix", 10);
    restoring_forces_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/restoring_forces", 10);
    
    center_of_mass_pub_ = create_publisher<geometry_msgs::msg::Vector3>("pinocchio/center_of_mass", 10);
    total_mass_pub_ = create_publisher<std_msgs::msg::Float64>("pinocchio/total_mass", 10);
  }

  void configure_timer()
  {
    const int publish_period_ms = get_parameter("publish_period_ms").as_int();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&PinocchioDynamicsNode::publish_dynamics, this));
  }

  // Compute and publish mass matrix, center of mass and total mass for the
  // current configuration. Future extensions can subscribe to joint states.
  void publish_dynamics()
  {
    if (!model_ || !data_ || !ud_model_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Pinocchio model or UnderwaterDynamics is not ready.");
      return;
    }

    // Compute complete dynamics via the newly created model
    ud_model_->computeDynamics(q_, v_);

    const Eigen::MatrixXd& M = ud_model_->getMassMatrix();
    const Eigen::MatrixXd& C = ud_model_->getCoriolisMatrix();
    const Eigen::MatrixXd& D = ud_model_->getDampingMatrix();
    const Eigen::VectorXd& N = ud_model_->getRestoringForces();

    const Eigen::Vector3d com = pinocchio::centerOfMass(*model_, *data_, q_, v_);
    const double mass = pinocchio::computeTotalMass(*model_, *data_);

    std_msgs::msg::Float64MultiArray mass_msg;
    // Encode the dense inertia matrix in row-major order to ease reshaping.
    mass_msg.layout.dim.resize(2);
    mass_msg.layout.dim[0].label = "rows";
    mass_msg.layout.dim[0].size = model_->nv;
    mass_msg.layout.dim[0].stride = model_->nv;
    mass_msg.layout.dim[1].label = "cols";
    mass_msg.layout.dim[1].size = model_->nv;
    mass_msg.layout.dim[1].stride = 1;
    mass_msg.data.reserve(static_cast<size_t>(model_->nv * model_->nv));

    for (Eigen::Index r = 0; r < M.rows(); ++r) {
      for (Eigen::Index c = 0; c < M.cols(); ++c) {
        mass_msg.data.push_back(M(r, c));
      }
    }
    mass_matrix_pub_->publish(mass_msg);

    // Publish Coriolis
    std_msgs::msg::Float64MultiArray coriolis_msg = mass_msg; // same dimensional schema
    coriolis_msg.data.clear();
    for (Eigen::Index r = 0; r < C.rows(); ++r) {
      for (Eigen::Index c = 0; c < C.cols(); ++c) {
        coriolis_msg.data.push_back(C(r, c));
      }
    }
    coriolis_matrix_pub_->publish(coriolis_msg);

    // Publish Damping
    std_msgs::msg::Float64MultiArray damping_msg = mass_msg;
    damping_msg.data.clear();
    for (Eigen::Index r = 0; r < D.rows(); ++r) {
      for (Eigen::Index c = 0; c < D.cols(); ++c) {
        damping_msg.data.push_back(D(r, c));
      }
    }
    damping_matrix_pub_->publish(damping_msg);

    // Publish Restoring Forces (1D mapping)
    std_msgs::msg::Float64MultiArray restoring_msg;
    restoring_msg.layout.dim.resize(1);
    restoring_msg.layout.dim[0].label = "length";
    restoring_msg.layout.dim[0].size = model_->nv;
    restoring_msg.layout.dim[0].stride = model_->nv;
    for (Eigen::Index r = 0; r < N.rows(); ++r) {
        restoring_msg.data.push_back(N(r));
    }
    restoring_forces_pub_->publish(restoring_msg);

    geometry_msgs::msg::Vector3 com_msg;
    com_msg.x = com.x();
    com_msg.y = com.y();
    com_msg.z = com.z();
    center_of_mass_pub_->publish(com_msg);

    std_msgs::msg::Float64 total_mass_msg;
    total_mass_msg.data = mass;
    total_mass_pub_->publish(total_mass_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mass_matrix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr coriolis_matrix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr damping_matrix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr restoring_forces_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr center_of_mass_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_mass_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::shared_ptr<UnderwaterDynamics> ud_model_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
};
}  // namespace asr_sdm_kinematic_dynamic_model

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Keep the node alive so visualization/analytics tools can consume results.
    auto node = std::make_shared<asr_sdm_kinematic_dynamic_model::PinocchioDynamicsNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("pinocchio_dynamics_node"), "Unhandled exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

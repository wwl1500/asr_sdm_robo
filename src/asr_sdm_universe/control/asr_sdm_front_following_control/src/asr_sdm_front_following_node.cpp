#include "asr_sdm_front_following_control/front_following_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "asr_sdm_control_msgs/msg/actuator_force_cmd.hpp"
#include "asr_sdm_control_msgs/msg/robot_state.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{

template<int N>
Eigen::Matrix<double, N, 1> vectorParameterToEigen(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != static_cast<size_t>(N)) {
    throw std::invalid_argument(
            parameter_name + " must contain exactly " + std::to_string(N) + " values.");
  }

  Eigen::Matrix<double, N, 1> result = Eigen::Matrix<double, N, 1>::Zero();
  for (int index = 0; index < N; ++index) {
    result(index) = values[static_cast<size_t>(index)];
  }
  return result;
}

std_msgs::msg::Float64MultiArray toMultiArray(const Eigen::VectorXd & vector)
{
  std_msgs::msg::Float64MultiArray message;
  message.layout.dim.resize(1);
  message.layout.dim[0].label = "length";
  message.layout.dim[0].size = static_cast<uint32_t>(vector.size());
  message.layout.dim[0].stride = static_cast<uint32_t>(vector.size());
  message.data.reserve(static_cast<size_t>(vector.size()));
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    message.data.push_back(vector(index));
  }
  return message;
}

asr_sdm_controller::RobotKinematicState toKinematicState(
  const asr_sdm_control_msgs::msg::RobotState & message)
{
  asr_sdm_controller::RobotKinematicState state;
  state.position <<
    message.position.x,
    message.position.y,
    message.position.z;
  state.orientation = Eigen::Quaterniond(
    message.orientation.w,
    message.orientation.x,
    message.orientation.y,
    message.orientation.z);
  state.linear_velocity_body <<
    message.linear_velocity_body.x,
    message.linear_velocity_body.y,
    message.linear_velocity_body.z;
  state.angular_velocity_body <<
    message.angular_velocity_body.x,
    message.angular_velocity_body.y,
    message.angular_velocity_body.z;

  for (size_t index = 0; index < message.joint_positions.size(); ++index) {
    state.joint_positions(static_cast<Eigen::Index>(index)) = message.joint_positions[index];
    state.joint_velocities(static_cast<Eigen::Index>(index)) = message.joint_velocities[index];
  }

  return state;
}

}  // namespace

class AsrSdmControllerNode : public rclcpp::Node
{
public:
  AsrSdmControllerNode()
  : Node("asr_sdm_front_following")
  {
    declareParameters();
    loadRobotModel();
    createInterfaces();
    RCLCPP_INFO(get_logger(), "ASR-SDM front-following controller started.");
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("robot_description_path", "");
    declare_parameter<std::string>("robot_description_package", "asr_sdm_description");
    declare_parameter<std::string>("robot_description_file", "urdf/deprecated/underwater_snakerobot.urdf");
    declare_parameter<int>("control_period_ms", 50);
    declare_parameter<double>("state_timeout_sec", 0.5);
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("robot_state_topic", "~/input/robot_state");

    declare_parameter<double>("link_length", 0.30);
    declare_parameter<double>("front_half_length", 0.15);
    declare_parameter<double>("finite_difference_dt", 1.0e-4);
    declare_parameter<double>("pinv_tolerance", 1.0e-6);
    declare_parameter<double>("heading_projection_min", 1.0e-6);
    declare_parameter<double>("pitch_cos_min", 1.0e-3);
    declare_parameter<double>("aff_condition_number_limit", 1.0e6);
    declare_parameter<std::vector<double>>("eta_reference", std::vector<double>(6, 0.0));
    declare_parameter<std::vector<double>>("k_eta_p", std::vector<double>(6, 4.0));
    declare_parameter<std::vector<double>>("k_eta_d", std::vector<double>(6, 2.0));
    declare_parameter<std::vector<double>>("k_p", std::vector<double>(12, 8.0));
    declare_parameter<std::vector<double>>("k_d", std::vector<double>(12, 4.0));
    declare_parameter<std::vector<double>>("max_propulsion_force", std::vector<double>(4, 50.0));
    declare_parameter<std::vector<double>>("max_joint_torque", std::vector<double>(6, 100.0));

    declare_parameter<double>("fluid_density", 1000.0);
    declare_parameter<double>("cd_n", 1.0);
    declare_parameter<double>("cd_t", 0.05);
    declare_parameter<std::vector<std::string>>(
      "hydrodynamic_link_names", std::vector<std::string>{});
    declare_parameter<std::vector<double>>("link_volumes", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_radii", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});
    declare_parameter<std::vector<double>>("added_mass_factors", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_added_mass_diagonal", std::vector<double>{});
    declare_parameter<std::vector<double>>("link_linear_damping_diagonal", std::vector<double>{});
  }

  asr_sdm_controller::ControllerParameters loadControllerParameters() const
  {
    asr_sdm_controller::ControllerParameters parameters;
    parameters.link_length = get_parameter("link_length").as_double();
    parameters.front_half_length = get_parameter("front_half_length").as_double();
    parameters.finite_difference_dt = get_parameter("finite_difference_dt").as_double();
    parameters.pinv_tolerance = get_parameter("pinv_tolerance").as_double();
    parameters.heading_projection_min = get_parameter("heading_projection_min").as_double();
    parameters.pitch_cos_min = get_parameter("pitch_cos_min").as_double();
    parameters.aff_condition_number_limit =
      get_parameter("aff_condition_number_limit").as_double();
    parameters.eta_reference = vectorParameterToEigen<6>(
      get_parameter("eta_reference").as_double_array(), "eta_reference");
    parameters.k_eta_p = vectorParameterToEigen<6>(
      get_parameter("k_eta_p").as_double_array(), "k_eta_p");
    parameters.k_eta_d = vectorParameterToEigen<6>(
      get_parameter("k_eta_d").as_double_array(), "k_eta_d");
    parameters.k_p = vectorParameterToEigen<12>(
      get_parameter("k_p").as_double_array(), "k_p");
    parameters.k_d = vectorParameterToEigen<12>(
      get_parameter("k_d").as_double_array(), "k_d");
    parameters.max_propulsion_force = vectorParameterToEigen<4>(
      get_parameter("max_propulsion_force").as_double_array(), "max_propulsion_force");
    parameters.max_joint_torque = vectorParameterToEigen<6>(
      get_parameter("max_joint_torque").as_double_array(), "max_joint_torque");
    return parameters;
  }

  asr_sdm_kinematic_dynamic_model::HydrodynamicParameters loadHydrodynamicParameters() const
  {
    asr_sdm_kinematic_dynamic_model::HydrodynamicParameters parameters;
    parameters.rho = get_parameter("fluid_density").as_double();
    parameters.Cd_n = get_parameter("cd_n").as_double();
    parameters.Cd_t = get_parameter("cd_t").as_double();
    get_parameter("hydrodynamic_link_names", parameters.hydrodynamic_link_names);
    get_parameter("link_volumes", parameters.link_volumes);
    get_parameter("link_radii", parameters.link_radii);
    get_parameter("link_lengths", parameters.link_lengths);
    get_parameter("added_mass_factors", parameters.added_mass_factors);
    get_parameter("link_added_mass_diagonal", parameters.link_added_mass_diagonal);
    get_parameter("link_linear_damping_diagonal", parameters.link_linear_damping_diagonal);
    return parameters;
  }

  void loadRobotModel()
  {
    const auto explicit_path = get_parameter("robot_description_path").as_string();
    const auto package_name = get_parameter("robot_description_package").as_string();
    const auto relative_path = get_parameter("robot_description_file").as_string();

    std::string resolved_urdf;
    if (!explicit_path.empty()) {
      resolved_urdf = explicit_path;
    } else {
      resolved_urdf =
        ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
    }

    if (!std::ifstream(resolved_urdf)) {
      throw std::runtime_error("URDF file not found: " + resolved_urdf);
    }

    auto full_model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(
      resolved_urdf, pinocchio::JointModelFreeFlyer(), *full_model);

    std::vector<pinocchio::JointIndex> joints_to_lock;
    joints_to_lock.reserve(static_cast<size_t>(full_model->njoints));
    for (pinocchio::JointIndex joint_id = 1;
      joint_id < static_cast<pinocchio::JointIndex>(full_model->njoints); ++joint_id)
    {
      if (full_model->names[joint_id].find("screw") != std::string::npos) {
        joints_to_lock.push_back(joint_id);
      }
    }

    model_ = std::make_shared<pinocchio::Model>(
      pinocchio::buildReducedModel(
        *full_model, joints_to_lock, pinocchio::neutral(*full_model)));
    data_ = std::make_shared<pinocchio::Data>(*model_);
    controller_ = std::make_unique<asr_sdm_controller::FrontFollowingController>(
      model_, data_, loadHydrodynamicParameters(), loadControllerParameters());
  }

  void createInterfaces()
  {
    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const auto robot_state_topic = get_parameter("robot_state_topic").as_string();
    const int control_period_ms = get_parameter("control_period_ms").as_int();
    state_timeout_sec_ = get_parameter("state_timeout_sec").as_double();

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&AsrSdmControllerNode::onCommand, this, std::placeholders::_1));
    sub_robot_state_ = create_subscription<asr_sdm_control_msgs::msg::RobotState>(
      robot_state_topic, rclcpp::SensorDataQoS{},
      std::bind(&AsrSdmControllerNode::onRobotState, this, std::placeholders::_1));

    pub_heartbeat_ =
      create_publisher<std_msgs::msg::String>("~/output/controller/heartbeat", 1);
    pub_actuator_force_cmd_ =
      create_publisher<asr_sdm_control_msgs::msg::ActuatorForceCmd>(
        "~/output/actuator_force_cmd", 1);
    pub_controller_status_ =
      create_publisher<std_msgs::msg::String>("~/output/controller/status", 1);
    pub_aff_condition_number_ =
      create_publisher<std_msgs::msg::Float64>("~/output/controller/aff_condition_number", 1);
    pub_y_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/output/y", 1);
    pub_y_d_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/output/y_d", 1);
    pub_u_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/output/u_cmd", 1);
    pub_allocation_residual_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/output/allocation_residual", 1);

    timer_heartbeat_ = create_wall_timer(
      1500ms, std::bind(&AsrSdmControllerNode::onHeartbeatTimer, this));
    timer_control_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms),
      std::bind(&AsrSdmControllerNode::onControlTimer, this));
  }

  void onCommand(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    latest_command_ = *message;
  }

  void onRobotState(const asr_sdm_control_msgs::msg::RobotState::SharedPtr message)
  {
    latest_state_ = *message;
    latest_state_received_time_ = now();
  }

  void onHeartbeatTimer()
  {
    std_msgs::msg::String heartbeat;
    heartbeat.data = "alive";
    pub_heartbeat_->publish(heartbeat);
  }

  void publishZeroForceCommand()
  {
    asr_sdm_control_msgs::msg::ActuatorForceCmd message;
    message.header.stamp = now();
    pub_actuator_force_cmd_->publish(message);
  }

  void publishStatus(const std::string & status, const double aff_condition_number)
  {
    std_msgs::msg::String status_message;
    status_message.data = status;
    pub_controller_status_->publish(status_message);

    std_msgs::msg::Float64 aff_condition_message;
    aff_condition_message.data = aff_condition_number;
    pub_aff_condition_number_->publish(aff_condition_message);
  }

  void publishDebugVectors(const asr_sdm_controller::ControlComputationResult & result)
  {
    pub_y_->publish(toMultiArray(result.y));
    pub_y_d_->publish(toMultiArray(result.y_d));
    pub_u_cmd_->publish(toMultiArray(result.u_cmd));
    pub_allocation_residual_->publish(toMultiArray(result.allocation_residual));
  }

  void onControlTimer()
  {
    const rclcpp::Time current_time = now();

    if (!latest_state_.has_value() || !latest_state_received_time_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for RobotState on the configured input topic.");
      controller_->reset();
      last_control_time_.reset();
      publishZeroForceCommand();
      publishStatus("waiting_for_robot_state", std::numeric_limits<double>::quiet_NaN());
      return;
    }

    if ((current_time - *latest_state_received_time_).seconds() > state_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "RobotState timed out; publishing zero actuator force command.");
      controller_->reset();
      last_control_time_.reset();
      publishZeroForceCommand();
      publishStatus("state_timeout", std::numeric_limits<double>::quiet_NaN());
      return;
    }

    double dt_seconds = 0.05;
    if (last_control_time_.has_value()) {
      dt_seconds = std::max((current_time - *last_control_time_).seconds(), 1.0e-4);
    }
    last_control_time_ = current_time;

    const auto state = toKinematicState(*latest_state_);
    asr_sdm_controller::MotionCommand command;
    if (latest_command_.has_value()) {
      command.v1 = std::isfinite(latest_command_->linear.x) ? latest_command_->linear.x : 0.0;
      command.omega1 =
        std::isfinite(latest_command_->angular.z) ? latest_command_->angular.z : 0.0;
    }

    const auto result = controller_->compute(state, command, dt_seconds);
    if (!result.ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Controller fell back to zero output: %s", result.error_message.c_str());
      publishZeroForceCommand();
      publishStatus(
        "fallback_zero_output: " + result.error_message,
        result.aff_condition_number);
      publishDebugVectors(result);
      return;
    }

    asr_sdm_control_msgs::msg::ActuatorForceCmd message;
    message.header.stamp = current_time;
    for (size_t index = 0; index < message.propulsion_forces.size(); ++index) {
      message.propulsion_forces[index] =
        result.propulsion_forces(static_cast<Eigen::Index>(index));
    }
    for (size_t index = 0; index < message.joint_torques.size(); ++index) {
      message.joint_torques[index] =
        result.joint_torques(static_cast<Eigen::Index>(index));
    }
    pub_actuator_force_cmd_->publish(message);
    publishStatus("ok", result.aff_condition_number);
    publishDebugVectors(result);
  }

  double state_timeout_sec_ = 0.5;

  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::unique_ptr<asr_sdm_controller::FrontFollowingController> controller_;

  std::optional<geometry_msgs::msg::Twist> latest_command_;
  std::optional<asr_sdm_control_msgs::msg::RobotState> latest_state_;
  std::optional<rclcpp::Time> latest_state_received_time_;
  std::optional<rclcpp::Time> last_control_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<asr_sdm_control_msgs::msg::RobotState>::SharedPtr sub_robot_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_controller_status_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_aff_condition_number_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ActuatorForceCmd>::SharedPtr
    pub_actuator_force_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_y_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_y_d_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_u_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_allocation_residual_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_control_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}

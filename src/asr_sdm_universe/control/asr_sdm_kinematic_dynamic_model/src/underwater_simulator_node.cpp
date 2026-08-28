// Copyright (c) 2025.

#include "asr_sdm_kinematic_dynamic_model/csv_logger.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_visualization.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace asr_sdm_kinematic_dynamic_model
{

class UnderwaterSimulatorNode : public rclcpp::Node
{
public:
  UnderwaterSimulatorNode()
  : Node("underwater_simulator_node")
  {
    const auto simulator_params = makeSimulatorParameters();
    dt_ = declare_parameter<double>("dt", 0.005);
    duration_ = declare_parameter<double>("duration", 60.0);
    run_forever_ = declare_parameter<bool>("run_forever", false);
    auto_shutdown_ = declare_parameter<bool>("auto_shutdown", true);
    csv_path_ = declare_parameter<std::string>("csv_path", "underwater_simulation.csv");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    root_frame_ = declare_parameter<std::string>("root_frame", "screwdrive_segment_0");
    z_to_x_map_ = declare_parameter<bool>("z_to_x_map", true);
    force_scale_ = declare_parameter<double>("force_scale", 0.02);
    max_trajectory_points_ = declare_parameter<int>("max_trajectory_points", 3000);
    if (!std::isfinite(dt_) || dt_ <= 0.0 || !std::isfinite(duration_) || duration_ < 0.0 ||
      !std::isfinite(force_scale_) || force_scale_ < 0.0 || max_trajectory_points_ < 2)
    {
      throw std::runtime_error(
        "dt/duration/force_scale must be finite and valid; max_trajectory_points must be at least 2");
    }

    simulator_ = std::make_unique<UnderwaterSimulator>(simulator_params);
    if (!simulator_->isValid()) {
      throw std::runtime_error(simulator_->error());
    }
    hydrodynamic_parameters_ = simulator_params.hydrodynamics;
    state_ = simulator_->makeInitialState();
    applyInitialState();
    input_.segment_thrust = readFixedVector<kNumLinks>("segment_thrust");
    input_.joint_torque = readFixedVector<kNumJointDofs>("joint_torque");
    input_.fluid_current_world = Eigen::Vector3d(
      declare_parameter<double>("fluid_current_x", 0.0),
      declare_parameter<double>("fluid_current_y", 0.0),
      declare_parameter<double>("fluid_current_z", 0.0));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&UnderwaterSimulatorNode::onParametersSet, this, std::placeholders::_1));
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/underwater_simulator/joint_states", rclcpp::SensorDataQoS());
    fluid_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/underwater_simulator/fluid_force_markers", rclcpp::QoS(1).transient_local());
    trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/underwater_simulator/trajectory", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    controller_mapping_ = simulator_->pinocchioModel().controllerDofMapping();
    for (std::size_t link = 0; link < kNumLinks; ++link) {
      rotor_joint_names_[2 * link] =
        "joint_screwdrive_segment_0__screw_rotor_left_0__" + std::to_string(link);
      rotor_joint_names_[2 * link + 1] =
        "joint_screwdrive_segment_0__screw_rotor_right_0__" + std::to_string(link);
    }

    logger_ = std::make_unique<CsvLogger>(csv_path_);
    if (!logger_->isOpen()) {
      throw std::runtime_error(logger_->error());
    }
    const auto initial_output = simulator_->evaluate(state_, input_);
    logger_->write(state_, input_, initial_output);
    publishVisualization(state_, initial_output);

    if (duration_ == 0.0 && !run_forever_) {
      RCLCPP_INFO(get_logger(), "Wrote initial state to %s", csv_path_.c_str());
      timer_ = create_wall_timer(
        std::chrono::milliseconds(1), []() {rclcpp::shutdown();});
      return;
    }
    const auto period = std::chrono::milliseconds(
      std::max<int64_t>(1, static_cast<int64_t>(std::llround(dt_ * 1000.0))));
    timer_ = create_wall_timer(period, std::bind(&UnderwaterSimulatorNode::step, this));
    RCLCPP_INFO(
      get_logger(),
      "Underwater simulation started: dt=%.6f s, duration=%.3f s, run_forever=%s, csv=%s",
      dt_, duration_, run_forever_ ? "true" : "false", csv_path_.c_str());
  }

private:
  template<int Size>
  Eigen::Matrix<double, Size, 1> readFixedVector(const std::string & name)
  {
    const auto values = declare_parameter<std::vector<double>>(name,
        std::vector<double>(Size, 0.0));
    if (values.size() != Size) {
      throw std::runtime_error(name + " must contain exactly " + std::to_string(Size) + " values");
    }
    Eigen::Matrix<double, Size, 1> output;
    for (int i = 0; i < Size; ++i) {
      output(i) = values[static_cast<std::size_t>(i)];
    }
    return output;
  }

  UnderwaterSimulatorParameters makeSimulatorParameters()
  {
    UnderwaterSimulatorParameters params;
    params.pinocchio.use_free_flyer = true;
    params.pinocchio.lock_rotor_joints = true;
    params.pinocchio.urdf_path = declare_parameter<std::string>("robot_description_path", "");
    if (params.pinocchio.urdf_path.empty()) {
      const auto package = declare_parameter<std::string>("robot_description_package", "asr_sdm");
      const auto relative = declare_parameter<std::string>(
        "robot_description_file", "urdf/generated/asr_sdm_segments_4.urdf");
      try {
        params.pinocchio.urdf_path = ament_index_cpp::get_package_share_directory(package) + "/" +
          relative;
      } catch (const std::exception & e) {
        throw std::runtime_error("Failed to find robot description package: " +
          std::string(e.what()));
      }
    }
    params.hydrodynamics.fluid_density = declare_parameter<double>("fluid_density", 1000.0);
    const auto volumes = declare_parameter<std::vector<double>>(
      "displaced_volume", std::vector<double>(kNumLinks, 5.81e-4));
    if (volumes.size() != kNumLinks) {
      throw std::runtime_error("displaced_volume must contain four values");
    }
    const double linear_damping = declare_parameter<double>("linear_damping", 0.1);
    const double quadratic_damping = declare_parameter<double>("quadratic_damping", 0.01);
    const auto added_mass_diagonal = declare_parameter<std::vector<double>>(
      "added_mass_diagonal", std::vector<double>(kNumLinks * kSpatialDofs, 0.0));
    if (added_mass_diagonal.size() != kNumLinks * kSpatialDofs) {
      throw std::runtime_error("added_mass_diagonal must contain 24 values");
    }
    for (std::size_t link = 0; link < kNumLinks; ++link) {
      params.hydrodynamics.links[link].displaced_volume = volumes[link];
      params.hydrodynamics.links[link].linear_damping.setConstant(linear_damping);
      params.hydrodynamics.links[link].quadratic_damping.setConstant(quadratic_damping);
      for (std::size_t component = 0; component < kSpatialDofs; ++component) {
        params.hydrodynamics.links[link].added_mass(
          static_cast<Eigen::Index>(component), static_cast<Eigen::Index>(component)) =
          added_mass_diagonal[link * kSpatialDofs + component];
      }
    }
    const auto maximum_thrust = declare_parameter<std::vector<double>>(
      "maximum_segment_thrust", std::vector<double>(kNumLinks, 1.0));
    if (maximum_thrust.size() != kNumLinks) {
      throw std::runtime_error("maximum_segment_thrust must contain four values");
    }
    for (std::size_t link = 0; link < kNumLinks; ++link) {
      params.actuators.maximum_segment_thrust(static_cast<Eigen::Index>(link)) =
        maximum_thrust[link];
    }
    return params;
  }

  void applyInitialState()
  {
    const auto initial_q = declare_parameter<std::vector<double>>(
      "initial_q", std::vector<double>{});
    const auto initial_v = declare_parameter<std::vector<double>>(
      "initial_v", std::vector<double>{});
    if (!initial_q.empty()) {
      if (initial_q.size() != kReducedNq) {
        throw std::runtime_error("initial_q must contain 13 values");
      }
      for (int i = 0; i < kReducedNq; ++i) {
        state_.configuration(i) = initial_q[static_cast<std::size_t>(i)];
      }
    }
    if (!initial_v.empty()) {
      if (initial_v.size() != kReducedNv) {
        throw std::runtime_error("initial_v must contain 12 values");
      }
      for (int i = 0; i < kReducedNv; ++i) {
        state_.velocity(i) = initial_v[static_cast<std::size_t>(i)];
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto candidate_hydrodynamics = hydrodynamic_parameters_;
    auto candidate_input = input_;
    bool hydrodynamics_changed = false;

    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "fluid_density") {
          candidate_hydrodynamics.fluid_density = parameter.as_double();
          hydrodynamics_changed = true;
        } else if (name == "linear_damping") {
          const double value = parameter.as_double();
          for (auto & link : candidate_hydrodynamics.links) {
            link.linear_damping.setConstant(value);
          }
          hydrodynamics_changed = true;
        } else if (name == "quadratic_damping") {
          const double value = parameter.as_double();
          for (auto & link : candidate_hydrodynamics.links) {
            link.quadratic_damping.setConstant(value);
          }
          hydrodynamics_changed = true;
        } else if (name == "displaced_volume") {
          const auto values = parameter.as_double_array();
          if (values.size() != kNumLinks) {
            throw std::invalid_argument("displaced_volume must contain four values");
          }
          for (std::size_t link = 0; link < kNumLinks; ++link) {
            candidate_hydrodynamics.links[link].displaced_volume = values[link];
          }
          hydrodynamics_changed = true;
        } else if (name == "fluid_current_x") {
          candidate_input.fluid_current_world.x() = parameter.as_double();
        } else if (name == "fluid_current_y") {
          candidate_input.fluid_current_world.y() = parameter.as_double();
        } else if (name == "fluid_current_z") {
          candidate_input.fluid_current_world.z() = parameter.as_double();
        } else if (name == "segment_thrust") {
          const auto values = parameter.as_double_array();
          if (values.size() != kNumLinks) {
            throw std::invalid_argument("segment_thrust must contain four values");
          }
          for (std::size_t link = 0; link < kNumLinks; ++link) {
            candidate_input.segment_thrust(static_cast<Eigen::Index>(link)) = values[link];
          }
        } else if (name == "joint_torque") {
          const auto values = parameter.as_double_array();
          if (values.size() != kNumJointDofs) {
            throw std::invalid_argument("joint_torque must contain six values");
          }
          for (std::size_t joint = 0; joint < kNumJointDofs; ++joint) {
            candidate_input.joint_torque(static_cast<Eigen::Index>(joint)) = values[joint];
          }
        } else if (name == "dt" || name == "duration" || name == "run_forever" ||
          name == "auto_shutdown" || name == "csv_path" ||
          name.rfind("qos_overrides.", 0) == 0)
        {
          continue;
        } else {
          result.successful = false;
          result.reason = "Parameter is not runtime adjustable: " + name;
          return result;
        }
      }

      if (!candidate_input.fluid_current_world.array().isFinite().all() ||
        !candidate_input.segment_thrust.array().isFinite().all() ||
        !candidate_input.joint_torque.array().isFinite().all())
      {
        throw std::invalid_argument("Runtime input must contain only finite values");
      }
      if (hydrodynamics_changed) {
        std::string error;
        if (!simulator_->replaceHydrodynamicParameters(candidate_hydrodynamics, &error)) {
          result.successful = false;
          result.reason = error;
          return result;
        }
        hydrodynamic_parameters_ = candidate_hydrodynamics;
      }
      input_ = candidate_input;
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
    }
    if (result.successful) {
      RCLCPP_INFO(get_logger(), "Applied runtime underwater simulation parameter update");
    }
    return result;
  }

  void publishVisualization(
    const UnderwaterSimulatorState & state, const UnderwaterDynamicsOutput & output)
  {
    const int64_t nanoseconds = get_clock()->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    const Eigen::Matrix3d rotation =
      Eigen::Quaterniond(
      state.configuration(6), state.configuration(3), state.configuration(4),
      state.configuration(5)).toRotationMatrix();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = root_frame_;
    const Eigen::Vector3d position = mapPosition(
      state.configuration.head<3>(), z_to_x_map_);
    const Eigen::Matrix3d mapped_rotation = mapRotation(rotation, z_to_x_map_);
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation = toRosQuaternion(mapped_rotation);
    tf_broadcaster_->sendTransform(transform);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.header.frame_id = root_frame_;
    for (const auto & mapping : controller_mapping_) {
      joint_state.name.push_back(mapping.joint_name);
      joint_state.position.push_back(state.configuration(mapping.q_index));
      joint_state.velocity.push_back(state.velocity(mapping.v_index));
    }
    for (const auto & name : rotor_joint_names_) {
      joint_state.name.push_back(name);
      joint_state.position.push_back(0.0);
      joint_state.velocity.push_back(0.0);
    }
    joint_state_pub_->publish(joint_state);

    visualization_msgs::msg::MarkerArray markers;
    constexpr std::array<float, 4> damping_color{0.95F, 0.15F, 0.15F, 1.0F};
    constexpr std::array<float, 4> buoyancy_color{0.15F, 0.45F, 0.95F, 1.0F};
    constexpr std::array<float, 4> total_color{0.75F, 0.25F, 0.95F, 1.0F};
    const auto & root_placement = output.kinematics.segment_placements[0];
    trajectory_points_.push_back(mapPosition(root_placement.translation(), z_to_x_map_));
    for (std::size_t link = 0; link < kNumLinks; ++link) {
      const auto & placement = output.kinematics.segment_placements[link];
      const Eigen::Vector3d origin = mapPosition(placement.translation(), z_to_x_map_);
      const Eigen::Matrix3d link_rotation = mapRotation(placement.rotation(), z_to_x_map_);
      const auto add_marker = [&](
        const char * name_space, int id, const SpatialVector & wrench,
        const std::array<float, 4> & color)
        {
          markers.markers.push_back(makeForceMarker(
              world_frame_, name_space, id, stamp, origin,
              link_rotation * wrench.head<3>(), force_scale_, color));
        };
      add_marker("damping", static_cast<int>(link), output.hydrodynamics.damping_wrenches[link],
        damping_color);
      add_marker("buoyancy", 4 + static_cast<int>(link),
        output.hydrodynamics.buoyancy_wrenches[link], buoyancy_color);
      add_marker("total_fluid", 8 + static_cast<int>(link),
        output.hydrodynamics.total_wrenches[link], total_color);
    }
    while (trajectory_points_.size() > static_cast<std::size_t>(max_trajectory_points_)) {
      trajectory_points_.erase(trajectory_points_.begin());
    }
    markers.markers.push_back(makeTrajectoryMarker(
        world_frame_, stamp, trajectory_points_, 0.01,
        std::array<float, 4>{1.0F, 1.0F, 1.0F, 1.0F}));
    fluid_marker_pub_->publish(markers);
    trajectory_pub_->publish(markers.markers.back());
  }

  void step()
  {
    try {
      const UnderwaterSimulatorState state_before_step = state_;
      const auto output = simulator_->step(state_, dt_, input_);
      logger_->write(state_before_step, input_, output);
      publishVisualization(state_before_step, output);
      if (!run_forever_ && duration_ > 0.0 && state_.time + 0.5 * dt_ >= duration_) {
        RCLCPP_INFO(get_logger(), "Underwater simulation reached t=%.6f", state_.time);
        if (auto_shutdown_) {
          rclcpp::shutdown();
        } else {
          timer_->cancel();
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Simulation stopped at t=%.6f: %s", state_.time, e.what());
      timer_->cancel();
    }
  }

  double dt_{0.01};
  double duration_{60.0};
  bool run_forever_{false};
  bool auto_shutdown_{true};
  bool z_to_x_map_{true};
  double force_scale_{0.02};
  int max_trajectory_points_{3000};
  std::string csv_path_;
  std::string world_frame_{"world"};
  std::string root_frame_{"screwdrive_segment_0"};
  HydrodynamicModelParameters hydrodynamic_parameters_;
  UnderwaterSimulatorInput input_;
  UnderwaterSimulatorState state_;
  std::unique_ptr<UnderwaterSimulator> simulator_;
  std::unique_ptr<CsvLogger> logger_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fluid_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::array<PinocchioControllerDofMapping, kNumJointDofs> controller_mapping_{};
  std::array<std::string, 2 * kNumLinks> rotor_joint_names_{};
  std::vector<Eigen::Vector3d> trajectory_points_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<asr_sdm_kinematic_dynamic_model::UnderwaterSimulatorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("underwater_simulator_node"), "Unhandled exception: %s",
      e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

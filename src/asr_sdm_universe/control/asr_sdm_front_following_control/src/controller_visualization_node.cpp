#include "asr_sdm_front_following_control/controller_visualization.hpp"

#include "asr_sdm_control_msgs/msg/actuator_force_cmd.hpp"
#include "asr_sdm_control_msgs/msg/robot_state.hpp"

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{

template<typename T>
std::string formatScalar(const T value)
{
  if (!std::isfinite(static_cast<double>(value))) {
    return "n/a";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const int64_t nanoseconds = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

class ControllerVisualizationNode : public rclcpp::Node
{
public:
  ControllerVisualizationNode()
  : Node("controller_visualization")
  {
    declare_parameter<std::string>("robot_state_topic", "/asr_sdm_front_following/input/robot_state");
    declare_parameter<std::string>(
      "actuator_force_cmd_topic", "/asr_sdm_front_following/output/actuator_force_cmd");
    declare_parameter<std::string>("y_topic", "/asr_sdm_front_following/output/y");
    declare_parameter<std::string>("y_d_topic", "/asr_sdm_front_following/output/y_d");
    declare_parameter<std::string>(
      "allocation_residual_topic", "/asr_sdm_front_following/output/allocation_residual");
    declare_parameter<std::string>(
      "controller_status_topic", "/asr_sdm_front_following/output/controller/status");
    declare_parameter<std::string>(
      "aff_condition_topic", "/asr_sdm_front_following/output/controller/aff_condition_number");
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<int>("publish_period_ms", 50);
    declare_parameter<int>("path_history_length", 200);
    declare_parameter<double>("state_timeout_sec", 0.5);

    world_frame_ = get_parameter("world_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    path_history_length_ =
      std::max(1, static_cast<int>(get_parameter("path_history_length").as_int()));
    state_timeout_sec_ = get_parameter("state_timeout_sec").as_double();

    sub_robot_state_ = create_subscription<asr_sdm_control_msgs::msg::RobotState>(
      get_parameter("robot_state_topic").as_string(),
      rclcpp::SensorDataQoS{},
      std::bind(&ControllerVisualizationNode::onRobotState, this, std::placeholders::_1));
    sub_actuator_force_cmd_ = create_subscription<asr_sdm_control_msgs::msg::ActuatorForceCmd>(
      get_parameter("actuator_force_cmd_topic").as_string(), 10,
      std::bind(&ControllerVisualizationNode::onActuatorForceCmd, this, std::placeholders::_1));
    sub_y_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("y_topic").as_string(), 10,
      std::bind(&ControllerVisualizationNode::onActualPath, this, std::placeholders::_1));
    sub_y_d_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("y_d_topic").as_string(), 10,
      std::bind(&ControllerVisualizationNode::onDesiredPath, this, std::placeholders::_1));
    sub_allocation_residual_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("allocation_residual_topic").as_string(), 10,
      std::bind(&ControllerVisualizationNode::onAllocationResidual, this, std::placeholders::_1));
    sub_controller_status_ = create_subscription<std_msgs::msg::String>(
      get_parameter("controller_status_topic").as_string(), 10,
      std::bind(&ControllerVisualizationNode::onControllerStatus, this, std::placeholders::_1));
    sub_aff_condition_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("aff_condition_topic").as_string(), 10,
      std::bind(&ControllerVisualizationNode::onAffCondition, this, std::placeholders::_1));

    pub_joint_state_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_front_path_actual_ = create_publisher<nav_msgs::msg::Path>("~/output/front_path_actual", 1);
    pub_front_path_desired_ =
      create_publisher<nav_msgs::msg::Path>("~/output/front_path_desired", 1);
    pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(get_parameter("publish_period_ms").as_int()),
      std::bind(&ControllerVisualizationNode::onTimer, this));
  }

private:
  void onRobotState(const asr_sdm_control_msgs::msg::RobotState::SharedPtr message)
  {
    latest_state_ = *message;
    latest_state_received_time_ = now();
  }

  void onActuatorForceCmd(const asr_sdm_control_msgs::msg::ActuatorForceCmd::SharedPtr message)
  {
    latest_actuator_force_cmd_ = *message;
  }

  void onActualPath(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    latest_y_ = *message;

    std::array<double, 3> planar_pose{};
    if (!asr_sdm_controller::extractPlanarPose(*message, planar_pose)) {
      return;
    }

    latest_front_actual_ = planar_pose;
    actual_path_history_.push_back(
      asr_sdm_controller::buildPlanarPoseStamped(
        planar_pose, toBuiltinTime(now()), world_frame_));
    trimPathHistory(actual_path_history_);
    publishPaths();
  }

  void onDesiredPath(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    latest_y_d_ = *message;

    std::array<double, 3> planar_pose{};
    if (!asr_sdm_controller::extractPlanarPose(*message, planar_pose)) {
      return;
    }

    latest_front_desired_ = planar_pose;
    desired_path_history_.push_back(
      asr_sdm_controller::buildPlanarPoseStamped(
        planar_pose, toBuiltinTime(now()), world_frame_));
    trimPathHistory(desired_path_history_);
    publishPaths();
  }

  void onAllocationResidual(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    latest_allocation_residual_ = *message;
  }

  void onControllerStatus(const std_msgs::msg::String::SharedPtr message)
  {
    latest_controller_status_ = *message;
  }

  void onAffCondition(const std_msgs::msg::Float64::SharedPtr message)
  {
    latest_aff_condition_ = *message;
  }

  void trimPathHistory(std::deque<geometry_msgs::msg::PoseStamped> & history)
  {
    while (static_cast<int>(history.size()) > path_history_length_) {
      history.pop_front();
    }
  }

  nav_msgs::msg::Path buildPathMessage(
    const std::deque<geometry_msgs::msg::PoseStamped> & history,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.poses.assign(history.begin(), history.end());
    return message;
  }

  void publishPaths()
  {
    const rclcpp::Time stamp = now();
    pub_front_path_actual_->publish(buildPathMessage(actual_path_history_, stamp));
    pub_front_path_desired_->publish(buildPathMessage(desired_path_history_, stamp));
  }

  std::string buildStatusText(const bool state_is_fresh) const
  {
    std::ostringstream stream;
    if (!latest_state_.has_value()) {
      stream << "status: waiting_for_robot_state";
    } else if (!state_is_fresh) {
      stream << "status: stale_robot_state";
    } else if (latest_controller_status_.has_value()) {
      stream << "status: " << latest_controller_status_->data;
    } else {
      stream << "status: controller_status_unavailable";
    }

    const double residual_norm =
      latest_allocation_residual_.has_value() ?
      asr_sdm_controller::computeEuclideanNorm(*latest_allocation_residual_) :
      std::numeric_limits<double>::quiet_NaN();
    stream << "\nresidual_norm: " << formatScalar(residual_norm);

    const double aff_condition =
      latest_aff_condition_.has_value() ? latest_aff_condition_->data :
      std::numeric_limits<double>::quiet_NaN();
    stream << "\ncond(A_ff): " << formatScalar(aff_condition);
    return stream.str();
  }

  visualization_msgs::msg::MarkerArray buildMarkerArray(
    const rclcpp::Time & stamp,
    const bool state_is_fresh) const
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(
      asr_sdm_controller::buildDeleteAllMarker(toBuiltinTime(stamp), world_frame_));

    double status_x = 0.0;
    double status_y = 0.0;
    double status_z = 0.35;

    if (state_is_fresh && latest_state_.has_value()) {
      status_x = latest_state_->position.x - 0.55;
      status_y = latest_state_->position.y + 0.38;
      status_z = latest_state_->position.z + 0.48;

      asr_sdm_control_msgs::msg::ActuatorForceCmd actuator_force_cmd;
      if (latest_actuator_force_cmd_.has_value()) {
        actuator_force_cmd = *latest_actuator_force_cmd_;
      }

      const auto propulsion_frames = asr_sdm_controller::propulsionFrameNames();
      for (size_t index = 0; index < propulsion_frames.size(); ++index) {
        marker_array.markers.push_back(
          asr_sdm_controller::buildPropulsionForceMarker(
            propulsion_frames[index],
            static_cast<int>(index),
            actuator_force_cmd.propulsion_forces[index],
            toBuiltinTime(stamp)));
      }

      const auto intermediate_frames = asr_sdm_controller::intermediateJointFrameNames();
      for (size_t index = 0; index < intermediate_frames.size(); ++index) {
        const size_t pitch_index = 2 * index;
        const size_t yaw_index = pitch_index + 1;
        marker_array.markers.push_back(
          asr_sdm_controller::buildJointTorqueMarker(
            intermediate_frames[index],
            100 + static_cast<int>(index),
            actuator_force_cmd.joint_torques[pitch_index],
            actuator_force_cmd.joint_torques[yaw_index],
            toBuiltinTime(stamp)));
      }

      marker_array.markers.push_back(
        asr_sdm_controller::buildBodyHeadingMarker(
          *latest_state_, toBuiltinTime(stamp), world_frame_, 180));
      marker_array.markers.push_back(
        asr_sdm_controller::buildBodyOrientationTextMarker(
          *latest_state_, toBuiltinTime(stamp), world_frame_, 181));

      if (latest_front_actual_.has_value()) {
        marker_array.markers.push_back(
          asr_sdm_controller::buildFrontPointMarker(
            *latest_front_actual_, toBuiltinTime(stamp), world_frame_, 200, false));
      }
      if (latest_front_desired_.has_value()) {
        marker_array.markers.push_back(
          asr_sdm_controller::buildFrontPointMarker(
            *latest_front_desired_, toBuiltinTime(stamp), world_frame_, 201, true));
      }
    }

    marker_array.markers.push_back(
      asr_sdm_controller::buildStatusTextMarker(
        world_frame_,
        300,
        status_x,
        status_y,
        status_z,
        buildStatusText(state_is_fresh),
        toBuiltinTime(stamp)));

    return marker_array;
  }

  void onTimer()
  {
    const rclcpp::Time stamp = now();
    const bool state_is_fresh =
      latest_state_.has_value() &&
      latest_state_received_time_.has_value() &&
      (stamp - *latest_state_received_time_).seconds() <= state_timeout_sec_;

    if (state_is_fresh && latest_state_.has_value()) {
      pub_joint_state_->publish(
        asr_sdm_controller::buildJointStateMessage(*latest_state_, toBuiltinTime(stamp)));
      tf_broadcaster_->sendTransform(
        asr_sdm_controller::buildBaseTransformMessage(
          *latest_state_, world_frame_, base_frame_, toBuiltinTime(stamp)));
    }

    publishPaths();
    pub_markers_->publish(buildMarkerArray(stamp, state_is_fresh));
  }

  std::string world_frame_ = "world";
  std::string base_frame_ = "base_link";
  int path_history_length_ = 200;
  double state_timeout_sec_ = 0.5;

  std::optional<asr_sdm_control_msgs::msg::RobotState> latest_state_;
  std::optional<rclcpp::Time> latest_state_received_time_;
  std::optional<asr_sdm_control_msgs::msg::ActuatorForceCmd> latest_actuator_force_cmd_;
  std::optional<std_msgs::msg::Float64MultiArray> latest_y_;
  std::optional<std_msgs::msg::Float64MultiArray> latest_y_d_;
  std::optional<std_msgs::msg::Float64MultiArray> latest_allocation_residual_;
  std::optional<std_msgs::msg::String> latest_controller_status_;
  std::optional<std_msgs::msg::Float64> latest_aff_condition_;
  std::optional<std::array<double, 3>> latest_front_actual_;
  std::optional<std::array<double, 3>> latest_front_desired_;

  std::deque<geometry_msgs::msg::PoseStamped> actual_path_history_;
  std::deque<geometry_msgs::msg::PoseStamped> desired_path_history_;

  rclcpp::Subscription<asr_sdm_control_msgs::msg::RobotState>::SharedPtr sub_robot_state_;
  rclcpp::Subscription<asr_sdm_control_msgs::msg::ActuatorForceCmd>::SharedPtr
    sub_actuator_force_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_y_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_y_d_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_allocation_residual_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_controller_status_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_aff_condition_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_front_path_actual_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_front_path_desired_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}

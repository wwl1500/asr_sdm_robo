#include "asr_sdm_online_path_following_control/controller_state_codec_3d.hpp"
#include "asr_sdm_online_path_following_control/online_path_following_controller_3d.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include <unistd.h>

using namespace std::chrono_literals;

namespace
{

constexpr double pi_value = 3.14159265358979323846;
constexpr size_t kDiagnosticSize3D = 93;
constexpr double kDiagnosticSchemaVersion3D = 5.0;

// 单位四元数 (w,x,y,z) 转旋转矩阵
asr::Mat3 quaternionToMat3(double w, double x, double y, double z)
{
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (!(n > 1.0e-9) || !std::isfinite(n)) {
    return asr::identityFrame();
  }
  w /= n; x /= n; y /= n; z /= n;
  asr::Mat3 m;
  m.v[0][0] = 1.0 - 2.0 * (y * y + z * z);
  m.v[0][1] = 2.0 * (x * y - w * z);
  m.v[0][2] = 2.0 * (x * z + w * y);
  m.v[1][0] = 2.0 * (x * y + w * z);
  m.v[1][1] = 1.0 - 2.0 * (x * x + z * z);
  m.v[1][2] = 2.0 * (y * z - w * x);
  m.v[2][0] = 2.0 * (x * z - w * y);
  m.v[2][1] = 2.0 * (y * z + w * x);
  m.v[2][2] = 1.0 - 2.0 * (x * x + y * y);
  return m;
}

// ZYX 顺序 RPY 转旋转矩阵，用于 IMU 安装外参
asr::Mat3 rpyToMat3(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  asr::Mat3 m;
  m.v[0][0] = cy * cp;
  m.v[0][1] = cy * sp * sr - sy * cr;
  m.v[0][2] = cy * sp * cr + sy * sr;
  m.v[1][0] = sy * cp;
  m.v[1][1] = sy * sp * sr + cy * cr;
  m.v[1][2] = sy * sp * cr - cy * sr;
  m.v[2][0] = -sp;
  m.v[2][1] = cp * sr;
  m.v[2][2] = cp * cr;
  return m;
}

asr::Mat3 transposeMat3(const asr::Mat3 & m)
{
  asr::Mat3 out;
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      out.v[r][c] = m.v[c][r];
    }
  }
  return out;
}

}  // namespace

class RealtimeOnlinePathFollowingController3DNode : public rclcpp::Node
{
public:
  RealtimeOnlinePathFollowingController3DNode()
  : Node("realtime_online_path_following_controller_3d_" + std::to_string(getpid())),
    controller_(makeControllerParameters())
  {
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/asr_sdm/cmd_vel");
    controller_state_topic_ = this->declare_parameter<std::string>(
      "controller_state_topic", "/asr_sdm/online_path_controller_state_3d");
    control_cmd_topic_ = this->declare_parameter<std::string>(
      "control_cmd_topic", "/asr_sdm/control_cmd_3d");
    diagnostics_topic_ = this->declare_parameter<std::string>(
      "diagnostics_topic", "/asr_sdm/online_path_diagnostics_3d");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
    // use_imu 已在 makeControllerParameters() 中声明，这里直接读取避免重复声明
    use_imu_ = this->get_parameter("use_imu").as_bool();
    imu_mount_roll_ = this->declare_parameter<double>("imu_mount_roll", 0.0);
    imu_mount_pitch_ = this->declare_parameter<double>("imu_mount_pitch", 0.0);
    imu_mount_yaw_ = this->declare_parameter<double>("imu_mount_yaw", 0.0);
    control_period_ms_ = this->declare_parameter<int>("control_period_ms", 20);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.3);
    publish_control_cmd_ = this->declare_parameter<bool>("publish_control_cmd", false);
    max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 0.12);
    max_pitch_rate_ = this->declare_parameter<double>("max_pitch_rate", 0.7);
    max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 0.7);
    if (publish_control_cmd_) {
      throw std::runtime_error(
        "publish_control_cmd=true 尚未支持：缺少 pitch/yaw 关节速度到双丝杠寄存器的硬件映射");
    }

    state_ = controller_.makeInitialState();
    last_cmd_time_ = this->now();
    last_control_time_ = this->now();
    imu_mount_rotation_ = rpyToMat3(imu_mount_roll_, imu_mount_pitch_, imu_mount_yaw_);
    imu_mount_rotation_transpose_ = transposeMat3(imu_mount_rotation_);

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&RealtimeOnlinePathFollowingController3DNode::onTwist, this,
      std::placeholders::_1));
    if (use_imu_) {
      sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RealtimeOnlinePathFollowingController3DNode::onImu, this,
        std::placeholders::_1));
    }
    pub_controller_state_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      controller_state_topic_, rclcpp::QoS(10));
    pub_diagnostics_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&RealtimeOnlinePathFollowingController3DNode::onControlTimer, this));

    if (use_imu_) {
      RCLCPP_WARN(
        this->get_logger(),
        "在线 3D 控制器以 IMU 辅助模式启动: imu=%s, cmd_vel=%s, state=%s, diagnostics=%s。"
        "姿态/角速度来自 IMU 实测，线速度暂用命令值（非实测），诊断中会明确标记。",
        imu_topic_.c_str(), cmd_vel_topic_.c_str(), controller_state_topic_.c_str(),
        diagnostics_topic_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "在线 3D 控制器已以无传感器回退模式启动: cmd_vel=%s, state=%s, diagnostics=%s。"
        "真实历史覆盖保持为 0，严格 shape-memory 不会被误报为 ready。",
        cmd_vel_topic_.c_str(), controller_state_topic_.c_str(), diagnostics_topic_.c_str());
    }
  }

private:
  asr::OnlinePathFollowing3DParameters makeControllerParameters()
  {
    asr::OnlinePathFollowing3DParameters params;
    params.link_length = this->declare_parameter<double>("link_length", 0.25);
    params.joint_rate_limit = this->declare_parameter<double>("joint_rate_limit", 2.0);
    params.joint_limit = this->declare_parameter<double>("joint_limit", 0.85 * pi_value);
    params.joint_limit_margin = this->declare_parameter<double>("joint_limit_margin", 0.05);
    params.max_curvature = this->declare_parameter<double>("max_curvature", 1.2);
    params.curvature_tolerance = this->declare_parameter<double>("curvature_tolerance", 0.05);
    params.curvature_velocity_epsilon = this->declare_parameter<double>(
      "curvature_velocity_epsilon", 1.0e-3);
    params.damping = this->declare_parameter<double>("damping", 0.02);
    params.path_sample_ds = this->declare_parameter<double>("path_sample_ds", 0.01);
    params.min_append_distance = this->declare_parameter<double>("min_append_distance", 0.002);
    params.history_length = this->declare_parameter<double>("history_length", 1.30);
    params.min_buffer_update_speed = this->declare_parameter<double>(
      "min_buffer_update_speed", 0.005);
    params.min_shape_speed = this->declare_parameter<double>("min_shape_speed", 0.015);
    params.min_history_coverage = this->declare_parameter<double>("min_history_coverage", 0.0);
    params.joint_tracking_gain = this->declare_parameter<double>("joint_tracking_gain", 8.0);
    params.feedforward_ds = this->declare_parameter<double>("feedforward_ds", 0.02);
    params.feedforward_gain = this->declare_parameter<double>("feedforward_gain", 1.0);
    params.use_command_predictor_feedforward = this->declare_parameter<bool>(
      "use_command_predictor_feedforward", true);
    params.command_filter_time_constant = this->declare_parameter<double>(
      "command_filter_time_constant", 0.03);
    params.angular_rate_slew_limit = this->declare_parameter<double>(
      "angular_rate_slew_limit", 10.0);
    params.linear_accel_limit = this->declare_parameter<double>("linear_accel_limit", 0.5);
    params.feedforward_slope_filter_length_constant = this->declare_parameter<double>(
      "feedforward_slope_filter_length_constant", 0.03);
    params.feedforward_slope_limit = this->declare_parameter<double>(
      "feedforward_slope_limit", 100.0);
    params.max_predictor_velocity_mismatch = this->declare_parameter<double>(
      "max_predictor_velocity_mismatch", 0.20);
    params.max_predictor_direction_error_rad = this->declare_parameter<double>(
      "max_predictor_direction_error_rad", 0.50);
    params.max_predictor_angular_velocity_error = this->declare_parameter<double>(
      "max_predictor_angular_velocity_error", 0.50);
    params.predictor_min_speed = this->declare_parameter<double>(
      "predictor_min_speed", 1.0e-3);
    params.predictor_max_arc_step = this->declare_parameter<double>(
      "predictor_max_arc_step", 0.01);
    params.predictor_arc_length_tolerance = this->declare_parameter<double>(
      "predictor_arc_length_tolerance", 1.0e-4);
    const int predictor_max_iterations = this->declare_parameter<int>(
      "predictor_max_iterations", 4096);
    if (predictor_max_iterations <= 0) {
      throw std::invalid_argument("predictor_max_iterations 必须大于 0");
    }
    params.predictor_max_iterations = static_cast<size_t>(predictor_max_iterations);
    params.sensor_timeout_sec = this->declare_parameter<double>("sensor_timeout_sec", 0.15);
    params.sensor_reset_timeout_sec = this->declare_parameter<double>(
      "sensor_reset_timeout_sec", 0.50);
    params.measured_state_timeout_sec = this->declare_parameter<double>(
      "measured_state_timeout_sec", 0.15);
    params.future_state_tolerance_sec = this->declare_parameter<double>(
      "future_state_tolerance_sec", 0.01);
    params.max_sensor_sample_dt = this->declare_parameter<double>(
      "max_sensor_sample_dt", 0.10);
    params.reverse_speed_tolerance = this->declare_parameter<double>(
      "reverse_speed_tolerance", 0.005);
    params.chord_residual_tolerance = this->declare_parameter<double>(
      "chord_residual_tolerance", 1.0e-5);
    params.chord_distance_monotonicity_tolerance = this->declare_parameter<double>(
      "chord_distance_monotonicity_tolerance", 1.0e-4);
    params.max_chord_progress_per_step = this->declare_parameter<double>(
      "max_chord_progress_per_step", 0.10);
    params.min_axis_continuity_cosine = this->declare_parameter<double>(
      "min_axis_continuity_cosine", 0.5);
    params.axis_singularity_epsilon = this->declare_parameter<double>(
      "axis_singularity_epsilon", 1.0e-4);
    params.synthetic_tail_margin = this->declare_parameter<double>(
      "synthetic_tail_margin", 0.02);
    params.max_attitude_step_rad = this->declare_parameter<double>(
      "max_attitude_step_rad", 0.5);
    params.max_attitude_orthogonality_error = this->declare_parameter<double>(
      "max_attitude_orthogonality_error", 0.05);
    params.max_attitude_determinant_error = this->declare_parameter<double>(
      "max_attitude_determinant_error", 0.05);
    const int max_pending_motion_samples = this->declare_parameter<int>(
      "max_pending_motion_samples", 256);
    const int max_motion_samples_per_control_step = this->declare_parameter<int>(
      "max_motion_samples_per_control_step", 64);
    if (max_pending_motion_samples <= 0 || max_motion_samples_per_control_step <= 0) {
      throw std::invalid_argument("运动样本队列容量和单周期处理上限必须大于 0");
    }
    params.max_pending_motion_samples = static_cast<size_t>(max_pending_motion_samples);
    params.max_motion_samples_per_control_step = static_cast<size_t>(
      max_motion_samples_per_control_step);
    // 该参数在成员初始化列表中先于其它 declare_parameter 求值，此处读取 use_imu 决定运动源
    params.front_motion_mode = this->declare_parameter<bool>("use_imu", true) ?
      asr::FrontMotionMode::ImuAssisted : asr::FrontMotionMode::LocalFallbackOnly;
    params.use_local_fallback = this->declare_parameter<bool>("use_local_fallback", true);
    params.freeze_reference_when_stopped = this->declare_parameter<bool>(
      "freeze_reference_when_stopped", true);
    params.joint_acceleration_limit = this->declare_parameter<double>(
      "joint_acceleration_limit", 20.0);
    params.max_axis_alignment_tolerance = this->declare_parameter<double>(
      "max_axis_alignment_tolerance", 0.1);
    params.curvature_filter_alpha = this->declare_parameter<double>(
      "curvature_filter_alpha", 0.3);
    this->declare_parameter<int>("estimator_epoch", 1);
    this->declare_parameter<int>("tracking_frame_epoch", 1);
    return params;
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_.linear_velocity = std::clamp(msg->linear.x, 0.0, std::abs(max_linear_velocity_));
    latest_cmd_.pitch_rate = clampSymmetric(msg->angular.y, max_pitch_rate_);
    latest_cmd_.yaw_rate = clampSymmetric(msg->angular.z, max_yaw_rate_);
    last_cmd_time_ = this->now();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "在线控制 cmd_vel -> v=%.3f pitch=%.3f yaw=%.3f", latest_cmd_.linear_velocity,
      latest_cmd_.pitch_rate, latest_cmd_.yaw_rate);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & q = msg->orientation;
    const double quat_norm = std::sqrt(
      q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(quat_norm) || quat_norm < 1.0e-6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU 姿态四元数无效，已丢弃该样本");
      return;
    }
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (!(stamp > 0.0)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU 时间戳为空，已丢弃该样本");
      return;
    }

    // R_head = R_imu * R_mount^T，omega_head = R_mount * omega_imu
    const asr::Mat3 imu_frame = quaternionToMat3(q.w, q.x, q.y, q.z);
    const asr::Mat3 head_frame = asr::orthonormalize(
      asr::multiply(imu_frame, imu_mount_rotation_transpose_));
    const asr::Vec3 omega_imu{
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    const asr::Vec3 omega_head = asr::multiply(imu_mount_rotation_, omega_imu);

    asr::RealizedFrontTwist3D sample;
    sample.stamp_sec = stamp;
    sample.linear_velocity_head = {controller_.appliedCommand().linear_velocity, 0.0, 0.0};
    sample.angular_velocity_head = omega_head;
    sample.tracking_from_head = head_frame;
    sample.estimator_epoch = static_cast<uint64_t>(std::max(
        0L, this->get_parameter("estimator_epoch").as_int()));
    sample.tracking_frame_epoch = static_cast<uint64_t>(std::max(
        0L, this->get_parameter("tracking_frame_epoch").as_int()));
    sample.twist_valid = true;
    sample.attitude_valid = true;
    // 完美执行假设：指令线速度即真实线速度（§8.3 命令等价实测向量），按实测处理
    sample.linear_velocity_measured = true;
    controller_.ingestRealizedFrontTwist(sample);
  }

  double clampSymmetric(double value, double limit) const
  {
    const double abs_limit = std::abs(limit);
    return std::clamp(value, -abs_limit, abs_limit);
  }

  void onControlTimer()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {
      dt = static_cast<double>(control_period_ms_) / 1000.0;
    }
    last_control_time_ = now;

    asr::HeadCommand3D cmd = latest_cmd_;
    if ((now - last_cmd_time_).seconds() > cmd_timeout_sec_) {
      cmd = {0.0, 0.0, 0.0};
    }

    asr::MeasuredRobotState3D measured_state = controller_.makeMeasuredState(state_);
    measured_state.stamp_sec = now.seconds();
    measured_state.tracking_frame_epoch = static_cast<uint64_t>(std::max(
        0L, this->get_parameter("tracking_frame_epoch").as_int()));
    const asr::OnlineFollowerOutput3D output = controller_.step(
      cmd, now.seconds(), dt, measured_state);
    latest_joint_velocity_ = output.joint_velocity_command;
    plant_.apply(output.applied_front_command, latest_joint_velocity_, dt, state_);
    publishControllerState(output.applied_front_command, latest_joint_velocity_);
    publishDiagnostics();
  }

  void publishControllerState(
    const asr::HeadCommand3D & cmd, const asr::JointVelocity3D & joint_velocity)
  {
    std_msgs::msg::Float64MultiArray msg;
    asr::encodeControllerState3D(state_, cmd, joint_velocity, msg.data);
    pub_controller_state_->publish(msg);
  }

  void publishDiagnostics()
  {
    const auto diag = controller_.diagnostics();
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(kDiagnosticSize3D, 0.0);
    msg.data[0] = diag.buffer_ready ? 1.0 : 0.0;
    msg.data[1] = diag.using_shape_memory ? 1.0 : 0.0;
    msg.data[2] = diag.using_local_fallback ? 1.0 : 0.0;
    msg.data[3] = diag.reference_frozen ? 1.0 : 0.0;
    msg.data[4] = diag.s_joint1;
    msg.data[5] = diag.history_coverage;
    msg.data[6] = diag.commanded_speed;
    msg.data[7] = diag.limited_speed;
    msg.data[8] = diag.joint1_path_curvature;
    msg.data[9] = diag.max_axis_alignment_error;
    msg.data[10] = diag.theta_ref_error_norm;
    msg.data[11] = diag.max_theta_dot_ff;
    msg.data[12] = diag.max_theta_dot_fb;
    msg.data[13] = diag.max_theta_dot_cmd;
    msg.data[14] = diag.future_prediction_distance;
    msg.data[15] = diag.reference_valid ? 1.0 : 0.0;
    msg.data[16] = diag.required_history;
    msg.data[17] = diag.history_deficit;
    msg.data[18] = diag.joint1_path_speed;
    msg.data[19] = diag.reference_span;
    msg.data[20] = kDiagnosticSchemaVersion3D;
    msg.data[21] = static_cast<double>(
      use_imu_ ? asr::FrontMotionMode::ImuAssisted : asr::FrontMotionMode::LocalFallbackOnly);
    msg.data[22] = diag.sensor_valid ? 1.0 : 0.0;
    msg.data[23] = diag.sensor_fresh ? 1.0 : 0.0;
    msg.data[24] = diag.sensor_age_sec;
    msg.data[25] = diag.geometry_coverage;
    msg.data[26] = diag.history_coverage;
    msg.data[27] = diag.synthetic_coverage;
    msg.data[28] = diag.applied_pitch_rate;
    msg.data[29] = diag.applied_yaw_rate;
    msg.data[30] = diag.signed_joint1_path_speed;
    msg.data[31] = diag.max_chord_residual;
    msg.data[32] = diag.max_raw_dtheta_ref_ds;
    msg.data[33] = diag.max_filtered_dtheta_ref_ds;
    msg.data[34] = diag.feedforward_enabled ? 1.0 : 0.0;
    msg.data[35] = diag.reverse_command_rejected ? 1.0 : 0.0;
    msg.data[36] = diag.reverse_motion_detected ? 1.0 : 0.0;
    msg.data[37] = diag.chord_search_failed_this_step ? 1.0 : 0.0;
    msg.data[38] = static_cast<double>(diag.chord_search_failure_count);
    msg.data[39] = static_cast<double>(diag.reset_count);
    msg.data[40] = static_cast<double>(diag.estimator_epoch);
    msg.data[41] = static_cast<double>(diag.dropped_duplicate_samples);
    msg.data[42] = static_cast<double>(diag.dropped_out_of_order_samples);
    msg.data[43] = diag.limited_speed;
    msg.data[44] = diag.linear_velocity_from_command ? 1.0 : 0.0;
    msg.data[45] = diag.actual_buffered_coverage;
    msg.data[46] = diag.pending_actual_distance;
    msg.data[47] = diag.effective_actual_coverage;
    msg.data[48] = diag.predictor_disabled_due_to_mismatch ? 1.0 : 0.0;
    msg.data[49] = diag.joint_limit_guard_triggered ? 1.0 : 0.0;
    msg.data[50] = diag.curvature_guard_triggered ? 1.0 : 0.0;
    msg.data[51] = diag.axis_singularity_guard_triggered ? 1.0 : 0.0;
    msg.data[52] = diag.chord_distance_nonmonotonic ? 1.0 : 0.0;
    msg.data[53] = diag.chord_continuity_guard_triggered ? 1.0 : 0.0;
    msg.data[54] = diag.control_time_jump_detected ? 1.0 : 0.0;
    msg.data[55] = diag.sensor_queue_overflow ? 1.0 : 0.0;
    msg.data[56] = diag.max_curvature_on_reference_span;
    msg.data[57] = diag.min_axis_horizontal_norm;
    msg.data[58] = diag.predictor_velocity_mismatch;
    msg.data[59] = diag.effective_path_speed;
    msg.data[60] = diag.chord_s[0];
    msg.data[61] = diag.chord_s[1];
    msg.data[62] = diag.chord_s[2];
    msg.data[63] = diag.chord_residual[0];
    msg.data[64] = diag.chord_residual[1];
    msg.data[65] = diag.chord_residual[2];
    msg.data[66] = static_cast<double>(diag.dropped_queue_overflow_samples);
    msg.data[67] = diag.small_displacement_pending ? 1.0 : 0.0;
    msg.data[68] = diag.new_reference_valid ? 1.0 : 0.0;
    msg.data[69] = diag.active_reference_available ? 1.0 : 0.0;
    msg.data[70] = diag.feedforward_valid ? 1.0 : 0.0;
    msg.data[71] = diag.degraded_front_motion ? 1.0 : 0.0;
    msg.data[72] = diag.buffer_commit_suppressed_low_speed ? 1.0 : 0.0;
    msg.data[73] = diag.overflow_reset ? 1.0 : 0.0;
    msg.data[74] = diag.formal_actual_coverage;
    msg.data[75] = diag.degraded_coverage;
    msg.data[76] = static_cast<double>(diag.tracking_frame_epoch);
    msg.data[77] = static_cast<double>(diag.geometry_epoch);
    msg.data[78] = diag.latest_raw_curvature;
    msg.data[79] = diag.latest_filtered_curvature;
    msg.data[80] = diag.max_predicted_filtered_curvature;
    msg.data[81] = diag.max_chord_root_jump;
    msg.data[82] = diag.min_chord_axis_dot;
    msg.data[83] = diag.predictor_arc_length_error;
    msg.data[84] = diag.measured_state_age_sec;
    msg.data[85] = diag.attitude_orthogonality_error;
    msg.data[86] = diag.attitude_determinant;
    msg.data[87] = diag.root_order_invalid ? 1.0 : 0.0;
    msg.data[88] = diag.measured_state_stale ? 1.0 : 0.0;
    msg.data[89] = diag.measured_state_from_future ? 1.0 : 0.0;
    msg.data[90] = diag.tracking_frame_redefined ? 1.0 : 0.0;
    msg.data[91] = static_cast<double>(diag.motion_samples_processed);
    msg.data[92] = static_cast<double>(diag.motion_samples_backlogged);
    pub_diagnostics_->publish(msg);
  }

  std::string cmd_vel_topic_;
  std::string controller_state_topic_;
  std::string control_cmd_topic_;
  std::string diagnostics_topic_;
  std::string imu_topic_;
  int control_period_ms_;
  double cmd_timeout_sec_;
  bool publish_control_cmd_;
  bool use_imu_{true};
  double max_linear_velocity_;
  double max_pitch_rate_;
  double max_yaw_rate_;
  double imu_mount_roll_{0.0};
  double imu_mount_pitch_{0.0};
  double imu_mount_yaw_{0.0};
  asr::Mat3 imu_mount_rotation_{};
  asr::Mat3 imu_mount_rotation_transpose_{};

  asr::OnlinePathFollowingController3D controller_;
  asr::KinematicPlant3D plant_{controller_.frontUnitParameters()};
  asr::SimulationState3D state_;
  asr::HeadCommand3D latest_cmd_{0.0, 0.0, 0.0};
  asr::JointVelocity3D latest_joint_velocity_{};

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_controller_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealtimeOnlinePathFollowingController3DNode>());
  rclcpp::shutdown();
  return 0;
}

#ifndef ONLINE_PATH_FOLLOWING_CONTROLLER_3D_HPP_
#define ONLINE_PATH_FOLLOWING_CONTROLLER_3D_HPP_

#include "asr_sdm_head_following_control/front_unit_following_controller_3d.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

namespace asr
{

enum class FrontMotionMode : uint8_t
{
  MeasuredTwistRequired = 0,
  LocalFallbackOnly = 1,
  ImuWithCommandedLinearSpeed = 2,
  ActualRequired = MeasuredTwistRequired,
  ImuAssisted = ImuWithCommandedLinearSpeed
};

struct RealizedFrontTwist3D
{
  double stamp_sec{0.0};
  Vec3 linear_velocity_head{0.0, 0.0, 0.0};
  Vec3 angular_velocity_head{0.0, 0.0, 0.0};
  Vec3 linear_acceleration_head{0.0, 0.0, 0.0};
  Vec3 angular_acceleration_head{0.0, 0.0, 0.0};
  // 将 head-frame 中的向量变换到固定 tracking frame。
  Mat3 tracking_from_head{};
  uint64_t estimator_epoch{0};
  uint64_t tracking_frame_epoch{0};
  bool twist_valid{false};
  bool attitude_valid{false};
  bool linear_velocity_measured{true};
  bool angular_velocity_measured{true};
  bool acceleration_valid{false};
};

struct OnlinePathFollowing3DParameters
{
  double link_length{0.25};
  double joint_rate_limit{2.0};
  double joint_limit{2.670353755551324};
  double joint_limit_margin{0.05};
  double max_curvature{1.2};
  double curvature_tolerance{0.05};
  double curvature_velocity_epsilon{1.0e-3};
  double curvature_filter_alpha{0.3};
  double damping{0.02};

  double path_sample_ds{0.01};
  double min_append_distance{0.002};
  double history_length{1.30};
  double min_buffer_update_speed{0.005};
  double min_shape_speed{0.015};
  double min_history_coverage{0.0};

  double joint_tracking_gain{8.0};
  double feedforward_ds{0.02};
  double feedforward_gain{1.0};
  double feedforward_slope_filter_length_constant{0.03};
  double feedforward_slope_limit{100.0};
  double max_predictor_velocity_mismatch{0.20};
  double max_predictor_direction_error_rad{0.50};
  double max_predictor_angular_velocity_error{0.50};
  double predictor_min_speed{1.0e-3};
  double predictor_max_arc_step{0.01};
  double predictor_arc_length_tolerance{1.0e-4};
  size_t predictor_max_iterations{4096};
  bool use_command_predictor_feedforward{true};

  double command_filter_time_constant{0.03};
  double angular_rate_slew_limit{10.0};
  double linear_accel_limit{0.5};

  double sensor_timeout_sec{0.15};
  double sensor_reset_timeout_sec{0.50};
  double measured_state_timeout_sec{0.15};
  double future_state_tolerance_sec{0.01};
  double max_sensor_sample_dt{0.10};
  double reverse_speed_tolerance{0.005};
  double chord_residual_tolerance{1.0e-5};
  double chord_distance_monotonicity_tolerance{1.0e-4};
  double max_chord_progress_per_step{0.10};
  double min_axis_continuity_cosine{0.5};
  double axis_singularity_epsilon{1.0e-4};
  double synthetic_tail_margin{0.02};
  double max_attitude_step_rad{0.5};
  double max_attitude_orthogonality_error{0.05};
  double max_attitude_determinant_error{0.05};
  size_t max_pending_motion_samples{256};
  size_t max_motion_samples_per_control_step{64};
  FrontMotionMode front_motion_mode{FrontMotionMode::MeasuredTwistRequired};

  double joint_acceleration_limit{20.0};
  bool use_local_fallback{true};
  bool freeze_reference_when_stopped{true};
  double max_axis_alignment_tolerance{0.1};

  // 文档 §8.6 多分支树搜索参数
  bool use_multi_branch_reconstruction{true};  // 启用三层分支树
  size_t max_root_candidates_per_parent{3};    // 每个父节点最多生成的候选数 (K_root,max)
  size_t max_segments_scanned{128};            // N_seg,k ≤ N_seg,max (8.105)
  size_t max_tree_nodes{512};                  // N_tree,k ≤ N_tree,max (8.105)
  double root_search_step{0.005};              // 弦根搜索步长（小于 path_sample_ds）
  double axis_enter_threshold{0.7};            // ε_axis,enter：严格分支的最小轴投影 (8.106)
  double axis_exit_threshold{0.5};             // ε_axis,exit：降级阈值 (8.133)
  double branch_ambiguity_margin{0.05};        // ε_J,amb：分支歧义容差 (8.118)

  // 评分权重 (8.115)
  double score_weight_residual{1.0};           // w_δ：残差项权重
  double score_weight_curvature{1.0};          // w_κ：曲率项权重
  double score_weight_monotonicity{10.0};      // w_m：单调性权重
  double score_weight_prior{0.5};              // λ^prior：prior 连续性权重
  double score_weight_axis_continuity{1.0};    // prior 内的轴连续性权重
  double score_weight_root_continuity{1.0};    // prior 内的根连续性权重

  // 连续性守卫 (8.131)
  double max_strict_theta_jump{0.2};           // Δθ_strict,max：严格连续性最大角度跳变
  double max_strict_root_jump{0.15};           // ΔΔ_strict,max：严格根跨度跳变
  double min_strict_axis_dot{0.85};            // c_strict,min：严格轴内积下限

  // 降级连续性守卫 (8.139)
  double max_degraded_theta_jump{0.5};         // Δθ_deg,max
  double max_degraded_root_jump{0.30};         // ΔΔ_deg,max
  double min_degraded_axis_dot{0.6};           // c_deg,min

  // 严格连续性停滞安全阀 (8.132)：当存在几何上严格可行的分支，但连续性守卫
  // 连续多步拒绝它时，强制接受该分支并重新锚定 prior，避免因过时 prior
  // 导致的永久性锁死（例如急转弯后 prior 与新几何不再匹配）。
  size_t max_strict_continuity_stall_count{5};  // Δ_stall,max
};

struct OnlinePathFollowing3DDiagnostics
{
  bool buffer_ready{false};
  bool using_shape_memory{false};
  bool using_local_fallback{false};
  bool new_reference_valid{false};
  bool active_reference_available{false};
  bool reference_frozen{false};
  bool reference_valid{false};
  bool feedforward_valid{false};
  bool sensor_valid{false};
  bool sensor_fresh{false};
  bool degraded_front_motion{false};
  bool buffer_commit_suppressed_low_speed{false};
  bool overflow_reset{false};
  bool reverse_command_rejected{false};
  bool reverse_motion_detected{false};
  bool chord_search_failed_this_step{false};
  bool feedforward_enabled{false};
  bool linear_velocity_from_command{false};
  bool small_displacement_pending{false};
  bool predictor_frame_consistent{true};
  bool predictor_disabled_due_to_mismatch{false};
  bool joint_limit_guard_triggered{false};
  bool curvature_guard_triggered{false};
  bool axis_singularity_guard_triggered{false};
  bool chord_distance_nonmonotonic{false};
  bool chord_continuity_guard_triggered{false};
  bool root_order_invalid{false};
  bool control_time_jump_detected{false};
  bool sensor_queue_overflow{false};
  bool invalid_rotation_matrix{false};
  bool measured_state_stale{false};
  bool measured_state_from_future{false};
  bool tracking_frame_redefined{false};
  bool buffer_capacity_overflow{false};
  bool joint_command_infeasible{false};
  bool geometry_budget_exceeded{false};  // 文档 (8.105) 预算守卫标志
  bool predictor_arc_length_invalid{false};
  bool branch_selection_ambiguous{false};      // 文档 (8.118)：最优/次优分支得分差小于 ε_J,amb
  bool strict_continuity_stall_override{false}; // 文档 (8.132)：触发停滞安全阀，强制重新锚定 prior

  double s_joint1{0.0};
  double history_coverage{0.0};
  double formal_actual_coverage{0.0};
  double degraded_coverage{0.0};
  double actual_buffered_coverage{0.0};
  double pending_actual_distance{0.0};
  double effective_actual_coverage{0.0};
  double geometry_coverage{0.0};
  double synthetic_coverage{0.0};
  double commanded_speed{0.0};
  double limited_speed{0.0};
  double applied_pitch_rate{0.0};
  double applied_yaw_rate{0.0};
  double joint1_path_curvature{0.0};
  double latest_raw_curvature{0.0};
  double latest_filtered_curvature{0.0};
  double max_predicted_filtered_curvature{0.0};
  double max_axis_alignment_error{0.0};
  double theta_ref_error_norm{0.0};
  double max_theta_dot_ff{0.0};
  double max_theta_dot_fb{0.0};
  double max_theta_dot_cmd{0.0};
  double future_prediction_distance{0.0};
  double predictor_arc_length_error{0.0};
  double required_history{0.0};
  double history_deficit{0.0};
  double joint1_path_speed{0.0};
  double signed_joint1_path_speed{0.0};
  double effective_path_speed{0.0};
  double reference_span{0.0};
  double sensor_age_sec{0.0};
  double measured_state_age_sec{0.0};
  double attitude_orthogonality_error{0.0};
  double attitude_determinant{1.0};
  double max_chord_residual{0.0};
  double max_chord_root_jump{0.0};
  double min_chord_axis_dot{1.0};
  double max_raw_dtheta_ref_ds{0.0};
  double max_filtered_dtheta_ref_ds{0.0};
  double max_curvature_on_reference_span{0.0};
  double min_axis_horizontal_norm{1.0};
  double predictor_velocity_mismatch{0.0};
  double predictor_speed_mismatch{0.0};
  double predictor_direction_error_rad{0.0};
  double predictor_angular_velocity_error{0.0};
  double predictor_velocity_vector_error{0.0};
  double branch_selection_margin{0.0};  // 文档 (8.118)：最优与次优分支的得分差
  std::array<double, 3> chord_s{};
  std::array<double, 3> chord_residual{};

  uint64_t estimator_epoch{0};
  uint64_t tracking_frame_epoch{0};
  uint64_t geometry_epoch{0};
  size_t chord_search_failure_count{0};
  size_t reset_count{0};
  size_t dropped_duplicate_samples{0};
  size_t dropped_out_of_order_samples{0};
  size_t dropped_queue_overflow_samples{0};
  size_t motion_samples_processed{0};
  size_t motion_samples_backlogged{0};
};

struct OnlineFollowerOutput3D
{
  JointVelocity3D joint_velocity_command{};
  HeadCommand3D applied_front_command{0.0, 0.0, 0.0};
  OnlinePathFollowing3DDiagnostics diagnostics{};
};

// 第八章 8.1～8.6 节的几何滚动窗口、弦长形状重构和关节速度参考控制器。
// 本类不包含 8.7～8.8 节的动力学模型、10 维执行器 OCP 或 MPC 求解器。
class OnlinePathFollowingController3D
{
public:
  explicit OnlinePathFollowingController3D(const OnlinePathFollowing3DParameters & params);
  ~OnlinePathFollowingController3D();

  OnlinePathFollowingController3D(const OnlinePathFollowingController3D &) = delete;
  OnlinePathFollowingController3D & operator=(const OnlinePathFollowingController3D &) = delete;
  OnlinePathFollowingController3D(OnlinePathFollowingController3D &&) = delete;
  OnlinePathFollowingController3D & operator=(OnlinePathFollowingController3D &&) = delete;

  SimulationState3D makeInitialState() const;
  MeasuredRobotState3D makeMeasuredState(const SimulationState3D & state) const;
  FrontUnitController3DParameters frontUnitParameters() const;
  HeadCommand3D limitCommand(const HeadCommand3D & cmd) const;
  bool ingestRealizedFrontTwist(const RealizedFrontTwist3D & sample);
  void notifyEstimatorReset(uint64_t estimator_epoch);
  OnlineFollowerOutput3D step(
    const HeadCommand3D & cmd, double control_time_sec, double dt,
    const MeasuredRobotState3D & measured_state);

  JointVelocity3D step(const HeadCommand3D & cmd, double dt, SimulationState3D & state);
  JointVelocity3D step(
    const HeadCommand3D & cmd, double control_time_sec, double dt, SimulationState3D & state);
  OnlinePathFollowing3DDiagnostics diagnostics() const;
  HeadCommand3D appliedCommand() const;

private:
  class RollingPathBuffer3D;
  struct PredictedPathSegment3D;

  enum class GeometryResetReason : uint8_t
  {
    EstimatorReset,
    TrackingFrameReset,
    ControlTimeJump,
    SensorGap,
    AttitudeJump,
    QueueOverflow,
    ReverseMotion,
    SensorTimeout
  };

  // 文档 §8.6 引用状态 (8.144)
  enum class ReferenceStatus : uint8_t
  {
    NoReference,         // 无可用引用
    ExactNew,           // 新的严格引用（χ_k^valid=1, z_k^axis=STRICT）
    DegradedContinuous, // 降级连续引用（χ_k^deg.cont=1）
    HoldLast            // 保持上一个有效引用（χ_k^hold=1）
  };

  // 文档 §8.6 单个弦根候选 (分支树节点)
  struct ChordRootCandidate
  {
    double s{0.0};            // 弦根弧长坐标
    Vec3 point{};             // J1 路径上的点
    Vec3 axis{};              // 归一化弦轴 (front - back)
    double residual{0.0};     // |‖chord‖ - L| 残差
    bool valid{false};        // 候选是否通过初步守卫
  };

  // 文档 §8.6 完整分支 (深度3，包含三个连续弦根 s_0^root > s_1 > s_2 > s_3)
  struct ReconstructionBranch
  {
    std::array<double, 3> chord_s{};           // [s_1, s_2, s_3] 三个弦根弧长
    std::array<Vec3, 3> chord_axes{};          // 归一化弦轴向量
    std::array<double, 3> chord_residual{};    // 残差
    std::array<double, kNum3dJointDofs> theta_ref{}; // IK 提取的关节角
    std::array<Mat3, kNum3dLinks> link_frames_ref{}; // 重建的连杆坐标系（4个link frame）

    double reference_span{0.0};                // s_0^root - s_3
    double max_curvature_on_span{0.0};         // 跨度内最大曲率
    double max_axis_alignment_error{0.0};      // IK 轴对齐误差
    double min_axis_projection{1.0};           // ρ_min,k (8.126)
    std::array<double, 3> monotonicity_violation{}; // 每段弦长单调性超差量 (8.92)

    // 评分组件 (8.115)
    double J_data{0.0};      // 残差/曲率/单调性得分
    double J_prior{0.0};     // prior 连续性得分
    double J_branch{0.0};    // 总分 = J_data + λ^prior * J_prior
    double branch_margin{0.0}; // 与次优分支的得分差 (8.118)

    // 守卫标志
    bool monotonicity_ok{true};
    bool residual_ok{true};
    bool curvature_ok{true};
    bool joint_limit_ok{true};
    bool axis_projection_ok{true};  // ρ_r,k ≥ ε_axis.enter for all r (8.126)
    bool continuity_ok{true};       // χ_k^strict.cont (8.131)

    // 状态标志
    bool is_strict{false};       // z_r,k^axis = STRICT for all r (8.140)
    bool is_degraded{false};     // z_r,k^axis = DEGRADED (8.140)
    bool is_common_admissible{false}; // χ_k^common.pre (8.106)
    bool is_hard_admissible{false};   // 通过所有 non-degradable 守卫 (8.107)

    bool valid{false};  // 最终是否可用
  };

  struct ShapeReferenceCandidate3D
  {
    std::array<double, kNum3dJointDofs> theta_ref{};
    std::array<double, kNum3dJointDofs> dtheta_ref_ds{};
    std::array<double, 3> chord_s{};
    std::array<double, 3> chord_residual{};
    std::array<Vec3, 3> chord_axes{};
    double max_axis_alignment_error{0.0};
    double curvature{0.0};
    double reference_span{0.0};
    bool current_geometry_valid{false};
    bool feedforward_valid{false};
    ReferenceStatus status{ReferenceStatus::NoReference};
  };

  // 文档 §8.6 held reference record (8.135)
  struct HeldReferenceRecord
  {
    std::array<double, kNum3dJointDofs> theta_held{};
    std::array<double, 3> chord_s_held{};
    std::array<Vec3, 3> chord_axes_held{};
    uint64_t geometry_epoch{0};
    bool valid{false};
  };

  FrontUnitController3DParameters makeFrontUnitParameters() const;
  void validateParameters() const;
  bool geometryState(
    const MeasuredRobotState3D & measured_state, SimulationState3D & state) const;

  Vec3 joint1Point(const SimulationState3D & state) const;
  Vec3 commandedJoint1Velocity(
    const SimulationState3D & state, const HeadCommand3D & cmd) const;
  Mat3 joint1FrameHint(const SimulationState3D & state, const HeadCommand3D & cmd) const;
  double requiredHistory() const;
  void resetBufferIfNeeded(const SimulationState3D & state, const HeadCommand3D & cmd);
  void beginNewGeometryEpoch(
    GeometryResetReason reason, const SimulationState3D & state,
    const HeadCommand3D & cmd, bool preserve_active_reference,
    bool clear_pending_samples = true);
  void processRealizedMotion(
    double control_time_sec, const SimulationState3D & state, const HeadCommand3D & cmd);
  bool processRealizedSample(
    const RealizedFrontTwist3D & sample,
    const SimulationState3D & state, const HeadCommand3D & cmd);
  Vec3 realizedJoint1Velocity(const RealizedFrontTwist3D & sample) const;
  void anchorMotionSample(
    const RealizedFrontTwist3D & sample, const Vec3 & velocity,
    bool formal_actual, const Mat3 & measured_frame);
  void warmupBuffer(const SimulationState3D & state, const HeadCommand3D & cmd);

  bool computeShapeReference(
    const SimulationState3D & state, const HeadCommand3D & cmd, double s_joint1,
    ShapeReferenceCandidate3D & candidate);
  // 新的三层分支树实现 (文档 §8.6)
  bool computeShapeReferenceMultiBranch(
    const SimulationState3D & state, const HeadCommand3D & cmd, double s_joint1,
    ShapeReferenceCandidate3D & candidate);

  std::vector<ChordRootCandidate> generateRootCandidates(
    const SimulationState3D & state, const HeadCommand3D & cmd,
    double parent_s, const Vec3 & parent_point) const;

  bool expandBranch(
    const SimulationState3D & state, const HeadCommand3D & cmd,
    const std::array<double, 4> & s_roots, const Mat3 & link0_frame,
    ReconstructionBranch & branch) const;

  // 文档 (8.90-8.94)：复核 [child_s, parent_s] 区间内弦长关于 s 的单调性，
  // 返回超出 chord_distance_monotonicity_tolerance 的最大回退幅度（0 表示单调）。
  bool checkChordMonotonicity(
    const SimulationState3D & state, const HeadCommand3D & cmd,
    double parent_s, const Vec3 & parent_point, double child_s,
    double & violation) const;

  void scoreBranch(
    ReconstructionBranch & branch,
    const HeldReferenceRecord & held_prior) const;

  ReferenceStatus evaluateFallbackStatus() const;

  bool selectBestBranch(
    const std::vector<ReconstructionBranch> & branches,
    size_t & best_index, double & margin) const;

  // 旧的单分支实现（保留作为 fallback）
  bool computeThetaReferenceAt(
    const SimulationState3D & state, const HeadCommand3D & cmd, double s_joint1,
    const Mat3 & link0_frame,
    std::array<double, kNum3dJointDofs> & theta_ref,
    double & max_axis_alignment_error, double & curvature, double & reference_span,
    std::array<double, 3> * chord_s = nullptr,
    std::array<double, 3> * chord_residual = nullptr,
    std::array<Vec3, 3> * chord_axes = nullptr) const;
  bool findPreviousChordPoint(
    const SimulationState3D & state, const HeadCommand3D & cmd,
    double front_s, const Vec3 & front_point, double & back_s, Vec3 & back_point,
    double previous_chord_s = std::numeric_limits<double>::quiet_NaN()) const;
  bool sampleJoint1Path(
    const SimulationState3D & state, const HeadCommand3D & cmd, double s,
    Vec3 & p, Vec3 & tangent, double & curvature, Mat3 & bishop_frame) const;
  bool buildPredictedPathSegment(
    const SimulationState3D & state, const HeadCommand3D & cmd, double max_delta_s) const;
  bool predictFutureJoint1(
    const SimulationState3D & state, const HeadCommand3D & cmd, double delta_s,
    Vec3 & p, Vec3 & tangent, double & curvature, Mat3 & bishop_frame,
    Mat3 * predicted_head_frame = nullptr) const;

  JointVelocity3D applyShapeReference(
    const std::array<double, kNum3dJointDofs> & theta_ref,
    const std::array<double, kNum3dJointDofs> & dtheta_ref_ds,
    double joint1_path_speed, double dt, const MeasuredRobotState3D & measured_state);
  JointVelocity3D applyFrozenReference(
    double dt, const MeasuredRobotState3D & measured_state);
  HeadCommand3D filterCommand(const HeadCommand3D & cmd, double dt);
  void resetFeedforwardFilter();

  OnlinePathFollowing3DParameters params_;
  std::unique_ptr<RollingPathBuffer3D> path_buffer_;
  mutable std::unique_ptr<PredictedPathSegment3D> predicted_path_segment_;
  FrontUnitFollowingController3D local_controller_;
  KinematicPlant3D plant_;
  mutable OnlinePathFollowing3DDiagnostics diag_;
  HeadCommand3D filtered_cmd_{0.0, 0.0, 0.0};
  HeadCommand3D applied_cmd_{0.0, 0.0, 0.0};
  std::array<double, kNum3dJointDofs> filtered_dtheta_ref_ds_{};
  std::array<double, kNum3dJointDofs> last_theta_ref_{};
  std::array<double, kNum3dJointDofs> last_theta_dot_{};
  std::array<double, 3> previous_chord_s_{};
  std::array<Vec3, 3> previous_chord_axes_{};
  HeldReferenceRecord held_reference_{}; // 文档 (8.135) held prior
  ReferenceStatus last_reference_status_{ReferenceStatus::NoReference};
  size_t strict_only_continuity_reject_count_{0}; // 文档 (8.132) 计数器
  std::deque<RealizedFrontTwist3D> pending_motion_samples_;
  mutable std::mutex motion_sample_mutex_;
  RealizedFrontTwist3D previous_motion_sample_{};
  Vec3 previous_realized_joint1_velocity_{0.0, 0.0, 0.0};
  Vec3 latest_realized_joint1_velocity_{0.0, 0.0, 0.0};
  Vec3 integrated_joint1_position_{0.0, 0.0, 0.0};
  Vec3 last_buffered_joint1_position_{0.0, 0.0, 0.0};
  double latest_signed_joint1_speed_{0.0};
  Mat3 last_realized_head_frame_{};
  double last_control_time_sec_{0.0};
  double last_sensor_stamp_sec_{0.0};
  uint64_t estimator_epoch_{0};
  uint64_t tracking_frame_epoch_{0};
  uint64_t geometry_epoch_{0};
  mutable std::atomic<size_t> chord_search_failure_count_{0};
  size_t reset_count_{0};
  std::atomic<size_t> dropped_duplicate_samples_{0};
  std::atomic<size_t> dropped_out_of_order_samples_{0};
  std::atomic<size_t> dropped_queue_overflow_samples_{0};
  bool queue_overflow_requires_reset_{false};
  bool estimator_reset_requires_reset_{false};
  bool has_previous_motion_sample_{false};
  bool previous_motion_was_formal_actual_{false};
  bool has_last_realized_head_frame_{false};
  bool latest_linear_velocity_from_command_{false};
  bool has_control_time_{false};
  bool has_last_theta_ref_{false};
  bool has_previous_chord_solution_{false};
  bool buffer_motion_was_stopped_{false};
  bool reverse_motion_active_{false};
};

}  // namespace asr

#endif  // ONLINE_PATH_FOLLOWING_CONTROLLER_3D_HPP_

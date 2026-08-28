#include "asr_sdm_online_path_following_control/online_path_following_controller_3d.hpp"

#include "asr_sdm_head_following_control/control_utils.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace asr
{

// 原 online_path_geometry_3d_internal.{hpp,cpp} 内容：四元数与旋转矩阵校验的
// 内部辅助工具，仅供本文件使用，不对外暴露。
namespace internal
{

struct QuaternionWxyz
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RotationValidation
{
  bool valid{false};
  double orthogonality_error{0.0};
  double determinant{0.0};
};

namespace
{

double quaternionDot(const QuaternionWxyz & lhs, const QuaternionWxyz & rhs)
{
  return lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

QuaternionWxyz scaledSum(
  const QuaternionWxyz & lhs, double lhs_scale,
  const QuaternionWxyz & rhs, double rhs_scale)
{
  return {
    lhs_scale * lhs.w + rhs_scale * rhs.w,
    lhs_scale * lhs.x + rhs_scale * rhs.x,
    lhs_scale * lhs.y + rhs_scale * rhs.y,
    lhs_scale * lhs.z + rhs_scale * rhs.z};
}

}  // namespace

QuaternionWxyz normalizeQuaternion(const QuaternionWxyz & q)
{
  const double n = std::sqrt(quaternionDot(q, q));
  if (!(n > 1.0e-15) || !std::isfinite(n)) {
    return {};
  }
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}

QuaternionWxyz multiplyQuaternion(
  const QuaternionWxyz & lhs, const QuaternionWxyz & rhs)
{
  return {
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w};
}

QuaternionWxyz quaternionFromMatrix(const Mat3 & matrix)
{
  QuaternionWxyz q;
  const double trace = matrix.v[0][0] + matrix.v[1][1] + matrix.v[2][2];
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(std::max(0.0, trace + 1.0));
    if (!(scale > 1.0e-15)) {
      return {};
    }
    q.w = 0.25 * scale;
    q.x = (matrix.v[2][1] - matrix.v[1][2]) / scale;
    q.y = (matrix.v[0][2] - matrix.v[2][0]) / scale;
    q.z = (matrix.v[1][0] - matrix.v[0][1]) / scale;
  } else if (matrix.v[0][0] > matrix.v[1][1] &&
    matrix.v[0][0] > matrix.v[2][2])
  {
    const double scale = 2.0 * std::sqrt(std::max(
        0.0, 1.0 + matrix.v[0][0] - matrix.v[1][1] - matrix.v[2][2]));
    if (!(scale > 1.0e-15)) {
      return {};
    }
    q.w = (matrix.v[2][1] - matrix.v[1][2]) / scale;
    q.x = 0.25 * scale;
    q.y = (matrix.v[0][1] + matrix.v[1][0]) / scale;
    q.z = (matrix.v[0][2] + matrix.v[2][0]) / scale;
  } else if (matrix.v[1][1] > matrix.v[2][2]) {
    const double scale = 2.0 * std::sqrt(std::max(
        0.0, 1.0 + matrix.v[1][1] - matrix.v[0][0] - matrix.v[2][2]));
    if (!(scale > 1.0e-15)) {
      return {};
    }
    q.w = (matrix.v[0][2] - matrix.v[2][0]) / scale;
    q.x = (matrix.v[0][1] + matrix.v[1][0]) / scale;
    q.y = 0.25 * scale;
    q.z = (matrix.v[1][2] + matrix.v[2][1]) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(std::max(
        0.0, 1.0 + matrix.v[2][2] - matrix.v[0][0] - matrix.v[1][1]));
    if (!(scale > 1.0e-15)) {
      return {};
    }
    q.w = (matrix.v[1][0] - matrix.v[0][1]) / scale;
    q.x = (matrix.v[0][2] + matrix.v[2][0]) / scale;
    q.y = (matrix.v[1][2] + matrix.v[2][1]) / scale;
    q.z = 0.25 * scale;
  }
  return normalizeQuaternion(q);
}

Mat3 matrixFromQuaternion(const QuaternionWxyz & input)
{
  const QuaternionWxyz q = normalizeQuaternion(input);
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  Mat3 matrix{};
  matrix.v[0][0] = 1.0 - 2.0 * (yy + zz);
  matrix.v[0][1] = 2.0 * (xy - wz);
  matrix.v[0][2] = 2.0 * (xz + wy);
  matrix.v[1][0] = 2.0 * (xy + wz);
  matrix.v[1][1] = 1.0 - 2.0 * (xx + zz);
  matrix.v[1][2] = 2.0 * (yz - wx);
  matrix.v[2][0] = 2.0 * (xz - wy);
  matrix.v[2][1] = 2.0 * (yz + wx);
  matrix.v[2][2] = 1.0 - 2.0 * (xx + yy);
  return matrix;
}

QuaternionWxyz shortestArcSlerp(
  const QuaternionWxyz & from_input, const QuaternionWxyz & to_input, double alpha)
{
  const QuaternionWxyz from = normalizeQuaternion(from_input);
  QuaternionWxyz to = normalizeQuaternion(to_input);
  double cosine = quaternionDot(from, to);
  if (cosine < 0.0) {
    to = {-to.w, -to.x, -to.y, -to.z};
    cosine = -cosine;
  }
  cosine = std::clamp(cosine, -1.0, 1.0);
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  if (cosine > 1.0 - 1.0e-10) {
    return normalizeQuaternion(scaledSum(from, 1.0 - clamped_alpha, to, clamped_alpha));
  }
  const double angle = std::acos(cosine);
  const double sine = std::sin(angle);
  if (!(std::abs(sine) > 1.0e-15)) {
    return from;
  }
  return normalizeQuaternion(scaledSum(
      from, std::sin((1.0 - clamped_alpha) * angle) / sine,
      to, std::sin(clamped_alpha * angle) / sine));
}

RotationValidation validateRotation(
  const Mat3 & matrix, double max_orthogonality_error,
  double max_determinant_error)
{
  RotationValidation result;
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      if (!std::isfinite(matrix.v[row][col])) {
        result.orthogonality_error = std::numeric_limits<double>::infinity();
        result.determinant = std::numeric_limits<double>::quiet_NaN();
        return result;
      }
    }
  }

  double squared_error = 0.0;
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      double value = 0.0;
      for (size_t k = 0; k < 3; ++k) {
        value += matrix.v[k][row] * matrix.v[k][col];
      }
      const double expected = row == col ? 1.0 : 0.0;
      const double error = value - expected;
      squared_error += error * error;
    }
  }
  result.orthogonality_error = std::sqrt(squared_error);
  result.determinant =
    matrix.v[0][0] * (matrix.v[1][1] * matrix.v[2][2] - matrix.v[1][2] * matrix.v[2][1]) -
    matrix.v[0][1] * (matrix.v[1][0] * matrix.v[2][2] - matrix.v[1][2] * matrix.v[2][0]) +
    matrix.v[0][2] * (matrix.v[1][0] * matrix.v[2][1] - matrix.v[1][1] * matrix.v[2][0]);
  result.valid = std::isfinite(result.orthogonality_error) &&
    std::isfinite(result.determinant) &&
    result.orthogonality_error <= max_orthogonality_error &&
    std::abs(result.determinant - 1.0) <= max_determinant_error;
  return result;
}

}  // namespace internal

namespace
{

HeadCommand3D clampCommandDelta(
  const HeadCommand3D & previous, const HeadCommand3D & target,
  double linear_delta_limit, double angular_delta_limit)
{
  auto clamp_delta = [](double prev, double next, double limit) {
      const double abs_limit = std::max(0.0, limit);
      return prev + std::clamp(next - prev, -abs_limit, abs_limit);
    };
  return {
    clamp_delta(previous.linear_velocity, target.linear_velocity, linear_delta_limit),
    clamp_delta(previous.pitch_rate, target.pitch_rate, angular_delta_limit),
    clamp_delta(previous.yaw_rate, target.yaw_rate, angular_delta_limit)};
}

bool isFinite(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

Mat3 transpose(const Mat3 & m)
{
  Mat3 out{};
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 3; ++col) {
      out.v[row][col] = m.v[col][row];
    }
  }
  return out;
}

Vec3 lerp(const Vec3 & a, const Vec3 & b, double alpha)
{
  return a * (1.0 - alpha) + b * alpha;
}

Mat3 slerpFrame(const Mat3 & a, const Mat3 & b, double alpha)
{
  return internal::matrixFromQuaternion(internal::shortestArcSlerp(
      internal::quaternionFromMatrix(a), internal::quaternionFromMatrix(b), alpha));
}

Vec3 rotateAroundAxis(const Vec3 & value, const Vec3 & axis, double angle)
{
  const double axis_norm = norm(axis);
  if (axis_norm < 1.0e-12) {
    return value;
  }
  const Vec3 k = axis / axis_norm;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return value * c + cross(k, value) * s + k * dot(k, value) * (1.0 - c);
}

Mat3 makeBishopFrameFromTangent(const Vec3 & tangent, const Mat3 * previous_frame = nullptr)
{
  const Vec3 ex = normalize(tangent);
  Vec3 ey{};
  if (previous_frame) {
    ey = projectPerpendicular(column(*previous_frame, 1), ex);
    if (norm(ey) < 1.0e-9) {
      ey = projectPerpendicular(column(*previous_frame, 2), ex);
    }
  }
  if (!previous_frame || norm(ey) < 1.0e-9) {
    const Vec3 reference = std::abs(dot(ex, {0.0, 0.0, 1.0})) > 0.92 ?
      Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0};
    ey = projectPerpendicular(reference, ex);
  }
  if (norm(ey) < 1.0e-9) {
    return frameFromAxis(ex);
  }
  ey = normalize(ey);
  return fromColumns(ex, ey, cross(ex, ey));
}

Mat3 makeFrameFromAxisAndReference(const Vec3 & axis, const Mat3 & reference_frame)
{
  const Vec3 ex = normalize(axis);
  Vec3 ey = projectPerpendicular(column(reference_frame, 1), ex);
  if (norm(ey) < 1.0e-9) {
    ey = projectPerpendicular(column(reference_frame, 2), ex);
  }
  if (norm(ey) < 1.0e-9) {
    return frameFromAxis(ex);
  }
  ey = normalize(ey);
  return fromColumns(ex, ey, cross(ex, ey));
}

// 沿曲线平行输运 Bishop frame：保持 ex 与切向对齐，ey 绕 ex 旋转最小角度
Mat3 advanceBishopFrame(
  const Mat3 & previous_frame, const Vec3 & previous_tangent, const Vec3 & current_tangent)
{
  const Vec3 axis = cross(previous_tangent, current_tangent);
  const double axis_norm = norm(axis);
  Vec3 ey = column(previous_frame, 1);
  if (axis_norm > 1.0e-8) {
    const double angle = std::atan2(axis_norm, dot(previous_tangent, current_tangent));
    ey = rotateAroundAxis(ey, axis, angle);
  }
  ey = projectPerpendicular(ey, current_tangent);
  if (norm(ey) < 1.0e-9) {
    return makeBishopFrameFromTangent(current_tangent, &previous_frame);
  }
  ey = normalize(ey);
  return fromColumns(current_tangent, ey, cross(current_tangent, ey));
}

bool computeThetaFromLinkAxes(
  const Mat3 & link0_frame,
  const std::array<Vec3, kNum3dLinks - 1> & desired_downstream_axes,
  double axis_singularity_epsilon,
  std::array<double, kNum3dJointDofs> & theta_ref,
  std::array<Mat3, kNum3dLinks> * realized_frames,
  double * max_axis_alignment_error,
  double * min_local_horizontal_norm)
{
  std::array<Mat3, kNum3dLinks> frames{};
  frames[0] = orthonormalize(link0_frame);
  double max_error = 0.0;
  double min_horizontal = std::numeric_limits<double>::infinity();

  for (size_t joint = 0; joint < kNum3dJoints; ++joint) {
    const Vec3 desired_axis = desired_downstream_axes[joint];
    const double desired_norm = norm(desired_axis);
    if (!(std::isfinite(desired_norm)) || desired_norm < 1.0e-9) {
      return false;
    }

    const Vec3 desired_unit = desired_axis / desired_norm;
    const Vec3 local_axis = multiply(transpose(frames[joint]), desired_unit);
    if (!(std::isfinite(local_axis.x) && std::isfinite(local_axis.y) &&
      std::isfinite(local_axis.z)))
    {
      return false;
    }

    // 在父连杆局部坐标系中检查俯仰-偏航奇异性：
    // 当轴在父连杆 x-y 平面上的投影接近零时，pitch 接近 ±π/2，yaw 退化
    const double local_horizontal = std::hypot(local_axis.x, local_axis.y);
    min_horizontal = std::min(min_horizontal, local_horizontal);
    if (local_horizontal < axis_singularity_epsilon) {
      return false;
    }

    const double pitch = std::atan2(
      local_axis.z, local_horizontal);
    const double yaw = std::atan2(-local_axis.y, local_axis.x);
    theta_ref[pitchIndex(joint)] = pitch;
    theta_ref[yawIndex(joint)] = yaw;
    frames[joint + 1] = orthonormalize(
      multiply(multiply(frames[joint], rotationZ(-yaw)), rotationY(-pitch)));

    const Vec3 realized_axis = column(frames[joint + 1], 0);
    max_error = std::max(max_error, norm(realized_axis - desired_unit));
  }

  if (realized_frames) {
    *realized_frames = frames;
  }
  if (max_axis_alignment_error) {
    *max_axis_alignment_error = max_error;
  }
  if (min_local_horizontal_norm) {
    *min_local_horizontal_norm = min_horizontal;
  }
  return true;
}

}  // namespace

class OnlinePathFollowingController3D::RollingPathBuffer3D
{
public:
  enum class Provenance : uint8_t
  {
    Actual,
    Degraded,
    BodySeedSynthetic,
    TailExtensionSynthetic
  };

  struct Sample
  {
    double s{0.0};
    double time{0.0};
    Vec3 p{0.0, 0.0, 0.0};
    Vec3 tangent{1.0, 0.0, 0.0};
    double raw_curvature{0.0};
    double filtered_curvature{0.0};
    Mat3 bishop_frame{};
    Provenance provenance{Provenance::BodySeedSynthetic};
    uint64_t geometry_epoch{0};
  };

  explicit RollingPathBuffer3D(const OnlinePathFollowing3DParameters & params)
  : sample_ds_(params.path_sample_ds),
    min_append_distance_(params.min_append_distance),
    max_history_length_(params.history_length),
    curvature_filter_alpha_(params.curvature_filter_alpha)
  {
    const double worst_case_ds = std::max(std::min(sample_ds_, min_append_distance_), 1.0e-3);
    const size_t estimated = static_cast<size_t>(
      std::ceil(max_history_length_ / worst_case_ds)) + 64;
    capacity_ = std::max<size_t>(estimated, 256);
    buffer_.resize(capacity_);
    initSegTree();
  }

  void resetWithSeedSamples(
    const std::vector<Sample> & seed_samples, const Mat3 & frame0, double time0)
  {
    head_ = 0;
    size_ = 0;
    recenter_offset_ = {0.0, 0.0, 0.0};
    std::fill(seg_tree_.begin(), seg_tree_.end(), 0.0);
    double previous_s = -std::numeric_limits<double>::infinity();
    for (const Sample & sample : seed_samples) {
      if (sample.s < -1.0e-12 && sample.s > previous_s + 1.0e-12) {
        ensureAppendCapacity();
        buffer_[ridx(size_)] = sample;
        ++size_;
        updateSegTree(ridx(size_ - 1), sample.filtered_curvature);
        previous_s = sample.s;
      }
    }
    appendAnchor(frame0, time0, Provenance::BodySeedSynthetic, 0);
    trimHistory();
  }

  void startMotionEpoch(
    const Vec3 & tangent, const Mat3 & reference_frame, double time,
    Provenance provenance, uint64_t geometry_epoch)
  {
    if (size_ == 0) {
      appendAnchor(reference_frame, time, provenance, geometry_epoch);
      return;
    }
    Sample & newest = buffer_[ridx(size_ - 1)];
    newest.time = time;
    newest.tangent = norm(tangent) > 1.0e-9 ?
      normalize(tangent) : normalize(column(reference_frame, 0));
    newest.bishop_frame = makeFrameFromAxisAndReference(newest.tangent, reference_frame);
    newest.provenance = provenance;
    newest.geometry_epoch = geometry_epoch;
  }

  bool appendDisplacement(
    const Vec3 & displacement, const Mat3 & frame_hint, double time,
    Provenance provenance, uint64_t geometry_epoch)
  {
    if (size_ == 0) {
      appendAnchor(frame_hint, time, provenance, geometry_epoch);
      return false;
    }
    const double distance = norm(displacement);
    if (!std::isfinite(distance) || !isFinite(displacement) ||
      distance < min_append_distance_)
    {
      return false;
    }

    const Vec3 start = buffer_[ridx(size_ - 1)].p;
    const double start_time = buffer_[ridx(size_ - 1)].time;
    const size_t segment_count = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(distance / sample_ds_)));
    for (size_t i = 1; i <= segment_count; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(segment_count);
      appendSample(
        start + alpha * displacement, frame_hint,
        start_time + alpha * (time - start_time),
        distance / static_cast<double>(segment_count), provenance, geometry_epoch);
    }
    recenter(buffer_[ridx(size_ - 1)].p);
    trimHistory();
    return true;
  }

  bool sample(
    double s, Vec3 & p, Vec3 & tangent, double & curvature,
    Mat3 & bishop_frame) const
  {
    if (size_ == 0 || s < oldestS() - 1.0e-12 || s > newestS() + 1.0e-12) {
      return false;
    }
    if (size_ == 1) {
      const Sample & only = buffer_[head_];
      p = only.p - recenter_offset_;
      tangent = only.tangent;
      curvature = only.filtered_curvature;
      bishop_frame = only.bishop_frame;
      return isFinite(p) && isFinite(tangent) && std::isfinite(curvature);
    }

    const size_t upper = upperBound(s);
    const size_t hi = std::min<size_t>(upper, size_ - 1);
    const size_t lo = hi > 0 ? hi - 1 : 0;
    const Sample & lower = buffer_[ridx(lo)];
    const Sample & upper_sample = buffer_[ridx(hi)];
    const double span = upper_sample.s - lower.s;
    const double alpha = span > 1.0e-12 ? clamp01((s - lower.s) / span) : 0.0;
    p = lerp(lower.p, upper_sample.p, alpha) - recenter_offset_;
    tangent = normalize(lerp(lower.tangent, upper_sample.tangent, alpha));
    curvature = lower.filtered_curvature * (1.0 - alpha) +
      upper_sample.filtered_curvature * alpha;
    bishop_frame = slerpFrame(lower.bishop_frame, upper_sample.bishop_frame, alpha);
    return isFinite(p) && isFinite(tangent) && std::isfinite(curvature);
  }

  bool empty() const {return size_ == 0;}
  double newestS() const {return size_ == 0 ? 0.0 : buffer_[ridx(size_ - 1)].s;}
  double oldestS() const {return size_ == 0 ? 0.0 : buffer_[head_].s;}
  double geometryCoverage() const {return newestS() - oldestS();}

  double coverage(Provenance provenance) const
  {
    if (size_ == 0 || buffer_[ridx(size_ - 1)].provenance != provenance) {
      return 0.0;
    }
    const uint64_t epoch = buffer_[ridx(size_ - 1)].geometry_epoch;
    size_t index = size_ - 1;
    while (index > 0 && buffer_[ridx(index - 1)].provenance == provenance &&
      buffer_[ridx(index - 1)].geometry_epoch == epoch)
    {
      --index;
    }
    return newestS() - buffer_[ridx(index)].s;
  }

  double actualOldestS() const
  {
    return newestS() - coverage(Provenance::Actual);
  }

  double actualCoverage() const {return coverage(Provenance::Actual);}
  double degradedCoverage() const {return coverage(Provenance::Degraded);}
  Vec3 newestTangent() const
  {
    return size_ == 0 ? Vec3{1.0, 0.0, 0.0} : buffer_[ridx(size_ - 1)].tangent;
  }
  double newestRawCurvature() const
  {
    return size_ == 0 ? 0.0 : buffer_[ridx(size_ - 1)].raw_curvature;
  }
  double newestCurvature() const
  {
    return size_ == 0 ? 0.0 : buffer_[ridx(size_ - 1)].filtered_curvature;
  }

  double maxCurvature(double s_begin, double s_end) const
  {
    if (size_ == 0) {
      return 0.0;
    }
    const double lo = std::max(oldestS(), std::min(s_begin, s_end));
    const double hi = std::min(newestS(), std::max(s_begin, s_end));
    if (lo > hi) {
      return 0.0;
    }
    double result = 0.0;
    Vec3 p{}, tangent{};
    Mat3 frame{};
    double curvature = 0.0;
    if (sample(lo, p, tangent, curvature, frame)) {
      result = std::max(result, curvature);
    }
    if (sample(hi, p, tangent, curvature, frame)) {
      result = std::max(result, curvature);
    }

    size_t lo_index = 0;
    while (lo_index < size_ && buffer_[ridx(lo_index)].s < lo - 1.0e-12) {
      ++lo_index;
    }
    size_t hi_index = lo_index;
    while (hi_index < size_ && buffer_[ridx(hi_index)].s <= hi + 1.0e-12) {
      ++hi_index;
    }
    if (hi_index > lo_index && lo_index < size_) {
      --hi_index;
      const size_t lo_physical = ridx(lo_index);
      const size_t hi_physical = ridx(hi_index);
      if (lo_physical <= hi_physical) {
        result = std::max(result, querySegTree(lo_physical, hi_physical));
      } else {
        result = std::max(result, querySegTree(lo_physical, capacity_ - 1));
        result = std::max(result, querySegTree(0, hi_physical));
      }
    }
    return result;
  }

private:
  size_t ridx(size_t index) const {return (head_ + index) % capacity_;}

  size_t upperBound(double value) const
  {
    size_t lo = 0;
    size_t hi = size_;
    while (lo < hi) {
      const size_t mid = lo + (hi - lo) / 2;
      if (buffer_[ridx(mid)].s > value) {
        hi = mid;
      } else {
        lo = mid + 1;
      }
    }
    return lo;
  }

  void initSegTree()
  {
    seg_tree_base_ = 1;
    while (seg_tree_base_ < capacity_) {
      seg_tree_base_ *= 2;
    }
    seg_tree_.assign(2 * seg_tree_base_, 0.0);
  }

  void updateSegTree(size_t physical_index, double value)
  {
    size_t node = seg_tree_base_ + physical_index;
    seg_tree_[node] = value;
    node /= 2;
    while (node > 0) {
      const double new_max = std::max(seg_tree_[2 * node], seg_tree_[2 * node + 1]);
      if (seg_tree_[node] == new_max) {
        break;
      }
      seg_tree_[node] = new_max;
      node /= 2;
    }
  }

  double querySegTree(size_t lo, size_t hi) const
  {
    if (lo > hi || lo >= capacity_ || hi >= capacity_) {
      return 0.0;
    }
    lo += seg_tree_base_;
    hi += seg_tree_base_;
    double result = 0.0;
    while (lo <= hi) {
      if (lo % 2 == 1) {
        result = std::max(result, seg_tree_[lo++]);
      }
      if (hi % 2 == 0) {
        result = std::max(result, seg_tree_[hi--]);
      }
      lo /= 2;
      hi /= 2;
    }
    return result;
  }

  void appendAnchor(
    const Mat3 & frame, double time, Provenance provenance, uint64_t geometry_epoch)
  {
    ensureAppendCapacity();
    Sample sample;
    sample.s = 0.0;
    sample.time = time;
    sample.p = {0.0, 0.0, 0.0};
    sample.tangent = normalize(column(frame, 0));
    sample.bishop_frame = makeFrameFromAxisAndReference(sample.tangent, frame);
    sample.provenance = provenance;
    sample.geometry_epoch = geometry_epoch;
    buffer_[ridx(size_)] = sample;
    ++size_;
    updateSegTree(ridx(size_ - 1), sample.filtered_curvature);
  }

  void appendSample(
    const Vec3 & p, const Mat3 & frame_hint, double time, double ds_from_previous,
    Provenance provenance, uint64_t geometry_epoch)
  {
    if (ds_from_previous <= 1.0e-12 || size_ == 0) {
      return;
    }
    const Sample previous = buffer_[ridx(size_ - 1)];
    Vec3 tangent = p - previous.p;
    if (norm(tangent) < 1.0e-12) {
      tangent = column(frame_hint, 0);
    }
    tangent = normalize(tangent);
    Sample sample;
    sample.s = previous.s + ds_from_previous;
    sample.time = time;
    sample.p = p;
    sample.tangent = tangent;
    sample.bishop_frame = advanceBishopFrame(previous.bishop_frame, previous.tangent, tangent);
    const double cosine = std::clamp(dot(tangent, previous.tangent), -1.0, 1.0);
    sample.raw_curvature = std::acos(cosine) / std::max(ds_from_previous, 1.0e-12);
    sample.filtered_curvature = (1.0 - curvature_filter_alpha_) *
      previous.filtered_curvature + curvature_filter_alpha_ * sample.raw_curvature;
    sample.provenance = provenance;
    sample.geometry_epoch = geometry_epoch;
    ensureAppendCapacity();
    buffer_[ridx(size_)] = sample;
    ++size_;
    updateSegTree(ridx(size_ - 1), sample.filtered_curvature);
  }

  void recenter(const Vec3 & newest_point)
  {
    recenter_offset_ = newest_point;
  }

  void trimHistory()
  {
    const double min_s = newestS() - max_history_length_;
    while (size_ > 2 && buffer_[ridx(1)].s < min_s) {
      updateSegTree(head_, 0.0);
      head_ = (head_ + 1) % capacity_;
      --size_;
    }
  }

  void ensureAppendCapacity()
  {
    if (size_ < capacity_) {
      return;
    }
    updateSegTree(head_, 0.0);
    head_ = (head_ + 1) % capacity_;
    --size_;
  }

  double sample_ds_{0.01};
  double min_append_distance_{0.002};
  double max_history_length_{1.30};
  double curvature_filter_alpha_{0.3};
  Vec3 recenter_offset_{0.0, 0.0, 0.0};
  std::vector<double> seg_tree_;
  size_t seg_tree_base_{0};
  std::vector<Sample> buffer_;
  size_t head_{0};
  size_t size_{0};
  size_t capacity_{0};
};

struct OnlinePathFollowingController3D::PredictedPathSegment3D
{
  struct Sample
  {
    double delta_s{0.0};
    Vec3 p{0.0, 0.0, 0.0};
    Vec3 tangent{1.0, 0.0, 0.0};
    double raw_curvature{0.0};
    double filtered_curvature{0.0};
    Mat3 bishop_frame{};
    Mat3 head_frame{};
  };

  std::vector<Sample> samples;
  double max_filtered_curvature{0.0};
  double requested_distance{0.0};
  double achieved_distance{0.0};
  bool valid{false};
};

OnlinePathFollowingController3D::~OnlinePathFollowingController3D() = default;

OnlinePathFollowingController3D::OnlinePathFollowingController3D(
  const OnlinePathFollowing3DParameters & params)
: params_(params),
  path_buffer_(nullptr),
  predicted_path_segment_(std::make_unique<PredictedPathSegment3D>()),
  local_controller_(makeFrontUnitParameters()),
  plant_(makeFrontUnitParameters())
{
  validateParameters();
  path_buffer_ = std::make_unique<RollingPathBuffer3D>(params_);
}

SimulationState3D OnlinePathFollowingController3D::makeInitialState() const
{
  return plant_.makeInitialState();
}

MeasuredRobotState3D OnlinePathFollowingController3D::makeMeasuredState(
  const SimulationState3D & state) const
{
  return plant_.measuredState(state);
}

FrontUnitController3DParameters OnlinePathFollowingController3D::frontUnitParameters() const
{
  return makeFrontUnitParameters();
}

HeadCommand3D OnlinePathFollowingController3D::limitCommand(const HeadCommand3D & cmd) const
{
  HeadCommand3D forward_only = cmd;
  forward_only.linear_velocity = std::max(0.0, forward_only.linear_velocity);
  return local_controller_.limitCommand(forward_only);
}

bool OnlinePathFollowingController3D::ingestRealizedFrontTwist(
  const RealizedFrontTwist3D & sample)
{
  if (!sample.twist_valid || !sample.attitude_valid || !std::isfinite(sample.stamp_sec) ||
    !isFinite(sample.linear_velocity_head) || !isFinite(sample.angular_velocity_head))
  {
    return false;
  }
  const auto rotation = internal::validateRotation(
    sample.tracking_from_head, params_.max_attitude_orthogonality_error,
    params_.max_attitude_determinant_error);
  if (!rotation.valid) {
    return false;
  }

  std::lock_guard<std::mutex> lock(motion_sample_mutex_);
  if (!pending_motion_samples_.empty() &&
    sample.stamp_sec <= pending_motion_samples_.back().stamp_sec)
  {
    if (std::abs(sample.stamp_sec - pending_motion_samples_.back().stamp_sec) <= 1.0e-12) {
      dropped_duplicate_samples_.fetch_add(1, std::memory_order_relaxed);
    } else {
      dropped_out_of_order_samples_.fetch_add(1, std::memory_order_relaxed);
    }
    return false;
  }
  if (pending_motion_samples_.size() >= params_.max_pending_motion_samples) {
    dropped_queue_overflow_samples_.fetch_add(1, std::memory_order_relaxed);
    pending_motion_samples_.clear();
    queue_overflow_requires_reset_ = true;
  }
  pending_motion_samples_.push_back(sample);
  return true;
}

void OnlinePathFollowingController3D::notifyEstimatorReset(uint64_t estimator_epoch)
{
  std::lock_guard<std::mutex> lock(motion_sample_mutex_);
  estimator_epoch_ = estimator_epoch;
  pending_motion_samples_.clear();
  estimator_reset_requires_reset_ = true;
}

OnlineFollowerOutput3D OnlinePathFollowingController3D::step(
  const HeadCommand3D & cmd, double control_time_sec, double dt,
  const MeasuredRobotState3D & measured_state)
{
  OnlineFollowerOutput3D result;
  diag_ = OnlinePathFollowing3DDiagnostics{};
  diag_.chord_search_failure_count = chord_search_failure_count_.load(std::memory_order_relaxed);
  diag_.reset_count = reset_count_;
  diag_.dropped_duplicate_samples = dropped_duplicate_samples_.load(std::memory_order_relaxed);
  diag_.dropped_out_of_order_samples =
    dropped_out_of_order_samples_.load(std::memory_order_relaxed);
  diag_.dropped_queue_overflow_samples =
    dropped_queue_overflow_samples_.load(std::memory_order_relaxed);
  diag_.commanded_speed = cmd.linear_velocity;
  diag_.reverse_command_rejected = cmd.linear_velocity < 0.0;
  diag_.required_history = requiredHistory();
  diag_.geometry_epoch = geometry_epoch_;
  diag_.estimator_epoch = estimator_epoch_;
  diag_.tracking_frame_epoch = tracking_frame_epoch_;

  auto safe_return = [&]() {
      applied_cmd_ = {0.0, 0.0, 0.0};
      resetFeedforwardFilter();
      result.applied_front_command = applied_cmd_;
      result.diagnostics = diag_;
      return result;
    };

  if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(control_time_sec) ||
    !measured_state.attitude_valid || !measured_state.joints_valid ||
    !std::isfinite(measured_state.stamp_sec) || measured_state.stamp_sec < 0.0)
  {
    return safe_return();
  }
  for (double theta : measured_state.joints.theta) {
    if (!std::isfinite(theta)) {
      return safe_return();
    }
  }

  diag_.measured_state_age_sec = control_time_sec - measured_state.stamp_sec;
  if (diag_.measured_state_age_sec > params_.measured_state_timeout_sec) {
    diag_.measured_state_stale = true;
    return safe_return();
  }
  if (diag_.measured_state_age_sec < -params_.future_state_tolerance_sec) {
    diag_.measured_state_from_future = true;
    return safe_return();
  }

  SimulationState3D state;
  if (!geometryState(measured_state, state)) {
    return safe_return();
  }

  if (has_control_time_ && control_time_sec + 1.0e-12 < last_control_time_sec_) {
    diag_.control_time_jump_detected = true;
    beginNewGeometryEpoch(
      GeometryResetReason::ControlTimeJump, state, applied_cmd_, true);
  }
  // tracking_frame_epoch 由实测样本 (processRealizedSample) 中的
  // sample.tracking_frame_epoch 负责更新和触发 epoch 切换。
  // 此处仅用于诊断上报。
  diag_.tracking_frame_epoch = tracking_frame_epoch_;

  HeadCommand3D nonnegative_cmd = cmd;
  nonnegative_cmd.linear_velocity = std::max(0.0, nonnegative_cmd.linear_velocity);
  applied_cmd_ = limitCommand(filterCommand(nonnegative_cmd, dt));
  result.applied_front_command = applied_cmd_;
  diag_.limited_speed = applied_cmd_.linear_velocity;
  diag_.applied_pitch_rate = applied_cmd_.pitch_rate;
  diag_.applied_yaw_rate = applied_cmd_.yaw_rate;

  resetBufferIfNeeded(state, applied_cmd_);
  processRealizedMotion(control_time_sec, state, applied_cmd_);

  diag_.geometry_coverage = path_buffer_->geometryCoverage();
  diag_.formal_actual_coverage = path_buffer_->actualCoverage();
  diag_.history_coverage = diag_.formal_actual_coverage;
  diag_.degraded_coverage = path_buffer_->degradedCoverage();
  diag_.actual_buffered_coverage = diag_.formal_actual_coverage;
  diag_.pending_actual_distance = previous_motion_was_formal_actual_ ?
    norm(integrated_joint1_position_ - last_buffered_joint1_position_) : 0.0;
  diag_.small_displacement_pending = diag_.pending_actual_distance > 1.0e-9;
  diag_.effective_actual_coverage =
    diag_.actual_buffered_coverage + diag_.pending_actual_distance;
  diag_.synthetic_coverage = std::max(
    0.0, diag_.geometry_coverage - diag_.formal_actual_coverage - diag_.degraded_coverage);
  diag_.history_deficit = std::max(
    0.0, diag_.required_history - diag_.formal_actual_coverage);
  diag_.buffer_ready = diag_.sensor_fresh &&
    diag_.history_deficit <= 1.0e-9 &&
    params_.front_motion_mode == FrontMotionMode::MeasuredTwistRequired;
  diag_.s_joint1 = path_buffer_->newestS();
  diag_.latest_raw_curvature = path_buffer_->newestRawCurvature();
  diag_.latest_filtered_curvature = path_buffer_->newestCurvature();
  diag_.joint1_path_curvature = diag_.latest_filtered_curvature;
  diag_.active_reference_available = has_last_theta_ref_;

  const double path_speed = diag_.joint1_path_speed;
  const bool moving_for_shape = path_speed >= params_.min_shape_speed;
  if ((!moving_for_shape || reverse_motion_active_) && has_last_theta_ref_ &&
    params_.freeze_reference_when_stopped && diag_.sensor_valid)
  {
    diag_.using_shape_memory = true;
    diag_.reference_frozen = true;
    diag_.reference_valid = true;
    diag_.active_reference_available = true;
    result.joint_velocity_command = applyFrozenReference(dt, measured_state);
  } else {
    ShapeReferenceCandidate3D candidate;
    const bool reference_valid = diag_.buffer_ready && moving_for_shape &&
      !diag_.reverse_motion_detected &&
      computeShapeReference(state, applied_cmd_, path_buffer_->newestS(), candidate);

    if (reference_valid) {
      diag_.using_shape_memory = true;
      diag_.new_reference_valid = true;
      diag_.active_reference_available = true;
      diag_.reference_valid = true;
      diag_.feedforward_valid = candidate.feedforward_valid;
      diag_.joint1_path_curvature = candidate.curvature;
      diag_.max_axis_alignment_error = candidate.max_axis_alignment_error;
      diag_.reference_span = candidate.reference_span;
      result.joint_velocity_command = applyShapeReference(
        candidate.theta_ref, candidate.dtheta_ref_ds, path_speed, dt, measured_state);
      last_theta_ref_ = candidate.theta_ref;
      has_last_theta_ref_ = true;
    } else if (has_last_theta_ref_ && diag_.sensor_valid) {
      resetFeedforwardFilter();
      diag_.using_shape_memory = true;
      diag_.reference_frozen = true;
      diag_.reference_valid = true;
      diag_.active_reference_available = true;
      result.joint_velocity_command = applyFrozenReference(dt, measured_state);
    } else {
      resetFeedforwardFilter();
      diag_.using_local_fallback = params_.use_local_fallback;
      if (params_.use_local_fallback) {
        result.joint_velocity_command =
          local_controller_.computeJointVelocity(applied_cmd_, measured_state);
      }
    }
  }

  last_control_time_sec_ = control_time_sec;
  has_control_time_ = true;
  diag_.chord_search_failure_count = chord_search_failure_count_.load(std::memory_order_relaxed);
  diag_.reset_count = reset_count_;
  diag_.dropped_duplicate_samples = dropped_duplicate_samples_.load(std::memory_order_relaxed);
  diag_.dropped_out_of_order_samples =
    dropped_out_of_order_samples_.load(std::memory_order_relaxed);
  diag_.dropped_queue_overflow_samples =
    dropped_queue_overflow_samples_.load(std::memory_order_relaxed);
  diag_.geometry_epoch = geometry_epoch_;
  diag_.estimator_epoch = estimator_epoch_;
  diag_.tracking_frame_epoch = tracking_frame_epoch_;
  result.diagnostics = diag_;
  return result;
}

JointVelocity3D OnlinePathFollowingController3D::step(
  const HeadCommand3D & cmd, double dt, SimulationState3D & state)
{
  return step(cmd, state.time + std::max(0.0, dt), dt, state);
}

JointVelocity3D OnlinePathFollowingController3D::step(
  const HeadCommand3D & cmd, double control_time_sec, double dt, SimulationState3D & state)
{
  const OnlineFollowerOutput3D result = step(cmd, control_time_sec, dt, makeMeasuredState(state));
  plant_.apply(result.applied_front_command, result.joint_velocity_command, dt, state);
  return result.joint_velocity_command;
}

OnlinePathFollowing3DDiagnostics OnlinePathFollowingController3D::diagnostics() const
{
  return diag_;
}

HeadCommand3D OnlinePathFollowingController3D::appliedCommand() const
{
  return applied_cmd_;
}

FrontUnitController3DParameters OnlinePathFollowingController3D::makeFrontUnitParameters() const
{
  FrontUnitController3DParameters params;
  params.link_length = params_.link_length;
  params.joint_rate_limit = params_.joint_rate_limit;
  params.joint_limit = params_.joint_limit;
  params.max_curvature = params_.max_curvature;
  params.curvature_velocity_epsilon = params_.curvature_velocity_epsilon;
  params.damping = params_.damping;
  return params;
}

void OnlinePathFollowingController3D::validateParameters() const
{
  const auto require_finite = [](double value, const char * name) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be finite");
      }
    };
  const std::array<std::pair<double, const char *>, 40> finite_parameters{{
    {params_.link_length, "link_length"},
    {params_.joint_rate_limit, "joint_rate_limit"},
    {params_.joint_limit, "joint_limit"},
    {params_.joint_limit_margin, "joint_limit_margin"},
    {params_.max_curvature, "max_curvature"},
    {params_.curvature_tolerance, "curvature_tolerance"},
    {params_.curvature_velocity_epsilon, "curvature_velocity_epsilon"},
    {params_.curvature_filter_alpha, "curvature_filter_alpha"},
    {params_.damping, "damping"},
    {params_.path_sample_ds, "path_sample_ds"},
    {params_.min_append_distance, "min_append_distance"},
    {params_.history_length, "history_length"},
    {params_.min_buffer_update_speed, "min_buffer_update_speed"},
    {params_.min_shape_speed, "min_shape_speed"},
    {params_.min_history_coverage, "min_history_coverage"},
    {params_.joint_tracking_gain, "joint_tracking_gain"},
    {params_.feedforward_ds, "feedforward_ds"},
    {params_.feedforward_gain, "feedforward_gain"},
    {params_.feedforward_slope_filter_length_constant,
      "feedforward_slope_filter_length_constant"},
    {params_.feedforward_slope_limit, "feedforward_slope_limit"},
    {params_.max_predictor_velocity_mismatch, "max_predictor_velocity_mismatch"},
    {params_.max_predictor_direction_error_rad, "max_predictor_direction_error_rad"},
    {params_.max_predictor_angular_velocity_error,
      "max_predictor_angular_velocity_error"},
    {params_.predictor_min_speed, "predictor_min_speed"},
    {params_.predictor_max_arc_step, "predictor_max_arc_step"},
    {params_.predictor_arc_length_tolerance, "predictor_arc_length_tolerance"},
    {params_.command_filter_time_constant, "command_filter_time_constant"},
    {params_.angular_rate_slew_limit, "angular_rate_slew_limit"},
    {params_.linear_accel_limit, "linear_accel_limit"},
    {params_.sensor_timeout_sec, "sensor_timeout_sec"},
    {params_.sensor_reset_timeout_sec, "sensor_reset_timeout_sec"},
    {params_.measured_state_timeout_sec, "measured_state_timeout_sec"},
    {params_.future_state_tolerance_sec, "future_state_tolerance_sec"},
    {params_.max_sensor_sample_dt, "max_sensor_sample_dt"},
    {params_.reverse_speed_tolerance, "reverse_speed_tolerance"},
    {params_.chord_residual_tolerance, "chord_residual_tolerance"},
    {params_.chord_distance_monotonicity_tolerance,
      "chord_distance_monotonicity_tolerance"},
    {params_.max_chord_progress_per_step, "max_chord_progress_per_step"},
    {params_.min_axis_continuity_cosine, "min_axis_continuity_cosine"},
    {params_.axis_singularity_epsilon, "axis_singularity_epsilon"}}};
  for (const auto & entry : finite_parameters) {
    require_finite(entry.first, entry.second);
  }
  const std::array<std::pair<double, const char *>, 8> remaining_parameters{{
    {params_.synthetic_tail_margin, "synthetic_tail_margin"},
    {params_.max_attitude_step_rad, "max_attitude_step_rad"},
    {params_.max_attitude_orthogonality_error, "max_attitude_orthogonality_error"},
    {params_.max_attitude_determinant_error, "max_attitude_determinant_error"},
    {params_.joint_acceleration_limit, "joint_acceleration_limit"},
    {params_.max_axis_alignment_tolerance, "max_axis_alignment_tolerance"},
    {params_.curvature_tolerance, "curvature_tolerance"},
    {params_.damping, "damping"}}};
  for (const auto & entry : remaining_parameters) {
    require_finite(entry.first, entry.second);
  }

  const bool invalid =
    !(params_.link_length > 0.0) || !(params_.joint_rate_limit > 0.0) ||
    !(params_.joint_limit > 0.0) || params_.joint_limit_margin < 0.0 ||
    params_.joint_limit_margin >= params_.joint_limit || params_.max_curvature < 0.0 ||
    params_.curvature_tolerance < 0.0 || params_.curvature_velocity_epsilon < 0.0 ||
    params_.curvature_filter_alpha<0.0 || params_.curvature_filter_alpha>1.0 ||
    params_.damping<0.0 || !(params_.path_sample_ds > 0.0) ||
      !(params_.min_append_distance > 0.0) ||
      params_.min_append_distance> params_.path_sample_ds ||
    !(params_.history_length > 0.0) || params_.min_buffer_update_speed < 0.0 ||
    params_.min_shape_speed < 0.0 || params_.min_history_coverage < 0.0 ||
    params_.joint_tracking_gain < 0.0 || !(params_.feedforward_ds > 0.0) ||
    params_.feedforward_gain < 0.0 ||
    params_.feedforward_slope_filter_length_constant < 0.0 ||
    !(params_.feedforward_slope_limit > 0.0) ||
    params_.max_predictor_velocity_mismatch < 0.0 ||
    params_.max_predictor_direction_error_rad<0.0 ||
      params_.max_predictor_direction_error_rad>3.14159265358979323846 ||
    params_.max_predictor_angular_velocity_error < 0.0 ||
    !(params_.predictor_min_speed > 0.0) || !(params_.predictor_max_arc_step > 0.0) ||
    params_.predictor_arc_length_tolerance < 0.0 || params_.predictor_max_iterations == 0 ||
    params_.command_filter_time_constant < 0.0 || params_.angular_rate_slew_limit < 0.0 ||
    params_.linear_accel_limit < 0.0 || params_.sensor_timeout_sec < 0.0 ||
    params_.sensor_reset_timeout_sec < params_.sensor_timeout_sec ||
    params_.measured_state_timeout_sec < 0.0 || params_.future_state_tolerance_sec < 0.0 ||
    !(params_.max_sensor_sample_dt > 0.0) || params_.reverse_speed_tolerance < 0.0 ||
    !(params_.chord_residual_tolerance > 0.0) ||
    params_.chord_distance_monotonicity_tolerance < 0.0 ||
    params_.max_chord_progress_per_step < 0.0 ||
    params_.min_axis_continuity_cosine<-1.0 ||
      params_.min_axis_continuity_cosine>1.0 ||
    !(params_.axis_singularity_epsilon > 0.0) || params_.synthetic_tail_margin < 0.0 ||
    !(params_.max_attitude_step_rad > 0.0) ||
    !(params_.max_attitude_orthogonality_error > 0.0) ||
    !(params_.max_attitude_determinant_error > 0.0) ||
    params_.max_pending_motion_samples == 0 ||
    params_.max_motion_samples_per_control_step == 0 ||
    !(params_.joint_acceleration_limit > 0.0) ||
    !(params_.max_axis_alignment_tolerance > 0.0);
  if (invalid) {
    throw std::invalid_argument("online path following parameters are outside the valid range");
  }
  if (params_.max_curvature > 1.0e-9 &&
    params_.max_curvature * params_.link_length >= 2.0)
  {
    throw std::invalid_argument("max_curvature * link_length must be < 2");
  }
  const double required = requiredHistory();
  if (params_.history_length < required) {
    throw std::invalid_argument(
            "history_length (" + std::to_string(params_.history_length) +
            ") must be >= requiredHistory() (" + std::to_string(required) + ")");
  }
}

bool OnlinePathFollowingController3D::geometryState(
  const MeasuredRobotState3D & measured_state, SimulationState3D & state) const
{
  const auto rotation = internal::validateRotation(
    measured_state.tracking_from_head, params_.max_attitude_orthogonality_error,
    params_.max_attitude_determinant_error);
  diag_.attitude_orthogonality_error = rotation.orthogonality_error;
  diag_.attitude_determinant = rotation.determinant;
  if (!rotation.valid) {
    diag_.invalid_rotation_matrix = true;
    return false;
  }
  state = {};
  state.time = measured_state.stamp_sec;
  state.head_frame = measured_state.tracking_from_head;
  state.joints = measured_state.joints;
  state.link_frames = linkFrames(state.head_frame, state.joints.theta);
  state.link_axes = linkAxes(state.link_frames);
  state.body_points = bodyPoints(state.head_position, state.link_axes, params_.link_length);
  return true;
}

Vec3 OnlinePathFollowingController3D::joint1Point(const SimulationState3D & state) const
{
  return state.head_position - params_.link_length * column(state.head_frame, 0);
}

Vec3 OnlinePathFollowingController3D::commandedJoint1Velocity(
  const SimulationState3D & state, const HeadCommand3D & cmd) const
{
  const Vec3 b0 = column(state.head_frame, 0);
  const Vec3 head_velocity = cmd.linear_velocity * b0;
  const Vec3 head_omega = cmd.yaw_rate * column(state.head_frame, 2) +
    cmd.pitch_rate * column(state.head_frame, 1);
  return head_velocity - params_.link_length * cross(head_omega, b0);
}

Vec3 OnlinePathFollowingController3D::realizedJoint1Velocity(
  const RealizedFrontTwist3D & sample) const
{
  const Vec3 b1 = column(sample.tracking_from_head, 0);
  const Vec3 head_velocity = multiply(
    sample.tracking_from_head, sample.linear_velocity_head);
  const Vec3 head_omega = multiply(
    sample.tracking_from_head, sample.angular_velocity_head);
  return head_velocity - params_.link_length * cross(head_omega, b1);
}

Mat3 OnlinePathFollowingController3D::joint1FrameHint(
  const SimulationState3D & state, const HeadCommand3D & cmd) const
{
  Vec3 tangent = commandedJoint1Velocity(state, cmd);
  if (norm(tangent) < 1.0e-9) {
    tangent = column(state.head_frame, 0);
  }
  return makeFrameFromAxisAndReference(normalize(tangent), state.head_frame);
}

double OnlinePathFollowingController3D::requiredHistory() const
{
  double segment_span = params_.link_length;
  if (params_.max_curvature > 1.0e-9) {
    segment_span = 2.0 * std::asin(
      0.5 * params_.max_curvature * params_.link_length) / params_.max_curvature;
  }
  return std::max(
    params_.min_history_coverage, 3.0 * segment_span + 2.0 * params_.path_sample_ds);
}

void OnlinePathFollowingController3D::resetBufferIfNeeded(
  const SimulationState3D & state, const HeadCommand3D & cmd)
{
  if (path_buffer_->empty()) {
    warmupBuffer(state, cmd);
  }
}

void OnlinePathFollowingController3D::beginNewGeometryEpoch(
  GeometryResetReason reason, const SimulationState3D & state,
  const HeadCommand3D & cmd, bool preserve_active_reference,
  bool clear_pending_samples)
{
  ++geometry_epoch_;
  warmupBuffer(state, cmd);
  has_previous_motion_sample_ = false;
  previous_motion_was_formal_actual_ = false;
  has_last_realized_head_frame_ = false;
  if (clear_pending_samples) {
    std::lock_guard<std::mutex> lock(motion_sample_mutex_);
    pending_motion_samples_.clear();
  }
  integrated_joint1_position_ = {0.0, 0.0, 0.0};
  last_buffered_joint1_position_ = {0.0, 0.0, 0.0};
  latest_realized_joint1_velocity_ = {0.0, 0.0, 0.0};
  previous_realized_joint1_velocity_ = {0.0, 0.0, 0.0};
  latest_signed_joint1_speed_ = 0.0;
  has_previous_chord_solution_ = false;
  previous_chord_s_.fill(0.0);
  previous_chord_axes_.fill(Vec3{0.0, 0.0, 0.0});
  held_reference_ = HeldReferenceRecord{};
  strict_only_continuity_reject_count_ = 0;
  latest_linear_velocity_from_command_ = false;
  buffer_motion_was_stopped_ = false;
  reverse_motion_active_ = reason == GeometryResetReason::ReverseMotion;
  predicted_path_segment_->samples.clear();
  predicted_path_segment_->valid = false;
  if (!preserve_active_reference) {
    has_last_theta_ref_ = false;
    last_theta_ref_.fill(0.0);
  }
  last_theta_dot_.fill(0.0);
  resetFeedforwardFilter();
  ++reset_count_;
}

void OnlinePathFollowingController3D::processRealizedMotion(
  double control_time_sec, const SimulationState3D & state, const HeadCommand3D & cmd)
{
  std::deque<RealizedFrontTwist3D> local;
  bool overflow_reset = false;
  bool estimator_reset = false;
  {
    std::lock_guard<std::mutex> lock(motion_sample_mutex_);
    overflow_reset = queue_overflow_requires_reset_;
    estimator_reset = estimator_reset_requires_reset_;
    queue_overflow_requires_reset_ = false;
    estimator_reset_requires_reset_ = false;
    while (!pending_motion_samples_.empty() &&
      pending_motion_samples_.front().stamp_sec <= control_time_sec + 1.0e-12 &&
      local.size() < params_.max_motion_samples_per_control_step)
    {
      local.push_back(pending_motion_samples_.front());
      pending_motion_samples_.pop_front();
    }
    diag_.motion_samples_backlogged = pending_motion_samples_.size();
  }
  if (estimator_reset) {
    beginNewGeometryEpoch(
      GeometryResetReason::EstimatorReset, state, cmd, true, false);
  }
  if (overflow_reset) {
    diag_.sensor_queue_overflow = true;
    diag_.overflow_reset = true;
    beginNewGeometryEpoch(
      GeometryResetReason::QueueOverflow, state, cmd, true, false);
  }

  size_t processed = 0;
  for (const RealizedFrontTwist3D & sample : local) {
    processRealizedSample(sample, state, cmd);
    ++processed;
  }
  diag_.motion_samples_processed = processed;

  diag_.sensor_valid = has_previous_motion_sample_;
  diag_.estimator_epoch = estimator_epoch_;
  diag_.tracking_frame_epoch = tracking_frame_epoch_;
  diag_.geometry_epoch = geometry_epoch_;
  diag_.joint1_path_speed = norm(latest_realized_joint1_velocity_);
  diag_.signed_joint1_path_speed = latest_signed_joint1_speed_;
  diag_.reverse_motion_detected = reverse_motion_active_;
  diag_.linear_velocity_from_command = latest_linear_velocity_from_command_;
  diag_.degraded_front_motion = latest_linear_velocity_from_command_;
  if (has_previous_motion_sample_) {
    diag_.sensor_age_sec = std::max(0.0, control_time_sec - last_sensor_stamp_sec_);
    diag_.sensor_fresh = diag_.sensor_age_sec <= params_.sensor_timeout_sec;
    if (diag_.sensor_age_sec > params_.sensor_reset_timeout_sec) {
      beginNewGeometryEpoch(
        GeometryResetReason::SensorTimeout, state, cmd, true);
      diag_.sensor_valid = false;
      diag_.sensor_fresh = false;
    }
  }
  if (params_.front_motion_mode == FrontMotionMode::LocalFallbackOnly) {
    diag_.sensor_valid = false;
    diag_.sensor_fresh = false;
  }
}

void OnlinePathFollowingController3D::anchorMotionSample(
  const RealizedFrontTwist3D & sample, const Vec3 & velocity,
  bool formal_actual, const Mat3 & measured_frame)
{
  const auto provenance = formal_actual ? RollingPathBuffer3D::Provenance::Actual :
    RollingPathBuffer3D::Provenance::Degraded;
  path_buffer_->startMotionEpoch(
    velocity, measured_frame, sample.stamp_sec, provenance, geometry_epoch_);
  previous_motion_sample_ = sample;
  previous_realized_joint1_velocity_ = velocity;
  latest_realized_joint1_velocity_ = velocity;
  last_sensor_stamp_sec_ = sample.stamp_sec;
  estimator_epoch_ = sample.estimator_epoch;
  tracking_frame_epoch_ = sample.tracking_frame_epoch;
  has_previous_motion_sample_ = true;
  previous_motion_was_formal_actual_ = formal_actual;
  last_realized_head_frame_ = measured_frame;
  has_last_realized_head_frame_ = true;
  integrated_joint1_position_ = {0.0, 0.0, 0.0};
  last_buffered_joint1_position_ = {0.0, 0.0, 0.0};
}

bool OnlinePathFollowingController3D::processRealizedSample(
  const RealizedFrontTwist3D & sample,
  const SimulationState3D & state, const HeadCommand3D & cmd)
{
  if (params_.front_motion_mode == FrontMotionMode::LocalFallbackOnly) {
    return false;
  }
  // ImuWithCommandedLinearSpeed 模式下，若调用者已将指令速度作为实测线速度填入
  // （即 linear_velocity_measured=true），则满足 §8.3 Mode-A 条件，作为
  // formal_actual 处理，允许积累权威历史并激活 shape memory。
  // MeasuredTwistRequired 模式保持原有严格实测要求不变。
  const bool formal_actual =
    (params_.front_motion_mode == FrontMotionMode::MeasuredTwistRequired ||
    params_.front_motion_mode == FrontMotionMode::ImuWithCommandedLinearSpeed) &&
    sample.linear_velocity_measured && sample.angular_velocity_measured;
  if (params_.front_motion_mode == FrontMotionMode::MeasuredTwistRequired && !formal_actual) {
    return false;
  }

  const auto rotation = internal::validateRotation(
    sample.tracking_from_head, params_.max_attitude_orthogonality_error,
    params_.max_attitude_determinant_error);
  if (!rotation.valid) {
    diag_.invalid_rotation_matrix = true;
    return false;
  }
  const Mat3 measured_frame = sample.tracking_from_head;
  const Vec3 velocity = realizedJoint1Velocity(sample);
  const Vec3 tangent = path_buffer_->empty() ?
    column(measured_frame, 0) : path_buffer_->newestTangent();
  const double forward_indicator = dot(velocity, tangent);
  const double speed = norm(velocity);
  latest_realized_joint1_velocity_ = velocity;
  latest_signed_joint1_speed_ = forward_indicator;
  latest_linear_velocity_from_command_ = !formal_actual;
  diag_.joint1_path_speed = speed;
  diag_.effective_path_speed = speed;
  diag_.signed_joint1_path_speed = forward_indicator;
  diag_.degraded_front_motion = !formal_actual;

  if (has_previous_motion_sample_ && sample.estimator_epoch != estimator_epoch_) {
    estimator_epoch_ = sample.estimator_epoch;
    beginNewGeometryEpoch(
      GeometryResetReason::EstimatorReset, state, cmd, true, false);
  }
  if (has_previous_motion_sample_ && sample.tracking_frame_epoch != tracking_frame_epoch_) {
    tracking_frame_epoch_ = sample.tracking_frame_epoch;
    diag_.tracking_frame_redefined = true;
    beginNewGeometryEpoch(
      GeometryResetReason::TrackingFrameReset, state, cmd, true, false);
  }

  bool attitude_jump = false;
  if (has_last_realized_head_frame_) {
    const Mat3 delta = multiply(transpose(last_realized_head_frame_), measured_frame);
    const double trace = delta.v[0][0] + delta.v[1][1] + delta.v[2][2];
    attitude_jump = std::acos(std::clamp(0.5 * (trace - 1.0), -1.0, 1.0)) >
      params_.max_attitude_step_rad;
  }
  last_realized_head_frame_ = measured_frame;
  has_last_realized_head_frame_ = true;

  if (forward_indicator < -params_.reverse_speed_tolerance) {
    diag_.reverse_motion_detected = true;
    if (!reverse_motion_active_) {
      beginNewGeometryEpoch(
        GeometryResetReason::ReverseMotion, state, cmd, true, false);
      reverse_motion_active_ = true;
    }
    previous_motion_sample_ = sample;
    previous_realized_joint1_velocity_ = velocity;
    last_sensor_stamp_sec_ = sample.stamp_sec;
    has_previous_motion_sample_ = true;
    previous_motion_was_formal_actual_ = false;
    return false;
  }

  if (reverse_motion_active_) {
    reverse_motion_active_ = false;
    anchorMotionSample(sample, velocity, formal_actual, measured_frame);
    return false;
  }
  if (!has_previous_motion_sample_) {
    anchorMotionSample(sample, velocity, formal_actual, measured_frame);
    return true;
  }
  if (formal_actual != previous_motion_was_formal_actual_) {
    anchorMotionSample(sample, velocity, formal_actual, measured_frame);
    return false;
  }

  const double sample_dt = sample.stamp_sec - previous_motion_sample_.stamp_sec;
  if (sample_dt <= 1.0e-12) {
    dropped_duplicate_samples_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (sample_dt > params_.max_sensor_sample_dt || attitude_jump) {
    beginNewGeometryEpoch(
      sample_dt > params_.max_sensor_sample_dt ? GeometryResetReason::SensorGap :
      GeometryResetReason::AttitudeJump, state, cmd, true, false);
    anchorMotionSample(sample, velocity, formal_actual, measured_frame);
    return false;
  }

  if (speed < params_.min_buffer_update_speed) {
    diag_.buffer_commit_suppressed_low_speed = true;
    buffer_motion_was_stopped_ = true;
    integrated_joint1_position_ = last_buffered_joint1_position_;
    previous_motion_sample_ = sample;
    previous_realized_joint1_velocity_ = velocity;
    last_sensor_stamp_sec_ = sample.stamp_sec;
    return true;
  }
  if (buffer_motion_was_stopped_) {
    buffer_motion_was_stopped_ = false;
    previous_motion_sample_ = sample;
    previous_realized_joint1_velocity_ = velocity;
    last_sensor_stamp_sec_ = sample.stamp_sec;
    return true;
  }

  const Vec3 displacement = 0.5 * sample_dt *
    (previous_realized_joint1_velocity_ + velocity);
  integrated_joint1_position_ = integrated_joint1_position_ + displacement;
  const Vec3 buffered_displacement =
    integrated_joint1_position_ - last_buffered_joint1_position_;
  const auto provenance = formal_actual ? RollingPathBuffer3D::Provenance::Actual :
    RollingPathBuffer3D::Provenance::Degraded;
  if (norm(buffered_displacement) >= params_.min_append_distance &&
    path_buffer_->appendDisplacement(
      buffered_displacement, makeFrameFromAxisAndReference(
        speed > 1.0e-9 ? normalize(velocity) : tangent, measured_frame),
      sample.stamp_sec, provenance, geometry_epoch_))
  {
    last_buffered_joint1_position_ = integrated_joint1_position_;
  }
  previous_motion_sample_ = sample;
  previous_realized_joint1_velocity_ = velocity;
  last_sensor_stamp_sec_ = sample.stamp_sec;
  return true;
}

bool OnlinePathFollowingController3D::computeShapeReference(
  const SimulationState3D & state, const HeadCommand3D & cmd, double s_joint1,
  ShapeReferenceCandidate3D & candidate)
{
  // 文档 §8.6：根据配置选择使用三层分支树或单分支顺序搜索
  if (params_.use_multi_branch_reconstruction) {
    return computeShapeReferenceMultiBranch(state, cmd, s_joint1, candidate);
  }

  // 回退到旧的单分支实现
  const Mat3 link0_frame = has_last_realized_head_frame_ ?
    last_realized_head_frame_ : state.head_frame;
  if (!computeThetaReferenceAt(
      state, cmd, s_joint1, link0_frame, candidate.theta_ref,
      candidate.max_axis_alignment_error, candidate.curvature,
      candidate.reference_span, &candidate.chord_s, &candidate.chord_residual,
      &candidate.chord_axes))
  {
    candidate.status = ReferenceStatus::NoReference;
    return false;
  }

  for (size_t i = 0; i < candidate.chord_s.size(); ++i) {
    const double parent_s = i == 0 ? s_joint1 : candidate.chord_s[i - 1];
    if (!(candidate.chord_s[i] < parent_s - 1.0e-12)) {
      diag_.root_order_invalid = true;
      candidate.status = ReferenceStatus::NoReference;
      return false;
    }
    if (has_previous_chord_solution_) {
      const double root_jump = std::abs(candidate.chord_s[i] - previous_chord_s_[i]);
      const double axis_dot = dot(candidate.chord_axes[i], previous_chord_axes_[i]);
      diag_.max_chord_root_jump = std::max(diag_.max_chord_root_jump, root_jump);
      diag_.min_chord_axis_dot = std::min(diag_.min_chord_axis_dot, axis_dot);
      if (root_jump > params_.max_chord_progress_per_step ||
        axis_dot < params_.min_axis_continuity_cosine)
      {
        diag_.chord_continuity_guard_triggered = true;
        candidate.status = ReferenceStatus::NoReference;
        return false;
      }
    }
  }

  candidate.current_geometry_valid = true;
  candidate.status = ReferenceStatus::ExactNew; // 单分支默认为 EXACT_NEW
  diag_.chord_s = candidate.chord_s;
  diag_.chord_residual = candidate.chord_residual;
  previous_chord_s_ = candidate.chord_s;
  previous_chord_axes_ = candidate.chord_axes;
  has_previous_chord_solution_ = true;
  candidate.dtheta_ref_ds.fill(0.0);

  auto disable_feedforward = [&]() {
      candidate.feedforward_valid = false;
      candidate.dtheta_ref_ds.fill(0.0);
      resetFeedforwardFilter();
      return true;
    };
  if (!params_.use_command_predictor_feedforward) {
    return disable_feedforward();
  }

  const Vec3 commanded_velocity = commandedJoint1Velocity(state, cmd);
  const double command_speed = norm(commanded_velocity);
  const double realized_speed = norm(latest_realized_joint1_velocity_);
  diag_.predictor_speed_mismatch = realized_speed > 1.0e-9 ?
    std::abs(command_speed - realized_speed) : 0.0;
  diag_.predictor_velocity_mismatch = diag_.predictor_speed_mismatch;

  double direction_error = 0.0;
  if (command_speed > 1.0e-9 && realized_speed > 1.0e-9) {
    direction_error = std::acos(std::clamp(
        dot(commanded_velocity, latest_realized_joint1_velocity_) /
          (command_speed * realized_speed), -1.0, 1.0));
  }
  diag_.predictor_direction_error_rad = direction_error;
  const double vector_error = norm(commanded_velocity - latest_realized_joint1_velocity_);
  diag_.predictor_velocity_vector_error = vector_error;
  const Vec3 commanded_omega = cmd.yaw_rate * column(state.head_frame, 2) +
    cmd.pitch_rate * column(state.head_frame, 1);
  const Vec3 realized_omega = has_last_realized_head_frame_ ?
    multiply(last_realized_head_frame_, previous_motion_sample_.angular_velocity_head) :
    commanded_omega;
  const double omega_error = norm(commanded_omega - realized_omega);
  diag_.predictor_angular_velocity_error = omega_error;
  if (vector_error > params_.max_predictor_velocity_mismatch ||
    direction_error > params_.max_predictor_direction_error_rad ||
    omega_error > params_.max_predictor_angular_velocity_error)
  {
    diag_.predictor_disabled_due_to_mismatch = true;
    return disable_feedforward();
  }

  const double ds = std::max(params_.feedforward_ds, params_.path_sample_ds);
  if (!buildPredictedPathSegment(state, cmd, ds)) {
    return disable_feedforward();
  }
  diag_.max_predicted_filtered_curvature =
    predicted_path_segment_->max_filtered_curvature;
  if (predicted_path_segment_->max_filtered_curvature >
    params_.max_curvature + params_.curvature_tolerance)
  {
    diag_.curvature_guard_triggered = true;
    return disable_feedforward();
  }

  Vec3 predicted_point{}, predicted_tangent{};
  double predicted_curvature = 0.0;
  Mat3 predicted_path_frame{}, predicted_head_frame{};
  if (!predictFutureJoint1(
      state, cmd, ds, predicted_point, predicted_tangent, predicted_curvature,
      predicted_path_frame, &predicted_head_frame))
  {
    return disable_feedforward();
  }

  std::array<double, kNum3dJointDofs> theta_next{};
  double next_axis_error = 0.0;
  double next_curvature = 0.0;
  double next_span = 0.0;
  if (!computeThetaReferenceAt(
      state, cmd, s_joint1 + ds, predicted_head_frame, theta_next,
      next_axis_error, next_curvature, next_span))
  {
    return disable_feedforward();
  }

  const double tau = params_.feedforward_slope_filter_length_constant;
  const double alpha = tau <= 1.0e-12 ? 1.0 :
    std::clamp(ds / (tau + ds), 0.0, 1.0);
  for (size_t i = 0; i < kNum3dJointDofs; ++i) {
    const double raw_slope = saturate(
      wrapAngle(theta_next[i] - candidate.theta_ref[i]) / ds,
      params_.feedforward_slope_limit);
    filtered_dtheta_ref_ds_[i] += alpha *
      (raw_slope - filtered_dtheta_ref_ds_[i]);
    candidate.dtheta_ref_ds[i] = filtered_dtheta_ref_ds_[i];
    diag_.max_raw_dtheta_ref_ds = std::max(
      diag_.max_raw_dtheta_ref_ds, std::abs(raw_slope));
    diag_.max_filtered_dtheta_ref_ds = std::max(
      diag_.max_filtered_dtheta_ref_ds,
      std::abs(candidate.dtheta_ref_ds[i]));
  }
  candidate.feedforward_valid = true;
  diag_.feedforward_enabled = true;
  diag_.future_prediction_distance = ds;
  candidate.max_axis_alignment_error = std::max(
    candidate.max_axis_alignment_error, next_axis_error);
  return true;
}

bool OnlinePathFollowingController3D::computeThetaReferenceAt(
  const SimulationState3D & state, const HeadCommand3D & cmd, double s_joint1,
  const Mat3 & link0_frame,
  std::array<double, kNum3dJointDofs> & theta_ref,
  double & max_axis_alignment_error, double & curvature, double & reference_span,
  std::array<double, 3> * chord_s,
  std::array<double, 3> * chord_residual,
  std::array<Vec3, 3> * chord_axes) const
{
  Vec3 front_point{}, tangent{};
  Mat3 path_frame{};
  if (!sampleJoint1Path(state, cmd, s_joint1, front_point, tangent, curvature, path_frame)) {
    return false;
  }

  std::array<Vec3, kNum3dLinks - 1> desired_axes{};
  double front_s = s_joint1;
  const double initial_s = s_joint1;
  for (size_t link = 0; link < desired_axes.size(); ++link) {
    double back_s = 0.0;
    Vec3 back_point{};
    if (!findPreviousChordPoint(state, cmd, front_s, front_point, back_s, back_point,
        has_previous_chord_solution_ ? previous_chord_s_[link] :
          std::numeric_limits<double>::quiet_NaN()))
    {
      ++chord_search_failure_count_;
      diag_.chord_search_failed_this_step = true;
      return false;
    }
    const double chord_residual_value = std::abs(
      norm(front_point - back_point) - params_.link_length);
    diag_.max_chord_residual = std::max(diag_.max_chord_residual, chord_residual_value);
    const Vec3 axis = front_point - back_point;
    if (norm(axis) < 1.0e-9) {
      return false;
    }
    const Vec3 unit_axis = normalize(axis);
    // 奇异性检查已移至 computeThetaFromLinkAxes() 中，使用父连杆局部坐标系
    desired_axes[link] = unit_axis;
    if (chord_s) {
      (*chord_s)[link] = back_s;
    }
    if (chord_residual) {
      (*chord_residual)[link] = chord_residual_value;
    }
    if (chord_axes) {
      (*chord_axes)[link] = unit_axis;
    }
    front_s = back_s;
    front_point = back_point;
  }
  reference_span = initial_s - front_s;
  double min_local_horizontal = 1.0;
  const bool ik_ok = computeThetaFromLinkAxes(
    link0_frame, desired_axes, params_.axis_singularity_epsilon,
    theta_ref, nullptr, &max_axis_alignment_error, &min_local_horizontal);
  diag_.min_axis_horizontal_norm = std::min(
    diag_.min_axis_horizontal_norm, min_local_horizontal);
  if (!ik_ok) {
    diag_.axis_singularity_guard_triggered = true;
    return false;
  }
  if (max_axis_alignment_error > params_.max_axis_alignment_tolerance) {
    diag_.axis_singularity_guard_triggered = true;
    return false;
  }
  // 使用整个参考跨度上的最大曲率进行曲率守卫检查
  // 弦搜索范围 = [front_s, initial_s]，此区间内的曲率可能远大于起点处
  if (path_buffer_) {
    const double span_max_curvature = path_buffer_->maxCurvature(front_s, initial_s);
    diag_.max_curvature_on_reference_span = span_max_curvature;
    if (span_max_curvature > params_.max_curvature + params_.curvature_tolerance) {
      diag_.curvature_guard_triggered = true;
      return false;
    }
  }
  const double ref_limit = std::abs(params_.joint_limit) - params_.joint_limit_margin;
  for (double angle : theta_ref) {
    if (!std::isfinite(angle) || std::abs(angle) > ref_limit) {
      diag_.joint_limit_guard_triggered = true;
      return false;
    }
  }
  return true;
}

bool OnlinePathFollowingController3D::findPreviousChordPoint(
  const SimulationState3D & state, const HeadCommand3D & cmd,
  double front_s, const Vec3 & front_point, double & back_s, Vec3 & back_point,
  double previous_chord_s) const
{
  static_cast<void>(previous_chord_s);
  const double target = params_.link_length;
  const double step = std::max(params_.path_sample_ds, 1.0e-3);
  const double oldest = path_buffer_->actualOldestS();
  double near_s = front_s;
  double near_distance = 0.0;

  for (double candidate_s = front_s - step; candidate_s >= oldest - 1.0e-12;
    candidate_s -= step)
  {
    const double far_s = std::max(candidate_s, oldest);
    Vec3 far_point{}, tangent{};
    double curvature = 0.0;
    Mat3 frame{};
    if (!sampleJoint1Path(
        state, cmd, far_s, far_point, tangent, curvature, frame))
    {
      return false;
    }
    const double far_distance = norm(front_point - far_point);
    if (far_distance + params_.chord_distance_monotonicity_tolerance < near_distance) {
      diag_.chord_distance_nonmonotonic = true;
      return false;
    }
    if (near_distance < target && far_distance >= target) {
      double bracket_far_s = far_s;
      double bracket_near_s = near_s;
      Vec3 bracket_far_point = far_point;
      for (size_t iteration = 0; iteration < 40; ++iteration) {
        const double mid_s = 0.5 * (bracket_far_s + bracket_near_s);
        Vec3 mid_point{};
        if (!sampleJoint1Path(
            state, cmd, mid_s, mid_point, tangent, curvature, frame))
        {
          return false;
        }
        if (norm(front_point - mid_point) >= target) {
          bracket_far_s = mid_s;
          bracket_far_point = mid_point;
        } else {
          bracket_near_s = mid_s;
        }
      }
      back_s = bracket_far_s;
      back_point = bracket_far_point;
      return std::abs(norm(front_point - back_point) - target) <=
             params_.chord_residual_tolerance;
    }
    near_s = far_s;
    near_distance = far_distance;
    if (far_s <= oldest + 1.0e-12) {
      break;
    }
  }
  return false;
}

bool OnlinePathFollowingController3D::sampleJoint1Path(
  const SimulationState3D & state, const HeadCommand3D & cmd, double s,
  Vec3 & p, Vec3 & tangent, double & curvature, Mat3 & bishop_frame) const
{
  if (path_buffer_->empty() || s < path_buffer_->oldestS() - 1.0e-12) {
    return false;
  }
  if (s <= path_buffer_->newestS() + 1.0e-12) {
    return path_buffer_->sample(
      std::min(s, path_buffer_->newestS()), p, tangent, curvature, bishop_frame);
  }
  if (!params_.use_command_predictor_feedforward) {
    return false;
  }
  return predictFutureJoint1(
    state, cmd, s - path_buffer_->newestS(), p, tangent, curvature,
    bishop_frame, nullptr);
}

bool OnlinePathFollowingController3D::buildPredictedPathSegment(
  const SimulationState3D & state, const HeadCommand3D & cmd, double max_delta_s) const
{
  auto & segment = *predicted_path_segment_;
  segment = {};
  segment.requested_distance = max_delta_s;
  if (max_delta_s < -1.0e-12) {
    return false;
  }

  SimulationState3D predicted;
  predicted.head_frame = has_last_realized_head_frame_ ?
    last_realized_head_frame_ : state.head_frame;
  predicted.joints = state.joints;
  predicted.link_frames = linkFrames(predicted.head_frame, predicted.joints.theta);
  predicted.link_axes = linkAxes(predicted.link_frames);
  predicted.body_points = bodyPoints(
    predicted.head_position, predicted.link_axes, params_.link_length);
  const Vec3 initial_joint1 = joint1Point(predicted);
  Vec3 previous_joint1 = initial_joint1;
  Vec3 previous_tangent = commandedJoint1Velocity(predicted, cmd);
  if (norm(previous_tangent) < params_.predictor_min_speed) {
    return false;
  }
  previous_tangent = normalize(previous_tangent);
  Mat3 previous_bishop = makeFrameFromAxisAndReference(
    previous_tangent, predicted.head_frame);
  segment.samples.push_back({
      0.0, {0.0, 0.0, 0.0}, previous_tangent, 0.0, 0.0,
      previous_bishop, predicted.head_frame});

  double accumulated = 0.0;
  for (size_t iteration = 0;
    iteration < params_.predictor_max_iterations &&
    accumulated + params_.predictor_arc_length_tolerance < max_delta_s;
    ++iteration)
  {
    const Vec3 joint1_velocity = commandedJoint1Velocity(predicted, cmd);
    const double speed = norm(joint1_velocity);
    if (speed < params_.predictor_min_speed) {
      return false;
    }
    const double remaining = max_delta_s - accumulated;
    const double requested_step = std::min(params_.predictor_max_arc_step, remaining);
    const double step_dt = requested_step / speed;
    const SimulationState3D before = predicted;
    const Vec3 before_joint1 = previous_joint1;
    JointVelocity3D zero_joint_velocity{};
    plant_.apply(cmd, zero_joint_velocity, step_dt, predicted);
    Vec3 current_joint1 = joint1Point(predicted);
    double actual_step = norm(current_joint1 - before_joint1);
    if (!(actual_step > 1.0e-12) || !std::isfinite(actual_step)) {
      return false;
    }
    if (accumulated + actual_step > max_delta_s) {
      const double alpha = (max_delta_s - accumulated) / actual_step;
      current_joint1 = lerp(before_joint1, current_joint1, alpha);
      predicted.head_position = lerp(before.head_position, predicted.head_position, alpha);
      predicted.head_frame = slerpFrame(before.head_frame, predicted.head_frame, alpha);
      predicted.link_frames = linkFrames(predicted.head_frame, predicted.joints.theta);
      predicted.link_axes = linkAxes(predicted.link_frames);
      predicted.body_points = bodyPoints(
        predicted.head_position, predicted.link_axes, params_.link_length);
      actual_step = max_delta_s - accumulated;
    }
    accumulated += actual_step;
    Vec3 tangent = commandedJoint1Velocity(predicted, cmd);
    if (norm(tangent) < params_.predictor_min_speed) {
      tangent = current_joint1 - before_joint1;
    }
    if (norm(tangent) < 1.0e-12) {
      return false;
    }
    tangent = normalize(tangent);
    const double raw_curvature = std::acos(std::clamp(
        dot(previous_tangent, tangent), -1.0, 1.0)) /
      std::max(actual_step, 1.0e-12);
    const double filtered_curvature =
      (1.0 - params_.curvature_filter_alpha) *
      segment.samples.back().filtered_curvature +
      params_.curvature_filter_alpha * raw_curvature;
    const Mat3 bishop = advanceBishopFrame(
      previous_bishop, previous_tangent, tangent);
    segment.samples.push_back({
        accumulated, current_joint1 - initial_joint1, tangent,
        raw_curvature, filtered_curvature, bishop, predicted.head_frame});
    segment.max_filtered_curvature = std::max(
      segment.max_filtered_curvature, filtered_curvature);
    previous_joint1 = current_joint1;
    previous_tangent = tangent;
    previous_bishop = bishop;
  }

  segment.achieved_distance = accumulated;
  diag_.predictor_arc_length_error = std::abs(accumulated - max_delta_s);
  segment.valid = !segment.samples.empty() &&
    diag_.predictor_arc_length_error <= params_.predictor_arc_length_tolerance;
  if (!segment.valid) {
    diag_.predictor_arc_length_invalid = true;
  }
  return segment.valid;
}

bool OnlinePathFollowingController3D::predictFutureJoint1(
  const SimulationState3D & state, const HeadCommand3D & cmd, double delta_s,
  Vec3 & p, Vec3 & tangent, double & curvature, Mat3 & bishop_frame,
  Mat3 * predicted_head_frame) const
{
  if (delta_s < -1.0e-12) {
    return false;
  }
  if (!predicted_path_segment_->valid ||
    predicted_path_segment_->requested_distance +
    params_.predictor_arc_length_tolerance < delta_s)
  {
    if (!buildPredictedPathSegment(state, cmd, delta_s)) {
      return false;
    }
  }
  const auto & samples = predicted_path_segment_->samples;
  if (samples.empty()) {
    return false;
  }
  auto upper = std::upper_bound(
    samples.begin(), samples.end(), delta_s,
    [](double value, const PredictedPathSegment3D::Sample & sample) {
      return value < sample.delta_s;
    });
  const size_t hi = upper == samples.end() ? samples.size() - 1 :
    static_cast<size_t>(std::distance(samples.begin(), upper));
  const size_t lo = hi == 0 ? 0 : hi - 1;
  const double span = samples[hi].delta_s - samples[lo].delta_s;
  const double alpha = span > 1.0e-12 ?
    clamp01((delta_s - samples[lo].delta_s) / span) : 0.0;
  p = lerp(samples[lo].p, samples[hi].p, alpha);
  tangent = normalize(lerp(samples[lo].tangent, samples[hi].tangent, alpha));
  curvature = samples[lo].filtered_curvature * (1.0 - alpha) +
    samples[hi].filtered_curvature * alpha;
  bishop_frame = slerpFrame(
    samples[lo].bishop_frame, samples[hi].bishop_frame, alpha);
  if (predicted_head_frame) {
    *predicted_head_frame = slerpFrame(
      samples[lo].head_frame, samples[hi].head_frame, alpha);
  }
  return isFinite(p) && isFinite(tangent) && std::isfinite(curvature);
}

JointVelocity3D OnlinePathFollowingController3D::applyShapeReference(
  const std::array<double, kNum3dJointDofs> & theta_ref,
  const std::array<double, kNum3dJointDofs> & dtheta_ref_ds,
  double joint1_path_speed, double dt, const MeasuredRobotState3D & measured_state)
{
  JointVelocity3D output{};
  double theta_error_sum = 0.0;
  const double safe_joint_limit = std::abs(params_.joint_limit) - params_.joint_limit_margin;
  for (size_t i = 0; i < kNum3dJointDofs; ++i) {
    const double measured_theta = measured_state.joints.theta[i];
    const double error = wrapAngle(theta_ref[i] - measured_theta);
    theta_error_sum += error * error;
    const double theta_dot_ff = params_.feedforward_gain * dtheta_ref_ds[i] * joint1_path_speed;
    const double theta_dot_fb = params_.joint_tracking_gain * error;
    // 位置、速度、加速度约束求交集，避免串联限位导致越界
    double lower = -params_.joint_rate_limit;
    double upper = params_.joint_rate_limit;
    if (dt > 1.0e-9) {
      // 加速度约束
      const double accel_delta = params_.joint_acceleration_limit * dt;
      lower = std::max(lower, last_theta_dot_[i] - accel_delta);
      upper = std::min(upper, last_theta_dot_[i] + accel_delta);
      // 位置约束（关节限位）
      lower = std::max(lower, (-safe_joint_limit - measured_theta) / dt);
      upper = std::min(upper, (safe_joint_limit - measured_theta) / dt);
    }
    double theta_dot = 0.0;
    if (lower <= upper) {
      theta_dot = std::clamp(theta_dot_ff + theta_dot_fb, lower, upper);
    } else {
      // 约束不可行：输出安全命令
      diag_.joint_command_infeasible = true;
      theta_dot = 0.0;
    }
    last_theta_dot_[i] = theta_dot;
    output.theta_dot[i] = theta_dot;
    diag_.max_theta_dot_ff = std::max(diag_.max_theta_dot_ff, std::abs(theta_dot_ff));
    diag_.max_theta_dot_fb = std::max(diag_.max_theta_dot_fb, std::abs(theta_dot_fb));
    diag_.max_theta_dot_cmd = std::max(diag_.max_theta_dot_cmd, std::abs(theta_dot));
  }
  diag_.theta_ref_error_norm = std::sqrt(theta_error_sum);
  return output;
}

JointVelocity3D OnlinePathFollowingController3D::applyFrozenReference(
  double dt, const MeasuredRobotState3D & measured_state)
{
  std::array<double, kNum3dJointDofs> zero_feedforward{};
  return applyShapeReference(last_theta_ref_, zero_feedforward, 0.0, dt, measured_state);
}

HeadCommand3D OnlinePathFollowingController3D::filterCommand(const HeadCommand3D & cmd, double dt)
{
  HeadCommand3D target = clampCommandDelta(
    filtered_cmd_, cmd,
    std::max(0.0, params_.linear_accel_limit) * dt,
    std::max(0.0, params_.angular_rate_slew_limit) * dt);
  const double tau = std::max(0.0, params_.command_filter_time_constant);
  if (tau <= 1.0e-9) {
    filtered_cmd_ = target;
  } else {
    const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
    filtered_cmd_.linear_velocity += alpha *
      (target.linear_velocity - filtered_cmd_.linear_velocity);
    filtered_cmd_.pitch_rate += alpha * (target.pitch_rate - filtered_cmd_.pitch_rate);
    filtered_cmd_.yaw_rate += alpha * (target.yaw_rate - filtered_cmd_.yaw_rate);
  }
  return filtered_cmd_;
}

void OnlinePathFollowingController3D::resetFeedforwardFilter()
{
  filtered_dtheta_ref_ds_.fill(0.0);
  last_theta_dot_.fill(0.0);
  diag_.feedforward_enabled = false;
}

void OnlinePathFollowingController3D::warmupBuffer(
  const SimulationState3D & state, const HeadCommand3D & cmd)
{
  struct WarmupPoint
  {
    Vec3 p;
    RollingPathBuffer3D::Provenance provenance;
  };

  std::vector<Vec3> body_seed;
  for (size_t i = 1; i < state.body_points.size(); ++i) {
    if (body_seed.empty() || norm(state.body_points[i] - body_seed.back()) > 1.0e-12) {
      body_seed.push_back(state.body_points[i]);
    }
  }
  if (body_seed.size() < 2) {
    return;
  }

  double seed_length = 0.0;
  for (size_t i = 1; i < body_seed.size(); ++i) {
    seed_length += norm(body_seed[i] - body_seed[i - 1]);
  }
  const double target_length = requiredHistory() + params_.synthetic_tail_margin;
  const double extension_length = std::max(0.0, target_length - seed_length);
  const double ds = std::max(params_.path_sample_ds, 1.0e-3);
  const Vec3 tail = body_seed.back();
  Vec3 tail_tangent = normalize(tail - body_seed[body_seed.size() - 2]);
  if (norm(tail_tangent) < 1.0e-12) {
    tail_tangent = column(state.head_frame, 0);
  }

  std::vector<WarmupPoint> points;
  if (extension_length > 1.0e-12) {
    const size_t extension_samples = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(extension_length / ds)));
    for (size_t i = 0; i <= extension_samples; ++i) {
      const double distance = extension_length *
        (1.0 - static_cast<double>(i) / static_cast<double>(extension_samples));
      points.push_back({
          tail + distance * tail_tangent,
          RollingPathBuffer3D::Provenance::TailExtensionSynthetic});
    }
  }

  const size_t seed_samples = std::max<size_t>(
    1, static_cast<size_t>(std::ceil(seed_length / ds)));
  const size_t seed_start = points.empty() ? 0 : 1;
  for (size_t i = seed_start; i <= seed_samples; ++i) {
    const double target_distance = seed_length *
      static_cast<double>(i) / static_cast<double>(seed_samples);
    double accumulated = 0.0;
    Vec3 point = body_seed.back();
    for (int j = static_cast<int>(body_seed.size()) - 2; j >= 0; --j) {
      const double segment_length = norm(body_seed[j + 1] - body_seed[j]);
      if (accumulated + segment_length >= target_distance - 1.0e-12) {
        const double alpha = (target_distance - accumulated) /
          std::max(segment_length, 1.0e-12);
        point = lerp(body_seed[j + 1], body_seed[j], alpha);
        break;
      }
      accumulated += segment_length;
      point = body_seed[j];
    }
    if (points.empty() || norm(point - points.back().p) > 1.0e-12) {
      points.push_back({point, RollingPathBuffer3D::Provenance::BodySeedSynthetic});
    }
  }
  if (points.size() < 2) {
    return;
  }

  std::vector<double> arc_length(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arc_length[i] = arc_length[i - 1] + norm(points[i].p - points[i - 1].p);
  }
  const double total_length = arc_length.back();
  const Mat3 reference_frame = joint1FrameHint(state, cmd);
  std::vector<RollingPathBuffer3D::Sample> samples;
  samples.reserve(points.size());
  Vec3 previous_tangent = normalize(points[1].p - points[0].p);
  Mat3 previous_frame = makeFrameFromAxisAndReference(
    previous_tangent, reference_frame);
  double previous_filtered_curvature = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    Vec3 tangent = i + 1 < points.size() ?
      normalize(points[i + 1].p - points[i].p) : previous_tangent;
    const Mat3 frame = i == 0 ? previous_frame :
      advanceBishopFrame(previous_frame, previous_tangent, tangent);
    double raw_curvature = 0.0;
    if (i > 0) {
      const double segment_length = arc_length[i] - arc_length[i - 1];
      raw_curvature = std::acos(std::clamp(
          dot(previous_tangent, tangent), -1.0, 1.0)) /
        std::max(segment_length, 1.0e-12);
    }
    const double filtered_curvature =
      (1.0 - params_.curvature_filter_alpha) * previous_filtered_curvature +
      params_.curvature_filter_alpha * raw_curvature;
    const double sample_s = arc_length[i] - total_length;
    if (sample_s < -1.0e-12) {
      RollingPathBuffer3D::Sample sample;
      sample.s = sample_s;
      sample.time = state.time;
      sample.p = points[i].p - joint1Point(state);
      sample.tangent = tangent;
      sample.raw_curvature = raw_curvature;
      sample.filtered_curvature = filtered_curvature;
      sample.bishop_frame = frame;
      sample.provenance = points[i].provenance;
      sample.geometry_epoch = geometry_epoch_;
      samples.push_back(sample);
    }
    previous_tangent = tangent;
    previous_frame = frame;
    previous_filtered_curvature = filtered_curvature;
  }
  path_buffer_->resetWithSeedSamples(
    samples, reference_frame, state.time);
}

// ============================================================================
// 文档 §8.6 完整三层分支树弦搜索实现
// ============================================================================

bool OnlinePathFollowingController3D::computeShapeReferenceMultiBranch(
  const SimulationState3D & state, const HeadCommand3D & cmd, double s_joint1,
  ShapeReferenceCandidate3D & candidate)
{
  candidate = {};

  const Mat3 link0_frame = has_last_realized_head_frame_ ?
    last_realized_head_frame_ : state.head_frame;

  // 文档 (8.84)：当前实测 J1 锚点是 s_0^root = s_0|k^root
  Vec3 s0_point{}, tangent{};
  double curvature = 0.0;
  Mat3 path_frame{};
  if (!sampleJoint1Path(state, cmd, s_joint1, s0_point, tangent, curvature, path_frame)) {
    candidate.status = ReferenceStatus::NoReference;
    return false;
  }

  // 初始化深度 0 的根节点
  std::array<double, 4> s_roots{s_joint1, 0.0, 0.0, 0.0};
  std::array<Vec3, 4> root_points{s0_point, {}, {}, {}};

  // 文档 (8.102-8.104)：递归展开三层分支树
  std::vector<ReconstructionBranch> all_branches;
  all_branches.reserve(params_.max_tree_nodes);

  // 深度 1：从 s_0 生成第一层候选根
  auto depth1_candidates = generateRootCandidates(state, cmd, s_roots[0], root_points[0]);

  size_t tree_nodes = 0;
  size_t segments_scanned = depth1_candidates.size();

  for (const auto & c1 : depth1_candidates) {
    if (!c1.valid || tree_nodes >= params_.max_tree_nodes) {continue;}
    s_roots[1] = c1.s;
    root_points[1] = c1.point;
    ++tree_nodes;

    // 深度 2：从 s_1 生成第二层候选根
    auto depth2_candidates = generateRootCandidates(state, cmd, s_roots[1], root_points[1]);
    segments_scanned += depth2_candidates.size();

    for (const auto & c2 : depth2_candidates) {
      if (!c2.valid || tree_nodes >= params_.max_tree_nodes) {continue;}
      s_roots[2] = c2.s;
      root_points[2] = c2.point;
      ++tree_nodes;

      // 深度 3：从 s_2 生成第三层候选根
      auto depth3_candidates = generateRootCandidates(state, cmd, s_roots[2], root_points[2]);
      segments_scanned += depth3_candidates.size();

      for (const auto & c3 : depth3_candidates) {
        if (!c3.valid || tree_nodes >= params_.max_tree_nodes) {continue;}
        s_roots[3] = c3.s;
        root_points[3] = c3.point;
        ++tree_nodes;

        // 展开完整分支并评分
        ReconstructionBranch branch;
        if (expandBranch(state, cmd, s_roots, link0_frame, branch)) {
          scoreBranch(branch, held_reference_);
          all_branches.push_back(branch);
        }

        if (segments_scanned >= params_.max_segments_scanned) {
          break;
        }
      }
      if (segments_scanned >= params_.max_segments_scanned) {
        break;
      }
    }
    if (segments_scanned >= params_.max_segments_scanned) {
      break;
    }
  }

  // 文档 (8.105)：预算守卫
  if (tree_nodes >= params_.max_tree_nodes || segments_scanned >= params_.max_segments_scanned) {
    diag_.geometry_budget_exceeded = true;
  }

  // 文档 (8.107)：提取 T_k^hard（通过所有非降级守卫的分支）
  std::vector<ReconstructionBranch> hard_branches;
  for (const auto & b : all_branches) {
    if (b.is_hard_admissible) {
      hard_branches.push_back(b);
    }
  }

  if (hard_branches.empty()) {
    // 无可行分支，尝试降级或 hold
    candidate.status = evaluateFallbackStatus();
    return candidate.status != ReferenceStatus::NoReference;
  }

  // 文档 (8.117)：词典序选择最佳分支
  size_t best_idx = 0;
  double margin = 0.0;
  if (!selectBestBranch(hard_branches, best_idx, margin)) {
    candidate.status = ReferenceStatus::NoReference;
    return false;
  }
  hard_branches[best_idx].branch_margin = margin;

  const auto & best = hard_branches[best_idx];

  // 文档 (8.118)：最优与次优分支得分差小于 ε_J,amb 时判定为分支歧义——
  // 几何上仍然可行，但对该分支的置信度不足以授予最高的 EXACT_NEW 状态。
  diag_.branch_selection_margin = margin;
  const bool ambiguous = margin < params_.branch_ambiguity_margin;
  diag_.branch_selection_ambiguous = ambiguous;
  diag_.strict_continuity_stall_override = false;

  // 文档 (8.144)：确定引用状态
  if (best.is_strict && best.continuity_ok) {
    // 几何严格且连续性守卫通过：不存在"连续性拒绝了严格分支"的情形，
    // 停滞计数器清零；分支歧义仅影响是否授予最高置信度。
    strict_only_continuity_reject_count_ = 0;
    candidate.status = ambiguous ?
      ReferenceStatus::DegradedContinuous : ReferenceStatus::ExactNew;
  } else if (best.is_degraded && best.continuity_ok) {
    candidate.status = ReferenceStatus::DegradedContinuous;
  } else if (best.is_strict && !best.continuity_ok) {
    // 文档 (8.132)：几何上严格可行的分支被连续性守卫连续拒绝达到阈值，
    // 说明 held prior 已与当前几何不再匹配（例如刚经历一次急转弯），
    // 强制接受该分支并重新锚定 prior，避免永久性锁死。
    ++strict_only_continuity_reject_count_;
    if (strict_only_continuity_reject_count_ >= params_.max_strict_continuity_stall_count) {
      candidate.status = ReferenceStatus::ExactNew;
      diag_.strict_continuity_stall_override = true;
      strict_only_continuity_reject_count_ = 0;
    } else {
      candidate.status = ReferenceStatus::HoldLast;
    }
  } else {
    candidate.status = ReferenceStatus::HoldLast;
  }

  // 填充候选输出
  candidate.theta_ref = best.theta_ref;
  candidate.chord_s = best.chord_s;
  candidate.chord_residual = best.chord_residual;
  candidate.chord_axes = best.chord_axes;
  candidate.max_axis_alignment_error = best.max_axis_alignment_error;
  candidate.curvature = best.max_curvature_on_span;
  candidate.reference_span = best.reference_span;
  candidate.current_geometry_valid = true;

  // 文档 (8.135, 8.148)：更新 held reference record
  if (candidate.status == ReferenceStatus::ExactNew ||
    candidate.status == ReferenceStatus::DegradedContinuous)
  {
    held_reference_.theta_held = best.theta_ref;
    held_reference_.chord_s_held = best.chord_s;
    held_reference_.chord_axes_held = best.chord_axes;
    held_reference_.geometry_epoch = geometry_epoch_;
    held_reference_.valid = true;
  }

  // 更新 prior 用于下一轮连续性检查
  previous_chord_s_ = best.chord_s;
  previous_chord_axes_ = best.chord_axes;
  has_previous_chord_solution_ = true;
  last_reference_status_ = candidate.status;

  return true;
}

std::vector<OnlinePathFollowingController3D::ChordRootCandidate>
OnlinePathFollowingController3D::generateRootCandidates(
  const SimulationState3D & state, const HeadCommand3D & cmd,
  double parent_s, const Vec3 & parent_point) const
{
  std::vector<ChordRootCandidate> candidates;
  const double L = params_.link_length;
  const double step = params_.root_search_step;
  const double oldest = path_buffer_->actualOldestS();
  const double search_min = std::max(oldest, parent_s - params_.history_length);

  // 文档 (8.87)：后向搜索区间
  double near_s = parent_s;
  double near_distance = 0.0;
  Vec3 near_point = parent_point;

  // 存储 bracket 时需要保存对应的 near_s（bracket 的上界）
  struct BracketInfo
  {
    double far_s;
    Vec3 far_point;
    double near_s;
    Vec3 near_point;
  };
  std::vector<BracketInfo> bracket_candidates;

  for (double candidate_s = parent_s - step; candidate_s >= search_min - 1.0e-12;
    candidate_s -= step)
  {
    const double far_s = std::max(candidate_s, oldest);
    Vec3 far_point{}, tangent{};
    double curv = 0.0;
    Mat3 frame{};

    if (!sampleJoint1Path(state, cmd, far_s, far_point, tangent, curv, frame)) {
      break;
    }

    const double far_distance = norm(parent_point - far_point);

    // 文档 (8.90-8.94)：检查弦条件（段-球交点）
    if (near_distance < L && far_distance >= L) {
      bracket_candidates.push_back({far_s, far_point, near_s, near_point});

      // 限制候选数量
      if (bracket_candidates.size() >= params_.max_root_candidates_per_parent) {
        break;
      }
    }

    near_s = far_s;
    near_distance = far_distance;
    near_point = far_point;

    if (far_s <= oldest + 1.0e-12) {
      break;
    }
  }

  // 对每个 bracket 进行二分精化（文档 (8.95) 求根）
  for (const auto & bracket : bracket_candidates) {
    double lo_s = bracket.far_s;
    double hi_s = bracket.near_s;
    Vec3 result_point = bracket.far_point;

    for (size_t iter = 0; iter < 40; ++iter) {
      const double mid_s = 0.5 * (lo_s + hi_s);
      Vec3 mid_point{}, tangent{};
      double curv = 0.0;
      Mat3 frame{};

      if (!sampleJoint1Path(state, cmd, mid_s, mid_point, tangent, curv, frame)) {
        break;
      }

      if (norm(parent_point - mid_point) >= L) {
        lo_s = mid_s;
        result_point = mid_point;
      } else {
        hi_s = mid_s;
      }
    }

    ChordRootCandidate cand;
    cand.s = lo_s;
    cand.point = result_point;
    cand.axis = normalize(parent_point - result_point);
    cand.residual = std::abs(norm(parent_point - result_point) - L);
    cand.valid = cand.residual <= params_.chord_residual_tolerance;

    if (cand.valid) {
      candidates.push_back(cand);
    }
  }

  // 文档 (8.100)：合并相邻重复根
  if (candidates.size() > 1) {
    std::vector<ChordRootCandidate> merged;
    merged.push_back(candidates[0]);
    for (size_t i = 1; i < candidates.size(); ++i) {
      if (std::abs(candidates[i].s - merged.back().s) > 1.0e-6) {
        merged.push_back(candidates[i]);
      }
    }
    candidates = merged;
  }

  return candidates;
}

bool OnlinePathFollowingController3D::checkChordMonotonicity(
  const SimulationState3D & state, const HeadCommand3D & cmd,
  double parent_s, const Vec3 & parent_point, double child_s,
  double & violation) const
{
  // violation 是沿 [child_s, parent_s] 观测到的最大原始回退幅度
  // (未扣除容差)，与 chord_residual 采用相同的连续评分约定：
  // violation <= tolerance 时仍视为单调（可能有得分惩罚），
  // 超过 tolerance 时由调用方判定为硬性不可行。
  violation = 0.0;
  const double step = std::max(params_.root_search_step, 1.0e-4);
  double near_distance = 0.0;
  for (double s = parent_s - step; s > child_s + 1.0e-12; s -= step) {
    const double sample_s = std::max(s, child_s);
    Vec3 p{}, tangent{};
    double curvature = 0.0;
    Mat3 frame{};
    if (!sampleJoint1Path(state, cmd, sample_s, p, tangent, curvature, frame)) {
      return false;
    }
    const double distance = norm(parent_point - p);
    violation = std::max(violation, near_distance - distance);
    near_distance = distance;
    if (sample_s <= child_s + 1.0e-12) {
      break;
    }
  }
  return true;
}

bool OnlinePathFollowingController3D::expandBranch(
  const SimulationState3D & state, const HeadCommand3D & cmd,
  const std::array<double, 4> & s_roots, const Mat3 & link0_frame,
  ReconstructionBranch & branch) const
{
  branch = {};

  // 填充弦根坐标（跳过 s_0 = 当前锚点）
  for (size_t i = 0; i < 3; ++i) {
    branch.chord_s[i] = s_roots[i + 1];
  }

  // 计算弦轴、残差和单调性
  for (size_t link = 0; link < 3; ++link) {
    const double parent_s = s_roots[link];
    const double child_s = s_roots[link + 1];

    Vec3 parent_p{}, child_p{}, tangent{};
    double curv = 0.0;
    Mat3 frame{};

    if (!sampleJoint1Path(state, cmd, parent_s, parent_p, tangent, curv, frame) ||
      !sampleJoint1Path(state, cmd, child_s, child_p, tangent, curv, frame))
    {
      return false;
    }

    const Vec3 chord = parent_p - child_p;
    const double chord_len = norm(chord);
    if (chord_len < 1.0e-9) {
      return false;
    }

    branch.chord_axes[link] = normalize(chord);
    branch.chord_residual[link] = std::abs(chord_len - params_.link_length);

    // 文档 (8.101)：残差守卫
    if (branch.chord_residual[link] > params_.chord_residual_tolerance) {
      branch.residual_ok = false;
    }

    // 文档 (8.90-8.94)：单调性守卫。generateRootCandidates 只保证在越过
    // L 的那一段满足 bracket 条件，这里沿整个 [child_s, parent_s] 复核，
    // 防止路径在到达该根之前先出现非单调回退（可能对应错误的分支）。
    double monotonicity_violation = 0.0;
    if (!checkChordMonotonicity(state, cmd, parent_s, parent_p, child_s, monotonicity_violation)) {
      return false;
    }
    branch.monotonicity_violation[link] = monotonicity_violation;
    if (monotonicity_violation > params_.chord_distance_monotonicity_tolerance) {
      branch.monotonicity_ok = false;
    }
  }

  // 文档 (8.125-8.127)：IK 提取关节角
  double min_proj = 1.0;
  const bool ik_ok = computeThetaFromLinkAxes(
    link0_frame, branch.chord_axes, params_.axis_singularity_epsilon,
    branch.theta_ref, &branch.link_frames_ref,
    &branch.max_axis_alignment_error, &min_proj);

  branch.min_axis_projection = min_proj;

  if (!ik_ok || branch.max_axis_alignment_error > params_.max_axis_alignment_tolerance) {
    return false;
  }

  // 文档 (8.129)：关节限位守卫
  const double ref_limit = std::abs(params_.joint_limit) - params_.joint_limit_margin;
  for (double angle : branch.theta_ref) {
    if (!std::isfinite(angle) || std::abs(angle) > ref_limit) {
      branch.joint_limit_ok = false;
      return false;
    }
  }

  // 计算引用跨度和曲率
  branch.reference_span = s_roots[0] - s_roots[3];
  branch.max_curvature_on_span = path_buffer_->maxCurvature(s_roots[3], s_roots[0]);

  // 文档 (8.106)：曲率守卫
  if (branch.max_curvature_on_span > params_.max_curvature + params_.curvature_tolerance) {
    branch.curvature_ok = false;
  }

  // 文档 (8.106, 8.140)：轴投影守卫（严格/降级分类）
  bool all_strict = true;
  bool any_degraded = false;

  for (size_t r = 0; r < 3; ++r) {
    if (min_proj < params_.axis_exit_threshold) {
      branch.axis_projection_ok = false;
      return false;
    }
    if (min_proj < params_.axis_enter_threshold) {
      all_strict = false;
      any_degraded = true;
    }
  }

  branch.is_strict = all_strict;
  branch.is_degraded = any_degraded && !all_strict;

  // 文档 (8.106)：common admissible（不区分严格/降级的公共前置守卫）
  branch.is_common_admissible =
    branch.monotonicity_ok && branch.residual_ok &&
    branch.curvature_ok && branch.joint_limit_ok;

  // 文档 (8.107)：hard admissible = common admissible ∧ 轴投影守卫
  branch.is_hard_admissible = branch.is_common_admissible && branch.axis_projection_ok;

  branch.valid = branch.is_hard_admissible;
  return branch.valid;
}

void OnlinePathFollowingController3D::scoreBranch(
  ReconstructionBranch & branch,
  const HeldReferenceRecord & held_prior) const
{
  // 文档 (8.115)：数据项得分
  double residual_term = 0.0;
  double curvature_term = 0.0;
  double monotonicity_term = 0.0;

  for (size_t r = 0; r < 3; ++r) {
    residual_term += std::pow(branch.chord_residual[r] / params_.chord_residual_tolerance, 2.0);
    monotonicity_term += std::pow(
      branch.monotonicity_violation[r] / params_.chord_distance_monotonicity_tolerance, 2.0);
  }

  if (params_.max_curvature > 1.0e-9) {
    curvature_term = std::pow(branch.max_curvature_on_span / params_.max_curvature, 2.0);
  }

  branch.J_data =
    params_.score_weight_residual * residual_term +
    params_.score_weight_curvature * curvature_term +
    params_.score_weight_monotonicity * monotonicity_term;

  // 文档 (8.115)：prior 连续性得分
  branch.J_prior = 0.0;

  if (held_prior.valid && held_prior.geometry_epoch == geometry_epoch_) {
    double axis_continuity_penalty = 0.0;
    double root_continuity_penalty = 0.0;

    for (size_t r = 0; r < 3; ++r) {
      const double axis_dot = dot(branch.chord_axes[r], held_prior.chord_axes_held[r]);
      const double root_diff = std::abs(branch.chord_s[r] - held_prior.chord_s_held[r]);

      axis_continuity_penalty += std::pow(1.0 - axis_dot, 2.0);
      root_continuity_penalty += std::pow(
        root_diff / params_.max_chord_progress_per_step, 2.0);
    }

    branch.J_prior =
      params_.score_weight_axis_continuity * axis_continuity_penalty +
      params_.score_weight_root_continuity * root_continuity_penalty;
  }

  // 文档 (8.115)：总分
  branch.J_branch = branch.J_data + params_.score_weight_prior * branch.J_prior;

  // 文档 (8.131, 8.139)：连续性守卫
  if (held_prior.valid && held_prior.geometry_epoch == geometry_epoch_) {
    bool strict_cont = true;
    bool deg_cont = true;

    for (size_t r = 0; r < 3; ++r) {
      const double theta_jump = std::abs(
        wrapAngle(branch.theta_ref[r] - held_prior.theta_held[r]));
      const double root_jump = std::abs(branch.chord_s[r] - held_prior.chord_s_held[r]);
      const double axis_dot = dot(branch.chord_axes[r], held_prior.chord_axes_held[r]);

      if (theta_jump > params_.max_strict_theta_jump ||
        root_jump > params_.max_strict_root_jump ||
        axis_dot < params_.min_strict_axis_dot)
      {
        strict_cont = false;
      }

      if (theta_jump > params_.max_degraded_theta_jump ||
        root_jump > params_.max_degraded_root_jump ||
        axis_dot < params_.min_degraded_axis_dot)
      {
        deg_cont = false;
      }
    }

    if (branch.is_strict) {
      branch.continuity_ok = strict_cont;
    } else if (branch.is_degraded) {
      branch.continuity_ok = deg_cont;
    }
  } else {
    // 无 prior，连续性默认通过
    branch.continuity_ok = true;
  }
}

bool OnlinePathFollowingController3D::selectBestBranch(
  const std::vector<ReconstructionBranch> & branches,
  size_t & best_index, double & margin) const
{
  if (branches.empty()) {
    return false;
  }

  // 文档 (8.117)：词典序选择
  best_index = 0;
  double best_score = branches[0].J_branch;

  for (size_t i = 1; i < branches.size(); ++i) {
    if (branches[i].J_branch < best_score) {
      best_score = branches[i].J_branch;
      best_index = i;
    }
  }

  // 文档 (8.118)：计算与次优的得分差
  double second_best_score = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < branches.size(); ++i) {
    if (i != best_index && branches[i].J_branch < second_best_score) {
      second_best_score = branches[i].J_branch;
    }
  }

  margin = (branches.size() > 1) ? (second_best_score - best_score) :
    std::numeric_limits<double>::infinity();

  return true;
}

OnlinePathFollowingController3D::ReferenceStatus
OnlinePathFollowingController3D::evaluateFallbackStatus() const
{
  // 文档 (8.144)：无 hard 分支时的降级策略
  if (held_reference_.valid && held_reference_.geometry_epoch == geometry_epoch_) {
    return ReferenceStatus::HoldLast;
  }
  return ReferenceStatus::NoReference;
}

}  // namespace asr

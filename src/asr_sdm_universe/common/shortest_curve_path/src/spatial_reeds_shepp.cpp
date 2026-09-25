#include "shortest_curve_path/spatial_reeds_shepp.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace shortest_curve_path
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kFreqEps = 1.0e-12;
constexpr double kLengthEps = 1.0e-9;
constexpr double kAngleEps = 1.0e-9;
constexpr double kPitchLimit = 0.5 * kPi - 1.0e-3;
constexpr double kPerpTol = 1.0e-3;
constexpr double kRefineTol = 1.0e-6;

double wrapToPiImpl(double angle)
{
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

double shortestYawDelta(double from, double to) { return wrapToPiImpl(to - from); }

Eigen::Vector3d headingImpl(double yaw, double pitch)
{
  const double cp = std::cos(pitch);
  return {cp * std::cos(yaw), cp * std::sin(yaw), -std::sin(pitch)};
}

double integrateCos(double freq, double phase, double length)
{
  if (std::abs(freq) < kFreqEps) {
    return length * std::cos(phase);
  }
  return (std::sin(freq * length + phase) - std::sin(phase)) / freq;
}

double integrateSin(double freq, double phase, double length)
{
  if (std::abs(freq) < kFreqEps) {
    return length * std::sin(phase);
  }
  return (-std::cos(freq * length + phase) + std::cos(phase)) / freq;
}

Eigen::Vector3d curveDisplacement(
  double yaw0, double pitch0, double kappa_yaw, double kappa_pitch, double length, double gear)
{
  if (length <= kLengthEps) {
    return Eigen::Vector3d::Zero();
  }

  const double ix = 0.5 * (integrateCos(kappa_pitch + kappa_yaw, pitch0 + yaw0, length) +
                           integrateCos(kappa_pitch - kappa_yaw, pitch0 - yaw0, length));
  const double iy = 0.5 * (integrateSin(kappa_yaw + kappa_pitch, yaw0 + pitch0, length) +
                           integrateSin(kappa_yaw - kappa_pitch, yaw0 - pitch0, length));
  const double iz = -integrateSin(kappa_pitch, pitch0, length);
  return gear * Eigen::Vector3d(ix, iy, iz);
}

Pose3d integrateSegment(const Pose3d & start, const Path3dSegment & segment)
{
  Pose3d out = start;
  out.position += curveDisplacement(
    start.yaw, start.pitch, segment.kappa_yaw, segment.kappa_pitch, segment.length, segment.gear);
  out.yaw = wrapToPiImpl(start.yaw + segment.kappa_yaw * segment.length);
  out.pitch = start.pitch + segment.kappa_pitch * segment.length;
  return out;
}

bool pitchInRange(double pitch) { return std::abs(pitch) <= kPitchLimit; }

struct CurvePiece
{
  Path3dSegment segment;
  bool valid = false;
};

CurvePiece makeConstantRate(
  double /*yaw0*/, double pitch0, double d_yaw, double d_pitch, double length, double gear)
{
  CurvePiece piece;
  if (length <= kLengthEps) {
    piece.valid = true;
    piece.segment.gear = gear;
    return piece;
  }
  if (!pitchInRange(pitch0) || !pitchInRange(pitch0 + d_pitch)) {
    return piece;
  }

  piece.segment.length = length;
  piece.segment.gear = gear;
  piece.segment.kappa_yaw = d_yaw / length;
  piece.segment.kappa_pitch = d_pitch / length;
  piece.valid = true;
  return piece;
}

std::vector<Path3dSegment> parallelHeadingChange(
  double yaw0, double pitch0, double d_yaw, double d_pitch, double gear, double rho_yaw,
  double rho_pitch)
{
  const double length = std::max(std::abs(d_yaw) * rho_yaw, std::abs(d_pitch) * rho_pitch);
  const CurvePiece piece = makeConstantRate(yaw0, pitch0, d_yaw, d_pitch, length, gear);
  if (!piece.valid) {
    return {};
  }
  if (piece.segment.length <= kLengthEps) {
    return {};
  }
  return {piece.segment};
}

std::vector<Path3dSegment> sequentialHeadingChange(
  double yaw0, double pitch0, double d_yaw, double d_pitch, double gear, double rho_yaw,
  double rho_pitch, bool yaw_first)
{
  std::vector<Path3dSegment> pieces;
  pieces.reserve(2);

  const auto append = [&](double y0, double p0, double dy, double dp) {
    const double axis_length = std::abs(dy) > kAngleEps
                                 ? std::abs(dy) * rho_yaw
                                 : (std::abs(dp) > kAngleEps ? std::abs(dp) * rho_pitch : 0.0);
    const CurvePiece piece = makeConstantRate(y0, p0, dy, dp, axis_length, gear);
    if (!piece.valid) {
      pieces.clear();
      return false;
    }
    if (piece.segment.length > kLengthEps) {
      pieces.push_back(piece.segment);
    }
    return true;
  };

  if (yaw_first) {
    if (!append(yaw0, pitch0, d_yaw, 0.0)) {
      return {};
    }
    if (!append(yaw0 + d_yaw, pitch0, 0.0, d_pitch)) {
      return {};
    }
  } else {
    if (!append(yaw0, pitch0, 0.0, d_pitch)) {
      return {};
    }
    if (!append(yaw0, pitch0 + d_pitch, d_yaw, 0.0)) {
      return {};
    }
  }
  return pieces;
}

std::vector<double> yawDeltaCandidates(double from, double to)
{
  const double wrap = shortestYawDelta(from, to);
  return {wrap, wrap + kTwoPi, wrap - kTwoPi};
}

enum class HeadingPrimitive
{
  kParallel = 0,
  kYawThenPitch = 1,
  kPitchThenYaw = 2,
};

std::vector<Path3dSegment> buildHeadingChange(
  HeadingPrimitive primitive, double yaw0, double pitch0, double d_yaw, double d_pitch, double gear,
  double rho_yaw, double rho_pitch)
{
  switch (primitive) {
    case HeadingPrimitive::kParallel:
      return parallelHeadingChange(yaw0, pitch0, d_yaw, d_pitch, gear, rho_yaw, rho_pitch);
    case HeadingPrimitive::kYawThenPitch:
      return sequentialHeadingChange(
        yaw0, pitch0, d_yaw, d_pitch, gear, rho_yaw, rho_pitch, true);
    case HeadingPrimitive::kPitchThenYaw:
      return sequentialHeadingChange(
        yaw0, pitch0, d_yaw, d_pitch, gear, rho_yaw, rho_pitch, false);
  }
  return {};
}

Pose3d applySegments(const Pose3d & start, const std::vector<Path3dSegment> & segments)
{
  Pose3d pose = start;
  for (const Path3dSegment & segment : segments) {
    pose = integrateSegment(pose, segment);
  }
  return pose;
}

double segmentsLength(const std::vector<Path3dSegment> & segments)
{
  double length = 0.0;
  for (const Path3dSegment & segment : segments) {
    length += segment.length;
  }
  return length;
}

struct Candidate
{
  std::vector<Path3dSegment> segments;
  double length = std::numeric_limits<double>::infinity();
  double perp_error = std::numeric_limits<double>::infinity();
};

void consider(
  Candidate & best, const Pose3d & start, const std::vector<Path3dSegment> & start_curve,
  const std::vector<Path3dSegment> & end_curve, const Pose3d & goal)
{
  if (start_curve.empty() && end_curve.empty()) {
    const Eigen::Vector3d delta = goal.position - start.position;
    const Eigen::Vector3d h = headingImpl(start.yaw, start.pitch);
    const double along = delta.dot(h);
    const Eigen::Vector3d perp = delta - along * h;
    const double perp_error = perp.norm();
    Path3dSegment straight;
    straight.length = std::abs(along);
    straight.gear = along >= 0.0 ? 1.0 : -1.0;
    Candidate candidate;
    candidate.perp_error = perp_error;
    candidate.length = straight.length;
    if (straight.length > kLengthEps) {
      candidate.segments.push_back(straight);
    }
    const bool better = candidate.perp_error <= kPerpTol
                          ? (best.perp_error > kPerpTol || candidate.length < best.length)
                          : (candidate.perp_error < best.perp_error - 1.0e-9 ||
                             (std::abs(candidate.perp_error - best.perp_error) <= 1.0e-9 &&
                              candidate.length < best.length));
    if (better) {
      best = std::move(candidate);
    }
    return;
  }

  const Pose3d after_start = applySegments(start, start_curve);
  Pose3d before_end = goal;
  // Integrate the end curve backward: reverse gear and reverse heading rates.
  for (auto it = end_curve.rbegin(); it != end_curve.rend(); ++it) {
    Path3dSegment backward = *it;
    backward.gear = -it->gear;
    backward.kappa_yaw = -it->kappa_yaw;
    backward.kappa_pitch = -it->kappa_pitch;
    before_end = integrateSegment(before_end, backward);
  }

  const Eigen::Vector3d residual = before_end.position - after_start.position;
  const Eigen::Vector3d h = headingImpl(after_start.yaw, after_start.pitch);
  const double along = residual.dot(h);
  const Eigen::Vector3d perp = residual - along * h;
  const double perp_error = perp.norm();

  Candidate candidate;
  candidate.perp_error = perp_error;
  candidate.segments = start_curve;
  if (std::abs(along) > kLengthEps) {
    Path3dSegment straight;
    straight.length = std::abs(along);
    straight.gear = along >= 0.0 ? 1.0 : -1.0;
    candidate.segments.push_back(straight);
  }
  candidate.segments.insert(candidate.segments.end(), end_curve.begin(), end_curve.end());
  candidate.length = segmentsLength(candidate.segments);

  const bool better = candidate.perp_error <= kPerpTol
                        ? (best.perp_error > kPerpTol || candidate.length < best.length)
                        : (candidate.perp_error < best.perp_error - 1.0e-9 ||
                           (std::abs(candidate.perp_error - best.perp_error) <= 1.0e-9 &&
                            candidate.length < best.length));
  if (better) {
    best = std::move(candidate);
  }
}

struct HeadingConnect
{
  std::vector<Path3dSegment> start_curve;
  std::vector<Path3dSegment> end_curve;
  bool valid = false;
};

HeadingConnect connectThroughMid(
  const Pose3d & start, const Pose3d & goal, double yaw_m, double pitch_m, double gear_start,
  double gear_end, HeadingPrimitive start_prim, HeadingPrimitive end_prim, double d_yaw_start,
  double d_yaw_end, double rho_yaw, double rho_pitch)
{
  HeadingConnect out;
  if (!pitchInRange(pitch_m)) {
    return out;
  }

  const double d_pitch_start = pitch_m - start.pitch;
  const double d_pitch_end = goal.pitch - pitch_m;
  out.start_curve = buildHeadingChange(
    start_prim, start.yaw, start.pitch, d_yaw_start, d_pitch_start, gear_start, rho_yaw, rho_pitch);
  out.end_curve = buildHeadingChange(
    end_prim, yaw_m, pitch_m, d_yaw_end, d_pitch_end, gear_end, rho_yaw, rho_pitch);

  if ((std::abs(d_yaw_start) > kAngleEps || std::abs(d_pitch_start) > kAngleEps) &&
      out.start_curve.empty()) {
    return out;
  }
  if ((std::abs(d_yaw_end) > kAngleEps || std::abs(d_pitch_end) > kAngleEps) &&
      out.end_curve.empty()) {
    return out;
  }
  out.valid = true;
  return out;
}

struct ConnectionGeometry
{
  bool valid = false;
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Vector3d heading = Eigen::Vector3d::UnitX();
  Eigen::Vector2d perp_coords = Eigen::Vector2d::Zero();
  double perp_error = std::numeric_limits<double>::infinity();
};

void attachYawFamily(double start_yaw, double goal_yaw, double yaw_m, double & d_yaw_start, double & d_yaw_end)
{
  const double start_off = std::round((d_yaw_start - shortestYawDelta(start_yaw, yaw_m)) / kTwoPi);
  const double end_off = std::round((d_yaw_end - shortestYawDelta(yaw_m, goal_yaw)) / kTwoPi);
  d_yaw_start = shortestYawDelta(start_yaw, yaw_m) + start_off * kTwoPi;
  d_yaw_end = shortestYawDelta(yaw_m, goal_yaw) + end_off * kTwoPi;
}

ConnectionGeometry connectionGeometry(
  const Pose3d & start, const Pose3d & goal, double yaw_m, double pitch_m, double gear_start,
  double gear_end, HeadingPrimitive start_prim, HeadingPrimitive end_prim, double d_yaw_start,
  double d_yaw_end, double rho_yaw, double rho_pitch)
{
  ConnectionGeometry out;
  const HeadingConnect link = connectThroughMid(
    start, goal, yaw_m, pitch_m, gear_start, gear_end, start_prim, end_prim, d_yaw_start, d_yaw_end,
    rho_yaw, rho_pitch);
  if (!link.valid) {
    return out;
  }

  const Pose3d after_start = applySegments(start, link.start_curve);
  Pose3d before_end = goal;
  for (auto it = link.end_curve.rbegin(); it != link.end_curve.rend(); ++it) {
    Path3dSegment backward = *it;
    backward.gear = -it->gear;
    backward.kappa_yaw = -it->kappa_yaw;
    backward.kappa_pitch = -it->kappa_pitch;
    before_end = integrateSegment(before_end, backward);
  }

  out.valid = true;
  out.residual = before_end.position - after_start.position;
  out.heading = headingImpl(after_start.yaw, after_start.pitch);
  const Eigen::Vector3d rperp = out.residual - out.residual.dot(out.heading) * out.heading;
  Eigen::Vector3d e1 = out.heading.unitOrthogonal();
  if (e1.squaredNorm() < 1.0e-12) {
    e1 = Eigen::Vector3d::UnitY();
  }
  const Eigen::Vector3d e2 = out.heading.cross(e1).normalized();
  out.perp_coords = Eigen::Vector2d(rperp.dot(e1.normalized()), rperp.dot(e2));
  out.perp_error = rperp.norm();
  return out;
}

bool refineMidHeading(
  const Pose3d & start, const Pose3d & goal, double & yaw_m, double & pitch_m, double gear_start,
  double gear_end, HeadingPrimitive start_prim, HeadingPrimitive end_prim, double & d_yaw_start,
  double & d_yaw_end, double rho_yaw, double rho_pitch)
{
  constexpr double kStep = 1.0e-4;

  auto eval = [&](double yaw, double pitch, double dy0, double dy1) {
    return connectionGeometry(
      start, goal, yaw, pitch, gear_start, gear_end, start_prim, end_prim, dy0, dy1, rho_yaw,
      rho_pitch);
  };

  ConnectionGeometry current = eval(yaw_m, pitch_m, d_yaw_start, d_yaw_end);
  if (!current.valid) {
    return false;
  }

  for (int iter = 0; iter < 16; ++iter) {
    if (current.perp_error < kRefineTol) {
      return true;
    }

    const ConnectionGeometry yp = eval(yaw_m + kStep, pitch_m, d_yaw_start, d_yaw_end);
    const ConnectionGeometry ym = eval(yaw_m - kStep, pitch_m, d_yaw_start, d_yaw_end);
    const ConnectionGeometry pp = eval(yaw_m, pitch_m + kStep, d_yaw_start, d_yaw_end);
    const ConnectionGeometry pm = eval(yaw_m, pitch_m - kStep, d_yaw_start, d_yaw_end);
    if (!yp.valid || !ym.valid || !pp.valid || !pm.valid) {
      break;
    }

    Eigen::Matrix2d jacobian;
    jacobian.col(0) = (yp.perp_coords - ym.perp_coords) / (2.0 * kStep);
    jacobian.col(1) = (pp.perp_coords - pm.perp_coords) / (2.0 * kStep);
    const Eigen::Matrix2d jtj = jacobian.transpose() * jacobian + 1.0e-6 * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d step = jtj.ldlt().solve(-jacobian.transpose() * current.perp_coords);
    if (!step.allFinite() || step.norm() < 1.0e-8) {
      break;
    }

    const Eigen::Vector2d limited = step.cwiseMax(-0.5).cwiseMin(0.5);
    double next_yaw = wrapToPiImpl(yaw_m + limited(0));
    double next_pitch = std::clamp(pitch_m + limited(1), -kPitchLimit, kPitchLimit);
    double next_dy0 = d_yaw_start;
    double next_dy1 = d_yaw_end;
    attachYawFamily(start.yaw, goal.yaw, next_yaw, next_dy0, next_dy1);
    ConnectionGeometry next = eval(next_yaw, next_pitch, next_dy0, next_dy1);

    if (!next.valid || next.perp_error >= current.perp_error) {
      next_yaw = wrapToPiImpl(yaw_m + 0.25 * limited(0));
      next_pitch = std::clamp(pitch_m + 0.25 * limited(1), -kPitchLimit, kPitchLimit);
      next_dy0 = d_yaw_start;
      next_dy1 = d_yaw_end;
      attachYawFamily(start.yaw, goal.yaw, next_yaw, next_dy0, next_dy1);
      next = eval(next_yaw, next_pitch, next_dy0, next_dy1);
      if (!next.valid || next.perp_error >= current.perp_error) {
        break;
      }
    }

    yaw_m = next_yaw;
    pitch_m = next_pitch;
    d_yaw_start = next_dy0;
    d_yaw_end = next_dy1;
    current = next;
  }
  return current.valid;
}

struct SearchHit
{
  double yaw_m = 0.0;
  double pitch_m = 0.0;
  double gear_start = 1.0;
  double gear_end = 1.0;
  HeadingPrimitive start_prim = HeadingPrimitive::kParallel;
  HeadingPrimitive end_prim = HeadingPrimitive::kParallel;
  double d_yaw_start = 0.0;
  double d_yaw_end = 0.0;
  double perp_error = std::numeric_limits<double>::infinity();
};

void evaluateConnection(
  Candidate & best, const Pose3d & start, const Pose3d & goal, double yaw_m, double pitch_m,
  double gear_start, double gear_end, HeadingPrimitive start_prim, HeadingPrimitive end_prim,
  double d_yaw_start, double d_yaw_end, double rho_yaw, double rho_pitch, bool refine)
{
  if (refine) {
    refineMidHeading(
      start, goal, yaw_m, pitch_m, gear_start, gear_end, start_prim, end_prim, d_yaw_start,
      d_yaw_end, rho_yaw, rho_pitch);
  }

  const HeadingConnect link = connectThroughMid(
    start, goal, yaw_m, pitch_m, gear_start, gear_end, start_prim, end_prim, d_yaw_start, d_yaw_end,
    rho_yaw, rho_pitch);
  if (!link.valid) {
    return;
  }
  consider(best, start, link.start_curve, link.end_curve, goal);
}

void rememberHit(std::vector<SearchHit> & hits, const SearchHit & hit, std::size_t max_hits)
{
  if (!std::isfinite(hit.perp_error)) {
    return;
  }
  hits.push_back(hit);
  std::sort(hits.begin(), hits.end(), [](const SearchHit & a, const SearchHit & b) {
    return a.perp_error < b.perp_error;
  });
  if (hits.size() > max_hits) {
    hits.resize(max_hits);
  }
}

std::vector<std::pair<double, double>> midHeadingSeeds(const Pose3d & start, const Pose3d & goal)
{
  std::vector<std::pair<double, double>> seeds;
  const Eigen::Vector3d delta = goal.position - start.position;
  const double range = delta.norm();
  double los_yaw = start.yaw;
  double los_pitch = start.pitch;
  if (range > 1.0e-6) {
    const Eigen::Vector3d dir = delta / range;
    const double sxy = std::hypot(dir.x(), dir.y());
    los_yaw = std::atan2(dir.y(), dir.x());
    los_pitch = std::atan2(-dir.z(), std::max(sxy, 1.0e-9));
    los_pitch = std::clamp(los_pitch, -kPitchLimit, kPitchLimit);
  }

  const double yaws[] = {
    start.yaw,
    goal.yaw,
    los_yaw,
    start.yaw + 0.25 * kPi,
    start.yaw - 0.25 * kPi,
    start.yaw + 0.5 * kPi,
    start.yaw - 0.5 * kPi,
    start.yaw + 0.75 * kPi,
    start.yaw - 0.75 * kPi,
    goal.yaw + 0.25 * kPi,
    goal.yaw - 0.25 * kPi,
    goal.yaw + 0.5 * kPi,
    goal.yaw - 0.5 * kPi,
    goal.yaw + 0.75 * kPi,
    goal.yaw - 0.75 * kPi,
    los_yaw + 0.25 * kPi,
    los_yaw - 0.25 * kPi,
    los_yaw + 0.5 * kPi,
    los_yaw - 0.5 * kPi,
    los_yaw + 0.75 * kPi,
    los_yaw - 0.75 * kPi,
    0.5 * (start.yaw + goal.yaw),
  };
  const double pitches[] = {
    start.pitch,
    goal.pitch,
    los_pitch,
    0.0,
    0.5 * (start.pitch + goal.pitch),
    std::clamp(start.pitch + 0.25, -kPitchLimit, kPitchLimit),
    std::clamp(start.pitch - 0.25, -kPitchLimit, kPitchLimit),
    std::clamp(goal.pitch + 0.25, -kPitchLimit, kPitchLimit),
    std::clamp(goal.pitch - 0.25, -kPitchLimit, kPitchLimit),
  };

  seeds.reserve(80);
  for (double yaw : yaws) {
    for (double pitch : pitches) {
      seeds.emplace_back(wrapToPiImpl(yaw), std::clamp(pitch, -kPitchLimit, kPitchLimit));
    }
  }

  for (int iy = 0; iy < 8; ++iy) {
    const double yaw = wrapToPiImpl(-kPi + (iy + 0.5) * (kTwoPi / 8.0));
    for (int ip = 0; ip < 5; ++ip) {
      const double pitch = -0.8 + ip * 0.4;
      seeds.emplace_back(yaw, std::clamp(pitch, -kPitchLimit, kPitchLimit));
    }
  }
  return seeds;
}

}  // namespace

SpatialReedsShepp::SpatialReedsShepp(double yaw_radius, double pitch_radius)
: yaw_radius_(std::max(yaw_radius, 1.0e-6)), pitch_radius_(std::max(pitch_radius, 1.0e-6))
{
}

Eigen::Vector3d SpatialReedsShepp::heading(double yaw, double pitch)
{
  return headingImpl(yaw, pitch);
}

double SpatialReedsShepp::wrapToPi(double angle) { return wrapToPiImpl(angle); }

Path3d SpatialReedsShepp::plan(const Pose3d & start, const Pose3d & goal) const
{
  Pose3d from = start;
  Pose3d to = goal;
  from.yaw = wrapToPiImpl(from.yaw);
  to.yaw = wrapToPiImpl(to.yaw);
  from.pitch = std::clamp(from.pitch, -kPitchLimit, kPitchLimit);
  to.pitch = std::clamp(to.pitch, -kPitchLimit, kPitchLimit);

  Candidate best;
  const auto seeds = midHeadingSeeds(from, to);
  const HeadingPrimitive primitives[] = {
    HeadingPrimitive::kParallel,
    HeadingPrimitive::kYawThenPitch,
    HeadingPrimitive::kPitchThenYaw,
  };
  const double gears[] = {1.0, -1.0};
  std::vector<SearchHit> hits;

  for (const auto & seed : seeds) {
    for (double gear_start : gears) {
      for (double gear_end : gears) {
        for (HeadingPrimitive start_prim : primitives) {
          for (HeadingPrimitive end_prim : primitives) {
            const double d_yaw_start = shortestYawDelta(from.yaw, seed.first);
            const double d_yaw_end = shortestYawDelta(seed.first, to.yaw);
            const ConnectionGeometry geom = connectionGeometry(
              from, to, seed.first, seed.second, gear_start, gear_end, start_prim, end_prim,
              d_yaw_start, d_yaw_end, yaw_radius_, pitch_radius_);
            if (!geom.valid) {
              continue;
            }
            SearchHit hit;
            hit.yaw_m = seed.first;
            hit.pitch_m = seed.second;
            hit.gear_start = gear_start;
            hit.gear_end = gear_end;
            hit.start_prim = start_prim;
            hit.end_prim = end_prim;
            hit.d_yaw_start = d_yaw_start;
            hit.d_yaw_end = d_yaw_end;
            hit.perp_error = geom.perp_error;
            rememberHit(hits, hit, 24);
            evaluateConnection(
              best, from, to, seed.first, seed.second, gear_start, gear_end, start_prim, end_prim,
              d_yaw_start, d_yaw_end, yaw_radius_, pitch_radius_, false);
          }
        }
      }
    }
  }

  if (best.perp_error > kRefineTol) {
    for (SearchHit hit : hits) {
      evaluateConnection(
        best, from, to, hit.yaw_m, hit.pitch_m, hit.gear_start, hit.gear_end, hit.start_prim,
        hit.end_prim, hit.d_yaw_start, hit.d_yaw_end, yaw_radius_, pitch_radius_, true);
      if (best.perp_error <= kRefineTol) {
        break;
      }
    }
  }

  if (best.perp_error > kPerpTol) {
    for (SearchHit hit : hits) {
      for (double d_yaw_start : yawDeltaCandidates(from.yaw, hit.yaw_m)) {
        for (double d_yaw_end : yawDeltaCandidates(hit.yaw_m, to.yaw)) {
          evaluateConnection(
            best, from, to, hit.yaw_m, hit.pitch_m, hit.gear_start, hit.gear_end, hit.start_prim,
            hit.end_prim, d_yaw_start, d_yaw_end, yaw_radius_, pitch_radius_, true);
          if (best.perp_error <= kRefineTol && best.length < 1.0e9) {
            break;
          }
        }
        if (best.perp_error <= kRefineTol) {
          break;
        }
      }
      if (best.perp_error <= kRefineTol) {
        break;
      }
    }
  }

  Path3d path;
  path.segments = std::move(best.segments);
  path.length = best.length;
  path.position_error = std::isfinite(best.perp_error) ? best.perp_error
                                                       : std::numeric_limits<double>::infinity();
  if (path.position_error > 10.0 * kPerpTol && path.segments.empty()) {
    return {};
  }
  return path;
}

Pose3d SpatialReedsShepp::sample(const Pose3d & start, const Path3d & path, double s) const
{
  if (s <= 0.0 || path.segments.empty()) {
    Pose3d pose = start;
    pose.yaw = wrapToPiImpl(pose.yaw);
    return pose;
  }

  Pose3d pose = start;
  pose.yaw = wrapToPiImpl(pose.yaw);
  double remaining = s;
  for (const Path3dSegment & segment : path.segments) {
    if (remaining <= segment.length) {
      Path3dSegment part = segment;
      part.length = remaining;
      return integrateSegment(pose, part);
    }
    pose = integrateSegment(pose, segment);
    remaining -= segment.length;
  }
  return pose;
}

std::vector<Pose3d> SpatialReedsShepp::discretize(
  const Pose3d & start, const Path3d & path, double ds) const
{
  std::vector<Pose3d> samples;
  samples.push_back(start);
  if (path.empty() || ds <= 0.0) {
    return samples;
  }

  const double total = path.length > 0.0 ? path.length : segmentsLength(path.segments);
  for (double s = ds; s < total; s += ds) {
    samples.push_back(sample(start, path, s));
  }
  samples.push_back(sample(start, path, total));
  return samples;
}

}  // namespace shortest_curve_path

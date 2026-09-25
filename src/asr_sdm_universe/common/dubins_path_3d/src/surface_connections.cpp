// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include "dubins_path_3d/surface_connections.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "dubins_path_3d/cylinder_dubins.hpp"
#include "dubins_path_3d/math_utils.hpp"
#include "dubins_path_3d/planar_dubins.hpp"
#include "dubins_path_3d/sphere_dubins.hpp"
#include "parallel.hpp"

namespace dubins_path_3d
{

namespace
{

constexpr double kTwoPi = 2.0 * M_PI;

/// How the surface normal relates to the radial direction on a given surface.
///
/// On a pitch (inner/outer) sphere the surface normal is radial, up to a sign;
/// on a yaw (left/right) sphere it is the tangent-normal that is radial. The two
/// coefficients below capture both cases so the frame can be reconstructed with
/// one expression.
struct NormalConvention
{
  double pitch_sign{0.0};  ///< non-zero for inner/outer spheres
  double yaw_sign{0.0};    ///< non-zero for left/right spheres

  NormalConvention flipped() const {return NormalConvention{-pitch_sign, -yaw_sign};}
};

NormalConvention conventionFor(SphereChoice choice)
{
  // The sign for inner/outer is opposite to the paper's; the reconstruction
  // below compensates, matching the reference implementation.
  switch (choice) {
    case SphereChoice::Outer:
      return {1.0, 0.0};
    case SphereChoice::Inner:
      return {-1.0, 0.0};
    case SphereChoice::Left:
      return {0.0, 1.0};
    case SphereChoice::Right:
      return {0.0, -1.0};
  }
  return {0.0, 0.0};
}

/// Radius of the sphere a construction turns on.
double sphereRadiusFor(SphereChoice choice, const PlannerOptions & options)
{
  return (choice == SphereChoice::Inner || choice == SphereChoice::Outer) ?
         options.pitch_radius : options.yaw_radius;
}

/// Turning radius available on the flat or developable connecting surface: the
/// rate that is *not* saturated on the spheres.
double bridgeTurnRadiusFor(SphereChoice choice, const PlannerOptions & options)
{
  return (choice == SphereChoice::Inner || choice == SphereChoice::Outer) ?
         options.yaw_radius : options.pitch_radius;
}

/// Completes a body frame from a position on a sphere and a tangent.
PathSample sampleOnSphere(
  const Vec3 & radial, const Vec3 & tangent, double sphere_radius,
  const Vec3 & centre, const NormalConvention & convention)
{
  PathSample sample;
  sample.position = radial + centre;
  sample.tangent = tangent;

  if (convention.pitch_sign != 0.0) {
    sample.surface_normal = convention.pitch_sign * radial / sphere_radius;
    sample.tangent_normal = sample.surface_normal.cross(sample.tangent);
  } else {
    sample.tangent_normal = -convention.yaw_sign * radial / sphere_radius;
    sample.surface_normal = sample.tangent.cross(sample.tangent_normal);
  }
  return sample;
}

/// Appends a sample, dropping the duplicate that appears where two surfaces
/// meet: each piece of the path carries both of its own end points.
void appendSample(const PathSample & sample, std::vector<PathSample> & out)
{
  constexpr double kJunctionTolerance = 1e-9;
  if (!out.empty() && (out.back().position - sample.position).norm() < kJunctionTolerance) {
    return;
  }
  out.push_back(sample);
}

void appendSphereSamples(
  const sphere::Samples & samples, double sphere_radius, const Vec3 & centre,
  const NormalConvention & convention, std::vector<PathSample> & out)
{
  out.reserve(out.size() + samples.positions.size());
  for (std::size_t i = 0; i < samples.positions.size(); ++i) {
    appendSample(
      sampleOnSphere(
        samples.positions[i], samples.tangents[i], sphere_radius, centre, convention),
      out);
  }
}

/// Orthonormal frame of the bridge: columns are (x, y, axis).
Mat3 bridgeFrame(const Vec3 & axis, const Vec3 & x_axis)
{
  Mat3 frame;
  frame.col(0) = x_axis;
  frame.col(1) = axis.cross(x_axis);
  frame.col(2) = axis;
  return frame;
}

/// Entry/exit configuration on the cylinder profile, in the bridge frame.
void cylinderBodyConfig(
  double theta, double phi, double radius, double height, Vec3 & position, Vec3 & tangent)
{
  position = Vec3(radius * std::cos(theta), radius * std::sin(theta), height);
  tangent = Vec3(
    -std::sin(theta) * std::cos(phi), std::cos(theta) * std::cos(phi), std::sin(phi));
}

}  // namespace

// ---------------------------------------------------------------------------
// Sphere - cylindrical envelope - sphere
// ---------------------------------------------------------------------------

SurfaceResult sphereCylinderSphere(
  const Configuration & start, const Configuration & goal, const SphereBridge & bridge,
  SphereChoice choice, const PlannerOptions & options)
{
  SurfaceResult result;
  if (!(bridge.separation > 0.0)) {
    return result;
  }

  const double sphere_radius = sphereRadiusFor(choice, options);
  const double bridge_radius = bridgeTurnRadiusFor(choice, options);
  const double turn_radius = options.resolvedTightTurnRadius();
  const NormalConvention convention = conventionFor(choice);

  const int location_count = options.location_samples;
  const int heading_count = options.heading_samples;

  const Vec3 x_axis = orthogonalUnitVector(bridge.axis, start.rotation());
  const Mat3 frame = bridgeFrame(bridge.axis, x_axis);

  const sphere::Config start_on_sphere =
    sphere::makeConfig(start.position, bridge.initial_centre, start.tangent);
  const sphere::Config goal_on_sphere =
    sphere::makeConfig(goal.position, bridge.final_centre, goal.tangent);

  // Cost of reaching the cylinder entry, indexed [theta_in][phi_in].
  std::vector<double> entry_cost(
    static_cast<std::size_t>(location_count) * heading_count, kInfinity);
  // Cost of leaving the cylinder exit, indexed [theta_out][phi_out].
  std::vector<double> exit_cost(
    static_cast<std::size_t>(location_count) * heading_count, kInfinity);
  // Cost on the cylinder, indexed [phi_in][delta_theta][phi_out]. The cylinder
  // path only depends on the difference between the exit and entry angles.
  std::vector<double> bridge_cost(
    static_cast<std::size_t>(heading_count) * location_count * heading_count, kInfinity);

  parallelFor(
    entry_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int i = static_cast<int>(flat) / heading_count;
      const int j = static_cast<int>(flat) % heading_count;
      const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, i);
      const double phi = linspaceValue(0.0, M_PI, heading_count, j);

      Vec3 body_position;
      Vec3 body_tangent;
      cylinderBodyConfig(theta, phi, sphere_radius, 0.0, body_position, body_tangent);
      const Vec3 position = frame * body_position + bridge.initial_centre;
      const Vec3 tangent = frame * body_tangent;

      entry_cost[flat] = sphere::optimalPathLength(
        start_on_sphere,
        sphere::makeConfig(position, bridge.initial_centre, tangent),
        turn_radius, sphere_radius, options.sphere_tolerance);
    });

  parallelFor(
    exit_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int k = static_cast<int>(flat) / heading_count;
      const int l = static_cast<int>(flat) % heading_count;
      const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, k);
      const double phi = linspaceValue(0.0, M_PI, heading_count, l);

      Vec3 body_position;
      Vec3 body_tangent;
      cylinderBodyConfig(
        theta, phi, sphere_radius, bridge.separation, body_position, body_tangent);
      const Vec3 position = frame * body_position + bridge.initial_centre;
      const Vec3 tangent = frame * body_tangent;

      exit_cost[flat] = sphere::optimalPathLength(
        sphere::makeConfig(position, bridge.final_centre, tangent),
        goal_on_sphere, turn_radius, sphere_radius, options.sphere_tolerance);
    });

  parallelFor(
    bridge_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int j = static_cast<int>(flat) / (location_count * heading_count);
      const int remainder = static_cast<int>(flat) % (location_count * heading_count);
      const int m = remainder / heading_count;
      const int l = remainder % heading_count;

      const double phi_in = linspaceValue(0.0, M_PI, heading_count, j);
      const double phi_out = linspaceValue(0.0, M_PI, heading_count, l);
      const double theta_out = linspaceValueOpen(0.0, kTwoPi, location_count, m);

      Vec3 entry_position;
      Vec3 entry_tangent;
      Vec3 exit_position;
      Vec3 exit_tangent;
      cylinderBodyConfig(0.0, phi_in, sphere_radius, 0.0, entry_position, entry_tangent);
      cylinderBodyConfig(
        theta_out, phi_out, sphere_radius, bridge.separation, exit_position, exit_tangent);

      bridge_cost[flat] = cylinder::optimalPathLength(
        entry_position, entry_tangent, exit_position, exit_tangent, sphere_radius,
        bridge_radius);
    });

  int best_i = -1;
  int best_j = -1;
  int best_k = -1;
  int best_l = -1;
  for (int i = 0; i < location_count; ++i) {
    for (int j = 0; j < heading_count; ++j) {
      const double entry = entry_cost[static_cast<std::size_t>(i) * heading_count + j];
      if (!std::isfinite(entry)) {
        continue;
      }
      for (int k = 0; k < location_count; ++k) {
        const int delta = ((k - i) % location_count + location_count) % location_count;
        for (int l = 0; l < heading_count; ++l) {
          const double exit = exit_cost[static_cast<std::size_t>(k) * heading_count + l];
          const double middle = bridge_cost[
            (static_cast<std::size_t>(j) * location_count + delta) * heading_count + l];
          const double total = entry + middle + exit;
          if (total < result.length) {
            result.length = total;
            best_i = i;
            best_j = j;
            best_k = k;
            best_l = l;
          }
        }
      }
    }
  }

  if (best_i < 0) {
    result.length = kInfinity;
    return result;
  }

  // Rebuild the geometry of the winning parameter set.
  const double theta_in = linspaceValueOpen(0.0, kTwoPi, location_count, best_i);
  const double phi_in = linspaceValue(0.0, M_PI, heading_count, best_j);
  const double theta_out = linspaceValueOpen(0.0, kTwoPi, location_count, best_k);
  const double phi_out = linspaceValue(0.0, M_PI, heading_count, best_l);

  Vec3 entry_body_position;
  Vec3 entry_body_tangent;
  Vec3 exit_body_position;
  Vec3 exit_body_tangent;
  cylinderBodyConfig(
    theta_in, phi_in, sphere_radius, 0.0, entry_body_position, entry_body_tangent);
  cylinderBodyConfig(
    theta_out, phi_out, sphere_radius, bridge.separation, exit_body_position,
    exit_body_tangent);

  const Vec3 entry_position = frame * entry_body_position + bridge.initial_centre;
  const Vec3 entry_tangent = frame * entry_body_tangent;
  const Vec3 exit_position = frame * exit_body_position + bridge.initial_centre;
  const Vec3 exit_tangent = frame * exit_body_tangent;

  const sphere::Solution first = sphere::optimalPath(
    start_on_sphere, sphere::makeConfig(entry_position, bridge.initial_centre, entry_tangent),
    turn_radius, sphere_radius, options.sphere_tolerance);
  const sphere::Solution last = sphere::optimalPath(
    sphere::makeConfig(exit_position, bridge.final_centre, exit_tangent), goal_on_sphere,
    turn_radius, sphere_radius, options.sphere_tolerance);
  const cylinder::Result middle = cylinder::optimalPath(
    entry_body_position, entry_body_tangent, exit_body_position, exit_body_tangent,
    sphere_radius, bridge_radius, options.sample_spacing);

  if (!first.valid() || !last.valid() || !middle.valid()) {
    result.length = kInfinity;
    return result;
  }

  appendSphereSamples(
    sphere::pathSamples(
      start_on_sphere, turn_radius, sphere_radius, first.angles, first.type,
      options.sample_spacing),
    sphere_radius, bridge.initial_centre, convention, result.samples);

  for (std::size_t i = 0; i < middle.positions.size(); ++i) {
    PathSample sample;
    sample.position = frame * middle.positions[i] + bridge.initial_centre;
    sample.tangent = frame * middle.tangents[i];
    const Vec3 normal = frame * middle.normals[i];
    if (convention.pitch_sign != 0.0) {
      sample.surface_normal = convention.pitch_sign * normal;
      sample.tangent_normal = sample.surface_normal.cross(sample.tangent);
    } else {
      sample.tangent_normal = -convention.yaw_sign * normal;
      sample.surface_normal = sample.tangent.cross(sample.tangent_normal);
    }
    appendSample(sample, result.samples);
  }

  appendSphereSamples(
    sphere::pathSamples(
      sphere::makeConfig(exit_position, bridge.final_centre, exit_tangent), turn_radius,
      sphere_radius, last.angles, last.type, options.sample_spacing),
    sphere_radius, bridge.final_centre, convention, result.samples);

  return result;
}

// ---------------------------------------------------------------------------
// Sphere - cross-tangent plane - sphere
// ---------------------------------------------------------------------------

SurfaceResult spherePlaneSphere(
  const Configuration & start, const Configuration & goal, const SphereBridge & bridge,
  SphereChoice choice, const PlannerOptions & options)
{
  SurfaceResult result;

  const double sphere_radius = sphereRadiusFor(choice, options);
  const double plane_radius = bridgeTurnRadiusFor(choice, options);
  const double turn_radius = options.resolvedTightTurnRadius();
  const NormalConvention convention = conventionFor(choice);

  // A plane can only be tangent to both spheres when they are far enough apart.
  if (2.0 * sphere_radius > bridge.separation) {
    return result;
  }

  const int location_count = options.location_samples;
  const int heading_count = options.heading_samples;

  const Vec3 x_axis = orthogonalUnitVector(bridge.axis, start.rotation());
  const Vec3 y_axis = bridge.axis.cross(x_axis);
  const double alpha = std::acos(2.0 * sphere_radius / bridge.separation);
  const double plane_span =
    std::sqrt(bridge.separation * bridge.separation - 4.0 * sphere_radius * sphere_radius);

  const sphere::Config start_on_sphere =
    sphere::makeConfig(start.position, bridge.initial_centre, start.tangent);
  const sphere::Config goal_on_sphere =
    sphere::makeConfig(goal.position, bridge.final_centre, goal.tangent);

  // Tangency points and the frame of the tangent plane selected by `theta`.
  const auto tangencyFrame = [&](double theta, Vec3 & entry, Vec3 & exit, Vec3 & along,
    Vec3 & lateral) {
      entry = bridge.initial_centre + sphere_radius * std::cos(alpha) * bridge.axis +
        sphere_radius * std::sin(alpha) * (std::cos(theta) * x_axis + std::sin(theta) * y_axis);
      exit = bridge.final_centre - sphere_radius * std::cos(alpha) * bridge.axis +
        sphere_radius * std::sin(alpha) *
        (std::cos(theta + M_PI) * x_axis + std::sin(theta + M_PI) * y_axis);
      along = (exit - entry).normalized();
      lateral = ((entry - bridge.initial_centre) / sphere_radius).cross(along);
    };

  std::vector<double> entry_cost(
    static_cast<std::size_t>(location_count) * heading_count, kInfinity);
  std::vector<double> exit_cost(
    static_cast<std::size_t>(location_count) * heading_count, kInfinity);
  std::vector<double> plane_cost(
    static_cast<std::size_t>(heading_count) * heading_count, kInfinity);

  parallelFor(
    entry_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int i = static_cast<int>(flat) / heading_count;
      const int j = static_cast<int>(flat) % heading_count;
      const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, i);
      const double phi = linspaceValue(-M_PI_2, M_PI_2, heading_count, j);

      Vec3 entry;
      Vec3 exit;
      Vec3 along;
      Vec3 lateral;
      tangencyFrame(theta, entry, exit, along, lateral);
      const Vec3 tangent = std::cos(phi) * along + std::sin(phi) * lateral;

      entry_cost[flat] = sphere::optimalPathLength(
        start_on_sphere, sphere::makeConfig(entry, bridge.initial_centre, tangent),
        turn_radius, sphere_radius, options.sphere_tolerance);
    });

  parallelFor(
    exit_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int i = static_cast<int>(flat) / heading_count;
      const int j = static_cast<int>(flat) % heading_count;
      const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, i);
      const double phi = linspaceValue(-M_PI_2, M_PI_2, heading_count, j);

      Vec3 entry;
      Vec3 exit;
      Vec3 along;
      Vec3 lateral;
      tangencyFrame(theta, entry, exit, along, lateral);
      const Vec3 tangent = std::cos(phi) * along + std::sin(phi) * lateral;

      exit_cost[flat] = sphere::optimalPathLength(
        sphere::makeConfig(exit, bridge.final_centre, tangent), goal_on_sphere,
        turn_radius, sphere_radius, options.sphere_tolerance);
    });

  parallelFor(
    plane_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int j = static_cast<int>(flat) / heading_count;
      const int k = static_cast<int>(flat) % heading_count;
      const planar::Config from{0.0, 0.0, linspaceValue(-M_PI_2, M_PI_2, heading_count, j)};
      const planar::Config to{
        plane_span, 0.0, linspaceValue(-M_PI_2, M_PI_2, heading_count, k)};
      plane_cost[flat] = planar::optimalPath(from, to, plane_radius).length;
    });

  int best_i = -1;
  int best_j = -1;
  int best_k = -1;
  for (int i = 0; i < location_count; ++i) {
    for (int j = 0; j < heading_count; ++j) {
      const double entry = entry_cost[static_cast<std::size_t>(i) * heading_count + j];
      if (!std::isfinite(entry)) {
        continue;
      }
      for (int k = 0; k < heading_count; ++k) {
        const double total = entry +
          plane_cost[static_cast<std::size_t>(j) * heading_count + k] +
          exit_cost[static_cast<std::size_t>(i) * heading_count + k];
        if (total < result.length) {
          result.length = total;
          best_i = i;
          best_j = j;
          best_k = k;
        }
      }
    }
  }

  if (best_i < 0) {
    result.length = kInfinity;
    return result;
  }

  const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, best_i);
  const double phi_in = linspaceValue(-M_PI_2, M_PI_2, heading_count, best_j);
  const double phi_out = linspaceValue(-M_PI_2, M_PI_2, heading_count, best_k);

  Vec3 entry;
  Vec3 exit;
  Vec3 along;
  Vec3 lateral;
  tangencyFrame(theta, entry, exit, along, lateral);
  const Vec3 entry_tangent = std::cos(phi_in) * along + std::sin(phi_in) * lateral;
  const Vec3 exit_tangent = std::cos(phi_out) * along + std::sin(phi_out) * lateral;
  const Vec3 entry_radial = (entry - bridge.initial_centre) / sphere_radius;

  const sphere::Solution first = sphere::optimalPath(
    start_on_sphere, sphere::makeConfig(entry, bridge.initial_centre, entry_tangent),
    turn_radius, sphere_radius, options.sphere_tolerance);
  const sphere::Solution last = sphere::optimalPath(
    sphere::makeConfig(exit, bridge.final_centre, exit_tangent), goal_on_sphere,
    turn_radius, sphere_radius, options.sphere_tolerance);
  const planar::Config plane_start{0.0, 0.0, phi_in};
  const planar::Solution middle = planar::optimalPath(
    plane_start, planar::Config{plane_span, 0.0, phi_out}, plane_radius);

  if (!first.valid() || !last.valid() || !middle.valid()) {
    result.length = kInfinity;
    return result;
  }

  appendSphereSamples(
    sphere::pathSamples(
      start_on_sphere, turn_radius, sphere_radius, first.angles, first.type,
      options.sample_spacing),
    sphere_radius, bridge.initial_centre, convention, result.samples);

  Mat3 plane_frame;
  plane_frame.col(0) = along;
  plane_frame.col(1) = lateral;
  plane_frame.col(2) = entry_radial;

  for (const planar::Config & point :
    planar::pathSamples(
      plane_start, plane_radius, middle.params, middle.type, options.sample_spacing))
  {
    PathSample sample;
    sample.position = entry + plane_frame * Vec3(point.x, point.y, 0.0);
    sample.tangent = std::cos(point.heading) * along + std::sin(point.heading) * lateral;
    if (convention.pitch_sign != 0.0) {
      sample.tangent_normal = convention.pitch_sign * entry_radial.cross(sample.tangent);
      sample.surface_normal = convention.pitch_sign * entry_radial;
    } else {
      sample.tangent_normal = -convention.yaw_sign * entry_radial;
      sample.surface_normal = sample.tangent.cross(sample.tangent_normal);
    }
    appendSample(sample, result.samples);
  }

  // The path arrives on the opposite side of the sphere at the far end, so the
  // normal convention flips.
  appendSphereSamples(
    sphere::pathSamples(
      sphere::makeConfig(exit, bridge.final_centre, exit_tangent), turn_radius,
      sphere_radius, last.angles, last.type, options.sample_spacing),
    sphere_radius, bridge.final_centre, convention.flipped(), result.samples);

  return result;
}

// ---------------------------------------------------------------------------
// Sphere - intermediary sphere - sphere
// ---------------------------------------------------------------------------

SurfaceResult sphereSphereSphere(
  const Configuration & start, const Configuration & goal, const SphereBridge & bridge,
  SphereChoice choice, const PlannerOptions & options)
{
  SurfaceResult result;

  const double sphere_radius = sphereRadiusFor(choice, options);
  const double turn_radius = options.resolvedTightTurnRadius();
  const NormalConvention convention = conventionFor(choice);

  // An intermediary sphere of the same radius can only touch both spheres when
  // their centres are at most four radii apart.
  if (bridge.separation > 4.0 * sphere_radius) {
    return result;
  }

  const int location_count = options.location_samples;
  const int heading_count = options.heading_samples;

  const Vec3 x_axis = orthogonalUnitVector(bridge.axis, start.rotation());
  const Vec3 y_axis = bridge.axis.cross(x_axis);
  const double alpha = std::acos(bridge.separation / (4.0 * sphere_radius));

  const sphere::Config start_on_sphere =
    sphere::makeConfig(start.position, bridge.initial_centre, start.tangent);
  const sphere::Config goal_on_sphere =
    sphere::makeConfig(goal.position, bridge.final_centre, goal.tangent);

  /// Centre of the intermediary sphere and the two tangency points, plus the
  /// reference directions used to parameterise the tangent vectors there.
  const auto intermediary = [&](double theta, Vec3 & centre, Vec3 & entry, Vec3 & exit,
    Vec3 & entry_reference, Vec3 & exit_reference) {
      centre = bridge.initial_centre + 0.5 * (bridge.final_centre - bridge.initial_centre) +
        2.0 * sphere_radius * std::sin(alpha) *
        (std::cos(theta) * x_axis + std::sin(theta) * y_axis);
      entry = 0.5 * (bridge.initial_centre + centre);
      exit = 0.5 * (bridge.final_centre + centre);
      entry_reference = bridge.axis / std::sin(alpha) -
        (entry - bridge.initial_centre) / (sphere_radius * std::tan(alpha));
      exit_reference = -bridge.axis / std::sin(alpha) -
        (exit - bridge.final_centre) / (sphere_radius * std::tan(alpha));
    };

  const auto tangentAt = [&](const Vec3 & radial, const Vec3 & reference, double phi) {
      return std::cos(phi) * reference +
             std::sin(phi) * radial.cross(reference) / sphere_radius;
    };

  std::vector<double> entry_cost(
    static_cast<std::size_t>(location_count) * heading_count, kInfinity);
  std::vector<double> exit_cost(
    static_cast<std::size_t>(location_count) * heading_count, kInfinity);
  std::vector<double> middle_cost(
    static_cast<std::size_t>(heading_count) * heading_count, kInfinity);

  parallelFor(
    entry_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int i = static_cast<int>(flat) / heading_count;
      const int j = static_cast<int>(flat) % heading_count;
      const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, i);
      const double phi = linspaceValueOpen(0.0, kTwoPi, heading_count, j);

      Vec3 centre;
      Vec3 entry;
      Vec3 exit;
      Vec3 entry_reference;
      Vec3 exit_reference;
      intermediary(theta, centre, entry, exit, entry_reference, exit_reference);
      const Vec3 tangent =
      tangentAt(entry - bridge.initial_centre, entry_reference, phi);

      entry_cost[flat] = sphere::optimalPathLength(
        start_on_sphere, sphere::makeConfig(entry, bridge.initial_centre, tangent),
        turn_radius, sphere_radius, options.sphere_tolerance);
    });

  parallelFor(
    exit_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int i = static_cast<int>(flat) / heading_count;
      const int j = static_cast<int>(flat) % heading_count;
      const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, i);
      const double phi = linspaceValueOpen(0.0, kTwoPi, heading_count, j);

      Vec3 centre;
      Vec3 entry;
      Vec3 exit;
      Vec3 entry_reference;
      Vec3 exit_reference;
      intermediary(theta, centre, entry, exit, entry_reference, exit_reference);
      const Vec3 tangent = tangentAt(exit - bridge.final_centre, exit_reference, phi);

      exit_cost[flat] = sphere::optimalPathLength(
        sphere::makeConfig(exit, bridge.final_centre, tangent), goal_on_sphere,
        turn_radius, sphere_radius, options.sphere_tolerance);
    });

  // As the intermediary sphere rotates about the axis, the entry and exit
  // configurations rotate with it, so the path across it is the same for every
  // location parameter and only needs to be computed once.
  parallelFor(
    middle_cost.size(), options.num_threads, [&](std::size_t flat) {
      const int j = static_cast<int>(flat) / heading_count;
      const int k = static_cast<int>(flat) % heading_count;
      const double phi_in = linspaceValueOpen(0.0, kTwoPi, heading_count, j);
      const double phi_out = linspaceValueOpen(0.0, kTwoPi, heading_count, k);

      Vec3 centre;
      Vec3 entry;
      Vec3 exit;
      Vec3 entry_reference;
      Vec3 exit_reference;
      intermediary(
        linspaceValueOpen(0.0, kTwoPi, location_count, 0), centre, entry, exit,
        entry_reference, exit_reference);

      const Vec3 entry_tangent =
      tangentAt(entry - bridge.initial_centre, entry_reference, phi_in);
      const Vec3 exit_tangent =
      tangentAt(exit - bridge.final_centre, exit_reference, phi_out);

      middle_cost[flat] = sphere::optimalPathLength(
        sphere::makeConfig(entry, centre, entry_tangent),
        sphere::makeConfig(exit, centre, exit_tangent),
        turn_radius, sphere_radius, options.sphere_tolerance);
    });

  int best_i = -1;
  int best_j = -1;
  int best_k = -1;
  for (int i = 0; i < location_count; ++i) {
    for (int j = 0; j < heading_count; ++j) {
      const double entry = entry_cost[static_cast<std::size_t>(i) * heading_count + j];
      if (!std::isfinite(entry)) {
        continue;
      }
      for (int k = 0; k < heading_count; ++k) {
        const double total = entry +
          middle_cost[static_cast<std::size_t>(j) * heading_count + k] +
          exit_cost[static_cast<std::size_t>(i) * heading_count + k];
        if (total < result.length) {
          result.length = total;
          best_i = i;
          best_j = j;
          best_k = k;
        }
      }
    }
  }

  if (best_i < 0) {
    result.length = kInfinity;
    return result;
  }

  const double theta = linspaceValueOpen(0.0, kTwoPi, location_count, best_i);
  const double phi_in = linspaceValueOpen(0.0, kTwoPi, heading_count, best_j);
  const double phi_out = linspaceValueOpen(0.0, kTwoPi, heading_count, best_k);

  Vec3 centre;
  Vec3 entry;
  Vec3 exit;
  Vec3 entry_reference;
  Vec3 exit_reference;
  intermediary(theta, centre, entry, exit, entry_reference, exit_reference);
  const Vec3 entry_tangent =
    tangentAt(entry - bridge.initial_centre, entry_reference, phi_in);
  const Vec3 exit_tangent = tangentAt(exit - bridge.final_centre, exit_reference, phi_out);

  const sphere::Config entry_on_first =
    sphere::makeConfig(entry, bridge.initial_centre, entry_tangent);
  const sphere::Config entry_on_middle = sphere::makeConfig(entry, centre, entry_tangent);
  const sphere::Config exit_on_middle = sphere::makeConfig(exit, centre, exit_tangent);
  const sphere::Config exit_on_last =
    sphere::makeConfig(exit, bridge.final_centre, exit_tangent);

  const sphere::Solution first = sphere::optimalPath(
    start_on_sphere, entry_on_first, turn_radius, sphere_radius, options.sphere_tolerance);
  const sphere::Solution middle = sphere::optimalPath(
    entry_on_middle, exit_on_middle, turn_radius, sphere_radius, options.sphere_tolerance);
  const sphere::Solution last = sphere::optimalPath(
    exit_on_last, goal_on_sphere, turn_radius, sphere_radius, options.sphere_tolerance);

  if (!first.valid() || !middle.valid() || !last.valid()) {
    result.length = kInfinity;
    return result;
  }

  appendSphereSamples(
    sphere::pathSamples(
      start_on_sphere, turn_radius, sphere_radius, first.angles, first.type,
      options.sample_spacing),
    sphere_radius, bridge.initial_centre, convention, result.samples);

  // The vehicle rides the far side of the intermediary sphere.
  appendSphereSamples(
    sphere::pathSamples(
      entry_on_middle, turn_radius, sphere_radius, middle.angles, middle.type,
      options.sample_spacing),
    sphere_radius, centre, convention.flipped(), result.samples);

  appendSphereSamples(
    sphere::pathSamples(
      exit_on_last, turn_radius, sphere_radius, last.angles, last.type,
      options.sample_spacing),
    sphere_radius, bridge.final_centre, convention, result.samples);

  return result;
}

}  // namespace dubins_path_3d

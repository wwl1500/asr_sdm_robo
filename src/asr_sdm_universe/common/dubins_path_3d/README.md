# dubins_path_3d

3D Dubins path planning for a vehicle with bounded pitch and yaw rates, packaged
as a standalone `ament_cmake` library.

The planner library itself has no node and no launch files. Eigen is its only
algorithmic dependency. ROS 2 message and service definitions live in
`asr_sdm_control_msgs` (`Configuration`, `PathSample`, `DubinsPath`, `PlanPath`);
convert to and from the C++ types with `dubins_path_3d/msg_conversions.hpp`.

## The method

Four osculating spheres surround each configuration: two for the pitch rate
(nose up on the inner sphere, nose down on the outer one) and two for the yaw
rate (left and right turns). A path leaves a sphere at the start and arrives on
a sphere at the goal, and the two are joined by one of three intermediary
surfaces:

| Construction | Joining surface | `Path3D::type` |
| --- | --- | --- |
| SCS | cylinder tangent to both spheres | `cyc_<sphere>` |
| SPS | plane cross-tangent to both spheres | `plane_<near>_<far>` |
| SSS | a third sphere tangent to both | `sphere_<sphere>` |

`<sphere>` is one of `inner`, `outer`, `left` or `right`, naming the osculating
sphere the path departs on, so a returned type reads for example `cyc_inner` or
`plane_right_left`.

Each construction has free parameters (where the surface touches the spheres and
the heading at entry and exit). The planner sweeps them on a grid, evaluates
every feasible combination, and returns the shortest path. The sweep is
multi-threaded, so it is a heuristic search over a discretisation rather than a
proof of global optimality.

## Usage

Add the dependency to `package.xml`:

```xml
<depend>dubins_path_3d</depend>
```

and to `CMakeLists.txt`:

```cmake
find_package(dubins_path_3d REQUIRED)
target_link_libraries(your_target dubins_path_3d::dubins_path_3d)
# Optional: ROS message conversions
# target_link_libraries(your_target dubins_path_3d::dubins_path_3d_conversions)
```

Then plan:

```cpp
#include "dubins_path_3d/planner.hpp"

dubins_path_3d::PlannerOptions options;
options.pitch_radius = 40.0;  // radius at maximum pitch rate, zero yaw rate
options.yaw_radius = 50.0;    // radius at maximum yaw rate, zero pitch rate

const dubins_path_3d::DubinsPath3D planner(options);

const auto start = dubins_path_3d::configurationFromEuler(
  dubins_path_3d::Vec3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
const auto goal = dubins_path_3d::configurationFromEuler(
  dubins_path_3d::Vec3(100.0, 50.0, 40.0), M_PI_2, M_PI_4, 0.0);

const auto result = planner.plan(start, goal);
if (result.success()) {
  for (const auto & sample : result.best.samples) {
    // sample.position, and the body frame as sample.tangent,
    // sample.tangent_normal, sample.surface_normal
  }
}
```

`plan` throws `std::invalid_argument` on degenerate input frames, and the
constructor throws on inconsistent options. A failure to find a path is not an
exception: check `result.success()`.

Instances are stateless apart from the options, so one planner may be shared
between threads.

## Public headers

| Header | Contents |
| --- | --- |
| `planner.hpp` | `DubinsPath3D`, the entry point |
| `types.hpp` | `Configuration`, `PathSample`, `Path3D`, `PlannerOptions`, `PlanningResult`, Euler conversions |
| `msg_conversions.hpp` | conversions to `asr_sdm_control_msgs` (`Configuration`, `PathSample`, `DubinsPath`, `PlanPath`) |
| `surface_connections.hpp` | the SCS / SPS / SSS constructions, to run one in isolation |
| `planar_dubins.hpp` | planar Dubins paths (CSC and CCC families) |
| `sphere_dubins.hpp` | Dubins paths on a sphere (3 to 5 segments) |
| `cylinder_dubins.hpp` | Dubins paths on a cylinder |
| `math_utils.hpp` | angle wrapping, cubic solver, and other shared helpers |

Everything lives in namespace `dubins_path_3d`, with the surface-specific
algorithms in the nested `planar`, `sphere` and `cylinder` namespaces.

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `pitch_radius` | 40.0 | radius at maximum pitch rate, zero yaw rate |
| `yaw_radius` | 50.0 | radius at maximum yaw rate, zero pitch rate |
| `tight_turn_radius` | -1.0 | tight-turn radius on the spheres; non-positive derives it from the two radii |
| `location_samples` | 15 | grid resolution of the surface location parameter |
| `heading_samples` | 15 | grid resolution of the entry and exit headings |
| `sample_spacing` | 2.0 | target spacing in metres between output samples |
| `num_threads` | 0 | sweep worker threads; zero uses the hardware concurrency |
| `sphere_tolerance` | 1e-4 | tolerance when verifying a spherical inverse-kinematics solution |

Raising `location_samples` and `heading_samples` improves the path at a cost
that grows with their product.

## Build and test

```bash
colcon build --packages-up-to dubins_path_3d
colcon test --packages-select dubins_path_3d
```

## Reference

Kumar, Darbha, Rathinam, Casbeer, Manyam and Weintraub, *Motion Planning for a
Generalized Dubins Vehicle with Pitch and Yaw Rate Constraints*. This is a C++
port of the authors' Python implementation.

One deliberate difference: in the degenerate single-turn branch of the
three-segment spherical solver, the reference code validates the candidate with
the unit radius instead of the tight-turn radius. This port uses the tight-turn
radius, which matters whenever the sphere radius is not 1.

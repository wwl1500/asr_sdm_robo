# asr_sdm_guidance_planner

Front-end guidance path search library. It holds the Fast-Planner style grid-based A* search
that used to live inside `asr_sdm_local_path_modifier`, plus a 3D Dubins curve search used
through `GuidancePlanner`:

```text
EDTEnvironment + start + goal
  -> grid A* expansion on the ESDF map
    -> geometric guide path (std::vector<Eigen::Vector3d>)

EDTEnvironment + start / goal pose
  -> dubins_path_3d::DubinsPath3D
  -> sampled curve (position, yaw, pitch)
```

The package is a pure library; it has no node, launch file, or runtime parameter file of its
own. Search parameters are read from the ROS node that is passed to `Astar::setParam` or
`GuidancePlanner::setParam`, so the owning node (currently `planning_manager_node`) declares
and supplies them.

## Package structure

```text
asr_sdm_guidance_planner/
├── include/asr_sdm_guidance_planner/astar.h
├── include/asr_sdm_guidance_planner/guidance_planner.h
├── include/asr_sdm_guidance_planner/kinodynamic_astar.h
├── src/astar.cpp
└── src/guidance_planner.cpp
```

`kinodynamic_astar.h` / `src/kinodynamic_astar.cpp` are legacy Fast-Planner sources that were
already excluded from the build before the split. They are kept here as reference for a future
kinodynamic front end and are intentionally not compiled.

## Public include

```cpp
#include <asr_sdm_guidance_planner/astar.h>
#include <asr_sdm_guidance_planner/guidance_planner.h>
```

## Library usage

```cpp
#include <asr_sdm_guidance_planner/astar.h>

std::unique_ptr<amprobo::Astar> geo_path_finder(new amprobo::Astar);
geo_path_finder->setParam(node);          // reads the astar.* parameters from the node
geo_path_finder->setEnvironment(edt_environment);
geo_path_finder->init();

geo_path_finder->reset();
if (geo_path_finder->search(start_pt, end_pt) == amprobo::Astar::REACH_END) {
  std::vector<Eigen::Vector3d> path = geo_path_finder->getPath();
}

std::unique_ptr<amprobo::GuidancePlanner> curve_path_finder(new amprobo::GuidancePlanner);
curve_path_finder->setParam(node);
curve_path_finder->setEnvironment(edt_environment);
curve_path_finder->init();

curve_path_finder->reset();
if (curve_path_finder->search(start_pt, start_yaw, start_pitch, end_pt, end_yaw, end_pitch) ==
    amprobo::GuidancePlanner::REACH_END) {
  std::vector<Eigen::Vector3d> path = curve_path_finder->getPath();
}
```

## Parameters

`Astar::setParam` declares and reads the following parameters on the node it is given:

| Parameter | Meaning |
| --- | --- |
| `astar.resolution_astar` | Grid step used for node expansion and index mapping |
| `astar.time_resolution` | Time discretization used by the dynamic search variant |
| `astar.lambda_heu` | Heuristic weight applied to the Euclidean heuristic |
| `astar.margin` | Minimum ESDF clearance a node must keep to be expanded |
| `astar.allocate_num` | Size of the pre-allocated node pool |

`GuidancePlanner::setParam` declares and reads:

| Parameter | Meaning |
| --- | --- |
| `guidance_planner.dubins_path_3d.yaw_radius` | Radius at maximum yaw rate, zero pitch rate |
| `guidance_planner.dubins_path_3d.pitch_radius` | Radius at maximum pitch rate, zero yaw rate |
| `guidance_planner.dubins_path_3d.sample_ds` | Target spacing used to sample the curve |
| `guidance_planner.dubins_path_3d.margin` | Minimum ESDF clearance along sampled points |
| `guidance_planner.dubins_path_3d.position_tol` | Maximum leftover position error accepted as feasible |
| `guidance_planner.dubins_path_3d.tight_turn_radius` | Tight-turn radius on the spheres; non-positive derives it from the two radii |
| `guidance_planner.dubins_path_3d.location_samples` | Grid resolution of the intermediary-surface location parameter |
| `guidance_planner.dubins_path_3d.heading_samples` | Grid resolution of the entry and exit headings |
| `guidance_planner.dubins_path_3d.num_threads` | Sweep worker threads; zero uses the hardware concurrency |

## Build

```bash
colcon build --symlink-install --packages-up-to asr_sdm_guidance_planner
```

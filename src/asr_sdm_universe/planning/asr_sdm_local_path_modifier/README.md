# asr_sdm_local_path_modifier

This package provides a C++ local path modifier library plus an RViz test executable.
The library-level responsibility is only:

```text
input path + collision checker
  -> Fast-Planner TopologyPRM-style local candidate paths
  -> selected modified path
```

The RViz test node source code is kept under `test/`; runtime assets follow the usual ROS package layout at package root: `config/`, `launch/`, and `rviz/`. It subscribes to `/planning/waypoints`, accepts temporary spherical obstacles from RViz `Publish Point` on `/planning/add_virtual_obstacle`, converts those test obstacles into the library-level `CollisionChecker`, and publishes `/planning/topo_candidate_paths`.

## Package structure

```text
asr_sdm_local_path_modifier/
├── include/topo_path_modifier.hpp
├── src/topo_path_modifier.cpp
├── test/local_path_modifier_test.cpp
├── config/local_path_modifier.yaml
├── launch/local_path_modifier_test.launch.py
└── rviz/local_path_modifier_test.rviz
```

## Public include

The public header is installed directly under `include/`:

```cpp
#include <topo_path_modifier.hpp>
```

The modifier header is intentionally flat under `include/`.

## Runtime logic

1. Subscribe to the latest `/planning/waypoints` with `reliable + transient_local` QoS, so the modifier can receive the latest guidance result even if it starts after the guidance planner.
2. In the test node only, add temporary spherical obstacles from RViz `Publish Point` on `/planning/add_virtual_obstacle`.
3. The core library checks whether the input path collides with the supplied `CollisionChecker`.
4. If the input path is collision-free, the original path is returned unchanged.
5. If the input path collides, only the blocked local segment is modified. The modifier builds a Fast-Planner `TopologyPRM`-style guard/connector roadmap, searches raw paths, shortcuts them, prunes topologically equivalent paths, applies `selectShortPaths`, and splices the selected local detour back into the original waypoints.

Non-uniform B-spline optimization is intentionally not included yet. The current scope is the topo path logic only. The grid A* front-end search that used to live here now has its own package, [`asr_sdm_guidance_planner`](../asr_sdm_guidance_planner/).

## Launch

Usually start guidance first so it opens the shared RViz page:

```bash
ros2 launch asr_sdm_guidance_planner_dev astar_lbfgs_planner.launch.py
```

Then start this modifier without opening a second RViz window:

```bash
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py
```

For standalone modifier visualization:

```bash
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py use_rviz:=true
```

Clear all temporary obstacles:

```bash
ros2 topic pub --once /planning/clear_virtual_obstacles std_msgs/msg/Empty "{}"
```

## Library usage

Manager-side code should call the collision-checker overload instead of depending on RViz virtual obstacles:

```cpp
#include <topo_path_modifier.hpp>

amprobo::CollisionChecker checker;
checker.pointInCollision = ...;
checker.segmentInCollision = ...;
checker.clearance = ...;

amprobo::TopoModifierResult result =
  modifier.modify(input_path, checker);
```


## Current topo candidate output

The C++ library returns `TopoModifierResult::candidate_paths`, a vector of complete selected topo candidate paths. The RViz test node does not publish a single `/planning/modified_path`; it publishes all selected candidates as yellow `visualization_msgs/msg/MarkerArray` on `/planning/topo_candidate_paths`.

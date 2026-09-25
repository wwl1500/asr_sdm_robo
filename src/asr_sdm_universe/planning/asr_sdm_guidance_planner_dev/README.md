# asr_sdm_guidance_planner_dev

ROS 2 3D A* + Bubble-Planner-style sphere corridor + L-BFGS waypoint planner for a fixed inflated occupancy map and an externally computed ESDF map.

The planner no longer recomputes ESDF internally. It uses:

```text
occupancy.bin or /esdf_map/occupancy_inflate
  -> occupied voxel map for A* collision checking and RViz occupied-map mesh

esdf.bin or /esdf_map/esdf_distance
  -> external ESDF distance map for L-BFGS obstacle cost and gradient
```

`/esdf_map/esdf_distance` must be a `sensor_msgs/msg/PointCloud2` compatible with `pcl::PointXYZI`:

```cpp
pt.x = pos(0);
pt.y = pos(1);
pt.z = pos(2);
pt.intensity = dist;  // real ESDF distance, not normalized
```


## Package structure

`asr_sdm_guidance_planner_dev` is organized as a C++ planning library plus an RViz test executable.
The installed public library exposes `GuidancePlanner` through:

```cpp
#include <guidance_planner.hpp>
```

The library-level responsibility is only:

```text
VoxelEsdfMap + start + goal
  ->  A* guide path
  ->  sphere corridor
  ->  L-BFGS optimized guidance waypoints
```

`RvizPlanningNode`, RViz point selection, topic publishing, and marker visualization are kept under `test/` as test executable source code. Runtime assets follow the usual ROS package layout at package root: `config/`, `launch/`, `rviz/`, and `maps/`. The launch file still starts the test executable `rviz_astar_lbfgs_planner` so package-level testing remains the same. The RViz config intentionally keeps the modifier-related displays and the second Publish Point tool on `/planning/add_virtual_obstacle`, so the local path modifier test can reuse this RViz layout.

In the full planning stack, `planning_manager` should link against this package and call `GuidancePlanner::plan(...)` directly. `planning_manager` remains the central module that consumes the returned global guidance waypoints and then handles trajectory generation, collision checking, local path modifier / topo replan, and B-spline optimization.

## Binary map files

Put the two binary files in the test asset directory before building/installing the package:

```text
asr_sdm_guidance_planner_dev/maps/occupancy.bin
asr_sdm_guidance_planner_dev/maps/esdf.bin
```

The default config uses a package-relative directory:

```yaml
binary_map.directory: "maps"
binary_map.occupancy_filename: "occupancy.bin"
binary_map.esdf_filename: "esdf.bin"
```

After installation, this resolves to:

```text
install/asr_sdm_guidance_planner_dev/share/asr_sdm_guidance_planner_dev/maps
```

If you use `colcon build --symlink-install`, copying the `.bin` files into the source package `maps/` directory is usually enough. Without `--symlink-install`, rebuild or reinstall after copying the files.

## Topic mode

Set:

```yaml
map.source: "topic"
```

Then the node subscribes to both:

```text
/esdf_map/occupancy_inflate   sensor_msgs/msg/PointCloud2, occupied voxel centers
/esdf_map/esdf_distance       sensor_msgs/msg/PointCloud2, x/y/z/intensity = ESDF distance
```

Planning is allowed only after both occupancy and ESDF are ready.

## Main inputs

```text
Binary mode:
  maps/occupancy.bin
  maps/esdf.bin

Topic mode:
  /esdf_map/occupancy_inflate
  /esdf_map/esdf_distance

RViz/user selection:
  /clicked_point  (first click = start, second click = goal)
```

## Main outputs

```text
/planning/waypoints           nav_msgs/msg/Path, optimized waypoints inside the safe corridor
/planning/safe_corridor       visualization_msgs/msg/MarkerArray, semi-transparent sphere corridor
/planning/astar_path_marker   visualization_msgs/msg/Marker
/planning/waypoints_marker    visualization_msgs/msg/Marker
/planning/start_goal_marker   visualization_msgs/msg/Marker
/esdf_map/occupied_map        visualization_msgs/msg/Marker, TRIANGLE_LIST occupied-map mesh
```

`/planning/waypoints` and the planning visualization markers are published with
`reliable + transient_local + keep_last(1)` QoS. This is equivalent to a cached
latest result: `local_path_modifier_test` can start after a plan has already been
published and still receive the most recent guidance waypoints.

## Run

```bash
cd <your_ros2_workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to asr_sdm_guidance_planner_dev --symlink-install
source install/setup.bash
ros2 launch asr_sdm_guidance_planner_dev astar_lbfgs_planner.launch.py
```

In RViz2, use `Publish Point: Start/Goal` twice:

```text
first click  -> start
second click -> goal and plan
```

## Important parameters

```yaml
frame_id: "world"
map.source: "binary"        # binary or topic
binary_map.directory: "maps"
binary_map.auto_bounds: false   # keep false to match asr_sdm_esdf_map mesh alignment
map.resolution: 0.15
visualization.ground_height: -1.0
visualization.visualization_truncate_height: 3.0
selection.clicked_point_use_msg_z: true
selection.default_planning_z: 0.5
selection.project_start_goal_to_safe: true
selection.safe_point_search_radius: 4.0
corridor.enabled: true
corridor.safety_margin: 0.10
corridor.min_radius: 0.10
corridor.batch_sample_count: 80
optimizer.safe_distance: 0.20
optimizer.corridor_weight: 60.0
```

For `/esdf_map/occupied_map` to render exactly like `asr_sdm_esdf_map`, keep these values consistent with `asr_sdm_esdf_map/config/esdf_map_config.yaml`:

```text
map.resolution
map.origin_x / map.origin_y / map.origin_z
map.size_x / map.size_y / map.size_z
visualization.ground_height
visualization.visualization_truncate_height
visualization.occupied_map_stride
visualization.occupied_map_alpha
visualization.occupied_map_mesh_resolution
```

`binary_map.auto_bounds` is now `false` by default because auto-changing the map origin/size can make the coarse mesh blocks misalign with `asr_sdm_esdf_map`. Set it to `true` only if your bin files use different bounds and you accept different RViz mesh alignment.

## Notes

- A* collision checking uses only the inflated occupancy map and now only provides guide points.
- The 3D safe corridor is generated as overlapping spheres. Each sphere radius is `ESDF_distance(center) - drone_radius - safety_margin`; the real sphere center is selected by BatchSample and scored by sphere volume plus adjacent overlap volume.
- L-BFGS optimizes the corridor-initialized waypoints and adds a corridor penalty so the published `/planning/waypoints` stay inside the sphere corridor.
- The `/esdf_map/occupied_map` Marker generation mirrors `SDFMap::publishOccupiedMap()` from `asr_sdm_esdf_map`: `TRIANGLE_LIST`, exposed cube faces only, height-based colors, `ground_height`, and `visualization_truncate_height` cropping.
- The old internal `computeEsdf()` logic has been removed.
- If the optimized waypoint path is unsafe or optimization fails, `/planning/waypoints` publishes the corridor-initialized safe waypoints as fallback when `selection.use_optimized_only_if_safe: true`.

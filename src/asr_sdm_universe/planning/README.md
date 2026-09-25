# ASR-SDM planning packages

This directory contains the planning-side packages used by the ASR-SDM stack.
Four executables use ROS 2 topics; the remaining packages are libraries called
by those nodes.

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

### Packages

| Package | Responsibility | Main products |
|---|---|---|
| [`asr_sdm_guidance_planner`](asr_sdm_guidance_planner/) | Fast-Planner grid A* front-end path search | library `Astar` |
| [`asr_sdm_guidance_planner_dev`](asr_sdm_guidance_planner_dev/) | 3D A* + sphere corridor + L-BFGS guidance waypoints | library `GuidancePlanner`; test node `rviz_astar_lbfgs_planner` |
| [`asr_sdm_local_path_modifier`](asr_sdm_local_path_modifier/) | Fast-Planner TopologyPRM, plus a TopologyPRM-style local detour library | libraries `TopologyPRM`, `TopoPathModifier`; test node `local_path_modifier_test` |
| [`asr_sdm_trajectory_optimizer`](asr_sdm_trajectory_optimizer/) | L-BFGS B-spline optimization: smoothness, clearance, feasibility and nonholonomic heading costs | library only |
| [`asr_sdm_trajectory_generator`](asr_sdm_trajectory_generator/) | Closed-form minimum-snap polynomial trajectory | library only; the legacy `traj_generator` node is not built |
| [`asr_sdm_trajectory_visualizer`](asr_sdm_trajectory_visualizer/) | RViz markers for goals, paths, B-splines, heading | library used by `planning_manager_node` |
| [`asr_sdm_planning_manager`](asr_sdm_planning_manager/) | Topological replan FSM, heading B-splines, trajectory execution | `planning_manager_node`, `traj_server`; msg `Bspline` |

Production dependencies are one-way:

```text
asr_sdm_esdf_map ------------------+
asr_sdm_guidance_planner ----------+  (Astar)
asr_sdm_local_path_modifier -------+  (TopologyPRM)
asr_sdm_trajectory_generator ------+
asr_sdm_trajectory_optimizer ------+--> asr_sdm_planning_manager
asr_sdm_trajectory_visualizer -----+
bspline ---------------------------+
```

`GuidancePlanner` and `TopoPathModifier` are not linked into `planning_manager`
yet. Those libraries are exercised by their test nodes.

The production planning path is:

```text
/goal_pose (or /waypoint_generator/waypoints) + odom
  -> planning_manager_node
       global min-snap polynomial
       local B-spline: TopologyPRM candidates → L-BFGS → lowest-jerk pick
       refine: stretch duration (capped) and re-optimize the selected candidate
       yaw / pitch from the trajectory tangent
  -> /planning/bspline
  -> traj_server (100 Hz)
  -> /control/asr_sdm/robot_cmd
```

The robot is treated as a nonholonomic head: body +x follows the path tangent
(`R = Rz(yaw) * Ry(pitch)`). The optimizer penalizes yaw rate, pitch rate, and
too-low forward speed on the position B-spline. After a topological detour is
chosen, refinement only reallocates time (`manager.max_time_lengthen_ratio`) and
trims clearance. `traj_server` then samples that heading into the twist fields
that `asr_sdm_control_manager` consumes.

`planning_simulator` can start this chain with `planning:=enable`.

### Topics

#### 1. `planning_manager_node` (`asr_sdm_planning_manager`)

`TopoReplanFSM` plus the in-process ESDF map (`asr_sdm_esdf_map`). Parameters
come from `asr_sdm_planning_manager/config/topo_replan.yaml`.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz goal. Position is the target; orientation +x is the arrival heading |
| `/waypoint_generator/waypoints` | `nav_msgs/msg/Path` | Waypoint list. Arrival heading is left unspecified |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 2D Pose Estimate. Drops the current plan and waits for the next goal |
| `odom` | `nav_msgs/msg/Odometry` | Robot pose. Remapped to `/visual_slam/odom` in `asr_sdm_planning_manager.launch.py` |

Launch also remaps the in-process ESDF map inputs (`/esdf_map/odom`,
`/esdf_map/cloud`, `/esdf_map/pose`, `/esdf_map/depth`).

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/planning/bspline` | `asr_sdm_planning_manager/msg/Bspline` | Local B-spline after a successful plan: position control points, knots, yaw, pitch |
| `/planning/replan` | `std_msgs/msg/Empty` | Replan trigger; tells `traj_server` to truncate the current trajectory |
| `/planning/new` | `std_msgs/msg/Empty` | New trajectory is ready |
| `/planning/stop` | `std_msgs/msg/Empty` | After a pose reset: halt execution and allow the next B-spline |

Visualization is published through `asr_sdm_trajectory_visualizer`:

| Topic | Type | Role |
|---|---|---|
| `/planning_vis/trajectory` | `visualization_msgs/msg/Marker` | Goal, geometric path, global min-snap polynomial (black), local B-spline (red) |
| `/planning_vis/topo_path` | `visualization_msgs/msg/Marker` | Topological candidate paths (phase 1 / phase 2) |
| `/planning_vis/prediction` | `visualization_msgs/msg/Marker` | Dynamic obstacle prediction |
| `/planning_vis/visib_constraint` | `visualization_msgs/msg/Marker` | Visibility constraints |
| `/planning_vis/frontier` | `visualization_msgs/msg/Marker` | Frontier |
| `/planning_vis/yaw` | `visualization_msgs/msg/Marker` | Body-axis heading along the trajectory (yaw + pitch) |

#### 2. `traj_server` (`asr_sdm_planning_manager`)

Subscribes to `/planning/bspline` and samples it at 100 Hz.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `planning/bspline` | `asr_sdm_planning_manager/msg/Bspline` | Trajectory to execute (resolves to `/planning/bspline` with no namespace) |
| `planning/replan` | `std_msgs/msg/Empty` | Truncate the current trajectory |
| `planning/new` | `std_msgs/msg/Empty` | Clear travelled-path visualization |
| `planning/stop` | `std_msgs/msg/Empty` | Full stop after a pose reset; later B-splines are accepted again |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Same RViz 2D Pose Estimate as the controller; stops immediately |
| `odom` | `nav_msgs/msg/Odometry` | Same remap as the planner |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/position_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | Sampled command. Remapped to `/control/asr_sdm/robot_cmd` in `asr_sdm_planning_manager.launch.py` |
| `planning/position_cmd_vis` | `visualization_msgs/msg/Marker` | Arrow along the current body +x (yaw + pitch) |
| `planning/travel_traj` | `visualization_msgs/msg/Marker` | Commanded positions so far |

The command fills Cartesian `position` / `velocity` / `acceleration` and `yaw` /
`yaw_dot`, and the head-frame twist that the controller uses:

- `vel.linear.x` — forward speed
- `vel.angular.y` — pitch rate
- `vel.angular.z` — yaw rate

With no namespace the last two visualization topics resolve to
`/planning/position_cmd_vis` and `/planning/travel_traj`.

#### 3. `rviz_astar_lbfgs_planner` (`asr_sdm_guidance_planner_dev` test node)

The library does not publish. The RViz test node does.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | First click = start, second click = goal and plan |
| `/esdf_map/occupancy_inflate` | `sensor_msgs/msg/PointCloud2` | Occupied voxels (topic map mode, or binary-load fallback) |
| `/esdf_map/esdf_distance` | `sensor_msgs/msg/PointCloud2` | ESDF distance in `intensity` (topic map mode, or binary-load fallback) |

Default map source is the package `maps/` binaries (`occupancy.bin`, `esdf.bin`).

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | L-BFGS-optimized guidance waypoints (QoS: reliable + transient_local) |
| `/planning/astar_path_marker` | `visualization_msgs/msg/Marker` | Raw A* path |
| `/planning/waypoints_marker` | `visualization_msgs/msg/Marker` | Optimized waypoint polyline |
| `/planning/start_goal_marker` | `visualization_msgs/msg/Marker` | Start and goal |
| `/planning/safe_corridor` | `visualization_msgs/msg/MarkerArray` | Spherical safe corridor |
| `/esdf_map/occupied_map` | `visualization_msgs/msg/Marker` | Occupied-map mesh (`TRIANGLE_LIST`) |

#### 4. `local_path_modifier_test` (`asr_sdm_local_path_modifier` test node)

The library does not publish. The test node does.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | Latest guidance waypoints (reliable + transient_local) |
| `/planning/add_virtual_obstacle` | `geometry_msgs/msg/PointStamped` | RViz click: add a temporary spherical obstacle |
| `/planning/clear_virtual_obstacles` | `std_msgs/msg/Empty` | Clear all temporary obstacles |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/planning/virtual_obstacles` | `visualization_msgs/msg/Marker` | Temporary spherical obstacles |
| `/planning/topo_candidate_paths` | `visualization_msgs/msg/MarkerArray` | Selected TopologyPRM candidate detours (yellow) |

It does **not** publish `/planning/modified_path`; it publishes all selected
candidates.

### Packages that do not publish

- `asr_sdm_trajectory_optimizer`: B-spline optimizer library (L-BFGS)
- `asr_sdm_trajectory_generator`: min-snap polynomial library; the standalone
  `traj_generator` demo is intentionally not built
- `asr_sdm_guidance_planner`: grid A* front-end search library
- Library parts of `asr_sdm_guidance_planner_dev` and `asr_sdm_local_path_modifier`:
  called by the manager or test nodes

### Production interfaces

The topics that matter for the production stack:

1. `/goal_pose` — operator goal (RViz)
2. `/control/initial_pose` — RViz 2D Pose Estimate; resets planning and the controller
3. `/planning/bspline` — planning output (position + yaw + pitch)
4. `/planning/replan`, `/planning/new`, `/planning/stop` — trajectory lifetime
5. `/position_cmd` (launch remaps to `/control/asr_sdm/robot_cmd`) — command for
   `asr_sdm_control_manager`

### Launch

```bash
# Topological replan + traj_server (production chain)
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py

# Same chain inside the planning simulator
ros2 launch planning_simulator planning_simulator.launch.py planning:=enable

# Guidance planner test (RViz start/goal clicks)
ros2 launch asr_sdm_guidance_planner_dev astar_lbfgs_planner.launch.py

# Local path modifier test (usually after guidance)
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py
```

Launch arguments for the production chain: `odom_topic` (default
`/visual_slam/odom`) and `cmd_topic` (default `/control/asr_sdm/robot_cmd`).

### Build

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-up-to asr_sdm_planning_manager asr_sdm_guidance_planner_dev asr_sdm_local_path_modifier \
  --symlink-install
source install/setup.bash
```

---

<a id="中文"></a>

## 中文

`planning` 目录里真正走 ROS topic 的是 4 个节点：`planning_manager_node`、
`traj_server`、`rviz_astar_lbfgs_planner`、`local_path_modifier_test`。其余包是库。

正式规划路径：

```text
/goal_pose（或 /waypoint_generator/waypoints）+ odom
  -> planning_manager_node
       全局 min-snap 多项式
       局部 B-spline：TopologyPRM 候选 → L-BFGS → 选 jerk 最小
       精修：拉长时间（有上限）并对选中候选再优化一次
       由轨迹切向得到 yaw / pitch
  -> /planning/bspline
  -> traj_server（100 Hz）
  -> /control/asr_sdm/robot_cmd
```

机器人按非完整头部处理：机体系 +x 跟随路径切向（`R = Rz(yaw) * Ry(pitch)`）。
优化器在位置 B-spline 上惩罚偏航角速度、俯仰角速度和过低前向速度。
选出拓扑绕行后，精修只重新分配时间（`manager.max_time_lengthen_ratio`）并修
间隙。`traj_server` 再把该朝向采样进 `asr_sdm_control_manager` 使用的
twist 字段。

`GuidancePlanner` 和 `TopoPathModifier` 还没有接到 `planning_manager`；
它们目前只在各自的测试节点里跑。`planning_manager` 用的是
`asr_sdm_guidance_planner` 的 `Astar` 和 `asr_sdm_local_path_modifier` 的
`TopologyPRM`。

`planning_simulator` 可用 `planning:=enable` 拉起这条正式链。

### 发布 / 订阅的 topic

#### 1. `planning_manager_node`（正式规划栈）

包：`asr_sdm_planning_manager`，由 `TopoReplanFSM` 发布。参数在
`config/topo_replan.yaml`。ESDF 地图在节点内通过 `asr_sdm_esdf_map` 初始化。

**Subscribe**

| Topic | 类型 | 内容 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 目标。位置是终点；姿态 +x 是到达朝向 |
| `/waypoint_generator/waypoints` | `nav_msgs/msg/Path` | 航点列表。不指定到达朝向 |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 2D Pose Estimate。丢掉当前规划，等待下一个目标 |
| `odom` | `nav_msgs/msg/Odometry` | 机器人位姿。launch 默认 remap 成 `/visual_slam/odom` |

launch 同时 remap 进程内 ESDF 地图输入（`/esdf_map/odom`、`/esdf_map/cloud`、
`/esdf_map/pose`、`/esdf_map/depth`）。

**Publish**

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/bspline` | `asr_sdm_planning_manager/msg/Bspline` | 规划成功后的局部 B-spline：控制点、knots、yaw、pitch |
| `/planning/replan` | `std_msgs/msg/Empty` | 触发重规划，通知 `traj_server` 截断当前轨迹 |
| `/planning/new` | `std_msgs/msg/Empty` | 新轨迹生成完成 |
| `/planning/stop` | `std_msgs/msg/Empty` | 位姿复位后：停止执行，并允许下一条 B-spline |

可视化通过库 `asr_sdm_trajectory_visualizer` 发出：

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning_vis/trajectory` | `visualization_msgs/msg/Marker` | 目标点、几何路径、全局 min-snap 多项式（黑）、局部 B-spline（红） |
| `/planning_vis/topo_path` | `visualization_msgs/msg/Marker` | 拓扑候选路径（phase1/phase2） |
| `/planning_vis/prediction` | `visualization_msgs/msg/Marker` | 动态障碍预测 |
| `/planning_vis/visib_constraint` | `visualization_msgs/msg/Marker` | 可见性约束 |
| `/planning_vis/frontier` | `visualization_msgs/msg/Marker` | frontier |
| `/planning_vis/yaw` | `visualization_msgs/msg/Marker` | 沿轨迹的机体系朝向（yaw + pitch） |

#### 2. `traj_server`（轨迹执行）

同一包，订阅 `/planning/bspline`，按 100 Hz 采样后下发控制指令。

**Subscribe**

| Topic | 类型 | 内容 |
|---|---|---|
| `planning/bspline` | `asr_sdm_planning_manager/msg/Bspline` | 待执行轨迹（无 namespace 时即 `/planning/bspline`） |
| `planning/replan` | `std_msgs/msg/Empty` | 截断当前轨迹 |
| `planning/new` | `std_msgs/msg/Empty` | 清空已飞轨迹可视化 |
| `planning/stop` | `std_msgs/msg/Empty` | 位姿复位后完全停止；之后的 B-spline 重新接受 |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 与控制器同一路 RViz 2D Pose Estimate；立即停轨迹 |
| `odom` | `nav_msgs/msg/Odometry` | 与规划节点同一 remap |

**Publish**

| Topic | 类型 | 内容 |
|---|---|---|
| `/position_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | 采样指令。launch 里 remap 成 `/control/asr_sdm/robot_cmd` |
| `planning/position_cmd_vis` | `visualization_msgs/msg/Marker` | 当前机体系 +x 方向箭头（yaw + pitch） |
| `planning/travel_traj` | `visualization_msgs/msg/Marker` | 已发出的指令位置 |

指令同时填笛卡尔 `position` / `velocity` / `acceleration` 和 `yaw` /
`yaw_dot`，以及控制器实际使用的机体系 twist：

- `vel.linear.x`：前进速度
- `vel.angular.y`：俯仰角速度
- `vel.angular.z`：偏航角速度

无 namespace 时后两个实际是 `/planning/position_cmd_vis`、`/planning/travel_traj`。

#### 3. `rviz_astar_lbfgs_planner`（guidance 测试节点）

包：`asr_sdm_guidance_planner_dev`。库本身不发 topic；测试节点发。

**Subscribe**

| Topic | 类型 | 内容 |
|---|---|---|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | 第一次点击为起点，第二次为终点并规划 |
| `/esdf_map/occupancy_inflate` | `sensor_msgs/msg/PointCloud2` | 占据体素（topic 地图模式，或二进制加载失败后的回退） |
| `/esdf_map/esdf_distance` | `sensor_msgs/msg/PointCloud2` | ESDF 距离在 `intensity`（同上） |

默认用地图包内 `maps/occupancy.bin`、`maps/esdf.bin`。

**Publish**

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | L-BFGS 优化后的引导路点（QoS：reliable + transient_local） |
| `/planning/astar_path_marker` | `visualization_msgs/msg/Marker` | A* 原始路径 |
| `/planning/waypoints_marker` | `visualization_msgs/msg/Marker` | 优化路点折线 |
| `/planning/start_goal_marker` | `visualization_msgs/msg/Marker` | 起终点 |
| `/planning/safe_corridor` | `visualization_msgs/msg/MarkerArray` | 球形安全走廊 |
| `/esdf_map/occupied_map` | `visualization_msgs/msg/Marker` | 占据地图 mesh（TRIANGLE_LIST） |

#### 4. `local_path_modifier_test`（局部改路测试节点）

包：`asr_sdm_local_path_modifier`。库本身不发 topic。

**Subscribe**

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | 最新引导路点（reliable + transient_local） |
| `/planning/add_virtual_obstacle` | `geometry_msgs/msg/PointStamped` | RViz 点选，加临时球形障碍 |
| `/planning/clear_virtual_obstacles` | `std_msgs/msg/Empty` | 清空临时障碍 |

**Publish**

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/virtual_obstacles` | `visualization_msgs/msg/Marker` | 临时球形障碍 |
| `/planning/topo_candidate_paths` | `visualization_msgs/msg/MarkerArray` | TopologyPRM 选出的黄色候选绕行路径 |

不发 `/planning/modified_path`，只发全部候选路径。

### 不发布 topic 的包

- `asr_sdm_trajectory_optimizer`：B-spline 优化库（L-BFGS）
- `asr_sdm_trajectory_generator`：min-snap 多项式库；独立 `traj_generator` 演示节点故意不编译
- `asr_sdm_guidance_planner`：栅格 A* 前端搜索库
- `asr_sdm_guidance_planner_dev` / `asr_sdm_local_path_modifier` 的库部分：被 manager 或测试节点调用

### 正式栈对外接口

1. `/goal_pose`：操作员目标（RViz）
2. `/control/initial_pose`：RViz 2D Pose Estimate，同时复位规划和控制器
3. `/planning/bspline`：规划输出（位置 + yaw + pitch）
4. `/planning/replan` / `/planning/new` / `/planning/stop`：轨迹生命周期
5. `/position_cmd`（launch 后为 `/control/asr_sdm/robot_cmd`）：给 `asr_sdm_control_manager` 的指令

### Launch

```bash
# 拓扑重规划 + traj_server（正式链）
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py

# 在 planning_simulator 里拉起同一条链
ros2 launch planning_simulator planning_simulator.launch.py planning:=enable

# guidance 测试（RViz 点选起终点）
ros2 launch asr_sdm_guidance_planner_dev astar_lbfgs_planner.launch.py

# 局部改路测试（通常接在 guidance 之后）
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py
```

正式链 launch 参数：`odom_topic`（默认 `/visual_slam/odom`）、`cmd_topic`
（默认 `/control/asr_sdm/robot_cmd`）。

### Build

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-up-to asr_sdm_planning_manager asr_sdm_guidance_planner_dev asr_sdm_local_path_modifier \
  --symlink-install
source install/setup.bash
```

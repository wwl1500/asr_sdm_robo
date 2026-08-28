# ASR-SDM control packages

This directory contains the control-side packages used by the ASR-SDM robot and
its simulators. Two executables use ROS 2 topics; the remaining code is
libraries called by those nodes.

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

### Packages

| Package | Responsibility | Main products |
|---|---|---|
| [`asr_sdm_head_following_control`](asr_sdm_head_following_control/) | ROS-independent 2D and 3D head-following algorithms | `front_unit_following_controller_2d_core`, `front_unit_following_controller_3d_core` |
| [`asr_sdm_online_path_following_control`](asr_sdm_online_path_following_control/) | Online local-window path following algorithm and optional realtime node | `online_path_following_controller_3d_core`, `realtime_online_path_following_controller_3d` |
| [`asr_sdm_front_following_control`](asr_sdm_front_following_control/) | Dynamics-based front-following (force/torque) and visualization | `asr_sdm_front_following`, `controller_visualization_node` |
| [`asr_sdm_kinematic_dynamic_model`](asr_sdm_kinematic_dynamic_model/) | Pinocchio kinematic/dynamic model, underwater hydrodynamics, and the retained legacy URDF dynamics utility | `asr_sdm_kinematic_model`, `underwater_dynamics`, `pinocchio_dynamics_node` |
| [`asr_sdm_control_manager`](asr_sdm_control_manager/) | ROS 2 topics, parameters, command handling, state integration, and robot-state publication | `asr_sdm_control_manager` |

Production dependencies are one-way:

```text
asr_sdm_head_following_control -----------+
                                           +--> asr_sdm_control_manager
asr_sdm_kinematic_dynamic_model ----------+

asr_sdm_head_following_control --> asr_sdm_online_path_following_control
asr_sdm_kinematic_dynamic_model --> asr_sdm_front_following_control
```

The runtime control path is:

```text
robot_cmd
  -> asr_sdm_control_manager
  -> head-following controller + Pinocchio model
  -> joint_states + odometry + optional hardware command
```

`planning_simulator` includes the control-manager launch file and uses the 3D
manager as the supported ROS control entry point.

### Topics

#### 1. `asr_sdm_control_manager` (production)

Launch name: `asr_sdm_kinematic_controller`. Topic names come from
`*_topic` parameters in `asr_sdm_control_manager/config/asr_sdm_control_manager.yaml`.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/robot_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | Velocity command in `vel`. Uses `vel.linear.x` (forward), `vel.angular.y` (pitch rate), `vel.angular.z` (yaw rate) |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reset pose (e.g. RViz 2D Pose Estimate) |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/odom` | `nav_msgs/msg/Odometry` | Integrated controller odometry (`world` → `base`) |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | 6 articulation joints + 8 screw rotors for `robot_state_publisher` |
| `/control/asr_sdm/controller_state_3d` | `std_msgs/msg/Float64MultiArray` | 43-D internal controller state |
| `/control/asr_sdm/control_cmd_3d` | `asr_sdm_control_msgs/msg/ActuatorCmd` | Hardware joint-angle command; **off by default** (`publish_control_cmd: false`) |

Control period defaults to 20 ms. If no `robot_cmd` arrives within
`cmd_timeout_sec` (0.3 s), the command is zeroed.

#### 2. `pinocchio_dynamics_node` (`asr_sdm_kinematic_dynamic_model`)

URDF-based dynamics utility. The control manager uses the URDF-free library
instead and does not subscribe to this node. No subscriptions; publishes at
`publish_period_ms` (default 200 ms) for the current configuration.

**Publish** (relative names; `/` prefix with no namespace)

| Topic | Type | Role |
|---|---|---|
| `pinocchio/mass_matrix` | `std_msgs/msg/Float64MultiArray` | Mass matrix (row-major, `nv × nv`) |
| `pinocchio/center_of_mass` | `geometry_msgs/msg/Vector3` | Center of mass |
| `pinocchio/total_mass` | `std_msgs/msg/Float64` | Total mass |

It does not subscribe to `joint_states`. Published values are for the
neutral configuration (or `initial_joint_positions`).

### Packages that do not publish

- `asr_sdm_head_following_control`: 2D/3D front-unit following libraries; optional demos are offline plots, not ROS nodes
- Library part of `asr_sdm_kinematic_dynamic_model`: `AsrSdmKinematicModel` and `UnderwaterDynamics`
- `asr_sdm_online_path_following_control` library: online path following; the optional realtime node is a separate executable, not the production manager
- `asr_sdm_front_following_control` library: dynamics inversion; its ROS nodes are a parallel stack, not the production manager

### Production interfaces

The three topics that matter for the production stack:

1. `/control/asr_sdm/robot_cmd` — teleop / planning input
2. `/control/asr_sdm/odom` — controller odometry
3. `/control/joint_states` — robot-model visualization / TF

`planning_simulator` uses the same names (see its `topics:` section).

### Build

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-up-to asr_sdm_control_manager \
  --symlink-install \
  --parallel-workers 1
source install/setup.bash
```

See the model package README for the Pinocchio installation procedure. The
controller plotting demos are optional and are disabled in a normal library
build.

---

<a id="中文"></a>

## 中文

`control` 目录里正式控制栈走 ROS topic 的是 `asr_sdm_control_manager` 和
`pinocchio_dynamics_node`。另外还有两套并行节点：在线路径跟随和动力学前端跟随。
`asr_sdm_head_following_control` 是算法库，不发 topic。

运行路径：

```text
robot_cmd
  -> asr_sdm_control_manager
  -> 前端跟随控制器 + Pinocchio 运动学
  -> joint_states + odom + 可选硬件指令
```

### 发布 / 订阅的 topic

#### 1. `asr_sdm_control_manager`（正式控制栈）

节点名 launch 里是 `asr_sdm_kinematic_controller`。话题名来自
`config/asr_sdm_control_manager.yaml` 的 `*_topic` 参数。

**Subscribe**

| Topic | 类型 | 内容 |
|---|---|---|
| `/control/asr_sdm/robot_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | 速度指令在 `vel`。只用 `vel.linear.x`（前进）、`vel.angular.y`（俯仰角速度）、`vel.angular.z`（偏航角速度） |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 重置位姿（如 RViz 2D Pose Estimate） |

**Publish**

| Topic | 类型 | 内容 |
|---|---|---|
| `/control/asr_sdm/odom` | `nav_msgs/msg/Odometry` | 控制器积分得到的里程计（`world` → `base`） |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | 6 个关节 + 8 个螺旋桨关节，供给 `robot_state_publisher` |
| `/control/asr_sdm/controller_state_3d` | `std_msgs/msg/Float64MultiArray` | 43 维内部控制状态 |
| `/control/asr_sdm/control_cmd_3d` | `asr_sdm_control_msgs/msg/ActuatorCmd` | 硬件关节角指令；**默认不发布**（`publish_control_cmd: false`） |

控制周期默认 20 ms。`robot_cmd` 超过 `cmd_timeout_sec`（0.3 s）无新消息会清零指令。

#### 2. `pinocchio_dynamics_node`（URDF 动力学工具）

包：`asr_sdm_kinematic_dynamic_model`。控制管理节点用的是 URDF-free 库，不订阅这个节点。无订阅；按 `publish_period_ms`（默认 200 ms）发布当前构型的动力学量。

**Publish**（相对名，无 namespace 时前面加 `/`）

| Topic | 类型 | 内容 |
|---|---|---|
| `pinocchio/mass_matrix` | `std_msgs/msg/Float64MultiArray` | 质量矩阵（行优先，`nv × nv`） |
| `pinocchio/center_of_mass` | `geometry_msgs/msg/Vector3` | 质心 |
| `pinocchio/total_mass` | `std_msgs/msg/Float64` | 总质量 |

目前不订阅 `joint_states`，发布的是中性构型（或 `initial_joint_positions`）下的量。

### 不发布 topic 的包

- `asr_sdm_head_following_control`：2D/3D 前端跟随算法库；可选 demo 是离线绘图，不是 ROS 节点
- `asr_sdm_kinematic_dynamic_model` 的库部分：`AsrSdmKinematicModel` 和 `UnderwaterDynamics`
- `asr_sdm_online_path_following_control` 的库部分：在线局部窗口路径跟随
- `asr_sdm_front_following_control` 的库部分：动力学前端跟随；其 ROS 节点是并行栈

### 正式栈对外接口（最关键的 3 个）

1. `/control/asr_sdm/robot_cmd`：teleop / planning 的输入
2. `/control/asr_sdm/odom`：控制器里程计
3. `/control/joint_states`：驱动机器人模型可视化 / TF

`planning_simulator` 使用同一组名字（见其 `topics:` 段）。

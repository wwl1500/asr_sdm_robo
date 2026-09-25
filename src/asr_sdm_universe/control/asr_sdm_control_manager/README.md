# asr_sdm_control_manager

Front-unit following kinematic controller: turns `robot_cmd` into joint states and odometry to drive the robot model.
Default node is the **control manager** `asr_sdm_control_manager` (used by this package’s launch and by `planning_simulator`).

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

Topic names live under `topics` in `config/asr_sdm_control_manager.yaml`. Callers (e.g. `planning_simulator`) can pass another file with the same schema via `config_file`.

### Topics

#### Control manager (default, `asr_sdm_control_manager`)

Main link with teleop / planning: `/control/asr_sdm/robot_cmd`.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/robot_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | Velocity command in `vel` (teleop / planning) |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reset pose (e.g. RViz 2D Pose Estimate). `asr_sdm_planning_manager` listens on the same topic and drops the current plan |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/odom` | `nav_msgs/msg/Odometry` | Controller odometry |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | Joint states for `robot_state_publisher` |
| `/control/asr_sdm/controller_state_3d` | `std_msgs/msg/Float64MultiArray` | Internal controller state |
| `/control/asr_sdm/control_cmd_3d` | `asr_sdm_control_msgs/msg/ActuatorCmd` | Hardware command; **off by default** (`publish_control_cmd: false`) |

YAML keys:

```yaml
topics:
  odom: /control/asr_sdm/odom
  joint_states: /control/joint_states
  initialpose: /control/initial_pose
  robot_cmd: /control/asr_sdm/robot_cmd
  controller_state: /control/asr_sdm/controller_state_3d
  control_cmd: /control/asr_sdm/control_cmd_3d
```

#### Controller libraries and optional demos

The ROS-independent 2D/3D controller libraries are provided by
`asr_sdm_head_following_control`. Its offline plotting demos are disabled in a
normal build and can be enabled with `-DBUILD_CONTROLLER_DEMOS=ON`. The ROS
control node remains in this package.

### Build

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_control_manager
source install/setup.bash
```

Optional controller plotting demos need Matplot++ and Python dependencies:

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
./install_matplotplusplus.sh
colcon build --packages-select asr_sdm_head_following_control \
  --cmake-args -DBUILD_CONTROLLER_DEMOS=ON
```

### Launch

This package’s launch starts the control manager:

```bash
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
```

Optional: `config_file:=/path/to.yaml` (same schema as `config/asr_sdm_control_manager.yaml`).

With `planning_simulator`:

```bash
ros2 launch planning_simulator planning_simulator.launch.py control:=enable
# gamepad teleop
ros2 launch planning_simulator planning_simulator.launch.py control:=enable teleop:=enable
```

`planning_simulator` includes this package’s launch and passes its own `config/planning_simulator.yaml` as `config_file`.

Run the manager directly:

```bash
ros2 run asr_sdm_control_manager asr_sdm_control_manager
```

Gamepad input comes from `asr_sdm_teleop` (+ `joy`), which publishes to `/control/asr_sdm/robot_cmd`.

### Optional offline plotting demos

Build the algorithm package with `-DBUILD_CONTROLLER_DEMOS=ON` first, then run:

```bash
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_2d
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_3d
```

### Main parameters (3D)

Override with `--ros-args -p name:=value` or edit the yaml.

| Parameter | Default |
|---|---|
| `robot_cmd_topic` | `/control/asr_sdm/robot_cmd` |
| `initialpose_topic` | `/control/initial_pose` |
| `odom_topic` | `/control/asr_sdm/odom` |
| `joint_state_topic` | `/control/joint_states` |
| `controller_state_topic` | `/control/asr_sdm/controller_state_3d` |
| `control_cmd_topic` | `/control/asr_sdm/control_cmd_3d` |
| `control_period_ms` | `20` |
| `cmd_timeout_sec` | `0.3` |
| `max_linear_velocity` | `0.12` |
| `max_pitch_rate` | `0.35` |
| `max_yaw_rate` | `0.35` |
| `publish_control_cmd` | `false` |
| `link_length` | `0.25` |
| `joint_signs` | `[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]` |
| `joint_rate_limit` | `2.0` |
| `joint_limit` | `~π/2` |
| `max_curvature` | `1.2` |
| `damping` | `0.02` |

---

<a id="中文"></a>

## 中文

前端跟随运动学控制器：将 `robot_cmd` 转为关节状态与里程计，驱动机器人模型。  
默认使用 **控制管理节点** `asr_sdm_control_manager`（本包 launch 与 `planning_simulator` 均如此）。

话题名集中在 `config/asr_sdm_control_manager.yaml` 的 `topics` 段；`planning_simulator` 可通过 `config_file` 传入同 schema 配置覆盖。

### Topics

#### 控制管理节点（默认，`asr_sdm_control_manager`）

与 teleop / planning 的主通道是 `/control/asr_sdm/robot_cmd`。

**Subscribe（输入）**

| Topic | 类型 | 用途 |
|---|---|---|
| `/control/asr_sdm/robot_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | 速度指令在 `vel`（teleop / planning） |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 重置位姿（如 RViz 2D Pose Estimate）。`asr_sdm_planning_manager` 订阅同一话题并丢掉当前规划 |

**Publish（输出）**

| Topic | 类型 | 用途 |
|---|---|---|
| `/control/asr_sdm/odom` | `nav_msgs/msg/Odometry` | 控制器里程计 |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | 关节状态（供给 `robot_state_publisher`） |
| `/control/asr_sdm/controller_state_3d` | `std_msgs/msg/Float64MultiArray` | 内部控制状态 |
| `/control/asr_sdm/control_cmd_3d` | `asr_sdm_control_msgs/msg/ActuatorCmd` | 硬件指令；**默认不发布**（`publish_control_cmd: false`） |

对应 yaml 键：

```yaml
topics:
  odom: /control/asr_sdm/odom
  joint_states: /control/joint_states
  initialpose: /control/initial_pose
  robot_cmd: /control/asr_sdm/robot_cmd
  controller_state: /control/asr_sdm/controller_state_3d
  control_cmd: /control/asr_sdm/control_cmd_3d
```

#### 控制算法库和可选演示程序

ROS-independent 的 2D/3D 控制算法库由
`asr_sdm_head_following_control` 提供。离线绘图演示程序在普通构建中默认关闭，可通过
`-DBUILD_CONTROLLER_DEMOS=ON` 显式启用；ROS 控制节点仍由本包提供。

### 编译

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_control_manager
source install/setup.bash
```

可选控制器绘图演示需要 Matplot++ 和 Python 依赖：

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
./install_matplotplusplus.sh
colcon build --packages-select asr_sdm_head_following_control \
  --cmake-args -DBUILD_CONTROLLER_DEMOS=ON
```

### 启动

本包 launch 启动控制管理节点：

```bash
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
```

可选：`config_file:=/path/to.yaml`（与 `config/asr_sdm_control_manager.yaml` 同 schema）。

与 `planning_simulator` 一起：

```bash
ros2 launch planning_simulator planning_simulator.launch.py control:=enable
# 手柄遥控
ros2 launch planning_simulator planning_simulator.launch.py control:=enable teleop:=enable
```

`planning_simulator` 会 include 本包 launch，并用自己的 `config/planning_simulator.yaml` 作为 `config_file`。

直接运行控制管理节点：

```bash
ros2 run asr_sdm_control_manager asr_sdm_control_manager
```

手柄链路由 `asr_sdm_teleop`（及 `joy`）提供，发布到 `/control/asr_sdm/robot_cmd`。

### 可选离线绘图演示

先使用 `-DBUILD_CONTROLLER_DEMOS=ON` 构建算法包，然后运行：

```bash
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_2d
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_3d
```

### 主要参数（3D）

覆盖方式：`--ros-args -p 参数名:=参数值`，或改 yaml。

| 参数 | 默认 |
|---|---|
| `robot_cmd_topic` | `/control/asr_sdm/robot_cmd` |
| `initialpose_topic` | `/control/initial_pose` |
| `odom_topic` | `/control/asr_sdm/odom` |
| `joint_state_topic` | `/control/joint_states` |
| `controller_state_topic` | `/control/asr_sdm/controller_state_3d` |
| `control_cmd_topic` | `/control/asr_sdm/control_cmd_3d` |
| `control_period_ms` | `20` |
| `cmd_timeout_sec` | `0.3` |
| `max_linear_velocity` | `0.12` |
| `max_pitch_rate` | `0.35` |
| `max_yaw_rate` | `0.35` |
| `publish_control_cmd` | `false` |
| `link_length` | `0.25` |
| `joint_signs` | `[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]` |
| `joint_rate_limit` | `2.0` |
| `joint_limit` | `约 π/2` |
| `max_curvature` | `1.2` |
| `damping` | `0.02` |

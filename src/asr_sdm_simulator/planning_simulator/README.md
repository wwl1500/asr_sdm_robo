# planning_simulator

Planning simulation stack: map generation, RViz, plus optional robot model, kinematic control, gamepad teleop, and planning.

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

### Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to planning_simulator
source install/setup.bash
```

### Launch

```bash
ros2 launch planning_simulator planning_simulator.launch.py
```

By default this starts the random map, RViz, the `asr_sdm` robot model, the kinematic controller, and `asr_sdm_log_collector`. The model appears at `(-5, 0, 0)`. Gamepad teleop and planning are off. Logs land under `~/log/vehicle/latest`.

List all launch arguments:

```bash
ros2 launch planning_simulator planning_simulator.launch.py --show-args
```

### Launch arguments

| Argument | Values | Default | Role |
|---|---|---|---|
| `robot_model` | installed model package name | `asr_sdm` | Selects the robot model package; must provide `launch/<name>_description.launch.py` |
| `odom_source` | `auto` / `control` / `vins` | `auto` | Odometry that becomes the `world -> base` transform of the robot model. `auto` follows whichever source published most recently |
| `control` | `enable` / `disable` | `enable` | Starts `asr_sdm_control_manager` (kinematic controller) |
| `teleop` | `enable` / `disable` | `disable` | Starts `asr_sdm_teleop` (joy driver + teleop node) |
| `planning` | `enable` / `disable` | `disable` | Starts `asr_sdm_planning_manager` (topological replanning) |
| `log_collector` | `enable` / `disable` | `enable` | Starts `asr_sdm_log_collector` (merged rotating logs under `~/log/vehicle`) |

Common combinations:

```bash
# Default: map + model + controller (drive with robot_cmd)
ros2 launch planning_simulator planning_simulator.launch.py

# Add gamepad teleop
ros2 launch planning_simulator planning_simulator.launch.py teleop:=enable

# Add planning
ros2 launch planning_simulator planning_simulator.launch.py planning:=enable

# Pin the model pose to one source instead of following whoever publishes
ros2 launch planning_simulator planning_simulator.launch.py odom_source:=vins
ros2 launch planning_simulator planning_simulator.launch.py odom_source:=control

# Everything on
ros2 launch planning_simulator planning_simulator.launch.py \
  robot_model:=asr_sdm control:=enable teleop:=enable planning:=enable

# Without the log collector
ros2 launch planning_simulator planning_simulator.launch.py log_collector:=disable

# Map and RViz only, no controller (model will not move; may be invisible in RViz if Fixed Frame is world)
ros2 launch planning_simulator planning_simulator.launch.py control:=disable
```

### Parameters and config

Node parameters and topic names live in:

```text
config/planning_simulator.yaml
```

This file documents the topic names used by the launch. Initial pose and controller gains live in `asr_sdm_control_manager/config/asr_sdm_control_manager.yaml`. Restart the launch after editing (this package uses symlink install).

### Topics

| Topic | Description |
|---|---|
| `/control/asr_sdm/robot_cmd` | Input from teleop / planning; drives the kinematic controller |
| `/control/asr_sdm/odom` | Odometry published by the controller; model pose source with `odom_source:=control`. Planning and map sensing also use this topic so a new goal starts from the current robot pose |
| `/localization/video_inertial_navigation_systems/odometry` | VINS odometry; model pose source with `odom_source:=vins` |
| `/control/joint_states` | Joint states for `robot_state_publisher` |
| `/control/initial_pose` | RViz 2D Pose Estimate: reset controller pose and drop the current plan |
| `/simulator/planning_simulator/add_static_obstacle` | Click-to-add pillar obstacles (RViz Static Obstacle tool) |

Without a gamepad, publish a RobotCommand (using `vel`) to exercise the controller:

```bash
ros2 topic pub --rate 20 /control/asr_sdm/robot_cmd asr_sdm_control_msgs/msg/RobotCommand \
  "{vel: {linear: {x: 0.50, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.20}}}"
```

### Package conventions

Optional stacks are included by name:

```text
<package>/launch/<package>.launch.py
```

Robot models use:

```text
<robot_model>/launch/<robot_model>_description.launch.py
```

Example: default `asr_sdm` → `asr_sdm/launch/asr_sdm_description.launch.py`.

Responsibilities:

| Package / launch | Responsibility |
|---|---|
| `planning_simulator` | Map, RViz, top-level assemble |
| `asr_sdm` / `asr_sdm_description.launch.py` | URDF and static TF |
| `asr_sdm_control_manager` | Kinematic controller |
| `asr_sdm_teleop` | `joy` + teleop |
| `asr_sdm_planning_manager` | Planning and trajectory server |
| `asr_sdm_log_collector` | Merged rotating logs (`~/log/vehicle`) |

### Standalone launches (debug)

```bash
ros2 launch asr_sdm asr_sdm_description.launch.py
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
ros2 launch asr_sdm_teleop asr_sdm_teleop.launch.py
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py
ros2 launch asr_sdm_log_collector asr_sdm_log_collector.launch.py
```

The model launch alone does not publish a world-frame pose. The controller’s odom only becomes `world → base` when this package’s `odom_visualization` (`tf45`) is running. For normal use, start `planning_simulator.launch.py`.

---

<a id="中文"></a>

## 中文

### 编译

在工作空间根目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to planning_simulator
source install/setup.bash
```

### 启动

```bash
ros2 launch planning_simulator planning_simulator.launch.py
```

默认会启动随机地图、RViz，并加载 `asr_sdm` 机器人模型、运动学控制器和 `asr_sdm_log_collector`。模型会显示在 `(-5, 0, 0)`，手柄遥控和规划模块默认关闭。日志写在 `~/log/vehicle/latest`。

查看全部启动参数：

```bash
ros2 launch planning_simulator planning_simulator.launch.py --show-args
```

### 启动参数

| 参数 | 可选值 | 默认 | 作用 |
|---|---|---|---|
| `robot_model` | 已安装的模型包名 | `asr_sdm` | 选择机器人模型包，需提供 `launch/<包名>_description.launch.py` |
| `odom_source` | `auto` / `control` / `vins` | `auto` | 选择哪一路里程计作为模型的 `world -> base` 变换来源，`auto` 跟随最近发消息的那一路 |
| `control` | `enable` / `disable` | `enable` | 启动 `asr_sdm_control_manager`（运动学控制器） |
| `teleop` | `enable` / `disable` | `disable` | 启动 `asr_sdm_teleop`（手柄驱动 + teleop 节点） |
| `planning` | `enable` / `disable` | `disable` | 启动 `asr_sdm_planning_manager`（拓扑重规划） |
| `log_collector` | `enable` / `disable` | `enable` | 启动 `asr_sdm_log_collector`（合并滚动日志，目录 `~/log/vehicle`） |

常用组合：

```bash
# 默认：地图 + 模型 + 控制器（可用 robot_cmd 驱动）
ros2 launch planning_simulator planning_simulator.launch.py

# 加手柄遥控
ros2 launch planning_simulator planning_simulator.launch.py teleop:=enable

# 加规划
ros2 launch planning_simulator planning_simulator.launch.py planning:=enable

# 固定用某一路里程计驱动模型，不再自动跟随
ros2 launch planning_simulator planning_simulator.launch.py odom_source:=vins
ros2 launch planning_simulator planning_simulator.launch.py odom_source:=control

# 全开
ros2 launch planning_simulator planning_simulator.launch.py \
  robot_model:=asr_sdm control:=enable teleop:=enable planning:=enable

# 不启动日志采集
ros2 launch planning_simulator planning_simulator.launch.py log_collector:=disable

# 只看地图和 RViz，不启动控制器（模型不会动，RViz Fixed Frame 为 world 时可能看不见模型）
ros2 launch planning_simulator planning_simulator.launch.py control:=disable
```

### 参数与配置

节点参数和话题名集中在：

```text
config/planning_simulator.yaml
```

本文件记录 launch 使用的话题名。初始位姿和控制器增益在 `asr_sdm_control_manager/config/asr_sdm_control_manager.yaml`。改完后重新启动 launch（本包用 symlink install）。

### 相关话题

| 话题 | 说明 |
|---|---|
| `/control/asr_sdm/robot_cmd` | 手柄 / 规划侧输入，驱动运动学控制器 |
| `/control/asr_sdm/odom` | 控制器发布的里程计；`odom_source:=control` 时作为模型位姿来源。规划与地图感知也订阅该话题，第二次设目标会从当前机器人位置开始 |
| `/localization/video_inertial_navigation_systems/odometry` | VINS 里程计；`odom_source:=vins` 时作为模型位姿来源 |
| `/control/joint_states` | 控制器发布的关节状态，供给 `robot_state_publisher` |
| `/control/initial_pose` | RViz 2D Pose Estimate：重置控制器位姿并丢掉当前规划 |
| `/simulator/planning_simulator/add_static_obstacle` | RViz Static Obstacle 工具点击加点柱障碍 |

无手柄时可用 topic 直接发速度测试控制器：

```bash
ros2 topic pub --rate 20 /control/asr_sdm/robot_cmd asr_sdm_control_msgs/msg/RobotCommand \
  "{vel: {linear: {x: 0.50, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.20}}}"
```

### 模块约定

可选栈按「包名等于目录名」约定被 include：

```text
<package>/launch/<package>.launch.py
```

机器人模型额外约定：

```text
<robot_model>/launch/<robot_model>_description.launch.py
```

例如当前默认模型是 `asr_sdm` → `asr_sdm/launch/asr_sdm_description.launch.py`。

责任划分：

| 包 / launch | 负责 |
|---|---|
| `planning_simulator` | 地图、RViz、一键拼装 |
| `asr_sdm` / `asr_sdm_description.launch.py` | URDF 与静态 TF |
| `asr_sdm_control_manager` | 运动学控制器 |
| `asr_sdm_teleop` | `joy` + teleop |
| `asr_sdm_planning_manager` | 规划与轨迹服务 |
| `asr_sdm_log_collector` | 合并滚动日志（`~/log/vehicle`） |

### 独立启动（调试用）

```bash
ros2 launch asr_sdm asr_sdm_description.launch.py
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
ros2 launch asr_sdm_teleop asr_sdm_teleop.launch.py
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py
ros2 launch asr_sdm_log_collector asr_sdm_log_collector.launch.py
```

模型独立启动时没有世界坐标系下的位姿；控制器提供的 odom 需要配合本包的 `odom_visualization`（`tf45`）才会出现 `world → base`。正常使用请直接启动本包的 `planning_simulator.launch.py`。

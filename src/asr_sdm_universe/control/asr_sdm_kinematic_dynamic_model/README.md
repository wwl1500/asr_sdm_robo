# ASR-SDM 运动学和动力学模型

本包包含两个不同功能的模型堆栈。

## 坐标系约定

本包遵循标准的 ROS REP-103 坐标系约定：
- **世界坐标系（World frame）**：+X 前进，+Y 左侧，+Z 上方
- **机器人基座坐标系（Robot base frame）** (`base`)：+X 前进，与世界坐标系方向一致
- **URDF 变换**：CAD 模型沿节段的 +Z 轴构建。
  通过 `rpy="0 -pi/2 0"` 的固定关节旋转节段，使得：
  - CAD +Z → World +X（前进方向）
  - CAD +X → World -Z（向下）
  - CAD +Y → World +Y（不变）

### 转子推力方向

转子推力沿**节段局部坐标系的 -X 轴**施加（在 CAD 坐标系中）。
经过 URDF 旋转 `rpy="0 -pi/2 0"` 后，产生沿世界坐标系 +X（前进）的推力。

`ActuatorModelParameters` 中默认的 `rotor_force_axis_local` 为 `{-1.0, 0.0, 0.0}`，
确保正推力命令使机器人在世界坐标系中向前移动。

**重要说明**：这与 `planning_simulator`（四旋翼）不同，后者的推力沿机体 +Z 轴（`R.col(2)`）。
对于蛇形机器人，URDF 预旋转意味着我们必须使用局部 -X 来实现世界坐标系 +X 推力。

## 模型堆栈

`asr_sdm_kinematic_model` 是 ASR-SDM 控制管理器使用的无 URDF 的 Pinocchio 模型。
它直接从配置的连杆和关节参数构建模型，并提供配置、运动学、雅可比、积分、质量矩阵和重力辅助功能。
控制管理器当前以运动学方式驱动它，积分命令速度。

`underwater_simulator` 是基于 URDF 的水下动力学堆栈：Pinocchio 刚体动力学耦合附加质量、
交叉耦合和莫里森阻尼、浮力、Froude-Krylov 力、带反作用力矩和推进器延迟的螺旋驱动推力，
以及被动关节摩擦和限位，可选择半隐式欧拉或 RK4 积分。有关运动方程、参数集和稳定性限制，
请参阅 [`README_simulator.md`](README_simulator.md)。

本包还保留了 `pinocchio_dynamics_node`，用于现有的基于 URDF 的动力学工具。

## Pinocchio 安装

支持的二进制设置是 Ubuntu 24.04 amd64 和 ROS 2 Jazzy。
本工作空间从 robotpkg 安装 Pinocchio、coal 和 eigenpy，路径为 `/opt/openrobots`。

从工作空间根目录：

```bash
cd ~/asr_sdm_robo
./setup-dev-env.sh --dependency
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
```

如果尚未安装 ROS 2 Jazzy，请同时安装两个环境：

```bash
./setup-dev-env.sh --ros --dependency
```

安装其余工作空间依赖项，但不替换 robotpkg 的 Pinocchio 安装：

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --skip-keys pinocchio
```

验证安装是否可见：

```bash
pkg-config --modversion pinocchio
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

不要在同一构建环境中混合 `/opt/openrobots` 中的 robotpkg Pinocchio/coal 库
和 `ros-jazzy-pinocchio`、`ros-jazzy-coal`。

旧版 Pinocchio 可能会打印消息要求用户将 `hpp-fcl` 更新为 `coal`。
本包不直接包含 hpp-fcl；该消息来自已安装 Pinocchio 版本使用的兼容层。
不要仅为抑制该消息而安装第二个 hpp-fcl 堆栈。

对于高级源码安装，请使用仓库级别的 `install_pinocchio_from_source.py`。
该脚本在运行时发现最新的稳定兼容 Pinocchio、eigenpy 和 Coal 版本，
而不是跟踪 `devel` 或固定版本标签。预览选定的标签和提交而不安装任何内容：

```bash
python3 install_pinocchio_from_source.py --resolve-only
```

Coal 会自动选择和构建。碰撞支持默认禁用，可通过 `PINOCCHIO_BUILD_COLLISION=ON` 启用。
所有三个项目使用检测到的逻辑 CPU 的一半构建（至少一个作业）。
有关源目录、引用和安装前缀覆盖，请参阅 
[`docs/pinocchio_installation.md`](../../../../docs/pinocchio_installation.md)。

## 水下仿真使用说明

### 快速开始

1. **编译包**：
```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_kinematic_dynamic_model
source install/setup.bash
```

2. **启动带 RViz2 的仿真**（持续运行模式）：
```bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=0 \
  run_forever:=true \
  with_rviz:=true \
  csv_path:=/tmp/underwater_sim.csv
```

### Launch 参数完整列表

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dt` | float | 0.005 | 仿真时间步长（秒），推荐 0.003–0.005 |
| `duration` | float | 60.0 | 仿真时长（秒），0 表示只输出初始状态 |
| `run_forever` | bool | false | 是否无限运行（忽略 duration） |
| `auto_shutdown` | bool | false | 仿真结束后是否自动关闭节点 |
| `csv_path` | string | underwater_simulation.csv | CSV 日志输出路径 |
| `with_rviz` | bool | true | 是否启动 RViz2 可视化 |
| `with_control_panel` | bool | false | 是否启动控制面板（GUI） |
| `integrator` | string | semi_implicit_euler | 积分器：`semi_implicit_euler` 或 `rk4` |
| `root_frame` | string | base | 机器人根坐标系名称 |
| `z_to_x_map` | bool | false | 旧版 URDF 坐标系映射（通常为 false） |
| `force_scale` | float | 0.02 | RViz 力矢量可视化缩放因子 |
| `max_trajectory_points` | int | 3000 | 轨迹可视化最大点数 |
| **关节动力学参数** | | | |
| `joint_viscous_damping` | array[6] | [0, 0, 0, 0, 0, 0] | 关节粘性阻尼 (Nm·s/rad)，推荐 20–100 |
| `joint_coulomb_friction` | array[6] | [0, 0, 0, 0, 0, 0] | 关节库仑摩擦 (Nm)，推荐 5–20 |
| `joint_limit_stiffness` | float | 0.0 | 关节限位弹簧刚度 (Nm/rad)，0 禁用 |
| `joint_limit_damping` | float | 0.0 | 关节限位阻尼 (Nm·s/rad) |
| `joint_limit` | float | 1.5708 | 关节行程限制 ±rad（默认 ±π/2） |
| `joint_friction_velocity_scale` | float | 0.001 | 摩擦正则化速度尺度 (rad/s) |

**注意**：数组参数需要用引号包裹，例如：`joint_viscous_damping:="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]"`

3. **发送推力命令**（在新终端中）：
```bash
cd ~/asr_sdm_robo
source install/setup.bash

# 设置节段推力（4个节段）
ros2 param set /underwater_simulator_node segment_thrust "[0.3, 0.3, 0.3, 0.3]"

# 设置转子转速（8个转子：每个节段的左右转子）
ros2 param set /underwater_simulator_node rotor_rate "[3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]"
```

### RViz2 配置

**Fixed Frame 设置**：
- `world`：世界坐标系（推荐）- 显示完整轨迹
- `base`：机器人基座坐标系 - 相机跟随机器人

两个选项现在都能正确显示水平放置的机器人，+X 朝前。

**Views 视图设置**：
- Target Frame：`base`（相机跟踪机器人）
- Distance：2-5 米
- Pitch：0.5-0.8

### 控制示例

#### 螺旋桨推力控制

**慢速前进**：
```bash
ros2 param set /underwater_simulator_node segment_thrust "[0.3, 0.3, 0.3, 0.3]"
ros2 param set /underwater_simulator_node rotor_rate "[3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]"
```

**蛇形运动**（交替推力）：
```bash
ros2 param set /underwater_simulator_node segment_thrust "[0.5, -0.5, 0.5, -0.5]"
ros2 param set /underwater_simulator_node rotor_rate "[5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]"
```

**快速前进**：
```bash
ros2 param set /underwater_simulator_node segment_thrust "[1.0, 1.0, 1.0, 1.0]"
ros2 param set /underwater_simulator_node rotor_rate "[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]"
```

**停止**：
```bash
ros2 param set /underwater_simulator_node segment_thrust "[0.0, 0.0, 0.0, 0.0]"
ros2 param set /underwater_simulator_node rotor_rate "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### 关节（舵机）主动控制

**关节弯曲**（6个关节独立控制）：
```bash
# 设置关节力矩 [joint1, joint2, joint3, joint4, joint5, joint6] (单位：N·m)
ros2 param set /underwater_simulator_node joint_torque "[0.5, 0.0, -0.5, 0.0, 0.5, 0.0]"
```

**S形姿态**（交替弯曲）：
```bash
ros2 param set /underwater_simulator_node joint_torque "[1.0, -1.0, 1.0, -1.0, 1.0, -1.0]"
```

**向上弯曲**（所有关节同向）：
```bash
ros2 param set /underwater_simulator_node joint_torque "[0.8, 0.8, 0.8, 0.8, 0.8, 0.8]"
```

**关节力矩归零**（回到被动状态）：
```bash
ros2 param set /underwater_simulator_node joint_torque "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### 组合控制示例

**推力 + 关节弯曲**（蛇形游动）：
```bash
# 设置推力
ros2 param set /underwater_simulator_node segment_thrust "[0.5, 0.5, 0.5, 0.5]"
# 设置关节弯曲形成波浪
ros2 param set /underwater_simulator_node joint_torque "[0.5, -0.5, 0.5, -0.5, 0.5, -0.5]"
```

**高速前进 + 稳定姿态**：
```bash
# 大推力前进
ros2 param set /underwater_simulator_node segment_thrust "[1.5, 1.5, 1.5, 1.5]"
# 关节保持小力矩稳定
ros2 param set /underwater_simulator_node joint_torque "[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]"
```

### 控制参数说明

#### 输入参数

仿真器接收三类主动控制输入：

1. **螺旋桨推力控制**（两种模式）：
   - `segment_thrust`：4个节段的总推力（每个节段的两个螺旋桨平分）
     - 数组长度：4
     - 单位：牛顿 (N)
     - 范围：受 `maximum_segment_thrust` 限制
   
   - `rotor_rate`：8个螺旋桨的角速度命令（在 `rotor_velocity` 模式下使用）
     - 数组长度：8（每个节段2个：左右转子）
     - 单位：弧度/秒 (rad/s)
     - 通过公式映射为推力：`f = k_lin * w + k_quad * w * |w|`

2. **关节（舵机）力矩控制**：
   - `joint_torque`：6个关节的主动控制力矩
     - 数组长度：6
     - 单位：牛顿·米 (N·m)
     - 默认值：`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`（被动状态）
     - 正值：使关节向正方向弯曲
     - 负值：使关节向负方向弯曲

3. **环境参数**（可选）：
   - `fluid_current_x/y/z`：世界坐标系下的水流速度 (m/s)
   - `fluid_current_acceleration_x/y/z`：水流加速度 (m/s²)

#### 被动关节动力学

当 `joint_torque` 为零时，关节处于被动状态，受以下力影响：

- **粘性阻尼**：`-b * θ_dot`
  - 参数：`joint_viscous_damping`（6维向量）
  - 单位：N·m·s/rad

- **库仑摩擦**：`-c * tanh(θ_dot / scale)`
  - 参数：`joint_coulomb_friction`（6维向量）
  - 单位：N·m
  - 平滑参数：`joint_friction_velocity_scale`

- **关节限位弹簧阻尼**：
  - 参数：`joint_limit_stiffness`、`joint_limit_damping`
  - 限位范围：`joint_limit`（默认：±π/2 弧度）

### 速度和力矩参考表

| 运动类型 | segment_thrust | rotor_rate | joint_torque | 使用场景 |
|---------|----------------|------------|--------------|----------|
| 静止观察 | 0.0 | 0.0 | 0.0 | 初始状态 |
| 慢速直行 | 0.3 | 3.0 | 0.0 | 推荐测试 |
| 中速直行 | 0.5 | 5.0 | 0.0 | 平衡速度 |
| 快速直行 | 1.0 | 10.0 | 0.0 | 高速运动 |
| 蛇形游动 | 0.5 | 5.0 | ±0.5 交替 | 仿生运动 |
| S形姿态 | 0.3 | 3.0 | ±1.0 交替 | 复杂姿态 |
| 最大性能 | 2.0 | 20.0 | ±2.0 | 极限测试 |

### 启动参数

```bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=<秒数>                 # 仿真持续时间（0表示持续运行）
  run_forever:=<true|false>        # 持续运行模式
  with_rviz:=<true|false>          # 启动 RViz2
  csv_path:=<路径>                 # CSV 日志文件路径
  dt:=<秒数>                       # 时间步长（默认：0.005）
```

### 实时参数调整

所有控制参数都支持在仿真运行时动态调整，无需重启：

```bash
# 查看当前参数
ros2 param list /underwater_simulator_node

# 获取当前值
ros2 param get /underwater_simulator_node segment_thrust
ros2 param get /underwater_simulator_node joint_torque

# 动态修改参数（立即生效）
ros2 param set /underwater_simulator_node segment_thrust "[新值]"
ros2 param set /underwater_simulator_node joint_torque "[新值]"
```

### 运动模式切换

#### 模式 1：纯推力模式（关节被动）
```bash
# 只用螺旋桨推进，关节随水流自然摆动
ros2 param set /underwater_simulator_node segment_thrust "[0.5, 0.5, 0.5, 0.5]"
ros2 param set /underwater_simulator_node joint_torque "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### 模式 2：纯关节模式（无推力）
```bash
# 只用关节弯曲，无推力
ros2 param set /underwater_simulator_node segment_thrust "[0.0, 0.0, 0.0, 0.0]"
ros2 param set /underwater_simulator_node joint_torque "[1.0, -1.0, 1.0, -1.0, 1.0, -1.0]"
```

#### 模式 3：推力+关节组合模式（推荐）
```bash
# 推力提供前进动力，关节调整姿态
ros2 param set /underwater_simulator_node segment_thrust "[0.8, 0.8, 0.8, 0.8]"
ros2 param set /underwater_simulator_node joint_torque "[0.3, -0.3, 0.3, -0.3, 0.3, -0.3]"
```

### RViz2 可视化元素

- **机器人模型**：带正确方向的 3D 模型
- **绿色轨迹线**：显示运动路径
- **红色力箭头**：显示流体力（阻力、浮力）
- **网格**：XY 平面参考网格

## 停止仿真与清理

下面是常见停机 / 清理场景的命令集合。仿真本身支持运行时调参（见上节），
所以大多数情况无需重启；这一节给出需要停机时的完整工具集。

### 切换仿真模式 / 重新启动前清理

如果需要切换不同的运动模式（推力 / 关节 / 组合），**不需要停止仿真**，
直接用 `ros2 param set` 修改即可（见上节）。只有当你想：
- 重新启动一个新仿真
- 切换 ROS2 域或参数
- 修改 launch 参数（如 `dt`、`integrator`）

时，才需要先停止再启动：

```bash
# 一键停止旧实例
pkill -f "underwater_simulator_node"

# 等 1-2 秒确保资源释放
sleep 2

# 启动新实例
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  integrator:=RK4 time_step:=0.003 \
  duration:=60 with_rviz:=true \
  csv_path:=/tmp/new_run.csv
```

如果旧节点因为参数发散卡住，无法响应 `pkill`，用下面"强制停止"中的
`SIGTERM` 或 `SIGKILL` 兜底。

### 停止仿真命令

#### 基本停止命令

```bash
# 方法 1：仅停止仿真节点（保留 RViz2 / 控制面板）
pkill -f "underwater_simulator_node"

# 方法 2：停止所有相关节点（仿真 + RViz2 + 控制面板）
pkill -f "underwater_simulator"
pkill -f "rviz2"
pkill -f "underwater_control_panel"

# 方法 3：一次性停止全部（推荐）
pkill -f "underwater"; pkill -f "rviz2"
```

#### 优雅停止（保留数据）

如果在 `duration:=0` + `run_forever:=true` 模式下运行仿真，
并在 `csv_log_path` 指定了日志文件，建议用优雅方式停止，让节点
正常关闭并写出最后的 CSV 行：

```bash
# 在启动仿真的终端按 Ctrl+C（推荐）
# 或者通过 ROS2 服务请求它关闭：
ros2 lifecycle get /underwater_simulator_node    # 查看状态
ros2 lifecycle shutdown /underwater_simulator_node  # 优雅关闭（需要 lifecycle 启用时）
```

#### 强制停止（紧急情况）

如果仿真陷入卡死（参数发散、终端无响应），用强制终止：

```bash
# SIGTERM（推荐先试这个，给节点清理机会）
pkill -SIGTERM -f "underwater_simulator_node"

# SIGKILL（最后手段，强制立刻终止，不留清理时间）
pkill -SIGKILL -f "underwater_simulator_node"
```

#### 验证进程已停止

```bash
# 检查水下仿真相关进程
ps -ef | grep -E "underwater_simulator|rviz2|control_panel" | grep -v grep

# 或者
pgrep -af "underwater"

# 查看 ROS2 当前节点
ros2 node list
```

输出应该为空（或不再出现 `underwater_simulator_node` / `rviz2`），
说明仿真已经完整关闭。

#### 清理残留数据（可选）

```bash
# 清理日志文件
rm -f /tmp/underwater_sim.csv
rm -f /tmp/test.csv

# 如果不再使用，可以清理编译目录
# cd ~/asr_sdm_robo
# rm -rf build install log  # 注意：这会删除所有已编译包，谨慎使用
```

#### 停止场景对照表

| 场景 | 推荐命令 | 说明 |
|------|---------|------|
| 正常测试结束 | `Ctrl+C` 或 `pkill -f "underwater_simulator_node"` | 优雅退出，CSV 完整 |
| 关闭可视化但保留节点 | `pkill -f "rviz2"` | 只关 RViz |
| 关闭控制面板 | `pkill -f "underwater_control_panel"` | 只关 Qt 面板 |
| 全部清理 | `pkill -f "underwater"; pkill -f "rviz2"` | 一次性清理 |
| 节点卡死 | `pkill -SIGTERM -f "underwater_simulator_node"` | 软终止，等待清理 |
| 完全卡死 | `pkill -SIGKILL -f "underwater_simulator_node"` | 硬终止，立即生效 |
| 重新启动前清理 | `pkill -f "underwater_simulator_node"; sleep 2` | 确保资源完全释放 |

## 模型验证

模型检查可执行文件仅在启用 `BUILD_TESTING` 时构建，
并且对 `asr_sdm_head_following_control` 有测试依赖。

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
source ~/.bashrc

colcon build \
  --packages-up-to asr_sdm_kinematic_dynamic_model \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON

source install/setup.bash
ros2 run asr_sdm_kinematic_dynamic_model asr_sdm_kinematic_model_check
```

动力学堆栈由 `underwater_dynamics_test` 覆盖，它检查刚体恒等式、
关节体力组装的独立交叉检查、雅可比时间变化的有限差分验证、
重力一致性、阻尼耗散性、螺旋反作用力矩、推进器延迟、
关节动力学和积分器精度：

```bash
cd build/asr_sdm_kinematic_dynamic_model && ctest --output-on-failure
```

## 🎯 关节控制测试报告

### 测试概述
- **测试日期**: 2026-09-05
- **测试时长**: 60秒
- **数据采样**: 20,002点 (333.4 Hz)
- **测试状态**: ✅ 成功，数值稳定无发散

### 测试结果
- **总移动距离**: 51.76米
- **最终位置**: (32.97, -13.55, -37.53) m
- **关节响应**: 所有6个关节正常响应力矩命令
- **关节总转动**: 约5000-6000度（多圈连续旋转）

### 测试的运动模式

| 时间 | 模式 | segment_thrust | joint_torque | 运动效果 |
|------|------|----------------|--------------|----------|
| 0-10s | 被动漂浮 | [0,0,0,0] | [0,0,0,0,0,0] | 重力下沉 |
| 10-20s | S形弯曲 | [0,0,0,0] | [0.5,-0.5,0.5,-0.5,0.5,-0.5] | 蛇形摆动 |
| 20-30s | 向上弯曲 | [0,0,0,0] | [1.0,1.0,1.0,1.0,1.0,1.0] | 身体上翻 |
| 30-40s | 推力+关节 | [0.5,0.5,0.5,0.5] | [0.3,-0.3,0.3,-0.3,0.3,-0.3] | 蛇形前进 |
| 40-50s | 高力矩关节 | [0,0,0,0] | [2.0,-2.0,2.0,-2.0,2.0,-2.0] | 大幅摆动 |
| 50-60s | 归零稳定 | [0.2,0.2,0.2,0.2] | [0,0,0,0,0,0] | 平稳前进 |

### 稳定性配置（推荐）

经过测试验证的稳定参数配置：

```bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  integrator:=rk4 \
  dt:=0.003 \
  duration:=60 \
  with_rviz:=true \
  joint_viscous_damping:="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]" \
  joint_coulomb_friction:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]"
```

**关键参数说明**：
- `integrator:=rk4` - 四阶龙格库塔积分器（最稳定，默认是 `semi_implicit_euler`）
- `dt:=0.003` - 3ms时间步长（足够小保证精度，默认 0.005）
- `joint_viscous_damping` - 关节粘性阻尼（Nm·s/rad），抑制震荡，推荐 20.0–100.0
- `joint_coulomb_friction` - 关节库仑摩擦（Nm），耗散能量，推荐 5.0–20.0

**其他可选关节参数**：
- `joint_limit_stiffness:=100.0` - 关节限位弹簧刚度（Nm/rad），默认 0.0（禁用）
- `joint_limit_damping:=5.0` - 关节限位阻尼（Nm·s/rad），默认 0.0
- `joint_limit:=1.57` - 关节行程范围（rad），默认 ±π/2
- `joint_friction_velocity_scale:=0.001` - 摩擦正则化速度尺度（rad/s）

**注意**：向量参数需要用引号包裹，例如 `"[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]"`

### 关键发现

✅ **验证通过的功能**：
1. 关节主动力矩控制完全正常
2. 螺旋桨推力+关节力矩组合控制有效
3. 实时参数调整（`ros2 param set`）即时生效
4. RK4积分器长时间运行稳定

⚠️ **注意事项**：
- 关节力矩建议限制在 ±2.0 Nm 范围内
- 使用欧拉积分器容易在高力矩下发散
- 时间步长过大（>0.005s）可能导致不稳定
- 阻尼和摩擦参数不足会引起震荡

### 复现测试

完整测试脚本：

```bash
# 1. 启动仿真（稳定配置）
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  integrator:=rk4 \
  dt:=0.003 \
  duration:=0 \
  run_forever:=true \
  with_rviz:=true \
  csv_path:=/tmp/test.csv \
  joint_viscous_damping:="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]" \
  joint_coulomb_friction:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]"

# 2. 新终端执行测试序列
sleep 10 && ros2 param set /underwater_simulator_node joint_torque "[0.5, -0.5, 0.5, -0.5, 0.5, -0.5]"
sleep 10 && ros2 param set /underwater_simulator_node joint_torque "[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
sleep 10 && ros2 param set /underwater_simulator_node segment_thrust "[0.5, 0.5, 0.5, 0.5]" && \
            ros2 param set /underwater_simulator_node joint_torque "[0.3, -0.3, 0.3, -0.3, 0.3, -0.3]"
sleep 10 && ros2 param set /underwater_simulator_node segment_thrust "[0.0, 0.0, 0.0, 0.0]" && \
            ros2 param set /underwater_simulator_node joint_torque "[2.0, -2.0, 2.0, -2.0, 2.0, -2.0]"
sleep 10 && ros2 param set /underwater_simulator_node segment_thrust "[0.2, 0.2, 0.2, 0.2]" && \
            ros2 param set /underwater_simulator_node joint_torque "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

详细测试报告见：`/tmp/joint_control_test_summary.md`

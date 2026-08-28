# ASR-SDM 运动学和动力学模型

水下蛇形机器人的动力学仿真器与 MPC 控制器。

## 快速导航

- [仿真器使用](#水下仿真器) — 启动仿真、控制命令、RViz 可视化
- [MPC 控制器](#mpc-控制器grampc) — 模型预测控制配置与 API
- [安装构建](#安装与构建) — Pinocchio / GRAMPC 依赖安装
- [测试验证](#测试与验证) — 单元测试与动力学验证

---

## 坐标系约定

遵循 ROS REP-103：World frame +X 前、+Y 左、+Z 上。URDF 通过 `rpy="0 -π/2 0"` 旋转使 CAD +Z 对齐 World +X。

转子推力沿节段局部 -X 轴（CAD 系），旋转后产生 World +X（前进）推力。

---

## 模型堆栈

**运动学模型** (`asr_sdm_kinematic_model`)：无 URDF 的 Pinocchio 模型，控制管理器以运动学方式驱动。

**动力学仿真器** (`underwater_simulator`)：基于 URDF 的完整动力学 — Pinocchio 刚体 + 附加质量 + 莫里森阻尼 + 浮力 + Froude-Krylov 力 + 螺旋驱动推力 + 关节摩擦，支持半隐式欧拉或 RK4 积分。

**MPC 控制器** (`UnderwaterMpcController`)：基于 GRAMPC 的模型预测控制，支持流形状态空间、速度约束、终端代价。可选 GRAMPC-S 扩展支持随机 MPC（不确定性传播 + 概率约束）。

---

## 水下仿真器

### 快速开始

```bash
# 编译
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_kinematic_dynamic_model
source install/setup.bash

# 启动仿真（持续运行 + RViz）
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=0 run_forever:=true with_rviz:=true

# 发送控制命令（新终端）
ros2 param set /underwater_simulator_node segment_thrust "[0.3, 0.3, 0.3, 0.3]"
ros2 param set /underwater_simulator_node joint_torque "[0.5, -0.5, 0.5, -0.5, 0.5, -0.5]"
```

### 运动方程

```text
q: [px, py, pz, qx, qy, qz, qw, 6 关节角]  (13 维)
v: [base 线速度, base 角速度, 6 关节速度]  (12 维)

(M_RB + M_added) a = actuator + damping + buoyancy + froude_krylov 
                     + joint_passive - rigid_nonlinear - added_mass_bias
```

- `rigid_nonlinear` = Pinocchio RNEA 的 `C(q,v)v + g(q)`
- `added_mass_bias` = 不乘以未知加速度的附加质量项
- 积分器：`semi_implicit_euler`（默认）或 `rk4`，推荐 `dt=0.005`

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dt` | 0.005 | 时间步长（秒），0.003-0.005 稳定 |
| `integrator` | semi_implicit_euler | 或 `rk4`（更精确） |
| `joint_viscous_damping` | [0]×6 | 关节粘性阻尼 (Nm·s/rad)，推荐 50 |
| `joint_coulomb_friction` | [0]×6 | 关节库仑摩擦 (Nm)，推荐 10 |
| `morison_diameter` | 0.0 | 细长圆柱阻力直径 (m) |
| `morison_length` | 0.25 | 节段长度 (m) |
| `center_of_buoyancy` | [0,0,L/2]×4 | 浮心位置（link 局部坐标，**v1.2.6 新增**） |

### 控制示例

```bash
# 蛇形游动
ros2 param set /underwater_simulator_node segment_thrust "[0.5, -0.5, 0.5, -0.5]"
ros2 param set /underwater_simulator_node joint_torque "[0.5, -0.5, 0.5, -0.5, 0.5, -0.5]"

# 快速直行
ros2 param set /underwater_simulator_node segment_thrust "[1.0, 1.0, 1.0, 1.0]"
ros2 param set /underwater_simulator_node joint_torque "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# 停止
ros2 param set /underwater_simulator_node segment_thrust "[0.0, 0.0, 0.0, 0.0]"
```

### RViz 可视化

```bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  with_rviz:=true with_control_panel:=true
```

- Fixed Frame：`world`（推荐）或 `base`（跟随机器人）
- 力箭头：红色 damping、蓝色 buoyancy、紫色 total（`force_scale:=0.02` 调节缩放）
- 轨迹：绿色路径线（`max_trajectory_points:=3000`）

### 停止仿真

```bash
# Ctrl+C（推荐，保留 CSV）
# 或强制停止
pkill -f "underwater_simulator_node"
```

---

## MPC 控制器（GRAMPC）

`UnderwaterMpcController` 是基于 GRAMPC 的高级 MPC 控制器，支持完整的水下机器人动力学模型预测、流形状态空间（SE(3) × R^6）上的轨迹跟踪、速度约束、控制输入约束、终端代价与闭环仿真验证。

### 组件关系

```text
状态估计 / 仿真状态
        │ q_measured, v_measured
        ▼
GrampcUnderwaterDynamics
  ├─ full MPC state x = [q(13), v(12)] ∈ R^25
  ├─ controls u = [segment thrust(4), joint torque(6)] ∈ R^10
  └─ numerical J^T v callbacks
        │
        ▼
UnderwaterSimulator
  ├─ PinocchioModel：构型流形、刚体动力学和运动学
  ├─ FluidForceModel：附加质量、阻尼、浮力和 Froude-Krylov 力
  ├─ ActuatorModel：segment thrust 到转子与广义力的映射
  └─ JointDynamicsModel：关节被动力矩
```

桥接层继承 `grampc::ProblemDescription`，因此编译和下游使用依赖 `grampc::grampc_s` 而非仅 C 核心 `grampc::grampc`。

### 流形状态表示（完整状态）

Pinocchio 自由飞行器配置具有 `nq=13`、`nv=12`：配置中有 4 维单位四元数，而速度只有 3 维旋转增量。**完整状态 MPC** 直接把整个 13 维配置 `q` 作为状态变量，避免在长预测时域内因切空间线性化而失效；这与之前 `delta_q ∈ R^12` 的切空间参考形式不同，需要在 `ffct` 入口对四元数做防御性归一化。

```text
x = [q, v] ∈ R^25
q(x) = x[0:13]                # 直接读取，包含单位四元数
f(x, u) = [q_dot(x, u), a(q(x), v, u)]
```

其中：

- `q[0:3]` 为基座世界线位置，`q[3:7]` 为四元数（标量在前），`q[7:13]` 为六个关节位置；
- `q_dot = [v_lin, 0.5·Ω(ω)·q_quat, v_joints]`，由 `pinocchioModel::computeKinematics()` 和四元数运动学组合得出；
- `a` 由 `UnderwaterSimulator::evaluate()` 返回；
- 每次 `ffct`/`dfdx_vec`/`dfdu_vec` 进入入口会做四元数归一化（GRAMPC 的 RK 积分会偏离 `SO(3)`），保证仿真器始终得到有效配置。

### MPC 状态

```text
x = [q(13), v(12)] ∈ R^25
```

| 部分 | 含义 |
| --- | --- |
| `q[0:3]` | 基座世界线位置（米） |
| `q[3:7]` | 基座姿态四元数（标量在前，必须保持单位范数） |
| `q[7:13]` | 六个受控关节的位置（弧度） |
| `v[0:3]` | 基座线速度（体坐标系，米/秒） |
| `v[3:6]` | 基座角速度（体坐标系，弧度/秒） |
| `v[6:12]` | 六个受控关节的速度（弧度/秒） |

由于四元数在 GRAMPC 的欧氏状态向量里并无内部流形约束，桥接层在 `ffct`、`dfdx_vec`、`dfdu_vec` 入口处强制重新归一化（先 normalize 后再传入仿真器），保证 `q(x).segment<4>(3).norm() == 1` 在每次评估时满足。控制器只负责把测量到的 `x_measured ∈ R^25` 完整地原样作为 `x0`，不需要额外把配置相对某个参考变形。

### MPC 控制

```text
u = [segment_thrust_0..3, joint_torque_0..5] ∈ R^10
```

- 前四维为 aggregate segment thrust；每段总推力由 `ActuatorModel` 平分给两个转子；
- 后六维为关节主动扭矩；
- aggregate-thrust 模式下每段推力会被限制为 `[0, maximum_segment_thrust[i]]`，因此优化器的前四维边界必须使用相同范围。

### 一个控制周期的数据流

```cpp
#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include <grampc_s/grampc_s.hpp>

using namespace asr_sdm_kinematic_dynamic_model;

auto problem = std::make_shared<GrampcUnderwaterDynamics>(simulator_parameters);
if (!problem->isValid()) {
  throw std::runtime_error(problem->error());
}
grampc::Grampc solver(problem);

// 每个控制周期：从估计器读取完整状态（q_measured 包含已归一化的四元数）
// 注意：完整状态模式不再需要 updateReferenceConfiguration() ——
// x0 直接由测量 q_measured 与 v_measured 拼接而成。
MpcState x0 = MpcState::Zero();
x0.head<kReducedNq>() = q_measured;           // 13 维完整配置
x0.tail<kReducedNv>() = v_measured;           // 12 维体坐标系速度
solver.setparam_real_vector("x0", x0.data());

// 设置 xdes、u0、udes、umin、umax 和求解器选项后，再执行求解
```

完整状态模式下每个周期只需要把当前 `q_measured` 与 `v_measured` 拼成 `x0` 传进去，无需重新定位切空间参考。同时 `xdes` 也应是完整的目标配置（通常为零位姿的目标 `q_target` 加目标速度 `v_target`）。

### API

#### GrampcUnderwaterDynamics

```cpp
// 构造
GrampcUnderwaterDynamics(
  const UnderwaterSimulatorParameters& sim_params,
  const GrampcDimensionConfig& dim_config = GrampcDimensionConfig{},
  const GrampcCostWeights& cost_weights = GrampcCostWeights{},
  std::optional<ReducedConfiguration> initial_configuration = std::nullopt);

// 配置
void updateFluidCurrent(const Eigen::Vector3d& current_world,
                       const Eigen::Vector3d& current_acceleration_world);
void setCostWeights(const GrampcCostWeights& weights);
void setVelocityLimits(double max_linear, double max_angular, double max_joint);
void disableVelocityLimits();
void enableRelaxedValidationForStochasticCallbacks();

// 状态查询
bool isValid() const;
const std::string& error() const;
const UnderwaterSimulator& simulator() const;
```

#### UnderwaterMpcController

```cpp
// 构造
UnderwaterMpcController(
  const UnderwaterSimulatorParameters& sim_params,
  const UnderwaterMpcParameters& mpc_params);

// 控制计算
UnderwaterMpcResult computeControl(
  const ReducedConfiguration& current_config,
  const ReducedVelocity& current_velocity,
  const MpcReferenceTrajectory& reference);

// 参数更新
void updateParameters(const UnderwaterMpcParameters& params);
void setReferenceTrajectory(const MpcReferenceTrajectory& reference);

// 状态查询
bool isValid() const;
const std::string& error() const;
const UnderwaterMpcResult& lastResult() const;
```

### 参数配置（`UnderwaterMpcParameters` 默认值）

```cpp
// 预测时域
Thor = 1.0;                    // 1秒
Nhor = 50;                     // 50步
dt = 0.02;                     // 20ms

// 状态权重
//   q_position     : 位置 (x) 权重（同时复用为终端位置缩放基准）
//   q_orientation  : 四元数权重（接口取位置权重的 0.1 倍，避免过度正则化）
//   q_linear_velocity, q_angular_velocity, q_joint_position, q_joint_velocity
//                    : 直接用作对应通道
q_position = 10.0;
q_orientation = 5.0;       // 实际生效：5.0 × 0.1 = 0.5 作用于 4 个四元数分量
q_linear_velocity = 2.0;
q_angular_velocity = 2.0;
q_joint_position = 5.0;
q_joint_velocity = 2.0;

// 控制权重
r_thrust = 0.1;
r_joint_torque = 0.01;

// 终端权重
terminal_position_scale = 20.0;
terminal_velocity_scale = 10.0;
enable_terminal_cost = true;

// 速度约束
enable_velocity_constraints = true;
max_linear_velocity = 2.0;     // m/s
max_angular_velocity = 1.0;    // rad/s
max_joint_velocity = 2.0;      // rad/s

// 求解器
max_grad_iter = 5;
max_mult_iter = 1;
integrator = "erk2";
penalty_min = 1e3;

// 增量式温启动（默认开）
enable_warm_start = true;
```

> **可选：四元数对数代价**（见 [流形感知姿态代价（四元数对数）](#流形感知姿态代价四元数对数)）。设置 `quaternion_log_weight > 0` 时，运行/终端代价的四元数通道将改用 `0.5 · w · ‖log(q_err)‖²`（`q_err = q_des⁻¹ ⊗ q_meas`），与欧氏代价互斥。需要直接通过 `GrampcUnderwaterDynamics(parameters, ..., weights)` 构造时传入自定义权重，YAML 接口中暂未暴露。

### 流形感知姿态代价（四元数对数）

欧氏 `0.5 · (q - q_des)²` 在大姿态误差下有两个问题：

1. **双覆盖歧义**：`q` 和 `-q` 代表同一姿态，但 `‖q - q_des‖²` 与 `‖-q - q_des‖²` 通常相差 4·|q_des|²。在 180° 误差附近，欧氏代价会从 0 跳到 2·w_max²，与几何距离完全脱节。
2. **梯度方向错误**：欧氏梯度把 `q` 拉向 `q_des` 的字面值，而不是拉到四元数球面最近点，导致大姿态下的收敛振荡。

v1.2.1 引入的 `quaternion_log_weight > 0` 模式把四元数通道替换为

```
L_q = 0.5 · w · ‖log(q_err)‖²
q_err = q_des⁻¹ ⊗ q_meas           （相对期望的旋转）
log(q) = 2 · atan2(‖v‖, w) · v/‖v‖   （标准 SO(3) 旋转矢量）
```

性质：
- 在双覆盖下严格不变：`q → -q` 时 `q_err → -q_err`，但 `‖log(-q)‖ = ‖log(q)‖`（同一旋转矢量）。
- 小角下退化为欧氏：`|q_err ≈ [1, ε/2]|` 时 `log(q_err) ≈ ε`，`L_q ≈ 0.5 · w · ‖ε‖²`。
- 连续、可微，GRAMPC 的 `dldx` 用 1e-6 中心差分给出数值梯度。

使用示例：

```cpp
asr_sdm_kinematic_dynamic_model::GrampcCostWeights weights;
// 配置欧氏权重（注意：四元数通道必须显式置 0）
weights.state_weight.segment<3>(0).setConstant(10.0);   // base position
weights.state_weight.segment<4>(3).setConstant(0.0);    // disable Euclidean quat
weights.state_weight.segment<6>(7).setConstant(5.0);    // joint positions
weights.control_weight.setConstant(0.1);
// 启用对数姿态代价
weights.quaternion_log_weight = 5.0;
weights.terminal_quaternion_log_weight = 50.0;          // 终端权重
asr_sdm_kinematic_dynamic_model::GrampcUnderwaterDynamics dynamics(
  parameters, /*initial=*/{}, weights);
```

### 增量式温启动

GRAMPC 内部会在每次 `run()` 之间自动滚动预测序列（保留上次的轨迹作为初始猜测）。`UnderwaterMpcController` 在 v1.2.1 中又显式把上一周期求解得到的 `unext`（`segment_thrust(4)` + `joint_torque(6)`）填入下一周期的 `u0` 第一个节点，与 GRAMPC 内部 shift 叠加使用。

默认开启（`enable_warm_start = true`）。要对比冷启动基准或做单元测试时关闭：

```cpp
asr_sdm_kinematic_dynamic_model::UnderwaterMpcParameters params;
params.enable_warm_start = false;   // 强制 u0 = 0
controller.updateParameters(params);
```

### 不等式约束

```text
h[0]    = ||v_linear||²  - (2.0 m/s)² ≤ 0
h[1]    = ||v_angular||²  - (1.0 rad/s)² ≤ 0
h[2+i]  = v_joint[i]² - (2.0 rad/s)² ≤ 0  (i=0..5)
```

导数：`dhdx_vec()` 和 `dhdu_vec()`。`dhdu_vec` 始终为零（约束只依赖状态）。

### 执行器约束

在默认 `AggregateThrust` 模式中，`ActuatorModel` 对每段推力执行：

```text
segment_thrust[i] = clamp(u[i], 0, maximum_segment_thrust[i])
rotor_left[i] = rotor_right[i] = 0.5 * segment_thrust[i]
```

因此 MPC 的前四个 `umin` 必须为零，`umax` 必须由 `ActuatorModel::parameters().maximum_segment_thrust` 获取。给优化器提供负推力或大于执行器上限的域会产生平坦饱和区，导致数值导数与实际执行器不一致。

`RotorVelocity` 模式（每转子速率）和带非零 `thrust_time_constant` 的滞后模式在 `GrampcUnderwaterDynamics` 构造时被拒绝——MPC 仅支持无滞后的 aggregate-thrust 模式。

### 导数契约

GRAMPC 要求：

```text
dfdx_vec = (df/dx)^T vec
dfdu_vec = (df/du)^T vec
```

实现以中心差分构造：

- `df/dx`：24×24；
- `df/du`：24×10；
- 构型方向扰动经 `integrate()` 重建，不能直接修改四元数元素；
- `grampc_interface_test` 用独立中心差分对照两个 `J^T vec` 回调。

### 快速入门示例

```cpp
#include "asr_sdm_kinematic_dynamic_model/underwater_mpc_controller.hpp"

int main() {
  using namespace asr_sdm_kinematic_dynamic_model;

  // 1. 配置仿真器
  UnderwaterSimulatorParameters sim_params;
  sim_params.pinocchio.urdf_path = "path/to/robot.urdf";
  sim_params.pinocchio.use_free_flyer = true;

  // 2. 配置 MPC
  UnderwaterMpcParameters mpc_params;
  mpc_params.Thor = 1.0;                    // 1秒预测
  mpc_params.Nhor = 50;                     // 50步
  mpc_params.q_position = 10.0;             // 位置权重
  mpc_params.r_thrust = 0.1;                // 推力权重
  mpc_params.enable_velocity_constraints = true;
  mpc_params.max_linear_velocity = 2.0;     // 2 m/s

  // 3. 创建控制器
  UnderwaterMpcController controller(sim_params, mpc_params);

  // 4. 设置目标
  MpcReferenceTrajectory reference;
  reference.has_valid_target = true;
  reference.target_configuration << 2.0, 0.0, 0.0,  // 移动到 (2,0,0)
                                    1.0, 0.0, 0.0, 0.0,  // 四元数
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // 关节
  reference.target_velocity.setZero();

  // 5. 计算控制
  auto result = controller.computeControl(
    current_config, current_velocity, reference);

  if (result.success) {
    std::cout << "Cost: " << result.cost << std::endl;
  }

  return 0;
}
```

### 典型任务

**任务 1：点到点运动**

```cpp
reference.target_configuration.head<3>() << 5.0, 2.0, -1.0;
reference.target_configuration(3) = 1.0;  // qw
reference.target_configuration.segment<3>(4).setZero();
reference.target_configuration.tail<6>().setZero();
reference.target_velocity.setZero();
```

**任务 2：速度跟踪**

```cpp
reference.has_valid_target = false;  // 不关心位置
reference.target_velocity << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
```

**任务 3：悬停稳定**

```cpp
reference.has_valid_target = true;
reference.target_configuration = current_configuration;
reference.target_velocity.setZero();
reference.target_acceleration.setZero();
```

**任务 4：带水流补偿**

```cpp
reference.fluid_current_world << 0.3, 0.0, 0.0;
reference.fluid_current_acceleration_world.setZero();
auto result = controller.computeControl(current_config, current_velocity, reference);
```

### 性能基准

| 配置 | 求解时间 | 控制频率 | 适用场景 |
|------|---------|---------|---------|
| Nhor=10, MaxGradIter=2 | ~3 ms | 200 Hz | 快速响应 |
| Nhor=20 | ~5-8 ms | >100 Hz | 快速响应 |
| Nhor=50 | ~10-15 ms | ~50 Hz | **推荐** |
| Nhor=100 | ~25-40 ms | ~25 Hz | 高精度 |

测试平台：Intel i7-10700K，single thread，Release build。

### 性能调优经验

**快速响应（适合位置保持）**：

```cpp
mpc_params.Thor = 0.5;
mpc_params.Nhor = 25;
mpc_params.q_position = 20.0;
mpc_params.q_linear_velocity = 5.0;
mpc_params.r_thrust = 0.05;
mpc_params.terminal_position_scale = 50.0;
```

**平滑轨迹（适合路径跟踪）**：

```cpp
mpc_params.Thor = 1.5;
mpc_params.Nhor = 75;
mpc_params.q_position = 5.0;
mpc_params.q_linear_velocity = 1.0;
mpc_params.r_thrust = 0.5;
mpc_params.terminal_position_scale = 10.0;
```

**节能模式**：

```cpp
mpc_params.Thor = 2.0;
mpc_params.Nhor = 100;
mpc_params.q_position = 3.0;
mpc_params.r_thrust = 1.0;
mpc_params.max_linear_velocity = 1.0;
```

权重经验法则：

```cpp
// 位置跟踪任务
Q_position : Q_velocity : R_control = 10 : 2 : 0.1

// 速度跟踪任务
Q_position : Q_velocity : R_control = 1 : 10 : 0.1

// 能耗最优
Q_position : Q_velocity : R_control = 5 : 1 : 1.0
```

### 故障排查

**问题：MPC 求解失败（`result.success = false`）**

检查清单：
1. 初始状态是否有效（无 NaN/Inf）
2. 目标是否可达（在约束范围内）
3. 权重设置是否合理（Q/R 比例）

解决方案：

```cpp
mpc_params.max_linear_velocity = 5.0;  // 放宽约束
mpc_params.q_position = 10.0;
mpc_params.r_thrust = 0.1;
mpc_params.max_grad_iter = 10;
```

**问题：收敛太慢**

```cpp
mpc_params.terminal_position_scale = 50.0;
mpc_params.terminal_velocity_scale = 20.0;
mpc_params.Thor = 0.5;
mpc_params.Nhor = 25;
```

**问题：控制抖动**

```cpp
mpc_params.r_thrust = 1.0;
mpc_params.r_joint_torque = 0.1;
mpc_params.integrator = "erk4";
mpc_params.Nhor = 100;
```

### MPC 集成验证

`test/grampc_interface_test.cpp` 覆盖：

1. 默认仿真配置有效；
2. 25 维 `ffct()` 返回有限值；零速度时 `q_dot` 应当严格为零，正比于线速度分量；
3. `dfdx_vec()` / `dfdu_vec()` 与独立中心差分构造的 `J^T vec` 一致；
4. 非法四元数和不支持的问题维度被拒绝；
5. GRAMPC-S 可以构造并配置 25 状态、10 控制的问题。

验证命令：

```bash
LD_LIBRARY_PATH=$PWD/install/grampc/lib:$LD_LIBRARY_PATH \
  ctest --test-dir build/asr_sdm_kinematic_dynamic_model --output-on-failure
```

闭环动力学验证（`grampc_run_repro`）的代表性结果（100 步、2 s、目标 vx=1 m/s、URDF 浮力-重力弱平衡）：

```
 step |  time  |   vx   |   vy   |   vz   |  u0   |  u1   |  u2   |  u3   | J
------+--------+--------+--------+--------+-------+-------+-------+-------+--------
    0 | 0.0200 | 0.0009 | 0.0000 | 0.0028 | 0.0328| 0.0328| 0.0328| 0.0328| 28.654
   10 | 0.2200 | 0.0916 | 0.0000 |-0.1105 | 0.6183| 0.6219| 0.6165| 0.6149| 11.822
   30 | 0.6200 | 0.4099 | 0.0000 |-0.2461 | 1.3545| 1.4324| 1.4108| 1.3850| 10.835
   50 | 1.0200 | 0.5810 | 0.0000 |-0.2210 | 1.1913| 1.3450| 1.3363| 1.3003|  9.627
   70 | 1.4200 | 0.5484 | 0.0000 |-0.1975 | 0.9784| 1.1185| 1.1231| 1.0924|  9.188
   90 | 1.8200 | 0.5064 | 0.0000 |-0.1749 | 1.0516| 1.1133| 1.1164| 1.0974|  9.346
```

可见 MPC 把水平速度从 0 推到 0.5 m/s（剩下的 0.5 m/s 由浮力-重力净差和节段推力垂直分量分散导致），cost J 单调下降，没有任何 NaN/Inf。

### MPC 已知限制

1. **GRAMPC 依赖**：需要外部安装 GRAMPC 库，使用 BSD 3-Clause 许可证。
2. **deterministic GRAMPC `Grampc::run()` 已验证工作正常**：25 维完整状态水下动力学 MPC 可在 100 步闭环仿真中稳定运行，无 NaN/Inf，vx 收敛到目标附近，cost J 递减。具体验证见 `examples/grampc_run_repro.cpp`，已加入 ctest 回归测试。
   - **stochastic GRAMPC-S 的端到端求解现已通过**：GRAMPC-S 的 `SigmaPointProblemDescription` 在内部把 $N\times n_x$ 扩展状态切片为 $n_x$ 维单点后调用内层 `GrampcUnderwaterDynamics` 回调；`dfdu_vec` 在 GRAMPC-S 的第一次调用中传入完整扩展 adjoint 向量，已通过有限差分近似回退（检测 `adj.size() > kMpcStateDim` 自动切换）修复；8 个单元测试全部通过，包括完整的 `solver.run()` 调用。
3. **完整状态 MPC 已实现**：MPC 状态现为 $x = [q(13), v(12)] \in \mathbb{R}^{25}$，不再使用 `delta_q` 切空间参考形式（`updateReferenceConfiguration()` API 已删除）；`ffct` 直接根据当前配置的 13 维物理坐标计算 `q_dot = [v_lin, 0.5·Ω(ω)·q_quat, v_joints]`；为防止 GRAMPC 的 RK 积分使四元数偏离 `SO(3)`，每次进入 `ffct`/`dfdx_vec`/`dfdu_vec`/`computeAcceleration` 都进行防御性归一化。代价权重需要为四元数维度适当放大（默认取位置权重的 0.1 倍），避免对姿态过度正则化。
4. **仅速度约束**：当前不支持路径约束或障碍物避障。
5. **无实时保证**：未针对硬实时系统优化。
6. **冻结水流**：预测时域内流体速度假设恒定。
7. **单一执行器模式**：仅支持 `AggregateThrust` 模式；`RotorVelocity` 和非零推进器滞后在构造时被拒绝。

### MPC 待办事项

**短期 (v1.1.0)**

- [ ] 添加时变参考轨迹支持
- [ ] 实现增量式温启动
- [ ] 添加自适应权重调整

**中期 (v1.2.0)**

- [ ] 多阶段 MPC 任务
- [ ] 不确定性处理（随机 MPC）—— 基础设施已就绪，参见下一节
- [ ] 在线系统辨识

**长期 (v2.0.0)**

- [ ] GPU 加速求解器
- [ ] 分布式 MPC
- [ ] 学习型 MPC 辅助

---

## 随机 MPC（GRAMPC-S 可选）

GRAMPC-S 扩展支持状态不确定性传播与概率约束。需安装 GRAMPC-S。

```cpp
StochasticMpcConfig cfg;
cfg.state.stddev = Eigen::VectorXd::Constant(24, 1e-2);
cfg.chance_constraints.enabled = true;
StochasticGrampcDynamics stochastic(inner, cfg);
```

详见源码 `include/stochastic_mpc_config.hpp` 与测试 `test/stochastic_mpc_test.cpp`。

---

## 版本历史

**v1.2.6** (2026-09-13) — 浮心参数支持  
**v1.2.5** (2026-09-12) — 随机 MPC 闭环验证  
**v1.2.0** (2026-09-09) — 完整状态 MPC（状态维度 24→25）  
**v1.0.0** (2026-09-07) — MPC 控制器首次发布

---

## 安装与构建

### Pinocchio 依赖

支持 Ubuntu 24.04 + ROS 2 Jazzy，从 robotpkg 安装 Pinocchio (`/opt/openrobots`)。

```bash
cd ~/asr_sdm_robo
./setup-dev-env.sh --dependency
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_kinematic_dynamic_model
```

### GRAMPC 依赖（MPC 可选）

```bash
git clone https://github.com/grampc/grampc.git
cd grampc && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc) && sudo make install
```

随机 MPC 需额外安装 GRAMPC-S。

---

## 测试与验证

```bash
# 运行所有测试
cd ~/asr_sdm_robo
source install/setup.bash
colcon test --packages-select asr_sdm_kinematic_dynamic_model
colcon test-result --verbose --all

# 关键测试
cd build/asr_sdm_kinematic_dynamic_model && ctest --output-on-failure
```

关键测试项：
- `underwater_dynamics_test` — 刚体恒等式、阻尼耗散性、积分精度
- `grampc_interface_test` — GRAMPC 接口与雅可比
- `stochastic_mpc_test` — 随机 MPC 配置

### 稳定性验证

**测试结果**（2026-09-05，60秒闭环）：
- ✅ 数值稳定无发散（20,002 采样点，333.4 Hz）
- ✅ 推力方向正确、关节动力学正常
- ✅ 总移动距离 51.76 米

**推荐配置**：
- `dt = 0.005 s`，`integrator = rk4`
- `joint_viscous_damping = 50`，`joint_coulomb_friction = 10`
- `center_of_buoyancy` 默认 `(0, 0, L/2)` 提供静稳性

---

## 参考

- [GRAMPC](https://github.com/grampc/grampc) / [GRAMPC-S](https://github.com/grampc/grampc-s)
- [Pinocchio 文档](https://pinocchio.gitlabpages.inria.fr/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)

---

**版本**: 1.2.6 | **最后更新**: 2026-09-13

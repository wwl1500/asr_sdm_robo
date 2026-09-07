# GRAMPC 水下动力学 MPC 架构

## 组件关系

```text
状态估计 / 仿真状态
        │ q_measured, v_measured
        ▼
GrampcUnderwaterDynamics
  ├─ reference configuration q_reference
  ├─ full MPC state x = [delta_q, v] ∈ R^24
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

桥接层继承 `grampc::ProblemDescription`，因此编译和下游使用均依赖
`grampc::grampc_s`，而非仅依赖 C 核心 `grampc::grampc`。

## 流形状态表示

Pinocchio 自由飞行器配置具有 `nq=13`、`nv=12`：配置中有 4 维单位四元数，而速度和切空间只有 3 维旋转增量。为避免 GRAMPC 将四元数作为无约束欧氏状态处理，接口定义：

```text
x = [delta_q, v] ∈ R^24
q(x) = integrate(q_reference, delta_q)
f(x, u) = [delta_q_dot, a(q(x), v, u)]
```

其中：

- `delta_q` 是相对 `q_reference` 的 12 维切空间位移；
- `delta_q_dot` 由 `PinocchioModel::configurationTangentDerivative()` 计算；
- `a` 由 `UnderwaterSimulator::evaluate()` 返回；
- `q_reference` 由控制器在每个 MPC 周期用测量构型更新。

这使所有用于动力学评估和有限差分的配置都保持四元数归一化。

## 一个控制周期的数据流

1. 获取状态估计 `q_measured`、`v_measured`；
2. 校验 `q_measured` 有限且四元数归一化；
3. 调用 `updateReferenceConfiguration(q_measured)`；
4. 设置 `x0 = [0, v_measured]`；
5. 配置参考状态、控制上下界和 GRAMPC 选项；
6. 求解后将前四个控制量映射到 `segment_thrust`，后六个映射到 `joint_torque`；
7. 在仿真中通过 `UnderwaterSimulator::step()` 应用控制；在真实系统中应经过单独设计的 control manager / hardware 安全映射。

## 执行器约束

在默认 `AggregateThrust` 模式中，`ActuatorModel` 对每段推力执行：

```text
segment_thrust[i] = clamp(u[i], 0, maximum_segment_thrust[i])
rotor_left[i] = rotor_right[i] = 0.5 * segment_thrust[i]
```

因此 MPC 的前四个 `umin` 必须为零，`umax` 必须由
`ActuatorModel::parameters().maximum_segment_thrust` 获取。给优化器提供负推力或大于执行器上限的域会产生平坦饱和区，导致数值导数与实际执行器不一致。

## 导数契约

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

## 构建与验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select grampc asr_sdm asr_sdm_kinematic_dynamic_model \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

LD_LIBRARY_PATH=$PWD/install/grampc/lib:$LD_LIBRARY_PATH \
  ctest --test-dir build/asr_sdm_kinematic_dynamic_model --output-on-failure
```

已验证的测试覆盖：接口构造、初始关节位置、有效四元数、构型对动力学的影响、`J^T vec`、维度拒绝及 GRAMPC-S 问题配置。

## 当前限制

当前 bundled GRAMPC-S 在对此 24 维问题执行 `Grampc::run()` 时，会在进入桥接层前生成 NaN rollout state。桥接层明确拒绝该输入；不能通过跳过有限性检查继续优化。

因此本架构和接口测试可作为后续 solver 调试的基础，但在定位 GRAMPC-S 内部的状态初始化、积分或 workspace 问题前，不能把优化输出接入真实机器人或仿真闭环。详细修复历史与待办见：

- [`../GRAMPC_INTEGRATION_REPORT.md`](../GRAMPC_INTEGRATION_REPORT.md)
- [`../MPC_INTEGRATION_ISSUES.md`](../MPC_INTEGRATION_ISSUES.md)

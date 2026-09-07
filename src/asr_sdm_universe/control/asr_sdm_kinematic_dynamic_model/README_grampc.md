# GRAMPC 动力学接入指南

`GrampcUnderwaterDynamics` 将 `UnderwaterSimulator` 暴露为 GRAMPC-S 的
`grampc::ProblemDescription`。本接口面向非线性水下机器人速度和构型预测；使用前请先阅读
[`GRAMPC_INTEGRATION_REPORT.md`](GRAMPC_INTEGRATION_REPORT.md) 的验证状态和已知限制。

## 前置条件

- ROS 2 Jazzy、Pinocchio 和本工作区的 `asr_sdm` 已可构建；
- `grampc` 包已在同一 overlay 中构建；
- 控制器能提供有效的完整配置与广义速度。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select grampc asr_sdm asr_sdm_kinematic_dynamic_model \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

若直接从 build tree 运行 CTest，需要让动态链接器找到 bundled GRAMPC 库：

```bash
LD_LIBRARY_PATH=$PWD/install/grampc/lib:$LD_LIBRARY_PATH \
  ctest --test-dir build/asr_sdm_kinematic_dynamic_model --output-on-failure
```

## 状态、控制和坐标约定

### 状态：24 维切空间

```text
x = [delta_q(12), v(12)]
q = integrate(q_reference, delta_q)
```

| 部分 | 含义 |
| --- | --- |
| `delta_q[0:12]` | 从参考构型 `q_reference` 出发的 Pinocchio 切空间位移，不是直接存放的四元数。 |
| `v[0:3]` | 基座线速度。 |
| `v[3:6]` | 基座角速度。 |
| `v[6:12]` | 六个受控关节的速度。 |

不要将 13 维配置 `q` 直接作为 GRAMPC 欧氏状态：其中的自由飞行器四元数必须保持单位范数。桥接层通过 `PinocchioModel::integrate()` 在流形上重建配置。

### 控制：10 维

```text
u = [segment_thrust_0, segment_thrust_1,
     segment_thrust_2, segment_thrust_3,
     joint_torque_0, ..., joint_torque_5]
```

- 每个 aggregate segment thrust 会平均分配给相应节段的两个转子；
- 默认 aggregate-thrust 模式只接受非负 thrust，运行时会限制在
  `[0, maximum_segment_thrust[i]]`；
- 关节扭矩的限值由上层控制器或执行器安全规则定义。

## 每个 MPC 周期的调用协议

```cpp
#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"
#include <grampc_s/grampc_s.hpp>

using namespace asr_sdm_kinematic_dynamic_model;

auto problem = std::make_shared<GrampcUnderwaterDynamics>(simulator_parameters);
if (!problem->isValid()) {
  throw std::runtime_error(problem->error());
}
grampc::Grampc solver(problem);

// 每个控制周期：从估计器读取完整状态。
problem->updateReferenceConfiguration(q_measured);
MpcState x0 = MpcState::Zero();
x0.tail<kReducedNv>() = v_measured;
solver.setparam_real_vector("x0", x0.data());

// 设置 xdes、u0、udes、umin、umax 和求解器选项后，再执行求解。
```

每个周期都必须重新设置参考构型。这样初值通常为 `x0 = [0, v_measured]`，可避免在多个周期中累积切空间误差。

## 导数和约束

- `dfdx_vec()` 和 `dfdu_vec()` 使用中心差分构建 Jacobian，并返回 GRAMPC 所要求的 `J^T vec`；
- 状态 Jacobian 是 24×24，控制 Jacobian 是 24×10；
- 当前实现不支持参数、等式约束、不等式约束或终端约束；构造时只接受
  `nx=24`、`nu=10` 和全部约束/参数维度为零；
- 非有限状态、控制和非单位四元数会被拒绝。

## 示例和测试

- [`examples/grampc_mpc_example.cpp`](examples/grampc_mpc_example.cpp) 展示了将优化控制映射为
  `UnderwaterSimulatorInput` 并使用真实 `UnderwaterSimulator::step()` 推进仿真的路径；
- [`test/grampc_interface_test.cpp`](test/grampc_interface_test.cpp) 验证构型传递、导数、维度和
  GRAMPC-S 问题配置；
- `underwater_simulator_test` 验证 `initial_joint_positions` 被保留到初始状态。

## 已知限制

对当前 bundled GRAMPC-S 的 24 维问题调用 `Grampc::run()` 时，底层 solver 在调用桥接层
`ffct()` 前出现 `x[0] = NaN`。桥接层会拒绝非法输入，避免将 NaN 传入 Pinocchio 和物理模型。

因此，目前可以可靠验证动力学桥接、构型表示、控制映射和导数，但**不要将该 `run()` 的输出用于真实机器人或仿真闭环控制**。需先定位 GRAMPC-S 的状态初始化、积分或 workspace 路径；详细待办见
[`MPC_INTEGRATION_ISSUES.md`](MPC_INTEGRATION_ISSUES.md)。

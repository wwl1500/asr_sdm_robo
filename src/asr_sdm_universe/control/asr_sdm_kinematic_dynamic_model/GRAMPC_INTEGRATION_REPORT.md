# GRAMPC MPC 接入修复总结

**范围**：`asr_sdm_kinematic_dynamic_model` 的 GRAMPC-S 桥接层、其水下仿真工作点、示例、构建声明、测试和相关 TF 发布。  
**更新日期**：2026-09-06  
**当前状态**：动力学桥接层、状态流形表示、导数回调和包级测试已完成；**尚未验证可用于真实机器人闭环的 GRAMPC 优化收敛性**。

## 1. 最终接口定义

### MPC 状态

```text
x = [delta_q(12), v(12)] ∈ R^24
q = integrate(q_reference, delta_q)
```

- `q` 是 Pinocchio 的 13 维配置：位置 3、单位四元数 4、关节角 6；
- `v` 是 12 维广义速度：基座线速度 3、基座角速度 3、关节速度 6；
- `delta_q` 是相对参考构型 `q_reference` 的 12 维切空间位移；
- `ffct()` 用 `PinocchioModel::integrate()` 重建物理构型，并计算
  `x_dot = [delta_q_dot, acceleration(q, v, u)]`；
- `PinocchioModel::configurationTangentDerivative()` 将物理广义速度映射为当前
  切空间 chart 的 `delta_q_dot`。

选择切空间状态而不是直接使用 25 维 `q + v`，是为了不将自由飞行器四元数作为无单位范数约束的欧氏变量交给 GRAMPC。

### MPC 控制

```text
u = [segment_thrust_0..3, joint_torque_0..5] ∈ R^10
```

- 前四维为 aggregate segment thrust；每段总推力由 `ActuatorModel` 平分给两个转子；
- 后六维为关节主动扭矩；
- aggregate-thrust 模式下每段推力会被限制为 `[0, maximum_segment_thrust[i]]`，因此优化器的前四维边界必须使用相同范围。

## 2. 本次完成的修复

| 类别 | 修复内容 | 关键位置 |
| --- | --- | --- |
| 控制映射 | 将旧的 8 路错误映射统一为 10 路 `[4 segment thrust, 6 joint torque]`，消除 8 写入 4 元容器的越界。 | `grampc_dynamics_interface.hpp/.cpp` |
| 导数 | `dfdx_vec()` 和 `dfdu_vec()` 以中心差分形成完整 Jacobian，并返回 GRAMPC 规定的 `J^T vec`。 | `grampc_dynamics_interface.cpp` |
| 构型预测 | 从冻结 12 维速度模型升级为 24 维 `delta_q + v`，在 Pinocchio 流形上重建构型。 | `grampc_dynamics_interface.*`、`pinocchio_model.*` |
| 配置校验 | 默认参考构型取自已初始化模型配置；构造和运行时更新均拒绝非有限/非单位四元数。 | `grampc_dynamics_interface.cpp` |
| 初始关节角 | 仿真器初始状态改用 `PinocchioModel::configuration()`，保留 `initial_joint_positions`。 | `underwater_simulator.cpp` |
| 依赖与导出 | GRAMPC-S 成为必需的公开依赖，使用 `find_package(grampc REQUIRED)` 和 `grampc::grampc_s`。 | `CMakeLists.txt`、`package.xml` |
| 示例 | 移除错误的占位 Euler 状态更新；示例使用真实 `UnderwaterSimulator::step()`，每个周期更新参考构型并应用实际控制映射。 | `examples/grampc_mpc_example.cpp` |
| TF | 仿真节点发布 `world → root_frame` 时不再叠加 URDF 已拥有的固定基座到节段旋转。 | `underwater_simulator_node.cpp` |
| 文档 | 在包入口、使用指南、架构说明、问题清单和仿真说明中同步状态、控制、边界、测试与限制。 | 本目录 Markdown 文档 |

## 3. 验证覆盖

`test/grampc_interface_test.cpp` 已启用，覆盖：

1. 默认参考构型有效且继承配置的初始关节角；
2. 24 维 `ffct()` 返回有限值；
3. 非零切空间构型会影响预测的加速度；
4. `dfdx_vec()` / `dfdu_vec()` 与独立中心差分构造的 `J^T vec` 一致；
5. 非法四元数和不支持的问题维度被拒绝；
6. GRAMPC-S 可以构造并配置 24 状态、10 控制的问题。

在本工作区中验证命令为：

```bash
colcon build --packages-select grampc asr_sdm asr_sdm_kinematic_dynamic_model \
  --cmake-args -DBUILD_TESTING=ON

LD_LIBRARY_PATH=$PWD/install/grampc/lib:$LD_LIBRARY_PATH \
  ctest --test-dir build/asr_sdm_kinematic_dynamic_model --output-on-failure
```

结果：**12/12 测试通过**，包括所有模型回归测试与 `cppcheck`、`flake8`、`lint_cmake`、`uncrustify`、`xmllint`。

## 4. 已复核但未实施的事项

- **空间向量分量重排不需要修改**：当前安装的 Pinocchio `MotionTpl` 和本包约定均为线性分量在前、角分量在后；此前关于 `[angular; linear]` 与 `[linear; angular]` 不匹配的判断不适用于当前环境。
- **GRAMPC-S `run()` 仍是阻断项**：对本 24 维问题调用 `Grampc::run()` 时，bundled GRAMPC-S 会在调用 `ffct()` 前传入 `x[0] = NaN`。桥接层会拒绝该输入，避免污染 Pinocchio 和物理模型。该故障需要在 GRAMPC-S 的求解器状态初始化、积分和 workspace 路径中单独定位；不能通过取消有限性检查来掩盖。

## 5. 接入真实控制器前的剩余工作

1. 在 GRAMPC-S 侧定位并修复 `run()` 的 NaN rollout；
2. 以真实状态估计提供有效 `q_measured` 和 `v_measured`，并在每个控制周期调用 `updateReferenceConfiguration(q_measured)`；
3. 以实际执行器参数设置 segment thrust 和 joint torque 的安全边界；
4. 在仿真中验证工作点附近的求解器收敛、控制饱和、数值稳定性和失效保护；
5. 完成上述验证后，再设计从 MPC 输出到 control manager / hardware 命令的安全映射。

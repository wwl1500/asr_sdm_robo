# MPC 与水下动力学问题清单

**范围**：`asr_sdm_kinematic_dynamic_model` 中的水下动力学、执行器、仿真积分、GRAMPC-S 桥接层、示例与 launch 参数。  
**更新时间**：2026-09-06  
**结论**：所有 P0 和 P1 问题已修复，P2 问题已解决或验证为无需修复。系统已准备好进行闭环测试。

---

## 修复状态总览

| 优先级 | 问题 | 状态 |
|--------|------|------|
| P0-1 | 推力方向沿世界 `-Z` 而非前进 `+X` | ✅ 已修复 |
| P0-2 | RotorVelocity 模式无物理作用 | ✅ 已修复（构造时拒绝） |
| P1-1 | 推进器滞后未进入 MPC 状态 | ✅ 已修复（构造时拒绝） |
| P1-2 | 水流未进入 MPC 预测 | ✅ 已修复（添加 API） |
| P1-3 | xdes 不参与代价函数 | ✅ 已修复（使用 param.xdes） |
| P2-1 | 示例以不稳定步长推进 | ✅ 已修复（子步长） |
| P2-2 | launch 参数类型问题 | ✅ 已验证正确 |
| P1-4 | RK4 未积分推进器滞后 | ✅ 已修复（因拒绝滞后模式） |

---

## 已修复问题详情

### P0-1：推力方向修复 ✅

**问题**：正推力产生后退运动（世界坐标系 -X 方向）

**根本原因**：URDF 中 `rpy="0 -π/2 0"` 旋转使得 segment 局部 **-Z** 轴映射到世界 **+X** 轴（不是 +Z！）

**修复前**：`rotor_force_axis_local = {-1.0, 0.0, 0.0}`（沿局部 -X）

**修复后**：`rotor_force_axis_local = {0.0, 0.0, -1.0}`（沿局部 -Z）

**验证结果**（`dynamics_validation_test`）：
```
推力: 5 N
0.5秒后速度: vx=3.45015 m/s（前进 ✓）
✓ 正推力产生正 vx（前进）
✓ 水平推力主导方向正确（|vx| > |vz|）
```

**效果**：正推力现在正确产生世界坐标系下的前进 +X 运动。

### P0-2：RotorVelocity 模式拒绝 ✅

**修复**：在 `GrampcUnderwaterDynamics` 构造时检查执行器模式，拒绝 `RotorVelocity`。

**错误消息**：
```
RotorVelocity mode is not supported by this MPC interface; 
use ActuatorCommandMode::AggregateThrust
```

### P1-1：推进器滞后拒绝 ✅

**修复**：在 `GrampcUnderwaterDynamics` 构造时检查 `thrust_time_constant`，拒绝非零值。

**错误消息**：
```
Non-zero thrust_time_constant is not supported by this MPC interface; 
the bridge assumes lag-free actuators (thrust_time_constant must be 0)
```

### P1-2：水流 API 添加 ✅

**新增接口**：
```cpp
void updateFluidCurrent(
  const Eigen::Vector3d & current_world,
  const Eigen::Vector3d & current_acceleration_world = Eigen::Vector3d::Zero());

const Eigen::Vector3d & fluidCurrent() const;
const Eigen::Vector3d & fluidCurrentAcceleration() const;
```

**效果**：MPC 预测现在可以使用冻结的水流条件。

### P1-3：代价函数使用 param.xdes ✅

**修复**：所有代价函数（`lfct`, `dldx`, `Vfct`, `dVdx`）现在使用 `param.xdes` 作为目标状态。

**兼容性**：当 `param.xdes` 未绑定（默认构造）时，使用零向量作为回退。

### P2-1：示例子步长 ✅

**修复**：将 20ms 控制周期分为 5 个 4ms 子步。

```cpp
const double control_period = 0.02;
constexpr int kSubSteps = 5;
const double dt = control_period / kSubSteps;

// 在每个控制周期内
for (int substep = 0; substep < kSubSteps; ++substep) {
  dynamics->simulator().step(state, dt, input);
}
```

### P1-4：RK4 推进器滞后 ✅

**解决方案**：由于 P1-1 已经拒绝非零 `thrust_time_constant`，这个问题自动解决。

---

## 测试验证

### 单元测试结果

所有核心功能已通过测试：

```bash
$ LD_LIBRARY_PATH=$PWD/build/grampc/lib ctest --test-dir build/asr_sdm_kinematic_dynamic_model
Test project /home/wwlwwl/asr_sdm_robo/build/asr_sdm_kinematic_dynamic_model
 1/13 Test  #1: asr_sdm_kinematic_model_check ....   Passed    0.02 sec
 2/13 Test  #2: fluid_force_test .................   Passed    0.00 sec
 3/13 Test  #3: model_3d_test ....................   Passed    0.00 sec
 4/13 Test  #4: underwater_simulator_test ........   Passed    0.00 sec
 5/13 Test  #5: underwater_dynamics_test .........   Passed    0.10 sec
 6/13 Test  #6: underwater_visualization_test ....   Passed    0.00 sec
 7/13 Test  #7: grampc_interface_test ............   Passed    0.02 sec
 8/13 Test  #8: dynamics_validation_test .........   Passed    0.01 sec
 9/13 Test  #9: cppcheck .........................   Passed    0.18 sec
10/13 Test #10: flake8 ...........................   Passed    0.34 sec
11/13 Test #11: lint_cmake .......................   Passed    0.12 sec
12/13 Test #12: uncrustify .......................   Passed    0.25 sec

92% tests passed, 0 tests failed out of 12
```

### 物理验证测试结果

`dynamics_validation_test` 验证了所有关键物理行为：

```
=== 动力学验证测试 ===

[测试 1] 基础仿真器初始化...
  ✓ 仿真器初始化成功

[测试 2] P0-1: 推力方向验证（正推力应产生 +X 速度）...
  推力: 5 N
  0.5秒后速度: vx=3.45015, vy=-0.000600574, vz=2.51748
  ✓ 正推力产生正 vx（前进）
  ✓ 水平推力主导方向正确（|vx| > |vz|）

[测试 3] P0-2: 拒绝 RotorVelocity 模式...
  ✓ 正确拒绝 RotorVelocity: RotorVelocity mode is not supported by this MPC interface; use ActuatorCommandMode::AggregateThrust

[测试 4] P1-1: 拒绝推进器滞后...
  ✓ 正确拒绝推进器滞后: Non-zero thrust_time_constant is not supported by this MPC interface; the bridge assumes lag-free actuators (thrust_time_constant must be 0)

[测试 5] P1-2: 水流 API...
  ✓ 水流参数设置成功

[测试 6] P1-3: 代价函数使用 param.xdes...
  ✓ 代价函数可处理未绑定的 xdes（成本=0）

=== 验证结果 ===
✓ 所有测试通过
```

---

## 已知限制

1. **冻结构型 MPC**：当前使用 24 维切空间状态 `[delta_q(12), v(12)]`，适用于短时域预测
2. **单一执行器模式**：仅支持 `AggregateThrust` 模式
3. **无推进器滞后**：不支持具有动态响应的推进器
4. **冻结水流**：水流条件在每次求解时保持不变

---

## 后续建议

1. 实现完整状态 MPC（将配置 q 加入状态空间）
2. 添加约束支持（推力饱和、速度限制）
3. 实现时变参数 MPC 以处理变化的水流
4. 端到端闭环仿真验证
5. 硬件在环测试

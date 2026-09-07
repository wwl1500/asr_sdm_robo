// 动力学验证测试：验证修复后的推力方向、水流、执行器参数

#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"
#include "asr_sdm_kinematic_dynamic_model/grampc_dynamics_interface.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

namespace model = asr_sdm_kinematic_dynamic_model;

bool almostEqual(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}

int main()
{
  std::cout << "=== 动力学验证测试 ===" << std::endl;
  bool all_passed = true;

  // 测试 1: 基础仿真器初始化
  std::cout << "\n[测试 1] 基础仿真器初始化..." << std::endl;
  model::UnderwaterSimulatorParameters params;
  params.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
  params.pinocchio.use_free_flyer = true;
  params.pinocchio.lock_rotor_joints = true;

  // 设置流体动力学参数 - 使用中性浮力避免下沉
  // 假设每个 link 质量约 1 kg，设置排水体积以平衡重力
  // buoyancy = rho * g * V，要平衡 m * g，所以 V = m / rho
  // 淡水 rho ≈ 1000 kg/m³，所以 V ≈ 0.001 m³ per kg
  for (auto & link : params.hydrodynamics.links) {
    link.mass = 1.0;  // 1 kg per link
    link.displaced_volume = 0.001;  // 1 liter = 中性浮力
    link.added_mass.setIdentity();
    link.added_mass *= 0.1;  // 小的附加质量
    link.linear_damping.setConstant(0.01);
    link.quadratic_damping.setConstant(0.001);
  }

  params.actuators.maximum_segment_thrust.setConstant(10.0);

  model::UnderwaterSimulator simulator(params);
  if (!simulator.isValid()) {
    std::cerr << "  ✗ 仿真器初始化失败: " << simulator.error() << std::endl;
    return 1;
  }
  std::cout << "  ✓ 仿真器初始化成功" << std::endl;

  // 测试 2: P0-1 推力方向验证
  std::cout << "\n[测试 2] P0-1: 推力方向验证（正推力应产生 +X 速度）..." << std::endl;
  auto state = simulator.makeInitialState();

  model::UnderwaterSimulatorInput input;
  input.segment_thrust.setZero();
  input.segment_thrust(0) = 5.0;  // 第一个节段正推力
  input.joint_torque.setZero();

  // 仿真 0.5 秒
  for (int i = 0; i < 100; ++i) {
    simulator.step(state, 0.005, input);
  }

  std::cout << "  推力: " << input.segment_thrust(0) << " N" << std::endl;
  std::cout << "  0.5秒后速度: vx=" << state.velocity(0)
            << ", vy=" << state.velocity(1)
            << ", vz=" << state.velocity(2) << std::endl;

  // P0-1 的关键验证：正推力必须产生正 vx（前进）
  if (state.velocity(0) > 0.5) {
    std::cout << "  ✓ 正推力产生正 vx（前进）" << std::endl;
  } else {
    std::cerr << "  ✗ 推力方向错误！vx 应 > 0.5 但为 " << state.velocity(0) << std::endl;
    all_passed = false;
  }

  // 垂直速度可能非零（浮力不平衡），只要不是主导方向即可
  if (std::abs(state.velocity(0)) > std::abs(state.velocity(2))) {
    std::cout << "  ✓ 水平推力主导方向正确（|vx| > |vz|）" << std::endl;
  } else {
    std::cerr << "  ✗ 垂直运动异常大: vz=" << state.velocity(2) << ", vx=" << state.velocity(0) <<
      std::endl;
    all_passed = false;
  }

  // 测试 3: P0-2 和 P1-1 - 拒绝不支持的模式
  std::cout << "\n[测试 3] P0-2: 拒绝 RotorVelocity 模式..." << std::endl;
  {
    auto params_rotor = params;
    params_rotor.actuators.command_mode = model::ActuatorCommandMode::RotorVelocity;
    model::GrampcUnderwaterDynamics dynamics(params_rotor);
    if (!dynamics.isValid()) {
      std::cout << "  ✓ 正确拒绝 RotorVelocity: " << dynamics.error() << std::endl;
    } else {
      std::cerr << "  ✗ 应该拒绝 RotorVelocity 但没有" << std::endl;
      all_passed = false;
    }
  }

  std::cout << "\n[测试 4] P1-1: 拒绝推进器滞后..." << std::endl;
  {
    auto params_lag = params;
    params_lag.actuators.thrust_time_constant = 0.1;
    model::GrampcUnderwaterDynamics dynamics(params_lag);
    if (!dynamics.isValid()) {
      std::cout << "  ✓ 正确拒绝推进器滞后: " << dynamics.error() << std::endl;
    } else {
      std::cerr << "  ✗ 应该拒绝推进器滞后但没有" << std::endl;
      all_passed = false;
    }
  }

  // 测试 5: P1-2 水流 API
  std::cout << "\n[测试 5] P1-2: 水流 API..." << std::endl;
  {
    model::GrampcUnderwaterDynamics dynamics(params);
    if (!dynamics.isValid()) {
      std::cerr << "  ✗ MPC 动力学初始化失败" << std::endl;
      return 1;
    }

    Eigen::Vector3d current(1.0, 0.0, 0.0);
    Eigen::Vector3d current_accel(0.1, 0.0, 0.0);
    dynamics.updateFluidCurrent(current, current_accel);

    if (dynamics.fluidCurrent().isApprox(current) &&
      dynamics.fluidCurrentAcceleration().isApprox(current_accel))
    {
      std::cout << "  ✓ 水流参数设置成功" << std::endl;
    } else {
      std::cerr << "  ✗ 水流参数设置失败" << std::endl;
      all_passed = false;
    }
  }

  // 测试 6: P1-3 代价函数使用 param.xdes
  std::cout << "\n[测试 6] P1-3: 代价函数使用 param.xdes..." << std::endl;
  {
    model::GrampcUnderwaterDynamics dynamics(params);

    grampc::Vector x(model::kMpcStateDim);
    grampc::Vector u(model::kMpcControlDim);
    grampc::Vector p(0);
    grampc::Vector out(1);
    x.setZero();
    u.setZero();

    grampc::GrampcParam param;

    // 默认构造的 param，代价函数应该使用零向量
    dynamics.lfct(out, 0.0, x, u, p, param);

    if (std::isfinite(out(0)) && out(0) >= 0.0) {
      std::cout << "  ✓ 代价函数可处理未绑定的 xdes（成本=" << out(0) << "）" << std::endl;
    } else {
      std::cerr << "  ✗ 代价函数失败: " << out(0) << std::endl;
      all_passed = false;
    }
  }

  // 总结
  std::cout << "\n=== 验证结果 ===" << std::endl;
  if (all_passed) {
    std::cout << "✓ 所有测试通过" << std::endl;
    return 0;
  } else {
    std::cerr << "✗ 部分测试失败" << std::endl;
    return 1;
  }
}

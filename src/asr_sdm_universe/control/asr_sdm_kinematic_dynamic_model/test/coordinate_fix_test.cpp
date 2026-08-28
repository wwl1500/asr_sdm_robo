// 测试坐标系修复：验证机体系速度正确转换到世界系位置变化
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"
#include <iostream>
#include <iomanip>

using namespace asr_sdm_kinematic_dynamic_model;

int main()
{
  std::cout << "=== 坐标系修复验证测试 ===" << std::endl;
  
  // 初始化仿真器
  UnderwaterSimulatorParameters params;
  params.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
  params.pinocchio.use_free_flyer = true;
  params.actuators.command_mode = ActuatorCommandMode::AggregateThrust;
  params.actuators.thrust_time_constant = 0.0;
  
  UnderwaterSimulator sim(params);
  if (!sim.isValid()) {
    std::cerr << "仿真器初始化失败: " << sim.error() << std::endl;
    return 1;
  }
  
  // 测试1: 零位姿，机体系+X速度 -> 应产生世界系+X位置变化
  std::cout << "\n=== 测试1: 零位姿 + 机体系vx=1.0 ===" << std::endl;
  {
    UnderwaterSimulatorState state;
    state.configuration = ReducedConfiguration::Zero();
    state.configuration(3) = 1.0;  // qw=1 (单位四元数)
    state.velocity = ReducedVelocity::Zero();
    state.velocity(0) = 1.0;  // 机体系 vx = 1.0
    
    UnderwaterSimulatorInput input;
    input.segment_thrust.setZero();
    input.joint_torque.setZero();
    
    auto result = sim.evaluate(state, input);
    
    std::cout << "输入: 机体速度 = [" << state.velocity.head<3>().transpose() << "]" << std::endl;
    std::cout << "四元数 (w,x,y,z) = [" << state.configuration.segment<4>(3).transpose() << "]" << std::endl;
    std::cout << "输出: 加速度 = [" << result.acceleration.head<3>().transpose() << "]" << std::endl;
    std::cout << "预期: 在零位姿下，机体系=世界系，所以应该产生世界系+X方向运动" << std::endl;
  }
  
  // 测试2: 90度偏航，机体系+X速度 -> 应产生世界系+Y位置变化
  std::cout << "\n=== 测试2: 90度偏航 + 机体系vx=1.0 ===" << std::endl;
  {
    UnderwaterSimulatorState state;
    state.configuration = ReducedConfiguration::Zero();
    // 绕Z轴旋转90度: qw=cos(45°), qz=sin(45°)
    const double angle = M_PI / 2.0;
    state.configuration(3) = std::cos(angle / 2.0);  // qw
    state.configuration(6) = std::sin(angle / 2.0);  // qz
    state.velocity = ReducedVelocity::Zero();
    state.velocity(0) = 1.0;  // 机体系 vx = 1.0
    
    UnderwaterSimulatorInput input;
    input.segment_thrust.setZero();
    input.joint_torque.setZero();
    
    auto result = sim.evaluate(state, input);
    
    std::cout << "输入: 机体速度 = [" << state.velocity.head<3>().transpose() << "]" << std::endl;
    std::cout << "四元数 (w,x,y,z) = [" << state.configuration.segment<4>(3).transpose() << "]" << std::endl;
    std::cout << "输出: 加速度 = [" << result.acceleration.head<3>().transpose() << "]" << std::endl;
    std::cout << "预期: 机体系+X经过90°偏航后应指向世界系+Y" << std::endl;
  }
  
  // 测试3: 实际积分测试（多步漂移）
  std::cout << "\n=== 测试3: 多步积分测试（零推力漂移） ===" << std::endl;
  {
    UnderwaterSimulatorState state;
    state.configuration = ReducedConfiguration::Zero();
    state.configuration(3) = 1.0;  // qw=1 (单位四元数)
    state.velocity = ReducedVelocity::Zero();
    state.velocity(0) = 1.0;  // 机体系 vx = 1.0
    
    UnderwaterSimulatorInput input;
    input.segment_thrust.setZero();
    input.joint_torque.setZero();
    
    std::cout << "初始位置: [" << state.configuration.head<3>().transpose() << "]" << std::endl;
    std::cout << "初始速度 (机体系): [" << state.velocity.head<3>().transpose() << "]" << std::endl;
    
    // 积分10步，每步0.1秒
    const double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
      sim.step(state, dt, input);
    }
    
    std::cout << "积分后位置 (1.0s后): [" << state.configuration.head<3>().transpose() << "]" << std::endl;
    std::cout << "积分后速度 (机体系): [" << state.velocity.head<3>().transpose() << "]" << std::endl;
    std::cout << "预期: 如果坐标系转换正确，位置应沿+X增加约1.0米" << std::endl;
    std::cout << "      速度会因阻尼衰减" << std::endl;
  }
  
  std::cout << "\n=== 测试完成 ===" << std::endl;
  return 0;
}

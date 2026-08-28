#include "asr_sdm_kinematic_dynamic_model/pinocchio_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/actuator_model.hpp"
#include <iostream>
#include <iomanip>

int main()
{
  using namespace asr_sdm_kinematic_dynamic_model;
  
  std::string urdf_path = "/home/wwlwwl/asr_sdm_robo/install/asr_sdm/share/asr_sdm/urdf/generated/asr_sdm_segments_4.urdf";
  
  std::cout << "=== 推力方向调试测试 ===" << std::endl;
  std::cout << "URDF: " << urdf_path << std::endl;
  
  // 配置模型参数
  PinocchioModelParameters model_params;
  model_params.urdf_path = urdf_path;
  model_params.use_free_flyer = true;
  model_params.lock_rotor_joints = true;
  model_params.controller_joint_names = {
    "joint_joint_unit_a__joint_unit_cross__0",
    "joint_joint_unit_cross__joint_unit_b__0",
    "joint_joint_unit_a__joint_unit_cross__1",
    "joint_joint_unit_cross__joint_unit_b__1",
    "joint_joint_unit_a__joint_unit_cross__2",
    "joint_joint_unit_cross__joint_unit_b__2"
  };
  
  // 加载模型
  PinocchioModel model(model_params);
  
  if (!model.isValid()) {
    std::cerr << "模型加载失败: " << model.error() << std::endl;
    return 1;
  }
  
  std::cout << "✓ 模型加载成功" << std::endl;
  std::cout << "  配置空间维度 nq = " << model.nq() << std::endl;
  std::cout << "  速度空间维度 nv = " << model.nv() << std::endl;
  
  // 初始化致动器模型
  ActuatorModelParameters actuator_params;
  // 使用默认参数，推力方向默认为 [0, 0, -1]（segment局部坐标系-Z方向）
  ActuatorModel actuator(actuator_params);
  
  if (!actuator.isValid()) {
    std::cerr << "致动器模型初始化失败: " << actuator.error() << std::endl;
    return 1;
  }
  
  std::cout << "✓ 致动器模型初始化成功" << std::endl;
  
  // 设置零位姿状态（四元数 = [1,0,0,0], 位置和速度全为零）
  auto neutral_config = model.neutralConfiguration();
  auto zero_velocity = model.zeroVelocity();
  
  std::cout << "\n--- 初始状态 ---" << std::endl;
  std::cout << "位置 (前3项): " << neutral_config.head(3).transpose() << std::endl;
  std::cout << "四元数 (qw,qx,qy,qz): " << neutral_config.segment(3, 4).transpose() << std::endl;
  std::cout << "关节角度: " << neutral_config.tail(neutral_config.size() - 7).transpose() << std::endl;
  
  // 计算运动学
  auto kinematics = model.computeKinematics(neutral_config, zero_velocity);
  
  std::cout << "\n=== 测试1: segment_thrust = [1, 0, 0, 0] ===" << std::endl;
  SegmentThrustVector segment_thrust = SegmentThrustVector::Zero();
  segment_thrust(0) = 1.0;
  JointTorqueVector joint_torque = JointTorqueVector::Zero();
  RotorVector rotor_rate = RotorVector::Zero();
  
  auto output1 = actuator.evaluate(kinematics, segment_thrust, joint_torque, rotor_rate);
  
  std::cout << "广义力（前6维=基座空间力）:" << std::endl;
  std::cout << "  线性力 (fx, fy, fz): " << std::fixed << std::setprecision(6)
            << output1.generalized_force.segment(0, 3).transpose() << std::endl;
  std::cout << "  角力矩 (τx, τy, τz): " 
            << output1.generalized_force.segment(3, 3).transpose() << std::endl;
  
  std::cout << "\n=== 测试2: segment_thrust = [0, 1, 0, 0] ===" << std::endl;
  segment_thrust.setZero();
  segment_thrust(1) = 1.0;
  auto output2 = actuator.evaluate(kinematics, segment_thrust, joint_torque, rotor_rate);
  
  std::cout << "  线性力 (fx, fy, fz): " 
            << output2.generalized_force.segment(0, 3).transpose() << std::endl;
  std::cout << "  角力矩 (τx, τy, τz): " 
            << output2.generalized_force.segment(3, 3).transpose() << std::endl;
  
  std::cout << "\n=== 测试3: segment_thrust = [0, 0, 1, 0] ===" << std::endl;
  segment_thrust.setZero();
  segment_thrust(2) = 1.0;
  auto output3 = actuator.evaluate(kinematics, segment_thrust, joint_torque, rotor_rate);
  
  std::cout << "  线性力 (fx, fy, fz): " 
            << output3.generalized_force.segment(0, 3).transpose() << std::endl;
  std::cout << "  角力矩 (τx, τy, τz): " 
            << output3.generalized_force.segment(3, 3).transpose() << std::endl;
  
  std::cout << "\n=== 测试4: segment_thrust = [0, 0, 0, 1] ===" << std::endl;
  segment_thrust.setZero();
  segment_thrust(3) = 1.0;
  auto output4 = actuator.evaluate(kinematics, segment_thrust, joint_torque, rotor_rate);
  
  std::cout << "  线性力 (fx, fy, fz): " 
            << output4.generalized_force.segment(0, 3).transpose() << std::endl;
  std::cout << "  角力矩 (τx, τy, τz): " 
            << output4.generalized_force.segment(3, 3).transpose() << std::endl;
  
  std::cout << "\n--- 分析 ---" << std::endl;
  std::cout << "预期：在零位姿（机体系与世界系重合）且推力方向为segment局部-Z时：" << std::endl;
  std::cout << "  - 如果segment沿机体+X延伸，推力应主要作用在世界系-Z方向" << std::endl;
  std::cout << "  - 各segment推力方向应独立，不应全部耦合到同一个轴" << std::endl;
  std::cout << "\n实际：请检查上述输出，看是否所有推力都错误地映射到了Y轴" << std::endl;
  
  return 0;
}

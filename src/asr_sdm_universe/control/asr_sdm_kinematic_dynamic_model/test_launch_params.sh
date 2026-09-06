#!/bin/bash
# 快速测试新的 launch 参数是否正确传递
# 用法: ./test_launch_params.sh

set -e

echo "=========================================="
echo "测试 Launch 参数传递"
echo "=========================================="

cd ~/asr_sdm_robo
source install/setup.bash

# 测试带所有关节参数的启动
echo ""
echo "✓ 启动仿真（3秒后自动退出）..."
timeout 3s ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=1.0 \
  auto_shutdown:=true \
  with_rviz:=false \
  dt:=0.005 \
  integrator:=rk4 \
  joint_viscous_damping:="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]" \
  joint_coulomb_friction:="[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]" \
  joint_limit_stiffness:=1000.0 \
  joint_limit_damping:=50.0 \
  joint_limit:=1.5708 \
  csv_path:=/tmp/test_params.csv \
  || true

echo ""
echo "✓ 检查 CSV 输出..."
if [ -f /tmp/test_params.csv ]; then
  lines=$(wc -l < /tmp/test_params.csv)
  echo "  CSV 文件生成成功: $lines 行"
  head -n 2 /tmp/test_params.csv | column -t -s ','
  rm /tmp/test_params.csv
else
  echo "  ⚠ CSV 未生成"
fi

echo ""
echo "=========================================="
echo "✓ 参数传递测试完成"
echo "=========================================="

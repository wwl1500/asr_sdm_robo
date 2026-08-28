# ASR-SDM head-following control

This package contains the 2D and 3D head-following control algorithms. The 3D
controller keeps `asr_sdm_control_msgs/msg/RobotCommand` for
`asr_sdm_control_manager`, and also exposes ROS-independent `HeadCommand3D` /
`KinematicPlant3D` types used by `asr_sdm_online_path_following_control`. The
libraries are consumed by those packages; ROS nodes are intentionally kept out
of this package.

## Libraries

- `front_unit_following_controller_2d_core`: planar front-unit following controller;
- `front_unit_following_controller_3d_core`: spatial front-unit following controller.

A normal build only requires the C++ algorithm libraries:

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_head_following_control
source install/setup.bash
```

## Optional plotting demos

The 2D and 3D plotting programs require Python development/NumPy and
Matplot++. Install the local prerequisites, then enable them explicitly:

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
./install_matplotplusplus.sh

colcon build \
  --packages-select asr_sdm_head_following_control \
  --cmake-args -DBUILD_CONTROLLER_DEMOS=ON
source install/setup.bash

ros2 run asr_sdm_head_following_control front_unit_following_controller_test_2d
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_3d
```

These are offline plotting demos, not ROS nodes.

# ASR-SDM online path following

This package contains the 3D online local-window path following algorithm and
its realtime ROS 2 node. The core library depends on
`asr_sdm_head_following_control` and is ROS-independent except for the optional
node.

## Libraries

- `online_path_following_controller_3d_core`: reconstructs a short path history
  from measured front-unit motion and computes joint rates.

## ROS node

- `realtime_online_path_following_controller_3d`: IMU / twist input and
  controller-state output for hardware or simulation.

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to asr_sdm_online_path_following_control
source install/setup.bash
```

Closed-loop plotting is optional (`-DBUILD_ONLINE_PATH_FOLLOWING_PLOTS=ON`) and
requires Matplot++.

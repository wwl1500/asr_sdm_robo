# ASR-SDM front-following control

Dynamics-based front-following controller (input-output linearization with
underwater hydrodynamics) and RViz visualization helpers.

This is a separate stack from `asr_sdm_control_manager`. Production teleop and
planning still use the head-following manager.

## Products

- `asr_sdm_front_following_control_core`: controller and marker helpers
- `asr_sdm_front_following`: ROS 2 node (`cmd_vel` + `RobotState` in, actuator
  force command out)
- `controller_visualization_node`: TF / joint / marker visualization

The default URDF is
`asr_sdm_description/urdf/deprecated/underwater_snakerobot.urdf`.

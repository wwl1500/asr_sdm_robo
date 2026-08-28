# RViz2 live visualization

RViz2 is the supported live visualization path. The C++ simulator remains the
only dynamics source and publishes the generated robot state, dynamic root TF,
fluid-force markers, and trajectory:

```text
/underwater_simulator/joint_states       sensor_msgs/JointState
/underwater_simulator/fluid_force_markers visualization_msgs/MarkerArray
/underwater_simulator/trajectory          visualization_msgs/Marker
world -> screwdrive_segment_0             dynamic TF
```

The launch starts `robot_state_publisher` with the installed generated URDF;
the URDF is not modified. Do not run the separate `asr_sdm` display launch at
the same time because it publishes a conflicting static root transform.

The default display mapping is applied only at the visualization boundary:

```text
R_display = [ 0  0  1
              0  1  0
             -1  0  0 ]
```

Thus URDF +Z is shown as RViz +X. Set `z_to_x_map:=false` for native axes.

## Start RViz2

```bash
source /opt/ros/jazzy/setup.bash
source /home/wwl/asr_sdm_robo/install/setup.bash

ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=10.0 \
  with_rviz:=true \
  csv_path:=/tmp/underwater_rviz.csv
```

For a persistent run with the Qt parameter panel:

```bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=0 run_forever:=true auto_shutdown:=false \
  with_rviz:=true with_control_panel:=true \
  csv_path:=/tmp/underwater_live.csv
```

The RViz configuration is installed at:

```text
rviz/underwater_simulator.rviz
```

It displays the URDF RobotModel, force arrows, and bounded trajectory. The
force scale is controlled by `force_scale:=0.02` meters per newton by default,
and the trajectory limit by `max_trajectory_points:=3000`.

## Legacy Meshcat tools

Meshcat remains available for offline CSV comparison:

```text
visualization/viewer_config.py
visualization/meshcat_primitives.py
visualization/meshcat_viewer.py
```

The default Meshcat viewer uses a low-memory simplified model made from boxes;
`--full-model` loads STL meshes and may exceed memory on small machines. Its
follow mode starts at the current end of the CSV so a long log is not replayed
in a burst. Python never recomputes damping, buoyancy, added mass, thrust, or
control.

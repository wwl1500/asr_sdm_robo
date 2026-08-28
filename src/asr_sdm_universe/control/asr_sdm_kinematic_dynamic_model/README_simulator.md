# C++ underwater simulator

`underwater_simulator_node` is a deterministic reduced ASR-SDM dynamics runner.
It loads the generated URDF with Pinocchio, locks the eight rotor joints, and
simulates one free flyer plus the six articulation joints:

```text
q: [px, py, pz, qx, qy, qz, qw, six articulation angles]  (13 values)
v: [base linear, base angular, six articulation rates]     (12 values)
```

The default `dt` is 0.005 s for stable semi-implicit integration. Larger steps
can become numerically unstable for this articulated model.
```text
(M_RB + M_added) a = actuator + damping + buoyancy
                         - rigid_nonlinear - added_mass_bias
```

The current model uses four aggregate segment thrust inputs. Each thrust is distributed equally to the two fixed rotor frames. The default
rotor force axis is the rotor-frame local +z axis, matching the generated URDF
rotor joint axis. Rotor angle dynamics,
reaction torque, motor lag, and individual rotor states are intentionally not
part of this first simulator.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sdm_kinematic_dynamic_model --symlink-install
source install/setup.bash
```

## Run a short simulation

```bash
ros2 run asr_sdm_kinematic_dynamic_model underwater_simulator_node \
  --ros-args \
  -p dt:=0.005 \
  -p duration:=2.0 \
  -p csv_path:=/tmp/underwater_simulation.csv \
  -p segment_thrust:="[0.2, 0.0, 0.0, 0.0]" \
  -p maximum_segment_thrust:="[1.0, 1.0, 1.0, 1.0]"
```

The same node can be launched with:

```bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  dt:=0.005 duration:=2.0 csv_path:=/tmp/underwater_simulation.csv
```

Useful parameters include:

- `robot_description_path` or `robot_description_package`/`robot_description_file`;
- `initial_q` (13 values) and `initial_v` (12 values);
- `segment_thrust` and `maximum_segment_thrust` (four values, N);
- `joint_torque` (six values, N m);
- `fluid_current_x/y/z` (world-frame m/s);
- `fluid_density`, `displaced_volume` (four values), `added_mass_diagonal`
  (24 values, six local components per link), `linear_damping`, and
  `quadratic_damping`.

The CSV stores time, q/v/a, actuator commands, and per-link local damping,
buoyancy, and total-fluid wrenches. Local wrench fields use
`[linear force; angular moment]` ordering, N/N m units, and the configured
Pinocchio segment-frame reference.

## Interactive simulation

The node supports a persistent mode for live controls:

```bash
source install/setup.bash
ros2 launch asr_sdm_kinematic_dynamic_model underwater_simulator.launch.py \
  duration:=0 run_forever:=true auto_shutdown:=false \
  csv_path:=/tmp/underwater_live.csv
```

`duration:=0` with `run_forever:=true` keeps the timer alive until Ctrl+C.
The runtime-adjustable ROS parameters are `fluid_density`, `linear_damping`,
`quadratic_damping`, `fluid_current_x/y/z`, `displaced_volume` (four values),
and `segment_thrust` (four values). Invalid density, damping, volume, or
non-finite input updates are rejected atomically.

The optional Qt control panel is launched separately:

```bash
python3 install/asr_sdm_kinematic_dynamic_model/lib/asr_sdm_kinematic_dynamic_model/underwater_control_panel.py
```

The panel provides sliders and presets for the fluid parameters. The C++ node
remains the only source of dynamics; the panel only sends ROS parameter updates.

The node also publishes live RViz2 data:

```text
/underwater_simulator/joint_states
/underwater_simulator/fluid_force_markers
/underwater_simulator/trajectory
world -> screwdrive_segment_0 (dynamic TF)
```

Use `z_to_x_map:=true` (the default) to display native URDF +Z as RViz +X.
The mapping is applied to TF and markers only; joint values and the URDF are
not modified. `force_scale` controls arrow length and
`max_trajectory_points` bounds the trajectory history.

## Scope and limitations

The generated URDF remains the rigid-body source of truth. The legacy
`RobotModel3D` and its tests remain available for controller geometry, but are
not used for simulator dynamics. Detailed rotor dynamics, reaction torque,
motor lag, and spatially varying current fields remain deferred.

# C++ underwater simulator

`underwater_simulator_node` is a deterministic reduced ASR-SDM dynamics runner.
It loads the generated URDF with Pinocchio, locks the eight rotor joints, and
simulates one free flyer plus the six articulation joints:

```text
q: [px, py, pz, qx, qy, qz, qw, six articulation angles]  (13 values)
v: [base linear, base angular, six articulation rates]     (12 values)
```

The equation of motion is

```text
(M_RB + M_added) a = actuator + damping + buoyancy + froude_krylov + joint_passive
                         - rigid_nonlinear - added_mass_bias
```

where `rigid_nonlinear` is `C(q, v) v + g(q)` from Pinocchio's RNEA and
`added_mass_bias` collects every added-mass term that does not multiply the
unknown acceleration.

## Integration and stability

Both integrators are explicit, and this model is stiff: the articulation rows of
the mass matrix are about three orders of magnitude lighter than the base rows
(condition number ≈ 4·10³). Steps of 0.01 s or larger diverge for either method,
which is why the default `dt` is 0.005 s. Divergence is reported as an exception
rather than left to abort inside Pinocchio's quaternion exponential.

Select the method with `integrator`:

- `semi_implicit_euler` (default) — one dynamics evaluation per step.
- `rk4` — four evaluations per step, taken in the tangent space at the start of
  the step so the free-flyer quaternion stays on the manifold.

RK4 is far more accurate at the same step. Over a 0.4 s rollout at `dt=0.005`,
RK4 lands within 3.6·10⁻⁵ of a converged reference while semi-implicit Euler is
off by 4.3·10⁻², and RK4 at `dt=0.005` already beats Euler at `dt=0.0002`. It
does not extend the stable step range, only the accuracy within it.

## Hydrodynamics

Added mass acts on the link acceleration **relative to the fluid**. A constant
world current still contributes, because its local representation rotates with
the link: `d/dt(R' u) = R' u_dot - omega x (R' u)`. Supplying
`fluid_current_acceleration_*` additionally enables the Froude-Krylov term
`rho * V * a_fluid`; it is zero by default and then costs nothing.

Damping is `-(D_l * V_r) - (D_q * (abs(V_r) .* V_r))`. Each matrix is the
configured diagonal coefficient vector plus an optional cross-coupling matrix.
Only the symmetric part has to be positive semidefinite, which is exactly the
condition for the damping to be dissipative, and the model rejects parameters
that would inject energy. Setting `morison_diameter` replaces the flat quadratic
coefficient with slender-cylinder strip drag derived from the segment geometry.

## Actuators

Four aggregate segment thrust inputs remain the default (`aggregate_thrust`),
each split equally between the segment's two screws. The default rotor force axis
is the rotor-frame local +z axis, matching the generated URDF rotor joint axis.

Setting `actuator_command_mode:=rotor_velocity` instead maps eight per-rotor rate
commands through `f = k_lin * w + k_quad * w * abs(w)`, saturating each pair
against the segment budget.

Each screw feeds its ideal-screw reaction torque `tau = f * r * tan(alpha)` back
into its segment. The two screws of a segment have opposite handedness, so
symmetric thrust produces no net torque and only differential thrust does.
Disable it with `enable_reaction_torque:=false`.

A positive `thrust_time_constant` adds first-order thruster lag, integrated with
the exact exponential update so any `dt` stays stable. It carries one extra state
vector, `rotor_thrust`. With the default of zero the actuator is memoryless and
applies its command within the same step.

Rotor angle dynamics and individual rotor spin states remain out of scope.

## Joint dynamics

The six articulation DOFs can carry passive torques, all of which default to zero
so an unconfigured model keeps ideal joints:

- viscous `-b * theta_dot` via `joint_viscous_damping`;
- Coulomb `-c * tanh(theta_dot / scale)` via `joint_coulomb_friction`, tanh
  regularized so the right-hand side stays continuous through zero velocity;
- a one-sided limit spring-damper via `joint_limit_stiffness`,
  `joint_limit_damping`, and `joint_limit`, which only resists deeper penetration
  and never pulls a joint back towards the stop it is escaping.

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
- `gravity_world` (three values, m/s²), which drives both the rigid weight and
  the buoyancy so the two cannot disagree;
- `integrator` (`semi_implicit_euler` or `rk4`);
- `segment_thrust` and `maximum_segment_thrust` (four values, N);
- `rotor_rate` (eight values, rad/s) for `rotor_velocity` mode, together with
  `rotor_thrust_linear` and `rotor_thrust_quadratic` (eight values each);
- `actuator_command_mode`, `thrust_time_constant`, `enable_reaction_torque`,
  `screw_helix_angle`, `screw_radius`, and `screw_handedness` (eight values each);
- `joint_torque` (six values, N m);
- `joint_viscous_damping` and `joint_coulomb_friction` (six values each),
  `joint_friction_velocity_scale`, `joint_limit_stiffness`,
  `joint_limit_damping`, and `joint_limit`;
- `fluid_current_x/y/z` (world-frame m/s) and
  `fluid_current_acceleration_x/y/z` (world-frame m/s²);
- `fluid_density`, `displaced_volume` (four values), `added_mass_diagonal`
  (24 values, six local components per link), `linear_damping`, and
  `quadratic_damping`;
- `morison_diameter`, `morison_length`, and the
  `morison_transverse/axial/rolling_drag_coefficient` set.

The CSV stores time, q/v/a, actuator commands, per-rotor thrust and reaction
torque, passive joint torques, the force-balance residual norm, and per-link
local damping, buoyancy, and total-fluid wrenches. Local wrench fields use
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
`quadratic_damping`, `fluid_current_x/y/z`,
`fluid_current_acceleration_x/y/z`, `displaced_volume` (four values),
`segment_thrust` (four values), `rotor_rate` (eight values), and `joint_torque`
(six values). Invalid density, damping, volume, or non-finite input updates are
rejected atomically. Gravity is deliberately not runtime adjustable, so the
rigid-body and hydrodynamic sides stay consistent.

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
world -> base (dynamic TF)
```

Display axes match the kinematic simulation: the generated URDF roots at `base`
(+X forward) and already maps CAD segment +Z onto that frame with a fixed joint
(`rpy = 0, -pi/2, 0`). The simulator therefore publishes the free-flyer pose as
`world -> base` with `z_to_x_map:=false` by default. Set
`root_frame:=screwdrive_segment_0 z_to_x_map:=true` only for a legacy URDF that
still roots at the segment. `force_scale` controls arrow length and
`max_trajectory_points` bounds the trajectory history.

## Verification

`underwater_dynamics_test` pins the physics down with identities rather than
recorded numbers:

- `C(q, v) v + g(q)` equals RNEA's nonlinear effects, and RNEA and ABA invert
  each other;
- with added mass zeroed, the simulator's acceleration matches Pinocchio's
  articulated-body algorithm driven by the same external forces, which checks the
  whole right-hand-side assembly independently of the mass-matrix path;
- link accelerations match finite differences of the link twists, including the
  rotating-current contribution to the relative acceleration;
- sizing the displaced volume to the rigid weight makes the total world-frame
  buoyancy cancel the total weight, and doubling gravity doubles both;
- cross-coupled damping never injects energy over random velocities, and
  indefinite damping matrices are rejected;
- counter-rotating screws cancel their reaction torque under symmetric thrust;
- lagged thrust reaches `1 - 1/e` of its command after one time constant;
- RK4 is more than an order of magnitude closer to a converged reference than
  semi-implicit Euler, and oversized steps raise instead of aborting.

## Scope and limitations

The generated URDF remains the rigid-body source of truth. The legacy
`RobotModel3D` and its analytic `FluidForceModel::evaluate` path remain available
for controller geometry; that path builds its own Jacobian and has no J-dot, so
callers must pass `explicit_local_accelerations` to include it. Simulator
dynamics go through `evaluatePinocchio`, which gets J-dot from Pinocchio.

Individual rotor spin states, rotor angle dynamics, spatially varying current
fields, free-surface effects, and implicit integration for the stiff articulation
DOFs remain deferred.

# asr_sdm_planning_manager

High-level planner for the production stack: topological replanning, B-spline
optimization, heading from the path tangent, and trajectory execution.

Stack-wide topics and launch are documented in [`../README.md`](../README.md).
This page covers what this package actually optimizes.

```text
/goal_pose (or waypoints) + odom
  -> TopoReplanFSM / PlanningManager
       global min-snap polynomial
       local B-spline: TopologyPRM → L-BFGS → lowest-jerk pick
       refine: stretch duration, re-optimize
       yaw / pitch from the position tangent
  -> /planning/bspline
  -> traj_server (100 Hz)
  -> /control/asr_sdm/robot_cmd

/control/initial_pose (RViz 2D Pose Estimate)
  -> control manager: reset pose
  -> TopoReplanFSM: drop target, WAIT_TARGET
  -> traj_server: stop publishing robot_cmd
```

## Nodes

| Executable | Role |
|---|---|
| `planning_manager_node` | `TopoReplanFSM` + in-process ESDF map |
| `traj_server` | Sample `/planning/bspline` at 100 Hz into `RobotCommand` |

Parameters live in `config/topo_replan.yaml`. The B-spline costs themselves are
implemented in `asr_sdm_trajectory_optimizer`; this package **selects which
terms are on** at each stage.

## Cost functions

`PlanningManager` never evaluates a scalar cost of its own. It calls
`BsplineOptimizer::BsplineOptimizeTraj` with a bit-mask. `combineCost` then
sums the enabled terms. Leading and trailing `order` control points stay fixed
unless `ENDPOINT` is set (it is not, in this package).

Weights are `optimization.lambda*` in `topo_replan.yaml`. Limits that the
hinge terms compare against (`max_vel`, `max_acc`, `max_yaw_rate`,
`max_pitch_rate`, `min_vel`) come from `manager.*`.

### Terms the optimizer can evaluate

| Flag | Weight | Formula (on control points \(q_i\)) | When it is nonzero |
|---|---|---|---|
| `SMOOTHNESS` | `lambda1` | \(\sum \|q_{i+3}-3q_{i+2}+3q_{i+1}-q_i\|^2\) | Always, on the free polygon. This is discrete jerk, not physical \(\mathrm{m/s}^3\). |
| `DISTANCE` | `lambda2` | \(\sum (d_i-d_0)^2\) for \(d_i < d_0\) | ESDF clearance below `dist0`. Gradient is the normalized ESDF gradient. Inactive in open space. |
| `FEASIBILITY` | `lambda3` | Per-axis hinge on \(\|v\|^2>v_{\max}^2\) and \(\|a\|^2>a_{\max}^2\) | Overspeed / over-acceleration after the knot span \(\Delta t\) is fixed. |
| `GUIDE` | `lambda5` | \(\sum \|q_i-g_i\|^2\) | Interior points vs. the TopologyPRM guide polyline. |
| `WAYPOINTS` | `lambda7` | \(\|(q_k+4q_{k+1}+q_{k+2})/6 - w\|^2\) | 1-D yaw/pitch fit: the cubic B-spline value at knot \(k\) vs. a sampled angle. |
| `NONHOLONOMIC` | `lambda_yaw_rate`, `lambda_pitch_rate`, `lambda_min_vel` | Saturated hinges, see below | Only if `manager.nonholonomic` is true **and** the spline is 3-D (position). |

`ENDPOINT` (`lambda4`), visibility (`lambda6`) and acc-smoothness (`lambda8`)
are still declared on the node but **not combined**. `heading_planner.lambda*`
in the same yaml is leftover; `planHeading` does not read it.

### Nonholonomic hinges

Body \(+x\) is the trajectory tangent, with the same `R = Rz(yaw)·Ry(pitch)`
convention as `asr_sdm_control_manager`. Angles are read from velocity control
points \(v_i=(q_{i+1}-q_i)/\Delta t\):

\[
\psi=\mathrm{atan2}(v_y,v_x),\qquad
\theta=\mathrm{atan2}(-v_z,\sqrt{v_x^2+v_y^2}).
\]

Yaw rate and pitch rate use the same one-sided penalty on
\(\omega=(\alpha_{i+1}-\alpha_i)/\Delta t\) (yaw wrapped to \((-\pi,\pi]\)):

\[
e=\lvert\omega\rvert/\omega_{\max}-1,\qquad
c=\frac{e^2}{1+e^2}\quad(\lvert\omega\rvert>\omega_{\max}).
\]

The map \(e^2/(1+e^2)\) saturates so a large initial violation does not blow
up the L-BFGS step. `lambda_min_vel` is the matching *under*-limit hinge on
\(\|v\|<\texttt{min\_vel}\), so the tangent (and therefore the heading) stays
well defined. A vanishing velocity is skipped rather than penalized.

### Composite phases

```text
GUIDE_PHASE                = SMOOTHNESS | GUIDE
GUIDE_NONHOLONOMIC_PHASE   = GUIDE_PHASE | NONHOLONOMIC
NORMAL_PHASE               = SMOOTHNESS | DISTANCE | FEASIBILITY
NONHOLONOMIC_PHASE         = NORMAL_PHASE | NONHOLONOMIC
```

| Stage | Code | Mask | Solver budget |
|---|---|---|---|
| Topo candidate, phase 1 | `optimizeTopoBspline` | `topoGuideCostFunction()` | `max_iteration_num1` / `max_iteration_time1` |
| Topo candidate, phase 2 | `optimizeTopoBspline` | `localCostFunction()` | `max_iteration_num2` / `max_iteration_time2` |
| Refine (collision or not) | `refineTraj` | `localCostFunction()` | same as phase 2 |
| Yaw / pitch fit | `fitAngleBspline` | `SMOOTHNESS \| WAYPOINTS` | same as phase 2 |

`topoGuideCostFunction()` is `GUIDE_NONHOLONOMIC_PHASE` when
`manager.nonholonomic` is true, otherwise `GUIDE_PHASE`.
`localCostFunction()` is `NONHOLONOMIC_PHASE` or `NORMAL_PHASE` the same way.

Phase 1 starts from the *colliding* local segment. `lambda5` has to outweigh
`lambda1` there, or the polygon never reaches the detour homotopy. The yaw /
pitch hinges keep that first pull from locking onto a knife-edge corner of the
TopologyPRM polyline. Phase 2 then adds clearance and feasibility.

A detour is longer than the window it replaces but inherits that window's
duration, which saturates the heading hinges. `optimizeTopoBspline` therefore
stretches the knot span by `guide_len / (max_vel · duration)`, capped at
`max_time_lengthen_ratio`, before either phase runs.

### Costs outside L-BFGS

These are used by the manager but are not `lambda*` terms:

| Stage | What is minimized |
|---|---|
| Global reference | Closed-form **minimum snap** (`minSnapTraj`) through densified waypoints |
| Candidate pick | `getJerk() * headingRateRatio()`; lowest wins. The ratio is the peak tangent yaw / pitch rate over the matching limit (1.0 if already feasible). |
| Refine timing | `max(checkRatio(), headingRateRatio())`, then duration stretch capped by `max_time_lengthen_ratio` |

### Default weights (`topo_replan.yaml`)

| Parameter | Default | Role |
|---|---|---|
| `lambda1` | 10 | Smoothness |
| `lambda2` | 5 | Clearance inside `dist0` (0.4 m) |
| `lambda3` | 1 | Residual vel/acc after time stretch |
| `lambda4` | 0.001 | Unused (`ENDPOINT`) |
| `lambda5` | 1.5 | Guide pull in phase 1 |
| `lambda6` | 10 | Unused (visibility) |
| `lambda7` | 20 | Heading waypoint fit |
| `lambda_yaw_rate` | 10 | Yaw-rate hinge |
| `lambda_pitch_rate` | 10 | Pitch-rate hinge |
| `lambda_min_vel` | 1 | Minimum forward speed |

Open space: `DISTANCE` is ~0, so `lambda1` dominates how fast the polygon
straightens. Dense obstacles: raise `lambda2` / `lambda5` and drop `lambda1`,
or smoothness and a weak guide will cut the homotopy through a gap. Heading
rate weights should stay below clearance in clutter; otherwise the solver
flattens a tight turn back into the obstacle (time stretch in refine is what
actually lowers \(\omega\)).

## Launch

```bash
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py
```

---

<a id="中文"></a>

## 中文：代价函数

`PlanningManager` 自己不算标量代价，只给 `BsplineOptimizer` 传位掩码。
权重在 `config/topo_replan.yaml` 的 `optimization.lambda*`，限幅在 `manager.*`。

### 位置 B 样条（L-BFGS）

| 项 | 权重 | 含义 |
|---|---|---|
| `SMOOTHNESS` | `lambda1` | 控制点离散 jerk \(\|q_{i+3}-3q_{i+2}+3q_{i+1}-q_i\|^2\) |
| `DISTANCE` | `lambda2` | ESDF 距离小于 `dist0` 时罚 \((d-d_0)^2\)；空旷处为 0 |
| `FEASIBILITY` | `lambda3` | 分轴速度/加速度超 `max_vel` / `max_acc` 的二次铰链 |
| `GUIDE` | `lambda5` | 控制点贴 TopologyPRM 引导折线 |
| `WAYPOINTS` | `lambda7` | 1 维 yaw/pitch 样条贴采样角 |
| `NONHOLONOMIC` | `lambda_yaw_rate` / `lambda_pitch_rate` / `lambda_min_vel` | 切向偏航/俯仰角速度超限、前向速度过低；\(e^2/(1+e^2)\) 饱和铰链 |

`ENDPOINT`（`lambda4`）、可见性（`lambda6`）、`lambda8` 以及
`heading_planner.lambda*` **未接入**。

非完整项只在 `manager.nonholonomic` 且样条为 3 维位置时生效。机体系 \(+x\)
等于轨迹切向（`R = Rz(yaw)·Ry(pitch)`），角速度在固定 \(\Delta t\) 上计算。
优化器**不改时长**；绕障段先经 `refineTraj` 按 `max_time_lengthen_ratio` 拉长时间，
再跑第二阶段。

### 何时打开哪几项

| 阶段 | 掩码 |
|---|---|
| 拓扑候选第一阶段 | `GUIDE_NONHOLONOMIC_PHASE`（默认）= 平滑 + 引导 + 非完整 |
| 拓扑候选第二阶段、精修 | `NONHOLONOMIC_PHASE`（默认）= 平滑 + 距离 + 可行性 + 非完整 |
| 航向拟合 | 平滑 + 路点 |

第一阶段初值仍在碰撞段上。`lambda5` 必须压过 `lambda1`，否则到不了绕障同伦。
偏航/俯仰铰链避免第一阶段把控制点焊在 TopologyPRM 折线的尖角上。绕障段比
原窗口长却继承原时长，铰链会饱和，因此 `optimizeTopoBspline` 会按
`guide_len / (max_vel · duration)` 拉长 knot span（不超过
`max_time_lengthen_ratio`）。

空旷主要靠 `lambda1`；密障应加大 `lambda2`/`lambda5`、减小 `lambda1`，
航向限幅权重要低于间隙项，避免把弯抹平穿障。

### L-BFGS 之外

- 全局参考：闭式 **minimum snap**
- 多条拓扑候选：选 `getJerk() * headingRateRatio()` 最小的一条
- 精修：按速度/加速度/航向角速度超限比拉长时间，再优化间隙

# planner

Sampling-based motion planning for CTR deployment. Builds a collision-free,
follow-the-leader path from the current configuration to a target using OMPL's
RRT-Connect in a two-phase scheme (rotate, then deploy analytically), with the PINN
supplying kinematics and the EKF force estimate shaping the plan. (An AORRTC
configuration exists in `Planner.hpp` but the production path selects
`PLANNER_RRT_CONNECT` — see `planner_node.cpp`.)

Pure computation — no hardware. It hands finished paths to `manager` through a CSV in
`Shared_Files/`.

## Quick start

```bash
source install/setup.bash
ros2 launch planner launch.py
```

Then ask for a plan:

```bash
ros2 service call /planner/command interfaces/srv/Planner \
  "{command: 'generateTrajectory', value: [0,0,0]}"
```

In normal operation `manager`'s GUI issues these requests, not you.

## Nodes

| Executable | Node name in code | Name at launch | Role |
|---|---|---|---|
| `plan` | `path_planner` | `planner_node` | Two-phase OMPL planning with mid-deployment replanning |

## Launch file

`launch/launch.py` starts `planner_node` on CPU core 9. It declares **no** launch
arguments at all — the simplest case of the workspace-wide rule that
`config/*_params.yaml` is authoritative. Everything is configured through the YAML.
(Not at runtime: `solve_time` and `verbose_planner_log` are read once at startup, and
the node registers no parameter callback.)

## Parameters

From `config/planner_params.yaml` — the single source of truth. Editing it needs
`colcon build --packages-select planner`, because launch reads the installed `share/`
copy rather than the source tree. See [Configuration and
parameters](../../README.md#configuration-and-parameters).

| Parameter | Default | Meaning |
|---|---|---|
| `model_name` | `ctr_8x91_0.18_tanh_9K_9K_50K_v3` | PINN model. **Must match `robot_params.yaml` and `mpc_params.yaml`** — `_v3` and `_FP64` are different networks with byte-identical `parameters.json`, so a mismatch is invisible except as an unattributable tip error |
| `models_dir` | `""` | Empty → installed `share/ctr_kinematics_pinn/models` |
| `solve_time` | 3.0 | OMPL solve budget per request, seconds |
| `data_root` | `""` | Empty → env `CTR_DATA_ROOT` → legacy workspace climb |
| `verbose_planner_log` | `false` | Print the planning library's debug diagnostics (state-space bounds, planner range) |

Every `generateTrajectory` request also appends a structured record (target, azimuth,
start/goal configurations, IK diagnostics, projection deltas, timings, outcome) to
`<data_root>/Output_Files/diagnostics/planner/<timestamp>/planner_diag.csv`; correlate
with the manager's `manager_diag.csv` by wall time.

`temp_dir` is deliberately **not** set in the YAML. The node defaults it to
`<data_root>/Shared_Files`, which is the live path channel to `manager` — overriding
it breaks the handoff.

## Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| sub | `task_space/feedback/base_tool` | `interfaces/msg/Taskspace` |
| sub | `task_space/force_estimate` | `interfaces/msg/Force` |
| pub | `task_space/path` | `std_msgs/msg/Float64MultiArray` |
| pub | `joint_space/target` | `interfaces/msg/Jointspace` |

A 50 ms timer services TF lookups.

## Services

| Service | Type | Direction |
|---|---|---|
| `planner/command` | `interfaces/srv/Planner` | provided |

### Concurrency contract

The ~3 s OMPL solve runs **inside the service callback**, deliberately. `manager`
keeps at most one outstanding request, and the whole
`write(plannedPath.csv) → respond → manager reads` sequence is ordered by the service
round-trip. The source carries a comment saying exactly this. Adding a second client
without revisiting that ordering will corrupt the handoff.

## Path handoff

`plannedPath.csv` is written **atomically** — to a `.tmp` file, then renamed — so
`manager`'s poll never observes a half-written path. The planner also publishes
`task_space/path` for visualization through `igtlink_bridge`.

## Planning approach

Two-phase: a revolute rotation phase, then a prismatic deployment phase evaluated
over candidate deployment schedules under a follow-the-leader objective that
penalizes swept volume. `motion_planning/include/DeploymentSchedule.hpp` holds the
schedule geometry, split out so it can be tested without OMPL or Torch.

Mid-deployment replanning re-solves only the remaining tail; `manager` decides when
to ask, based on EKF force drift.

## Dependencies

Beyond the usual ROS set: **OMPL ≥ 2.0** (AORRTC first shipped in 2.0.0), FCL,
Assimp, pugixml, Boost, Blaze, TBB, NLopt, BLAS/LAPACK, OpenMP, and
`ctr_kinematics_pinn` for kinematics. The in-tree static library `CTR_MPL` is built
from `motion_planning/` and holds the OMPL state space, sampler, validity checker,
and objectives.

OMPL headers must be reachable as `/usr/local/include/ompl`.

## Tests

```bash
colcon test --packages-select planner
```

Two targets, neither linking OMPL or Torch:

- `test/test_deployment_schedule.cpp` — pure geometry: candidate start/end correctness,
  monotonicity and step bounds, the degenerate zero-displacement case, swept-cost segment
  summation, and empty/no-function edges.
- `test/test_dataset_bounds.cpp` — pins the dataset β₁ convention that
  `ctr_kinematics_pinn::PINNs::getInputPosBounds()` feeds into this package's OMPL bounds
  and `CTR_StateValidityChecker`. β₁ is stored *relative* to β₂; using it as an absolute
  bound empties the admissible β₁ interval at the retracted pose and every
  `setStartState()` throws. See `ctr_kinematics_pinn/README.md`.

## Status notes

The `manual_target` service registration is commented out in `planner_node.cpp` — the
`interfaces/srv/Config` service named `manual_target` is **not** available at runtime.
`planner/command` is the only service this node provides.

Dead inputs that mislead triage: the 50 ms tf2 timer fills `m_manual_target` and nothing
reads it; the `task_space/feedback/base_tool` subscription fills `m_x` and nothing reads
it; the `joint_space/target` publisher is created and never published to. The node's only
live inputs are `joint_space/feedback`, `task_space/force_estimate`, and the service
request itself — and neither feedback topic is checked for "never received", so a request
that arrives before the robot is up plans from `q = 0` and `f = 0`.

`igtlink_bridge` also holds a `planner/command` client, so the "single outstanding
request" note above is enforced only by the manager. A Slicer-injected request serialises
against the manager's in the service's (mutually exclusive) callback group.

IK non-convergence now warns. `inverseKin` uses the `bool` from
`Planner::solveInverseKinematics` and logs an `RCLCPP_WARN` with the residual when the
solve misses its 1 mm tolerance; planning still proceeds with the best-effort
configuration, exactly as before. The residual is also `response->value`, and
`manager/master` rejects the plan outright above 3 mm (`k_ik_error_threshold`), so that
warning is the first place a "target unreachable" retry loop shows up.

IK latency is **not** covered by `solve_time` — it is additive to the service
round-trip, and `posCTRL`'s budget is up to 3000 descent steps
(`ctr_kinematics_pinn/README.md`). Worst-case planning is 1.5 × `solve_time`, and the
manager gives up after `planner_timeout_s` (15 s), after which the planner keeps
computing and answers a request nobody is waiting for. Watch the `IK time:` line if
either budget is raised — measured worst case is currently ~4.5 s.

`CTR_StateValidityChecker::isValid` delegates to `ctr_kinematics_pinn::isFeasible4`
(4-DoF layout) so it cannot disagree with the IK about which configurations are
legal. It previously enforced only the β₁ ≤ β₂ − clearance half of β₁'s coupled
window and omitted the β₁ ≥ β₂ − 0.084 half — the tube-protrusion constraint — so the
planner explored states where the inner tube retracts inside the middle one, outside
the box the PINN was trained on. Adding it **shrinks the valid set**: plans that used
to route through non-protruding states will not any more. Both poses the hardware
homes to remain valid (they sit exactly on opposite corners of the set, which
`test/test_dataset_bounds.cpp` pins).

The reverse disagreement was worse and is what actually blocked planning: `posCTRL`
capped β₂ 4 mm looser than this checker, so **21% of converged IK solutions made
`setGoalState` throw** — measured with `ctr_kinematics_pinn/benchmark/ik_bench.cpp`.
`posCTRL` now projects its result into the feasible set, so that rate is a hard zero
by construction rather than a small number. If you ever see `setGoalState` reject an
IK result again, that projection or this checker has drifted from the shared
predicate.

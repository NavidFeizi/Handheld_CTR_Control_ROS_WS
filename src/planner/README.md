# planner

Sampling-based motion planning for CTR deployment. Builds a collision-free,
follow-the-leader path from the current configuration to a target using OMPL's AORRTC,
with the PINN supplying kinematics and the EKF force estimate shaping the plan.

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
`config/*_params.yaml` is authoritative. Everything is configured through the YAML, or
at runtime with `ros2 param set planner_node <param> <value>`.

## Parameters

From `config/planner_params.yaml` — the single source of truth. Editing it needs
`colcon build --packages-select planner`, because launch reads the installed `share/`
copy rather than the source tree. See [Configuration and
parameters](../../README.md#configuration-and-parameters).

| Parameter | Default | Meaning |
|---|---|---|
| `model_name` | `ctr_8x91_0.18_tanh_9K_9K_50K_FP64` | PINN model; the planner uses the FP64 build for solver stability |
| `models_dir` | `""` | Empty → installed `share/ctr_kinematics_pinn/models` |
| `solve_time` | 3.0 | OMPL solve budget per request, seconds |
| `data_root` | `""` | Empty → env `CTR_DATA_ROOT` → legacy workspace climb |

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
penalizes backbone motion normal to itself (lateral swept area). `motion_planning/include/DeploymentSchedule.hpp` holds the
schedule geometry, split out so it can be tested without OMPL or Torch;
`motion_planning/include/FtlSweptArea.hpp` holds the FTL metric. Joint bounds and
the FTL formulation are analysed in [docs/ik_ftl_forensic_report.md](docs/ik_ftl_forensic_report.md).

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

Both suites are pure geometry and link neither OMPL nor Torch.
`test/test_deployment_schedule.cpp`: candidate start/end correctness, monotonicity and
step bounds, the degenerate zero-displacement case, swept-cost segment summation, and
empty/no-function edges. `test/test_ftl_swept_area.cpp`: zero cost for a perfect FTL
extension and for tangential slip, area = shift × length for a lateral shift,
additivity under path splitting, mirror invariance, degenerate inputs.

## Status notes

The `manual_target` service registration is commented out in `planner_node.cpp` — the
`interfaces/srv/Config` service named `manual_target` is **not** available at runtime.
`planner/command` is the only service this node provides.

## Diagnostic logging

`planner_node` installs a package log file (`ctr_common::installPackageFileLog`) as
the first statement of its constructor body: every rcutils record from this process
(console output included) is also teed to
`<log_root>/[<CTR_RUN_ID>/]planner/<timestamp>_planner_node_<pid>.log`. See
`ctr_common`'s `package_log.hpp` for `log_root` resolution.

Every log line is tagged by pipeline stage:

| Tag | Stage |
|---|---|
| `[lifecycle]` | startup config/domain summary, non-positive `solve_time` |
| `[comm]` | service request/response (sequence number, target, timings, reason code), feedback-topic status |
| `[pinn]` | PINN trained-domain checks on the IK start state and solution |
| `[ik]` | `solveInverseKinematics`/`posCTRL` seed, result, convergence diagnostics |
| `[plan]` | `planTwoPhase`/`planDeployment` start/goal, result, exceptions |
| `[traj]` | CSV → task-space path conversion and publishing |

A `generateTrajectory` request logs the trail `[comm] request` → `[plan]` force/force-aware
setup → `[pinn]` start-domain check (if violated) → `[ik]` seed/result → `[plan]`
start/goal/result → `[comm] response`, so a failed request can be diagnosed from the log
alone. The `[comm] response` line carries a `reason` code:

| Reason | Meaning |
|---|---|
| `OK` | planning succeeded and the IK solution converged within `posTol` |
| `INVALID_TARGET` | the requested target had a non-finite coordinate |
| `START_INVALID` | `Planner` rejected the current joint state as a start state |
| `GOAL_INVALID` | `Planner` rejected the IK-derived joint state as a goal state |
| `IK_NOT_CONVERGED` | planning succeeded but IK error exceeded its internal tolerance (the manager's separate accept threshold is 3 mm) |
| `PLAN_FAILED` | `planTwoPhase` found no solution |
| `EXCEPTION` | any other exception from `Planner`/OMPL |

A non-finite target is rejected before any IK/plan work runs: `response->success =
false`, `response->value = 0.0`, and a new `message` string
("invalid target: non-finite value(s) in target") that neither `manager` nor
`igtlink_bridge` pattern-match on — both only log/forward `response->message` verbatim.
No other response message text changed; the success-path message
("Path generated successfully.") is unchanged.

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
arguments — all configuration comes from the YAML.

## Parameters

From `config/planner_params.yaml`.

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
colcon test --packages-select planner --ctest-args -R test_deployment_schedule
```

`test/test_deployment_schedule.cpp` is pure geometry and links neither OMPL nor
Torch: candidate start/end correctness, monotonicity and step bounds, the degenerate
zero-displacement case, swept-cost segment summation, and empty/no-function edges.

## Status notes

The `manual_target` service registration is commented out in `planner_node.cpp` — the
`interfaces/srv/Config` service named `manual_target` is **not** available at runtime.
`planner/command` is the only service this node provides.

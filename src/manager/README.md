# manager

Procedure orchestration and data recording. `master` is the operator's top-level GUI:
it holds the target list, asks `planner` for a path, then drives deployment
waypoint-by-waypoint, requesting a replan when the estimated tip force drifts away
from the plan's baseline. `record` writes synchronized CSV logs of a run.

Needs no hardware of its own — it is pure ROS orchestration on top of `robot`,
`planner`, and `emtracker`.

## Quick start

```bash
source install/setup.bash
ros2 launch manager launch.py       # master GUI + recorder
```

Launch this separately from `ctr_bringup`'s system bring-up, after the robot and
tracker are up. `planner` must also be running for the plan/deploy workflow.

## Nodes

| Executable | Node name | Role |
|---|---|---|
| `master` | `master_node` | Operator GUI, target management, plan requests, deployment loop |
| `record` | `recorder` (launched as `recorder_node`) | Synchronized CSV recording |

`master_qt_gui.cpp` is **not** a separate node — it is the Qt GUI implementation mixed
into the same class (`class MasterNode : public QWidget, public rclcpp::Node`).

## Launch file

`launch/launch.py` starts both nodes on CPU core 2.

| Argument | Default | Overrides |
|---|---|---|
| `recorder_sample_time` | *(none)* | `recorder_node.sample_time` |

It carries no default: unset it is dropped and `config/manager_params.yaml` supplies the
value. Pass it explicitly to override for a single run:

```bash
ros2 launch manager launch.py recorder_sample_time:=0.01
```

`master_node` has no launch arguments at all — every one of its tunables comes from the
YAML. Editing the YAML needs `colcon build --packages-select manager`, since launch reads
the installed `share/` copy. See [Configuration and
parameters](../../README.md#configuration-and-parameters) for the mechanism.

## Parameters

From `config/manager_params.yaml`. The defaults below are the YAML's values and are what
the nodes actually receive; the launch file does not shadow them.

### `master_node`

| Parameter | Default | Meaning |
|---|---|---|
| `force_replan_threshold` | 0.12 | N — ‖f_now − f_at_plan‖ that triggers a deployment replan |
| `replan_cooldown_s` | 2.0 | s between replan requests |
| `min_remaining_waypoints` | 5 | Below this, a replan is not worth the pause |
| `targets_csv` | `random_interior_points.csv` | Target list in `<data_root>/Input_Files/` |
| `planner_timeout_s` | 30.0 | s — diagnostic-only: logs an ERROR if a `planner/command` request stays outstanding this long; never resets or retries |
| `data_root` | `""` | Empty → env `CTR_DATA_ROOT` → legacy workspace climb |

### `recorder_node`

| Parameter | Default | Meaning |
|---|---|---|
| `sample_time` | 0.025 | s |
| `data_root` | `""` | As above |

## Topics

| Node | Direction | Topic | Type |
|---|---|---|---|
| `master` | sub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| `master` | sub | `robot_status` | `interfaces/msg/Status` |
| `master` | sub | `manual_interface` | `interfaces/msg/Interface` |
| `master` | sub | `task_space/sim_out` | `interfaces/msg/Taskspace` |
| `master` | sub | `task_space/force_estimate` | `interfaces/msg/Force` |
| `master` | pub | `joint_space/manual_vel` | `interfaces/msg/Jointspace` |
| `master` | pub | `joint_space/target` | `interfaces/msg/Jointspace` |
| `master` | pub | `task_space/target` | `interfaces/msg/Taskspace` |
| `record` | sub | `task_space/feedback/base_tool`, `task_space/target`, `task_space/sim_out` | `interfaces/msg/Taskspace` |
| `record` | sub | `joint_space/feedback`, `joint_space/target` | `interfaces/msg/Jointspace` |
| `record` | sub | `task_space/force_estimate` | `interfaces/msg/Force` |
| `record` | sub | `mpc/computation_time` | `std_msgs/msg/Float64` |

## Services

| Service | Type | Direction |
|---|---|---|
| `recording` | `interfaces/srv/Recording` | provided by `record` |
| `robot_config`, `robot_enable` | `interfaces/srv/Config` | called by `master` |
| `planner/command` | `interfaces/srv/Planner` | called by `master` |
| `freeze_robot` | `std_srvs/srv/SetBool` | called by `master` |
| `recording` | `interfaces/srv/Recording` | called by `master` |

## The deployment loop

`master` and `planner` communicate through **two** channels, and both matter:

1. **The service** — `master` sends a `planner/command` request and waits. Only one
   request is outstanding at a time, by design.
2. **The file** — `planner` writes `Shared_Files/plannedPath.csv` atomically (write
   `.tmp`, then rename) and `master` polls for it. The atomic rename is what makes
   the poll safe; a partially written file is never visible under the final name.

With a path in hand, `master` steps through the waypoints. On each step it compares
the current EKF force estimate against the baseline captured when the plan was made.
If the difference exceeds `force_replan_threshold`, and the cooldown has elapsed, and
more than `min_remaining_waypoints` remain, it requests a replan of the remaining
tail. Rejected requests back off exponentially with hysteresis, and the previous plan
keeps executing — a failed replan degrades to the old path rather than stopping. A
replan the planner skips because its planning force is unchanged (e.g. both gated to
zero) is not a rejection: the current plan continues and the force baseline is reset.

Waypoints are downsampled with `ctr_common::downsamplePath` (every 2 mm of any β or 5°
of any α), the same function the planner uses for the drawn path. When the planner
rejects a Select Target (`generateTrajectory`) request and no insertion is outstanding,
the previous unexecuted plan is discarded and the planner is asked (`clearPath`) to
clear the drawn path; Auto Insert then stops with `[cmd] no accepted plan` instead of
driving to the old target. A rejected closed-loop re-plan keeps the active plan.

Path CSVs are accepted in two layouts: a 6-column form and the planner's 4-column
form, which is expanded with zero β3/α3. See `include/manager/csv_path_io.hpp`.

## Recording

`recording` takes a command, a duration, and a session name. Output lands in a
timestamped session directory under the resolved data root
(`<data_root>/Output_Files/<name>/<YYYY-MM-DD_HH-MM-SS>/`) via
`ctr_common::makeSessionDir`.

## Threading

`master` runs a `MultiThreadedExecutor` beside the Qt thread. Widget mutations from
ROS callbacks go through `QMetaObject::invokeMethod(..., Qt::QueuedConnection)`, and
cross-thread state is atomic or mutex-snapshotted. The GUI uses 500 ms readiness
timers to gate its controls rather than blocking in the constructor waiting for
services.

## Diagnostic logs

`master` and `record` each tee their rcutils log stream (via
`ctr_common::installPackageFileLog`) into a package-separated file, on top of the
normal console/`~/.ros/log` output:

```
<log_root>/[<CTR_RUN_ID>/]manager/<YYYY-MM-DD_HH-MM-SS>_master_node_<pid>.log
<log_root>/[<CTR_RUN_ID>/]manager/<YYYY-MM-DD_HH-MM-SS>_recorder_<pid>.log
<log_root>/[<CTR_RUN_ID>/]manager/master_node_latest.log   (symlink)
<log_root>/[<CTR_RUN_ID>/]manager/recorder_latest.log      (symlink)
```

`log_root` resolves from the `log_dir` parameter → env `CTR_LOG_DIR` →
`<data_root>/log`. Since `master` and `record` are separate processes, each gets
its own file even though both belong to the `manager` package.

Every message starts with a grep-able stage tag:

| Tag | Node(s) | Meaning |
|---|---|---|
| `[lifecycle]` | both | Startup params/data root, service readiness, recording session open/close |
| `[comm]` | both | Service requests/responses (`robot_config`, `robot_enable`, `recording`), `robot_status` edge-triggers, tf2 lookup health |
| `[procedure]` | `master` | Why the Planner-mode branch is/isn't calling `planner/command`: procedure inactive, tube-rotation alignment progress/convergence, waiting for reached/response, target unchanged |
| `[plan]` | `master` | `planner/command` responses: success, message, IK error (mm), round-trip time. The GUI "Planner Status" row mirrors the outcome in its **Message** column (`accepted: ik_error=… mm, N waypoints - <planner message>`, `rejected: <planner message>`, `rejected: ik_error=… mm >= 3.00 mm - …`, `replan accepted/rejected: …`); **Success: No** now also covers plans rejected by the 3 mm IK-error gate. An accepted message ending in `planned with zero tip force)` means the EKF force estimate was saturated and the planner ignored it (see `planner` README) |
| `[traj]` | `master` | `plannedPath.csv` load: row counts, first/last waypoint, per-joint range, out-of-limits warnings |
| `[cmd]` | `master` | Deployment insertion/retraction start/stop/complete, per-waypoint dispatch, stalled-at-waypoint warnings |

Only records that pass the logger's own severity threshold reach the file. Most
per-cycle detail is `DEBUG`; raise it for one node at a time with:

```bash
ros2 launch manager launch.py --ros-args --log-level master_node:=debug
ros2 launch manager launch.py --ros-args --log-level recorder:=debug
```

## Tests

```bash
colcon test --packages-select manager --ctest-args -R test_csv_path_io
```

`test/test_csv_path_io.cpp` covers the pure functions in `csv_path_io.hpp`:
`parsePathRows` (6-column pass-through, 4-column expansion, malformed-row skipping,
end-to-end text parse).

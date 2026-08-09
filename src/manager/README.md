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

| Argument | Default |
|---|---|
| `recorder_sample_time` | `.025` |

## Parameters

From `config/manager_params.yaml`.

### `master_node`

| Parameter | Default | Meaning |
|---|---|---|
| `force_replan_threshold` | 0.12 | N — ‖f_now − f_at_plan‖ that triggers a deployment replan |
| `replan_cooldown_s` | 2.0 | s between replan requests |
| `min_remaining_waypoints` | 5 | Below this, a replan is not worth the pause |
| `targets_csv` | `random_interior_points.csv` | Target list in `<data_root>/Input_Files/` |
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
keeps executing — a failed replan degrades to the old path rather than stopping.

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

## Tests

```bash
colcon test --packages-select manager --ctest-args -R test_csv_path_io
```

`test/test_csv_path_io.cpp` covers the pure functions in `csv_path_io.hpp`:
`parsePathRows` (6-column pass-through, 4-column expansion, malformed-row skipping,
end-to-end text parse) and `adjustConfigurationListStepSize` (empty in/out, first and
last preserved, downsampling on the deployment coordinate).

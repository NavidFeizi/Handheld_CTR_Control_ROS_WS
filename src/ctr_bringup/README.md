# ctr_bringup

Launch-only package: one file, `system_bringup.launch.py`, that starts the live
system in the right order. No nodes, no library code.

## Quick start

```bash
source install/setup.bash
ros2 launch ctr_bringup system_bringup.launch.py
```

The workspace-root `launch/system_bringup.launch.py` is a shim that forwards here, so
the older invocation still works:

```bash
ros2 launch launch/system_bringup.launch.py
```

## What it starts, and when

| t | Package | Contents |
|---|---|---|
| 0 s | `emtracker` | `emt_node` — NDI Aurora driver |
| +14 s | `robot` | `robot_node`, `gui_node` (itself +5 s inside `robot.py`), `pinn_fk_node`, `ekf_node` |
| +20 s | `igtlink_bridge` | `igtlink_bridge_slicer_node` |

**Preserve this ordering.** The delays are `TimerAction`s that give the NDI tracker
time to finish initializing before anything tries to localize against it. Shortening
them produces a system that comes up looking fine and then reports bad poses.

## Launch arguments

All are forwarded to the included launch files. **None of them carries a default value.**
Left unset they resolve to the empty string, the included launch file drops them, and the
target package's `config/*_params.yaml` supplies the value. Pass one explicitly to
override the YAML for a single run.

| Argument | Default | Goes to | YAML that owns the value |
|---|---|---|---|
| `Kp` | *(none)* | `robot` | `robot/config/robot_params.yaml` |
| `Ki` | *(none)* | `robot` | `robot/config/robot_params.yaml` |
| `maxVel` | *(none)* | `robot` | `robot/config/robot_params.yaml` |
| `maxAcc` | *(none)* | `robot` | `robot/config/robot_params.yaml` |
| `f_dot` | *(none)* | `robot` (EKF) | `robot/config/robot_params.yaml` |
| `host_name` | *(none)* | `emtracker` | `emtracker/config/emtracker_params.yaml` |
| `send_on_igtl` | *(none)* | `emtracker` | `emtracker/config/emtracker_params.yaml` |
| `enable_position_logging` | *(none)* | `emtracker` | `emtracker/config/emtracker_params.yaml` |
| `hostname_slicer` | *(none)* | `igtlink_bridge` | `igtlink_bridge/config/igtlink_params.yaml` |
| `port_slicer` | *(none)* | `igtlink_bridge` | `igtlink_bridge/config/igtlink_params.yaml` |
| `hostname_module` | *(none)* | `igtlink_bridge` | `igtlink_bridge/config/igtlink_params.yaml` |
| `port_module` | *(none)* | `igtlink_bridge` | `igtlink_bridge/config/igtlink_params.yaml` |
| `auto_connect` | *(none)* | `igtlink_bridge` | `igtlink_bridge/config/igtlink_params.yaml` |

```bash
ros2 launch ctr_bringup system_bringup.launch.py                          # everything from the YAMLs
ros2 launch ctr_bringup system_bringup.launch.py host_name:=/dev/ttyUSB0  # one-off override
ros2 launch ctr_bringup system_bringup.launch.py --show-args              # empty defaults + owning YAML
```

Overrides work through the workspace-root shim as well — `ros2 launch
launch/system_bringup.launch.py host_name:=/dev/ttyUSB0` reaches `emt_node`, because
launch configurations are inherited by included launch descriptions.

**This package is the one most likely to break the rule.** It re-declares arguments that
the included launch files also declare, and forwards them unconditionally — so a concrete
default here beats the YAML on every run even when the included launch file is written
correctly. Keep every `default_value` empty. See [Configuration and
parameters](../../README.md#configuration-and-parameters) for the mechanism.

## What it does *not* start

`manager`, `planner`, and `mpc` are **by design** launched separately, not from here.
They are per-workflow rather than part of base bring-up:

```bash
ros2 launch planner launch.py
ros2 launch manager launch.py      # master GUI + recorder
ros2 launch mpc launch.py          # separate workflow from plan-and-deploy
```

`mpc` and `manager` both publish `joint_space/target`, so run one or the other.

## CPU pinning

This package sets no CPU affinity itself — the included launch files do. The
workspace-wide core map, documented in a comment in `robot/launch/robot.py`:

| Core | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|
| Node | manager | ekf | mpc | robot | gui | pinn_fk | igtl | planner | emtracker |

Core 3 was otherwise unused, and the EKF was moved onto it because sharing core 7
with `pinn_fk` starved both Torch inference loops.

## Dependencies

`<exec_depend>` on `robot`, `emtracker`, and `igtlink_bridge`. Nothing is built.

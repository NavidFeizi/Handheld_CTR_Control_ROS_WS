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

All are forwarded to the included launch files.

| Argument | Default | Goes to |
|---|---|---|
| `Kp` | 30.0 | `robot` |
| `Ki` | 5.0 | `robot` |
| `maxVel` | `[3.0, 0.012, 3.0, 0.012]` | `robot` |
| `maxAcc` | `[10.0, 0.10, 10.0, 0.10]` | `robot` |
| `f_dot` | 0.2 | `robot` (EKF) |
| `host_name` | `/dev/ttyUSB1` | `emtracker` |
| `send_on_igtl` | `false` | `emtracker` |
| `enable_position_logging` | `false` | `emtracker` |
| `hostname_slicer` | `localhost` | `igtlink_bridge` |
| `port_slicer` | `18944` | `igtlink_bridge` |
| `hostname_module` | `10.15.232.114` | `igtlink_bridge` |
| `port_module` | `18975` | `igtlink_bridge` |
| `auto_connect` | `false` | `igtlink_bridge` |

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

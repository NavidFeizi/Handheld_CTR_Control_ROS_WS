# igtlink_bridge

OpenIGTLink bridge between the ROS graph and 3D Slicer. Forwards the CTR backbone
shape, the planned path, and target poses to Slicer for visualization, and converts
metres to millimetres at that boundary.

Needs no hardware, but does need a reachable OpenIGTLink peer.

## Quick start

```bash
source install/setup.bash
ros2 launch igtlink_bridge launch.py
```

Started automatically 20 s into `ctr_bringup`'s system bring-up. Slicer should be
listening on `localhost:18944` before you connect; see the [root
README](../../README.md) for the Slicer-side setup.

## Nodes

| Executable | Node name in code | Name at launch | Role |
|---|---|---|---|
| `bridge` | `igtlink_bridge` | `igtlink_bridge_slicer_node` | Slicer endpoint — the one that runs |
| `bridge` | `igtlink_bridge` | `igtlink_bridge_module_node` | Second endpoint, configured but **not started** — see [Status notes](#status-notes) |

Both entries run the same executable under different node names, so each picks up its
own parameter block.

## Launch file

`launch/launch.py`, CPU core 8.

| Argument | Default |
|---|---|
| `hostname_slicer` | `localhost` |
| `port_slicer` | `18944` |
| `hostname_module` | `10.15.232.114` |
| `port_module` | `18975` |
| `auto_connect` | `false` |

## Parameters

From `config/igtlink_params.yaml`, keyed per node name.

### `igtlink_bridge_slicer_node`

| Parameter | Default | Meaning |
|---|---|---|
| `hostname` | `localhost` | OpenIGTLink peer |
| `port` | 18944 | Slicer's default IGTL port |
| `convert_to_startrack` | `false` | Emit StarTrack-format messages |
| `auto_connect` | `false` | Connect at startup instead of waiting for the service |

### `igtlink_bridge_module_node`

| Parameter | Default |
|---|---|
| `hostname` | `10.15.232.114` |
| `port` | 18975 |
| `convert_to_startrack` | `true` |
| `auto_connect` | `false` |

The node also declares `sample_time` (5e-3 s) in code; it has no YAML entry.

## Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `shape/tube_1`, `shape/tube_2`, `shape/tube_3` | `std_msgs/msg/Float64MultiArray` |
| sub | `task_space/path` | `std_msgs/msg/Float64MultiArray` |
| sub | `task_space/target` | `interfaces/msg/Taskspace` |
| pub | `task_space/ct_path` | `std_msgs/msg/Float64MultiArray` |
| pub | `igtl_bridge/connected` | `std_msgs/msg/String` |

TF transforms are broadcast as well.

**Units:** everything on the ROS side is in metres. This node multiplies translations
by 1e3 on the way out, because Slicer works in millimetres. That conversion lives
here and nowhere else — do not pre-scale upstream.

`igtl_bridge/connected` reports connection state; `robot`'s GUI subscribes to it to
show whether Slicer is attached.

## Services

| Service | Type | Direction |
|---|---|---|
| `igtl_bridge/connect` | `std_srvs/srv/SetBool` | provided — connect or disconnect the IGTL session |
| `robot_config` | `interfaces/srv/Config` | called |
| `planner/command` | `interfaces/srv/Planner` | called |

```bash
ros2 service call /igtl_bridge/connect std_srvs/srv/SetBool "{data: true}"
```

With `auto_connect` false (the default), the bridge waits for this call, so you can
start Slicer after the ROS system is already up.

## Dependencies

OpenIGTLink 3.1, found at the hardcoded CMake path
`/usr/local/lib/igtl/cmake/igtl-3.1`. Upstream never tagged 3.1 — build from master,
which carries the 3.1 version that lands at that path.

## Tests

None. This package has no `test/` directory; its logic is mostly I/O against a live
IGTL peer, which the workspace's hardware-free suites deliberately avoid.

## Status notes

- **The module endpoint never starts.** `igtlink_bridge_module_node` is fully
  configured — launch arguments, a `Node` action, and its own parameter block — but
  `ld.add_action(module_node)` is commented out in `launch/launch.py`. Only the
  Slicer endpoint runs. Uncomment that line to enable it.
- No test coverage, unlike the other node packages in this workspace.

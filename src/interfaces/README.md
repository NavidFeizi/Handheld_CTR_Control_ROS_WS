# interfaces

Custom ROS 2 message and service definitions shared by every node in the workspace.
Pure `rosidl` package — no nodes, no library code.

Because everything depends on this package, **rebuild it first whenever a definition
changes**. After *deleting* a `.msg` or `.srv`, also purge `build/interfaces`:
`rosidl` does not garbage-collect generated code, so stale headers keep compiling
until the build directory is cleared.

```bash
colcon build --packages-select interfaces
```

## Messages

### `Jointspace.msg`

The joint-space workhorse. Every `joint_space/*` topic carries this.

| Field | Type | Meaning |
|---|---|---|
| `position` | `float64[4]` | Joint positions |
| `position_abs` | `float64[4]` | Absolute (uncoupled) positions |
| `velocity` | `float64[4]` | Joint velocities |
| `current` | `float64[4]` | Motor currents |

`position[4]` is in **wire order** `[α1, β1, α2, β2]` — α rotation (rad), β
translation (m). The PINN, Cosserat model, and planner all use a different *physics*
order. Always convert with
[`ctr_common::joint_conventions`](../ctr_common/README.md#joint_conventionshpp);
never hand-roll the permutation.

Used by: `robot`, `manager`, `mpc`, `planner`.

### `Taskspace.msg`

| Field | Type | Meaning |
|---|---|---|
| `p` | `float64[3]` | Position |
| `h` | `float64[4]` | Orientation (quaternion) |
| `q` | `float64[3]` | Translational velocity |
| `w` | `float64[3]` | Angular velocity |
| `f` | `float64[3]` | Tip force |

Positions are in metres on the wire. Used by: `emtracker`, `robot`, `manager`, `mpc`,
`planner`, `igtlink_bridge`.

### `Force.msg`

| Field | Type |
|---|---|
| `x`, `y`, `z`, `magnitude` | `float64` |

The EKF tip-force estimate. Published by `robot`'s `ekf_node`; consumed by `mpc`,
`planner`, `manager`, and `robot`'s `pinn_fk` and GUI.

### `Status.msg`

Robot health and mode, published by `ctr_robot`.

| Field | Type | Meaning |
|---|---|---|
| `enable`, `encoder`, `reached` | `bool[4]` | Per-joint drive state |
| `engaged`, `ready_to_engage` | `bool` | Collet state |
| `locked`, `control_mode` | `int64` | Lock state, active control mode |
| `head_attached`, `trans_limit_en`, `procedure` | `bool` | Head presence, translation limiting, procedure active |
| `min_pos_limit`, `max_pos_limit` | `float64[4]` | Per-joint position limits |
| `cpu_temp`, `winding_temp` | `float64[4]` | Drive temperatures |

Used by: `robot`, `manager`, `igtlink_bridge`.

### `Interface.msg`

| Field | Type | Meaning |
|---|---|---|
| `interface_key` | `bool[7]` | The handheld unit's seven buttons |

Used by: `robot`, `manager`.

### `EKFResidual.msg`

| Field | Type | Meaning |
|---|---|---|
| `x`, `y`, `z`, `pos_mag` | `float64` | Position residual and its magnitude |
| `theta_x`, `theta_y`, `theta_z`, `orientation_mag` | `float64` | Orientation residual and its magnitude |

Diagnostic output of the EKF — a growing residual means the PINN prediction and the
tracker have diverged. Used by: `robot` (`ekf_node` publishes, GUI displays).

## Services

### `Config.srv`

```
string command
int64 value 0
---
bool success
string message
```

The general command channel. `robot_config` and `robot_enable` both use it, with the
action selected by the `command` string — this is how homing and collet operations
are invoked. See [`robot`](../robot/README.md#homing-and-collets) for the accepted
command strings.

Used by: `robot`, `manager`, `planner`, `igtlink_bridge`.

### `Planner.srv`

```
string command
float64[3] value
---
bool success
string message
float64 value
```

Backs `planner/command`. `value` in the request is a target point. Note the response
`value` is a scalar, unrelated to the request's array.

Used by: `planner`, `manager`, `igtlink_bridge`.

### `Recording.srv`

```
string command
float64 duration
string name
---
bool success
string message
```

`name` selects the session folder under `Output_Files/`. Used by: `manager`, `mpc`.

### `Transformation.srv`

```
---
float64[16] transformation_matrix
```

Empty request. Returns a 4×4 homogeneous transform flattened row-major. Backs
`get_transformation` in `emtracker`.

## Status notes

- `package.xml` is stale relative to the rest of the workspace: version `0.0.0` (all
  other packages are `1.0.0`), a placeholder `<license>TODO: License declaration`,
  and a different maintainer address. It predates the 2026-08 refactor.
- `Taskspace.msg` carries a commented-out legacy block of `backbone_x/y/z[50]`
  fields. Backbone shape now travels on the `shape/tube_*` topics as
  `Float64MultiArray`, not in this message.

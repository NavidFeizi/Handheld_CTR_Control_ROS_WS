# robot

The hardware layer and operator GUI for the handheld CTR, plus the two model-based
nodes that run beside them: PINN forward kinematics and EKF tip-force estimation.

`ctr_robot` drives the four joint motors through `ICtrJointGroup` (implemented by
`ctr_robot_driver`) and is the only node in the workspace that talks to the CAN bus.
Everything else here is pure software.

## Quick start

```bash
source install/setup.bash
ros2 launch robot robot.py       # robot_node + GUI + pinn_fk + ekf_node
```

There is no `launch.py` in this package — the three launch files are `robot.py`,
`sim.py`, and `ekf.py`. Normally you do not launch this package directly; the system
bring-up in `ctr_bringup` includes it after the EM tracker has initialized.

## Nodes

| Executable | Node name in code | Name at launch | Role |
|---|---|---|---|
| `ctr_robot` | `ctr_robot` | `robot_node` | CANopen hardware layer: joint targets, feedback, homing, collets |
| `qt_gui` | `qt_gui_node` | `gui_node` | Operator GUI |
| `pinn_fk` | `forward_kinematics_node` | `pinn_fk_node` (`robot_sim_node` in `sim.py`) | PINN forward kinematics and backbone shape |
| `ekf_node` | `kalman_filter_node` | `ekf_node` | EKF tip-force estimation |
| `cosserat_fk` | `cosserat_fk_node` | — | Cosserat-rod forward kinematics. **Not launched** — see [Status notes](#status-notes) |

The node name declared in the source differs from the launch name for three of these.
When calling `ros2 param set`, use the **launch** name (`robot_node`, `gui_node`,
`pinn_fk_node`, `ekf_node`) — that is what the parameter file keys on.

## Launch files

| File | Starts | Arguments |
|---|---|---|
| `robot.py` | `robot_node`, `gui_node` (+5 s), `pinn_fk_node`, `ekf_node` | `Kp`, `Ki`, `maxVel`, `maxAcc`, `f_dot` |
| `sim.py` | `pinn_fk` as `robot_sim_node`, `ekf_node` — no hardware | same names |
| `ekf.py` | `ekf_node` only | `f_dot` |

The arguments mean the same thing wherever they appear:

| Argument | Default | Overrides |
|---|---|---|
| `Kp` | *(none)* | `robot_node.Kp` |
| `Ki` | *(none)* | `robot_node.Ki` |
| `maxVel` | *(none)* | `robot_node.maxVel` |
| `maxAcc` | *(none)* | `robot_node.maxAcc` |
| `f_dot` | *(none)* | `ekf_node.f_dot` |

None of these carries a default: unset they are dropped and `config/robot_params.yaml`
supplies the value, identically for all three launch files. Pass one explicitly to
override it for a single run:

```bash
ros2 launch robot ekf.py f_dot:=0.5
ros2 launch robot robot.py maxVel:='[1.0, 0.006, 1.0, 0.006]'
```

Editing the YAML needs `colcon build --packages-select robot` — launch reads the
installed `share/` copy. See [Configuration and
parameters](../../README.md#configuration-and-parameters) for the mechanism.

`sim.py` declares `Kp`/`Ki`/`maxVel`/`maxAcc` too, but no node in that file consumes
them — it starts no `robot_node`.

### CPU pinning

Every node is pinned with `taskset`. The workspace-wide core map, documented in a
comment in `robot.py`:

| Core | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|
| Node | manager | ekf | mpc | robot | gui | pinn_fk | igtl | planner | emtracker |

The EKF was moved to core 3 — otherwise unused — because sharing core 7 with
`pinn_fk` starved both Torch inference loops. Keep them apart.

## Parameters

Single source of truth is `config/robot_params.yaml`, shared by all three launch files.
The defaults below are the YAML's values and are what the nodes actually receive; no
launch file shadows them.

### `robot_node`

| Parameter | Default | Meaning |
|---|---|---|
| `Kp` | 30.0 | Position-loop proportional gain |
| `Ki` | 5.0 | Position-loop integral gain |
| `maxVel` | `[3.0, 0.012, 3.0, 0.012]` | rad/s, m/s per joint |
| `maxAcc` | `[10.0, 0.10, 10.0, 0.10]` | rad/s², m/s² per joint |
| `can_interface` | `can0` | SocketCAN interface |
| `canopen_dir` | `""` | Empty → installed `share/ctr_robot_driver/canopen` |
| `encoder_memory_dir` | `""` | Empty → `$HOME/Documents/handheld_CTR/encoder_memory/` |
| `boot_max_attempts` | 10 | `master.Reset()` retries before bring-up is declared failed |
| `boot_timeout_s` | 5.0 | Seconds to wait per boot attempt for all four drives |

Raise `boot_max_attempts` if a drive is a slow starter: bring-up gives up after
`boot_max_attempts × boot_timeout_s` and then reports the robot as not connected.

### `pinn_fk_node`

| Parameter | Default | Meaning |
|---|---|---|
| `sample_time` | 0.025 | s |
| `num_backbone` | 50 | Backbone nodes per tube shape |
| `model_name` | `ctr_8x91_0.18_tanh_9K_9K_50K_v3` | Model in the PINN pool |
| `models_dir` | `""` | Empty → installed `share/ctr_kinematics_pinn/models` |
| `q0` | `[-0.100, -0.055, 0.0, 0.0]` | Initial joint configuration |

### `ekf_node`

| Parameter | Default | Meaning |
|---|---|---|
| `sample_time` | 0.025 | s |
| `model_name` | `ctr_8x91_0.18_tanh_9K_9K_50K_v3` | Model in the PINN pool |
| `models_dir` | `""` | Empty → installed pool |
| `f_dot` | 0.2 | N/s — process-noise rate on the force state |
| `R` | 36 elements | 6×6 measurement covariance, row-major |

`R` lives **only** in the YAML. It was previously rebuilt with numpy in three
separate launch files, one of which was malformed; the value in the YAML is the
effective production matrix (base `R` with the lower 3×3 block scaled by 5e4, then
the whole thing by 1.5). Do not re-derive it in a launch file.

## Topics

| Node | Direction | Topic | Type |
|---|---|---|---|
| `ctr_robot` | sub | `joint_space/target` | `interfaces/msg/Jointspace` |
| `ctr_robot` | sub | `joint_space/manual_vel` | `interfaces/msg/Jointspace` |
| `ctr_robot` | sub | `task_space/feedback/base_tool` | `interfaces/msg/Taskspace` |
| `ctr_robot` | pub | `robot_status` | `interfaces/msg/Status` |
| `ctr_robot` | pub | `manual_interface` | `interfaces/msg/Interface` |
| `ctr_robot` | pub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| `pinn_fk` | sub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| `pinn_fk` | sub | `task_space/force_estimate` | `interfaces/msg/Force` |
| `pinn_fk` | pub | `task_space/sim_out` | `interfaces/msg/Taskspace` |
| `pinn_fk` | pub | `shape/tube_1`, `shape/tube_2`, `shape/tube_3` | `std_msgs/msg/Float64MultiArray` |
| `ekf_node` | sub | `/task_space/feedback/base_tool` | `interfaces/msg/Taskspace` |
| `ekf_node` | sub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| `ekf_node` | pub | `task_space/force_estimate` | `interfaces/msg/Force` |
| `ekf_node` | pub | `EKF/computation_time` | `std_msgs/msg/Float64` |
| `ekf_node` | pub | `EKF/residual_error` | `interfaces/msg/EKFResidual` |
| `qt_gui` | sub | `joint_space/feedback`, `robot_status`, `manual_interface`, `/task_space/feedback/base_tool`, `task_space/force_estimate`, `EKF/residual_error`, `igtl_bridge/connected` | — |
| `qt_gui` | pub | `joint_space/manual_vel` | `interfaces/msg/Jointspace` |
| `cosserat_fk` | sub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| `cosserat_fk` | pub | `shape/tube_1`, `shape/tube_2`, `shape/tube_3` | `std_msgs/msg/Float64MultiArray` |

Tube shapes are published in **metres**; `igtlink_bridge` converts to millimetres at
the Slicer boundary.

## Services

| Service | Type | Node |
|---|---|---|
| `robot_config` | `interfaces/srv/Config` | provided by `ctr_robot` |
| `robot_enable` | `interfaces/srv/Config` | provided by `ctr_robot` |
| `robot_config`, `robot_enable` | `interfaces/srv/Config` | called by `qt_gui` |
| `freeze_robot` | `std_srvs/srv/SetBool` | called by `qt_gui` (served by `emtracker`) |
| `igtl_bridge/connect` | `std_srvs/srv/SetBool` | called by `qt_gui` (served by `igtlink_bridge`) |

### Homing and collets

**There is no `/homing` service.** Homing, collet, and mode changes are dispatched
through the `command` string field of `robot_config`:

```bash
ros2 service call /robot_config interfaces/srv/Config "{command: 'findLinearHome', value: 0}"
```

Accepted commands: `findLinearHome`, `findRotaryHome`, `findRotaryHomeAndGoHome`,
`goHome`, `engageCollets`, `disengageCollets`, `lockCollets`, `unlockCollets`,
`engageAndUnlock`, `lockAndDisengage`, `startProcedure`, `endProcedure`,
`toggleEnable`, `disable`, `setCtrlMode`, `setTransLimMode`.

Run these from the GUI in normal operation. The homing order matters — **linear
first, then rotary**; the [root README](../../README.md) has the full operator
procedure.

## Hardware

`ctr_robot` needs a live SocketCAN bus (`can0` by default) with the four drives
powered. CAN bring-up, the IXXAT driver, and homing are covered in the
[root README](../../README.md).

Bus bring-up happens in `main()` before the executor spins; on failure the node logs
`RCLCPP_FATAL` but stays alive, so a running `robot_node` is not by itself proof that
the hardware is connected — check `robot_status`.

`qt_gui`, `pinn_fk`, `ekf_node`, and `cosserat_fk` need no hardware. Use `sim.py` to
run the model nodes alone.

## Threading

`qt_gui` runs a `MultiThreadedExecutor` beside the Qt thread. Widget mutations from
ROS callbacks must go through `QMetaObject::invokeMethod(..., Qt::QueuedConnection)`;
cross-thread state is atomic or mutex-snapshotted. The GUI no longer blocks in its
constructor waiting for services — 500 ms readiness timers gate the controls instead.

## Tests

```bash
colcon test --packages-select robot --ctest-args -R test_quat_utils
```

`test/test_quat_utils.cpp` covers the quaternion helpers extracted from the EKF into
`include/robot/quat_utils.hpp` — multiply, inverse, rotate, and rotation-vector
conversion — so they can be tested without Torch or hardware.

## Status notes

- **`cosserat_fk` is built and installed but never launched.** The
  `ld.add_action(cosserat_fk_node)` line is commented out in `robot.py`; the
  PINN-based `pinn_fk` superseded it. It remains useful as a reference
  implementation — see [`ctr_cosserat`](../ctr_cosserat/README.md).
- **`f_dot` no longer disagrees across launch files.** `robot.py`, `sim.py` and
  `ekf.py` used to hardcode 0.2 / 1.0 / 0.1 as launch-argument defaults that silently
  beat the YAML; all three now default to unset, so every entry point gets the YAML's
  0.2. Pass `f_dot:=` explicitly to deviate for a single run.
- **A drive that refuses to enable is now visible.** `robot_status` carries
  `bool[4] enable_fault`; `ctr_robot` logs it at `ERROR` on the false→true edge and
  `qt_gui` shows the joint as `FAULT` (red) instead of an ordinary `OFF`. Previously
  the latch existed only inside the driver and nothing above it read the flag, so a
  single joint failing the CiA-402 transition looked identical to one that was simply
  disabled. See [`ctr_robot_driver`](../ctr_robot_driver/README.md).

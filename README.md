<div align="center">

# CTR ROS2 Packages

</div>


---

## Repository Structure

```
Handheld_CTR_Control_ROS_WS/
├── src/
│   ├── interfaces/           # Custom ROS2 messages and services
│   ├── ctr_common/           # Shared conventions: joint order, CSV parsing, data-root/model paths
│   ├── ctr_cosserat/         # ROS-free Cosserat-rod CTR model (static lib)
│   ├── ctr_kinematics_pinn/  # ROS-free TorchScript PINN inference + unified models/ pool
│   ├── ctr_robot_driver/     # ROS-free CANopen joint driver (lely) + dcfgen config in share/
│   ├── ctr_bringup/          # System bring-up launch (EM tracker → robot → IGTL bridge)
│   ├── emtracker/
│   │   ├── lib_emtracker/                    # NDI Aurora driver library (+ vendored NDI API)
│   │   └── src/EMtracker_node.cpp            # track
│   ├── igtlink_bridge/
│   │   └── src/igtl_bridge_node.cpp          # bridge
│   ├── manager/
│   │   ├── include/                          # Qt/ROS manager headers (+ csv_path_io.hpp)
│   │   └── src/
│   │       ├── recorder_node.cpp             # record
│   │       ├── master_node.cpp
│   │       └── master_qt_gui.cpp             # master
│   ├── mpc/
│   │   ├── ctr_mpc/                          # QP MPC library (OSQP)
│   │   └── src/mpc_node.cpp                  # mpc
│   ├── planner/
│   │   ├── motion_planning/                  # OMPL planner library (+ DeploymentSchedule.hpp)
│   │   └── src/planner_node.cpp              # plan
│   ├── robot/
│   │   ├── include/robot/                    # quat_utils.hpp (EKF helpers)
│   │   └── src/
│   │       ├── robot_node.cpp                # ctr_robot (drives hardware via ICtrJointGroup)
│   │       ├── qt_node.cpp                   # qt_gui
│   │       ├── cosserat_fk_node.cpp          # cosserat_fk
│   │       ├── pinn_fk_node.cpp              # pinn_fk
│   │       └── ekf_node.cpp                  # ekf_node
│   └── target_gen/                           # target generation utilities (plain Python)
├── launch/                                   # shim → ctr_bringup's system_bringup.launch.py
├── docker/                                   # build-verification image + check.sh (see docker/README.md)
├── Input_Files/                              # target CSV inputs
├── Output_Files/                             # recorder sessions (new sessions untracked)
├── 3DSlicer/                                 # Slicer scenes, transforms, and models
└── Shared_Files/                             # planner ↔ manager path channel (plannedPath.csv)
```

Runtime data directories resolve via the `data_root` parameter, the
`CTR_DATA_ROOT` env var, or (fallback) the legacy workspace-layout climb.
Node parameters follow the rule below.

## Configuration and parameters

Each package with parameters carries one `config/*_params.yaml`, installed to
`share/<pkg>/`, and its launch file loads it. **That YAML is the single source of
truth.** Launch arguments declare *no* default value:

- **unset** → the argument is dropped and the YAML value stands;
- **typed on the command line** → it overrides the YAML, for that run only.

```bash
ros2 launch emtracker launch.py                          # host_name from emtracker_params.yaml
ros2 launch emtracker launch.py host_name:=/dev/ttyUSB0  # one-off override
```

### Why the arguments have no defaults

`Node(parameters=[params_file, {...}])` hands rclcpp each list entry as a separate
`--params-file`, applied in order — **the last one wins**. The inline dict is therefore
always applied after the YAML. An argument declared as
`DeclareLaunchArgument('host_name', default_value='/dev/ttyUSB1')` always has a value,
even when nothing is typed, so that default would silently overwrite the YAML on *every*
launch and make YAML edits look ignored. That was a real bug in this workspace.

Each launch file instead declares `default_value=''` and resolves the arguments inside an
`OpaqueFunction`, through a small local helper:

```python
def _parameters(context, params_file, mapping):
    overrides = {}
    for param, arg in mapping.items():
        config = LaunchConfiguration(arg)
        if config.perform(context) != '':
            overrides[param] = ParameterValue(config)
    return [params_file, overrides] if overrides else [params_file]
```

Empty arguments are dropped, so `parameters=` is just `[params_file]` on a plain launch.
Survivors stay `ParameterValue(LaunchConfiguration(...))` rather than the performed
string, which keeps launch_ros's type inference: `Kp:=99.0` reaches the node as a double
and `q0:='[-0.1, -0.05, 0.0, 0.0]'` as a double array. Storing the plain string writes
`Kp: '99.0'` into the generated params file and rclcpp rejects the type.

**When editing a launch file, never give a parameter-bearing argument a concrete
default** — including in `ctr_bringup`, which forwards all of its arguments
unconditionally to the launch files it includes.

### Editing a YAML takes a rebuild

Launch files read the **installed** copy under `share/`, not `src/`. After editing
`src/<pkg>/config/*_params.yaml`:

```bash
colcon build --packages-select <pkg>
source install/setup.bash
```

A workspace built with `--symlink-install` picks the edit up without rebuilding.

### Checking what a node actually got

```bash
ros2 launch <pkg> <file> --show-args   # empty defaults; each description names its YAML
ros2 param get <node_name> <param>     # the value the running node received
```

Use the **launch** node name (`emt_node`, `robot_node`, `ekf_node`, …), which is what the
parameter files key on.

## Package Documentation

Every package carries its own README with its nodes, launch arguments, parameters,
topics, services, and tests.

| Package | Kind | Documentation |
|---------|------|---------------|
| `interfaces` | messages | [src/interfaces/README.md](src/interfaces/README.md) |
| `ctr_common` | shared lib | [src/ctr_common/README.md](src/ctr_common/README.md) |
| `ctr_cosserat` | static lib, ROS-free | [src/ctr_cosserat/README.md](src/ctr_cosserat/README.md) |
| `ctr_kinematics_pinn` | header-only lib, ROS-free | [src/ctr_kinematics_pinn/README.md](src/ctr_kinematics_pinn/README.md) |
| `ctr_robot_driver` | static lib, ROS-free | [src/ctr_robot_driver/README.md](src/ctr_robot_driver/README.md) |
| `ctr_bringup` | launch only | [src/ctr_bringup/README.md](src/ctr_bringup/README.md) |
| `robot` | nodes | [src/robot/README.md](src/robot/README.md) |
| `emtracker` | node | [src/emtracker/README.md](src/emtracker/README.md) |
| `manager` | nodes | [src/manager/README.md](src/manager/README.md) |
| `planner` | node | [src/planner/README.md](src/planner/README.md) |
| `mpc` | node | [src/mpc/README.md](src/mpc/README.md) |
| `igtlink_bridge` | node | [src/igtlink_bridge/README.md](src/igtlink_bridge/README.md) |
| `target_gen` | Python, not a ROS package | [src/target_gen/README.md](src/target_gen/README.md) |

The build-verification image has its own notes in
[docker/README.md](docker/README.md).

## Dependencies

| Library | Purpose | Minimum version |
|---------|---------|-----------------|
| [LibTorch](https://pytorch.org/get-started/locally/) (CPU) | PINN inference & auto-diff Jacobians | 2.9.0 |
| [OMPL](https://ompl.kavrakilab.org/) | Sampling-based motion planning | 2.0 (AORRTC) |
| [Blaze](https://bitbucket.org/blaze-lib/blaze) | Dense/sparse linear algebra | 3.8 |
| [Boost](https://www.boost.org/) | `serialization`, `filesystem`, `algorithm` | 1.74 |
| [TBB](https://github.com/oneapi-src/oneTBB) | Parallel neighbour-graph construction | 2021 |
| [FCL](https://github.com/flexible-collision-library/fcl) | Collision geometry | 0.7 |
| [OpenMP](https://www.openmp.org/) | CPU parallelism | — |
| [LAPACK / BLAS](https://netlib.org/lapack/) | Matrix factorizations (pseudo-inverse) | — |
| [nlohmann/json](https://github.com/nlohmann/json) | Model parameter deserialization | 3.x |
| [pugixml](https://pugixml.org/) | OMPL scene description | 1.x |
| CMake | Build system | 3.22 |

### LibTorch installation note

After downloading LibTorch to `/usr/local/libtorch`, add the following to your `~/.bashrc`:

```bash
export Torch_DIR="/usr/local/libtorch/share/cmake/Torch"
```

> **Important:** If you have a system PyTorch package installed alongside a manual LibTorch build, conflicting headers in `/usr/local/include/` can cause subtle compilation errors. `ctr_kinematics_pinn/cmake/torch_pin.cmake` handles this by pinning `Torch_DIR` explicitly and pre-populating `c10_LIBRARY`; the pin also runs for every package that `find_package`s `ctr_kinematics_pinn`, so no manual `Torch_DIR` export is required (override the prefix with `-DCTR_TORCH_DIR=` if needed).

---


## Build Packages Instruction

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Compile checks without ROS (Docker)

On a machine without ROS 2, `docker/check.sh` runs the same full colcon build
inside a `ros:jazzy` container mirroring the lab's `/usr/local` dependency
layout — see `docker/README.md` for the pinned versions and usage.

### Tests

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select ctr_common emtracker manager robot ctr_robot_driver mpc planner
colcon test-result
```

The suites are hardware-, Torch-, and OMPL-free (joint conventions, CSV
parsing, Butterworth/rigid transforms, quaternion utils, CiA-402 helpers,
MPC QP behaviour incl. warm-start equivalence, deployment scheduling).

### Lab-machine smoke checklist (after the 2026-08 refactor)

One-time verification on the real robot, in order:

1. CAN bring-up and homing per the sections below (the driver now reads
   `master.dcf`/`master.bin` from `share/ctr_robot_driver/canopen`, not the
   build tree; `canopen_dir`/`can_interface`/`encoder_memory_dir` are
   parameters of `robot_node`).
2. EM tracker: first run seeds `$HOME/Documents/handheld_CTR/emtracker_config/`
   from the installed config; registration results now land there (writable),
   not in the source tree.
3. EKF convergence and MPC tracking/cycle time: robot & mpc moved from the
   ambient system Torch to the pinned LibTorch 2.9.0, the OSQP warm start is
   now actually used, and `-march=native` no longer leaks into the OSQP ABI.
4. `f_dot` is single-sourced to 0.2 N/s (production value; robot.py's shadowed
   0.1 default lost).
5. Shared_Files round-trip: plan → `plannedPath.csv` (written atomically) →
   deployment; Slicer's RobotTrajectory command path is functional for the
   first time (service type fixed).

### Lab-machine azimuth sweep (after the 2026-08-31 α-domain fix)

The α-domain unification (α₂ absolute ±1.5π, α₁ relative α₂ ± π — see CLAUDE.md
"One definition of a legal joint configuration") specifically targets the failures
where probe targets near the robot_base **+y** axis either never produced a plan or
produced plans that executed and landed away from the probe. Acceptance test:

1. Bring up the full system (`system_bringup`, `planner`, `manager`), enable, home,
   start the procedure as usual.
2. Place the EM probe at a fixed radius at ~8 bearings around the tool axis in 45°
   steps, **including the +y sector that used to fail**. For each bearing:
   - The pre-rotation must settle (watch the new
     `Pre-rotating tubes: bearing ... -> command alpha ...` line — the commanded α is
     now the nearest representative, so it must never swing ~350° for a small bearing
     change).
   - A plan must be produced without the 5 s reject-retry loop
     (`Plan rejected: ...` repeating means a gate is still closed — the message now
     names it, including the "rotary axes are wound up" diagnosis).
   - Watch for the new WARNs; each one is a specific defect signature:
     `IK queried the network OUTSIDE its trained alpha domain` (extrapolation
     tripwire — should never fire now), `clampJointPositions` clamps in `pinn_fk`,
     zero-norm quaternion in `ekf_node`, joint-target limit warnings in `robot_node`.
   - Deploy, then read the `Deployment complete:` line — it prints target vs EM tip
     vs PINN tip. |EM tip − target| < 3 mm is the pass criterion; if the EM and PINN
     tips agree but both miss the target, suspect registration, not the model.
3. Collect `Output_Files/diagnostics/planner/<ts>/planner_diag.csv` and
   `Output_Files/diagnostics/manager/<ts>/manager_diag.csv` (one row per request /
   gate event / deployment; correlate by wall time) together with the usual
   `log/Robot/*.txt`.
---


## Setup CANopen Connection to the Robot

### To set up the CANopen connection using the Kvaser interface (current handheld CTR), follow these steps:

1. **Load the CANopen driver:**
   ```bash
   modprobe kvaser_usb
   ```

2. **Configure CAN interface (for Kvaser):**
   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set can0 up
   ```

### To set up the CANopen connection using the IXXAT interface (currently attached to the catheter setup), follow these steps:

1. **Load the CANopen driver:**
   ```bash
   modprobe ix_usb_can
   ```
   If you encounter an error, the IXXAT SocketCAN driver is not installed. Install the version compatible with your Linux kernel (IXXAT_SocketCAN_2_0_378_Modified_2023-03-15).

2. **Configure CAN interface (for IXXAT):**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set can0 down
   sudo ip link set can0 type can loopback on
   sudo ip link set can0 up
   ```

### Monitor CAN connection:
   Open a terminal and run:
   ```bash
   candump can0
   ```

### Send reset command to check all nodes:
   Open a separate terminal and run:
   ```bash
   cansend can0 000#8200
   ```
   If you see a returned packet similar to the line below (echo of the reset command), the setup is ready. Otherwise, loopback may not be enabled. In some cases, unplugging and reconnecting the USB interface resolves the issue.
   ```plaintext
   can0  000   [2]  82 00
   ```
---

## Setup EM tracker USB Connection

1. **Identify connected USB ports:**
   ```bash
   ls /dev/tty*
   ```

2. **Grant access permission to the port:**
   ```bash
   sudo chmod a+rw /dev/ttyUSB*
   ```
   Replace `*` with the number of the connected USB port, usually `0`.

   You may need to adjust the port number in `launch/system_bringup.launch.py` and rebuild the packages.

---
## Usage

### Setup 3D Slicer

Open 3D Slicer with FlexibleToolViz.
Start the server under the IGT/FlexibleToolViz module.
Import `./3DSlicer/Scene.mrml` into the scene.
Under the IGT/FlexibleToolViz module, set the base transform to `Transform_Shape` and click **Visualize Tool**.

## Launching the robot and EM tracker

To set up the robot node,  EM tracker node, and igtlink node to establish connection to 3D Slicer. 

Make sure EM tracker USB access is granted and the CANopen connection is established. Also make sure the motor section is attached to the robot.

Open a terminal and navigate to the workspace, then run:
  
   ```bash
   source ./install/setup.bash
   ros2 launch launch/system_bringup.launch.py
   ```

   This starts the EM tracker node first. You will hear the EM tracker booting sequence. A few seconds later, the robot node starts and the CANopen connection is established. The Handheld CTR GUI then appears, followed by IGTLink connection setup. If everything is successful, you should see EM tracker status as **Active** and IGTLink status as **Connected** in the Handheld CTR GUI. You should also see joint information in the GUI; if not, the robot node connection is likely not established correctly.

At this stage, you can set encoder homing using the GUI. Follow the steps below:
1. Click **Find** in the GUI. This moves the linear joints to the proximal mechanical stops to set the linear encoders. Always perform this step first before setting rotational homing. Monitor stage spacing carefully; the distance between linear stages must stay within limits. If limits are exceeded, the inner tube may buckle when pushed into the middle tube. At the end of this stage, the rotary joints perform a full rotation to ensure couplings are engaged. If you detach the motor section, repeat this step to ensure couplings are engaged and linear joints are reset.
2. Click **Find Rotary Home**. This fully extends the linear joints and rotates the rotary joints to set rotary encoder homing using EM tracker feedback. The EM tracker must be active before this step, and the tip EM sensor must be installed.
3. Click **Go To Home**. This moves the robot to the home position (all tubes flush, curvature pointing downward). Once homed, the tip position reading in the GUI should be close to 60.0 mm in the Z direction. If not, the tip sensor is likely not exactly at the tip. Manually adjust tip sensor position by moving the wire in/out from the proximal end until the GUI reading is around 60.0 mm in Z.

The robot is now ready to use. You should see the robot body and tubes in 3D Slicer. If you touch and bend the tubes, you should see live shape updates in 3D Slicer. If a probe is connected, it should also appear in 3D Slicer.

For manual joint control, set the **Control Mode** radio button to **Manual**. You can then use the `arrow keys` and `AWSD` keys to control linear and rotary joints. Keep the **Trans Limit** radio button set to **ON** to avoid exceeding linear stage limits and colliding with mechanical stops. Use **OFF** only for special cases, and with caution.

## Launching the planner and manager

### Which GUI owns what

Two Qt windows come up, and they are not interchangeable:

| Window | Package / node | Owns |
|---|---|---|
| **Handheld CTR** | `robot` / `gui_node` | Homing (**Find**, **Find Rotary Home**, **Go to Home**), collet engage/lock, control-mode radios, manual jogging, EM tracker + IGTLink status |
| **Handheld CTR Master** | `manager` / `master_node` | The whole plan-and-deploy workflow: target selection, **Start Procedure**, **Auto Insert/Retract**, planner status, recording |

Planner status is shown **only** in the master window.

### There is no "plan" button

This surprises people, so it is worth stating plainly: nothing in either GUI directly
commands a plan. `master_node` runs a 100 ms control loop that issues
`planner/command` by itself, but only once **every** one of these holds at the same time:

| Precondition | How to satisfy it | Where it comes from |
|---|---|---|
| All five services up | Run all three launch files (below) | readiness timer, logs "All robot/planner/recorder services are ready" |
| Procedure active | Click **Enable**, then **Start Procedure** | `robot_status.procedure`; Robot info table shows Procedure = ON |
| Mode = "Select Target" | Click the **Select Target** radio | the radio maps to the internal `Planner` mode |
| Tubes aimed at the target | Automatic — wait for the rotary joints to swing round | within 10° on both rotary joints |
| All four joints reached | Automatic — wait for motion to settle | `robot_status.reached[0..3]` |
| Target actually moved | Move the probe more than 2 mm, or click **Next ►** in CSV mode | compared against the last planned target |

When a plan is requested you will see `Planner called.` in the `master` terminal and
`Planning with force estimate: f = [...]` in the `planner` terminal. If neither appears,
the master node now prints a throttled `Planner idle: <reason>` naming the gate that is
still closed — read that line rather than guessing.

### Launch sequence

Bring these up in order, each in its own terminal, after the robot and tracker are
already running and homed.

1. **Planner:**

   ```bash
   source ./install/setup.bash
   ros2 launch planner launch.py
   ```

   You should see: `Path Planner Node has been initialized.`

2. **Manager:**

   ```bash
   source ./install/setup.bash
   ros2 launch manager launch.py
   ```

   This starts both `master` (the GUI) and `record`. The master window appears, and
   within about a second the terminal prints:

   ```
   [master_node]: All robot/planner/recorder services are ready
   ```

   > **This line is not optional.** `master_node` gates its *entire* control loop on all
   > five services: `robot_config` and `robot_enable` (from `ctr_robot`), `planner/command`
   > (from `planner`), `freeze_robot` (from **emtracker**, not the robot), and `recording`
   > (from `record`). If any one is missing, no button and no automatic behaviour in the
   > master GUI does anything. The terminal now names the missing service every 5 s.

3. At this stage, retract the linear stages to the home position and click **Freeze
   Robot**. This stops updates of robot body position, mitigating EM tracking sensor
   deviation during actuation caused by magnetic interference from the middle tube stage.
   After clicking **Freeze Robot**, do not move the robot or the EM tracker field
   generator.

### Set target, plan, and deploy

1. Click **Enable** (the button reads **Disable** once the drives are on).
2. Click **Start Procedure**. Confirm the Robot info table shows **Procedure = ON** and
   the robot GUI's **Control Mode** switches to **Position**. Without this step nothing
   below has any effect.
3. Select the **Select Target** radio.
4. Move the probe to the desired target position. The rotary joints align toward the
   target angle, then the planner runs and the generated path from the current tip
   position to the target is shown in 3D Slicer.
5. Once the path is satisfactory, switch the Mode radio to **Deployment** and click
   **Auto Insert** to insert all the way to the target — or use the insert/retract
   physical buttons on the robot for step-by-step motion.

To read targets from a file instead of the probe, click **Toggle: Probe Mode** to switch
to **CSV Mode**. Targets are then read one by one from
`Input_Files/random_interior_points.csv` (set by the `targets_csv` parameter), and
**◄ Previous** / **Next ►** step through them. CSV mode does not need the probe sensor.

### Troubleshooting: "nothing happens"

The master node prints a throttled reason whenever the control loop is idle. Map it as
follows:

| Message | Meaning and fix |
|---|---|
| `Control loop idle: waiting on services: ...` | The named node is not running. Start it; check the `planner` and `record` terminals. |
| `Control loop idle: robot is not in Procedure` | Click **Enable**, then **Start Procedure**. |
| `Control loop idle: high-level mode is None` | The auto test left the mode cleared. Click the **Select Target** or **Deployment** radio to re-arm. |
| `Planner idle: rotating tubes toward the target bearing` | Normal for a few seconds. If the joints are not physically moving, a drive is not enabled — look for `drive enable fault` errors. |
| `Planner idle: joints have not reached their targets` | Motion has not settled, or a joint is stalled/limited. The message lists which joints. |
| `Planner idle: a plan request is still outstanding` | A solve is in flight (~3 s). If it never clears, the planner died; the request is abandoned after `planner_timeout_s` (15 s). |
| `Planner idle: target unchanged` | Move the probe further, or use **Next ►** in CSV mode. |
| `Planner idle: last plan was rejected - retrying ...` | The planner refused the last request; the same target is re-sent after `plan_retry_cooldown_s` (5 s). The reason is on the `Plan rejected:` line just above, and in the `planner` terminal. |
| `Plan rejected: Start state is invalid! q = [...]` | The current joint configuration is outside what the planner accepts. The message prints the live `q` and the active β₁/β₂ bounds and stage clearance — compare them: β₁ must satisfy both its own bounds and `β₁ ≤ β₂ − clearance`. |
| `Plan rejected: IK error ... exceeds the ... limit` | A solution was found but its tip error is above `k_ik_error_threshold` (3 mm). The target is likely outside the reachable workspace. |
| `Deployment idle: no waypoint list` | You are on the **Deployment** radio, which never requests a plan. Switch to **Select Target** and let the planner run first. |
| `Deployment idle: holding at waypoint i/N` | Normal. Hold an insert/retract button on the robot, or click **Auto Insert** / **Auto Retract**. |
| `Force-drift replanning suppressed after N consecutive rejections` | The planner rejected several replans in a row. Deployment continues on the previous plan; it re-arms when the drift halves or a plan is accepted. |
| `Force-drift replanning is inactive: no plan force baseline` | A full retraction cleared the baseline. Plan again to re-establish it. |
| `Could not transform robot_base to probe` | No probe sensor is tracked. Probe-mode targeting will not work; CSV mode still will. |

---

## Other Useful Commands

```bash
ros2 run tf2_tools view_frames

# Request a plan by hand (target is x, y, z in metres, in the robot_base frame)
ros2 service call /planner/command interfaces/srv/Planner \
  "{command: 'generateTrajectory', value: [0.0, 0.0, 0.0]}"
```

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

Each package with parameters carries a `config/*_params.yaml` — the single
source of truth, loaded by its launch file; launch arguments override.
Runtime data directories resolve via the `data_root` parameter, the
`CTR_DATA_ROOT` env var, or (fallback) the legacy workspace-layout climb.

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

1. Launch the planner node:
   
   Open a terminal and navigate to the workspace, then run:      

   ```bash
   source ./install/setup.bash
   ros2 launch planner launch.py
   ```
   You should see in terminal: `Path Planner Node has been initialized.`

2. Launch the manager node:

   Open another terminal and navigate to the workspace, then run:

   ```bash
   source ./install/setup.bash
   ros2 launch manager launch.py
   ```

   Another GUI with task-space information, robot control, and planner command buttons will appear.

   At this stage, it is recommended to retract the linear stages to the home position and then click **Freeze Robot**. This stops updates of robot body position to mitigate EM tracking sensor deviation during robot actuation caused by magnetic interference from the middle tube stage. After clicking **Freeze Robot**, do not move the robot or the EM tracker field generator.

3. Set target, plan, and command:

   1. Set the Mode to "Select Target"
   2. "Enable" if not enabled.
   3. "Start Procedure"
   4. Move the probe to set the desired target position. You should see **Control Mode** change to **Position**, and the rotary joints align toward the target angle. The planner-generated path from current tip position to target is shown in 3D Slicer.
   5. Once the path is satisfactory, change Mode to "Deployment" and click **Auto Insert** to insert all the way to the target, or use the insert/retract physical buttons on the robot for step-by-step insertion and retraction.

   If you want to read targets from a CSV file, use the `Toggle:xxx` button to switch from **Probe Mode** to **CSV Mode**. The system will then read targets from `Input_Files/random_interior_points.csv` one by one.
---  


## Other Useful Commands
ros2 run tf2_tools view_frames
ros2 service call /planner/command interfaces/srv/Config "{command: 'generateTrajectory', value: 0.0}"

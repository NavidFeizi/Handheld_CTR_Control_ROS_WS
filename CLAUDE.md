# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 (Jazzy) colcon workspace controlling a handheld concentric tube robot (CTR) for image-guided interventions. All packages are C++17 `ament_cmake`. The git root is the colcon workspace root; ROS packages live in `src/`. Runtime data directories (`Input_Files/`, `Output_Files/`, `Shared_Files/`, `3DSlicer/`) sit at the workspace root and are resolved at runtime via `ctr_common::resolveDataRoot()` (param `data_root` → env `CTR_DATA_ROOT` → legacy workspace-layout climb with a WARN).

## Build

Run from the workspace root (not `src/`), with ROS 2 Jazzy sourced:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

- Single package: `colcon build --packages-select <pkg>` (rebuild `interfaces` first if messages changed; after DELETING interface files also purge `build/interfaces` — rosidl does not garbage-collect generated code).
- **This dev machine has no ROS 2**: all compile checks run in Docker via `docker/check.sh` (full colcon build in a `ros:jazzy` image mirroring the lab's `/usr/local` layout; see `docker/README.md`). Never claim a change builds without it.
- Tests: configure with `-DBUILD_TESTING=ON`, then `colcon test --packages-select ctr_common emtracker manager robot ctr_robot_driver mpc planner && colcon test-result`. Suites are hardware/Torch/OMPL-free.

### Build prerequisites

Non-ROS dependencies at hardcoded `/usr/local` paths: LibTorch 2.9.0 (`/usr/local/libtorch`, pinned by `ctr_kinematics_pinn/cmake/torch_pin.cmake` — the pin re-runs for every consumer via `CONFIG_EXTRAS`; override with `-DCTR_TORCH_DIR=`), OMPL ≥ 2.0 (`Planner.hpp` uses AORRTC; headers must be reachable as `/usr/local/include/ompl` — the docker image symlinks the versioned dir), OpenIGTLink 3.1 (built from master; upstream never tagged 3.1), Blaze headers. Also: liblely-coapp (CANopen), NLopt, FCL, TBB, OSQP 0.6.3/OsqpEigen 0.8.1 (mpc), Qt5, BLAS/LAPACK, spdlog, nlohmann-json, yaml-cpp. The NDI tracker API is vendored inside `emtracker/lib_emtracker/ndi_api`.

⚠ Do not add `-march=`-class flags to INTERFACE/header-only libraries: `CtrMpc` once broadcast `-march=native` into consumer TUs and corrupted the OsqpEigen ABI (heap corruption). `-mtune=native` is safe.

## Run

Requires hardware (CAN interface for the motors, NDI EM tracker on `/dev/ttyUSB1`) and optionally 3D Slicer. CAN/USB setup, homing, and the operator workflow are in `README.md` — follow it rather than improvising.

```bash
source install/setup.bash
ros2 launch ctr_bringup system_bringup.launch.py   # EM tracker → (+14 s) robot nodes + GUI → (+20 s) IGTL bridge
# (ros2 launch launch/system_bringup.launch.py still works via a shim)
ros2 launch planner launch.py
ros2 launch manager launch.py                      # master GUI + recorder
ros2 launch mpc launch.py                          # MPC task-space controller (separate workflow)
```

The 14 s/20 s `TimerAction` delays let the NDI tracker finish initializing — preserve that ordering. Nodes are CPU-pinned with `taskset` (cores 2–10; EKF on core 3). The GUIs no longer block on absent services: 500 ms readiness timers gate the controls instead.

Useful: `ros2 service call /planner/command interfaces/srv/Planner "{command: 'generateTrajectory', value: [0,0,0]}"`.

## Architecture

### Shared library packages (extracted 2026-08; no more copy-pasted libraries)

| Package | Kind | Contents |
|---|---|---|
| `ctr_common` | compiled lib | `joint_conventions.hpp` (wire `[α1,β1,α2,β2]` ↔ physics `[β1,β2,α1,α2]` permutations + coupled joint clamp), `csv_io.hpp` (numeric CSV parsing, never creates dirs on read), `runtime_paths` (`resolveDataRoot()`, `resolveModelsDir()`), `output_session.hpp` (timestamped session dirs) |
| `ctr_cosserat` | static lib, ROS-free | Cosserat-rod CTR model (formerly `robot/ctr_library`) |
| `ctr_kinematics_pinn` | INTERFACE lib, ROS-free | TorchScript PINN FK/Jacobians/IK (`posCTRL`) — the single merged copy — plus the unified `models/` pool (each dir: `model_scripted.pt` + `parameters.json`, installed to share) |
| `ctr_robot_driver` | static lib, ROS-free | lely CANopen driver (formerly `robot/lib_robot`); dcfgen runs as a proper build rule and installs `master.dcf/.bin` to share; `ICtrJointGroup` is the hardware seam (`CTRobot` implements it) |
| `ctr_bringup` | launch-only | `system_bringup.launch.py` (workspace-root `launch/` file is a shim) |

### Packages → nodes

| Package | Executables (node names) | Role |
|---|---|---|
| `interfaces` | — | Custom msgs/srvs used by everything |
| `robot` | `ctr_robot` (robot_node), `qt_gui` (gui_node), `pinn_fk`, `ekf_node`, `cosserat_fk` (not launched) | Hardware layer via `ICtrJointGroup`, operator GUI, PINN FK, EKF tip-force estimation |
| `emtracker` | `track` (emt_node) | NDI EM tracker driver: poses, TF broadcast, registration |
| `igtlink_bridge` | `bridge` | OpenIGTLink bridge to 3D Slicer |
| `planner` | `plan` (planner_node) | OMPL planning with FTL objective, PINN-informed; two-phase + mid-deployment replanning |
| `mpc` | `mpc` | QP-based MPC for task-space tracking (OSQP, warm-started) |
| `manager` | `master` (master GUI), `record` | Procedure orchestration: targets, plan requests, deployment loop (with force-drift replanning + backoff), recording |
| `target_gen` | — | Not a ROS package; standalone Python utilities |

### Data flow (main loop)

EM tracker → `task_space/feedback/base_tool` (Taskspace) → controllers; operator/manager → `joint_space/target` (Jointspace) → `ctr_robot` → CANopen hardware → `joint_space/feedback` → `pinn_fk` → `shape/tube_1..3` (metres on the wire; the bridge converts to mm at the Slicer boundary) + `task_space/sim_out` → `igtlink_bridge` → 3D Slicer. In parallel, `ekf_node` fuses EM tip pose vs. PINN prediction into `task_space/force_estimate` (Force), consumed by MPC, `pinn_fk`, planner and the GUIs. The planner writes `Shared_Files/plannedPath.csv` **atomically** (`.tmp` + rename) and publishes `task_space/path`; `manager/master` polls that CSV and drives deployment waypoint-by-waypoint, requesting a replan when the EKF force drifts from the plan baseline (exponential backoff + hysteresis on rejection).

Key services: `robot_config` / `robot_enable` (Config, served by `ctr_robot`), `planner/command` (Planner — single outstanding request by design; the ~3 s OMPL solve runs in the service callback), `recording` (Recording), `freeze_robot` / `freeze_phantom` (SetBool, emtracker). TF frames (`em_tracker` → `robot_base`, `ctr_tip`, `phantom`, `probe`, …) come from the emtracker node.

Joint convention (`Jointspace.position[4]`): wire order `[α1, β1, α2, β2]`; physics/PINN order `[β1, β2, α1, α2]`. ALWAYS convert via `ctr_common::joint_conventions` — never hand-roll the permutation.

### Parameters

Every package with parameters has `config/*_params.yaml` (single source of truth, installed to share); launch files load the YAML first, then apply launch-argument overrides (same names/defaults as before). Notable: robot's EKF R matrix (36 elements) lives only in `robot_params.yaml`; `f_dot` = 0.2 N/s; manager's replanning tunables (`force_replan_threshold`, `replan_cooldown_s`, `min_remaining_waypoints`, `targets_csv`) are parameters; planner has `model_name`/`solve_time`; robot_node has `canopen_dir`/`can_interface`/`encoder_memory_dir`.

Writable runtime state: encoder memory in `$HOME/Documents/handheld_CTR/encoder_memory/` (param-overridable), EM-tracker config/registration in `$HOME/Documents/handheld_CTR/emtracker_config/` (seeded from share on first run).

### Threading rules learned the hard way (keep them)

- `master`/`qt_gui` run a MultiThreadedExecutor beside the Qt thread: widget mutations from ROS callbacks must go through `QMetaObject::invokeMethod(..., Qt::QueuedConnection)`; cross-thread state is atomic or mutex-snapshotted.
- The CANopen driver's `EnableOp_` must never throw (lely fibers are noexcept — a throw kills the process); it returns `bool` + `Flags::ENABLE_FAULT`.
- `CTRobot` owns two joinable threads (CAN event loop + monitor); `shutdown()` is idempotent and called from the destructor. Boot timeout retries ×3 then reports failure — no `raise(SIGINT)`.
- `MPC` is neither copyable nor movable (OSQP solver state); hold it in a smart pointer.

## Related docs

- `README.md` — dependencies, hardware setup (CAN, EM tracker), homing, operator workflow, and the post-refactor lab smoke checklist. Read before touching anything hardware-facing.
- `docker/README.md` — the build-verification image (pinned versions, lab-parity shims, copy-on-write source volume).
- `SYSTEM_ARCHITECTURE_REFERENCE.md` — data-flow diagrams and the dynamic replanning design.

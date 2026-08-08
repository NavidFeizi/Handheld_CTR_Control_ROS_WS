# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 (Jazzy) colcon workspace controlling a handheld concentric tube robot (CTR) for image-guided interventions. All packages are C++17 `ament_cmake`. The git root is the colcon workspace root; ROS packages live in `src/`. Runtime data directories (`Input_Files/`, `Output_Files/`, `Shared/`, `Shared_Files/`, `3DSlicer/`) sit at the workspace root and are referenced from C++ code by path (see "Path and parameter conventions").

## Build

Run from the workspace root (not `src/`), with ROS 2 Jazzy sourced:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --packages-select interfaces emtracker robot igtlink_bridge planner mpc
```

- Single package: `colcon build --packages-select <pkg>` (rebuild `interfaces` first if messages changed — everything depends on its generated headers).
- Clean: `rm -rf build/ install/ log/` at the workspace root.
- The package list above is the VS Code default build task (`.vscode/tasks.json`). `manager` builds fine but is excluded there (README's build line includes it); `controller` is legacy and not normally built.
- **There are no tests** anywhere in the workspace (no gtest/ament tests; only `interfaces` has a `BUILD_TESTING` block, and it just runs lint with copyright/cpplint disabled).

### Build prerequisites and fragility

Non-ROS dependencies are found at hardcoded `/usr/local` paths: LibTorch (`/usr/local/libtorch`), OMPL (`/usr/local/share/ompl/cmake/`), OpenIGTLink (`/usr/local/lib/igtl/cmake/igtl-3.1`), Blaze headers (`/usr/local/include/blaze`). Also required: liblely-coapp (CANopen), NLopt, FCL, TBB, OSQP/OsqpEigen (mpc only), Qt5, BLAS/LAPACK. The NDI tracker API is vendored inside `emtracker/lib_emtracker/ndi_api`.

- `src/planner/ctr_pinn_infer/CMakeLists.txt` deliberately pins `Torch_DIR` and pre-populates `c10_LIBRARY` to work around header/ABI conflicts between a system PyTorch and the manual LibTorch install. Do not "simplify" that block; see its comments and the README's LibTorch note.
- `src/robot/lib_robot` persists encoder offsets to `$HOME/Documents/handheld_CTR/encoder_memory/` — a compiled-in path outside the workspace.
- `src/robot/lib_robot/config/master.yml` is the CANopen master/slave topology; `dcfgen` runs on it at build time.

## Run

Requires hardware (CAN interface for the motors, NDI EM tracker on `/dev/ttyUSB1`) and optionally 3D Slicer. CAN/USB setup, homing procedure, and the full operator workflow are in `README.md` — follow it rather than improvising.

```bash
source install/setup.bash
ros2 launch launch/system_bringup.launch.py   # EM tracker → (+14 s) robot nodes + robot GUI → (+20 s) IGTLink bridge to Slicer
ros2 launch planner launch.py                 # path planner
ros2 launch manager launch.py                 # master GUI + recorder
ros2 launch mpc launch.py                     # MPC task-space controller (separate workflow)
```

The 14 s/20 s `TimerAction` delays in `system_bringup.launch.py` let the NDI tracker finish initializing before the robot and IGTL bridge start — preserve that ordering. Nodes are CPU-pinned with `taskset` in the launch files.

Useful: `ros2 service call /planner/command interfaces/srv/Config "{command: 'generateTrajectory', value: 0.0}"`.

## Architecture

### Packages → nodes

| Package | Executables (node names) | Role |
|---|---|---|
| `interfaces` | — | Custom msgs/srvs/actions used by everything |
| `robot` | `ctr_robot` (robot_node), `qt_gui` (gui_node), `pinn_fk`, `ekf_node`, `cosserat_fk` (unused in launch) | Hardware layer: CANopen motor control of 4 joints, operator GUI, PINN forward kinematics, EKF tip-force estimation |
| `emtracker` | `track` (emt_node) | NDI EM tracker driver: tip/base/probe poses, TF broadcast, registration transforms |
| `igtlink_bridge` | `bridge` | OpenIGTLink bridge to 3D Slicer (shape + path visualization) |
| `planner` | `plan` (planner_node) | OMPL sampling-based planning with follow-the-leader (FTL) objective, PINN-informed |
| `mpc` | `mpc`, `reference` (reference unused in launch) | QP-based MPC for task-space tracking (OSQP) |
| `manager` | `master` (master GUI), `record`; `manage`/`procedure`/`joint_path` exist but are commented out of the launch | High-level procedure orchestration: target selection, plan requests, deployment loop, data recording |
| `controller` | `control` | **Legacy** resolved-rate IK controller; subscribes to topics nobody publishes anymore |
| `target_gen` | — | Not a ROS package; standalone Python target-sampling utilities |

### Data flow (main loop)

EM tracker → `task_space/feedback/base_tool` (Taskspace) → controllers; operator/manager → `joint_space/target` (Jointspace) → `ctr_robot` → CANopen hardware → `joint_space/feedback` → `pinn_fk` → `shape/tube_1..3` + `task_space/sim_out` → `igtlink_bridge` → 3D Slicer. In parallel, `ekf_node` fuses EM tip pose vs. PINN prediction into `task_space/force_estimate` (Force), consumed by MPC, `pinn_fk`, and the GUIs. The planner writes its path to `Shared_Files/plannedPath.csv` and publishes `task_space/path`; `manager/master` reads that CSV and drives deployment waypoint-by-waypoint.

Key services: `robot_config` / `robot_enable` (Config, served by `ctr_robot` — the generic command channel), `planner/command` (Planner, served by planner), `recording` (Recording, served by recorder), `freeze_robot` / `freeze_phantom` (SetBool, served by emtracker). TF frames (`em_tracker` → `robot_base`, `ctr_tip`, `phantom`, `probe`, …) are broadcast by the emtracker node.

Joint convention throughout (`Jointspace.position[4]`): `[α1, β1, α2, β2]` — rotation (rad) and translation (m) per tube pair; velocity limits like `[3.0, 0.012, 3.0, 0.012]` are rad/s and m/s.

### Duplicated internal libraries — keep copies in sync by hand

There is no shared library package; physics/inference code is copy-pasted per package and has diverged:

- `ctr_pinn_infer` (TorchScript PINN inference): `robot/` and `mpc/` copies are **byte-identical** (`ctr_pinn_inference.hpp`) — a change in one must be mirrored in the other. `planner/ctr_pinn_infer` is a **divergent fork** (`PINNs.hpp`, different API surface and model directory) — do not assume it matches.
- `ctr_library` (Cosserat-rod CTR model): `robot/` and `controller/` copies have diverged; robot's is the newer one, controller's adds OpenIGTLink code.
- PINN model weights live in `robot|mpc/ctr_pinn_infer/models/` and `planner/shared/models/` (each model dir has a `parameters.json`). Model names are hardcoded in code/launch (e.g. `kModelName` in `planner/src/planner_node.cpp`, `model_name` param in `mpc/launch/launch.py`).

### Path and parameter conventions

- **No ROS parameter YAML files exist.** Parameters live in launch-file defaults and `declare_parameter` calls in C++; many values (model names, CSV paths, encoder/gear constants in `robot/lib_robot/include/Robot.hpp`, the EKF R matrix in `robot/launch/robot.py`) are hardcoded.
- Nodes locate the workspace root at runtime via `get_package_share_directory(pkg) + "/../../../../"` — climbing out of `install/`. This breaks with `--symlink-install`-less relocation or a non-standard install layout; keep the workspace layout intact.
- `Shared/` and `Shared_Files/` are **different directories with same-named files**: the planner and master node use `Shared_Files/plannedPath.csv`; `joint_path_node` uses `Shared/`. `Input_Files/` holds target CSVs (e.g. `random_interior_points.csv` read by master node in CSV mode); `Output_Files/` receives recorder sessions.
- EM sensor ↔ serial number ↔ SROM/transform mapping is in `src/emtracker/lib_emtracker/config/config.yaml` (compiled in via `-DCONFIG_DIRECTORY`).

### Legacy / dead code — don't take it as reference

`controller/` (entire package), `planner/old_motion_planning/`, `planner/old_ctr_pinn_infer/`, `controller/old_ctr_library/`, `robot/src/robot_sim_node.cpp`, `robot/src/qt_node_deprecated.cpp`, `src/planner_backup.zip`. Known wiring quirks: `igtlink_bridge` creates a `planner/command` client with the wrong service type (Config vs. Planner), and `joint_space/target` is also used as a service name (Jointstarget) with no server — neither path is functional.

## Related docs

- `README.md` — dependencies, hardware setup (CAN, EM tracker), homing and operator workflow. Read before touching anything hardware-facing.
- `SYSTEM_ARCHITECTURE_REFERENCE.md` — data-flow diagrams and the design for dynamic (mid-deployment) replanning based on EKF force updates; `CLAUDE_CTR_DYNAMIC_REPLANNING_PROMPT.md` / `CLAUDE_QUICK_CHAT_PROMPT.md` are the working prompts for that effort.
- `REFACTORING_GUIDE.md` — in-progress split of the manager master node into ROS logic + Qt GUI (`master_node.cpp` + `master_qt_gui.cpp`).

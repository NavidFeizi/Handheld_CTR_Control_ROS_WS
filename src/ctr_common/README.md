# ctr_common

Shared conventions and utilities for the handheld CTR workspace. Four headers, one
compiled translation unit (`src/runtime_paths.cpp`) — everything else is inline.

This is the package that keeps the rest of the workspace consistent. In particular,
**`joint_conventions.hpp` is the only correct way to convert between the two joint
orderings** used in this system; hand-rolling the permutation is the most common way
to break the robot.

## Using it from another package

```cmake
find_package(ctr_common REQUIRED)
target_link_libraries(my_target ctr_common::ctr_common)
```

```xml
<depend>ctr_common</depend>
```

Real consumers: `robot` (`pinn_fk`, `ekf_node`), `manager` (`record`, `master`),
`mpc`, `planner`.

## `joint_conventions.hpp`

Two joint orderings exist in this workspace and they are not the same:

| Convention | Order | Where it appears |
|---|---|---|
| **Wire** | `[α1, β1, α2, β2]` | Every `joint_space/*` topic — `interfaces/msg/Jointspace`, field `position[4]` |
| **Physics** (4) | `[β1, β2, α1, α2]` | PINN, Cosserat, planner internals |
| **Physics** (6) | `[β1, β2, β3, α1, α2, α3]` | Same, where β3/α3 are the unactuated outer tube and are always 0 |

α is a rotation (rad), β a translation (m).

| Function | Signature |
|---|---|
| `wireToPhysics4` | `blaze::StaticVector<double,4>(const std::array<double,4>&)` |
| `wireToPhysics6` | `blaze::StaticVector<double,6>(const std::array<double,4>&)` — pads β3 = α3 = 0 |
| `physicsToWire` | `std::array<double,4>(const blaze::StaticVector<double,4>&)` |
| `physicsToWire` | `std::array<double,4>(const blaze::StaticVector<double,6>&)` — overload, drops β3/α3 |
| `clampJointPositions<N>` | `void(StaticVector<double,N>& q, const StaticVector<double,N>& q_min, const StaticVector<double,N>& q_max, const rclcpp::Logger&)` |

`clampJointPositions` operates **in physics order** and implements the tube-coupling
rule: each tube's limits are shifted by the next-outer tube's current translation
before clamping, because the tubes telescope. It is specialized for `N == 4` and
`N == 6`; any other `N` clamps against the raw limits and emits an `RCLCPP_WARN`.

```cpp
#include "ctr_common/joint_conventions.hpp"

const auto q = ctr_common::wireToPhysics4(msg->position);   // topic → PINN
msg.position = ctr_common::physicsToWire(q);                // PINN → topic
```

## `runtime_paths.hpp`

Resolves the runtime data directories. Declared here, defined in
`src/runtime_paths.cpp` (needs `rclcpp` and `ament_index_cpp`).

| Function | Resolution order |
|---|---|
| `resolveDataRoot(rclcpp::Node&, const std::string& package_name)` | 1. node parameter `data_root` (declared if absent) → 2. env `CTR_DATA_ROOT` → 3. legacy fallback: climb four parents from the package's share directory |
| `resolveModelsDir(rclcpp::Node&)` | 1. node parameter `models_dir` (declared if absent) → 2. installed `share/ctr_kinematics_pinn/models` |

The data root holds `Input_Files/`, `Output_Files/`, and `Shared_Files/`. The legacy
fallback climbs `install/<pkg>/share/<pkg>` back to the workspace root, so it only
works in an in-workspace install layout — it emits `RCLCPP_WARN_ONCE` when it fires.
If you see that warning, set `data_root` or `CTR_DATA_ROOT` instead of relying on it.

## `csv_io.hpp`

Namespace `ctr_common::csv`. Line-oriented numeric CSV parsing shared by `manager`
and `planner`. **Reading never creates directories** or touches the filesystem beyond
opening the file.

| Function | Behavior |
|---|---|
| `parseNumericRows(std::istream&)` | Comma-split. Rows containing any unparseable cell (a header line, say) are skipped, as are empty lines. Ragged row lengths are preserved — validating the column count is the caller's job. |
| `readNumericCsv(const std::filesystem::path&)` | Returns `std::nullopt` if the file cannot be opened; otherwise delegates to `parseNumericRows`. |

## `output_session.hpp`

Timestamped output directories, laid out as `<output_base>/<session>/<YYYY-MM-DD_HH-MM-SS>/`.

| Function | Behavior |
|---|---|
| `currentTimestamp()` | Local wall-clock time as `%Y-%m-%d_%H-%M-%S`. |
| `makeSessionDir(output_base, session_name)` | Creates the directory tree via `create_directories` and returns the path. An empty session name maps to `default`. |

Unlike `csv_io`, this one **does** create directories — that is its purpose.

## `package_log.hpp`

Writes each package's diagnostic records to its own file:
`<log_root>/[<CTR_RUN_ID>/]<package>/<YYYY-MM-DD_HH-MM-SS>_<node>_<pid>.log`, plus a
`<node>_latest.log` symlink that points at the newest file.

| Function | Behavior |
|---|---|
| `installPackageFileLog(node, package)` | Wraps the process' rcutils output handler. The previous handler (console, `~/.ros/log`) still runs first, then the record is appended to the file and flushed on every line. Only the first call per process takes effect; later calls return the same path. Call it after `rclcpp::init()`. If the file cannot be opened, it logs a WARN and returns an empty path. |
| `writePackageLog(severity, logger, msg)` | Appends one record for loggers that do not go through rcutils, such as spdlog. |

`log_root` is taken from the first of these that is set: the `log_dir` parameter, the
`CTR_LOG_DIR` environment variable, then `<data_root>/log`. Records below a logger's
severity threshold are not written. To raise one node to debug, pass
`--ros-args --log-level <node>:=debug`.

## `path_downsample.hpp`

Header-only (needs Blaze on the consumer's include path). `downsamplePath<N>(q,
beta_step, alpha_step)` turns a planned joint path into waypoints: a row is kept when
any β moved by `beta_step` or any α by `alpha_step` since the last kept row; the first
row is always kept and the last appended once. `N = 4` is the planner layout
`[β1, β2, α1, α2]`, `N = 6` the manager layout. `kWaypointBetaStep` (2 mm) and
`kWaypointAlphaStep` (5°) are the steps both `planner` (drawn path) and `manager`
(executed waypoints) use, so the GUI dots are the executed waypoints.

## `planner_protocol.hpp`

`planner/command` response messages the manager matches exactly, e.g.
`planner_protocol::kReplanForceUnchanged` for a `replanDeployment` skipped because the
planning force equals the active plan's.

## Tests

```bash
colcon test --packages-select ctr_common --ctest-args -R test_joint_conventions
```

- `test/test_joint_conventions.cpp` — wire↔physics round-trips for both widths, plus
  three `clampJointPositions` cases (within limits untouched, translation coupling,
  rotation coupling).
- `test/test_csv_io.cpp` — numeric parsing, header and blank-line skipping, ragged
  rows preserved, scientific notation, missing file → `nullopt`.
- `test/test_path_downsample.cpp` — pure α rotation and β2-only phases are split into
  steps, empty/single-row inputs, and the last row is never duplicated.
- `test/test_package_log.cpp` covers three things: the previous handler still runs,
  records longer than the 1 KiB stack buffer are kept whole, and both the
  `<log_dir>/<package>/` placement and the `latest` symlink are correct.

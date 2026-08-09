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

## Tests

```bash
colcon test --packages-select ctr_common --ctest-args -R test_joint_conventions
```

- `test/test_joint_conventions.cpp` — wire↔physics round-trips for both widths, plus
  three `clampJointPositions` cases (within limits untouched, translation coupling,
  rotation coupling).
- `test/test_csv_io.cpp` — numeric parsing, header and blank-line skipping, ragged
  rows preserved, scientific notation, missing file → `nullopt`.

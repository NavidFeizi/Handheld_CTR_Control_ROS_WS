# ctr_robot_driver

ROS-free CANopen motor driver for the handheld CTR's four joint drives, built on
lely-coapp. Static library, no `rclcpp` dependency. Formerly `robot/lib_robot`.

Ships the CANopen master configuration too: `dcfgen` turns `config/master.yml` into
`master.dcf`/`master.bin` at build time, installed to
`share/ctr_robot_driver/canopen/` and read from there at runtime.

## Using it from another package

```cmake
find_package(ctr_robot_driver REQUIRED)
target_link_libraries(ctr_robot ctr_robot_driver::ctr_robot_driver)
```

```xml
<depend>ctr_robot_driver</depend>
```

Its consumer is `robot`'s `ctr_robot` node.

## `ICtrJointGroup` — the hardware seam

`include/ctr_robot_driver/ICtrJointGroup.hpp` is the abstract interface the rest of
the system programs against. `RobotNode` holds an `ICtrJointGroup`, not a `CTRobot`,
so a simulation or test double can implement the same surface without any hardware.

Every command and feedback vector is a `blaze::StaticVector<double,4>` — one slot per
joint, in **wire order** `[α1, β1, α2, β2]`. Convert with
`ctr_common::joint_conventions` before handing values to the PINN or Cosserat model.

| Group | Members |
|---|---|
| Lifecycle | `connect(int sample_time_ms) → bool` (blocking, false if hardware never comes up), `shutdown()` (safe to call more than once), `isConnected()` |
| Command | `enableOperation(bool)`, `setTargetPos`, `setTargetVel` |
| Configuration | `setMaxVel`, `setMaxAcc`, `setMaxTorque(neg, pos)`, `setProfileParams(vel, acc, dcc)`, `setOperationMode(OpMode)`, `setEncoders`, `setPosLimit(min, max)` |
| Feedback | `getCurrent`, `getVel`, `getPos`, `getPosLimit` |
| Status | `getSwitchStatus` (×2), `getEnableStatus`, `getEncoderStatus`, `getDisabledStatus` (×2), `getReachedStatus`, `getTemperature(cpu, driver)`, `getDigitalIn`, `getInterface` |
| Diagnostics | `logger()` — the implementation's spdlog logger, or `nullptr` |
| Waits | `waitUntilReach` / `waitUntilTransReach`, each with and without an `std::atomic<bool>&` cancel flag |

`logger()` is not decoration. `CTRobot`'s `"CTR"` logger owns the **only file sink in
the system** (`log/Robot/<timestamp>.txt`), and that file is what gets copied off the
lab machine after a run. `RobotNode` adopts it in its constructor; before that it
logged to `spdlog::default_logger()` (stdout), so none of its messages survived a
run. A double that owns no logger returns `nullptr` — check before use.

## `CTRobot` — the CANopen implementation

`class CTRobot final : public ICtrJointGroup`. Not copyable — it owns threads and the
CANopen master lifetime.

```cpp
CTRobot(blaze::StaticVector<double,4> maxVel,
        blaze::StaticVector<double,4> maxAcc);
CTRobot();
```

### Position limiting is not this class's job

`setTargetPos` performs **one** check — finiteness — and then writes the target
through. That check has to be here: `Cia301Node::setPos` does
`static_cast<int32_t>(value * ppu)`, which is undefined for NaN and yields
`INT32_MIN` on x86, i.e. a full-travel negative command.

Range limiting belongs to `RobotNode`, which recomputes the coupled window from live
joint feedback every 10 ms and pushes it to the drives as object `0x607D` over TPDO4;
the drive then clips the target itself. Note the consequence: **a clipped target
stops the axis short with no feedback of any kind.** `RobotNode::warnIfTargetOutsideLimits`
logs a target it can see will be clipped, and the manager's deployment stall warning
catches the case where it happened anyway.

`CTRobot::checkPosLimits()` used to exist for this and has been removed. It had no
live call site — the only one was commented out inside `setTargetPos` — and its
bounds (`β ∈ [0, 0.097] / [0, 0.052]`) predated the current joint frame
(`β ∈ [-0.156, -0.034]`), so re-enabling it would have rejected every target. The
`m_lowerBounds`/`m_upperBounds`/`m_posOffsets`/`m_minClearance`/`m_maxClearance`/
`m_flagPositionLimit` members and `convPosToRobotFrame()` went with it, along with
the constructor's `position_limit` parameter.

### `RuntimePaths` — set these first

These were compile-time macros before the refactor. They are now runtime values and
**must be set via `setRuntimePaths()` before `startRobotCommunication()`**:

| Field | Default | Meaning |
|---|---|---|
| `canopen_dir` | *(required, no default)* | Directory holding `master.dcf` / `master.bin` |
| `can_interface` | `can0` | SocketCAN interface name |
| `encoder_memory_dir` | empty → `$HOME/Documents/handheld_CTR/encoder_memory/` | Persisted encoder offsets |
| `log_dir` | `log/Robot/` | spdlog output |

`resolvedEncoderMemoryDir()` applies the `$HOME` fallback. `robot_node` exposes
`canopen_dir`, `can_interface`, and `encoder_memory_dir` as ROS parameters.

### `CanEssentials.hpp`

| Type | Purpose |
|---|---|
| `ControlWord` / `StatusWord` | CiA-402 bit structs; `getCiA402StatusMessage()` decodes a status word to text |
| `Flags` | Atomic bitset over `FlagIndex`: `BOOT_SUCCESS`, `TASKS_POSTED`, `ENCODER_SET`, `NEW_TARG_READY`, `ENCODER_MEM_READY`, `ENABLE_FAULT` |
| `OpMode` | `Disabled = 0`, `PositionProfile = 1`, `VelocityProfile = 3`, `Homing = 6`, `FaulhaberCommand = -1` |
| Helpers | `GetCommandFromHex`, `bin2Dec`, `ToBinaryString<T>`, plus the CANopen object-dictionary index constants |

`CiA301node.hpp` wraps a lely `FiberDriver` per axis. It is internal — program
against `ICtrJointGroup`.

## Threading rules

These were learned the hard way and must be preserved:

- **`EnableOp_` must never throw.** lely fibers are `noexcept`, so a throw kills the
  whole process. It returns `bool` and sets `Flags::ENABLE_FAULT` instead — the node
  stays up with operation not enabled.
- `CTRobot` owns two joinable threads (the CAN event loop and a monitor/watchdog).
  `shutdown()` is idempotent and is called from the destructor.
- Boot timeout retries three times and then reports failure. It does **not**
  `raise(SIGINT)`.

## CANopen configuration

`config/master.yml` defines master `node_id: 7`, `sync_period: 10000 µs`, and four
slaves at `node_id` 1–4, all sharing `605.0141.01-L.eds`.

The `dcfgen` step is a proper `add_custom_command` + `add_custom_target(canopen_dcf
ALL)`: it copies the YAML and EDS into the **build** tree and regenerates only when
those inputs change. The previous `POST_BUILD` version regenerated on every build and
wrote into the source tree. Generated and source files install together to
`share/ctr_robot_driver/canopen/`.

`dcfgen` comes from the PyPI `dcf-tools` package and must be on `PATH` at build time.

## Hardware

A real SocketCAN bus with the four drives powered and on the network. Bus bring-up
(`ip link set can0 up type can bitrate 1000000`, the IXXAT driver, `candump`
verification) is in the [root README](../../README.md) — follow it rather than
improvising.

## Build quirks

`liblely-coapp` is linked by plain path rather than through the PkgConfig imported
target, and `spdlog` is `PRIVATE`, so the exported link interface of this static
library stays resolvable downstream. As with `ctr_cosserat`, `PRIVATE` dependencies
of a static library surface in the export as `$<LINK_ONLY:...>`, so consumers must be
able to resolve `spdlog` when `find_package(ctr_robot_driver)` loads.

## Command handshake (`node_command.hpp`)

Configuration calls (`enableOperation`, `setMaxTorque`, `setProfileParams`,
`setEncoder`, `setOperationMode`) arrive on ROS threads but may only touch the
CANopen stack from inside the node's lely fiber, so they cross into `TaskTarget`
through `NodeCommandMailbox`: a single-slot mailbox where the producer writes the
payload and *then* release-stores the command tag, and the fiber acquire-loads the
tag before reading the payload.

The ordering is the whole point. The previous handshake set a plain
`int m_isConfiguring` **before** assigning a plain `std::string m_commandMsg`, with
no synchronisation: the fiber could observe the flag while the string was still
stale, fall through every dispatch branch, and clear both — silently swallowing
that node's command. Because the four nodes race independently, the visible
symptom was one motor left un-enabled while the other three came up.

`publish()` waits (up to 5 s) for the slot to drain and then refuses — returning
false, which the caller logs at `ERROR` — rather than overwriting a command the
fiber has not consumed yet. The timeout is sized to clear the slowest handler
(`SetEncoder_` sits on ~2.5 s of internal `AsyncWait`s), not the ~18 ms poll
period, so legitimate back-to-back calls such as `findLinearHome`'s
`setEncoders()` → `enableOperation(false)` block and succeed instead of the
second one being lost.

## Tests

```bash
colcon test --packages-select ctr_robot_driver
```

`test/test_can_essentials.cpp` compiles only `CanEssentials.cpp`, so it needs no lely
master and no hardware: `bin2Dec` correctness, `StatusWord` bit decoding
(ready / switched-on / operation-enabled / fault), `Flags` atomic set and get, and
`ToBinaryString<uint16_t>` width and spacing.

`test/test_node_command.cpp` covers the mailbox (`node_command.hpp` is header-only
and deliberately lely-free): payload/tag atomicity, slot release on consume,
refusal to overwrite an unconsumed command, and a two-thread stress run asserting
that no published command is lost or delivered with another command's payload.

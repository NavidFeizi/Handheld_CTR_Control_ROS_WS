# mpc

QP-based model-predictive controller for task-space tracking. Linearizes the PINN
about the current configuration each step, solves a small constrained QP with OSQP
(warm-started), and publishes joint targets.

Pure computation — no hardware. This is a **separate workflow** from the
plan-and-deploy loop driven by `manager`; run one or the other, not both, since both
publish `joint_space/target`.

## Quick start

```bash
source install/setup.bash
ros2 launch mpc launch.py
```

## Nodes

| Executable | Node name | Role |
|---|---|---|
| `mpc` | `mpc` | Task-space MPC; consumes a target pose, emits joint targets |

## Launch file

`launch/launch.py` starts `mpc` on CPU core 4. Every parameter below except `models_dir`
is also a launch argument of the same name, each with **no default**: unset they are
dropped and `config/mpc_params.yaml` supplies the value. Pass one explicitly to override
it for a single run — e.g. to drive the Grassmann CTR instead of the handheld:

```bash
ros2 launch mpc launch.py model_name:=grassmann_ctr_v4.4.4 \
    q0:='[-0.100, -0.055, -0.005, 0.0, 0.0, 0.0]'
```

Quote array values as shown; they reach the node as a double array, not a string.

Editing the YAML needs `colcon build --packages-select mpc` — launch reads the installed
`share/` copy. See [Configuration and
parameters](../../README.md#configuration-and-parameters) for the mechanism.

## Parameters

From `config/mpc_params.yaml` — the handheld CTR values. The defaults below are the
YAML's and are what the node actually receives; the launch file does not shadow them.

| Parameter | Default | Meaning |
|---|---|---|
| `sample_time` | 0.025 | s |
| `model_name` | `ctr_8x91_0.18_tanh_9K_9K_50K_v3` | PINN model |
| `models_dir` | `""` | Empty → installed `share/ctr_kinematics_pinn/models` |
| `q0` | `[-0.156, -0.072, 0.0, 0.0]` | Initial configuration, SI units |
| `u_max` | `[0.012, 0.012, 3.0, 3.0]` | Input magnitude bound (m, m, rad, rad) |
| `u_dot_max` | `[0.03, 0.03, 5.0, 5.0]` | Input rate bound |
| `q_scale` | `[0.5, 0.5, 0.2, 0.2]` | Per-joint scaling in the QP |
| `R_u` | 0.0 | Input-magnitude weight |
| `R_du` | 0.5 | Input-rate weight |
| `Q` | 2000.0 | Tracking-error weight |
| `error_c` | 1.0 | Error scaling constant |

## Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `task_space/target` | `interfaces/msg/Taskspace` |
| sub | `/task_space/feedback/base_tool` | `interfaces/msg/Taskspace` |
| sub | `joint_space/feedback` | `interfaces/msg/Jointspace` |
| sub | `task_space/force_estimate` | `interfaces/msg/Force` |
| pub | `joint_space/target` | `interfaces/msg/Jointspace` |
| pub | `mpc/computation_time` | `std_msgs/msg/Float64` |
| pub | `/task_space/target` | `interfaces/msg/Taskspace` |

Tip feedback comes from the **EM tracker** (`/task_space/feedback/base_tool`), not
from the simulated `task_space/sim_out`.

`mpc/computation_time` is the per-step solve time; `record` logs it, and it is the
first thing to check if tracking degrades.

## Services

| Service | Type | Direction |
|---|---|---|
| `recording` | `interfaces/srv/Recording` | called |

## The controller library

`ctr_mpc/` is an in-tree header + `.tpp` template library (target `CtrMpc`) holding
the QP formulation. `MPC` is neither copyable nor movable — it owns OSQP solver
state — so hold it in a smart pointer.

⚠ **Do not add `-march=`-class flags to `CtrMpc`.** It is header-only, so the flag
reaches every consumer translation unit, including inlined `OsqpEigen::Solver` code.
That gives inlined code different Eigen alignment and padding than the prebuilt
`libOsqpEigen.so`, which corrupts the heap on solver construction and destruction.
This actually happened and was reverted. `-mtune=native` is safe: it changes
scheduling, not the ISA or ABI.

## Tests

```bash
colcon test --packages-select mpc --ctest-args -R test_mpc_qp
./build/mpc/test_mpc_qp --gtest_filter='MpcQp.WarmStart*'
```

`test/test_mpc_qp.cpp` substitutes a linear plant for the PINN so the QP layer can be
tested without Torch: drives toward the target, respects input bounds, commands near
zero at the target, and warm-start results match a fresh re-initialization.

## Status notes

- `launch/launch.py` carries a commented-out alternate parameter set for a
  **Grassmann CTR MPC** configuration (`grassmann_ctr_v4.4.4`, 6-DOF `q0`/`u_max`,
  vector `R`/`Q`). It is reference material, not active configuration — the live
  values are the uncommented handheld set above.
- `mpc_node.cpp` retains a commented-out `task_space/sim_out` subscription beside the
  active `/task_space/feedback/base_tool` one, from when the feedback source was
  switched to the real tracker.

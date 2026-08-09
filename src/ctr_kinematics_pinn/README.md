# ctr_kinematics_pinn

ROS-free TorchScript PINN inference for the handheld CTR — forward kinematics,
Jacobians (incl. w.r.t. tip force), and resolved-rate inverse kinematics
(`posCTRL`) over Blaze types. This package is the merger of the formerly
duplicated `robot/ctr_pinn_infer` and `mpc/ctr_pinn_infer` copies (byte-identical)
and the planner's diverged `PINNs.hpp` fork, plus the unified `models/` pool.

## Usage

```cmake
find_package(ctr_kinematics_pinn REQUIRED)   # also runs the LibTorch pin
target_link_libraries(my_target ctr_kinematics_pinn::ctr_kinematics_pinn)
```

```cpp
#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
PINNs<4> pinn(models_dir, model_name, batch, backbone_nodes);
```

`models_dir` is usually resolved via `ctr_common::resolveModelsDir(node)`:
node parameter `models_dir` if set, else this package's installed
`share/ctr_kinematics_pinn/models`.

Consumers: `robot` (`pinn_fk`, `ekf_node`), `mpc`, `planner`.

## API

`template <size_t controlInputs> class PINNs` — `controlInputs` is 4 or 6, matching
the physics-order joint layout in `ctr_common::joint_conventions`. Non-copyable,
movable. The constructor loads `models_dir/model_name/model_scripted.pt` via
`torch::jit::load` and the sibling `parameters.json`; a malformed or missing
parameter file throws `ParameterLoadError`.

```cpp
explicit PINNs(std::string models_dir, std::string model_name,
               size_t batch_size, size_t num_nodes);
```

Joint values are called `tau` throughout, and every force-aware method has a
zero-force overload that omits `wf`.

| Group | Members |
|---|---|
| Forward kinematics | `getPosDistal(tau[, wf], pos)` — `pos` is 3-element (position) or 7-element (position + quaternion); `getPosDistalBatched`; `getPosTubes(tau[, wf], pos_t3, pos_t2, pos_t1)` |
| Shape | `getShape(tau[, wf], shape)` (`num_nodes × 3`); `getAllTubesShape(tau, wf)` → tuple of three matrices; `getEntireState(tau[, wf], states)` (`num_nodes × 15`) |
| Jacobians | `jacobian(tau[, wf], J)` (3×N, autograd); `jacobianBatched(tau_batch, J)`; `jacobianFinDif(tau, J)` (central differences); `jacobian_wrt_force(tau, wf, J)` |
| Inverse kinematics | `posCTRL(tau, target, posTol[, wf])` — resolved-rate IK, mutates `tau` in place |
| Geometry / limits | `getArclengthEnd`, `getStraightLen`, `getOverallLen`, `getInputPosBounds()` → `(lb, ub)`, `getPrismaticJointRanges`, `getRevoluteJointRanges`, `getNumNodes`, `getStageThickness` |
| Pseudoinverse | `pInv` (fixed 3×6) and `static pInvN<N>` (damped, λ = 1e-12) |

`posCTRL` is resolved-rate with a null-space joint-limit-avoidance term and a hard
cap of 750 iterations, so it can return without reaching `posTol` — check the
resulting tip position if convergence matters.

Note that `getPrismaticJointRanges` accounts for `beta1_range` storing offsets
**relative to β2**, not absolute values.

## LibTorch pin

`cmake/torch_pin.cmake` pins Torch to the self-contained install at
`/usr/local/libtorch` (override with `-DCTR_TORCH_DIR=<prefix>`) and works
around stale-header/ABI conflicts with any system PyTorch — see the comments
in that file. It runs both when this package builds and, via `CONFIG_EXTRAS`,
whenever a downstream package calls `find_package(ctr_kinematics_pinn)`.

## Models

Each `models/<name>/` directory holds exactly `model_scripted.pt` (TorchScript
module) and `parameters.json` (physics/model/dataset parameters). Training
artifacts stay out of the repo.

`parameters.json` carries four sections: `physics_params` (per-tube Young's and shear
moduli, curve radius, straight/curve lengths, inner/outer diameters), `model_params`
(`tau_idx`, `layers`, activation, normalizers), `dataset_params` (`beta{1,2,3}_range`,
`alpha{1,2,3}_range`, sample counts, `f_max`), and `train_params` (loss, epochs,
LBFGS settings, dtype). The loader reads the first three; `train_params` is
provenance only.

The pool installs to `share/ctr_kinematics_pinn/models`:

| Model | Used by |
|---|---|
| `ctr_8x91_0.18_tanh_9K_9K_50K` | — |
| `ctr_8x91_0.18_tanh_9K_9K_50K_FP32` | — |
| `ctr_8x91_0.18_tanh_9K_9K_50K_FP64` | `planner` (`model_name` default) |
| `ctr_8x91_0.18_tanh_9K_9K_50K_v3` | `robot` (`pinn_fk`, `ekf_node`) and `mpc` (`model_name` default) |
| `grassmann_ctr_v4.2.7`, `v4.4.4`, `v4.4.5` | — (alternate Grassmann formulation) |
| `handheld_ctr_v3.1.1`, `v3.4.1`, `v3.5.1` | — |

Models with no listed consumer are kept for comparison and offline work; select one
by setting the owning node's `model_name` parameter.

## Tests

None. Exercising this package requires LibTorch and a loaded TorchScript model, which
the workspace's hardware- and Torch-free test suites deliberately avoid. `mpc`'s
`test_mpc_qp` covers the QP layer by substituting a linear plant for the PINN.

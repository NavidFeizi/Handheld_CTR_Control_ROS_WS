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
| Inverse kinematics | `posCTRL(tau, target, posTol[, wf][, PosCtrlReport*])` — resolved-rate IK, mutates `tau` in place; the optional report pointer overload fills diagnostics without changing numerics |
| Geometry / limits | `getArclengthEnd`, `getStraightLen`, `getOverallLen`, `getInputPosBounds()` → `(lb, ub)`, `getPrismaticJointRanges`, `getRevoluteJointRanges`, `getNumNodes`, `getStageThickness` |
| Pseudoinverse | `pInv` (fixed 3×6) and `static pInvN<N>` (damped, λ = 1e-12) |

`posCTRL` is resolved-rate with a null-space joint-limit-avoidance term and a hard
cap of 750 iterations, so it can return without reaching `posTol` — check the
resulting tip position if convergence matters.

Note that `getPrismaticJointRanges` accounts for `beta1_range` storing offsets
**relative to β2**, not absolute values; `getInputPosBounds()` returns the raw
dataset ranges, so its β1 entry is that relative offset. `posCTRL` clamps β1 to
`[β2 + lb[0], β2 + ub[0]]` and β2 to `[lb[1], ub[1]]`, i.e. the trained domain.

### IK diagnostics

`posCTRL`'s 5-argument overload takes an extra `PosCtrlReport *report` (may be
`nullptr`); when non-null it is filled with:

```cpp
struct PosCtrlReport
{
    enum class Termination { AlreadyWithinTolerance, Converged, Stalled, LimitStalled, MaxIterations };
    std::size_t iterations;         // resolved-rate iterations run
    double initialError;            // ||target - x_CTR|| at the tau passed in [m]
    double finalError;              // ||target - x_CTR|| at the returned tau (tau_min) [m]
    Termination termination;
    std::size_t prismaticLimitHits; // iterations where a prismatic step was clamped
};
```

`Stalled` is the `linfNorm(dtau_dt) <= 1e-6` early exit; `LimitStalled` means every step
of the last 100 iterations was clamped to a prismatic joint limit and the error reduction
over that window, extrapolated to the remaining iteration budget, cannot reach `posTol`
(a solve pinned against a limit); `MaxIterations` is the 750-iteration cap; `Converged` means the loop
exited because `dist2Tgt <= posTol`. The 3- and 4-argument overloads call the report
overload with `nullptr`, so the limit-streak exit applies to them too; the report itself
is purely observational.

`[[nodiscard]] std::string domainViolation(tau, tol = 1e-9) const` checks a
4-DoF `tau` (physics order `[β1, β2, α1, α2]`) against the trained domain and
returns an empty string when inside it, otherwise a `"; "`-separated list of
violated bounds (e.g. `"b1-b2=-0.0861 < -0.0840; |a2|=4.9000 > 4.7124"`).
6-DoF layouts are not domain-checked (always returns empty). Trained 4-DoF
domain (absolute, physics order):

| Joint | Range |
|---|---|
| β2 | `[-0.072, -0.034]` m |
| β1 − β2 | `[-0.084, -0.030]` m (⇒ absolute β1 `[-0.156, -0.064]` m) |
| α2 | `[-1.5π, 1.5π]` rad |
| α1 − α2 | `[-π, π]` rad (⇒ absolute α1 `[-2.5π, 2.5π]` rad) |

Stage clearance (`getStageThickness()`) is `0.030` m, enforced as `β1 ≤ β2 - 0.030`.

Tip-force domain: `getMaxTipForce()` returns the trained bound `f_max` (N) from
`dataset_params.f_max` (`0.5` for the `_FP32`/`_FP64`/`_v3` models, `0.25` for the
unsuffixed one; `0` for models whose metadata lacks the key, e.g. `grassmann_*`,
`handheld_*`). `[[nodiscard]] std::string forceDomainViolation(wf, tol = 1e-6) const`
returns empty when `‖wf‖ < f_max − tol` (or `f_max == 0`), otherwise e.g.
`"|f|=0.6403 N >= f_max=0.5000 N"`. A value on the surface counts as a violation:
the robot EKF clamps its estimate to that surface, so a value there is railed.

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

`test/test_pos_ctrl_bounds.cpp` (gtest; loads the `ctr_8x91_0.18_tanh_9K_9K_50K_FP64`
model from the source `models/` directory, ~20 s) checks that `posCTRL` from the fully
retracted home converges for targets that need a retracted inner tube, solves
x-mirrored targets alike, and keeps every solution inside the dataset joint domain.

```bash
colcon test --packages-select ctr_kinematics_pinn
```

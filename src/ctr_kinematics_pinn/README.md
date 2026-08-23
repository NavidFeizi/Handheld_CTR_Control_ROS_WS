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
| Geometry / limits | `getArclengthEnd`, `getStraightLen`, `getOverallLen`, `getInputPosBounds()` → `(lb, ub)`, `getDatasetInputRanges()` → `(lb, ub)`, `getPrismaticJointRanges`, `getRevoluteJointRanges`, `getNumNodes`, `getStageThickness` |
| Pseudoinverse | `pInv` (fixed 3×6) and `static pInvN<N>` (damped, λ = 1e-12) |

`posCTRL` is resolved-rate with a null-space joint-limit-avoidance term. It is
**best-effort and returns `void`**: on failure it silently writes back the
closest-seen configuration, so callers must check the resulting tip position if
convergence matters (`Planner::solveInverseKinematics` does this and returns a
`bool`).

Its budget is a **total of 3000 descent steps** per call, split into attempts of
at most 750 (the historical cap). When an attempt stalls — the resolved rate
falls below `linfNorm(dtau_dt) ≤ 1e-6` — it is abandoned and the joint vector is
re-seeded from a random feasible configuration, up to 3 extra seeds. A single
resolved-rate descent is a local method, so a poor initial guess cannot be
rescued by more steps in the same basin; re-seeding is what actually reaches
distant targets. The best-seen configuration is tracked across *all* attempts, so
re-seeding can never return a worse answer than a single descent would have.

Re-seeding is **deterministic**: the RNG is function-local with a fixed seed, so
the same target from the same initial guess always yields the same joint vector.
Do not make that seed time- or state-dependent — two identical plan requests
would then deploy the robot differently.

The integral term is anti-windup limited: integration freezes while a prismatic
joint is saturated against its limit, and the accumulator is capped so `ki·∫e`
cannot outgrow `kp·e`. Without this the descent limit-cycles rather than
converging, and a larger iteration budget just buys more oscillation.

Cost per step is one `jacobian` (a TorchScript forward plus three autograd
backward passes) and one `getPosDistal`, so worst-case latency scales linearly in
the total budget. `planner_node` prints `IK time:` for every solve; keep it well
under the manager's `planner_timeout_s` (15 s), which also has to cover the OMPL
solve.

### β₁ is stored relative to β₂ — pick the right accessor

In the 4-DoF datasets `parameters.json` stores `beta1_range` as an offset **relative to
β₂**, not as an absolute bound. For the shipped handheld model that is
`[-0.084, -0.030]` against `beta2_range = [-0.072, -0.034]`, so β₁'s absolute travel is
`[-0.156, -0.064]` — exactly `robot_node`'s `k_home_pos[1]` … `k_pos_preEngage[1]`. The
relative window *is* the tube-coupling window: 30 mm stage thickness at the top, the
216 mm − 132 mm active-length difference at the bottom.

Two accessors, and they are not interchangeable:

| Accessor | β₁ frame | Use it when |
|---|---|---|
| `getInputPosBounds()` | **absolute** | you constrain a joint value directly — OMPL state-space bounds, `CTR_StateValidityChecker`, the samplers, `posCTRL`'s joint-limit avoidance |
| `getDatasetInputRanges()` | dataset-native (**relative**) | the caller re-applies the coupling itself by shifting the window by the live β₂ — `ctr_common::clampJointPositions`, used by `pinn_fk` |

`getPrismaticJointRanges` reports absolute travel and converts internally. The
conversion itself lives in `dataset_bounds.hpp`, which is deliberately Torch-free so it
can be unit tested (`planner`'s `test_dataset_bounds`).

Mixing the two is not a rounding error: feeding the relative window in where an absolute
bound is expected collapses β₁'s admissible interval to the empty set at the retracted
pose, and every `setStartState()` throws `Start state is invalid!`. That was a real
regression in this workspace — `3efc430` dropped the conversion when it merged the three
PINN copies, and the planner could not plan from home until it was restored.

### α₁ is relative to α₂ — and that one is a HARD constraint

The loader prints the convention for every joint, not just β₁:

```
Dataset parameters:
    beta1_range: beta2 + [-0.084, -0.03]
    beta2_range: beta3 + [-0.072, -0.034]
    alpha1_range: alpha2 + [-3.14159, 3.14159]
    alpha2_range: alpha3 + [-6.28319, 6.28319]
```

Every range is stored relative to the next-outer tube; β₃ and α₃ are always 0, so β₂ and
α₂ are effectively absolute and only β₁ and α₁ carry an offset.

For α₁ the relative form is not a bookkeeping detail to be converted away — it is the
constraint the model was **trained** under:

> **α₂ − π ≤ α₁ ≤ α₂ + π must hold for every configuration**, everywhere: sampled states,
> interpolated motions, IK outputs, planned waypoints, and joint targets sent to hardware.

Feed the PINN a configuration outside that band and its output is not merely inaccurate,
it is unconstrained extrapolation. Do **not** "fix" α₁'s bound the way β₁'s was fixed:
the planner enforces the band through `CTR_StateValidityChecker`'s `conditionAngle` term
and `CTR_DiscreteMotionValidator`, which is the correct place for a relative constraint.
Widening α₁'s box bound would also change `m_space->getMaximumExtent()`, which sets both
the planner's step range and `setLongestValidSegmentFraction` (`Planner.hpp:417-424`,
`:864`) — retuning the motion validator's granularity as a side effect.

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

No test target in this package: exercising the inference path requires LibTorch and a
loaded TorchScript model, which the workspace's hardware- and Torch-free test suites
deliberately avoid. `mpc`'s `test_mpc_qp` covers the QP layer by substituting a linear
plant for the PINN, and `planner`'s `test_dataset_bounds` covers `dataset_bounds.hpp`
(the β₁ relative→absolute conversion), which is kept Torch-free for exactly that reason.

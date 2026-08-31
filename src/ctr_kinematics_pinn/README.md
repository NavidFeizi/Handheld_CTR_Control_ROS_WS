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
| Inverse kinematics | `posCTRL(tau, target, posTol[, wf][, IkDiagnostics*])` — resolved-rate IK, mutates `tau` in place; the optional out-param reports iterations, restarts, clamp/uphill-step counts and conditioning |
| Geometry / limits | `getArclengthEnd`, `getStraightLen`, `getOverallLen`, `getInputPosBounds()` → `(lb, ub)`, `getDatasetInputRanges()` → `(lb, ub)`, `getJointLimits4()` → the shared feasible set, `getPrismaticJointRanges`, `getRevoluteJointRanges`, `getNumNodes`, `getStageThickness` |
| Pseudoinverse | `pInv` (fixed 3×6) and `static pInvN<N>` (damped, λ = 1e-12) |

`posCTRL` is resolved-rate with a null-space joint-limit-avoidance term. It is
**best-effort and returns `void`**: on failure it silently writes back the
closest-seen configuration, so callers must check the resulting tip position if
convergence matters (`Planner::solveInverseKinematics` does this and returns a
`bool`).

It **never returns a configuration the planner would reject**: the value written
back is projected into the feasible set (see the shared predicate below) before
`finish()` reports. That is a guarantee by construction, not a statistical claim,
and it matters because `Planner::setGoalState` *throws* on an infeasible goal — a
solution a few millimetres out is not a slightly worse answer, it is no plan at
all.

Its budget is a **total of 3000 descent steps** per call, split into attempts of
at most 250. When an attempt stalls — the resolved rate falls below
`linfNorm(dtau_dt) ≤ 1e-6` — it is abandoned and the joint vector is re-seeded
uniformly from the feasible set, up to 11 times. The split is set from measured
data, not taste: attempts that succeed do so in well under 250 steps, while
attempts that will stall burn their whole allowance and contribute nothing, so
the same budget buys far more as many short tries than as a few long ones.
Re-seeding is what actually rescues a hard target; the best-seen configuration is
tracked across *all* attempts, so it can never return a worse answer than a
single descent would have.

Re-seeding is **deterministic**: the RNG is function-local with a fixed seed, so
the same target from the same initial guess always yields the same joint vector.
Do not make that seed time- or state-dependent — two identical plan requests
would then deploy the robot differently. Keep every restart independent, too: a
deterministic mid-range seed for the first retry was tried and measured *worse*
(misses 3.2% → 7.0%), because basin diversity buys more than avoiding the
boundary corner the robot homes to.

The integral term is anti-windup limited: integration freezes while a prismatic
joint is saturated against its limit, and the accumulator is capped so `ki·∫e`
cannot outgrow `kp·e`.

Cost per step is one `jacobian` (a TorchScript forward plus three autograd
backward passes) and one `getPosDistal`, so worst-case latency scales linearly in
the total budget. `planner_node` prints `IK time:` for every solve; keep it well
under the manager's `planner_timeout_s` (15 s), which also has to cover the OMPL
solve.

### Measured behaviour — and what is NOT wrong with it

`benchmark/ik_bench.cpp` (opt-in, `-DCTR_PINN_BUILD_BENCH=ON`) measures all of
this. It forward-samples targets — draw a feasible joint vector, take its FK tip —
so a solution provably exists and every miss is a solver failure rather than an
unreachable target. Over 500 targets solved from the `k_pos_preEngage` pose:

| | before | after |
|---|---|---|
| goal rejected by `setGoalState` | 21.2% | **0.0%** |
| residual < 3 mm (manager's gate) | 100%¹ | 100% in every bucket ≥ 20 mm |
| worst-case solve time | 15.8 s² | 4.5 s |

¹ achieved by parking β₂ outside the legal set — a larger, easier problem than
the planner accepts. ² which alone exceeded `planner_timeout_s`.

Three things the measurement **ruled out**, so nobody re-litigates them:

- *Overshoot.* Uphill steps are 0.00% of the median run. The descent does not
  oscillate, and a line search would fix nothing.
- *Singularity blow-up.* `‖J⁺‖_F` peaks around 1.2e3 with λ = 1e-12. Not
  exploding; adaptive damping is not needed.
- *The nullspace term causing the joint clamping.* Cutting its gain tenfold left
  the clamp rate at 83% unchanged. The clamp is driven by the **task** step
  wanting β outside its coupled window, which is intrinsic to the mechanism.

What the nullspace gradient *was* missing is dimensional scaling: the bracketed
factor is dimensionless in [0, 1] and was used directly as a velocity in metres,
against windows only ~40–54 mm wide. It is now scaled by the window half-width.

Note that the pose the robot homes to is a **corner** of the feasible set: at
β₁ = −0.064 the coupling pins β₂ to exactly −0.034, a zero-width window. Targets
*near* the retracted pose are therefore the hard ones, not the distant ones — the
opposite of the intuition.

### The feasible joint set lives in one place

`dataset_bounds.hpp` owns the answer to "is this configuration legal": `JointLimits4`,
`isFeasible4()`, `beta1Window()`, `beta2Window()`. Both `posCTRL` and the planner's
`CTR_StateValidityChecker` go through it, and they must keep doing so.

They did not always. Each carried its own copy and each dropped a *different*
constraint, in opposite directions:

| | dropped | consequence |
|---|---|---|
| `posCTRL` | β₂'s own ceiling `beta2_range[1]` (−0.034); it capped at −stage-thickness (−0.030) | returned converged goals 4 mm inside a band the planner rejects → `setGoalState` throws → **no plan at all**, measured at 21% of solves |
| `CTR_StateValidityChecker` | β₁ ≥ β₂ + `beta1_range[0]` | accepted states where the inner tube retracts inside the middle one — physically meaningless and outside the PINN's training box |

For the 4-DoF layout the whole set is four constraints, exactly as the dataset
states them:

```
β₂      ∈ beta2_range     [-0.072, -0.034]
β₁ − β₂ ∈ beta1_range     [-0.084, -0.030]
α₂      ∈ alpha2_range    [-1.5π, 1.5π]
α₁ − α₂ ∈ alpha1_range    [-π, π]
```

The β₁ relative window is doing double duty, which is exactly why half of it is easy
to lose: its **upper** edge −0.030 *is* the stage clearance (β₁ ≤ β₂ − clr) and its
**lower** edge −0.084 *is* the tube-protrusion constraint β₁ + L₁ ≥ β₂ + L₂
(L₂ − L₁ = 0.132 − 0.216). Enforce the window and both come for free; enforce one
bound alone and you silently drop the other. The other geometry terms are not extra
constraints — L₃ − L₁ = −0.156 and L₃ − L₂ = −0.072 are precisely the absolute floors.

`isFeasible4` takes a tolerance because **both poses the hardware homes to sit exactly
on the boundary, at opposite corners**: `k_home_pos` (β₁ −0.156, β₂ −0.072) on the
lower one, `k_pos_preEngage` (−0.0640, −0.0340) on the upper. Measured encoder values
straddle those by floating-point noise. Pass a small tolerance for a measured pose,
zero for a computed one. `planner/test/test_dataset_bounds.cpp` pins all of this, and
is Torch-free so it runs anywhere.

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

### α₁ is relative to α₂ — the exact same convention as β₁

The loader prints the convention for every joint, not just β₁:

```
Dataset parameters:
    beta1_range: beta2 + [-0.084, -0.03]
    beta2_range: beta3 + [-0.072, -0.034]
    alpha1_range: alpha2 + [-3.14159, 3.14159]
    alpha2_range: alpha3 + [-4.71239, 4.71239]
```

Every range is stored relative to the next-outer tube; β₃ and α₃ are always 0, so β₂ and
α₂ are effectively absolute and only β₁ and α₁ carry an offset. The absolute forms are
therefore α₂ ∈ ±1.5π (which is also the drives' rotary travel, `robot_node`'s
`k_maxStaticLimitAll[2]`) and α₁ ∈ ±2.5π.

For α₁ the relative window is additionally the constraint the model was **trained**
under:

> **α₂ − π ≤ α₁ ≤ α₂ + π must hold for every configuration**, everywhere: sampled states,
> interpolated motions, IK outputs, planned waypoints, and joint targets sent to hardware.

Feed the PINN a configuration outside that band — or outside α₂'s ±1.5π travel — and its
output is not merely inaccurate, it is unconstrained extrapolation (the normaliser baked
into the TorchScript archive spans exactly 1.1× those ranges; there is no clamp inside
the network). `getInputPosBounds()` converts **both** relative windows (β₁ and α₁) to
absolute box bounds; `isFeasible4` enforces the α pair in its dataset-native form.

Historical note: until 2026-08 `alpha1_range` was consumed as an *absolute* α₁ box with
α₂ anchored to α₁ — the mirror image of the correct domain. That spilled α₂ out to ±2π
(untrained AND physically unreachable), creating an azimuthal wedge around α₁ ≈ ±π where
the IK "converged" on extrapolated garbage and executed plans landed visibly off target.
The wedge is exactly 1/16 of α-space, which is why the failure looked like "targets on
one side of the workspace plan fine, diametrically opposed ones never work". posCTRL's
wrap, its restart seeding, `finish()`'s projection, the OMPL bounds/sampler and the
manager's pre-rotation all anchor on α₂ now; `fk_xcheck` (in `ctr_cosserat`) documents
the extrapolation error in the old wedge against the Cosserat model.

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

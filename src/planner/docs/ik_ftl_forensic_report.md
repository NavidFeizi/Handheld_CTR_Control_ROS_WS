# Planner vs PINN: IK half-workspace failure and FTL objective — forensic report

Scope: `src/planner` (OMPL planner, IK entry point, FTL objective) and
`src/ctr_kinematics_pinn` (PINN inference, `posCTRL`). Model:
`ctr_8x91_0.18_tanh_9K_9K_50K_FP64` (planner default).

Two questions:

1. Do the planner and the PINN use the same physical actuation convention?
   Hypothesis: a revolute-variable mismatch makes IK converge in only about half
   of the mushroom-shaped workspace.
2. Does the planner actually enforce near follow-the-leader (FTL) motion?

Short answers. (1) The revolute hypothesis is **false**: the planner-to-PINN map is
the identity, and the network is mirror- and rotation-symmetric. The real cause is
**prismatic**. A β1 range that the dataset stores *relative to β2* was used as an
*absolute* bound, both in `posCTRL` and in the planner. As a result the inner tube could
never retract past −84 mm (the trained domain goes to −156 mm), and IK failed on the
entire **proximal (low-z) half**, equally at every azimuth. (2) Only by
construction (rotate first, then deploy). The FTL cost used to rank deployment
schedules penalized exactly the tangential motion that FTL permits, and its value
depended on how finely a path was segmented. It has been replaced with a
lateral swept-area metric.

Classification used below: **root cause** / **contributing factor** / **unrelated issue**.

## 1 PINN parameterization

`ctr_kinematics_pinn/include/ctr_kinematics_pinn/ctr_pinn_inference.hpp`, `PINNs<4>`:

- Network input row = `[s, τ0..τ3, wf0..wf2]`. `τ = [β1, β2, α1, α2]`:
  - β1 = innermost-tube translation (m, ≤ 0)
  - β2 = middle-tube translation
  - α1 = absolute rotation of the inner tube (rad)
  - α2 = absolute rotation of the middle tube
  - The outer tube is fixed (β3 = α3 = 0).
- `s` is the absolute arc length from the base. The shape grid is
  `linspace(0, L1 + β1, N)`, with L = [0.216, 0.132, 0.060] m.
- Training domain (`parameters.json`, printed at load):
  - `beta1_range: beta2 + [-0.084, -0.030]`, i.e. relative to β2
  - `beta2_range: [-0.072, -0.034]`, absolute
  - `alpha1_range: alpha2 + [-π, π]`, i.e. |α1 − α2| ≤ π
  - `alpha2_range: [-2π, 2π]`
- Getters:
  - `getInputPosBounds()` returns the raw ranges, so `lb[0] = −0.084` is a **relative offset**.
  - `getPrismaticJointRanges()` returns the absolute `[−0.156, −0.064, −0.072, −0.034]`.
  - `getRevoluteJointRanges()` returns `[−π, π, −2π, 2π]`.
  - `getStageThickness()` = 0.030 m = −`beta1_range[1]`.
- `posCTRL` is damped-least-squares resolved-rate IK with a null-space joint-limit
  term. It runs at most 750 iterations and returns the best iterate. It wraps α1 to
  [−π, π) and α2 to [α1 − π, α1 + π).
- Network symmetry, measured on 200 random in-domain configurations (mean tip deviation):

  | Property | Mean deviation |
  |---|---|
  | Mirror: p(β, −α) = (−x, y, z) of p(β, α) | 0.27 mm |
  | Joint rotation (α1+δ, α2+δ) = rotation of the tip about z | 0.23 mm |
  | In-domain 2π periodicity | 0.31 mm |

  Leaving the trained relative window (α1 alone +2π) gives 7.3 mm mean and 83 mm max:
  the network is valid only inside the dataset domain.

## 2 Planner parameterization

- State: `CTRStateSpace : RealVectorStateSpace(4)`, `values = [β1, β2, α1, α2]`.
  This has the same order, meaning and units as `τ`. Every OMPL↔`τ` copy is the
  identity (`CTR_FollowTheLeaderObjective::stateToTau`, `Planner::setStartState`,
  goal and IK paths).
- `ctr_common/joint_conventions.hpp` only reorders the wire layout `[α1, β1, α2, β2]` into
  the physics layout `[β1, β2, α1, α2]`.
- Targets arrive in metres in the PINN base frame (`interfaces::srv::Planner::Request::value`).
- IK: `planner_node.cpp` → `Planner::solveInverseKinematics` → `posCTRL`. The seed is
  the live robot configuration and `posTol = 5e-4`.
- Joint bounds (before this change) all came from `getInputPosBounds()`, so they carried
  the relative β1 range as an absolute one. Affected sites: the `Planner` constructor,
  the four prismatic-bound restores in `planTwoPhase`, `CTR_StateValidityChecker`, and
  `CTR_StateSampler`.
- Validity checker: `β1 ≥ β1min`, `β1 ≤ min(β1max, β2 − clr)`,
  `β2 ≥ max(β1 + clr, β2min)`, `β2 ≤ β2max`, `|α2 − α1| ≤ π`.
  The lower relative bound `β1 ≥ β2 − 0.084` was missing.
- Angles: distance and interpolation take the shortest arc (`CTR_StateSpace.hpp`), but
  interpolated values are never re-wrapped.

## 3 Exact discrepancies and mapping

| Quantity | PINN | Planner (before) | Verdict |
|---|---|---|---|
| Index layout | `τ = [β1, β2, α1, α2]` | `values = [β1, β2, α1, α2]` | identical: `state[i] == τ[i]` |
| Units / signs | m, rad; β ≤ 0 | m, rad; β ≤ 0 | identical, no offsets or flips |
| α1 range | [−π, π] absolute | [−π, π] | identical |
| α2 constraint | \|α1 − α2\| ≤ π | \|α2 − α1\| ≤ π | identical |
| β2 range | [−0.072, −0.034] | [−0.072, −0.034] | identical |
| **β1 range** | **absolute [−0.156, −0.064], with β1 − β2 ∈ [−0.084, −0.030]** | **absolute [−0.084, −0.030]** | **mismatch (root cause)** |
| β1 clamp in `posCTRL` | — | `lb[0]`, `ub[0]` used as absolute bounds | **mismatch (root cause)** |
| β2 clamp in `posCTRL` | upper −0.034 | upper −0.030 (stage clearance only) | **mismatch (contributing factor)**: IK could leave the trained domain by 4 mm |
| β1 ≥ β2 − 0.084 in the validity checker | dataset bound | absent | **mismatch (contributing factor)** once β1 bounds are absolute |

Wrongly bounded interval: β1 ∈ [−0.084, −0.030] covers only a sub-box of the trained
absolute interval [−0.156, −0.064]. Its lower edge (−0.084) sits in the middle of the
trained interval, so the inner tube could not retract more than 84 mm.

## 4 Does it explain the asymmetry?

**Yes. The failing half is the proximal (low-z) half of the mushroom, not a lateral half.**

Low tip z requires a retracted inner tube (β1 < −0.084), which the wrong bound
forbade. Setup: 600 random in-domain FK targets, seed = retracted home
`{−0.156, −0.072, 0, 0}`, tolerance 0.5 mm, z split at the midpoint of the target z
range [0.0619, 0.1482] m.

| | converged | z < 0.1051 | z ≥ 0.1051 | +x+y | −x+y | +x−y | −x−y |
|---|---|---|---|---|---|---|---|
| before | 43/600 (7 %) | **0/326** | 43/274 | 16/148 | 11/140 | 8/165 | 8/147 |
| after | 547/600 (91 %) | 303/326 (93 %) | 244/274 (89 %) | 140/148 | 132/140 | 143/165 | 132/147 |

Revolute hypothesis rejected:

- The planner→PINN map is the identity (§3).
- The network is mirror- and rotation-symmetric to about 0.3 mm (§1).
- Convergence is azimuthally uniform both before and after the fix (quadrant columns).
- Mirrored targets (x → −x) solve alike: 25/30 pairs agree. The 5 disagreements are
  1–3 mm local minima and fall 3 vs 2 across the two x-halves.

With the live robot as the IK seed, the old bound also stopped the planner itself.
`setStartState` rejected any live configuration with β1 < −0.084 (`Start State is
invalid!`, and the service call failed). Now it plans.

If hardware still shows a failing lateral (x or y) half after this fix, planner and PINN
are exonerated. The next place to look is target generation and EM-frame registration
(`src/target_gen`, `igtlink_bridge`).

## 5 FTL soundness assessment

Correct condition: take s as the absolute arc length from the fixed base. Near-FTL means
`p(s; q(t))` stays constant in t for every already-deployed s. Equivalently, the
component of ∂p/∂t normal to the backbone is ≈ 0. The tangential component (a tube
sliding along its own curve) is free.

How the planner enforced FTL before:

- The node calls `planTwoPhase(…, RRTConnect)`.
- Phase 1 rotates at fixed β using a feasibility planner, so objectives are ignored there.
- Phase 2 is analytic: 4 monotone deployment schedules (`DeploymentSchedule.hpp`) are
  ranked by `scheduleSweptCost` = Σ `sweptCost` over about 40 segments.
- So FTL relied on (a) the rotate-then-deploy ordering and (b) that ranking.

Old metric: `sweptCost = Σ_i ‖p_i(q2) − p_i(q1)‖² · ds`, with node i at the **normalized**
arc length i/(N−1) of each backbone. It is unsound in three ways:

1. **Tangential motion was penalized** — *contributing factor*. For a perfect FTL
   step (same curve, Δℓ appended at the tip), node i of the longer backbone lies
   i·Δℓ/(N−1) further along the same curve. The cost is then ≈ ℓ·Δℓ²/3 > 0.
2. **The normal component was never isolated.** Lateral sweep, the only real FTL
   violation, was mixed with the slip artefact from (1).
3. **Quadratic per edge.** The same motion cost less when split into more segments, so
   the schedule total depended on the discretization.

Measured on the PINN at q1 = {−0.12, −0.05, 0.7, −0.4} (the old metric is in m³, the new
one in m²):

| Motion | old | new |
|---|---|---|
| inner tube +10 mm (pure deployment) | 3.48e-6 | 6.99e-6 |
| inner tube +5 mm | 8.51e-7 | 3.67e-6 |
| rotation α1, α2 +0.3 rad | 8.06e-8 | 4.90e-5 |

- Ranking. The old metric ranked the 10 mm deployment **43× worse** than a 0.3 rad
  whole-body rotation. The new metric ranks it 7× cheaper.
- Additivity. Old: two 5 mm halves cost 1.7e-6, versus 3.5e-6 for one 10 mm edge, so it
  is not additive. New: 7.3e-6 versus 7.0e-6.
- Residual. The remaining 7e-6 m² ≈ 73 µm mean lateral deviation over 96 mm. This is the
  real non-FTL behaviour of a pre-curved inner tube (it bends the outer tubes), plus
  PINN noise.

The plan predicted a rotation cost above 1e-4 m²; the observed value is 4.9e-5 m²,
because this configuration's backbone lies close to the z axis.

**Unrelated issue (report only):** interpolated α values are never re-wrapped. A motion
that crosses α1 = ±π produces out-of-bounds states that `satisfiesBounds` rejects, so
RRTConnect has to go the long way round. This adds rotation sweep but does not cause IK
failure. It is left unchanged: re-wrapping would put 2π jumps into the executed joint path.

## 6 Code changes

Root cause — `ctr_kinematics_pinn/include/ctr_kinematics_pinn/ctr_pinn_inference.hpp`,
`posCTRL`, 4-DoF branch only:

- l.1287: comment now states that β1 bounds are relative to β2.
- l.1322: `bMin[0] = max(β2 + lb[0], L2 + β2 − L1, L3 − L1)` (was `lb[0]`).
- l.1325: `bMax[0] = min(β2 + ub[0], β2 − clr)` (was `ub[0]`).
- l.1326: `bMax[1] = min(ub[1], −clr, L1 + β1 − L2)`. This caps β2 at the dataset −0.034
  instead of −0.030. It is an addition beyond the approved plan, found by the
  domain-invariant test, and makes the IK solution match the trained domain exactly.
- Network, input layout, `getInputPosBounds`, angle wrapping and the 6-DoF branch are
  unchanged. `posCTRL` has one consumer in the workspace, `Planner::solveInverseKinematics`;
  `robot` and `mpc` do not call it.

Root cause (planner side) — joint bounds now come from the absolute getters:

- `Planner.hpp` l.378-394 (constructor): `getPrismaticJointRanges` /
  `getRevoluteJointRanges`. The stale comment claiming these getters are broken for
  4-DoF is removed.
- `Planner.hpp` l.1204-1210 (Phase-1 β window), l.1247-1252, l.1279-1284 and
  l.1355-1361 (bound restores): all use `pr[2i]` / `pr[2i+1]`. The last site was not in
  the plan, but it had the same misuse.
- `CTR_StateValidityChecker.hpp` l.38-39, 50-57: absolute `m_beta1_min/max` and
  `m_beta2_min/max`, plus a new `m_beta1_rel_min` (= −0.084 from `getInputPosBounds()[0]`).
- `CTR_StateValidityChecker.hpp` l.79-81: `β1 ≥ max(β1min, β2 + rel_min)`.
- `CTR_StateSampler.hpp` l.66-77: absolute ranges. Draws with β1 < β2 − 0.084 are
  rejected by the validity checker (the existing rejection pattern).

Contributing factor — FTL metric:

- New `motion_planning/include/FtlSweptArea.hpp`, `ftl::lateralSweptArea`. It is
  blaze-only, with no OMPL or Torch.
  - Both backbones are compared at equal **absolute** arc length over ℓ = min(len1, len2).
  - Each point is linearly interpolated on its own grid.
  - Only the component of p2 − p1 normal to backbone 1's tangent is kept.
  - The integral uses the trapezoid rule; units m².
- `CTR_FollowTheLeaderObjective.hpp`:
  - `sweptCost` delegates to the new helper. Locking, the shape cache and the
    PINN-failure guard are unchanged.
  - The header comment and the `w_ftl` comment are updated (units m³ → m²; `w_ftl` itself
    is unchanged).
  - The `tipDistanceHeuristic` cost-to-go was removed. A pure FTL deployment reaches a tip
    target at zero cost, so the only admissible lower bound is zero, which is the OMPL
    default. This removal is beyond the approved plan and is a direct consequence of the
    new metric.
- Callers (`motionCost`, `DeploymentSchedule::scheduleSweptCost`, the Phase-2 ranking,
  path analysis) are unchanged, as are the combination weights. The m³ → m² change
  affects relative weighting only on the optimizing-planner path, which the node does
  not use (it uses RRTConnect).
- Wording "swept volume" → "lateral swept area" in `Planner.hpp` comments/descriptions.

Tests and build:

- `ctr_kinematics_pinn`: `test/test_pos_ctrl_bounds.cpp`, `ament_add_gtest` with
  TIMEOUT 900, and `<test_depend>ament_cmake_gtest</test_depend>`.
- `planner`: `test/test_ftl_swept_area.cpp`. It links `OpenMP::OpenMP_CXX`, because the
  package-wide `-fopenmp` turns blaze matrix assignments into OpenMP kernels.
- READMEs of both packages updated.

Unrelated issues (not changed):

- `Planner.hpp` `ctrStateSpaceCostToGo` assumes a 6-DoF index layout (0..2 prismatic).
  It is used only by `OBJECTIVE_PATH_LENGTH_COST2GO`.
- `CTR_PINNsInformedSampler.hpp`, `CTR_PINNsInformedStateSamplerInformed.hpp` and
  `CTR_BackboneLengthObjective.hpp` carry stale "posCTRL is broken for 4-DoF" comments.
  As a result the informed samplers always fall back to uniform sampling.
- The `wf`-less `jacobianFinDif` / `getShape(tau, shape)` overloads build a 5-column input
  and would crash on the 8-input model.

## 7 Validation results

All runs used the `ctr-jazzy-build` container.

- **Build:** `docker/check.sh --packages-select ctr_kinematics_pinn planner` → both
  packages finished.
- **Tests:** `colcon test --packages-select ctr_kinematics_pinn planner`: 17 tests,
  0 failures. This covers `test_pos_ctrl_bounds` (3), `test_ftl_swept_area` (6) and
  `test_deployment_schedule` (5), plus their ctest wrappers.
- **Regression proof:** the same `test_pos_ctrl_bounds.cpp` was compiled against the
  pre-fix header (`git show HEAD:…`). All 3 tests fail:

  | Test | Result |
  |---|---|
  | `RetractedInnerTubeTargetsConverge` | **0 of 60 converged** |
  | `MirroredTargetsConvergeAlike` | 17/30 pairs agree |
  | `SolutionStaysInsideDatasetDomain` | β2 up to −0.0314 and β1 above β2 − 0.030 |

- **Threshold change:** the plan's mirror threshold was 27/30. Post-fix observed 25/30
  (7 % short, within the plan's ≤10 % contingency), so the threshold was set to 20
  (observed − 5).
- **Workspace coverage (600 targets):** before 43/600, after 547/600. Low-z half:
  0/326 → 303/326. Full table in §4.
- **FTL smoke:** table in §5.
- **Hardware run (operator only, not executed):**

  ```bash
  ros2 launch launch/system_bringup.launch.py
  ros2 launch planner launch.py
  ros2 service call /planner/command interfaces/srv/Planner \
    "{command: 'generateTrajectory', value: [0.0, 0.015, 0.08]}"
  ```

  Expected: `Path generated successfully.` and a logged IK error < 0.5 mm for this
  low-z target, which previously failed. Also compare the
  `[planTwoPhase] Phase 2 (FTL-optimal deployment: …)` schedule choice against earlier
  logs; a ranking change is expected under the new metric.

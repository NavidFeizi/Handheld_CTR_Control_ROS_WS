# Saturated EKF force estimate and the "one half" IK failure — forensic report

Scope: field run 2026-10-12 (`refactor/architecture`). Packages touched: `ctr_kinematics_pinn`,
`robot` (EKF, `robot_node`), `planner`, `manager`, `mpc`, `emtracker`, `igtlink_bridge`.
Sibling of [`ik_ftl_forensic_report.md`](ik_ftl_forensic_report.md); the fixes recorded there
(β1 relative→absolute in `posCTRL`, α2 ±1.5π, `m_flag_manual` init, package-log tee, stage
tags) were present in this build and are not repeated here.

Evidence:

- `~/Desktop/Logs/ctr_logs/20261012_174030/{bringup,planner,manager}.log` (console tees)
- `~/Desktop/Logs/log/2026-10-12_17-40-30/{robot,planner,manager}/*.log` (package logs)
- `~/Desktop/Logs/log/Robot/2026-10-12_17-41-19.txt` (CANopen driver)

There are no screenshots in that directory, so the GUI state described below is reconstructed
from the manager log and the GUI code.

## 1 Field failure (2026-10-12)

Four procedures ran. The three whose targets had y > 0 planned and deployed (17:49, 18:27,
18:39). In the one at 18:31:38–18:39:47 the operator picked a target with y < 0,
`[-0.0036, -0.0261, 0.1252]` m, and pressed **Start Procedure**. Nothing visible happened after that.

The manager re-requested the plan over and over (requests #20…#90). Each of those requests:

```
[plan] force estimate f = [-0.3512, 0.5000, 0.1855] N                       (#20)
[ik] q=[…] tip=[-0.0088, 0.0012, 0.1206] error=28.1839 mm iterations=750
     termination=MaxIterations … limit_hits=750
[comm] response #20 success=true value=0.028184
       message="Path generated successfully." reason=IK_NOT_CONVERGED
```

`master_node`: `[plan] plan rejected: ik_error=28.184 mm >= threshold=3.000 mm`, and the
same pattern repeats up to #90 (`f=[-0.5000, 0.5000, -0.3878]`, ‖f‖ = 0.81 N, 39.7 mm). The
GUI showed **Success: Yes** next to a red IK error, with no reason. From the operator's side,
"nothing happens".

Every `generateTrajectory` request of the day, parsed from `planner.log` (86 requests with a
force line and an `[ik]` line; "saturated" means ‖f‖ ≥ 0.5 N):

| Target half | Force estimate | Requests | IK error < 3 mm | IK error range (mm) |
|---|---|---|---|---|
| y < 0 | saturated | 73 | 0 | 8.4 – 43.8 |
| y < 0 | in domain | 0 | — | — |
| y > 0 | saturated | 5 | 2 | 0.84 – 7.2 |
| y > 0 | in domain | 8 | 3 | 0.49 – 5.7 |

78 of the 86 force snapshots were at or beyond ‖f‖ = 0.5 N; 74 had at least one axis railed
at exactly ±0.5000 N. The largest norm was 0.81 N.

The contrast within a single target is the cleanest evidence. Requests #13 and #14 (y > 0,
`f = [-0.10, 0.27, 0.06]`, ‖f‖ = 0.29 N) converged in about 100 iterations to 0.49 mm.
Request #11 asked for the same target as #13 with `f = [-0.10, -0.03, -0.5000]` (z railed)
and stopped at 4.2 mm.

## 2 Root cause

**The EKF force estimate saturated with no tissue contact. The planner fed it, unchecked, as
the tip load `wf` into PINN IK.**

1. `robot/src/ekf_node.cpp` clipped the state **per axis** to ±0.5 N (`m_force_threshold =
   0.50`). That admits ‖f‖ up to √3·0.5 = 0.87 N.
2. The PINN was trained on ‖f‖ ≤ `f_max` = 0.5 N
   (`models/ctr_8x91_0.18_tanh_9K_9K_50K_FP64/parameters.json` `dataset_params.f_max`; the
   normaliser half-range of about 0.548 N is 0.5 × 1.1 `enlarged_span_factor`). A railed
   estimate therefore drives the network outside its training domain. The PINN's own
   `domainViolation()` checks joints only.
3. `planner_node.cpp` copied the latest `task_space/force_estimate` into `setCTR_externalForce`
   and `posCTRL` with no check. Under a spurious lateral load of about 0.6–0.8 N, the
   y < 0 targets need a deflection the model cannot produce inside the joint box. IK runs
   into the limits on every iteration (`limit_hits=750/750`) and stops near `tip y ≈ 0`.
4. The estimates were railed while the robot was at retracted home with no contact
   (17:49:09, 18:31:38, 18:39:52). That makes them bias absorbed into the force state, not
   a load. The bias could come from a mismatch between the EM tracker and the PINN, from
   registration, or from the R tuning: `robot_params.yaml` R position block 1.1e-10…2.6e-8
   m² trusts the tracker to 10–160 µm. **Which of these it is depends on the hardware and
   cannot be checked off-robot.**

### Why "one half"

The split is not a kinematic symmetry. `ik_ftl_forensic_report.md` §4 already showed the PINN
is mirror-consistent. The bias had a persistent **+y** component (f_y = +0.44…+0.50 N in
almost every snapshot). A +y tip load pushes the tip toward +y. Targets with y > 0 sit on the
side the load helps, or need little correction. Targets with y < 0 need the tubes to work
against a load the model says is there and cannot overcome. All 73 y < 0 requests had
saturated estimates, so the field data alone cannot separate "y < 0" from "saturated force".
The container checks in §6 make that separation: with f = 0 the same field target converges.

## 3 Contributing: the reason never reached the operator

- `planner_node.cpp` answered `success=true`, `message="Path generated successfully."`
  even at 43 mm. The reason code `IK_NOT_CONVERGED` existed only in the planner log.
- `master_node.cpp` `handle_planner_response` rejected on `value ≥ 3 mm` but left
  `m_planner_success = true`, so the GUI showed **Success: Yes**. The `plannerStatusUpdated`
  signal carried no text.

## 4 PINN ↔ planner ↔ robot ↔ MPC range audit

Physics order (β1, β2, α1, α2); units m and rad.

| Stage | β1 | β2 | α1 | α2 | Source |
|---|---|---|---|---|---|
| Trained (relative, as stored) | β2 + [-0.084, -0.030] | [-0.072, -0.034] | α2 + [-π, π] | [-1.5π, 1.5π] | `parameters.json` (FP64 == v3) |
| Trained (absolute envelope) | [-0.156, -0.064] | [-0.072, -0.034] | [-2.5π, 2.5π] | [-1.5π, 1.5π] | `getPrismaticJointRanges` / `getRevoluteJointRanges` |
| `posCTRL` IK | relative β bounds re-derived each iteration incl. 0.030 m clearance; α wrapped so \|α1 − α2\| ≤ π | | | | `ctr_pinn_inference.hpp` |
| Planner OMPL box | absolute | absolute | [-2π, 2π] | absolute | `Planner.hpp:378-395`; coupling in `CTR_StateValidityChecker.hpp` |
| Robot static limits | [-0.156, -0.064] | [-0.072, -0.034] | ±3π | ±1.5π | `robot_node.cpp` (dynamic coupling separate) |
| Manager | no clamp; `[traj]` snapshot WARN only | | | | `master_node.cpp` |
| MPC before | **[-0.084, -0.030] (relative used as absolute)** | ok | **[-π, π]** | ok | `mpc_node.cpp` `getInputPosBounds()` |
| MPC after | [-0.156, -0.064] | [-0.072, -0.034] | [-2π, 2π] | [-1.5π, 1.5π] | `joint_box.hpp` `absoluteJointBox` |

| Force stage | Before | After |
|---|---|---|
| Trained domain | ‖f‖ ≤ 0.5 N | unchanged |
| EKF clamp | per axis ±0.5 N (‖f‖ ≤ 0.87 N) | ‖f‖ ≤ model `f_max` (0.5 N) |
| Planner | none | rejects ‖f‖ ≥ f_max − 1e-6 (`FORCE_OUT_OF_DOMAIN`) |
| MPC | none (consumes the estimate as is) | unchanged; now bounded by the EKF norm clamp |

Joint limits and the PINN's inputs and outputs are unchanged. The trained domain is read as
the ball ‖f‖ ≤ f_max. Whether the training set sampled a ball or a per-axis box cannot be
recovered from the metadata, so the ball is the conservative reading.

## 5 Changes

| Package | File | Change |
|---|---|---|
| `ctr_kinematics_pinn` | `ctr_pinn_inference.hpp` | `DatasetParameters::f_max` loaded with `dataset.value("f_max", 0.0)`; `getMaxTipForce()`; `forceDomainViolation(wf, tol)` (empty unless ‖wf‖ ≥ f_max − tol and f_max > 0) |
| `robot` | `ekf_node.cpp` | clamp **by norm** to the model's `f_max` (falls back to 0.50 N with a WARN for models without it); edge-triggered `[ekf] force estimate saturated` WARN / `left saturation` INFO; `[lifecycle]` tags |
| `planner` | `planner_node.cpp` | `generateTrajectory` and `replanDeployment` reject a railed or out-of-domain force **before** it reaches the FTL cache or IK: `success=false`, `value=0`, message `force estimate outside trained domain (…)`, reason `FORCE_OUT_OF_DOMAIN`. When IK does not converge the message now says so (`Path generated but IK did not converge: error=… mm > 0.50 mm`). `success=true` and `value=error` are kept: the manager's 3 mm gate remains the authority on acceptance. `f_max` added to the lifecycle line |
| `manager` | `master_node.{hpp,cpp}`, `master_qt_gui.{hpp,cpp}` | `plannerStatusUpdated` carries a message; `emitPlannerStatus()` is the single emit point, with the message guarded by a mutex; a threshold rejection sets `m_planner_success = false`; new GUI **Message** column (red when unsuccessful, tooltip shows the full text) |
| `mpc` | `ctr_mpc/include/joint_box.hpp`, `mpc_node.cpp`, `mpc_params.yaml` | absolute joint box; `q0` → `[-0.1558, -0.0719, 0, 0]` in both the yaml and the code default (the old -0.156 / -0.072 seed sits below `q_min + m_margin_q`); `models_dir` declared before the set-parameters callback. That callback rejected it, so the node aborted at startup (observed in the container smoke run); logging wired in; dead block removed |
| `emtracker` | `EMtracker_node.cpp` | `Taskspace.h` = tool orientation quaternion (scalar-first). It used to be all zeros, so the EKF orientation residual was identically 0. Logging wired in; the 40 Hz pose line moved to DEBUG `[pose]` |
| `igtlink_bridge` | `igtl_bridge_node.cpp` | logging wired in; `[lifecycle]`/`[comm]` tags; service responses log success, value and message |
| `robot` | `robot_node.cpp` | CiA-402 enable fault: the `[procedure]` tag, plus an INFO line when it clears |
| docs | root and package READMEs | log snippet exports `CTR_RUN_ID`, `LOGDIR`, `CTR_LOG_DIR` consistently; new symptom row; tag tables |

## 6 Validation

All of this ran in the `ctr-jazzy-build` container. No hardware.

- `docker/check.sh`: 12 packages finished, 0 failed.
- `colcon test` (ctr_kinematics_pinn, mpc, ctr_common, manager, planner, robot,
  ctr_robot_driver, emtracker): 80 tests, 0 failures.
  - `PosCtrl.ForceDomainMatchesTrainedFmax`: `f_max` = 0.5; the field estimate
    `[-0.35, 0.50, 0.19]` is a violation; the surface value `[0, 0.5, 0]` is a violation;
    `[0, 0.499, 0]` is not.
  - `PosCtrl.FieldTargetBelowAxisConvergesWithoutForceBias`: with wf = 0, the field target
    `[-0.0036, -0.0261, 0.1252]` and its y-mirror both converge to ≤ 0.5 mm from the home
    seed with the manager's alignment angle.
  - `MpcJointBox.AbsoluteBoxContainsRobotHome`: the absolute box corners are correct and the
    yaml seed clears the margin. The first run failed because of a 1-ulp FP comparison: the
    home β2 lies exactly on `q_min + margin`. The test now allows 1e-12.
- Planner smoke (`ros2 run planner plan` plus synthetic `joint_space/feedback` and
  `task_space/force_estimate`), same field target:
  - `f = [-0.35, 0.5, 0.19]`: `success=False`, `force estimate outside trained domain
    (|f|=0.6392 N >= f_max=0.5000 N)`, `reason=FORCE_OUT_OF_DOMAIN`, 0.33 ms, no `[ik]` line.
  - `f = 0`: `success=True`, `[ik] … error=0.4826 mm iterations=54 termination=Converged`.
    In the field the same target with a railed force gave 28 mm after 750 iterations.
- EKF smoke (`ros2 run robot ekf_node`, a tracker position 3 cm off the PINN prediction): the
  published `magnitude` is 0.5 (it was up to 0.87 before). A single `[ekf] force estimate
  saturated: |f|=9.411 N > f_max=0.500 N` WARN appeared, edge-triggered.
- MPC smoke: the node starts; `q_min: [-0.1560, -0.0720, -6.2832, -4.7124]`, `q_max:
  [-0.0640, -0.0340, 6.2832, 4.7124]`.
- Logging: per-run package files are created under `$CTR_LOG_DIR/$CTR_RUN_ID/{planner,mpc,igtlink_bridge}/`;
  the first line is `Package log: …`.
- Not exercised: the manager GUI, because there is no display in the container. It compiles,
  and the static check shows one `emit` (inside `emitPlannerStatus`) and one `connect`.
  emtracker needs the Aurora serial device; its changes are verified by compilation only.

## 7 Hardware-dependent / unverified

- **Where the +y bias comes from.** It happens with the robot at home and no contact. Possible
  causes: EM↔PINN tip-frame offset, registration error, or an over-confident R. After this
  change a biased EKF no longer produces bad plans; it produces an explicit rejection. The
  bias itself is still there and needs tuning on the robot.
- **emtracker orientation (F5).** The EKF orientation residual is now non-zero for the first
  time. If the tracker tool frame is not the PINN tip frame, the orientation rows inject a
  new bias. Check `orientation_mag` on `/EKF/residual_error`. If it stays large, revert the
  four `msg_base.h` lines.
- **Ball versus box force domain.** See §4.
- **Saturation used as a proxy.** The planner treats ‖f‖ ≥ f_max − 1e-6 as railed. A true load
  of exactly 0.5 N is indistinguishable from a railed one and is rejected too; that is
  intended, because it is the edge of the training domain.

## 8 Other findings (no code change)

- `[traj] … outside the current limits snapshot` warnings are expected. Dynamic coupled limits
  come from the live β, not the commanded β, and the drive clamps by ≤ 0.5 mm and converges
  on the next step.
- `robot_node` "reached status changed" toggles at about 10 ms during motion (322 of 531
  lines). The flicker is real; the log is edge-triggered.
- The `data_root` legacy-climb WARN on every node is benign; set `CTR_DATA_ROOT` to silence it.
- `Planner.hpp:389-391` says the inner-tube drive range is ±2π, but the robot's static α1
  limit is ±3π. The comment is stale; the ±2π box is still inside the trained envelope.
- `ctr_pinn_inference.hpp` keeps an unused `tau_idx` map.
- `posCTRL` is single-start, 750 iterations. A failed solve costs about 0.95 s per request,
  and the manager re-requests, so a persistent failure keeps the planner busy.
- `sendConfigCommand` in the manager gives no GUI feedback while its services are not ready.
- CiA-402 enable fault on joints 0/2 at 17:43:53 during `findLinearHome`, then normal homing at
  17:45. The clearing edge is now logged.

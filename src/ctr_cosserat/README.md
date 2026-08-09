# ctr_cosserat

ROS-free Cosserat-rod kinematics model of the three-tube concentric tube robot.
Boost.odeint integration over Blaze types, six selectable boundary-value-problem
root-finders, and NLopt-based constrained inverse kinematics.

Static library, no `rclcpp` dependency — it is a plain numerical library that happens
to be packaged with `ament_cmake`. Formerly `robot/ctr_library`.

This is the physics-based model. The PINN in `ctr_kinematics_pinn` is the fast
learned surrogate used in the live control loop; `ctr_cosserat` is the reference
implementation it is trained and checked against.

## Using it from another package

```cmake
find_package(ctr_cosserat REQUIRED)
target_link_libraries(cosserat_fk ctr_cosserat::ctr_cosserat)
ament_target_dependencies(cosserat_fk OpenMP Boost BLAS LAPACK Blaze TBB NLopt)
```

```xml
<depend>ctr_cosserat</depend>
```

Its only consumer today is `robot`'s `cosserat_fk` executable — which is built but
not launched, see [Status notes](#status-notes).

## API

### `CTR` — `include/ctr_cosserat/CTR.hpp`

```cpp
CTR(const std::array<std::shared_ptr<Tube>, 3>& Tb,
    blaze::StaticVector<double, 6>& q,
    double Tol,
    mathOp::rootFindingMethod method,
    double stageThickness = 5.0e-3);
```

The default constructor is deleted; copy and move are both available. `q` is the
6-element joint vector in **physics order** — see `ctr_common::joint_conventions`.

| Group | Members |
|---|---|
| Solve | `ODESolver`, `reset`, `jac_BVP` (5×5 BVP Jacobian), `jacobian` (3×6, w.r.t. actuation) |
| Actuation | `actuate_CTR`, `setConfiguration`, `getConfiguration`, `alignCurvatures` |
| Inverse kinematics | `posCTRL(initGuess, target, Tol) → bool`, `constrainedPosCTRL(initGuess, tgt_clx, tgt_ee, posTol) → tuple<double,double,bool>` (calyx avoidance, NLopt) |
| Query | `getTipPos`, `getDistalEnds`, `getTubeShapes`, `getShape`, `getMinMaxTubeTorsions`, `computeAnatomicalJacobians(initGuess, s)` |
| External load | `setDistalForce`, `setDistalMoment` |
| Config | `setBVPMethod`, `setStageThickness`, `getStageThickness` |

### BVP root-finders

Selected via `mathOp::rootFindingMethod` and switchable at runtime with
`setBVPMethod`. Each is also directly callable and returns `bool` (converged):

`NEWTON_RAPHSON`, `LEVENBERG_MARQUARDT`, `POWELL_DOG_LEG`,
`MODIFIED_NEWTON_RAPHSON`, `BROYDEN`, `BROYDEN_II`.

`Modified_Newton_Raphson` is the globally convergent variant.

### Supporting headers

| Header | Contents |
|---|---|
| `Tube.hpp` | `Tube` — per-tube geometry and material: OD, ID, Young's modulus `E`, shear modulus `G`, straight length `ls`, curved length `lc`, precurvature `u_ast`, stiffness `K`. Getters/setters for each. |
| `Segment.hpp` | Transition points between tube segments. |
| `ODESystem.hpp` | `state_type = blaze::StaticVector<double,15>`; the functor implementing the three-tube CTR state equations. |
| `Observer.hpp` | Captures states and arc-lengths from Boost.odeint at each integration step. |
| `mathOperations.hpp` | Namespace `mathOp`: the `rootFindingMethod` enum, `deg2Rad`, `wrapToPi`, `wrapTo2Pi`, `congruentAngle`, `orthogonal`. |
| `boostBlazeAlgebra.hpp` | `boost::numeric::odeint::custom_algebra` specialization that lets Boost.odeint integrate Blaze vectors directly. |

## Build quirks

`NLopt`, `BLAS`, `LAPACK`, and `gfortran` are linked **`PRIVATE`** into a static
library. CMake surfaces those in the generated export as `$<LINK_ONLY:NLopt::nlopt>`
and friends, so `ament_export_dependencies(NLopt BLAS LAPACK)` is required and
**consumers must be able to resolve those packages** when the export file loads. A
consumer that calls `find_package(ctr_cosserat)` without those available will fail to
resolve `NLopt::nlopt`. The CMakeLists carries a comment saying exactly this.

## Tests

None. The package has no `test/` directory — the model needs a full BVP solve to
exercise meaningfully, which is slow and sensitive to solver tolerances. The
hardware-free suites in this workspace cover the pure-geometry pieces elsewhere
(`planner`'s `DeploymentSchedule`, `ctr_common`'s conventions).

## Status notes

The package itself is current — it is the successor to `robot/ctr_library`, not a
leftover copy. However, its sole consumer `robot`'s `cosserat_fk` node **is built and
installed but never launched**: the `ld.add_action(cosserat_fk_node)` line is
commented out in `robot/launch/robot.py`, superseded by the PINN-based `pinn_fk`.
Treat `ctr_cosserat` as a reference/offline model rather than part of the live
control path.

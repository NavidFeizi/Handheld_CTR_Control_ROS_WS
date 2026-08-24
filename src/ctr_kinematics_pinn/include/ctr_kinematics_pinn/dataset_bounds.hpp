#ifndef CTR_KINEMATICS_PINN__DATASET_BOUNDS_HPP_
#define CTR_KINEMATICS_PINN__DATASET_BOUNDS_HPP_

// Joint-range conventions of the PINN training datasets.
//
// Deliberately free of LibTorch, blaze and ROS so the algebra below can be unit
// tested without pulling the inference stack in (see planner/test).
//
// The 4-DoF datasets ([β₁, β₂, α₁, α₂]) do NOT store β₁ in the same frame as β₂:
//
//   parameters.json  beta2_range = [-0.072, -0.034]   absolute stage travel [m]
//                    beta1_range = [-0.084, -0.030]   offset RELATIVE to β₂ [m]
//
// The relative window is the tube-coupling window: its upper end is the stage
// thickness (30 mm) and its lower end is the inner/middle active-length
// difference (216 mm − 132 mm = 84 mm). The same rule is what the hardware
// enforces in robot_node's dynamic position limits:
//
//   β₁ ∈ [max(β₁_floor, β₂ − 0.084), min(β₁_ceiling, β₂ − 0.030)]
//
// so β₁'s ABSOLUTE range is obtained by adding β₂'s matching endpoints:
//
//   β₁_abs = [β₂_min + β₁_rel_min, β₂_max + β₁_rel_max] = [-0.156, -0.064]
//
// which is exactly robot_node's k_home_pos[1] .. k_pos_preEngage[1].
//
// Anything that needs a box bound in the robot's own joint frame (the planner's
// OMPL state space, its state-validity checker and samplers, the resolved-rate
// IK's joint-limit-avoidance term) needs the ABSOLUTE form. Anything that
// re-applies the coupling itself by shifting the window by the live β₂ -- i.e.
// ctr_common::clampJointPositions, used by pinn_fk -- needs the RELATIVE form.
// Mixing the two collapses the reachable set and every plan fails its start
// state; keep the two accessors distinct.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace ctr_kinematics_pinn
{

/// β₁'s absolute [min, max] travel, from its relative window and β₂'s absolute range.
inline std::array<double, 2UL> absoluteBeta1Range(const std::array<double, 2UL> &beta1_relative,
                                                  const std::array<double, 2UL> &beta2_absolute)
{
  return {beta2_absolute[0UL] + beta1_relative[0UL],
          beta2_absolute[1UL] + beta1_relative[1UL]};
}

// ---------------------------------------------------------------------------
// The feasible joint set, in ONE place.
//
// Before this existed, three components each carried their own version of "is
// this configuration legal", and two of them were wrong in opposite directions:
//
//   PINNs::posCTRL          capped β₂ at −stageThickness (−0.030) and forgot that
//                           β₂ itself is bounded by beta2_range[1] (−0.034). It
//                           therefore returned converged solutions the planner
//                           rejects -- setGoalState() throws and no plan is made.
//   CTR_StateValidityChecker enforced the β₁ ≤ β₂ − clearance half of the β₁
//                           window but not the β₁ ≥ β₂ − 0.084 half, so it
//                           accepted states outside the PINN's training data.
//
// Both gaps disappear once the set is written the way the dataset already
// defines it. For the 4-DoF layout the whole thing is four constraints:
//
//   β₂        ∈ beta2_range                    [−0.072, −0.034]
//   β₁ − β₂   ∈ beta1_range                    [−0.084, −0.030]
//   α₁        ∈ alpha1_range                   [−π, π]
//   α₂ − α₁   ∈ [−π, π]
//
// The β₁ relative window is doing double duty, which is why it is easy to lose
// half of it: its upper edge −0.030 IS the stage thickness (β₁ ≤ β₂ − clr), and
// its lower edge −0.084 IS the tube-protrusion constraint β₁ + L₁ ≥ β₂ + L₂
// (L₂ − L₁ = 0.132 − 0.216 = −0.084 for the shipped tubes). Enforce the window
// and you get both for free; enforce either bound alone and you silently drop
// the other. The remaining geometry terms are not extra constraints: L₃ − L₁ =
// −0.156 and L₃ − L₂ = −0.072 are exactly the absolute β₁/β₂ floors.
// ---------------------------------------------------------------------------

/// The 4-DoF feasible set, verbatim from a model's dataset_params.
struct JointLimits4
{
  std::array<double, 2UL> beta2_absolute{};  ///< dataset beta2_range
  std::array<double, 2UL> beta1_relative{};  ///< dataset beta1_range, RELATIVE to β₂
  std::array<double, 2UL> alpha1_absolute{}; ///< dataset alpha1_range
  double alpha2_window = M_PI;               ///< |α₂ − α₁| ≤ this (PINN training constraint)
};

/// β₁'s live window given β₂: the relative window, intersected with β₁'s own
/// absolute travel. Callers that step β₁ should clamp into this.
inline std::array<double, 2UL> beta1Window(const double beta2, const JointLimits4 &lim)
{
  const auto abs1 = absoluteBeta1Range(lim.beta1_relative, lim.beta2_absolute);
  return {std::max(beta2 + lim.beta1_relative[0UL], abs1[0UL]),
          std::min(beta2 + lim.beta1_relative[1UL], abs1[1UL])};
}

/// β₂'s live window given β₁. Inverting β₁ − β₂ ∈ [r₀, r₁] gives
/// β₂ ∈ [β₁ − r₁, β₁ − r₀], intersected with β₂'s absolute range. The upper end
/// is where posCTRL used to walk out of the planner's accepted set.
inline std::array<double, 2UL> beta2Window(const double beta1, const JointLimits4 &lim)
{
  return {std::max(beta1 - lim.beta1_relative[1UL], lim.beta2_absolute[0UL]),
          std::min(beta1 - lim.beta1_relative[0UL], lim.beta2_absolute[1UL])};
}

/// Is q = [β₁, β₂, α₁, α₂] (PHYSICS order) feasible?
///
/// `tol` exists because the poses the robot actually homes to sit exactly ON the
/// bounds: k_pos_preEngage = {β₁ −0.0640, β₂ −0.0340} is precisely the upper
/// corner and k_home_pos precisely the lower one, so measured encoder values
/// straddle the boundary by floating-point noise. Pass a small tolerance when
/// validating a measured pose; pass 0 when validating a computed one.
inline bool isFeasible4(const std::array<double, 4UL> &q, const JointLimits4 &lim, const double tol = 0.00)
{
  const double beta1 = q[0UL], beta2 = q[1UL], alpha1 = q[2UL], alpha2 = q[3UL];

  if (beta2 < lim.beta2_absolute[0UL] - tol || beta2 > lim.beta2_absolute[1UL] + tol)
    return false;

  const double rel = beta1 - beta2;  // both clearance and protrusion live here
  if (rel < lim.beta1_relative[0UL] - tol || rel > lim.beta1_relative[1UL] + tol)
    return false;

  const auto abs1 = absoluteBeta1Range(lim.beta1_relative, lim.beta2_absolute);
  if (beta1 < abs1[0UL] - tol || beta1 > abs1[1UL] + tol)
    return false;

  if (alpha1 < lim.alpha1_absolute[0UL] - tol || alpha1 > lim.alpha1_absolute[1UL] + tol)
    return false;

  return std::fabs(alpha2 - alpha1) <= lim.alpha2_window + tol;
}

}  // namespace ctr_kinematics_pinn

#endif  // CTR_KINEMATICS_PINN__DATASET_BOUNDS_HPP_

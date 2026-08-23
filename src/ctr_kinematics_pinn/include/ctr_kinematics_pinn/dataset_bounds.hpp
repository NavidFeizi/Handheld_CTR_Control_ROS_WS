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

#include <array>
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

}  // namespace ctr_kinematics_pinn

#endif  // CTR_KINEMATICS_PINN__DATASET_BOUNDS_HPP_

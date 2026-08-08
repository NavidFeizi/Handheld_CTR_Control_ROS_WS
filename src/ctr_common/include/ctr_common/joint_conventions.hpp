#ifndef CTR_COMMON__JOINT_CONVENTIONS_HPP_
#define CTR_COMMON__JOINT_CONVENTIONS_HPP_

// Joint-order conventions for the handheld CTR.
//
// Wire order — every joint_space/* topic (interfaces/msg/Jointspace, position[4]):
//     [α1, β1, α2, β2]   rotation (rad) / translation (m) per tube pair
// Physics order — PINN / Cosserat / planner internals:
//     4-element: [β1, β2, α1, α2]
//     6-element: [β1, β2, β3, α1, α2, α3]   (β3/α3 = unactuated outer tube, always 0)

#include <algorithm>
#include <array>
#include <cstddef>

#include <blaze/Math.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ctr_common
{

/// Wire [α1, β1, α2, β2] → physics [β1, β2, α1, α2]
inline blaze::StaticVector<double, 4UL> wireToPhysics4(const std::array<double, 4UL> &p)
{
  return {p[1], p[3], p[0], p[2]};
}

/// Wire [α1, β1, α2, β2] → physics [β1, β2, β3, α1, α2, α3] with β3 = α3 = 0
inline blaze::StaticVector<double, 6UL> wireToPhysics6(const std::array<double, 4UL> &p)
{
  return {p[1], p[3], 0.0, p[0], p[2], 0.0};
}

/// Physics [β1, β2, α1, α2] → wire [α1, β1, α2, β2]
inline std::array<double, 4UL> physicsToWire(const blaze::StaticVector<double, 4UL> &q)
{
  return {q[2], q[0], q[3], q[1]};
}

/// Physics [β1, β2, β3, α1, α2, α3] → wire [α1, β1, α2, β2]
inline std::array<double, 4UL> physicsToWire(const blaze::StaticVector<double, 6UL> &q)
{
  return {q[3], q[0], q[4], q[1]};
}

/// Clamp joint positions in physics order, shifting each tube's translation
/// limits by the next-outer tube's translation (tube coupling rule).
template <size_t N>
void clampJointPositions(blaze::StaticVector<double, N> &q,
                         const blaze::StaticVector<double, N> &q_min,
                         const blaze::StaticVector<double, N> &q_max,
                         const rclcpp::Logger &logger)
{
  blaze::StaticVector<double, N> q_max_eff = q_max;
  blaze::StaticVector<double, N> q_min_eff = q_min;

  if constexpr (N == 6)
  {
    q_max_eff[1] += q[2];
    q_min_eff[1] += q[2];
    q_max_eff[0] += q[1];
    q_min_eff[0] += q[1];
    q_max_eff[4] += q[5];
    q_min_eff[4] += q[5];
    q_max_eff[3] += q[4];
    q_min_eff[3] += q[4];
  }
  else if constexpr (N == 4)
  {
    q_max_eff[0] += q[1];
    q_min_eff[0] += q[1];
    q_max_eff[2] += q[3];
    q_min_eff[2] += q[3];
  }
  else
  {
    RCLCPP_WARN(logger, "clampJointPositions: No coupling handling for N=%zu", N);
  }

  for (size_t i = 0; i < q.size(); ++i)
  {
    q[i] = std::clamp(q[i], q_min_eff[i], q_max_eff[i]);
  }
}

}  // namespace ctr_common

#endif  // CTR_COMMON__JOINT_CONVENTIONS_HPP_

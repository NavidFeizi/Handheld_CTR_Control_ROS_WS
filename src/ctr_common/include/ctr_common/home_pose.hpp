#ifndef CTR_COMMON__HOME_POSE_HPP_
#define CTR_COMMON__HOME_POSE_HPP_

// The two mechanically distinguished CTR poses, in ONE place.
//
// Deliberately free of rclcpp, blaze and LibTorch: `manager` needs these to
// build a retract-to-home leg, `robot` needs them for its static joint limits
// and goHome(), and the unit tests need them without an inference stack.
//
// WIRE ORDER throughout this header -- [alpha1, beta1, alpha2, beta2] -- the
// order of interfaces/msg/Jointspace::position and of everything inside
// `robot`/`ctr_robot_driver`. Convert with ctr_common::wireToPhysics4() before
// handing a pose to the PINN, the planner or the Cosserat model.
//
// Relationship to ctr_kinematics_pinn/dataset_bounds.hpp: these two poses are
// exactly the opposite corners of the feasible joint set that header defines.
//
//   kHomePose      beta1 -0.156, beta2 -0.072  -> both at their MINIMA,
//                                                 beta1 - beta2 = -0.084, the
//                                                 floor of the relative window
//   kPreEngagePose beta1 -0.0640, beta2 -0.0340 -> both at their MAXIMA,
//                                                 beta1 - beta2 = -0.030, the
//                                                 ceiling of that window
//
// Both therefore sit ON the bounds rather than inside them, which is why
// isFeasible4() takes a tolerance and why goHome() commands
// kHomePose + kHomePoseMargin instead of kHomePose itself: encoder noise alone
// puts a measured carriage a few microns outside the box.
//
// A corollary that matters whenever anything moves toward home: beta1's
// admissible window is anchored to the LIVE beta2
// (beta1 in [beta2 - 0.084, beta2 - 0.030]), so the two carriages must travel
// together. Commanding kHomePose in one step from a deployed pose asks beta1
// to go below beta2 - 0.084 while beta2 is still forward; the drive clips that
// at its POSITION_LIMIT (0x607D) and stops short with no feedback. Interpolate.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ctr_common
{

/// Active tube lengths [m], outermost tube last.
inline constexpr double kInnerActiveLength = 0.216;
inline constexpr double kMiddleActiveLength = 0.132;
inline constexpr double kOuterActiveLength = 0.060;

/// Carriage clearance window [m]: the stage thickness, and the inner/middle
/// active-length difference that keeps the inner tube protruding.
inline constexpr double kLinearStageMinClearance = 0.030;
inline constexpr double kLinearStageMaxClearance = kInnerActiveLength - kMiddleActiveLength;  // 0.084

/// Fully retracted mechanical home, wire order [a1, b1, a2, b2].
inline constexpr std::array<double, 4UL> kHomePose = {
    0.0, kOuterActiveLength - kInnerActiveLength,   // beta1 = -0.156
    0.0, kOuterActiveLength - kMiddleActiveLength}; // beta2 = -0.072

/// Offset added to kHomePose before commanding it, so a pose that sits exactly
/// on the lower bound is not rejected/clipped by encoder noise.
inline constexpr std::array<double, 4UL> kHomePoseMargin = {0.0, 0.0002, 0.0, 0.0001};

/// Fully advanced pose the collets are engaged at, wire order.
inline constexpr std::array<double, 4UL> kPreEngagePose = {0.0, -0.0640, 0.0, -0.0340};

/// kHomePose + kHomePoseMargin -- the pose that is actually commanded.
inline constexpr std::array<double, 4UL> homePoseCommanded()
{
  return {kHomePose[0UL] + kHomePoseMargin[0UL], kHomePose[1UL] + kHomePoseMargin[1UL],
          kHomePose[2UL] + kHomePoseMargin[2UL], kHomePose[3UL] + kHomePoseMargin[3UL]};
}

/// Extra clearance a sequenced move leaves against the coupling window, so the
/// joint that moves second is never asked to sit exactly on its bound.
inline constexpr double kHomeLegClearanceGuard = 1.0e-3;

/// Straight-line joint-space waypoints from `from` to `to` (both wire order),
/// subdivided so no step moves a prismatic joint by more than `prismatic_step`
/// or a revolute joint by more than `revolute_step`. `from` is NOT included;
/// `to` is always the last element. Empty only when `from` and `to` are
/// identical -- a difference smaller than one step still yields one waypoint,
/// because silently not commanding the endpoint is never the right answer.
///
/// Straight-line interpolation keeps every WAYPOINT feasible, and that is not
/// an accident: the feasible joint set is the intersection of half-spaces
/// (beta2 in a box, beta1 - beta2 in a box, beta1 in a box, and likewise for
/// the alpha pair -- see ctr_kinematics_pinn/dataset_bounds.hpp), hence convex.
/// Any segment between two feasible poses therefore stays feasible for its
/// whole length.
///
/// That is NOT sufficient for a move that ends on a corner of the set, and the
/// difference matters. The drive evaluates beta1's POSITION_LIMIT against the
/// LIVE beta2, which still holds the PREVIOUS waypoint's value when a new
/// target is issued. Moving both carriages at once therefore tightens beta1's
/// bound by up to one step of beta2 travel, which is far more than the ~0.1 mm
/// of margin the home pose has. Callers that need to reach a corner must
/// SEQUENCE the joints -- move one while the other is stationary at a value it
/// has already reached, which removes the lag entirely. manager_csv::buildHomeLeg
/// is the worked example.
inline std::vector<std::array<double, 4UL>> interpolatePose(
    const std::array<double, 4UL> &from, const std::array<double, 4UL> &to,
    const double prismatic_step, const double revolute_step)
{
  // Wire order: indices 1 and 3 are prismatic, 0 and 2 revolute.
  const double p_step = (prismatic_step > 0.0) ? prismatic_step : 1.0e-3;
  const double r_step = (revolute_step > 0.0) ? revolute_step : 0.1;

  std::size_t n = 0UL;
  for (std::size_t i = 0UL; i < 4UL; ++i)
  {
    const double limit = (i == 1UL || i == 3UL) ? p_step : r_step;
    const double needed = std::ceil(std::fabs(to[i] - from[i]) / limit);
    if (needed > static_cast<double>(n))
    {
      n = static_cast<std::size_t>(needed);
    }
  }

  std::vector<std::array<double, 4UL>> out;
  if (n == 0UL)
  {
    return out;  // already there, to within one step on every joint
  }

  out.reserve(n);
  for (std::size_t k = 1UL; k <= n; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    std::array<double, 4UL> q{};
    for (std::size_t i = 0UL; i < 4UL; ++i)
    {
      q[i] = from[i] + t * (to[i] - from[i]);
    }
    out.push_back(q);
  }
  // Land exactly on the endpoint rather than on an accumulated fraction of it.
  out.back() = to;
  return out;
}

}  // namespace ctr_common

#endif  // CTR_COMMON__HOME_POSE_HPP_

#ifndef MANAGER__BEARING_GATE_HPP_
#define MANAGER__BEARING_GATE_HPP_

// Pure pre-rotation gate maths, extracted from MasterNode::control_loop so it
// is testable without ROS/Qt.
//
// The manager rotates both tubes toward the target's bearing before asking the
// planner for a path (empirically tip azimuth = alpha1 - pi/2, so the bearing
// alpha is atan2(y, x) + pi/2). Two things here used to be wrong and made
// targets near the +y axis unplannable:
//   1. the gate compared RAW angle differences, so a tube 2pi - eps away in
//      motor angle -- but perfectly aligned in bearing -- read as "off by 350
//      degrees" and the gate never opened;
//   2. the commanded angle was the bearing wrapped into [-pi, pi] regardless of
//      where the tube currently was, commanding up to a full turn of pointless
//      travel and parking the start state exactly on the old alpha1 = +-pi
//      branch cut.
// The gate now measures bearing misalignment (wrapped: it is a direction, not
// a travel), while the COMMAND is travel-true: the nearest 2pi-representative
// of the bearing from the current motor angle, folded into the trained alpha
// box when the drive cannot cross its travel limit.

#include <array>
#include <cmath>

#include "ctr_kinematics_pinn/dataset_bounds.hpp"

namespace manager_gate
{

struct PreRotation
{
  double target_theta = 0.0;  ///< target bearing, principal branch [-pi, pi)
  double gate_diff_1 = 0.0;   ///< |bearing - alpha1| as a direction (wrapped, <= pi)
  double gate_diff_2 = 0.0;   ///< |bearing - alpha2| as a direction (wrapped, <= pi)
  double cmd_alpha = 0.0;     ///< shared motor-angle command for BOTH tubes
};

/// Evaluate the pre-rotation gate for target (xd_x, xd_y) with current motor
/// angles alpha1/alpha2 (wire indices 0 and 2). `lim` supplies the trained
/// alpha boxes the command must stay inside.
///
/// Both tubes are steered to ONE shared motor angle (the pre-rotation pose has
/// alpha1 = alpha2, i.e. zero relative twist): representatives 2pi apart would
/// put |alpha1 - alpha2| = 2pi, a pose the planner's start check rightly
/// rejects. Among the bearing's representatives that fit inside the tighter
/// alpha2 box (which the alpha1 box contains when the relative twist is zero),
/// the one minimising the pair's worst-case travel wins.
inline PreRotation computePreRotation(const double xd_x, const double xd_y,
                                      const double alpha1, const double alpha2,
                                      const ctr_kinematics_pinn::JointLimits4 &lim)
{
  PreRotation out;
  out.target_theta = ctr_kinematics_pinn::wrapToPi(std::atan2(xd_y, xd_x) + M_PI / 2.0);

  out.gate_diff_1 = std::fabs(ctr_kinematics_pinn::wrapToPi(out.target_theta - alpha1));
  out.gate_diff_2 = std::fabs(ctr_kinematics_pinn::wrapToPi(out.target_theta - alpha2));

  double best = out.target_theta;  // principal-branch representative, always in-box
  double bestCost = std::max(std::fabs(best - alpha1), std::fabs(best - alpha2));
  for (const double k : {-1.0, 1.0})
  {
    const double cand = out.target_theta + k * 2.0 * M_PI;
    if (cand < lim.alpha2_absolute[0UL] || cand > lim.alpha2_absolute[1UL])
      continue;
    const double cost = std::max(std::fabs(cand - alpha1), std::fabs(cand - alpha2));
    if (cost < bestCost)
    {
      bestCost = cost;
      best = cand;
    }
  }
  out.cmd_alpha = best;

  return out;
}

}  // namespace manager_gate

#endif  // MANAGER__BEARING_GATE_HPP_

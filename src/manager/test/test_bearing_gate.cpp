// Pins the pre-rotation gate maths (manager/bearing_gate.hpp).
//
// The two regressions this guards against (both made +y targets unplannable):
//   1. gate diffs computed as RAW |bearing - alpha|, so a tube that was
//      bearing-aligned but wound up by ~2pi read as "off by 350 degrees";
//   2. the command was the bearing wrapped into [-pi, pi] regardless of the
//      current motor angle, commanding up to a full pointless turn and parking
//      the start state on the old alpha1 = +-pi branch cut.

#include <gtest/gtest.h>

#include <cmath>

#include "manager/bearing_gate.hpp"

namespace
{

constexpr double kTol = 1e-9;
const ctr_kinematics_pinn::JointLimits4 kLim{};  // dataset defaults: alpha2 +-1.5pi, rel +-pi

using manager_gate::computePreRotation;

TEST(BearingGate, MinusYTargetMapsToZeroBearing)
{
  // Empirically tip azimuth = alpha1 - pi/2, so a target on -y (azimuth -pi/2)
  // wants alpha = 0.
  const auto pr = computePreRotation(0.0, -0.1, 0.0, 0.0, kLim);
  EXPECT_NEAR(pr.target_theta, 0.0, kTol);
  EXPECT_NEAR(pr.cmd_alpha, 0.0, kTol);
  EXPECT_NEAR(pr.gate_diff_1, 0.0, kTol);
  EXPECT_NEAR(pr.gate_diff_2, 0.0, kTol);
}

TEST(BearingGate, GateMeasuresBearingNotTravel)
{
  // Tube wound to 1.4pi; target bearing 1.4pi - 2pi = -0.6pi (same direction).
  const double alpha = 1.4 * M_PI;
  const double bearing = alpha - 2.0 * M_PI;
  // Choose (x, y) whose azimuth + pi/2 equals `bearing`.
  const double az = bearing - M_PI / 2.0;
  const auto pr = computePreRotation(std::cos(az), std::sin(az), alpha, alpha, kLim);
  // Bearing-aligned: the gate must read ~0 even though the raw difference is 2pi.
  EXPECT_NEAR(pr.gate_diff_1, 0.0, 1e-6);
  EXPECT_NEAR(pr.gate_diff_2, 0.0, 1e-6);
}

TEST(BearingGate, CommandIsTravelTrueRepresentative)
{
  // Tubes at 1.4pi, bearing just past the +-pi seam (principal value -pi + 0.1).
  const double alpha = 1.4 * M_PI;
  const double bearing = -M_PI + 0.1;
  const double az = bearing - M_PI / 2.0;
  const auto pr = computePreRotation(std::cos(az), std::sin(az), alpha, alpha, kLim);
  // Nearest representative is pi + 0.1 (0.3pi - 0.1 of travel), not -pi + 0.1
  // (2.4pi - 0.1 of travel). The old code could only ever command the latter.
  EXPECT_NEAR(pr.cmd_alpha, M_PI + 0.1, kTol);
  EXPECT_LE(pr.cmd_alpha, kLim.alpha2_absolute[1]);
}

TEST(BearingGate, CommandStaysInsideTrainedBox)
{
  // Representative that would minimise travel (2pi - 0.1) exceeds the +-1.5pi
  // box, so the in-box principal representative must win despite longer travel.
  const double alpha = 1.45 * M_PI;
  const double bearing = -0.1;  // representatives: -0.1 (in box), 2pi - 0.1 (out)
  const double az = bearing - M_PI / 2.0;
  const auto pr = computePreRotation(std::cos(az), std::sin(az), alpha, alpha, kLim);
  EXPECT_NEAR(pr.cmd_alpha, -0.1, kTol);
  EXPECT_GE(pr.cmd_alpha, kLim.alpha2_absolute[0]);
  EXPECT_LE(pr.cmd_alpha, kLim.alpha2_absolute[1]);
}

TEST(BearingGate, SharedCommandKeepsZeroRelativeTwist)
{
  // Whatever branch is chosen, BOTH tubes get one command: representatives
  // 2pi apart would violate |alpha1 - alpha2| <= pi at the pre-rotated pose.
  const auto pr = computePreRotation(0.05, 0.02, 1.2 * M_PI, -0.3, kLim);
  EXPECT_GE(pr.cmd_alpha, kLim.alpha2_absolute[0]);
  EXPECT_LE(pr.cmd_alpha, kLim.alpha2_absolute[1]);
  // A single shared command trivially satisfies the relative-twist constraint.
  SUCCEED();
}

TEST(BearingGate, PlusYTargetIsCommandable)
{
  // The historically failing sector: target on +y (azimuth +pi/2) wants a
  // bearing of +-pi. From home (alpha = 0) some in-box representative must be
  // commanded, and it must be pi of travel, not 2pi - eps.
  const auto pr = computePreRotation(0.0, 0.1, 0.0, 0.0, kLim);
  EXPECT_NEAR(std::fabs(pr.cmd_alpha), M_PI, kTol);
  EXPECT_NEAR(pr.gate_diff_1, M_PI, kTol);
}

}  // namespace

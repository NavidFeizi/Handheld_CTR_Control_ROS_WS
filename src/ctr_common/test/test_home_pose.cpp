#include <gtest/gtest.h>

#include "ctr_common/home_pose.hpp"

#include <cmath>

namespace
{
constexpr size_t kA1 = 0, kB1 = 1, kA2 = 2, kB2 = 3;  // wire order

// The coupling window both carriages must respect at every instant.
bool couplingOk(const std::array<double, 4UL> &q, double tol = 1.0e-12)
{
  const double rel = q[kB1] - q[kB2];
  return rel >= -ctr_common::kLinearStageMaxClearance - tol &&
         rel <= -ctr_common::kLinearStageMinClearance + tol;
}
}  // namespace

// These four numbers are the interface between three packages (robot's static
// limits, the manager's retract-to-home leg, and the PINN dataset's feasible
// box). Pin them so a tube-set change cannot move one copy and not the others.
TEST(HomePose, MatchesTheHardwareGeometry)
{
  EXPECT_DOUBLE_EQ(ctr_common::kHomePose[kB1], 0.060 - 0.216);   // -0.156
  EXPECT_DOUBLE_EQ(ctr_common::kHomePose[kB2], 0.060 - 0.132);   // -0.072
  EXPECT_DOUBLE_EQ(ctr_common::kHomePose[kA1], 0.0);
  EXPECT_DOUBLE_EQ(ctr_common::kHomePose[kA2], 0.0);

  EXPECT_DOUBLE_EQ(ctr_common::kPreEngagePose[kB1], -0.0640);
  EXPECT_DOUBLE_EQ(ctr_common::kPreEngagePose[kB2], -0.0340);

  EXPECT_DOUBLE_EQ(ctr_common::kLinearStageMaxClearance, 0.216 - 0.132);  // 0.084
  EXPECT_DOUBLE_EQ(ctr_common::kLinearStageMinClearance, 0.030);
}

// Home and pre-engage are the OPPOSITE CORNERS of the feasible set: each sits
// exactly on a bound, which is why a margin exists at all.
TEST(HomePose, BothPosesSitExactlyOnOppositeCorners)
{
  EXPECT_DOUBLE_EQ(ctr_common::kHomePose[kB1] - ctr_common::kHomePose[kB2],
                   -ctr_common::kLinearStageMaxClearance);
  EXPECT_NEAR(ctr_common::kPreEngagePose[kB1] - ctr_common::kPreEngagePose[kB2],
              -ctr_common::kLinearStageMinClearance, 1.0e-15);
}

// The margin must move BOTH carriages strictly inside the box, otherwise the
// commanded home pose is still on the bound and encoder noise rejects it.
TEST(HomePose, CommandedHomeIsStrictlyInsideTheBox)
{
  const auto cmd = ctr_common::homePoseCommanded();
  EXPECT_GT(cmd[kB1], ctr_common::kHomePose[kB1]);
  EXPECT_GT(cmd[kB2], ctr_common::kHomePose[kB2]);
  const double rel = cmd[kB1] - cmd[kB2];
  EXPECT_GT(rel, -ctr_common::kLinearStageMaxClearance);
  EXPECT_LT(rel, -ctr_common::kLinearStageMinClearance);
}

TEST(InterpolatePose, EmptyOnlyWhenIdentical)
{
  const std::array<double, 4UL> a = {0.0, -0.1000, 0.0, -0.0600};
  EXPECT_TRUE(ctr_common::interpolatePose(a, a, 2.0e-3, 0.10).empty());
}

// A sub-step difference still produces the endpoint. Skipping it because it is
// "close enough" would leave the robot short of the pose the caller asked for,
// with nothing in the log to say so.
TEST(InterpolatePose, SubStepMoveStillCommandsTheEndpoint)
{
  const std::array<double, 4UL> a = {0.0, -0.1000, 0.0, -0.0600};
  const std::array<double, 4UL> b = {0.0, -0.1005, 0.0, -0.0605};
  const auto legs = ctr_common::interpolatePose(a, b, 2.0e-3, 0.10);
  ASSERT_EQ(legs.size(), 1u);
  EXPECT_DOUBLE_EQ(legs[0][kB1], b[kB1]);
  EXPECT_DOUBLE_EQ(legs[0][kB2], b[kB2]);
}

TEST(InterpolatePose, LandsExactlyOnTheEndpoint)
{
  const std::array<double, 4UL> from = {0.5, -0.0900, -0.3, -0.0500};
  const auto to = ctr_common::homePoseCommanded();
  const auto legs = ctr_common::interpolatePose(from, to, 2.0e-3, 0.10);
  ASSERT_FALSE(legs.empty());
  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_DOUBLE_EQ(legs.back()[i], to[i]) << "joint " << i;
  }
}

TEST(InterpolatePose, RespectsBothStepCaps)
{
  const std::array<double, 4UL> from = {3.0, -0.0640, -2.0, -0.0340};
  const auto to = ctr_common::homePoseCommanded();
  const double p_step = 2.0e-3, r_step = 0.10;
  const auto legs = ctr_common::interpolatePose(from, to, p_step, r_step);
  ASSERT_FALSE(legs.empty());

  std::array<double, 4UL> prev = from;
  for (const auto &q : legs)
  {
    EXPECT_LE(std::fabs(q[kB1] - prev[kB1]), p_step + 1.0e-12);
    EXPECT_LE(std::fabs(q[kB2] - prev[kB2]), p_step + 1.0e-12);
    EXPECT_LE(std::fabs(q[kA1] - prev[kA1]), r_step + 1.0e-12);
    EXPECT_LE(std::fabs(q[kA2] - prev[kA2]), r_step + 1.0e-12);
    prev = q;
  }
}

// The property the whole design rests on: the feasible set is convex, so a
// straight line between two legal poses never leaves it. If this ever fails,
// the subdivided home leg would be clipped by the drives mid-move.
TEST(InterpolatePose, ConvexityKeepsEveryWaypointInsideTheCouplingWindow)
{
  const std::array<double, 4UL> corners[] = {
      {0.0, ctr_common::kHomePose[kB1], 0.0, ctr_common::kHomePose[kB2]},
      {0.0, ctr_common::kPreEngagePose[kB1], 0.0, ctr_common::kPreEngagePose[kB2]},
      {0.0, -0.1200, 0.0, -0.0600},
      {0.0, -0.0900, 0.0, -0.0500},
  };

  for (const auto &a : corners)
  {
    for (const auto &b : corners)
    {
      ASSERT_TRUE(couplingOk(a));
      ASSERT_TRUE(couplingOk(b));
      for (const auto &q : ctr_common::interpolatePose(a, b, 1.0e-3, 0.05))
      {
        EXPECT_TRUE(couplingOk(q))
            << "beta1 - beta2 = " << (q[kB1] - q[kB2]) << " left the window";
      }
    }
  }
}

// The reason manager_csv::buildHomeLeg sequences the two carriages instead of
// interpolating them together. Subdividing alone is NOT enough to reach a
// corner of the feasible set: the drive checks beta1 against the LIVE beta2,
// which still holds the previous waypoint's value, and the home pose has only
// kHomePoseMargin (0.1 mm) of slack on that bound. This test asserts the
// failure so that a future "simplification" back to a single straight line has
// to confront it.
TEST(InterpolatePose, SimultaneousMoveCannotReachTheCornerLagFree)
{
  const std::array<double, 4UL> from = {0.0, -0.1000, 0.0, -0.0500};
  const auto to = ctr_common::homePoseCommanded();

  // A single unsubdivided jump plainly violates the limit implied by the start.
  EXPECT_LT(to[kB1], from[kB2] - ctr_common::kLinearStageMaxClearance);

  // Every WAYPOINT is feasible against its own beta2 (convexity)...
  for (const auto &q : ctr_common::interpolatePose(from, to, 2.0e-3, 0.10))
  {
    EXPECT_TRUE(couplingOk(q));
  }

  // ...but at least one step violates the limit implied by the PREVIOUS
  // waypoint's beta2, which is the one the drive actually enforces.
  std::array<double, 4UL> prev = from;
  bool lagged_violation = false;
  for (const auto &q : ctr_common::interpolatePose(from, to, 2.0e-3, 0.10))
  {
    if (q[kB1] < prev[kB2] - ctr_common::kLinearStageMaxClearance)
    {
      lagged_violation = true;
    }
    prev = q;
  }
  EXPECT_TRUE(lagged_violation)
      << "if this ever stops holding, buildHomeLeg's sequencing can be simplified";
}

// The guard has to be big enough to cover a whole step of carriage travel,
// which is the worst-case lag a sequenced leg still has to tolerate at the
// handover between sub-legs.
TEST(HomePose, ClearanceGuardExceedsTheCornerMargin)
{
  EXPECT_GT(ctr_common::kHomeLegClearanceGuard, ctr_common::kHomePoseMargin[kB1]);
  EXPECT_GT(ctr_common::kHomeLegClearanceGuard, ctr_common::kHomePoseMargin[kB2]);
}

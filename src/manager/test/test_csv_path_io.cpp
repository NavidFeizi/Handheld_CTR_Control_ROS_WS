#include <gtest/gtest.h>

#include "manager/csv_path_io.hpp"

#include "ctr_common/csv_io.hpp"
#include "ctr_kinematics_pinn/dataset_bounds.hpp"

#include <sstream>

TEST(ParsePathRows, SixColumnPassThrough)
{
  const std::vector<std::vector<double>> rows = {{1, 2, 3, 4, 5, 6}};
  const auto qs = manager_csv::parsePathRows(rows);
  ASSERT_EQ(qs.size(), 1u);
  for (size_t i = 0; i < 6; ++i)
    EXPECT_DOUBLE_EQ(qs[0][i], static_cast<double>(i + 1));
}

TEST(ParsePathRows, FourColumnExpandsUnactuatedTube)
{
  // planner layout [β1, β2, α1, α2] -> [β1, β2, 0, α1, α2, 0]
  const std::vector<std::vector<double>> rows = {{-0.08, -0.05, 1.5, -2.5}};
  const auto qs = manager_csv::parsePathRows(rows);
  ASSERT_EQ(qs.size(), 1u);
  EXPECT_DOUBLE_EQ(qs[0][0], -0.08);
  EXPECT_DOUBLE_EQ(qs[0][1], -0.05);
  EXPECT_DOUBLE_EQ(qs[0][2], 0.0);
  EXPECT_DOUBLE_EQ(qs[0][3], 1.5);
  EXPECT_DOUBLE_EQ(qs[0][4], -2.5);
  EXPECT_DOUBLE_EQ(qs[0][5], 0.0);
}

TEST(ParsePathRows, BadWidthCountedAndSkipped)
{
  const std::vector<std::vector<double>> rows = {{1, 2, 3}, {1, 2, 3, 4}, {1, 2, 3, 4, 5}};
  size_t bad = 0;
  const auto qs = manager_csv::parsePathRows(rows, &bad);
  EXPECT_EQ(qs.size(), 1u);
  EXPECT_EQ(bad, 2u);
}

TEST(ParsePathRows, EndToEndFromCsvText)
{
  std::istringstream in("-0.08,-0.05,1.0,2.0\n-0.078,-0.05,1.0,2.0\n");
  const auto rows = ctr_common::csv::parseNumericRows(in);
  const auto qs = manager_csv::parsePathRows(rows);
  ASSERT_EQ(qs.size(), 2u);
  EXPECT_DOUBLE_EQ(qs[1][0], -0.078);
}

TEST(AdjustStepSize, EmptyInEmptyOut)
{
  EXPECT_TRUE(manager_csv::adjustConfigurationListStepSize({}, 1e-3).empty());
}

TEST(AdjustStepSize, KeepsFirstAndLast)
{
  std::vector<blaze::StaticVector<double, 6>> in;
  for (int i = 0; i < 10; ++i)
    in.push_back({i * 1e-4, 0, 0, 0, 0, 0}); // spacing below the step
  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);
  ASSERT_EQ(out.size(), 2u); // first + last only
  EXPECT_DOUBLE_EQ(out[0][0], 0.0);
  EXPECT_DOUBLE_EQ(out[1][0], 9e-4);
}

TEST(AdjustStepSize, DownsamplesOnFirstCoordinate)
{
  std::vector<blaze::StaticVector<double, 6>> in;
  for (int i = 0; i <= 10; ++i)
    in.push_back({i * 1e-3, 0, 0, 0, 0, 0});
  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);
  // kept: 0, 2e-3, 4e-3, 6e-3, 8e-3, 10e-3, + duplicated last (10e-3)
  ASSERT_GE(out.size(), 6u);
  for (size_t i = 1; i + 1 < out.size(); ++i)
    EXPECT_GE(std::abs(out[i][0] - out[i - 1][0]), 2e-3 - 1e-12);
}

// Regression: the keep rule measured travel on beta1 (index 0) only, so a
// beta2-dominant deployment of any magnitude was invisible to it and collapsed
// to a single unmanaged jump. Phase 2's "least-travel stops first" schedule
// produces beta2-dominant sub-phases routinely.
TEST(AdjustStepSize, DownsamplesOnSecondPrismaticCoordinate)
{
  std::vector<blaze::StaticVector<double, 6>> in;
  for (int i = 0; i <= 10; ++i)
    in.push_back({0, i * 1e-3, 0, 0, 0, 0}); // beta2 travels, beta1 does not

  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);

  // Must keep intermediate waypoints, not just first + last.
  ASSERT_GT(out.size(), 2u) << "beta2 travel was invisible to the keep rule";
  for (size_t i = 1; i + 1 < out.size(); ++i)
    EXPECT_GE(std::abs(out[i][1] - out[i - 1][1]), 2e-3 - 1e-12);
  EXPECT_DOUBLE_EQ(out.front()[1], 0.0);
  EXPECT_DOUBLE_EQ(out.back()[1], 10e-3);
}

// Regression: back() used to be appended unconditionally after a loop that ran
// to the end, so the final waypoint was emitted twice whenever the loop had
// already kept it. That is why a two-waypoint plan reported "holding at
// waypoint 2/2" on a duplicate.
TEST(AdjustStepSize, DoesNotDuplicateFinalWaypoint)
{
  std::vector<blaze::StaticVector<double, 6>> in = {
      {0.0, 0, 0, 0, 0, 0},
      {2e-3, 0, 0, 0, 0, 0},
      {4e-3, 0, 0, 0, 0, 0}};

  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);

  ASSERT_EQ(out.size(), 3u);
  EXPECT_DOUBLE_EQ(out[0][0], 0.0);
  EXPECT_DOUBLE_EQ(out[1][0], 2e-3);
  EXPECT_DOUBLE_EQ(out[2][0], 4e-3);
}

TEST(AdjustStepSize, SingleWaypointIsNotDuplicated)
{
  const std::vector<blaze::StaticVector<double, 6>> in = {{0.0, 0, 0, 0, 0, 0}};
  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);
  ASSERT_EQ(out.size(), 1u);
}

TEST(AdjustStepSize, TwoWaypointsSurviveExactly)
{
  const std::vector<blaze::StaticVector<double, 6>> in = {
      {0.0, 0, 0, 0, 0, 0}, {50e-3, 0, 0, 0, 0, 0}};
  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out[1][0], 50e-3);
}

// The revolute term must still work, and must still be measured on both alphas.
TEST(AdjustStepSize, DownsamplesOnRevolute)
{
  std::vector<blaze::StaticVector<double, 6>> in;
  for (int i = 0; i <= 10; ++i)
    in.push_back({0, 0, 0, 0, i * 0.05, 0}); // alpha2 (index 4) travels

  const auto out = manager_csv::adjustConfigurationListStepSize(in, 2e-3);

  ASSERT_GT(out.size(), 2u);
  for (size_t i = 1; i + 1 < out.size(); ++i)
    EXPECT_GE(std::abs(out[i][4] - out[i - 1][4]), 0.10 - 1e-12);
}

// ---------------------------------------------------------------------------
// buildHomeLeg -- the leg that turns "retract" into "retract to home".
// ---------------------------------------------------------------------------

namespace
{
// The manager's own view of the feasible set: alpha box from the dataset
// defaults, beta box reconstructed from the hardware geometry. Mirrors
// MasterNode::k_joint_limits.
const ctr_kinematics_pinn::JointLimits4 kLimits{
    {ctr_common::kHomePose[3], ctr_common::kPreEngagePose[3]},
    {-ctr_common::kLinearStageMaxClearance, -ctr_common::kLinearStageMinClearance}};

constexpr double kStep = 2e-3;

// physics order [β1, β2, β3, α1, α2, α3]
blaze::StaticVector<double, 6> q6(double b1, double b2, double a1, double a2)
{
  return {b1, b2, 0.0, a1, a2, 0.0};
}
}  // namespace

TEST(BuildHomeLeg, EndsExactlyAtCommandedHome)
{
  const auto home = ctr_common::homePoseCommanded();
  const auto leg = manager_csv::buildHomeLeg(q6(-0.0900, -0.0500, 0.8, 0.4), kStep);
  ASSERT_FALSE(leg.empty());
  EXPECT_DOUBLE_EQ(leg.back()[0], home[1]);  // β1
  EXPECT_DOUBLE_EQ(leg.back()[1], home[3]);  // β2
  EXPECT_DOUBLE_EQ(leg.back()[3], home[0]);  // α1
  EXPECT_DOUBLE_EQ(leg.back()[4], home[2]);  // α2
}

TEST(BuildHomeLeg, EmptyWhenAlreadyHome)
{
  const auto home = ctr_common::homePoseCommanded();
  const auto leg = manager_csv::buildHomeLeg(q6(home[1], home[3], home[0], home[2]), kStep);
  EXPECT_TRUE(leg.empty());
}

// The ordering guarantee: tubes are withdrawn BEFORE they are unwound. If the
// two sub-legs were interpolated together, the tubes would rotate while still
// inside the anatomy -- the thing the follow-the-leader plan exists to avoid.
TEST(BuildHomeLeg, RetractsBeforeUnwinding)
{
  const auto home = ctr_common::homePoseCommanded();
  const double a1 = 0.8, a2 = 0.4;
  const auto leg = manager_csv::buildHomeLeg(q6(-0.0900, -0.0500, a1, a2), kStep);
  ASSERT_FALSE(leg.empty());

  bool beta_home_reached = false;
  for (const auto &q : leg)
  {
    if (!beta_home_reached)
    {
      // Still retracting: alphas must not have moved yet.
      EXPECT_DOUBLE_EQ(q[3], a1);
      EXPECT_DOUBLE_EQ(q[4], a2);
      if (std::abs(q[0] - home[1]) < 1e-12 && std::abs(q[1] - home[3]) < 1e-12)
        beta_home_reached = true;
    }
    else
    {
      // Unwinding: betas are pinned at home.
      EXPECT_DOUBLE_EQ(q[0], home[1]);
      EXPECT_DOUBLE_EQ(q[1], home[3]);
    }
  }
  EXPECT_TRUE(beta_home_reached) << "the beta sub-leg never reached home";
}

TEST(BuildHomeLeg, EveryWaypointIsFeasible)
{
  const blaze::StaticVector<double, 6> starts[] = {
      q6(-0.0900, -0.0500, 0.8, 0.4),
      q6(ctr_common::kPreEngagePose[1], ctr_common::kPreEngagePose[3], -2.0, -1.4),
      q6(-0.1200, -0.0600, 0.0, 0.0),
      q6(-0.1490, -0.0680, 3.0, 2.2),
  };
  for (const auto &start : starts)
  {
    const auto leg = manager_csv::buildHomeLeg(start, kStep);
    for (const auto &q : leg)
    {
      EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({q[0], q[1], q[3], q[4]}, kLimits, 1e-9))
          << "b1=" << q[0] << " b2=" << q[1] << " a1=" << q[3] << " a2=" << q[4];
    }
  }
}

// The invariant the whole sequenced design exists for: the drive checks beta1
// against the LIVE beta2, i.e. the PREVIOUS waypoint's value, and beta2 against
// the previous beta1. Every step of the home leg must satisfy BOTH lagged
// bounds, not just the coupling at its own waypoint.
TEST(BuildHomeLeg, EveryStepIsLegalAgainstTheLaggingLiveCompanion)
{
  const blaze::StaticVector<double, 6> starts[] = {
      q6(-0.0900, -0.0500, 0.8, 0.4),
      q6(ctr_common::kPreEngagePose[1], ctr_common::kPreEngagePose[3], -2.0, -1.4),
      q6(-0.1200, -0.0600, 0.0, 0.0),
      q6(-0.1490, -0.0680, 3.0, 2.2),
      q6(-0.1400, -0.0560, -1.0, -0.5),
  };

  // Same tolerance rationale as isFeasible4's: the last start below sits
  // EXACTLY on the coupling floor, where the two sides of the comparison differ
  // by one ULP. This is about floating point, not about margin -- the guard
  // gives the generated waypoints a full millimetre.
  constexpr double kTol = 1e-9;

  for (const auto &start : starts)
  {
    blaze::StaticVector<double, 6> prev = start;
    for (const auto &q : manager_csv::buildHomeLeg(start, kStep))
    {
      // beta1 >= beta2_live - maxClearance
      EXPECT_GE(q[0], prev[1] - ctr_common::kLinearStageMaxClearance - kTol)
          << "beta1 " << q[0] << " vs live beta2 " << prev[1];
      // beta1 <= beta2_live - minClearance
      EXPECT_LE(q[0], prev[1] - ctr_common::kLinearStageMinClearance + kTol)
          << "beta1 " << q[0] << " vs live beta2 " << prev[1];
      // beta2 >= beta1_live + minClearance, beta2 <= beta1_live + maxClearance
      EXPECT_GE(q[1], prev[0] + ctr_common::kLinearStageMinClearance - kTol)
          << "beta2 " << q[1] << " vs live beta1 " << prev[0];
      EXPECT_LE(q[1], prev[0] + ctr_common::kLinearStageMaxClearance + kTol)
          << "beta2 " << q[1] << " vs live beta1 " << prev[0];
      prev = q;
    }
  }
}

// One carriage at a time is what makes the lagged bounds satisfiable at all.
TEST(BuildHomeLeg, BetaCarriagesNeverMoveTogether)
{
  const auto start = q6(-0.0900, -0.0500, 0.8, 0.4);
  blaze::StaticVector<double, 6> prev = start;
  for (const auto &q : manager_csv::buildHomeLeg(start, kStep))
  {
    const bool b1_moved = std::abs(q[0] - prev[0]) > 1e-12;
    const bool b2_moved = std::abs(q[1] - prev[1]) > 1e-12;
    EXPECT_FALSE(b1_moved && b2_moved)
        << "both carriages moved in one step: db1=" << (q[0] - prev[0])
        << " db2=" << (q[1] - prev[1]);
    prev = q;
  }
}

TEST(BuildHomeLeg, NoStepExceedsTheInsertionStep)
{
  const auto start = q6(ctr_common::kPreEngagePose[1], ctr_common::kPreEngagePose[3], 2.5, 1.9);
  const auto leg = manager_csv::buildHomeLeg(start, kStep);
  ASSERT_FALSE(leg.empty());

  blaze::StaticVector<double, 6> prev = start;
  for (const auto &q : leg)
  {
    EXPECT_LE(std::abs(q[0] - prev[0]), kStep + 1e-12);
    EXPECT_LE(std::abs(q[1] - prev[1]), kStep + 1e-12);
    EXPECT_LE(std::abs(q[3] - prev[3]), manager_csv::kAlphaStepDefault + 1e-12);
    EXPECT_LE(std::abs(q[4] - prev[4]), manager_csv::kAlphaStepDefault + 1e-12);
    prev = q;
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

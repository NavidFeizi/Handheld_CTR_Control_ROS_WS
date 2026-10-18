#include <gtest/gtest.h>

#include "manager/deployment_path.hpp"

#include <cmath>

using manager_deploy::Config;

namespace
{

constexpr double kBetaTol = 5.0e-3;              // m, mirrors MasterNode::k_plan_match_beta_tol
constexpr double kAlphaTol = 10.0 * M_PI / 180.0; // rad, mirrors MasterNode::k_theta_threshold

Config makeConfig(double beta1, double beta2, double alpha1, double alpha2)
{
  return Config{beta1, beta2, 0.0, alpha1, alpha2, 0.0};
}

std::vector<Config> makeActive()
{
  std::vector<Config> active;
  for (int i = 0; i < 10; ++i)
    active.push_back(makeConfig(-0.156 + 0.002 * i, -0.08, 1.0, 1.0));
  return active;
}

} // namespace

TEST(SpliceReplan, SpliceKeepsExecutedPrefixSoRetractReturnsToPlanStart)
{
  const auto active = makeActive();
  const int index = 4;
  const Config r1 = makeConfig(active[index][0] + 0.002, -0.08, 1.3, 1.3);
  const Config r2 = makeConfig(active[index][0] + 0.004, -0.08, 1.5, 1.5);
  const Config r3 = makeConfig(active[index][0] + 0.006, -0.08, 1.7, 1.7);
  const std::vector<Config> replanned = {active[index], r1, r2, r3};

  const auto result = manager_deploy::spliceReplan(active, index, replanned, kBetaTol, kAlphaTol);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 8u);

  for (int i = 0; i <= index; ++i)
    for (size_t k = 0; k < 6; ++k)
      EXPECT_DOUBLE_EQ((*result)[i][k], active[i][k]) << "i=" << i << " k=" << k;

  for (size_t k = 0; k < 6; ++k)
  {
    EXPECT_DOUBLE_EQ((*result)[5][k], r1[k]);
    EXPECT_DOUBLE_EQ((*result)[6][k], r2[k]);
    EXPECT_DOUBLE_EQ((*result)[7][k], r3[k]);
  }

  // Reverse-order retract must walk active[index..0] exactly.
  for (int i = index; i >= 0; --i)
    for (size_t k = 0; k < 6; ++k)
      EXPECT_DOUBLE_EQ((*result)[static_cast<size_t>(i)][k], active[static_cast<size_t>(i)][k]);
}

TEST(SpliceReplan, SpliceWithGoalAlreadyReached)
{
  const auto active = makeActive();
  const int index = 4;
  const std::vector<Config> replanned = {active[index]};

  const auto result = manager_deploy::spliceReplan(active, index, replanned, kBetaTol, kAlphaTol);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 5u);
  for (int i = 0; i <= index; ++i)
    for (size_t k = 0; k < 6; ++k)
      EXPECT_DOUBLE_EQ((*result)[i][k], active[i][k]);
}

TEST(SpliceReplan, SpliceRejectsReplanNotStartingAtCurrentWaypoint)
{
  const auto active = makeActive();
  const int index = 4;

  {
    Config bad_front = active[index];
    bad_front[0] += kBetaTol + 1e-4;
    const std::vector<Config> replanned = {bad_front, active[index]};
    EXPECT_FALSE(manager_deploy::spliceReplan(active, index, replanned, kBetaTol, kAlphaTol).has_value());
  }
  {
    Config bad_front = active[index];
    bad_front[3] += kAlphaTol + 1e-3;
    const std::vector<Config> replanned = {bad_front, active[index]};
    EXPECT_FALSE(manager_deploy::spliceReplan(active, index, replanned, kBetaTol, kAlphaTol).has_value());
  }
}

TEST(SpliceReplan, SpliceRejectsInvalidInputs)
{
  const auto active = makeActive();
  const std::vector<Config> replanned = {active[4]};

  EXPECT_FALSE(manager_deploy::spliceReplan(active, -1, replanned, kBetaTol, kAlphaTol).has_value());
  EXPECT_FALSE(
      manager_deploy::spliceReplan(active, static_cast<int>(active.size()), replanned, kBetaTol, kAlphaTol)
          .has_value());
  EXPECT_FALSE(manager_deploy::spliceReplan({}, 0, replanned, kBetaTol, kAlphaTol).has_value());
  EXPECT_FALSE(manager_deploy::spliceReplan(active, 4, {}, kBetaTol, kAlphaTol).has_value());
}

TEST(ConfigMatches, BoundariesNoWrap)
{
  const Config a = makeConfig(0.0, 0.0, 0.0, 0.0);

  Config at_tol = a;
  at_tol[0] += kBetaTol;
  at_tol[1] += kBetaTol;
  EXPECT_TRUE(manager_deploy::configMatches(a, at_tol, kBetaTol, kAlphaTol));

  Config over_tol = a;
  over_tol[0] += kBetaTol + 1e-9;
  EXPECT_FALSE(manager_deploy::configMatches(a, over_tol, kBetaTol, kAlphaTol));

  Config full_turn = a;
  full_turn[3] += 2.0 * M_PI;
  EXPECT_FALSE(manager_deploy::configMatches(a, full_turn, kBetaTol, kAlphaTol));
}

TEST(InsertionOutstanding, Cases)
{
  EXPECT_FALSE(manager_deploy::insertionOutstanding(0, 0, 0));
  EXPECT_FALSE(manager_deploy::insertionOutstanding(10, 0, 0));
  EXPECT_TRUE(manager_deploy::insertionOutstanding(10, 3, 0));
  EXPECT_FALSE(manager_deploy::insertionOutstanding(0, 0, 1));
  EXPECT_TRUE(manager_deploy::insertionOutstanding(0, 0, 2));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

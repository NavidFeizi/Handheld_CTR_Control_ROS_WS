#include <gtest/gtest.h>

#include "DeploymentSchedule.hpp"

#include <cmath>

namespace ds = deployment_schedule;
constexpr size_t N4 = 4;
using V = ds::JointVector<N4>;

namespace
{
// β travel must be monotone toward the goal for every candidate.
bool monotoneInBeta(const ds::Waypoints<N4> &wps, const V &q_from, const V &q_to)
{
  for (size_t tube = 0; tube < 2; ++tube)
  {
    const double dir = (q_to[tube] >= q_from[tube]) ? 1.0 : -1.0;
    for (size_t i = 1; i < wps.size(); ++i)
      if ((wps[i][tube] - wps[i - 1][tube]) * dir < -1e-12)
        return false;
  }
  return true;
}
}  // namespace

TEST(DeploymentSchedule, FourCandidatesStartAndEndCorrectly)
{
  const V q_from = {-0.080, -0.050, 0.5, -0.5};
  const V q_to = {-0.030, -0.010, 0.5, -0.5};
  const auto cands = ds::buildDeploymentCandidates<N4>(q_from, q_to, 2e-3);
  ASSERT_EQ(cands.size(), 4u);
  for (const auto &c : cands)
  {
    ASSERT_GE(c.wps.size(), 2u) << c.name;
    for (size_t i = 0; i < N4; ++i)
    {
      EXPECT_NEAR(c.wps.front()[i], q_from[i], 1e-12) << c.name;
    }
    // β coordinates reach the goal exactly; α held at q_from
    EXPECT_NEAR(c.wps.back()[0], q_to[0], 1e-12) << c.name;
    EXPECT_NEAR(c.wps.back()[1], q_to[1], 1e-12) << c.name;
    EXPECT_NEAR(c.wps.back()[2], q_from[2], 1e-12) << c.name;
    EXPECT_NEAR(c.wps.back()[3], q_from[3], 1e-12) << c.name;
  }
}

TEST(DeploymentSchedule, MonotoneAndStepBounded)
{
  const double safeStep = 2e-3;
  const V q_from = {-0.084, -0.020, 0.0, 0.0};
  const V q_to = {-0.030, -0.060, 0.0, 0.0}; // tube 2 retracts while tube 1 advances
  const auto cands = ds::buildDeploymentCandidates<N4>(q_from, q_to, safeStep);
  for (const auto &c : cands)
  {
    EXPECT_TRUE(monotoneInBeta(c.wps, q_from, q_to)) << c.name;
    for (size_t i = 1; i < c.wps.size(); ++i)
      for (size_t tube = 0; tube < 2; ++tube)
        EXPECT_LE(std::abs(c.wps[i][tube] - c.wps[i - 1][tube]), safeStep + 1e-12) << c.name;
  }
}

TEST(DeploymentSchedule, ZeroDisplacementDegenerates)
{
  const V q = {-0.05, -0.03, 1.0, -1.0};
  const auto cands = ds::buildDeploymentCandidates<N4>(q, q, 2e-3);
  for (const auto &c : cands)
  {
    for (const auto &wp : c.wps)
      for (size_t i = 0; i < N4; ++i)
        EXPECT_NEAR(wp[i], q[i], 1e-12) << c.name;
  }
}

TEST(DeploymentSchedule, SweptCostSumsSegments)
{
  // Cost provider = β1 path length; the swept cost must equal total β1 travel
  // regardless of the coarse stride.
  ds::Waypoints<N4> wps;
  for (int i = 0; i <= 100; ++i)
    wps.push_back({i * 1e-3, 0, 0, 0});
  const auto cost = ds::scheduleSweptCost<N4>(
      wps, [](const V &a, const V &b) { return std::abs(b[0] - a[0]); });
  EXPECT_NEAR(cost, 0.1, 1e-12);
}

TEST(DeploymentSchedule, SweptCostEmptyOrNoFunction)
{
  ds::Waypoints<N4> wps = {{0, 0, 0, 0}};
  EXPECT_DOUBLE_EQ(ds::scheduleSweptCost<N4>(wps, nullptr), 0.0);
  wps.push_back({1, 0, 0, 0});
  EXPECT_DOUBLE_EQ(ds::scheduleSweptCost<N4>(wps, nullptr), 0.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>

#include "ctr_common/path_downsample.hpp"

#include <cmath>
#include <vector>

using ctr_common::downsamplePath;
using ctr_common::kWaypointAlphaStep;
using ctr_common::kWaypointBetaStep;

// Phase-1 rotation of a two-phase plan: β fixed, both tubes rotate together.
// A β1-only criterion keeps just the endpoints (one ~1 rad joint jump).
TEST(DownsamplePath, PureRotationIsSampledInAlphaSteps)
{
  constexpr int kRows = 200;
  std::vector<blaze::StaticVector<double, 4>> in;
  for (int i = 0; i < kRows; ++i)
  {
    const double a = static_cast<double>(i) / (kRows - 1);  // 0 -> 1 rad
    in.push_back({-0.15, -0.07, a, a});
  }

  const auto out = downsamplePath(in, kWaypointBetaStep, kWaypointAlphaStep);
  ASSERT_GE(out.size(), 12u);
  EXPECT_DOUBLE_EQ(out.front()[2], 0.0);
  EXPECT_DOUBLE_EQ(out.back()[2], 1.0);
  for (size_t i = 1; i < out.size(); ++i)
  {
    EXPECT_LE(std::abs(out[i][2] - out[i - 1][2]), kWaypointAlphaStep + 1.0 / (kRows - 1)) << "kept pair " << i;
    EXPECT_LE(std::abs(out[i][3] - out[i - 1][3]), kWaypointAlphaStep + 1.0 / (kRows - 1)) << "kept pair " << i;
  }
}

// β2-only deployment phase in the manager's 6-column layout [β1, β2, β3, α1, α2, α3].
TEST(DownsamplePath, Beta2OnlyMotionIsSampledInBetaSteps)
{
  constexpr int kRows = 200;
  std::vector<blaze::StaticVector<double, 6>> in;
  for (int i = 0; i < kRows; ++i)
  {
    const double b2 = 0.02 * static_cast<double>(i) / (kRows - 1);  // 0 -> 20 mm
    in.push_back({-0.15, b2, 0.0, 0.3, 0.3, 0.0});
  }

  const auto out = downsamplePath(in, kWaypointBetaStep, kWaypointAlphaStep);
  EXPECT_GE(out.size(), 10u);
  EXPECT_DOUBLE_EQ(out.back()[1], 0.02);
}

TEST(DownsamplePath, EmptyAndSingleRow)
{
  EXPECT_TRUE(downsamplePath(std::vector<blaze::StaticVector<double, 4>>{}, kWaypointBetaStep, kWaypointAlphaStep).empty());

  const std::vector<blaze::StaticVector<double, 4>> one{{-0.15, -0.07, 0.1, 0.2}};
  const auto out = downsamplePath(one, kWaypointBetaStep, kWaypointAlphaStep);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0][3], 0.2);
}

// When the last row itself crosses a step it is kept once, not appended again.
TEST(DownsamplePath, LastRowNotDuplicated)
{
  std::vector<blaze::StaticVector<double, 4>> in;
  for (int i = 0; i <= 6; ++i)
  {
    in.push_back({i * 1.5e-3, 0.0, 0.0, 0.0});  // keeps rows 0, 2, 4, 6 (0, 3, 6, 9 mm)
  }

  const auto out = downsamplePath(in, kWaypointBetaStep, kWaypointAlphaStep);
  ASSERT_EQ(out.size(), 4u);
  EXPECT_DOUBLE_EQ(out.back()[0], 9.0e-3);
  EXPECT_NE(out[out.size() - 2][0], out.back()[0]);
}

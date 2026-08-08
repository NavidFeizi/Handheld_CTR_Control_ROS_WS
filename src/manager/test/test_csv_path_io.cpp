#include <gtest/gtest.h>

#include "manager/csv_path_io.hpp"

#include "ctr_common/csv_io.hpp"

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

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

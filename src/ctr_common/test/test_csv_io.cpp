#include <gtest/gtest.h>

#include "ctr_common/csv_io.hpp"

#include <sstream>

TEST(CsvIo, ParsesNumericRows)
{
  std::istringstream in("1.0,2.0,3.0\n4.5,-5.5,6.25\n");
  const auto rows = ctr_common::csv::parseNumericRows(in);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_DOUBLE_EQ(rows[0][0], 1.0);
  EXPECT_DOUBLE_EQ(rows[1][1], -5.5);
  EXPECT_DOUBLE_EQ(rows[1][2], 6.25);
}

TEST(CsvIo, SkipsHeaderAndEmptyLines)
{
  std::istringstream in("x,y,z\n1,2,3\n\n4,5,6\n");
  const auto rows = ctr_common::csv::parseNumericRows(in);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_DOUBLE_EQ(rows[0][2], 3.0);
  EXPECT_DOUBLE_EQ(rows[1][0], 4.0);
}

TEST(CsvIo, RaggedRowsPreserved)
{
  // Column-count policy belongs to the caller; the parser keeps row lengths.
  std::istringstream in("1,2\n3,4,5,6\n");
  const auto rows = ctr_common::csv::parseNumericRows(in);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].size(), 2u);
  EXPECT_EQ(rows[1].size(), 4u);
}

TEST(CsvIo, MissingFileIsNullopt)
{
  const auto rows = ctr_common::csv::readNumericCsv("/nonexistent/definitely_missing.csv");
  EXPECT_FALSE(rows.has_value());
}

TEST(CsvIo, ScientificNotation)
{
  std::istringstream in("2.625e-08,-4.830e-09\n");
  const auto rows = ctr_common::csv::parseNumericRows(in);
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_DOUBLE_EQ(rows[0][0], 2.625e-08);
  EXPECT_DOUBLE_EQ(rows[0][1], -4.830e-09);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

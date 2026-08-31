#include <gtest/gtest.h>

#include "ctr_common/diag_csv.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> readLines(const std::filesystem::path &p)
{
  std::ifstream in(p);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line))
    lines.push_back(line);
  return lines;
}

std::filesystem::path tempFile(const char *name)
{
  return std::filesystem::temp_directory_path() / "ctr_common_diag_csv_test" / name;
}

TEST(DiagCsv, WritesHeaderOnceAndFlushesEveryRecord)
{
  const auto path = tempFile("basic.csv");
  std::filesystem::remove_all(path.parent_path());

  ctr_common::DiagCsv csv(path, "a,b,c");
  csv.append("1,2,3");
  csv.append("4,5,6");

  // Flush-per-record: the file must be complete while the writer still lives.
  const auto lines = readLines(path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "a,b,c");
  EXPECT_EQ(lines[1], "1,2,3");
  EXPECT_EQ(lines[2], "4,5,6");
}

TEST(DiagCsv, CreatesMissingParentDirectories)
{
  const auto path = tempFile("deep/nested/dirs.csv");
  std::filesystem::remove_all(tempFile(""));

  ctr_common::DiagCsv csv(path, "x");
  csv.append("42");
  EXPECT_TRUE(std::filesystem::exists(path));
}

TEST(DiagCsv, UnconfiguredWriterIsANoOp)
{
  ctr_common::DiagCsv csv;
  csv.append("dropped");  // must not throw or create anything
  SUCCEED();
}

TEST(DiagCsv, ReopeningAnExistingFileDoesNotDuplicateTheHeader)
{
  const auto path = tempFile("reopen.csv");
  std::filesystem::remove_all(path.parent_path());

  {
    ctr_common::DiagCsv csv(path, "h1,h2");
    csv.append("1,1");
  }
  {
    ctr_common::DiagCsv csv(path, "h1,h2");
    csv.append("2,2");
  }

  const auto lines = readLines(path);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "h1,h2");
  EXPECT_EQ(lines[1], "1,1");
  EXPECT_EQ(lines[2], "2,2");
}

}  // namespace

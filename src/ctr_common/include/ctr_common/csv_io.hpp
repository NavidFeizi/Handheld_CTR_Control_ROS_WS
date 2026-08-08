#ifndef CTR_COMMON__CSV_IO_HPP_
#define CTR_COMMON__CSV_IO_HPP_

// Line-oriented numeric CSV parsing shared by manager and planner.
// Reading never creates directories or otherwise touches the filesystem
// beyond opening the file.

#include <filesystem>
#include <fstream>
#include <istream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ctr_common::csv
{

/// Parse comma-separated numeric rows from a stream. Rows with any
/// unparseable cell (e.g. a header line) are skipped; empty lines too.
inline std::vector<std::vector<double>> parseNumericRows(std::istream &in)
{
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(in, line))
  {
    std::istringstream ss(line);
    std::string cell;
    std::vector<double> row;
    bool ok = true;
    while (std::getline(ss, cell, ','))
    {
      try
      {
        row.push_back(std::stod(cell));
      }
      catch (const std::exception &)
      {
        ok = false;
        break;
      }
    }
    if (ok && !row.empty())
    {
      rows.push_back(std::move(row));
    }
  }
  return rows;
}

/// Open and parse a numeric CSV file; std::nullopt if it cannot be opened.
inline std::optional<std::vector<std::vector<double>>> readNumericCsv(const std::filesystem::path &file)
{
  std::ifstream in(file, std::ifstream::in);
  if (!in.is_open())
  {
    return std::nullopt;
  }
  return parseNumericRows(in);
}

}  // namespace ctr_common::csv

#endif  // CTR_COMMON__CSV_IO_HPP_

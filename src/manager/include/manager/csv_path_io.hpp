#ifndef MANAGER__CSV_PATH_IO_HPP_
#define MANAGER__CSV_PATH_IO_HPP_

// Pure planned-path CSV semantics, extracted from MasterNode so they are
// testable without ROS or a filesystem.

#include <cmath>
#include <cstddef>
#include <vector>

#include <blaze/Math.h>

namespace manager_csv
{

/// Numeric rows -> 6-element configurations [β1, β2, β3, α1, α2, α3].
/// 4-column rows (the planner's layout [β1, β2, α1, α2]) are expanded with
/// zero β3/α3; rows with any other width are counted in bad_rows and skipped.
inline std::vector<blaze::StaticVector<double, 6>> parsePathRows(
    const std::vector<std::vector<double>> &rows, size_t *bad_rows = nullptr)
{
  std::vector<blaze::StaticVector<double, 6>> out;
  size_t bad = 0;
  for (const auto &row : rows)
  {
    if (row.size() == 6)
    {
      out.push_back({row[0], row[1], row[2], row[3], row[4], row[5]});
    }
    else if (row.size() == 4)
    {
      out.push_back({row[0], row[1], 0.0, row[2], row[3], 0.0});
    }
    else
    {
      ++bad;
    }
  }
  if (bad_rows != nullptr)
  {
    *bad_rows = bad;
  }
  return out;
}

/// Downsample a configuration list so consecutive kept waypoints differ by at
/// least step_size in the FIRST coordinate (β1, the deployment translation).
/// The first and last configurations are always kept.
inline std::vector<blaze::StaticVector<double, 6>> adjustConfigurationListStepSize(
    const std::vector<blaze::StaticVector<double, 6>> &q_list_in, double step_size)
{
  std::vector<blaze::StaticVector<double, 6>> q_list_out;
  if (q_list_in.empty())
  {
    return q_list_out;
  }

  size_t prev_idx = 0;
  q_list_out.push_back(q_list_in[0]);
  for (size_t i = 1; i < q_list_in.size(); ++i)
  {
    if (std::abs(q_list_in[i][0] - q_list_in[prev_idx][0]) >= step_size)
    {
      prev_idx = i;
      q_list_out.push_back(q_list_in[i]);
    }
  }
  q_list_out.push_back(q_list_in.back());

  return q_list_out;
}

}  // namespace manager_csv

#endif  // MANAGER__CSV_PATH_IO_HPP_

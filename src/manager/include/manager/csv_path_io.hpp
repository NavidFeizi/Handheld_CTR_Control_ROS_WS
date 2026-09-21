#ifndef MANAGER__CSV_PATH_IO_HPP_
#define MANAGER__CSV_PATH_IO_HPP_

// Pure planned-path CSV semantics, extracted from MasterNode so they are
// testable without ROS or a filesystem.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <blaze/Math.h>

#include "ctr_common/home_pose.hpp"

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

/// Default revolute keep-threshold for adjustConfigurationListStepSize [rad].
inline constexpr double kAlphaStepDefault = 0.10;  // ≈ 5.7° per commanded step

/// Downsample a configuration list so consecutive kept waypoints differ by at
/// least step_size in EITHER prismatic joint (indices 0 and 1) OR by at least
/// alpha_step in either revolute joint (indices 3 and 4).
///
/// Two rules learned the hard way:
///   - Filtering on β1 alone collapsed every pure-rotation segment (the whole
///     Phase 1 rotation of a two-phase plan) into a single commanded step, so
///     the entire α slew executed as one unmanaged swing. Hence the α term.
///   - Filtering on β1 alone ALSO made β2 travel invisible: a β2-dominant
///     deployment of any magnitude collapsed to one unmanaged jump. Phase 2's
///     "least-travel stops first" schedule produces β2-dominant sub-phases
///     routinely, so this was reachable in normal operation. Hence the max over
///     both prismatic joints.
///
/// The first and last configurations are always kept.
inline std::vector<blaze::StaticVector<double, 6>> adjustConfigurationListStepSize(
    const std::vector<blaze::StaticVector<double, 6>> &q_list_in, double step_size,
    double alpha_step = kAlphaStepDefault)
{
  std::vector<blaze::StaticVector<double, 6>> q_list_out;
  if (q_list_in.empty())
  {
    return q_list_out;
  }

  size_t prev_idx = 0;
  q_list_out.push_back(q_list_in[0]);
  for (size_t i = 1; i + 1 < q_list_in.size(); ++i)
  {
    const double d_beta = std::max(std::abs(q_list_in[i][0] - q_list_in[prev_idx][0]),
                                   std::abs(q_list_in[i][1] - q_list_in[prev_idx][1]));
    const double d_alpha = std::max(std::abs(q_list_in[i][3] - q_list_in[prev_idx][3]),
                                    std::abs(q_list_in[i][4] - q_list_in[prev_idx][4]));
    if (d_beta >= step_size || d_alpha >= alpha_step)
    {
      prev_idx = i;
      q_list_out.push_back(q_list_in[i]);
    }
  }
  // Always end on the plan's final configuration, but do not duplicate it: the
  // loop above stops before the last element, so this appends it exactly once.
  // (It used to run to the end and then push back() unconditionally, emitting
  // the final waypoint twice whenever the loop had already kept it -- which is
  // why a 2-state plan reported "holding at waypoint 2/2" on a duplicate.)
  if (q_list_in.size() > 1)
  {
    q_list_out.push_back(q_list_in.back());
  }

  return q_list_out;
}

/// Waypoints that take a configuration from `from` to the mechanical home pose
/// (`ctr_common::homePoseCommanded()`), in the manager's physics order
/// [β1, β2, β3, α1, α2, α3]. `from` is not included; the last element is home.
///
/// Auto Retract used to stop at waypoint 0 of plannedPath.csv -- the pose the
/// robot happened to be in when the plan was made -- because the manager had no
/// concept of home at all. This is the leg that closes that gap.
///
/// FOUR SUB-LEGS, and the order of all four is load-bearing.
///
///   1. β₁ retracts to the gate pose, β₂ stationary.
///   2. β₂ retracts to home, β₁ stationary.
///   3. β₁ retracts the rest of the way to home, β₂ stationary.
///   4. α unwinds to zero, both carriages parked at home.
///
/// *Why α is last:* interpolating it alongside β would rotate tubes that are
/// still inside the anatomy, which is exactly what the follow-the-leader plan
/// exists to avoid.
///
/// *Why the β legs move ONE CARRIAGE AT A TIME:* home is a corner of the
/// feasible set -- β₁ − β₂ sits at the −0.084 floor with only the
/// `kHomePoseMargin` (0.1 mm) to spare. The drive evaluates β₁'s
/// `POSITION_LIMIT` against the **live** β₂, which still holds the previous
/// waypoint's value at the instant a new target arrives, so moving both at once
/// tightens β₁'s bound by a whole step of β₂ travel (sub-millimetre, but an
/// order of magnitude more than the margin) and the drive clips β₁ and stops
/// short, silently. Moving one carriage while the other is parked at a value it
/// has already reached removes the lag entirely. `test_csv_path_io` pins this.
///
/// The gate pose in sub-leg 1 is what makes sub-leg 2 legal: β₂ may not pass
/// `β₁_live + 0.030`, so β₁ must be at least `kHomeLegClearanceGuard` below
/// `β₂_home − 0.030` before β₂ can reach home. β₁ is only ever moved backwards
/// (`std::min`), so a configuration already retracted past the gate skips it.
inline std::vector<blaze::StaticVector<double, 6>> buildHomeLeg(
    const blaze::StaticVector<double, 6> &from, double step_size,
    double alpha_step = kAlphaStepDefault)
{
  // ctr_common works in wire order [α1, β1, α2, β2]; this list is physics order.
  const auto home = ctr_common::homePoseCommanded();
  const double home_alpha1 = home[0UL], home_beta1 = home[1UL];
  const double home_alpha2 = home[2UL], home_beta2 = home[3UL];

  const double beta1_gate =
      std::min(from[0], home_beta2 - ctr_common::kLinearStageMinClearance -
                            ctr_common::kHomeLegClearanceGuard);

  const std::array<double, 4UL> p0 = {from[3], from[0], from[4], from[1]};
  const std::array<double, 4UL> p1 = {from[3], beta1_gate, from[4], from[1]};
  const std::array<double, 4UL> p2 = {from[3], beta1_gate, from[4], home_beta2};
  const std::array<double, 4UL> p3 = {from[3], home_beta1, from[4], home_beta2};
  const std::array<double, 4UL> p4 = {home_alpha1, home_beta1, home_alpha2, home_beta2};

  std::vector<blaze::StaticVector<double, 6>> out;
  // NOT named `emit`: Qt defines that as an empty macro, and this header is
  // pulled into master_node.hpp.
  const auto appendLeg = [&out](const std::vector<std::array<double, 4UL>> &legs)
  {
    for (const auto &w : legs)
    {
      // wire [α1, β1, α2, β2] -> physics [β1, β2, β3, α1, α2, α3]
      out.push_back({w[1UL], w[3UL], 0.0, w[0UL], w[2UL], 0.0});
    }
  };

  appendLeg(ctr_common::interpolatePose(p0, p1, step_size, alpha_step));
  appendLeg(ctr_common::interpolatePose(p1, p2, step_size, alpha_step));
  appendLeg(ctr_common::interpolatePose(p2, p3, step_size, alpha_step));
  appendLeg(ctr_common::interpolatePose(p3, p4, step_size, alpha_step));
  return out;
}

/// Largest per-step revolute jump in a (downsampled) configuration list [rad].
/// A value near 2π means a rotation phase was collapsed and would execute as a
/// single unmanaged full turn.
inline double maxAlphaStep(const std::vector<blaze::StaticVector<double, 6>> &q_list)
{
  double max_step = 0.0;
  for (size_t i = 1; i < q_list.size(); ++i)
  {
    max_step = std::max({max_step,
                         std::abs(q_list[i][3] - q_list[i - 1][3]),
                         std::abs(q_list[i][4] - q_list[i - 1][4])});
  }
  return max_step;
}

}  // namespace manager_csv

#endif  // MANAGER__CSV_PATH_IO_HPP_

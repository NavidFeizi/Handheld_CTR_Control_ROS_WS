#ifndef MANAGER__DEPLOYMENT_PATH_HPP_
#define MANAGER__DEPLOYMENT_PATH_HPP_

// Pure deployment-path splice/match semantics, extracted from MasterNode so
// they are testable without ROS or a filesystem.

#include <blaze/Math.h>

#include <cmath>
#include <optional>
#include <vector>

namespace manager_deploy
{

using Config = blaze::StaticVector<double, 6>; // physics order [β1, β2, β3, α1, α2, α3]

/// |Δβ1|,|Δβ2| <= beta_tol and |Δα1|,|Δα2| <= alpha_tol; tube-3 entries ignored.
/// Angles are NOT wrapped: plans and feedback both carry absolute multi-turn angles.
inline bool configMatches(const Config &a, const Config &b, double beta_tol, double alpha_tol)
{
  return std::abs(a[0] - b[0]) <= beta_tol && std::abs(a[1] - b[1]) <= beta_tol &&
         std::abs(a[3] - b[3]) <= alpha_tol && std::abs(a[4] - b[4]) <= alpha_tol;
}

/// Splice a mid-insertion replan into the executed trajectory so retract still replays it:
/// returns active[0..index] followed by replanned[1..]. nullopt when active is empty,
/// index is outside [0, active.size()), replanned is empty, or replanned.front() does
/// not configMatches active[index].
inline std::optional<std::vector<Config>> spliceReplan(const std::vector<Config> &active, int index,
                                                         const std::vector<Config> &replanned, double beta_tol,
                                                         double alpha_tol)
{
  if (active.empty() || index < 0 || static_cast<size_t>(index) >= active.size() || replanned.empty())
  {
    return std::nullopt;
  }
  if (!configMatches(replanned.front(), active[static_cast<size_t>(index)], beta_tol, alpha_tol))
  {
    return std::nullopt;
  }

  std::vector<Config> out(active.begin(), active.begin() + index + 1);
  out.insert(out.end(), replanned.begin() + 1, replanned.end());
  return out;
}

/// Robot has left its plan start along an executed trajectory: open-loop list advanced past
/// waypoint 0, or closed-loop actuated stack holds more than its seed.
inline bool insertionOutstanding(size_t adjusted_size, int index, size_t actuated_size)
{
  return (adjusted_size > 0 && index > 0) || actuated_size > 1;
}

} // namespace manager_deploy

#endif // MANAGER__DEPLOYMENT_PATH_HPP_

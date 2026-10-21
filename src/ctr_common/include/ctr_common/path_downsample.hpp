#ifndef CTR_COMMON__PATH_DOWNSAMPLE_HPP_
#define CTR_COMMON__PATH_DOWNSAMPLE_HPP_

// Joint-space waypoint downsampling shared by the planner (displayed path) and
// the manager (executed waypoints), so both see the same waypoints.

#include <cmath>
#include <cstddef>
#include <vector>

#include <blaze/Math.h>

namespace ctr_common
{

inline constexpr double kWaypointBetaStep = 2.0e-3;               // m
inline constexpr double kWaypointAlphaStep = 5.0 * M_PI / 180.0;  // rad

/// Keep a configuration when any prismatic joint moved by >= beta_step or any
/// revolute joint by >= alpha_step since the last kept configuration. Covers
/// every actuated joint, so rotation-only and β2-only phases are not collapsed
/// into a single jump. The first configuration is always kept; the last is
/// appended unless it was already kept. Layouts: N = 4 [β1, β2, α1, α2],
/// N = 6 [β1, β2, β3, α1, α2, α3] (β3/α3 unactuated, ignored).
template <size_t N>
std::vector<blaze::StaticVector<double, N>> downsamplePath(const std::vector<blaze::StaticVector<double, N>> &q,
                                                           double beta_step, double alpha_step)
{
  static_assert(N == 4 || N == 6, "downsamplePath supports the 4- and 6-input joint layouts");
  constexpr size_t kBeta[2] = {0, 1};
  constexpr size_t kAlpha[2] = {N == 4 ? 2 : 3, N == 4 ? 3 : 4};

  std::vector<blaze::StaticVector<double, N>> out;
  if (q.empty())
  {
    return out;
  }

  size_t last_kept = 0;
  out.push_back(q.front());
  for (size_t i = 1; i < q.size(); ++i)
  {
    bool keep = false;
    for (const size_t j : kBeta)
    {
      keep = keep || std::abs(q[i][j] - q[last_kept][j]) >= beta_step;
    }
    for (const size_t j : kAlpha)
    {
      keep = keep || std::abs(q[i][j] - q[last_kept][j]) >= alpha_step;
    }
    if (keep)
    {
      last_kept = i;
      out.push_back(q[i]);
    }
  }
  if (last_kept != q.size() - 1)
  {
    out.push_back(q.back());
  }
  return out;
}

}  // namespace ctr_common

#endif  // CTR_COMMON__PATH_DOWNSAMPLE_HPP_

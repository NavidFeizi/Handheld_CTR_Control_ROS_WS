#ifndef CTR_COMMON__FINITE_GUARD_HPP_
#define CTR_COMMON__FINITE_GUARD_HPP_

// Finiteness predicates shared by every node that ingests a value it did not
// compute itself (topic payloads, service responses, model outputs).
//
// Why this exists: a single NaN minted by the EM tracker on a missing sensor
// frame reached the EKF, latched into its force state, was published on
// task_space/force_estimate, and was then fed BACK INTO the PINN as a network
// input by pinn_fk, the planner and the MPC. Nothing along that path checked
// finiteness, and because every acceptance threshold in the system is a `<` or
// `>` comparison -- all of which are FALSE against NaN -- the failure reported
// success at every gate it passed. See the 2026-09-01 incident notes.
//
// The rule this header exists to enforce: never let a non-finite value cross a
// process boundary, and never write one into persistent state.
//
// Deliberately free of ROS, Blaze and Eigen headers so it can be included from
// ROS-free targets (ctr_robot_driver, ctr_cosserat) and unit-tested standalone.
// It works with anything exposing size()/operator[] (blaze::StaticVector,
// std::array -- which is what rosidl generates for a fixed-size float64[N] --
// std::vector, Eigen vectors) and with blaze matrices via columns()/rows().

#include <cmath>
#include <cstddef>
#include <type_traits>

namespace ctr_common
{

/// @brief True when v is neither NaN nor infinite.
[[nodiscard]] inline bool isFinite(const double v) noexcept
{
  return std::isfinite(v);
}

/// @brief True when every element of a raw buffer is finite.
[[nodiscard]] inline bool allFinite(const double *const data, const std::size_t n) noexcept
{
  if (data == nullptr)
  {
    return false;
  }
  for (std::size_t i = 0UL; i < n; ++i)
  {
    if (!std::isfinite(data[i]))
    {
      return false;
    }
  }
  return true;
}

namespace detail
{
// Blaze matrices expose columns(); Blaze/Eigen vectors and std::array do not.
// Detecting columns() rather than rows() matters: Eigen vectors have rows() but
// spell the other dimension cols(), so keying on rows() would pick the matrix
// branch for them and fail to compile.
template <typename T, typename = void>
struct is_matrix_like : std::false_type
{
};
template <typename T>
struct is_matrix_like<T, std::void_t<decltype(std::declval<const T &>().columns()),
                                    decltype(std::declval<const T &>().rows())>> : std::true_type
{
};
} // namespace detail

/// @brief True when every element of a vector- or matrix-like container is finite.
template <typename T>
[[nodiscard]] bool allFinite(const T &x) noexcept
{
  if constexpr (detail::is_matrix_like<T>::value)
  {
    const std::size_t rows = static_cast<std::size_t>(x.rows());
    const std::size_t cols = static_cast<std::size_t>(x.columns());
    for (std::size_t i = 0UL; i < rows; ++i)
    {
      for (std::size_t j = 0UL; j < cols; ++j)
      {
        if (!std::isfinite(x(i, j)))
        {
          return false;
        }
      }
    }
    return true;
  }
  else
  {
    const std::size_t n = static_cast<std::size_t>(x.size());
    for (std::size_t i = 0UL; i < n; ++i)
    {
      if (!std::isfinite(x[i]))
      {
        return false;
      }
    }
    return true;
  }
}

/// @brief True when a scalar-first quaternion [w,x,y,z] can be used for an
///        orientation update: finite AND far enough from zero-norm to invert.
///
/// Both halves are load-bearing. The zero-norm half is the one the 1f7bf11 and
/// 05b0645 guards tested; the finiteness half is the one they were missing, and
/// NaN passed straight through them because `NaN < 1e-12` is false.
template <typename Quat>
[[nodiscard]] bool quatIsUsable(const Quat &q, const double min_norm2 = 1.0e-12) noexcept
{
  if (!allFinite(q))
  {
    return false;
  }
  const double norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  return std::isfinite(norm2) && norm2 >= min_norm2;
}

} // namespace ctr_common

#endif // CTR_COMMON__FINITE_GUARD_HPP_

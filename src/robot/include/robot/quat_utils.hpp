#ifndef ROBOT__QUAT_UTILS_HPP_
#define ROBOT__QUAT_UTILS_HPP_

// Quaternion helpers used by the EKF (extracted from ekf_node.cpp so they are
// testable without ROS). Quaternion format: [w, x, y, z] (scalar first).

#include <blaze/Math.h>

#include <cmath>

namespace robot_quat
{

inline blaze::StaticVector<double, 4UL> quat_multiply(const blaze::StaticVector<double, 4UL> &q, const blaze::StaticVector<double, 4UL> &r)
{
  double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  double r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3];

  return blaze::StaticVector<double, 4UL>{
      q0 * r0 - q1 * r1 - q2 * r2 - q3 * r3,
      q0 * r1 + q1 * r0 + q2 * r3 - q3 * r2,
      q0 * r2 - q1 * r3 + q2 * r0 + q3 * r1,
      q0 * r3 + q1 * r2 - q2 * r1 + q3 * r0};
}

// Rotate vector v by quaternion q and store result in v_out
inline void quat_rotate(const blaze::StaticVector<double, 4UL> &q, const blaze::StaticVector<double, 3UL> &v, blaze::StaticVector<double, 3UL> &v_out)
{
  const double qw = q[0];
  blaze::StaticVector<double, 3UL> qv{q[1], q[2], q[3]};

  auto t = 2.0 * blaze::cross(qv, v);
  v_out = v + qw * t + blaze::cross(qv, t);
}

// Quaternion inverse: conjugate divided by norm squared.
//
// The division is guarded. Callers hand this the PINN's raw predicted quaternion
// (the network output is not normalised), and an unguarded 0/0 here manufactures
// Inf/NaN from finite inputs -- which then propagates into the EKF gain, the
// force state, and from there into every node that evaluates the model. The
// conditional normalisations at the call sites do NOT protect this: they skip
// normalising a degenerate quaternion and hand that same quaternion straight
// here, and a NaN norm fails their `> 1e-10` test too.
//
// A degenerate input has no meaningful inverse, so return the identity: it makes
// the resulting error quaternion "no rotation" rather than poisoning the caller.
// Use quatIsUsable() (ctr_common/finite_guard.hpp) to detect and skip the update
// instead of relying on this fallback.
inline blaze::StaticVector<double, 4UL> quat_inverse(const blaze::StaticVector<double, 4UL> &q)
{
  const double norm_sq = blaze::dot(q, q);
  if (!std::isfinite(norm_sq) || norm_sq < 1.0e-12)
  {
    return blaze::StaticVector<double, 4UL>{1.0, 0.0, 0.0, 0.0};
  }
  return blaze::StaticVector<double, 4UL>{q[0], -q[1], -q[2], -q[3]} / norm_sq;
}

// Convert quaternion error to rotation vector using small-angle approximation
inline blaze::StaticVector<double, 3UL> quat_to_rotvec(const blaze::StaticVector<double, 4UL> &quat_err)
{
  auto q = quat_err;
  if (q[0] < 0.0)
    q = -q;
  return 2.0 * blaze::subvector(q, 1UL, 3UL);
}

}  // namespace robot_quat

#endif  // ROBOT__QUAT_UTILS_HPP_

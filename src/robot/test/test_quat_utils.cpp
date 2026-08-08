#include <gtest/gtest.h>

#include "robot/quat_utils.hpp"

#include <cmath>

using namespace robot_quat;

TEST(QuatUtils, IdentityMultiplication)
{
  const blaze::StaticVector<double, 4UL> id = {1, 0, 0, 0};
  const blaze::StaticVector<double, 4UL> q = {0.5, 0.5, 0.5, 0.5};
  const auto qi = quat_multiply(id, q);
  for (size_t i = 0; i < 4; ++i)
    EXPECT_DOUBLE_EQ(qi[i], q[i]);
}

TEST(QuatUtils, MultiplyByInverseIsIdentity)
{
  const blaze::StaticVector<double, 4UL> q = {0.7071, 0.7071, 0.0, 0.0};
  const auto r = quat_multiply(q, quat_inverse(q));
  EXPECT_NEAR(r[0], 1.0, 1e-6);
  for (size_t i = 1; i < 4; ++i)
    EXPECT_NEAR(r[i], 0.0, 1e-6);
}

TEST(QuatUtils, Rotate90AboutZ)
{
  const double s = std::sqrt(0.5);
  const blaze::StaticVector<double, 4UL> q = {s, 0, 0, s}; // +90° about z
  blaze::StaticVector<double, 3UL> out;
  quat_rotate(q, {1, 0, 0}, out);
  EXPECT_NEAR(out[0], 0.0, 1e-12);
  EXPECT_NEAR(out[1], 1.0, 1e-12);
  EXPECT_NEAR(out[2], 0.0, 1e-12);
}

TEST(QuatUtils, RotationPreservesNorm)
{
  const double s = std::sqrt(0.5);
  const blaze::StaticVector<double, 4UL> q = {s, s, 0, 0};
  blaze::StaticVector<double, 3UL> out;
  const blaze::StaticVector<double, 3UL> v = {0.3, -0.4, 1.2};
  quat_rotate(q, v, out);
  EXPECT_NEAR(blaze::norm(out), blaze::norm(v), 1e-12);
}

TEST(QuatUtils, RotvecSmallAngle)
{
  // small rotation about x by angle a: q ≈ [1, a/2, 0, 0] -> rotvec ≈ [a, 0, 0]
  const double a = 1e-3;
  const blaze::StaticVector<double, 4UL> q = {std::cos(a / 2), std::sin(a / 2), 0, 0};
  const auto rv = quat_to_rotvec(q);
  EXPECT_NEAR(rv[0], a, 1e-8);
  EXPECT_NEAR(rv[1], 0.0, 1e-12);
}

TEST(QuatUtils, RotvecHandlesNegativeW)
{
  // q and -q are the same rotation; the helper flips sign for w < 0
  const double a = 0.2;
  const blaze::StaticVector<double, 4UL> q = {std::cos(a / 2), std::sin(a / 2), 0, 0};
  const auto rv_pos = quat_to_rotvec(q);
  const auto rv_neg = quat_to_rotvec(-q);
  for (size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(rv_pos[i], rv_neg[i], 1e-12);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

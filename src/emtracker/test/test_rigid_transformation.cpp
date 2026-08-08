#include <gtest/gtest.h>

#include "RigidTransformation.hpp"

#include <cmath>

namespace
{
quatTransformation makeTransform(double heading, double attitude, double bank,
                                 const blaze::StaticVector<double, 3UL> &t)
{
  quatTransformation tr;
  Euler2Quaternion(heading, attitude, bank, tr.rotation);
  tr.translation = t;
  return tr;
}
}  // namespace

TEST(RigidTransformation, IdentityByDefault)
{
  const quatTransformation id;
  EXPECT_DOUBLE_EQ(id.rotation[0], 1.0);
  EXPECT_DOUBLE_EQ(blaze::norm(id.translation), 0.0);
}

TEST(RigidTransformation, InverseComposesToIdentity)
{
  const auto tr = makeTransform(0.3, -0.2, 1.1, {0.10, -0.05, 0.20});
  quatTransformation composed;
  Combine_Quat_Transformation(tr, tr.inv(), composed);

  EXPECT_NEAR(std::fabs(composed.rotation[0]), 1.0, 1e-9); // w = ±1
  for (size_t i = 1; i < 4; ++i)
    EXPECT_NEAR(composed.rotation[i], 0.0, 1e-9);
  for (size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(composed.translation[i], 0.0, 1e-9);
}

TEST(RigidTransformation, CompositionAssociative)
{
  const auto a = makeTransform(0.5, 0.1, -0.4, {1.0, 2.0, 3.0});
  const auto b = makeTransform(-0.2, 0.7, 0.3, {-0.5, 0.25, 0.75});
  const auto c = makeTransform(1.2, -0.6, 0.05, {0.0, -1.0, 0.5});

  quatTransformation ab, ab_c, bc, a_bc;
  Combine_Quat_Transformation(a, b, ab);
  Combine_Quat_Transformation(ab, c, ab_c);
  Combine_Quat_Transformation(b, c, bc);
  Combine_Quat_Transformation(a, bc, a_bc);

  for (size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(ab_c.translation[i], a_bc.translation[i], 1e-9);
  for (size_t i = 0; i < 4; ++i)
    EXPECT_NEAR(std::fabs(ab_c.rotation[i]), std::fabs(a_bc.rotation[i]), 1e-9);
}

TEST(RigidTransformation, PureTranslationComposition)
{
  quatTransformation a, b, ab;
  a.translation = {1.0, 0.0, 0.0};
  b.translation = {0.0, 2.0, 0.0};
  Combine_Quat_Transformation(a, b, ab);
  EXPECT_DOUBLE_EQ(ab.translation[0], 1.0);
  EXPECT_DOUBLE_EQ(ab.translation[1], 2.0);
  EXPECT_DOUBLE_EQ(ab.translation[2], 0.0);
}

TEST(RigidTransformation, RotationMovesTranslation)
{
  // 180° about z, then translate +x in the rotated frame -> -x in the base frame
  quatTransformation rot, step, out;
  Euler2Quaternion(M_PI, 0.0, 0.0, rot.rotation); // heading = yaw
  step.translation = {1.0, 0.0, 0.0};
  Combine_Quat_Transformation(rot, step, out);
  EXPECT_NEAR(out.translation[0], -1.0, 1e-9);
  EXPECT_NEAR(std::fabs(out.translation[1]), 0.0, 1e-9);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

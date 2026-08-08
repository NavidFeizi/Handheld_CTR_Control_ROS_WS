#include <gtest/gtest.h>

#include "ctr_common/joint_conventions.hpp"

#include <rclcpp/rclcpp.hpp>

// Wire order: [α1, β1, α2, β2] — physics order: [β1, β2, α1, α2].

TEST(JointConventions, WireToPhysics4)
{
  const std::array<double, 4> wire = {1.0, 2.0, 3.0, 4.0}; // α1 β1 α2 β2
  const auto q = ctr_common::wireToPhysics4(wire);
  EXPECT_DOUBLE_EQ(q[0], 2.0); // β1
  EXPECT_DOUBLE_EQ(q[1], 4.0); // β2
  EXPECT_DOUBLE_EQ(q[2], 1.0); // α1
  EXPECT_DOUBLE_EQ(q[3], 3.0); // α2
}

TEST(JointConventions, WireToPhysics6PadsUnactuatedTube)
{
  const std::array<double, 4> wire = {1.0, 2.0, 3.0, 4.0};
  const auto q = ctr_common::wireToPhysics6(wire);
  EXPECT_DOUBLE_EQ(q[0], 2.0); // β1
  EXPECT_DOUBLE_EQ(q[1], 4.0); // β2
  EXPECT_DOUBLE_EQ(q[2], 0.0); // β3 (unactuated)
  EXPECT_DOUBLE_EQ(q[3], 1.0); // α1
  EXPECT_DOUBLE_EQ(q[4], 3.0); // α2
  EXPECT_DOUBLE_EQ(q[5], 0.0); // α3 (unactuated)
}

TEST(JointConventions, RoundTrip4)
{
  const std::array<double, 4> wire = {-0.5, 0.25, 1.5, -0.75};
  const auto back = ctr_common::physicsToWire(ctr_common::wireToPhysics4(wire));
  for (size_t i = 0; i < 4; ++i)
    EXPECT_DOUBLE_EQ(back[i], wire[i]);
}

TEST(JointConventions, RoundTrip6)
{
  const std::array<double, 4> wire = {-0.5, 0.25, 1.5, -0.75};
  const auto back = ctr_common::physicsToWire(ctr_common::wireToPhysics6(wire));
  for (size_t i = 0; i < 4; ++i)
    EXPECT_DOUBLE_EQ(back[i], wire[i]);
}

TEST(ClampJointPositions, WithinLimitsUntouched)
{
  blaze::StaticVector<double, 4> q = {-0.05, -0.03, 0.5, -0.5};
  const blaze::StaticVector<double, 4> q_min = {-0.084, -0.09, -6.28, -6.28};
  const blaze::StaticVector<double, 4> q_max = {-0.03, 0.0, 6.28, 6.28};
  const auto q_before = q;
  ctr_common::clampJointPositions(q, q_min, q_max, rclcpp::get_logger("test"));
  // limits for joint 0 are shifted by q[1] (coupling): [-0.084-0.03, -0.03-0.03]
  EXPECT_DOUBLE_EQ(q[1], q_before[1]);
  EXPECT_DOUBLE_EQ(q[2], q_before[2]);
  EXPECT_DOUBLE_EQ(q[3], q_before[3]);
}

TEST(ClampJointPositions, CouplingShiftsTranslationLimits)
{
  // Joint 0's limits are q_min[0]+q[1] .. q_max[0]+q[1].
  blaze::StaticVector<double, 4> q = {0.10, 0.05, 0.0, 0.0};
  const blaze::StaticVector<double, 4> q_min = {-0.10, -0.10, -1.0, -1.0};
  const blaze::StaticVector<double, 4> q_max = {0.00, 0.00, 1.0, 1.0};
  ctr_common::clampJointPositions(q, q_min, q_max, rclcpp::get_logger("test"));
  // Effective max for joint 0 = 0.00 + q[1]. q[1] itself is clamped to 0.00 first? No:
  // the effective limits are computed from the ORIGINAL q, then applied element-wise.
  EXPECT_DOUBLE_EQ(q[1], 0.00);         // clamped to its own max
  EXPECT_DOUBLE_EQ(q[0], 0.00 + 0.05);  // clamped to max shifted by the original q[1]
}

TEST(ClampJointPositions, RotationLimitsCoupledToOuterRotation)
{
  // Like the translations, α1's limits shift by α2 (stage-relative actuation);
  // α2's own limits are absolute in the 4-joint variant.
  blaze::StaticVector<double, 4> q = {0.0, 0.0, 7.0, -7.0};
  const blaze::StaticVector<double, 4> q_min = {-1.0, -1.0, -6.28, -6.28};
  const blaze::StaticVector<double, 4> q_max = {1.0, 1.0, 6.28, 6.28};
  ctr_common::clampJointPositions(q, q_min, q_max, rclcpp::get_logger("test"));
  EXPECT_DOUBLE_EQ(q[3], -6.28);          // clamped to its absolute min
  EXPECT_DOUBLE_EQ(q[2], 6.28 + (-7.0));  // clamped to max shifted by the original α2
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

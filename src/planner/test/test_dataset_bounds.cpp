#include <gtest/gtest.h>

#include "ctr_kinematics_pinn/dataset_bounds.hpp"

#include <algorithm>
#include <array>

namespace
{
// Values shipped in models/ctr_8x91_0.18_tanh_9K_9K_50K_FP64/parameters.json,
// the model planner_params.yaml selects.
constexpr std::array<double, 2> kBeta1Relative = {-0.084, -0.030};
constexpr std::array<double, 2> kBeta2Absolute = {-0.072, -0.034};

// robot_node.cpp's k_home_pos[1] and k_pos_preEngage[1]: the inner stage's
// mechanical travel. The dataset's relative window, referred to beta2's range,
// must reproduce these exactly -- that identity is what makes the planner's
// start state valid at the retracted pose.
constexpr double kBeta1HardwareMin = 0.060 - 0.216;  // -0.156 m
constexpr double kBeta1HardwareMax = -0.0640;
}  // namespace

TEST(DatasetBounds, Beta1RelativeWindowMapsOntoHardwareTravel)
{
  const auto beta1 = ctr_kinematics_pinn::absoluteBeta1Range(kBeta1Relative, kBeta2Absolute);

  EXPECT_NEAR(beta1[0], kBeta1HardwareMin, 1e-12);
  EXPECT_NEAR(beta1[1], kBeta1HardwareMax, 1e-12);
}

// Guards the regression directly: using the relative window as if it were
// absolute puts the retracted pose outside the bounds, and beta1's admissible
// interval [lb, min(ub, beta2 - clearance)] comes out empty.
TEST(DatasetBounds, RawRelativeWindowWouldRejectTheRetractedPose)
{
  constexpr double kStageThickness = 30.0e-3;
  constexpr double kBeta1AtHome = kBeta1HardwareMin;
  constexpr double kBeta2AtHome = -0.072;

  const auto correct = ctr_kinematics_pinn::absoluteBeta1Range(kBeta1Relative, kBeta2Absolute);
  EXPECT_GE(kBeta1AtHome, correct[0]);
  EXPECT_LE(kBeta1AtHome, std::min(correct[1], kBeta2AtHome - kStageThickness));

  // The pre-fix behaviour, for contrast.
  EXPECT_LT(kBeta1AtHome, kBeta1Relative[0]);
  EXPECT_GT(kBeta1Relative[0], kBeta2AtHome - kStageThickness);
}

// The configuration actually measured on the robot at the homed pose (robot GUI,
// 2026-08-15): it clears the corrected lower bounds by 0.2 mm and 0.1 mm, and is far
// outside the regressed ones. This is the pose the operator is told to plan from.
TEST(DatasetBounds, MeasuredHomePoseIsValidUnderCorrectedBoundsOnly)
{
  constexpr double kStageThickness = 30.0e-3;
  constexpr double kBeta1Measured = -0.1558;
  constexpr double kBeta2Measured = -0.0719;

  const auto beta1 = ctr_kinematics_pinn::absoluteBeta1Range(kBeta1Relative, kBeta2Absolute);

  // CTR_StateValidityChecker::isValid's two prismatic terms, corrected bounds.
  EXPECT_GE(kBeta1Measured, beta1[0]);
  EXPECT_LE(kBeta1Measured, std::min(beta1[1], kBeta2Measured - kStageThickness));
  EXPECT_GE(kBeta2Measured, kBeta2Absolute[0]);
  EXPECT_LE(kBeta2Measured, kBeta2Absolute[1]);

  // Margins are sub-millimetre, which is why the start state gets a clamp tolerance.
  EXPECT_LT(kBeta1Measured - beta1[0], 1.0e-3);
  EXPECT_LT(kBeta2Measured - kBeta2Absolute[0], 1.0e-3);

  // Under the regressed bounds the admissible beta1 interval is empty: the lower bound
  // sits ABOVE the coupling ceiling, so no beta1 whatsoever could have passed.
  EXPECT_GT(kBeta1Relative[0], kBeta2Measured - kStageThickness);
}

TEST(DatasetBounds, ConversionIsAdditivePerEndpoint)
{
  const std::array<double, 2> rel = {-0.5, 0.25};
  const std::array<double, 2> abs2 = {1.0, 3.0};
  const auto out = ctr_kinematics_pinn::absoluteBeta1Range(rel, abs2);

  EXPECT_DOUBLE_EQ(out[0], 0.5);
  EXPECT_DOUBLE_EQ(out[1], 3.25);
}

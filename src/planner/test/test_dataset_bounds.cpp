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

// ===========================================================================
// The shared feasible-set predicate.
//
// These pin the two bugs the IK convergence benchmark
// (ctr_kinematics_pinn/benchmark/ik_bench.cpp) found: posCTRL and
// CTR_StateValidityChecker each enforced a different half of beta1's window and
// disagreed about beta2's ceiling, so 21% of *converged* IK solutions were
// rejected by setGoalState() and no plan could be produced.
// ===========================================================================

namespace
{
constexpr double kStageClearance = 30.0e-3;

// L = Ls + Lc for the shipped tubes: [0.216, 0.132, 0.060] m.
constexpr double kL1 = 0.158 + 0.058;
constexpr double kL2 = 0.077 + 0.055;

ctr_kinematics_pinn::JointLimits4 shippedLimits()
{
  ctr_kinematics_pinn::JointLimits4 lim;
  lim.beta2_absolute = kBeta2Absolute;
  lim.beta1_relative = kBeta1Relative;
  lim.alpha2_absolute = {-1.5 * M_PI, 1.5 * M_PI};  // dataset alpha2_range (absolute; alpha3 = 0)
  lim.alpha1_relative = {-M_PI, M_PI};              // dataset alpha1_range, RELATIVE to alpha2
  return lim;
}
}  // namespace

// beta1's relative window is doing double duty; losing either end loses a real
// constraint. This is why both consumers could drop half of it unnoticed.
TEST(FeasibleSet, Beta1WindowEncodesClearanceAndProtrusionTogether)
{
  // Upper edge IS the stage clearance: beta1 <= beta2 - clr.
  EXPECT_NEAR(kBeta1Relative[1], -kStageClearance, 1e-12);
  // Lower edge IS tube protrusion: beta1 + L1 >= beta2 + L2.
  EXPECT_NEAR(kBeta1Relative[0], kL2 - kL1, 1e-12);
}

// Both poses the hardware actually homes to sit exactly on the feasible set's
// boundary -- opposite corners of it -- so tightening the predicate must not
// reject either. This is the regression guard for the F2 fix.
TEST(FeasibleSet, BothHardwarePosesAreFeasibleAtZeroTolerance)
{
  const auto lim = shippedLimits();

  // k_home_pos: beta1/beta2 both at their minima, relative offset at its floor.
  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({-0.156, -0.072, 0.0, 0.0}, lim, 0.0));
  // k_pos_preEngage: both at their maxima, relative offset at its ceiling.
  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({-0.0640, -0.0340, 0.0, 0.0}, lim, 0.0));
}

// F1: what posCTRL used to return. It capped beta2 at -stageThickness and never
// consulted beta2's own dataset ceiling, so it walked 4 mm into an illegal band.
TEST(FeasibleSet, RejectsTheBeta2OvershootTheSolverUsedToReturn)
{
  const auto lim = shippedLimits();

  // beta2 = -0.030 is what the old cap min(-clr, L1+b1-L2) permitted.
  EXPECT_GT(-kStageClearance, kBeta2Absolute[1]);  // -0.030 > -0.034: the old cap was looser
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({-0.060, -0.030, 0.0, 0.0}, lim, 0.0));

  // beta2Window must never hand back anything above the dataset ceiling, for any beta1.
  for (double beta1 = -0.156; beta1 <= -0.064; beta1 += 0.002)
  {
    const auto w = ctr_kinematics_pinn::beta2Window(beta1, lim);
    EXPECT_LE(w[1], kBeta2Absolute[1] + 1e-15) << "beta1 = " << beta1;
  }
}

// F2: what CTR_StateValidityChecker used to accept. Box bounds plus clearance
// alone let the inner tube retract inside the middle one.
TEST(FeasibleSet, RejectsNonProtrudingStateTheOldCheckerAccepted)
{
  const auto lim = shippedLimits();
  const auto abs1 = ctr_kinematics_pinn::absoluteBeta1Range(kBeta1Relative, kBeta2Absolute);

  constexpr double beta1 = -0.150;  // far below beta2 - 0.084
  constexpr double beta2 = -0.040;

  // The old rule: absolute box + beta1 <= beta2 - clr. This state passes it.
  EXPECT_GE(beta1, abs1[0]);
  EXPECT_LE(beta1, std::min(abs1[1], beta2 - kStageClearance));
  EXPECT_GE(beta2, std::max(beta1 + kStageClearance, kBeta2Absolute[0]));
  EXPECT_LE(beta2, kBeta2Absolute[1]);

  // The shared predicate rejects it: beta1 - beta2 = -0.110 < -0.084.
  EXPECT_LT(beta1 - beta2, kBeta1Relative[0]);
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({beta1, beta2, 0.0, 0.0}, lim, 0.0));
}

// The windows must be mutually consistent: anything drawn from beta1Window(beta2)
// has to satisfy the predicate, and vice versa. This is the property posCTRL
// relies on when it clamps a step into the window.
TEST(FeasibleSet, WindowsOnlyProduceFeasibleConfigurations)
{
  const auto lim = shippedLimits();

  for (double beta2 = kBeta2Absolute[0]; beta2 <= kBeta2Absolute[1]; beta2 += 0.001)
  {
    const auto w1 = ctr_kinematics_pinn::beta1Window(beta2, lim);
    ASSERT_LE(w1[0], w1[1] + 1e-15) << "empty beta1 window at beta2 = " << beta2;

    for (const double beta1 : {w1[0], 0.5 * (w1[0] + w1[1]), w1[1]})
      EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({beta1, beta2, 0.0, 0.0}, lim, 1e-12))
          << "beta1 = " << beta1 << ", beta2 = " << beta2;
  }
}

// ===========================================================================
// The alpha domain, dataset-native form: alpha2 absolute (+-1.5pi, matching the
// hardware travel and the normaliser baked into the TorchScript archive) and
// alpha1 RELATIVE to it (+-pi). The inverted form -- alpha1 absolute in
// [-pi, pi], alpha2 anchored to it -- is the same relative-vs-absolute mixup
// this suite already pins for beta1; it spilled alpha2 out to +-2pi (untrained,
// unreachable) exactly in the azimuthal wedge around alpha1 = +-pi.
// ===========================================================================

TEST(FeasibleSet, AlphaWindowIsAnchoredToAlpha2)
{
  const auto lim = shippedLimits();
  const std::array<double, 2> beta = {-0.0640, -0.0340};

  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 1.0 + M_PI, 1.0}, lim, 1e-12));
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 1.0 + 1.01 * M_PI, 1.0}, lim, 0.0));
  // alpha2 itself is bounded by the trained travel...
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 1.51 * M_PI, 1.51 * M_PI}, lim, 0.0));
  // ...while alpha1 may legally exceed 1.5pi through the relative window.
  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 2.4 * M_PI, 1.45 * M_PI}, lim, 1e-12));
}

// The pose family the OLD domain rejected: the robot wound up past alpha1 = pi
// (measured values reached +6.75 rad on hardware; 12,945 recorded samples with
// |alpha1| > pi had a 0.0% deployment rate purely because of the artificial
// branch cut).
TEST(FeasibleSet, WoundUpPosesInsideTheTrainedBoxAreFeasible)
{
  const auto lim = shippedLimits();
  const std::array<double, 2> beta = {-0.0640, -0.0340};

  // alpha1 = 1.2pi with alpha2 = 1.1pi: both in the trained box; the old
  // alpha1 in [-pi, pi] rule threw the start state away.
  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 1.2 * M_PI, 1.1 * M_PI}, lim, 0.0));
  // The extreme trained corner: alpha2 at its travel limit, alpha1 a full
  // relative window beyond it.
  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 2.5 * M_PI, 1.5 * M_PI}, lim, 1e-12));
  // One step beyond the trained alpha1 span is out.
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({beta[0], beta[1], 2.51 * M_PI, 1.5 * M_PI}, lim, 0.0));
}

TEST(AlphaHelpers, WrapToPiIsPeriodicOntoPrincipalBranch)
{
  using ctr_kinematics_pinn::wrapToPi;
  EXPECT_NEAR(wrapToPi(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrapToPi(2.0 * M_PI + 0.3), 0.3, 1e-12);
  EXPECT_NEAR(wrapToPi(-2.0 * M_PI - 0.3), -0.3, 1e-12);
  EXPECT_NEAR(wrapToPi(6.75), 6.75 - 2.0 * M_PI, 1e-12);  // the measured wound-up alpha1
  // Half-open branch [-pi, pi): +pi maps to -pi.
  EXPECT_NEAR(wrapToPi(M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrapToPi(-M_PI), -M_PI, 1e-12);
}

TEST(AlphaHelpers, NearestGoalRepresentativePreservesShapeAndMinimisesTravel)
{
  const auto lim = shippedLimits();

  // Start wound to ~1.4pi on both axes; IK returns the principal-branch goal
  // (-pi + 0.1). The +2pi representative (pi + 0.1) is a whole turn closer and
  // still inside the trained box.
  const std::array<double, 4> q_goal = {-0.09, -0.05, -M_PI + 0.1, -M_PI + 0.1};
  const std::array<double, 4> q_start = {-0.156, -0.072, 1.4 * M_PI, 1.4 * M_PI};
  const auto rep = ctr_kinematics_pinn::nearestGoalRepresentative(q_goal, q_start, lim);

  EXPECT_NEAR(rep[2], M_PI + 0.1, 1e-12);
  EXPECT_NEAR(rep[3], M_PI + 0.1, 1e-12);
  // The relative angle -- the tube shape -- is untouched.
  EXPECT_NEAR(rep[2] - rep[3], q_goal[2] - q_goal[3], 1e-12);
  // Betas pass through.
  EXPECT_DOUBLE_EQ(rep[0], q_goal[0]);
  EXPECT_DOUBLE_EQ(rep[1], q_goal[1]);
  // And the result is still feasible.
  EXPECT_TRUE(ctr_kinematics_pinn::alphaFeasible(rep[2], rep[3], lim));
}

TEST(AlphaHelpers, NearestGoalRepresentativeNeverLeavesTheTrainedBox)
{
  const auto lim = shippedLimits();

  // A shift toward the start would leave the box (alpha2 + 2pi > 1.5pi), so the
  // in-box k = 0 representative must survive even though it is farther.
  const std::array<double, 4> q_goal = {-0.09, -0.05, 0.4 * M_PI, 0.4 * M_PI};
  const std::array<double, 4> q_start = {-0.156, -0.072, 1.5 * M_PI, 1.5 * M_PI};
  const auto rep = ctr_kinematics_pinn::nearestGoalRepresentative(q_goal, q_start, lim);

  EXPECT_NEAR(rep[2], 0.4 * M_PI, 1e-12);
  EXPECT_NEAR(rep[3], 0.4 * M_PI, 1e-12);
}

// The tolerance exists for measured encoder values straddling a boundary by
// floating-point noise; it must not be a licence to drift millimetres out.
TEST(FeasibleSet, ToleranceAdmitsNoiseButNotRealViolations)
{
  const auto lim = shippedLimits();

  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({-0.0640, -0.0340 + 1e-9, 0.0, 0.0}, lim, 0.0));
  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4({-0.0640, -0.0340 + 1e-9, 0.0, 0.0}, lim, 1e-6));
  // 1 mm out is a genuine violation, not noise.
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({-0.0640, -0.0340 + 1e-3, 0.0, 0.0}, lim, 1e-6));
}

// --------------------------------------------------------------------------
// The FK seed both model-evaluating nodes start from.
//
// ekf_node used to initialise m_q to all zeros implicitly, and only gated its
// correction on having a TIP measurement. Bringup starts the EM tracker 14 s
// before the robot nodes, so tip poses always arrive before joint feedback and
// the first FK queries of every run were made at q = [0,0,0,0] -- which is not
// merely at the edge of the trained box, it is far outside it. Both nodes now
// declare the same q0 in robot_params.yaml.
// --------------------------------------------------------------------------

TEST(FeasibleSet, SharedFkSeedQ0IsFeasible)
{
  const auto lim = shippedLimits();

  // robot_params.yaml: pinn_fk_node.q0 and ekf_node.q0, physics order
  // [beta1, beta2, alpha1, alpha2].
  const std::array<double, 4> q0 = {-0.100, -0.055, 0.0, 0.0};

  EXPECT_TRUE(ctr_kinematics_pinn::isFeasible4(q0, lim, 0.0))
      << "the seed both FK nodes start from must be a legal configuration";

  // Spell out why it is legal, so a future edit to the YAML fails here loudly.
  EXPECT_GE(q0[1], lim.beta2_absolute[0]);
  EXPECT_LE(q0[1], lim.beta2_absolute[1]);
  EXPECT_GE(q0[0] - q0[1], lim.beta1_relative[0]);
  EXPECT_LE(q0[0] - q0[1], lim.beta1_relative[1]);
}

TEST(FeasibleSet, AllZeroSeedIsInfeasible)
{
  const auto lim = shippedLimits();

  // beta2 = 0 is outside [-0.072, -0.034] and beta1 - beta2 = 0 is outside
  // [-0.084, -0.030]: two independent violations. Generous tolerance, to show
  // this is nowhere near the boundary rather than a rounding question.
  EXPECT_FALSE(ctr_kinematics_pinn::isFeasible4({0.0, 0.0, 0.0, 0.0}, lim, 1e-3));
}

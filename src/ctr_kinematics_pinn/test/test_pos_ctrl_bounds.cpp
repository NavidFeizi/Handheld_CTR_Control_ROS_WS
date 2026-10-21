#include <gtest/gtest.h>

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

// posCTRL must search the whole trained joint domain. beta1_range in the
// dataset is an offset RELATIVE to β₂ (β₁ = β₂ + u), so the absolute β₁ range
// is [β₂min − 0.084, β₂max − 0.030] = [−0.156, −0.064]. Treating the relative
// range as absolute caps β₁ at −0.084 and makes every target that needs a
// retracted inner tube (the proximal half of the workspace) unreachable.

namespace
{
using Tau = blaze::StaticVector<double, 4UL>;
using Vec3 = blaze::StaticVector<double, 3UL>;

constexpr double kPosTol = 5.0e-4;
constexpr std::size_t kRetractedTargets = 60;
constexpr std::size_t kMirrorPairs = 30;

struct Solve
{
  Tau q;
  double err;
};

class PosCtrl : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    pinn_ = std::make_unique<PINNs<4>>(CTR_PINN_MODELS_DIR, "ctr_8x91_0.18_tanh_9K_9K_50K_FP64", 1, 50);

    std::mt19937 rng(3);
    std::uniform_real_distribution<double> uBeta2(-0.072, -0.034);
    std::uniform_real_distribution<double> uRel(-0.084, -0.030);
    std::uniform_real_distribution<double> uAng(-M_PI, M_PI);
    auto randomConfig = [&]() {
      Tau q;
      q[1] = uBeta2(rng);
      q[0] = q[1] + uRel(rng);
      q[3] = uAng(rng);
      q[2] = q[3] + 0.95 * uAng(rng);
      return q;
    };

    while (retracted_.size() < kRetractedTargets)
    {
      const Tau q = randomConfig();
      if (q[0] >= -0.09)
        continue;
      retracted_.push_back(solve(tipOf(q)));
    }

    for (std::size_t i = 0; i < kMirrorPairs; ++i)
    {
      const Vec3 t = tipOf(randomConfig());
      const Vec3 tm = {-t[0], t[1], t[2]};
      mirrored_.emplace_back(solve(t), solve(tm));
    }
  }

  static void TearDownTestSuite()
  {
    retracted_.clear();
    mirrored_.clear();
    pinn_.reset();
  }

  static Vec3 tipOf(const Tau &q)
  {
    Vec3 p;
    pinn_->getPosDistal(q, kZeroForce, p);
    return p;
  }

  // Every solve starts from the fully retracted home configuration.
  static Solve solve(const Vec3 &target)
  {
    Tau q = {-0.156, -0.072, 0.0, 0.0};
    pinn_->posCTRL(q, target, kPosTol, kZeroForce);
    return {q, blaze::norm(tipOf(q) - target)};
  }

  static inline const Vec3 kZeroForce{0.0, 0.0, 0.0};
  static inline std::unique_ptr<PINNs<4>> pinn_;
  static inline std::vector<Solve> retracted_;
  static inline std::vector<std::pair<Solve, Solve>> mirrored_;
};

TEST_F(PosCtrl, RetractedInnerTubeTargetsConverge)
{
  std::size_t converged = 0;
  for (const auto &s : retracted_)
    converged += (s.err <= kPosTol) ? 1U : 0U;
  EXPECT_GE(converged, 45U) << converged << " of " << retracted_.size() << " converged";
}

TEST_F(PosCtrl, MirroredTargetsConvergeAlike)
{
  // The network is mirror symmetric: (β, −α) maps the tip (x, y, z) to (−x, y, z).
  // The seed (α = 0) is its own mirror, so a mirrored target must be solved
  // alike; the ~0.3 mm network asymmetry lets an occasional pair fall into
  // different local minima, hence a count rather than a per-pair assertion.
  std::size_t agreeing = 0;
  for (const auto &[s, sm] : mirrored_)
  {
    const bool sameOutcome = (s.err <= kPosTol) == (sm.err <= kPosTol);
    agreeing += (sameOutcome && std::fabs(s.err - sm.err) <= 1.5e-3) ? 1U : 0U;
  }
  // Observed 25/30 (disagreements split 3 vs 2 between the two x-halves).
  EXPECT_GE(agreeing, 20U) << agreeing << " of " << mirrored_.size() << " pairs agree";
}

TEST_F(PosCtrl, SolutionStaysInsideDatasetDomain)
{
  std::vector<Tau> all;
  for (const auto &s : retracted_)
    all.push_back(s.q);
  for (const auto &[s, sm] : mirrored_)
  {
    all.push_back(s.q);
    all.push_back(sm.q);
  }

  for (const auto &q : all)
  {
    EXPECT_GE(q[0], q[1] - 0.084 - 1e-9);
    EXPECT_LE(q[0], q[1] - 0.030 + 1e-9);
    EXPECT_GE(q[1], -0.072 - 1e-9);
    EXPECT_LE(q[1], -0.034 + 1e-9);
    EXPECT_LE(std::fabs(q[3] - q[2]), M_PI + 1e-9);
    // α₂ trained range (and middle-tube drive limit) is ±1.5π. Before the wrap
    // fix α₂ was kept relative to a [−π, π) α₁ and reached 4.90 / 6.15 rad here.
    EXPECT_LE(std::fabs(q[3]), 1.5 * M_PI + 1e-9);
  }
}

// alpha1_range is relative to α₂ (like beta1_range to β₂) and the trained α₂
// range is ±1.5π (the model's input normaliser spans 1.1·1.5π; the middle-tube
// drive limit is also ±1.5π). The absolute α₁ envelope is α₂range + α₁range.
TEST_F(PosCtrl, RevoluteRangesAreAbsoluteAndMatchTrainedDomain)
{
  const auto rr = pinn_->getRevoluteJointRanges();
  EXPECT_NEAR(rr[0], -2.5 * M_PI, 1e-9);
  EXPECT_NEAR(rr[1], 2.5 * M_PI, 1e-9);
  EXPECT_NEAR(rr[2], -1.5 * M_PI, 1e-9);
  EXPECT_NEAR(rr[3], 1.5 * M_PI, 1e-9);
}

// The FP64 model is trained on tip forces with ||f|| <= f_max = 0.5 N. A value on
// the surface is what the robot EKF publishes when its estimate is clamped
// (railed), so it must be reported as a violation, not accepted.
TEST_F(PosCtrl, ForceDomainMatchesTrainedFmax)
{
  EXPECT_DOUBLE_EQ(pinn_->getMaxTipForce(), 0.5);
  EXPECT_TRUE(pinn_->forceDomainViolation(Vec3{0.10, 0.27, 0.06}).empty());
  EXPECT_TRUE(pinn_->forceDomainViolation(Vec3{0.0, 0.5 - 1.0e-3, 0.0}).empty());

  // 2026-10-12 field estimate: each axis within the old per-axis ±0.5 N clip, norm 0.64 N
  const std::string railed = pinn_->forceDomainViolation(Vec3{-0.35, 0.50, 0.19});
  EXPECT_FALSE(railed.empty());
  EXPECT_NE(railed.find("f_max=0.5000"), std::string::npos) << railed;

  EXPECT_FALSE(pinn_->forceDomainViolation(Vec3{0.0, 0.5, 0.0}).empty());
}

// 2026-10-12 field target (y < 0) that failed at 25-43 mm while the EKF estimate
// was railed. With no tip force it, and its y-mirror, must be solved from the
// home configuration with the tubes pre-aligned the way the manager does it
// (theta = atan2(y, x) + pi/2, wrapped to [-pi, pi]).
TEST_F(PosCtrl, FieldTargetBelowAxisConvergesWithoutForceBias)
{
  auto solveAligned = [](const Vec3 &target)
  {
    double theta = std::atan2(target[1], target[0]) + M_PI / 2.0;
    if (theta > M_PI)
      theta -= 2.0 * M_PI;
    else if (theta < -M_PI)
      theta += 2.0 * M_PI;
    Tau q = {-0.1558, -0.0719, theta, theta};
    pinn_->posCTRL(q, target, kPosTol, kZeroForce);
    return blaze::norm(tipOf(q) - target);
  };

  const double err = solveAligned(Vec3{-0.0036, -0.0261, 0.1252});
  const double errMirror = solveAligned(Vec3{-0.0036, 0.0261, 0.1252});
  EXPECT_LE(err, kPosTol) << "y<0 field target: " << err * 1e3 << " mm";
  EXPECT_LE(errMirror, kPosTol) << "y>0 mirror: " << errMirror * 1e3 << " mm";
}

// The same field target with the railed EKF force the planner received in the field
// (||f|| > f_max) cannot be reached: every step is clamped to a prismatic limit and
// the error stays near 30 mm. It used to burn all 750 iterations; a solve pinned
// against a limit with no useful progress must stop well before the cap.
TEST_F(PosCtrl, LimitPinnedSolveStopsEarly)
{
  const Vec3 target{-0.0036, -0.0261, 0.1252};
  const Vec3 railedForce{-0.35, 0.50, 0.19};
  double theta = std::atan2(target[1], target[0]) + M_PI / 2.0;
  if (theta > M_PI)
    theta -= 2.0 * M_PI;
  else if (theta < -M_PI)
    theta += 2.0 * M_PI;
  Tau q = {-0.1558, -0.0719, theta, theta};

  PosCtrlReport report;
  pinn_->posCTRL(q, target, kPosTol, railedForce, &report);

  EXPECT_GT(report.finalError, kPosTol);
  EXPECT_EQ(report.termination, PosCtrlReport::Termination::LimitStalled)
      << "termination=" << static_cast<int>(report.termination);
  EXPECT_LE(report.iterations, 400UL) << "limit_hits=" << report.prismaticLimitHits
                                      << " error=" << report.finalError * 1e3 << " mm";
}

// 2026-10-13 field log: the same probe point re-selected after Disable + Go Home
// intermittently failed IK because the raw EKF force (0.3-0.5 N rest bias) was fed
// into the solve. With zero tip force (the fixed generateTrajectory model), the
// solve from the same seed must be bitwise deterministic and must converge within
// the planner's plan-acceptance tolerance (ik_accept_tol = 3 mm), matching
// Planner::solveInverseKinematics's first posCTRLMultiStart attempt (the caller's
// seed) with kBatch = 1 and kBackbonePoints = 150.
TEST(PosCtrlFieldRetry, FieldRetrySameProbeIsDeterministicAndAccepted)
{
  constexpr double kAcceptTol = 3.0e-3;
  PINNs<4> pinn(CTR_PINN_MODELS_DIR, "ctr_8x91_0.18_tanh_9K_9K_50K_FP64", 1, 150);
  const Vec3 zeroForce{0.0, 0.0, 0.0};

  struct Case
  {
    const char *name;
    Tau seed;
    Vec3 target;
  };
  const std::vector<Case> cases = {
      {"A1", Tau{-0.1558, -0.0719, 1.6825, 1.6827}, Vec3{0.027713, 0.002467, 0.126148}},
      {"A2", Tau{-0.1558, -0.0719, 1.6678, 1.6675}, Vec3{0.027785, 0.002732, 0.126414}},
      {"B1", Tau{-0.1558, -0.0719, -1.8064, -1.8064}, Vec3{-0.027281, 0.006169, 0.136382}},
      {"B2", Tau{-0.1558, -0.0719, -1.7813, -1.7806}, Vec3{-0.026589, 0.005689, 0.136562}},
  };

  for (const Case &c : cases)
  {
    Tau q1 = c.seed;
    PosCtrlReport report1;
    pinn.posCTRL(q1, c.target, kPosTol, zeroForce, &report1);

    Tau q2 = c.seed;
    PosCtrlReport report2;
    pinn.posCTRL(q2, c.target, kPosTol, zeroForce, &report2);

    for (std::size_t i = 0; i < 4; ++i)
      EXPECT_EQ(q1[i], q2[i]) << c.name << ": joint " << i << " not bitwise deterministic";
    EXPECT_EQ(report1.finalError, report2.finalError)
        << c.name << ": error not bitwise deterministic";

    EXPECT_LE(report1.finalError, kAcceptTol)
        << c.name << ": error=" << report1.finalError * 1e3
        << " mm, termination=" << static_cast<int>(report1.termination);
  }
}

// 2026-10-18 field log: in-workspace probe targets were rejected because posCTRL
// pinned β2 at its upper bound (half-step clamp without re-solving the free joints,
// integral wind-up), with chaotic outcomes for near-identical requests: #53 and #106/
// #107 each have a logged converged neighbour within 0.4 mm. #61 also returned a
// state on the coupled β bound that the planner rejected as an invalid goal. From
// the logged seeds, the planner's IK (posCTRLMultiStart, zero force) must now be
// accepted and return a strictly valid planner goal state.
TEST(PosCtrlField20261018, LoggedRejectedTargetsSolve)
{
  constexpr double kAcceptTol = 3.0e-3;
  constexpr double kClearance = 0.030; // stage thickness: β1 ≤ β2 − 0.030
  constexpr double kRelMin = -0.084;   // dataset: β1 − β2 ≥ −0.084
  PINNs<4> pinn(CTR_PINN_MODELS_DIR, "ctr_8x91_0.18_tanh_9K_9K_50K_FP64", 1, 150);
  const Vec3 zeroForce{0.0, 0.0, 0.0};

  struct Case
  {
    const char *name;
    Tau seed;
    Vec3 target;
  };
  const std::vector<Case> cases = {
      {"#53", Tau{-0.1529, -0.0719, -1.9415, -1.9417}, Vec3{-0.010473, 0.002838, 0.127482}},
      {"#106", Tau{-0.1529, -0.0719, -1.8324, -1.8322}, Vec3{-0.010605, 0.003713, 0.120701}},
      {"#107", Tau{-0.1529, -0.0719, -1.8324, -1.8322}, Vec3{-0.010612, 0.003696, 0.120707}},
      {"#57", Tau{-0.1529, -0.0719, 1.6757, 1.6757}, Vec3{0.0166, 0.002247, 0.1214}},
      {"#61", Tau{-0.1529, -0.0719, 1.6757, 1.6755}, Vec3{0.020262, 0.002274, 0.122103}},
  };

  for (const Case &c : cases)
  {
    Tau q = c.seed;
    PosCtrlReport report;
    pinn.posCTRLMultiStart(q, c.target, kPosTol, zeroForce, &report);

    EXPECT_LE(report.finalError, kAcceptTol)
        << c.name << ": error=" << report.finalError * 1e3 << " mm, termination="
        << static_cast<int>(report.termination) << " seed=" << report.seedIndex
        << " attempts=" << report.attempts;
    EXPECT_TRUE(pinn.domainViolation(q).empty()) << c.name << ": " << pinn.domainViolation(q);
    EXPECT_LE(q[0], q[1] - kClearance) << c.name << ": b1-b2=" << q[0] - q[1];
    EXPECT_GE(q[0], q[1] + kRelMin) << c.name << ": b1-b2=" << q[0] - q[1];
  }
}
}  // namespace

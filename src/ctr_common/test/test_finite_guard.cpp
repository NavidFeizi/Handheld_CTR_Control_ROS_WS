#include <gtest/gtest.h>

#include "ctr_common/finite_guard.hpp"

#include <blaze/Math.h>

#include <array>
#include <limits>
#include <vector>

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
} // namespace

TEST(FiniteGuard, ScalarIsFinite)
{
  EXPECT_TRUE(ctr_common::isFinite(0.0));
  EXPECT_TRUE(ctr_common::isFinite(-1.0e-30));
  EXPECT_FALSE(ctr_common::isFinite(kNaN));
  EXPECT_FALSE(ctr_common::isFinite(kInf));
  EXPECT_FALSE(ctr_common::isFinite(-kInf));
}

TEST(FiniteGuard, RawBuffer)
{
  const std::array<double, 3UL> good = {1.0, 2.0, 3.0};
  const std::array<double, 3UL> bad = {1.0, kNaN, 3.0};
  EXPECT_TRUE(ctr_common::allFinite(good.data(), good.size()));
  EXPECT_FALSE(ctr_common::allFinite(bad.data(), bad.size()));
  EXPECT_FALSE(ctr_common::allFinite(nullptr, 3UL));
  // A zero-length buffer is vacuously finite.
  EXPECT_TRUE(ctr_common::allFinite(good.data(), 0UL));
}

// rosidl generates std::array<double, N> for a fixed-size float64[N] field, so
// this is the exact type that arrives on Taskspace.p / Taskspace.h.
TEST(FiniteGuard, StdArrayMatchesMessageFields)
{
  EXPECT_TRUE(ctr_common::allFinite(std::array<double, 4UL>{1.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(ctr_common::allFinite(std::array<double, 4UL>{kNaN, kNaN, kNaN, kNaN}));
  // The incident signature: finite position alongside a non-finite quaternion.
  EXPECT_TRUE(ctr_common::allFinite(std::array<double, 3UL>{-0.0034, 0.0022, 0.0599}));
  EXPECT_FALSE(ctr_common::allFinite(std::array<double, 4UL>{0.0, kNaN, 0.0, 0.0}));
}

TEST(FiniteGuard, StdVectorAndBlazeVector)
{
  EXPECT_TRUE(ctr_common::allFinite(std::vector<double>{1.0, 2.0}));
  EXPECT_FALSE(ctr_common::allFinite(std::vector<double>{1.0, kInf}));

  blaze::StaticVector<double, 3UL> v{0.1, 0.2, 0.3};
  EXPECT_TRUE(ctr_common::allFinite(v));
  v[2UL] = kNaN;
  EXPECT_FALSE(ctr_common::allFinite(v));

  blaze::DynamicVector<double> d(2UL, 1.0);
  EXPECT_TRUE(ctr_common::allFinite(d));
  d[1UL] = -kInf;
  EXPECT_FALSE(ctr_common::allFinite(d));
}

TEST(FiniteGuard, BlazeMatrixTakesTheMatrixBranch)
{
  blaze::StaticMatrix<double, 3UL, 2UL> m(0.0);
  EXPECT_TRUE(ctr_common::allFinite(m));
  m(2UL, 1UL) = kNaN;
  EXPECT_FALSE(ctr_common::allFinite(m));

  // Column-major storage must be walked correctly too (the EKF's H is columnMajor).
  blaze::StaticMatrix<double, 2UL, 3UL, blaze::columnMajor> cm(1.0);
  EXPECT_TRUE(ctr_common::allFinite(cm));
  cm(1UL, 2UL) = kInf;
  EXPECT_FALSE(ctr_common::allFinite(cm));

  blaze::DynamicMatrix<double> dm(2UL, 2UL, 0.5);
  EXPECT_TRUE(ctr_common::allFinite(dm));
  dm(0UL, 1UL) = kNaN;
  EXPECT_FALSE(ctr_common::allFinite(dm));
}

TEST(FiniteGuard, QuatIsUsableRejectsNaNAndZeroNorm)
{
  // Identity is usable.
  EXPECT_TRUE(ctr_common::quatIsUsable(std::array<double, 4UL>{1.0, 0.0, 0.0, 0.0}));
  // A real measured quaternion is usable.
  EXPECT_TRUE(ctr_common::quatIsUsable(std::array<double, 4UL>{0.7071, 0.7071, 0.0, 0.0}));

  // The zero default that Taskspace.h carried before commit 1f7bf11.
  EXPECT_FALSE(ctr_common::quatIsUsable(std::array<double, 4UL>{0.0, 0.0, 0.0, 0.0}));

  // The NaN sentinel EMTracker::ToolData2QuatTransform writes on a missing
  // frame. This is the case both previous zero-norm guards let through.
  EXPECT_FALSE(ctr_common::quatIsUsable(std::array<double, 4UL>{kNaN, kNaN, kNaN, kNaN}));
  EXPECT_FALSE(ctr_common::quatIsUsable(std::array<double, 4UL>{1.0, kNaN, 0.0, 0.0}));
  EXPECT_FALSE(ctr_common::quatIsUsable(std::array<double, 4UL>{kInf, 0.0, 0.0, 0.0}));

  // Near-zero norm: too small to invert without blowing up quat_inverse.
  EXPECT_FALSE(ctr_common::quatIsUsable(std::array<double, 4UL>{1.0e-9, 0.0, 0.0, 0.0}));

  blaze::StaticVector<double, 4UL> bq{0.0, 0.0, 0.0, 1.0};
  EXPECT_TRUE(ctr_common::quatIsUsable(bq));
  bq[3UL] = kNaN;
  EXPECT_FALSE(ctr_common::quatIsUsable(bq));
}

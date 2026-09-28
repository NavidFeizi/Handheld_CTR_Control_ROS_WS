#include <gtest/gtest.h>

#include "FtlSweptArea.hpp"

#include <cmath>

namespace
{
using Shape = blaze::DynamicMatrix<double, blaze::rowMajor>;

constexpr std::size_t N = 50;
constexpr double R = 0.05;

// Circular arc of radius R in the x-z plane starting at the origin tangent to
// +z, sampled at absolute arc length s_i = i * len / (N-1).
Shape arc(const double len, const std::size_t n = N)
{
  Shape m(n, 3UL, 0.0);
  for (std::size_t i = 0; i < n; ++i)
  {
    const double s = (n > 1) ? static_cast<double>(i) * len / static_cast<double>(n - 1) : 0.0;
    m(i, 0UL) = R * (1.0 - std::cos(s / R));
    m(i, 2UL) = R * std::sin(s / R);
  }
  return m;
}

Shape line(const double len)
{
  Shape m(N, 3UL, 0.0);
  for (std::size_t i = 0; i < N; ++i)
    m(i, 2UL) = static_cast<double>(i) * len / static_cast<double>(N - 1);
  return m;
}

Shape mirrorX(Shape m)
{
  for (std::size_t i = 0; i < m.rows(); ++i)
    m(i, 0UL) = -m(i, 0UL);
  return m;
}
}  // namespace

TEST(FtlSweptArea, PerfectFtlExtensionCostsZero)
{
  // Same curve, 20 mm appended at the tip: the deployed backbone does not move.
  // Residual is chord-interpolation error only (sagitta h²/8R ≈ 7 µm per node).
  const double cost = ftl::lateralSweptArea(arc(0.06), 0.06, arc(0.08), 0.08);
  EXPECT_LT(cost, 1e-6);
  // For scale: a 1 mm lateral shift of the same 60 mm backbone costs 6e-5 m².
  Shape shifted = arc(0.06);
  for (std::size_t i = 0; i < N; ++i)
    shifted(i, 1UL) += 1.0e-3;
  EXPECT_GT(ftl::lateralSweptArea(arc(0.06), 0.06, shifted, 0.06), 50.0 * cost);
}

TEST(FtlSweptArea, TangentialSlipCostsZero)
{
  const Shape s1 = line(0.10);
  Shape s2 = s1;
  for (std::size_t i = 0; i < N; ++i)
    s2(i, 2UL) += 2.0e-3;
  EXPECT_LT(ftl::lateralSweptArea(s1, 0.10, s2, 0.10), 1e-9);
}

TEST(FtlSweptArea, LateralShiftCostsAreaDTimesLength)
{
  const Shape s1 = line(0.10);
  Shape s2 = s1;
  for (std::size_t i = 0; i < N; ++i)
    s2(i, 0UL) += 0.003;
  EXPECT_NEAR(ftl::lateralSweptArea(s1, 0.10, s2, 0.10), 0.003 * 0.10, 1e-6);
}

TEST(FtlSweptArea, CostIsAdditiveAlongAMonotoneLateralMotion)
{
  // Splitting a lateral translation into two halves must not change its cost
  // (the former squared-displacement metric halved under each split).
  const Shape s0 = line(0.10);
  Shape s1 = s0, s2 = s0;
  for (std::size_t i = 0; i < N; ++i)
  {
    s1(i, 0UL) += 0.002;
    s2(i, 0UL) += 0.004;
  }
  const double whole = ftl::lateralSweptArea(s0, 0.10, s2, 0.10);
  const double split = ftl::lateralSweptArea(s0, 0.10, s1, 0.10) + ftl::lateralSweptArea(s1, 0.10, s2, 0.10);
  EXPECT_NEAR(split, whole, 1e-12);
}

TEST(FtlSweptArea, MirrorInvariance)
{
  const Shape a = arc(0.07);
  Shape b = arc(0.09);
  for (std::size_t i = 0; i < N; ++i)
  {
    b(i, 0UL) += 1.0e-3 * static_cast<double>(i) / static_cast<double>(N - 1);
    b(i, 1UL) += 5.0e-4;
  }
  const double cost = ftl::lateralSweptArea(a, 0.07, b, 0.09);
  EXPECT_GT(cost, 0.0);
  EXPECT_NEAR(ftl::lateralSweptArea(mirrorX(a), 0.07, mirrorX(b), 0.09), cost, 1e-12);
}

TEST(FtlSweptArea, DegenerateInputsReturnZero)
{
  EXPECT_EQ(ftl::lateralSweptArea(arc(0.05, 1), 0.05, arc(0.05, 1), 0.05), 0.0);
  EXPECT_EQ(ftl::lateralSweptArea(arc(0.05), 0.05, arc(0.05, N + 1), 0.05), 0.0);
  EXPECT_EQ(ftl::lateralSweptArea(arc(0.05), 0.0, arc(0.05), 0.05), 0.0);
  EXPECT_EQ(ftl::lateralSweptArea(arc(0.05), 0.05, arc(0.05), -0.01), 0.0);
}

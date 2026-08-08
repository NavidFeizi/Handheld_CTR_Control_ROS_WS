#include <gtest/gtest.h>

#include "Butterworth.hpp"

#include <cmath>

// 2nd-order low-pass Butterworth over blaze vectors.

TEST(Butterworth, ConstantSignalPassesThrough)
{
  ButterworthFilter<3UL> filter(0.01); // 100 Hz sampling
  filter.update_coeffs(5.0);           // 5 Hz cutoff

  const blaze::StaticVector<double, 3UL> u = {1.0, -2.0, 0.5};
  blaze::StaticVector<double, 3UL> y;
  for (int i = 0; i < 500; ++i)
    y = filter.add_data_point(u);

  for (size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(y[i], u[i], 1e-6);
}

TEST(Butterworth, AttenuatesAboveCutoff)
{
  const double fs = 100.0, fc = 2.0, f_test = 40.0;
  ButterworthFilter<1UL> filter(1.0 / fs);
  filter.update_coeffs(fc);

  double peak = 0.0;
  for (int i = 0; i < 1000; ++i)
  {
    const double t = i / fs;
    const blaze::StaticVector<double, 1UL> u = {std::sin(2.0 * M_PI * f_test * t)};
    const auto y = filter.add_data_point(u);
    if (i > 500) // past the transient
      peak = std::max(peak, std::fabs(y[0]));
  }
  // 2nd-order rolloff: 40 dB/decade above 2 Hz -> 40 Hz should be < 0.01x
  EXPECT_LT(peak, 0.02);
}

TEST(Butterworth, PassesBelowCutoff)
{
  const double fs = 100.0, fc = 10.0, f_test = 0.5;
  ButterworthFilter<1UL> filter(1.0 / fs);
  filter.update_coeffs(fc);

  double peak = 0.0;
  for (int i = 0; i < 2000; ++i)
  {
    const double t = i / fs;
    const blaze::StaticVector<double, 1UL> u = {std::sin(2.0 * M_PI * f_test * t)};
    const auto y = filter.add_data_point(u);
    if (i > 1000)
      peak = std::max(peak, std::fabs(y[0]));
  }
  EXPECT_NEAR(peak, 1.0, 0.05);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

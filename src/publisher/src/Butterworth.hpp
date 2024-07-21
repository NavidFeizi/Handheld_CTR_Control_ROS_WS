
#ifndef BTW_FILTER_HPP
#define BTW_FILTER_HPP

#include <iostream>
#include <blaze/Math.h>
#include <stdexcept>
#include <vector>
#include <fstream>
#include <string>

template <size_t N>
struct ButterworthFilterCoefficients
{
    blaze::StaticVector<double, N> b0, b1, b2, a1, a2;
};

template <size_t N>
class ButterworthFilter
{
public:
    // Explicitly state that sample_time is the period in seconds for clarity
    explicit ButterworthFilter(double sample_time) : m_sample_time(sample_time),
                                                     m_prev_U1(zeroVector()), m_prev_U2(zeroVector()),
                                                     m_prev_filtered_U1(zeroVector()), m_prev_filtered_U2(zeroVector())
    {
        m_fs = 1 / sample_time;
    }
    
    void update_coeffs(double fc)
    {
        const double pi = 3.14159265358979323846;
        double omega = 2 * pi * fc / m_fs; // Normalized cutoff frequency
        double sin_omega = sin(omega);
        double cos_omega = cos(omega);
        double alpha = sin_omega / sqrt(2.0); // sqrt(2) gives a maximally flat response (Q factor)

        double a0 = 1 + alpha;
        m_coeffs.b0 = blaze::StaticVector<double, N>((1 - cos_omega) / 2 / a0);
        m_coeffs.b1 = blaze::StaticVector<double, N>((1 - cos_omega) / a0);
        m_coeffs.b2 = m_coeffs.b0;
        m_coeffs.a1 = blaze::StaticVector<double, N>(-2 * cos_omega / a0);
        m_coeffs.a2 = blaze::StaticVector<double, N>((1 - alpha) / a0);
    }

    blaze::StaticVector<double, N> add_data_point(const blaze::StaticVector<double, N> &U)
    {
        blaze::StaticVector<double, N> filtered_U = m_coeffs.b0 * U + m_coeffs.b1 * m_prev_U1 + m_coeffs.b2 * m_prev_U2 - m_coeffs.a1 * m_prev_filtered_U1 - m_coeffs.a2 * m_prev_filtered_U2;

        m_prev_U2 = m_prev_U1;
        m_prev_U1 = U;
        m_prev_filtered_U2 = m_prev_filtered_U1;
        m_prev_filtered_U1 = filtered_U;

        return filtered_U;
    }

private:
    static blaze::StaticVector<double, N> zeroVector()
    {
        return blaze::StaticVector<double, N>(0);
    }

    double m_sample_time, m_fs;
    blaze::StaticVector<double, N> m_prev_U1, m_prev_U2;
    blaze::StaticVector<double, N> m_prev_filtered_U1, m_prev_filtered_U2;
    ButterworthFilterCoefficients<N> m_coeffs;
};

#endif // BTW_FILTER_HPP
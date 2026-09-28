#ifndef FTL_SWEPT_AREA_HPP_
#define FTL_SWEPT_AREA_HPP_

// Follow-the-leader deviation metric between two backbone shapes, kept free
// of OMPL and Torch so it is unit-testable (see test/test_ftl_swept_area.cpp).
//
// With s the absolute arc length from the fixed base, a perfect FTL motion
// leaves p(s) unchanged for every already-deployed s; only motion normal to
// the backbone violates FTL, tangential motion (tubes sliding along their own
// curve) is free. The metric therefore compares the two backbones at equal
// ABSOLUTE arc length over the common deployed length and integrates only the
// component of the displacement normal to the backbone.

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <blaze/Math.h>

namespace ftl
{

namespace detail
{

// Linear interpolation of a backbone polyline at absolute arc length s.
// Rows are points at s_i = i * len / (N-1); returns the bracketing index i0.
inline std::size_t interpolateAt(const blaze::DynamicMatrix<double, blaze::rowMajor> &shape, const double len,
                                 const double s, blaze::StaticVector<double, 3UL> &p)
{
    const std::size_t N = shape.rows();
    const double u = std::clamp(s / len, 0.00, 1.00) * static_cast<double>(N - 1UL);
    const std::size_t i0 = std::min(N - 2UL, static_cast<std::size_t>(std::floor(u)));
    const double w = u - static_cast<double>(i0);
    for (std::size_t c = 0UL; c < 3UL; ++c)
        p[c] = (1.00 - w) * shape(i0, c) + w * shape(i0 + 1UL, c);
    return i0;
}

}  // namespace detail

// Lateral (normal-to-backbone) area swept between two backbone polylines.
// shape rows = points at absolute arc length s_i = i * len / (N-1), i = 0..N-1.
// Returns  Σ_k w_k ||d_k − (d_k·t_k) t_k|| · Δs  (trapezoid rule: w_0 = w_{N-1} = ½, else 1),
// s_k = k * ℓ/(N-1), ℓ = min(len1, len2),
// d_k = p2(s_k) − p1(s_k) (linear interpolation on each shape's own grid),
// t_k = unit tangent of shape1 at s_k (difference of the bracketing rows;
// if its norm < 1e-12 the full d_k is used). Returns 0 if N < 2, rows differ, or ℓ ≤ 0.
// Units: m². Linear in the displacement, hence additive under path refinement.
inline double lateralSweptArea(const blaze::DynamicMatrix<double, blaze::rowMajor> &shape1, const double len1,
                               const blaze::DynamicMatrix<double, blaze::rowMajor> &shape2, const double len2)
{
    const std::size_t N = shape1.rows();
    if (N < 2UL || shape2.rows() != N || shape1.columns() < 3UL || shape2.columns() < 3UL)
        return 0.00;

    const double ell = std::min(len1, len2);
    if (!(ell > 0.00))
        return 0.00;

    const double ds = ell / static_cast<double>(N - 1UL);

    double area = 0.00;
    blaze::StaticVector<double, 3UL> p1, p2, t, d;
    for (std::size_t k = 0UL; k < N; ++k)
    {
        const double s = static_cast<double>(k) * ds;
        const std::size_t i0 = detail::interpolateAt(shape1, len1, s, p1);
        detail::interpolateAt(shape2, len2, s, p2);
        d = p2 - p1;

        for (std::size_t c = 0UL; c < 3UL; ++c)
            t[c] = shape1(i0 + 1UL, c) - shape1(i0, c);
        const double tn = blaze::norm(t);
        if (tn >= 1.00E-12)
        {
            t /= tn;
            d -= blaze::dot(d, t) * t;
        }

        const double w = (k == 0UL || k + 1UL == N) ? 0.50 : 1.00;
        area += w * blaze::norm(d) * ds;
    }

    return area;
}

}  // namespace ftl

#endif  // FTL_SWEPT_AREA_HPP_

#pragma once

#include <blaze/Math.h>
#include <cmath>
#include "MathOperations.hpp"

template <std::size_t N, std::size_t numTendons>
class PDESystem
{
public:
    // default constructor
    PDESystem();

    // overloaded constructor
    PDESystem(blaze::StaticVector<double, 3UL> v, blaze::StaticVector<double, 3UL> u, blaze::StaticVector<double, 4UL> h, blaze::StaticVector<double, 3UL> q, blaze::StaticVector<double, 3UL> w);

    // copy constructor
    PDESystem(const PDESystem &rhs);

    // move constructor
    PDESystem(PDESystem &&rhs) noexcept;

    // CTR destructor
    ~PDESystem() = default;

    // copy assignment operator
    PDESystem &operator=(const PDESystem &rhs);

    // move assignment operator
    PDESystem &operator=(PDESystem &&rhs) noexcept;

    // functor that implements the system of ODEs governing an N-tendon catheter
    void operator()(const blaze::StaticVector<double, 19UL + numTendons> &y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticVector<double, 18UL> &z_h, blaze::StaticVector<double, 18UL> &z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

    // Integrates the ODE model equations using Euler's method
    void euler_PDE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticMatrix<double, 18UL, N> &Z_h, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

    // Integrates the ODE model equations using the classic 4th-order Runge-Kutta algorithm
    void rungeKutta_PDE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticMatrix<double, 18UL, N> &Z_h, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

    void adamsBashford_PDE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticMatrix<double, 18UL, N> &Z_h, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

private:
    blaze::StaticMatrix<double, 3UL, 3UL> m_R;   // rotation matrix SO(3) -- orientation of local frame wrt global frame
    blaze::StaticVector<double, 4UL> m_h;        // non-unity quaternions -- parametrizing the rotation
    blaze::StaticVector<double, 3UL> m_u, m_v;   // u: curvature vector | v: linear rate of change of position wrt arc-length
    blaze::StaticVector<double, 3UL> m_q, m_w;   // q: linear velocity | w: angular valocity
    blaze::StaticMatrix<double, 6UL, 6UL> m_Phi; // linear system matrix for determining vs, us
    blaze::StaticVector<double, 3UL> m_us;       // first spatial derivative of the curvature vector
    blaze::StaticVector<double, 3UL> m_vs;       // first spatial derivative of the strain vector
};

// default constructor
template <std::size_t N, std::size_t numTendons>
PDESystem<N, numTendons>::PDESystem()
{
    m_v = 0.00;
    m_u = 0.00;
    m_q = 0.00;
    m_w = 0.00;
    m_h = {1.00, 0.00, 0.00, 0.00};
    mathOp::getSO3(m_h, m_R);
    m_Phi = 0.00;
    m_us = 0.00;
    m_vs = 0.00;
}

// overloaded constructor
template <std::size_t N, std::size_t numTendons>
PDESystem<N, numTendons>::PDESystem(blaze::StaticVector<double, 3UL> v, blaze::StaticVector<double, 3UL> u, blaze::StaticVector<double, 4UL> h, blaze::StaticVector<double, 3UL> q, blaze::StaticVector<double, 3UL> w) : m_v(v), m_u(u), m_h(h), m_q(q), m_w(w)
{
    this->getSO3();
    m_Phi = 0.0;
    m_us = 0.0;
    m_vs = 0.0;
}

// copy constructor
template <std::size_t N, std::size_t numTendons>
PDESystem<N, numTendons>::PDESystem(const PDESystem &rhs) : m_v(rhs.m_v), m_u(rhs.m_u), m_h(rhs.m_h), m_R(rhs.m_R), m_Phi(rhs.m_Phi), m_q(rhs.m_q), m_w(rhs.m_w), m_us(rhs.m_us), m_vs(rhs.m_vs) {}

// move constructor
template <std::size_t N, std::size_t numTendons>
PDESystem<N, numTendons>::PDESystem(PDESystem &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_v = std::move(rhs.m_v);
        this->m_u = std::move(rhs.m_u);
        this->m_h = std::move(rhs.m_h);
        this->m_R = std::move(rhs.m_R);
        this->m_q = std::move(rhs.m_q);
        this->m_w = std::move(rhs.m_w);
        this->m_Phi = std::move(rhs.m_Phi);
        this->m_us = std::move(rhs.m_us);
        this->m_vs = std::move(rhs.m_vs);
    }
}

// copy assignment operator
template <std::size_t N, std::size_t numTendons>
PDESystem<N, numTendons> &PDESystem<N, numTendons>::operator=(const PDESystem<N, numTendons> &rhs)
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_v = rhs.m_v;
        this->m_u = rhs.m_u;
        this->m_h = rhs.m_h;
        this->m_R = rhs.m_R;
        this->m_q = rhs.m_q;
        this->m_w = rhs.m_w;
        this->m_Phi = rhs.m_Phi;
        this->m_us = rhs.m_us;
        this->m_vs = rhs.m_vs;
    }

    return *this;
}

// move assignment operator
template <std::size_t N, std::size_t numTendons>
PDESystem<N, numTendons> &PDESystem<N, numTendons>::operator=(PDESystem<N, numTendons> &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_v = std::move(rhs.m_v);
        this->m_u = std::move(rhs.m_u);
        this->m_h = std::move(rhs.m_h);
        this->m_R = std::move(rhs.m_R);
        this->m_q = std::move(rhs.m_q);
        this->m_w = std::move(rhs.m_w);
        this->m_Phi = std::move(rhs.m_Phi);
        this->m_us = std::move(rhs.m_us);
        this->m_vs = std::move(rhs.m_vs);
    }

    return *this;
}

// functor that implements the system of PDEs governing an N-tendon catheter
template <std::size_t N, std::size_t numTendons>
void PDESystem<N, numTendons>::operator()(const blaze::StaticVector<double, 19UL + numTendons> &y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticVector<double, 18UL> &z_h, blaze::StaticVector<double, 18UL> &z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    // dyds := [ps, hs, vs, us, qs, ws, numTendons] ==>> First spatial derivative of the (19+numTendons)-dimensional state vector
    // zh := [vh uh qh wh vs_h us_h]  ==>> history terms for the time implicit discretization
    this->m_h = blaze::subvector<3UL, 4UL>(y);
    this->m_v = blaze::subvector<7UL, 3UL>(y);
    this->m_u = blaze::subvector<10UL, 3UL>(y);
    this->m_q = blaze::subvector<13UL, 3UL>(y);
    this->m_w = blaze::subvector<16UL, 3UL>(y);
    mathOp::getSO3(m_h, m_R);

    // declaring auxiliar variables
    blaze::StaticVector<double, 3UL> a, a_i, b, pbs_i;
    blaze::StaticMatrix<double, 3UL, 3UL> A_i, G, G_i;
    blaze::StaticVector<double, numTendons> pbsNorm;

    auto Blk1 = blaze::submatrix<0UL, 0UL, 3UL, 3UL>(this->m_Phi); // block matrix corresponding to: Kse + c0*Bse + A
    auto Blk2 = blaze::submatrix<3UL, 3UL, 3UL, 3UL>(this->m_Phi); // block matrix corresponding to: Kbt + c0*Bbt + H
    Blk1 = Kse + (c0 * Bse);
    Blk2 = Kbt + (c0 * Bbt);

    // iterating through the tendons in the robot sheath
    for (size_t idx = 0UL; idx < numTendons; ++idx)
    {
        pbs_i = blaze::cross(this->m_u, r[idx]) + this->m_v;
        pbsNorm[idx] = blaze::norm(pbs_i);
        A_i = -(tau[idx] / pow(pbsNorm[idx], 3)) * mathOp::hatSqr(pbs_i);
        G_i = -mathOp::hatPostMultiply(A_i, r[idx]);
        a_i = A_i * blaze::cross(this->m_u, pbs_i);

        a += a_i;
        b += blaze::cross(r[idx], a_i);
        Blk1 += A_i;
        G += G_i;
        Blk2 += mathOp::hatPreMultiply(r[idx], G_i);
    }

    // complete populating the PHI matrix for computing v_s & u_s
    blaze::submatrix<0UL, 3UL, 3UL, 3UL>(this->m_Phi) = G;
    blaze::submatrix<3UL, 0UL, 3UL, 3UL>(this->m_Phi) = blaze::trans(G);

    // computing time derivatives of the state vector from the implicit time discretizations
    blaze::StaticVector<double, 3UL> v_t, u_t, q_t, w_t;
    v_t = (c0 * this->m_v) + blaze::subvector<0UL, 3UL>(z_h);
    u_t = (c0 * this->m_u) + blaze::subvector<3UL, 3UL>(z_h);
    q_t = (c0 * this->m_q) + blaze::subvector<6UL, 3UL>(z_h);
    w_t = (c0 * this->m_w) + blaze::subvector<9UL, 3UL>(z_h);

    blaze::StaticVector<double, 3UL> vs_h, us_h;
    vs_h = blaze::subvector<12UL, 3UL>(z_h);
    us_h = blaze::subvector<15UL, 3UL>(z_h);

    // internal forces and moments
    blaze::StaticVector<double, 3UL> nb, mb, e_3 = {0.00, 0.00, 1.00};
    nb = Kse * (this->m_v - e_3) + Bse * v_t;
    mb = Kbt * this->m_u + Bbt * u_t;

    // right hand side vector -- for determining us, vs
    blaze::StaticVector<double, 6UL> rhs;
    blaze::subvector<0UL, 3UL>(rhs) = -a + rho * A * (blaze::cross(this->m_w, this->m_q) + q_t) + C * (this->m_q * blaze::abs(this->m_q)) - blaze::trans(this->m_R) * rho * A * g - blaze::cross(this->m_u, nb) - Bse * vs_h;
    blaze::subvector<3UL, 3UL>(rhs) = -b + rho * (blaze::cross(this->m_w, J * this->m_w) + J * w_t) - blaze::cross(this->m_v, nb) - blaze::cross(this->m_u, mb) - Bbt * us_h;

    // State differential equations
    blaze::subvector<0UL, 3UL>(dyds) = this->m_R * this->m_v;                                                          // p_s
    blaze::subvector<3UL, 4UL>(dyds) = mathOp::quaternionDiff(this->m_u, this->m_h);                                   // h_s
    blaze::subvector<7UL, 6UL>(dyds) = blaze::solve(this->m_Phi, rhs);                                                 // v_s & u_s
    blaze::subvector<13UL, 3UL>(dyds) = v_t - blaze::cross(this->m_u, this->m_q) + blaze::cross(this->m_w, this->m_v); // q_s
    blaze::subvector<16UL, 3UL>(dyds) = u_t - blaze::cross(this->m_u, this->m_w);                                      // w_s
    blaze::subvector<19UL, numTendons>(dyds) = pbsNorm;                                                                // rate of change of tendons' lenghts

    blaze::subvector<0UL, 3UL>(z) = this->m_v;                         // v
    blaze::subvector<3UL, 3UL>(z) = this->m_u;                         // u
    blaze::subvector<6UL, 3UL>(z) = this->m_q;                         // q
    blaze::subvector<9UL, 3UL>(z) = this->m_w;                         // w
    blaze::subvector<12UL, 6UL>(z) = blaze::subvector<7UL, 6UL>(dyds); // v_s & u_s
}

// Integrates the PDE model equations using Euler's method
template <std::size_t N, std::size_t numTendons>
void PDESystem<N, numTendons>::euler_PDE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticMatrix<double, 18UL, N> &Z_h, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    double ds = L / (N - 1);
    blaze::StaticVector<double, 18UL> z;
    blaze::StaticVector<double, 19UL + numTendons> y_i;

    // Euler's method
    for (size_t i = 0UL; i < N - 1UL; ++i)
    {
        z = blaze::column(Z, i);
        y_i = blaze::column(Y, i);

        this->operator()(y_i, dyds, blaze::column(Z_h, i), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
        blaze::column(Z, i) = z;

        blaze::column(Y, i + 1UL) = y_i + (ds * dyds); // blaze::column(Y, i) + (ds * dyds);
    }

    // updating the last element of Z
    z = blaze::column<N - 1UL>(Z);
    y_i = blaze::column<N - 1UL>(Y);

    this->operator()(y_i, dyds, blaze::column<N - 1UL>(Z_h), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
    blaze::column<N - 1UL>(Z) = z;
}

// Integrates the ODE model equations using the classic 4th-order Runge-Kutta algorithm
template <std::size_t N, std::size_t numTendons>
void PDESystem<N, numTendons>::rungeKutta_PDE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticMatrix<double, 18UL, N> &Z_h, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    const double ds = static_cast<double>(L / (N - 1));
    const double half_ds = ds / 2.00;
    const double sixth_ds = ds / 6.00;

    // 4th-order Runge-Kutta
    blaze::StaticVector<double, 19UL + numTendons> y_i, y_half, k1, k2, k3, k4;
    blaze::StaticVector<double, 18UL> z, z_half, zh_half;

    for (size_t i = 0UL; i < N - 1UL; ++i)
    {
        // k1 = f(tn, yn)
        z = blaze::column(Z, i);
        y_i = blaze::column(Y, i);

        this->operator()(y_i, k1, blaze::column(Z_h, i), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
        blaze::column(Z, i) = z;

        // k2 = f(tn + 0.5h, yn + 0.5h * k1)
        y_half = y_i + k1 * half_ds;
        z_half = 0.50 * (z + blaze::column(Z, i + 1UL));
        zh_half = 0.50 * (blaze::column(Z_h, i) + blaze::column(Z_h, i + 1));
        this->operator()(y_half, k2, zh_half, z_half, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);

        // k3 = f(tn + 0.5h, yn + 0.5h * k2)
        y_half = y_i + k2 * half_ds;
        z_half = 0.50 * (z + blaze::column(Z, i + 1UL));
        zh_half = 0.50 * (blaze::column(Z_h, i) + blaze::column(Z_h, i + 1));
        this->operator()(y_half, k3, zh_half, z_half, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);

        // k4 = f(tn + h, yn + h * k3)
        y_half = y_i + k3 * ds;
        z_half = blaze::column(Z, i + 1UL);
        zh_half = blaze::column(Z_h, i + 1UL);
        this->operator()(y_half, k4, zh_half, z_half, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);

        blaze::column(Y, i + 1UL) = y_i + sixth_ds * (k1 + 2.00 * (k2 + k3) + k4);
    }

    // updating the last element of Z
    z = blaze::column<N - 1UL>(Z);
    y_i = blaze::column<N - 1UL>(Y);

    this->operator()(y_i, k1, blaze::column<N - 1UL>(Z_h), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
    blaze::column<N - 1UL>(Z) = z;
}

// Integrates the ODE model equations using the classic three step Adams Bashford algorithm
template <std::size_t N, std::size_t numTendons>
void PDESystem<N, numTendons>::adamsBashford_PDE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, const blaze::StaticMatrix<double, 18UL, N> &Z_h, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const double c0, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Bbt, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &C, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &J, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    const double ds = L / (N - 1);

    // 4th-order Runge-Kutta
    blaze::StaticVector<double, 19UL + numTendons> f1, f2, f3;
    blaze::StaticVector<double, 18UL> z;

    blaze::column<0UL>(Y) = dyds;

    for (size_t i = 0UL; i < N - 1; ++i)
    {
        z = blaze::column(Z, i);
        this->operator()(blaze::column(Y, i), f1, blaze::column(Z_h, i), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
        blaze::column(Z, i) = z;

        if (i == 1)
        {
            z = blaze::column(Z, i - 1);
            this->operator()(blaze::column(Y, i), f2, blaze::column(Z_h, i - 1), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
        }
        else if (i >= 2)
        {
            z = blaze::column(Z, i - 1);
            this->operator()(blaze::column(Y, i), f2, blaze::column(Z_h, i - 1), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);

            z = blaze::column(Z, i - 2);
            this->operator()(blaze::column(Y, i), f3, blaze::column(Z_h, i - 2), z, L, c0, Kse, Kbt, Bse, Bbt, C, J, r, tau, A, rho, g);
        }

        blaze::column(Y, i + 1UL) = blaze::column(Y, i) + (ds / 12.00) * (23.00 * f1 - 16.00 * f2 + 5.00 * f3);
    }
}
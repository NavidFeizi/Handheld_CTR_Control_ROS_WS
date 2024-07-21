#pragma once

#include <blaze/Math.h>
#include <cmath>
#include "MathOperations.hpp"

template <std::size_t N, std::size_t numTendons>
class ODESystem
{
public:
    // default constructor
    ODESystem();

    // overloaded constructor
    ODESystem(blaze::StaticVector<double, 3UL> u, blaze::StaticVector<double, 3UL> v, blaze::StaticVector<double, 4UL> h);

    // copy constructor
    ODESystem(const ODESystem &rhs);

    // move constructor
    ODESystem(ODESystem &&rhs) noexcept;

    // CTR destructor
    ~ODESystem() = default;

    // copy assignment operator
    ODESystem &operator=(const ODESystem &rhs);

    // move assignment operator
    ODESystem &operator=(ODESystem &&rhs) noexcept;

    // functor that implements the system of ODEs governing an N-tendon catheter
    void operator()(const blaze::StaticVector<double, 19UL + numTendons> &y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticVector<double, 18UL> &z, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

    // Integrates the ODE model equations using Euler's method
    void euler_ODE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

    // Integrates the ODE model equations using the classic 4th-order Runge-Kutta algorithm
    void rungeKutta_ODE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g);

private:
    blaze::StaticMatrix<double, 3UL, 3UL> m_R;   // rotation matrix SO(3) -- orientation of local frame wrt global frame
    blaze::StaticVector<double, 4UL> m_h;        // non-unity quaternions -- parametrizing the rotation
    blaze::StaticVector<double, 3UL> m_u, m_v;   // u: curvature vector | v: linear rate of change of position wrt arc-length
    blaze::StaticMatrix<double, 6UL, 6UL> m_Phi; // linear system matrix for determining vs, us
};

// default constructor
template <std::size_t N, std::size_t numTendons>
ODESystem<N, numTendons>::ODESystem()
{
    this->m_v = 0.00;
    this->m_u = 0.00;
    this->m_h = {1.00, 0.00, 0.00, 0.00};
    mathOp::getSO3(this->m_h, this->m_R);
    this->m_Phi = 0.00;
}

// overloaded constructor
template <std::size_t N, std::size_t numTendons>
ODESystem<N, numTendons>::ODESystem(blaze::StaticVector<double, 3UL> u, blaze::StaticVector<double, 3UL> v, blaze::StaticVector<double, 4UL> h) : m_v(v), m_u(u), m_h(h)
{
    mathOp::getSO3(m_h, m_R);
    this->m_Phi = 0.00;
}

// copy constructor
template <std::size_t N, std::size_t numTendons>
ODESystem<N, numTendons>::ODESystem(const ODESystem<N, numTendons> &rhs) : m_v(rhs.m_v), m_u(rhs.m_u), m_h(rhs.m_h), m_R(rhs.m_R), m_Phi(rhs.m_Phi) {}

// move constructor
template <std::size_t N, std::size_t numTendons>
ODESystem<N, numTendons>::ODESystem(ODESystem<N, numTendons> &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_v = std::move(rhs.m_v);
        this->m_u = std::move(rhs.m_u);
        this->m_h = std::move(rhs.m_h);
        this->m_R = std::move(rhs.m_R);
        this->m_Phi = std::move(rhs.m_Phi);
    }
}

// copy assignment operator
template <std::size_t N, std::size_t numTendons>
ODESystem<N, numTendons> &ODESystem<N, numTendons>::operator=(const ODESystem<N, numTendons> &rhs)
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_v = rhs.m_v;
        this->m_u = rhs.m_u;
        this->m_h = rhs.m_h;
        this->m_R = rhs.m_R;
        this->m_Phi = rhs.m_Phi;
    }

    return *this;
}

// move assignment operator
template <std::size_t N, std::size_t numTendons>
ODESystem<N, numTendons> &ODESystem<N, numTendons>::operator=(ODESystem<N, numTendons> &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_v = std::move(rhs.m_v);
        this->m_u = std::move(rhs.m_u);
        this->m_h = std::move(rhs.m_h);
        this->m_R = std::move(rhs.m_R);
        this->m_Phi = std::move(rhs.m_Phi);
    }

    return *this;
}

// functor that implements the system of ODEs governing an N-tendon catheter
template <std::size_t N, std::size_t numTendons>
void ODESystem<N, numTendons>::operator()(const blaze::StaticVector<double, 19UL + numTendons> &y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticVector<double, 18UL> &z, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    // dyds := [ps, hs, vs, us, qs, ws, numTendons] ==>> First spatial derivative of the (19+numTendons)-dimensional state vector
    this->m_v = blaze::subvector<7UL, 3UL>(y);
    this->m_u = blaze::subvector<10UL, 3UL>(y);
    this->m_h = blaze::subvector<3UL, 4UL>(y);
    mathOp::getSO3(m_h, m_R);

    // declaring auxiliar variables
    blaze::StaticVector<double, 3UL> a, a_i, b, pbs_i;
    blaze::StaticMatrix<double, 3UL, 3UL> A_i, G, G_i;
    blaze::StaticVector<double, numTendons> pbsNorm;

    auto Blk1 = blaze::submatrix<0UL, 0UL, 3UL, 3UL>(this->m_Phi); // block matrix corresponding to: Kse + A
    auto Blk2 = blaze::submatrix<3UL, 3UL, 3UL, 3UL>(this->m_Phi); // block matrix corresponding to: Kbt + H
    Blk1 = Kse;
    Blk2 = Kbt;

    // iterating through the tendons in the robot design
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

    blaze::StaticVector<double, 3UL> nb, mb, e3 = {0.00, 0.00, 1.00};
    nb = Kse * (this->m_v - e3);
    mb = Kbt * this->m_u;

    blaze::StaticVector<double, 6UL> rhs;
    blaze::subvector<0UL, 3UL>(rhs) = -blaze::cross(this->m_u, nb) - blaze::trans(this->m_R) * rho * A * g - a;
    blaze::subvector<3UL, 3UL>(rhs) = -blaze::cross(this->m_u, mb) - blaze::cross(this->m_v, nb) - b;

    // ODE state equations
    blaze::subvector<0UL, 3UL>(dyds) = this->m_R * this->m_v;                        // p_s
    blaze::subvector<3UL, 4UL>(dyds) = mathOp::quaternionDiff(this->m_u, this->m_h); // h_s
    blaze::subvector<7UL, 6UL>(dyds) = blaze::solve(this->m_Phi, rhs);               // v_s & u_s
    blaze::subvector<13UL, 6UL>(dyds) = 0.00;                                        // q_s & w_s (zero linear & angular velocities)
    blaze::subvector<19UL, numTendons>(dyds) = pbsNorm;                              // tendon lengths

    // outputting elements for variables with time derivatives
    blaze::subvector<0UL, 3UL>(z) = this->m_v;                         // v
    blaze::subvector<3UL, 3UL>(z) = this->m_u;                         // u
    blaze::subvector<6UL, 6UL>(z) = 0.00;                              // q = w = 0.0 (zero linear & angular velocities)
    blaze::subvector<12UL, 6UL>(z) = blaze::subvector<7UL, 6UL>(dyds); // v_s & u_s
}

// Integrates the ODE model equations using Euler's method
template <std::size_t N, std::size_t numTendons>
void ODESystem<N, numTendons>::euler_ODE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    const double ds = L / (N - 1);

    blaze::StaticVector<double, 18UL> z;
    blaze::StaticVector<double, 19UL + numTendons> y_i;

    // Euler's method
    for (size_t i = 0UL; i < N - 1UL; ++i)
    {
        z = blaze::column(Z, i);
        y_i = blaze::column(Y, i);

        this->operator()(y_i, dyds, z, Kse, Kbt, r, tau, A, rho, g);
        blaze::column(Z, i) = z;

        blaze::column(Y, i + 1UL) = y_i + (ds * dyds);
    }

    // updating the last element of Z
    z = blaze::column<N - 1UL>(Z);
    y_i = blaze::column<N - 1UL>(Y);

    this->operator()(y_i, dyds, z, Kse, Kbt, r, tau, A, rho, g);
    blaze::column<N - 1UL>(Z) = z;
}

// Integrates the ODE model equations using the classic 4th-order Runge-Kutta algorithm
template <std::size_t N, std::size_t numTendons>
void ODESystem<N, numTendons>::rungeKutta_ODE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
{
    const double ds = static_cast<double>(L / (N - 1));
    const double half_ds = ds / 2.00;
    const double sixth_ds = ds / 6.00;

    // 4th-order Runge-Kutta
    blaze::StaticVector<double, 19UL + numTendons> y_i, y_half, k1, k2, k3, k4;
    blaze::StaticVector<double, 18UL> z, z_half;

    for (size_t i = 0UL; i < N - 1UL; ++i)
    {
        // k1 = f(tn, yn)
        z = blaze::column(Z, i);
        y_i = blaze::column(Y, i);

        this->operator()(y_i, k1, z, Kse, Kbt, r, tau, A, rho, g);
        blaze::column(Z, i) = z;

        // k2 = f(tn + 0.5h, yn + 0.5h * k1)
        y_half = y_i + half_ds * k1;
        z_half = 0.50 * (z + blaze::column(Z, i + 1UL));
        this->operator()(y_half, k2, z_half, Kse, Kbt, r, tau, A, rho, g);

        // k3 = f(tn + 0.5h, yn + 0.5h * k2)
        y_half = y_i + half_ds * k2;
        z_half = 0.50 * (z + blaze::column(Z, i + 1UL));
        this->operator()(y_half, k3, z_half, Kse, Kbt, r, tau, A, rho, g);

        // k4 = f(tn + h, yn + h * k3)
        y_half = y_i + ds * k3;
        z_half = blaze::column(Z, i + 1);
        this->operator()(y_half, k4, z_half, Kse, Kbt, r, tau, A, rho, g);

        blaze::column(Y, i + 1UL) = y_i + sixth_ds * (k1 + 2.00 * (k2 + k3) + k4);
    }

    // updating the last element of Z
    z = blaze::column<N - 1UL>(Z);
    y_i = blaze::column<N - 1UL>(Y);

    this->operator()(y_i, k1, z, Kse, Kbt, r, tau, A, rho, g);
    blaze::column<N - 1UL>(Z) = z;
}

// // Integrates the ODE model equations using the classic 4th-order Runge-Kutta algorithm
// template <std::size_t N, std::size_t numTendons>
// void ODESystem<N, numTendons>::rungeKutta_ODE(blaze::StaticMatrix<double, 19UL + numTendons, N> &Y, blaze::StaticVector<double, 19UL + numTendons> &dyds, blaze::StaticMatrix<double, 18UL, N> &Z, const double L, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kse, const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> &Kbt, const std::array<blaze::StaticVector<double, 3UL>, numTendons> &r, const blaze::StaticVector<double, numTendons> &tau, double A, double rho, const blaze::StaticVector<double, 3UL> &g)
// {
//     const double ds = L / (N - 1);
//     const double half_ds = ds / 2.00;
//     const double sixth_ds = ds / 6.00;

//     // 4th-order Runge-Kutta
//     blaze::StaticVector<double, 19UL + numTendons> y_half, k1, k2, k3, k4;
//     blaze::StaticVector<double, 18UL> z, z_half;

//     blaze::column<0UL>(Y) = dyds;

//     for (size_t i = 0UL; i < N - 1; ++i)
//     {
//         // k1 = f(tn, yn)
//         z = blaze::column(Z, i);
//         this->operator()(blaze::column(Y, i), k1, z, Kse, Kbt, r, tau, A, rho, g);
//         blaze::column(Z, i) = z;

//         // k2 = f(tn + 0.5h, yn + 0.5h * k1)
//         y_half = blaze::column(Y, i) + k1 * half_ds;
//         z_half = 0.50 * (blaze::column(Z, i) + blaze::column(Z, i + 1));
//         this->operator()(y_half, k2, z_half, Kse, Kbt, r, tau, A, rho, g);

//         // k3 = f(tn + 0.5h, yn + 0.5h * k2)
//         y_half = blaze::column(Y, i) + k2 * half_ds;
//         this->operator()(y_half, k3, z_half, Kse, Kbt, r, tau, A, rho, g);

//         // k4 = f(tn + h, yn + h * k3)
//         y_half = blaze::column(Y, i) + k3 * ds;
//         z_half = blaze::column(Z, i + 1);
//         this->operator()(y_half, k4, z_half, Kse, Kbt, r, tau, A, rho, g);

//         blaze::column(Y, i + 1) = blaze::column(Y, i) + sixth_ds * (k1 + 2.00 * (k2 + k3) + k4);
//     }
// }

// mpc.hpp
#pragma once

#include <functional>
#include <optional>
#include <blaze/Math.h>
#include <blaze/Forward.h>
#include <iostream>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

// Helpers: Blaze -> Eigen conversions
template <size_t N>
inline void logBlazeVec(const blaze::StaticVector<double, N> &v, const std::string &name);

template <class BlazeMat>
inline Eigen::SparseMatrix<double> toEigenSparse(const BlazeMat &B);

template <class BlazeDense>
inline Eigen::MatrixXd toEigenDense(const BlazeDense &B);

template <class BlazeVec>
inline Eigen::VectorXd toEigenVec(const BlazeVec &v);

template <size_t h, size_t n, size_t m>
class MPC
{
public:
    using VecN = blaze::StaticVector<double, n>;
    using VecM = blaze::StaticVector<double, m>;
    using VecHN = blaze::StaticVector<double, h * n>;
    using VecHM = blaze::StaticVector<double, h * m>;
    using MatMN = blaze::StaticMatrix<double, m, n, blaze::columnMajor>;
    using MatHN = blaze::StaticMatrix<double, h, n, blaze::columnMajor>;
    using MatHM = blaze::StaticMatrix<double, h, m, blaze::columnMajor>;
    using SparseMat = blaze::CompressedMatrix<double, blaze::columnMajor>;

    // // Constructor
    // explicit MPC(double dt,
    //              std::function<VecM(const VecN &)> fwd, std::function<MatMN(const VecN &)> jac,
    //              std::optional<std::function<MatHM(const MatHN &)>> fwdB = std::nullopt,
    //              std::optional<std::function<SparseMat(const MatHN &)>> jacB = std::nullopt);

    /// Constructor with external force (wf) support
    explicit MPC(double dt,
                    std::function<VecM(const VecN &, const VecM &)> fwd, 
                    std::function<MatMN(const VecN &, const VecM &)> jac,
                    std::optional<std::function<MatHM(const MatHN &)>> fwdB = std::nullopt,
                    std::optional<std::function<SparseMat(const MatHN &)>> jacB = std::nullopt);

    // Disable copy to avoid accidental heavy copies (shared_ptr for module is cheap)
    MPC(const MPC &) = delete;
    MPC &operator=(const MPC &) = delete;

    // Move allowed
    // Moves deleted: the defaulted move constructor corrupted the OsqpEigen
    // solver state (freed garbage in the moved-from destructor). Hold the MPC
    // in a smart pointer if ownership transfer is needed.
    MPC(MPC &&) = delete;
    MPC &operator=(MPC &&) = delete;

    //--------------------------------- Class API ---------------------------------//
    /// @brief: Update cost function weights
    /// @param Q_diag: state weighting diagonal (m)
    /// @param Qf_diag: terminal state weighting diagonal (m)
    /// @param R_diag: control input weighting diagonal (n)
    /// @uasage: can be called anytime when control loop is executed.
    void updateWeights(VecM Q_diag, VecM Qf_diag, VecN R_u_diag, VecN R_du_diag);

    /// @brief: Set joint position and velocity, and velocity increment limits for QP with delta u as decision variables
    /// @param q_min: minimum joint positions (n) @param q_max: maximum joint positions (n)
    /// @param u_min: minimum control inputs (n) @param u_max: maximum control inputs (n)
    /// @param du_min: minimum control input changes (n) @param du_max: maximum control input changes (n)
    /// @param margin_q: safety margin @param margin_u: safety margin @param margin_du: safety margin
    /// @uasage: can be called anytime when control loop is executed.
    void setJointsLimits(const VecN &q_min, const VecN &q_max,
                         const VecN &u_min, const VecN &u_max,
                         const VecN &du_min, const VecN &du_max,
                         const VecN &margin_q, const VecN &margin_u, const VecN &margin_du);

    // /// @brief: MPC step
    // /// @param q0: current joint positions (n)
    // /// @param yref: reference trajectory over horizon (h x m)
    // /// @param u_apply: output control to apply at current step (n)
    // void step(const VecN &q0, const MatHM &yref, VecN &u_apply);

    /// @brief: MPC step
    /// @param q0: current joint positions (n)
    /// @param wf: external distal force estimate (3)
    /// @param yref: reference trajectory over horizon (h x m)
    /// @param u_apply: output control to apply at current step (n)
    /// @brief One MPC cycle. Returns false when the QP could not be solved or
    ///        produced a non-finite solution; in that case u_apply is set to zero
    ///        velocity, the warm-start state is left untouched, and the solver is
    ///        re-initialised on the next call. The caller is responsible for
    ///        logging (this library is ROS-free).
    bool step(const VecN &q0, const blaze::StaticVector<double, 3UL> &wf, const MatHM &yref, VecN &u_apply);

    /// Test hook: force the next step() through the full solver re-init path
    /// (used to verify warm-started and re-initialized solves agree).
    void forceSolverReinit() { m_solver_ready = false; }

private:
    // std::function<VecM(const VecN &)> m_fwd;
    // std::function<MatMN(const VecN &)> m_jac;
    std::function<VecM(const VecN &, const blaze::StaticVector<double, 3UL> &)> m_fwd;
    std::function<MatMN(const VecN &, const blaze::StaticVector<double, 3UL> &)> m_jac;
    std::optional<std::function<MatHM(const MatHN &)>> m_fwdB;     // nullsafe
    std::optional<std::function<SparseMat(const MatHN &)>> m_jacB; // nullsafe

    std::shared_ptr<OsqpEigen::Solver> m_solver;
    bool m_solver_ready = false;  // set after the one-time initSolver()
    bool m_reinit_warned = false; // warn only once if in-place updates fail

    double m_dt;
    VecM m_Q_diag, m_Qf_diag;
    VecN m_R_diag;
    SparseMat m_C_ineq_q, m_C_ineq_u;
    VecN m_q_max, m_q_min, m_u_max, m_u_min, m_du_max, m_du_min;
    VecHN m_q_max_h, m_q_min_h, m_u_max_h, m_u_min_h, m_du_max_h, m_du_min_h;

    blaze::StaticVector<double, 3 * h * n, blaze::columnVector> m_ub, m_lb;
    VecHN m_u_nom_stack;

    // Pre-allocated
    SparseMat m_H;                                // OSQP Hessian (h*n × h*n)
    VecHN m_g;                                    // OSQP gradient (h*n)
    SparseMat m_s;                                // State propagation matrix (h*n × h*n)
    SparseMat m_Q_blk;                            // (h*m) x (h*m)
    SparseMat m_R_u_blk, m_R_du_blk; // (h*n) x (h*n)
    Eigen::SparseMatrix<double> m_H_eigen;        // OSQP Hessian (h*n × h*n)
    Eigen::VectorXd m_g_eigen;                    // OSQP gradient (h*n)
    Eigen::VectorXd m_lb_eigen, m_ub_eigen;       // OSQP bounds (3*h*n)

    //---------------------------------Class Helper functions ---------------------------------//
    SparseMat buildS(double dt);

    void integrateQ(const VecHN &u, const VecN &q0, VecHN &q1);

    /// @brief: Update A matrix for linear inequality constraints lb_ineq < Ax < ub_ineq
    /// @usage: Must call after solver object created
    void setLinearConstraintMatrix(const blaze::StaticMatrix<double, n, n> &A_max_in);

    /// @brief: Update linear constraint bounds lb_ineq and ub_ineq to be used for OSQP solver (lb_ineq < Ax < ub_ineq)
    void updateLinearConstraintsBounds(const VecHN &q_nom_stack, const VecHN &u_nom_stack,
                                       blaze::StaticVector<double, 3 * h * n, blaze::columnVector> &ub_ineq,
                                       blaze::StaticVector<double, 3 * h * n, blaze::columnVector> &lb_ineq);

    // /// @brief: Compute Hessian and gradient for QP with delta u as decision variables
    // void updateQP(const VecHN &q_nom_stack, const VecHN &u_nom_stack, const VecHM &yref_stack, SparseMat &H, VecHN &g);

    /// @brief: Initialize nominal control input stack as zero
    void initUnomStack();

    // /// @brief: Forward kinematics
    // VecM forward(const VecN &q) const { return m_fwd(q); }

    // /// @brief: Jacobian matrix
    // MatMN jacobian(const VecN &q) const { return m_jac(q); }

    /// @brief: Compute Hessian and gradient for QP with delta u as decision variables (with wf)
    void updateQP(const VecHN &q_nom_stack, const VecHN &u_nom_stack, const VecHM &yref_stack, const blaze::StaticVector<double, 3UL> &wf, SparseMat &H, VecHN &g);

    /// @brief: Forward kinematics with external force
    VecM forward(const VecN &q, const blaze::StaticVector<double, 3UL> &wf) const { return m_fwd(q, wf); }

    /// @brief: Jacobian matrix with external force
    MatMN jacobian(const VecN &q, const blaze::StaticVector<double, 3UL> &wf) const { return m_jac(q, wf); }

};

#include "mpc.tpp"

// // Tell other TUs “don’t instantiate these, they exist elsewhere”
// extern template class MPC<10,6,3>;    // add the combos you use

// mpc.tpp
#pragma once

// template <size_t h, size_t n, size_t m>
// MPC<h, n, m>::MPC(double dt,
//                   std::function<VecM(const VecN &)> fwd,
//                   std::function<MatMN(const VecN &)> jac,
//                   std::optional<std::function<MatHM(const MatHN &)>> fwdB,
//                   std::optional<std::function<SparseMat(const MatHN &)>> jacB)
//     : m_dt(dt),
//       m_fwd(std::move(fwd)),
//       m_jac(std::move(jac)),
//       m_fwdB(std::move(fwdB)),
//       m_jacB(std::move(jacB)),
//       m_s(h * n, h * n)
// {
//     std::cout << "Initializing MPC..." << std::endl;

//     // Pre-allocate block diagonal Q and R matrices
//     m_Q_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * m, h * m);
//     m_Q_blk.reserve(h * m); // max nnz
//     m_R_u_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
//     m_R_u_blk.reserve(h * n);
//     m_R_du_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
//     m_R_du_blk.reserve(h * n);
//     // m_R_ddu_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
//     // m_R_ddu_blk.reserve(h * n);
//     m_C_ineq_q = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
//     m_C_ineq_q.reserve(h * n * n); // max nnz
//     m_C_ineq_u = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
//     m_C_ineq_u.reserve(h * n * n);

//     // Pre-allocate Hessian, gradient and limita
//     size_t nnz_R = h * n * n;
//     size_t nnz_JS = h * n * (n + m);   // upper bound
//     m_H.resize(h * n, h * n);          // Set size
//     m_H.reserve(nnz_R + nnz_JS);       // Pre-allocate
//     m_H_eigen.resize(h * n, h * n);    // Set size
//     m_H_eigen.reserve(nnz_R + nnz_JS); // Pre-allocate CSR buffers
//     m_g_eigen.resize(h * n);
//     m_lb_eigen.resize(3 * h * n);
//     m_ub_eigen.resize(3 * h * n);
//     m_H_eigen.setZero();
//     m_g_eigen.setZero();
//     m_ub_eigen.setOnes();
//     m_lb_eigen = -m_ub_eigen;

//     // Build S matrix for state propagation
//     m_s = buildS(dt);

//     blaze::StaticMatrix<double, n, n> A_small(0.0);
//     if (n == 6)
//     {
//         A_small = blaze::StaticMatrix<double, n, n>{
//             {1, -1, 0, 0, 0, 0}, // x1 - x2
//             {0, 1, -1, 0, 0, 0}, // x2 - x3
//             {0, 0, 1, 0, 0, 0},  // x3
//             {0, 0, 0, 1, -1, 0}, // x4 - x5
//             {0, 0, 0, 0, 1, -1}, // x5 - x6
//             {0, 0, 0, 0, 0, 1},  // x6
//         };
//     }
//     else if (n == 4)
//     {
//         A_small = blaze::StaticMatrix<double, n, n>{
//             {1, -1, 0, 0}, // x1 - x2
//             {0, 1, 0, 0}, // x2 - x3
//             {0, 0, 1, -1},  // x3
//             {0, 0, 0, 1},  // x4
//         };
//     }
//     else
//     {
//         throw std::runtime_error("MPC currently only supports n=4 or n=6.");
//     }

//     // Initialize OSQP solver

//     m_solver = std::make_shared<OsqpEigen::Solver>();
//     m_solver->settings()->setWarmStart(true);
//     m_solver->settings()->setMaxIteration(2000);
//     m_solver->settings()->setAbsoluteTolerance(1e-4);
//     m_solver->settings()->setRelativeTolerance(1e-4);
//     m_solver->settings()->setVerbosity(false);
//     m_solver->data()->setNumberOfVariables(h * n);
//     m_solver->data()->setNumberOfConstraints(3 * h * n);
//     std::cout << "OSQP solver Initialized" << std::endl;

//     setLinearConstraintMatrix(A_small); // must call after solver created
//     initUnomStack();

//     if (m_fwdB)
//     {
//         std::cout << "Using batched forward call function for MPC." << std::endl;
//     }
//     else
//     {
//         std::cout << "Using non-batched forward call function for MPC." << std::endl;
//     }

//     if (m_jacB)
//     {
//         std::cout << "Using batched jacobian call function for MPC." << std::endl;
//     }
//     else
//     {
//         std::cout << "Using non-batched jacobian call function for MPC." << std::endl;
//     }

//     // m_solver->data()->setHessianMatrix(m_H_eigen);
//     // m_solver->data()->setGradient(m_g_eigen);
//     // m_solver->data()->setLowerBound(m_lb_eigen);
//     // m_solver->data()->setUpperBound(m_ub_eigen);
//     // m_solver->initSolver();
//     // if (m_solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
//     //     std::cout << "Failed to solve QP!" << std::endl;
//     // std::cout << "MPC solver initialized" << std::endl;
// }

template <size_t h, size_t n, size_t m>
MPC<h, n, m>::MPC(double dt,
                    std::function<VecM(const VecN &, const VecM &)> fwd,
                    std::function<MatMN(const VecN &, const VecM &)> jac,
                    std::optional<std::function<MatHM(const MatHN &)>> fwdB,
                    std::optional<std::function<SparseMat(const MatHN &)>> jacB)
    : m_dt(dt),
        m_fwd(std::move(fwd)),
        m_jac(std::move(jac)),
        m_fwdB(std::move(fwdB)),
        m_jacB(std::move(jacB)),
        m_s(h * n, h * n)
{
    std::cout << "Initializing MPC with wf support..." << std::endl;
    m_Q_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * m, h * m);
    m_Q_blk.reserve(h * m);
    m_R_u_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
    m_R_u_blk.reserve(h * n);
    m_R_du_blk = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
    m_R_du_blk.reserve(h * n);
    m_C_ineq_q = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
    m_C_ineq_q.reserve(h * n * n);
    m_C_ineq_u = blaze::CompressedMatrix<double, blaze::columnMajor>(h * n, h * n);
    m_C_ineq_u.reserve(h * n * n);
    size_t nnz_R = h * n * n;
    size_t nnz_JS = h * n * (n + m);
    m_H.resize(h * n, h * n);
    m_H.reserve(nnz_R + nnz_JS);
    m_H_eigen.resize(h * n, h * n);
    m_H_eigen.reserve(nnz_R + nnz_JS);
    m_g_eigen.resize(h * n);
    m_lb_eigen.resize(3 * h * n);
    m_ub_eigen.resize(3 * h * n);
    m_H_eigen.setZero();
    m_g_eigen.setZero();
    m_ub_eigen.setOnes();
    m_lb_eigen = -m_ub_eigen;
    m_s = buildS(dt);
    blaze::StaticMatrix<double, n, n> A_small(0.0);
    if (n == 6)
    {
        A_small = blaze::StaticMatrix<double, n, n>{{1, -1, 0, 0, 0, 0}, {0, 1, -1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, -1, 0}, {0, 0, 0, 0, 1, -1}, {0, 0, 0, 0, 0, 1}};
    }
    else if (n == 4)
    {
        A_small = blaze::StaticMatrix<double, n, n>{{1, -1, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, -1}, {0, 0, 0, 1}};
    }
    else
    {
        throw std::runtime_error("MPC currently only supports n=4 or n=6.");
    }
    m_solver = std::make_shared<OsqpEigen::Solver>();
    m_solver->settings()->setWarmStart(true);
    m_solver->settings()->setMaxIteration(2000);
    m_solver->settings()->setAbsoluteTolerance(1e-4);
    m_solver->settings()->setRelativeTolerance(1e-4);
    m_solver->settings()->setVerbosity(false);
    m_solver->data()->setNumberOfVariables(h * n);
    m_solver->data()->setNumberOfConstraints(3 * h * n);
    std::cout << "OSQP solver Initialized" << std::endl;
    setLinearConstraintMatrix(A_small);
    initUnomStack();
    if (m_fwdB)
        std::cout << "Using batched forward call function for MPC." << std::endl;
    else
        std::cout << "Using non-batched forward call function for MPC." << std::endl;
    if (m_jacB)
        std::cout << "Using batched jacobian call function for MPC." << std::endl;
    else
        std::cout << "Using non-batched jacobian call function for MPC." << std::endl;
}

//---------------------------------Class API ---------------------------------//
template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::setJointsLimits(const VecN &q_min, const VecN &q_max, const VecN &u_min, const VecN &u_max, const VecN &u_dot_min, const VecN &u_dot_max, const VecN &margin_q, const VecN &margin_u, const VecN &margin_du)
{
    m_q_max = q_max;
    m_q_min = q_min;
    m_u_max = u_max;
    m_u_min = u_min;
    m_du_max = u_dot_max * m_dt;
    m_du_min = u_dot_min * m_dt;

    const double eps = 1e-4;

    VecN q_max_eff = q_max - margin_q;
    VecN q_min_eff = q_min + margin_q;
    for (size_t j = 0; j < n; ++j)
    {
        if (q_max_eff[j] <= q_min_eff[j])
        {
            q_max_eff[j] = (q_max_eff[j] + q_min_eff[j]) / 2.0 + eps;
            q_min_eff[j] = (q_max_eff[j] + q_min_eff[j]) / 2.0 - eps;
            std::cout << "Warning: Adjusted joint " << j << " position constraints to avoid infeasibility." << std::endl;
        }
    }

    VecN u_max_eff = u_max - margin_u;
    VecN u_min_eff = u_min + margin_u;
    for (size_t j = 0; j < n; ++j)
    {
        if (u_max_eff[j] <= u_min_eff[j])
        {
            u_max_eff[j] = (u_max_eff[j] + u_min_eff[j]) / 2.0 + eps;
            u_min_eff[j] = (u_max_eff[j] + u_min_eff[j]) / 2.0 - eps;
            std::cout << "Warning: Adjusted joint " << j << " velocity constraints to avoid infeasibility." << std::endl;
        }
    }

    VecN du_max_eff = m_du_max - margin_du;
    VecN du_min_eff = m_du_min + margin_du;
    for (size_t j = 0; j < n; ++j)
    {
        if (du_max_eff[j] <= du_min_eff[j])
        {
            du_max_eff[j] = (du_max_eff[j] + du_min_eff[j]) / 2.0 + eps;
            du_min_eff[j] = (du_max_eff[j] + du_min_eff[j]) / 2.0 - eps;
            std::cout << "Warning: Adjusted joint " << j << " acceleration constraints to avoid infeasibility. du_max_eff=" << du_max_eff[j] << ", du_min_eff=" << du_min_eff[j] << std::endl;
        }
    }

    for (size_t k = 0; k < h; ++k)
    {
        subvector(m_q_max_h, k * n, n) = q_max_eff;
        subvector(m_q_min_h, k * n, n) = q_min_eff;
        subvector(m_u_max_h, k * n, n) = u_max_eff;
        subvector(m_u_min_h, k * n, n) = u_min_eff;
        subvector(m_du_max_h, k * n, n) = du_max_eff;
        subvector(m_du_min_h, k * n, n) = du_min_eff;
    }

    // Print the constraint limits for debugging
    std::cout << "q limits:" << std::endl;
    std::cout << "    ub: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << q_max[i];
        if (i < n - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "    lb: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << q_min[i];
        if (i < n - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "u limits:" << std::endl;
    std::cout << "    ub: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << u_max[i];
        if (i < n - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "    lb: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << u_min[i];
        if (i < n - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "u_dot limits:" << std::endl;
    std::cout << "    ub: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << u_dot_max[i];
        if (i < n - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "    lb: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << u_dot_min[i];
        if (i < n - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << std::endl;
}

template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::updateWeights(VecM Q_diag, VecM Qf_diag, VecN R_u_diag, VecN R_du_diag)
{
    const double dt = m_dt;

    // --- Update Q ---
    for (size_t i = 0; i < h; ++i)
    {
        const double val = (i < h - 1) ? Q_diag[i % m] * dt : Qf_diag[i % m] * dt;
        for (size_t j = 0; j < m; ++j)
        {
            const size_t idx = i * m + j;
            m_Q_blk(idx, idx) = val;
        }
    }

    // --- Update R u ---
    for (size_t i = 0; i < h; ++i)
    {
        for (size_t j = 0; j < n; ++j)
        {
            const size_t idx = i * n + j;
            m_R_u_blk(idx, idx) = R_u_diag[j] * dt;
        }
    }

    // --- Update R du ---
    for (size_t i = 0; i < h; ++i)
    {
        for (size_t j = 0; j < n; ++j)
        {
            const size_t idx = i * n + j;
            m_R_du_blk(idx, idx) = R_du_diag[j] * dt;
        }
    }

    // // --- Update R ddu ---
    // for (size_t i = 0; i < h; ++i)
    // {
    //     for (size_t j = 0; j < n; ++j)
    //     {
    //         const size_t idx = i * n + j;
    //         m_R_ddu_blk(idx, idx) = R_ddu_diag[j] * dt;
    //     }
    // }

    std::cout << "Updated Weights:" << std::endl;
    std::cout << "Q diagonal: [";
    for (size_t i = 0; i < m; ++i)
    {
        std::cout << Q_diag[i];
        if (i + 1 < m)
            std::cout << ", ";      
    }
    std::cout << "]" << std::endl;
    std::cout << "R_u diagonal: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << R_u_diag[i];
        if (i + 1 < n)
            std::cout << ", ";      
    }
    std::cout << "]" << std::endl;
    std::cout << "R_du diagonal: [";
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << R_du_diag[i];
        if (i + 1 < n)
            std::cout << ", ";      
    }
    std::cout << "]" << std::endl;
    // std::cout << "R_ddu diagonal: [";
    // for (size_t i = 0; i < n; ++i)
    // {
    //     std::cout << R_ddu_diag[i];
    //     if (i + 1 < n)
    //         std::cout << ", ";      
    // }
    // std::cout << "]" << std::endl;
}

// template <size_t h, size_t n, size_t m>
// void MPC<h, n, m>::step(const VecN &q0, const MatHM &yref, VecN &u_apply)
// {
//     const size_t nh = h * n;

//     // flatten yref
//     VecHM yref_stack;
//     for (size_t j = 0; j < h; ++j)
//         subvector(yref_stack, j * m, m) = blaze::StaticVector<double, m>({yref(j, 0), yref(j, 1), yref(j, 2)});
//     blaze::StaticVector<double, nh> q_nom_stack;
//     integrateQ(m_u_nom_stack, q0, q_nom_stack);
//     updateQP(q_nom_stack, m_u_nom_stack, yref_stack, m_H, m_g);
//     updateLinearConstraintsBounds(q_nom_stack, m_u_nom_stack, m_ub, m_lb);

//     for (size_t i = 0; i < 3 * n * h; ++i)
//     {
//         assert(m_lb[i] <= m_ub[i]);
//     }
//     m_H_eigen = toEigenSparse(m_H); // Hessian
//     m_g_eigen = toEigenVec(m_g);                // OSQP uses +qᵀ x
//     m_ub_eigen = toEigenVec(m_ub);
//     m_lb_eigen = toEigenVec(m_lb);

//     // std::cout << "\nlb_eigen: " << m_lb_eigen.head(std::min(20, (int)m_lb_eigen.size())).transpose() << std::endl;
//     // std::cout << "ub_eigen: " << m_ub_eigen.head(std::min(20, (int)m_ub_eigen.size())).transpose() << std::endl;
//     // better to re-init to update the matrices every time step since the sparsity pattern of H_eigen may change
//     m_solver->clearSolver();
//     m_solver->data()->clearHessianMatrix();
//     m_solver->data()->setHessianMatrix(m_H_eigen);
//     m_solver->data()->setGradient(m_g_eigen);
//     m_solver->data()->setLowerBound(m_lb_eigen);
//     m_solver->data()->setUpperBound(m_ub_eigen);
//     m_solver->initSolver();

//     if (m_solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
//     {
//         std::cout << "Failed to solve QP!" << std::endl;
//     }
//     Eigen::VectorXd dU = m_solver->getSolution();

//     // Convert Eigen::VectorXd to blaze::StaticVector
//     blaze::StaticVector<double, nh> du_stack;
//     for (size_t i = 0; i < nh; ++i)
//     {
//         size_t joint_idx = i % n;
//         du_stack[i] = std::clamp(dU(i), m_du_min[joint_idx], m_du_max[joint_idx]);
//         // double original = dU(i);
//         // if (du_stack[i] != original) {
//         //     std::cout << "du clamping at joint " << i << ": "
//         //               << original << " -> " << du_stack[i] << std::endl;
//         // }
//     }

//     m_u_nom_stack += du_stack;

//     // Clamp m_u_nom_stack to velocity constraints
//     for (size_t i = 0; i < nh; ++i)
//     {
//         // double original = m_u_nom_stack[i];
//         size_t joint_idx = i % n;
//         m_u_nom_stack[i] = std::clamp(m_u_nom_stack[i], m_u_min[joint_idx], m_u_max[joint_idx]);
//         // if (m_u_nom_stack[i] != original) {
//         //     std::cout << "u clamping at joint " << i << ": "
//         //               << original << " -> " << m_u_nom_stack[i] << std::endl;
//         // }
//     }

//     u_apply = subvector(m_u_nom_stack, 0, n);

//     subvector(m_u_nom_stack, 0, n * (h - 1)) = subvector(m_u_nom_stack, n, n * (h - 1)); // shift left
// }

template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::step(const VecN &q0, const blaze::StaticVector<double, 3UL> &wf, const MatHM &yref, VecN &u_apply)
{
    const size_t nh = h * n;
    VecHM yref_stack;
    for (size_t j = 0; j < h; ++j)
        subvector(yref_stack, j * m, m) = blaze::StaticVector<double, m>({yref(j, 0), yref(j, 1), yref(j, 2)});
    blaze::StaticVector<double, nh> q_nom_stack;
    integrateQ(m_u_nom_stack, q0, q_nom_stack);
    updateQP(q_nom_stack, m_u_nom_stack, yref_stack, wf, m_H, m_g);
    updateLinearConstraintsBounds(q_nom_stack, m_u_nom_stack, m_ub, m_lb);
    for (size_t i = 0; i < 3 * n * h; ++i)
    {
        assert(m_lb[i] <= m_ub[i]);
    }
    m_H_eigen = toEigenSparse(m_H);
    m_g_eigen = toEigenVec(m_g);
    m_ub_eigen = toEigenVec(m_ub);
    m_lb_eigen = toEigenVec(m_lb);
    // Warm start: initialize the solver once, then update matrices/vectors in
    // place. The Hessian's sparsity pattern is structurally identical every
    // cycle (same QP construction), which updateHessianMatrix requires; if an
    // update ever fails (e.g. pattern change from exact-zero entries), fall
    // back to a full re-init and warn once. The old code cleared and re-inited
    // the solver every cycle, defeating setWarmStart(true) entirely.
    bool solver_ok = true;
    if (!m_solver_ready)
    {
        m_solver->data()->clearHessianMatrix();
        solver_ok = m_solver->data()->setHessianMatrix(m_H_eigen) &&
                    m_solver->data()->setGradient(m_g_eigen) &&
                    m_solver->data()->setLowerBound(m_lb_eigen) &&
                    m_solver->data()->setUpperBound(m_ub_eigen) &&
                    m_solver->initSolver();
        m_solver_ready = solver_ok;
    }
    else
    {
        solver_ok = m_solver->updateHessianMatrix(m_H_eigen) &&
                    m_solver->updateGradient(m_g_eigen) &&
                    m_solver->updateBounds(m_lb_eigen, m_ub_eigen);
        if (!solver_ok)
        {
            if (!m_reinit_warned)
            {
                std::cout << "OSQP in-place update failed - falling back to solver re-init" << std::endl;
                m_reinit_warned = true;
            }
            m_solver->clearSolver();
            m_solver->data()->clearHessianMatrix();
            solver_ok = m_solver->data()->setHessianMatrix(m_H_eigen) &&
                        m_solver->data()->setGradient(m_g_eigen) &&
                        m_solver->data()->setLowerBound(m_lb_eigen) &&
                        m_solver->data()->setUpperBound(m_ub_eigen) &&
                        m_solver->initSolver();
        }
    }
    if (!solver_ok || m_solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cout << "Failed to solve QP!" << std::endl;
    }
    Eigen::VectorXd dU = m_solver->getSolution();
    blaze::StaticVector<double, nh> du_stack;
    for (size_t i = 0; i < nh; ++i)
    {
        size_t joint_idx = i % n;
        du_stack[i] = std::clamp(dU(i), m_du_min[joint_idx], m_du_max[joint_idx]);
    }
    m_u_nom_stack += du_stack;
    for (size_t i = 0; i < nh; ++i)
    {
        size_t joint_idx = i % n;
        m_u_nom_stack[i] = std::clamp(m_u_nom_stack[i], m_u_min[joint_idx], m_u_max[joint_idx]);
    }
    u_apply = subvector(m_u_nom_stack, 0, n);
    subvector(m_u_nom_stack, 0, n * (h - 1)) = subvector(m_u_nom_stack, n, n * (h - 1));
}

//---------------------------------Class Helper functions ---------------------------------//
template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::setLinearConstraintMatrix(const blaze::StaticMatrix<double, n, n> &A_max_in)
{
    const size_t nh = h * n;
    blaze::CompressedMatrix<double, blaze::columnMajor> Ac(3 * nh, nh);
    Ac.reserve(3 * nh * n);

    // ======= position constraints part =======
    // ---- C_q = kron(I_h, A_ineq)  → block diagonal (n*h x n*h)
    blaze::StaticMatrix<double, nh, nh, blaze::columnMajor> C_q(0.0);
    for (size_t k = 0; k < h; ++k)
        submatrix(C_q, k * n, k * n, n, n) = A_max_in;
    m_C_ineq_q = C_q;

    // ======= velocity constraints part =======
    // C_u = I_{hn}  → select ΔU directly (no accumulation)
    blaze::StaticMatrix<double, nh, nh, blaze::columnMajor> C_u(0.0);
    for (size_t j = 0; j < nh; ++j)
        C_u(j, j) = 1.0;
    m_C_ineq_u = C_u;

    // ======= dU constraints part and merge =======
    submatrix(Ac, 0, 0, nh, nh) = m_C_ineq_q * m_s;
    submatrix(Ac, 1 * nh, 0, nh, nh) = m_C_ineq_u;
    submatrix(Ac, 2 * nh, 0, nh, nh) = blaze::IdentityMatrix<double>(nh);

    const Eigen::SparseMatrix<double> Ac_eigen = toEigenSparse(Ac);
    if (!m_solver->data()->setLinearConstraintsMatrix(Ac_eigen))
        std::cout << "Failed to set linear constraints matrix!" << std::endl;
}

// template <size_t h, size_t n, size_t m>
// void MPC<h, n, m>::updateQP(const VecHN &q_nom_stack, const VecHN &u_nom_stack, const VecHM &yref_stack, SparseMat &H, VecHN &g)
// {
//     blaze::StaticMatrix<double, m, n> Jk; // jacobian for a single horizon
//     blaze::CompressedMatrix<double, blaze::columnMajor> A;
//     VecHM r; // residual
//     VecHM r_tilde;
//     VecHM y_stack;

//     // allocate and build J_block & y_stack
//     blaze::CompressedMatrix<double, blaze::columnMajor> J_block(h * m, h * n);
//     J_block.reserve(h * m * n);
//     // for (size_t k = 0; k < h; ++k)
//     //     for (size_t c = 0; c < n; ++c)
//     //         J_block.reserve(k * n + c, static_cast<size_t>(m)); // per-column capacity
    
    
//     if (m_fwdB) // batched version
//     {
//         blaze::StaticMatrix<double, h, n, blaze::rowMajor> qB;
//         for (size_t k = 0; k < h; ++k)
//         {
//             // row(qB, k) = trans(subvector(q_nom_stack, k * n, n));
//             if (n == 6) {
//                 row(qB, k) = {q_nom_stack[k * n + 0], q_nom_stack[k * n + 1], q_nom_stack[k * n + 2],
//                            q_nom_stack[k * n + 3], q_nom_stack[k * n + 4], q_nom_stack[k * n + 5]};
//             }
//             else if (n == 4) {
//                 row(qB, k) = {q_nom_stack[k * n + 0], q_nom_stack[k * n + 1],
//                            q_nom_stack[k * n + 2], q_nom_stack[k * n + 3]};
//             }
//         }
//         blaze::StaticMatrix<double, h, m, blaze::rowMajor> yB = (*m_fwdB)(qB);
//         for (size_t k = 0; k < h; ++k)
//             subvector(y_stack, k * m, m) = trans(row(yB, k));
//     }
//     else // non-batched version (default)
//     {
//         for (size_t k = 0; k < h; ++k)
//             subvector(y_stack, k * m, m) = forward(subvector(q_nom_stack, k * n, n));
//     }

    
//     if (m_jacB) // batched version
//     {
//         blaze::StaticMatrix<double, h, n, blaze::rowMajor> qB;
//         for (size_t k = 0; k < h; ++k)
//         {
//             // row(qB, k) = trans(subvector(q_nom_stack, k * n, n));
//             if (n == 6)
//             {
//                 row(qB, k) = {q_nom_stack[k * n + 0], q_nom_stack[k * n + 1], q_nom_stack[k * n + 2],
//                            q_nom_stack[k * n + 3], q_nom_stack[k * n + 4], q_nom_stack[k * n + 5]};
//             }
//             else if (n == 4)
//             {
//                 row(qB, k) = {q_nom_stack[k * n + 0], q_nom_stack[k * n + 1],
//                            q_nom_stack[k * n + 2], q_nom_stack[k * n + 3]};
//             }
            
//         }
//         J_block = (*m_jacB)(qB);
//     }
//     else // non-batched version (default)
//     {
//         for (size_t k = 0; k < h; ++k)
//         {
//             Jk = jacobian(subvector(q_nom_stack, k * n, n));
//             submatrix(J_block, k * m, k * n, m, n) = Jk;
//         }
//     }

//     A = J_block * m_s;
//     r = yref_stack - y_stack;
//     r_tilde = r - A * u_nom_stack;
//     H = 1 * (trans(A) * m_Q_blk * A + m_R_u_blk + m_R_du_blk); // to penalize u and du
//     g = -1 * (trans(A) * m_Q_blk * r_tilde - m_R_u_blk * u_nom_stack); // to penalize u 

//     // std::cout << "m_R_u_blk stored entries: "<< m_Q_blk.nonZeros() << std::endl;
//     // std::cout << "m_Q_blk stored entries: "<< m_Q_blk.nonZeros() << std::endl;
//     // std::cout << "J_block stored entries: "<< J_block.nonZeros() << std::endl;
//     // std::cout << "m_s stored entries: "<< m_s.nonZeros() << std::endl;
//     // std::cout << "A stored entries: "<< A.nonZeros() << std::endl;
//     // std::cout << "H stored entries: "<< H.nonZeros() << std::endl;

//     // H += 1e-4 * blaze::IdentityMatrix<double>(h * n);
//     // H = 0.5 * (H + trans(H));

//     //// logs
//     // logBalzeVec(y_stack, "y_stack");
//     // std::cout << "S \n" << std::fixed << std::setprecision(4) << m_s << std::endl;
//     // std::cout << "J_block \n" << std::fixed << std::setprecision(4) << J_block << std::endl;
//     // std::cout << "JS \n" << std::fixed << std::setprecision(4) << JS << std::endl;
//     // std::cout << "m_R_u_blk \n" << std::fixed << std::setprecision(6) << m_R_u_blk << std::endl;
//     // std::cout << "H \n" << std::fixed << std::setprecision(4) << H << std::endl;
//     // logBalzeVec(r, "r");
//     // logBalzeVec(g, "g");
// }

template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::updateQP(const VecHN &q_nom_stack, const VecHN &u_nom_stack, const VecHM &yref_stack, const blaze::StaticVector<double, 3UL> &wf, SparseMat &H, VecHN &g)
{
    blaze::StaticMatrix<double, m, n> Jk;
    blaze::CompressedMatrix<double, blaze::columnMajor> A;
    VecHM r;
    VecHM r_tilde;
    VecHM y_stack;
    blaze::CompressedMatrix<double, blaze::columnMajor> J_block(h * m, h * n);
    J_block.reserve(h * m * n);

    for (size_t k = 0; k < h; ++k)
    {
        subvector(y_stack, k * m, m) = forward(subvector(q_nom_stack, k * n, n), wf);
    }

    for (size_t k = 0; k < h; ++k)
    {
        Jk = jacobian(subvector(q_nom_stack, k * n, n), wf);
        submatrix(J_block, k * m, k * n, m, n) = Jk;
    }

    A = J_block * m_s;
    r = yref_stack - y_stack;
    r_tilde = r - A * u_nom_stack;
    H = 1 * (trans(A) * m_Q_blk * A + m_R_u_blk + m_R_du_blk);
    g = -1 * (trans(A) * m_Q_blk * r_tilde - m_R_u_blk * u_nom_stack);
}

template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::integrateQ(const VecHN &u_nom_stack, const VecN &q0, VecHN &q_nom_stack)
{
    subvector(q_nom_stack, 0, n) = q0 + m_dt * subvector(u_nom_stack, 0, n);
    for (size_t i = 1; i < h; ++i)
        subvector(q_nom_stack, i * n, n) = subvector(q_nom_stack, (i - 1) * n, n) + m_dt * subvector(u_nom_stack, i * n, n);
}

template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::updateLinearConstraintsBounds(const VecHN &q_nom_stack, const VecHN &u_nom_stack, blaze::StaticVector<double, 3 * h * n> &ub_ineq, blaze::StaticVector<double, 3 * h * n> &lb_ineq)
{
    const size_t hn = h * n;
    const VecHN temp1 = m_C_ineq_q * q_nom_stack;
    const VecHN temp2 = m_C_ineq_u * u_nom_stack;

    subvector(ub_ineq, 0, hn) = m_q_max_h - temp1;
    subvector(ub_ineq, hn, hn) = m_u_max_h - temp2;
    subvector(ub_ineq, 2 * hn, hn) = m_du_max_h;

    subvector(lb_ineq, 0, hn) = m_q_min_h - temp1;
    subvector(lb_ineq, hn, hn) = m_u_min_h - temp2;
    subvector(lb_ineq, 2 * hn, hn) = m_du_min_h;

    for (size_t i = 0; i < 3 * hn; ++i)
    {
        if (lb_ineq[i] >= ub_ineq[i])
        {
            std::cout << "Warning: Infeasible constraint bounds at index " << i << ": lb = " << lb_ineq[i] << ", ub = " << ub_ineq[i] << std::endl;
            double mid = 0.5 * (lb_ineq[i] + ub_ineq[i]);
            lb_ineq[i] = mid - 1e-4;
            ub_ineq[i] = mid + 1e-4;
        }
    }
}

template <size_t h, size_t n, size_t m>
void MPC<h, n, m>::initUnomStack()
{
    m_u_nom_stack = VecHN(0.0);
}

template <size_t h, size_t n, size_t m>
blaze::CompressedMatrix<double, blaze::columnMajor> MPC<h, n, m>::buildS(double dt)
{
    const size_t hn = h * n;
    blaze::CompressedMatrix<double, blaze::columnMajor> S(hn, hn);

    const size_t nnz = n * (h * (h + 1)) / 2;
    S.reserve(nnz);

    for (size_t k = 0; k < h; ++k)
    {
        for (size_t j = 0; j < n; ++j)
        {
            const size_t col = k * n + j;
            const size_t nz = h - k;
            S.reserve(col, nz); // per-column capacity
            for (size_t i = k; i < h; ++i)
            {                                 // lower-triangular
                const size_t row = i * n + j; // <-- stride is n
                S.append(row, col, dt);       // dt * I_n in (i,j) block
            }
            S.finalize(col);
        }
    }
    return S;
}

//--------------------------------- Helper functions ---------------------------------//
template <size_t N>
void logBlazeVec(const blaze::StaticVector<double, N> &v, const std::string &name)
{
    std::cout << name << " = [";
    for (size_t i = 0; i < v.size(); ++i)
    {
        std::cout << std::fixed << std::setprecision(6) << v[i];
        if (i < v.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

template <class BlazeMat>
Eigen::SparseMatrix<double> toEigenSparse(const BlazeMat &B)
{
    // Works if B is columnMajor compressed; otherwise iterate nonzeros
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(B.nonZeros());
    for (size_t j = 0; j < B.columns(); ++j)
    {
        for (auto it = B.begin(j); it != B.end(j); ++it)
        {
            trips.emplace_back(static_cast<int>(it->index()),
                               static_cast<int>(j),
                               it->value());
        }
    }
    Eigen::SparseMatrix<double> E(B.rows(), B.columns());
    E.setFromTriplets(trips.begin(), trips.end());
    E.makeCompressed();
    return E;
}

template <class BlazeDense>
Eigen::MatrixXd toEigenDense(const BlazeDense &B)
{
    Eigen::MatrixXd E(B.rows(), B.columns());
    for (size_t i = 0; i < B.rows(); ++i)
        for (size_t j = 0; j < B.columns(); ++j)
            E(i, j) = B(i, j);
    return E;
}

template <class BlazeVec>
Eigen::VectorXd toEigenVec(const BlazeVec &v)
{
    Eigen::VectorXd e(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        e(i) = v[i];
    return e;
}


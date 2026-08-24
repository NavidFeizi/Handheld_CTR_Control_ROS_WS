// IK convergence benchmark for PINNs::posCTRL.
//
// Why this exists: posCTRL is the single solve that produces the planner's goal
// configuration. If it misses, no plan can be requested at all -- so its
// convergence rate is a hard gate on the application, yet nothing measured it.
//
// Methodology (this is the part that makes the numbers mean something):
// targets are FORWARD-SAMPLED. We draw a feasible joint vector q_true, run FK,
// and use the resulting tip as the target. A solution therefore provably exists,
// so every miss is a solver failure rather than an unreachable target. Compare
// with sampling targets in Cartesian space, where a miss is ambiguous.
//
// Every solve is seeded from the same realistic start pose the robot actually
// homes to, so the reported distance buckets correspond to what the operator
// experiences as "a near target" vs "a far target".
//
// Build:
//   colcon build --packages-select ctr_kinematics_pinn \
//     --cmake-args -DCMAKE_BUILD_TYPE=Release -DCTR_PINN_BUILD_BENCH=ON
// Run:
//   ik_bench --models-dir <dir> [--model <name>] [--n 200] [--tol 1e-3] [--csv out.csv]

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr size_t kControlInputs = 4UL;  // handheld CTR: [b1, b2, a1, a2]
constexpr size_t kBackbonePoints = 150UL;
constexpr size_t kBatch = 1UL;

using JointVec = blaze::StaticVector<double, kControlInputs>;
using PosVec = blaze::StaticVector<double, 3UL>;

/// One solve's record.
struct Trial
{
    double seedDistance = 0.00;  // ||target - tip(seed)||, the "how far" axis
    double residual = 0.00;
    double seconds = 0.00;
    JointVec qOut{};       // returned configuration, for the violation breakdown
    IkDiagnostics diag{};
    bool plannerWouldAccept = false;  // passes satisfiesBounds AND isValid
    bool boundsOk = false;
    bool validOk = false;
};

/// Mirror of ompl RealVectorStateSpace::satisfiesBounds for the planner's space.
/// The planner sets its bounds directly from getInputPosBounds() for all four
/// indices (Planner.hpp: m_coordBound->setLow/setHigh 0..3), so this is exact.
bool satisfiesBounds(const JointVec &q, const JointVec &lb, const JointVec &ub)
{
    for (size_t i = 0UL; i < kControlInputs; ++i)
        if (q[i] < lb[i] || q[i] > ub[i])
            return false;
    return true;
}

/// Mirror of CTR_StateValidityChecker<4>::isValid. Kept deliberately verbatim
/// (including what it does NOT check) so the benchmark measures the planner's
/// real acceptance rule rather than an idealised one.
bool plannerIsValid(const JointVec &q, const JointVec &lb, const JointVec &ub, const double clr)
{
    const double beta1 = q[0UL], beta2 = q[1UL], alpha1 = q[2UL], alpha2 = q[3UL];

    const bool tb1 = (beta1 >= lb[0UL]) && (beta1 <= std::min(ub[0UL], beta2 - clr));
    const bool tb2 = (beta2 >= std::max(beta1 + clr, lb[1UL])) && (beta2 <= ub[1UL]);
    const bool ang = std::fabs(alpha2 - alpha1) <= M_PI;

    return tb1 && tb2 && ang;
}

/// The prismatic window posCTRL itself enforces, including the tube-protrusion
/// terms the planner's checker omits. Used to draw feasible samples.
void betaWindow(const JointVec &q, const JointVec &lb, const JointVec &ub,
                const PosVec &L, const double clr,
                double &b1Min, double &b1Max, double &b2Min, double &b2Max)
{
    b1Min = std::max({lb[0UL], L[1UL] + q[1UL] - L[0UL], L[2UL] - L[0UL]});
    b1Max = std::min(ub[0UL], q[1UL] - clr);
    b2Min = std::max({lb[1UL], q[0UL] + clr, L[2UL] - L[1UL]});
    // NOTE: ub[1] is included here but NOT in posCTRL, which caps at -clr only.
    // That difference is finding F1; the sampler must stay on the strict side so
    // every generated target is provably reachable by a planner-legal state.
    b2Max = std::min({ub[1UL], -clr, L[0UL] + q[0UL] - L[1UL]});
}

double percentile(std::vector<double> v, const double p)
{
    if (v.empty())
        return 0.00;
    std::sort(v.begin(), v.end());
    const size_t idx = static_cast<size_t>(p * static_cast<double>(v.size() - 1UL) + 0.5);
    return v[std::min(idx, v.size() - 1UL)];
}

std::string argValue(int argc, char **argv, const std::string &flag, const std::string &fallback)
{
    for (int i = 1; i + 1 < argc; ++i)
        if (flag == argv[i])
            return argv[i + 1];
    return fallback;
}

}  // namespace

int main(int argc, char **argv)
{
    const char *envModels = std::getenv("CTR_MODELS_DIR");
    const std::string modelsDir = argValue(argc, argv, "--models-dir", envModels ? envModels : "");
    const std::string modelName = argValue(argc, argv, "--model", "ctr_8x91_0.18_tanh_9K_9K_50K_FP64");
    const size_t nTrials = std::stoul(argValue(argc, argv, "--n", "200"));
    const double posTol = std::stod(argValue(argc, argv, "--tol", "1.0e-3"));
    const std::string csvPath = argValue(argc, argv, "--csv", "");
    const uint32_t rngSeed = static_cast<uint32_t>(std::stoul(argValue(argc, argv, "--seed", "12345")));

    if (modelsDir.empty())
    {
        std::cerr << "ik_bench: need --models-dir <dir> (or CTR_MODELS_DIR).\n"
                  << "  e.g. --models-dir /ws/src/ctr_kinematics_pinn/models\n";
        return 2;
    }

    PINNs<kControlInputs> pinn(modelsDir, modelName, kBatch, kBackbonePoints);

    const PosVec L = pinn.getOverallLen();
    const double clr = pinn.getStageThickness();
    const auto bounds = pinn.getInputPosBounds();
    const JointVec lb = std::get<0UL>(bounds);
    const JointVec ub = std::get<1UL>(bounds);
    const PosVec wf(0.00);  // benchmark the unloaded model; force is orthogonal to convergence

    // robot_node's k_pos_preEngage is {a1, b1, a2, b2} = {0, -0.0640, 0, -0.0340}
    // in WIRE order; posCTRL takes PHYSICS order [b1, b2, a1, a2]. This pose sits
    // exactly on ub[0] and ub[1] -- both ends of a real planning problem are on
    // the feasibility boundary.
    const JointVec qSeed{-0.0640, -0.0340, 0.00, 0.00};

    PosVec pSeed;
    pinn.getPosDistal(qSeed, wf, pSeed);

    std::cout << "ik_bench\n"
              << "  model      : " << modelName << "\n"
              << "  trials     : " << nTrials << "\n"
              << "  posTol     : " << posTol * 1.0E3 << " mm\n"
              << "  seed pose  : [" << qSeed[0] << ", " << qSeed[1] << ", " << qSeed[2] << ", " << qSeed[3] << "]\n"
              << "  seed tip   : [" << pSeed[0] << ", " << pSeed[1] << ", " << pSeed[2] << "]\n"
              << "  b1 bounds  : [" << lb[0] << ", " << ub[0] << "]   b2 bounds: [" << lb[1] << ", " << ub[1] << "]\n"
              << "  clearance  : " << clr << "\n"
              << std::endl;

    std::mt19937 rng(rngSeed);
    std::uniform_real_distribution<double> u01(0.00, 1.00);

    std::vector<Trial> trials;
    trials.reserve(nTrials);

    const auto tStart = std::chrono::steady_clock::now();

    // Draw one planner-legal joint vector, or report failure. Bounded so a
    // pathological bounds configuration cannot spin forever.
    const auto sampleFeasible = [&](JointVec &qOut) -> bool
    {
        for (size_t attempt = 0UL; attempt < 1000UL; ++attempt)
        {
            JointVec qTrue(0.00);
            // beta2 first (beta1's window depends on it), then beta1 inside it.
            qTrue[1UL] = lb[1UL] + u01(rng) * (ub[1UL] - lb[1UL]);
            double b1Min, b1Max, b2Min, b2Max;
            betaWindow(qTrue, lb, ub, L, clr, b1Min, b1Max, b2Min, b2Max);
            if (!(b1Max > b1Min))
                continue;  // degenerate window for this beta2; redraw

            qTrue[0UL] = b1Min + u01(rng) * (b1Max - b1Min);
            qTrue[2UL] = -M_PI + u01(rng) * 2.00 * M_PI;              // a1 in [-pi, pi]
            qTrue[3UL] = qTrue[2UL] - M_PI + u01(rng) * 2.00 * M_PI;  // a2 anchored to a1

            // Reject anything the planner itself would not accept: a target the
            // planner could never legally reach is not a fair test of the solver.
            if (!plannerIsValid(qTrue, lb, ub, clr) || !satisfiesBounds(qTrue, lb, ub))
                continue;

            qOut = qTrue;
            return true;
        }
        return false;
    };

    for (size_t i = 0UL; i < nTrials; ++i)
    {
        JointVec qTrue(0.00);
        if (!sampleFeasible(qTrue))
        {
            std::cerr << "ik_bench: could not draw a feasible joint vector in 1000 attempts; "
                         "check the model's dataset ranges.\n";
            return 3;
        }

        PosVec target;
        pinn.getPosDistal(qTrue, wf, target);

        // ---- solve from the fixed realistic seed ----
        JointVec q = qSeed;
        Trial t;
        t.seedDistance = blaze::norm(target - pSeed);

        const auto t0 = std::chrono::steady_clock::now();
        pinn.posCTRL(q, target, posTol, wf, &t.diag);
        const auto t1 = std::chrono::steady_clock::now();
        t.seconds = std::chrono::duration<double>(t1 - t0).count();

        PosVec tip;
        pinn.getPosDistal(q, wf, tip);
        t.residual = blaze::norm(target - tip);

        t.qOut = q;
        t.boundsOk = satisfiesBounds(q, lb, ub);
        t.validOk = plannerIsValid(q, lb, ub, clr);
        t.plannerWouldAccept = t.boundsOk && t.validOk;

        trials.push_back(t);

        if (((i + 1UL) % 25UL) == 0UL)
            std::cout << "  ... " << (i + 1UL) << "/" << nTrials << " ("
                      << std::fixed << std::setprecision(1)
                      << std::chrono::duration<double>(t1 - tStart).count() << " s)" << std::endl;
    }

    // ------------------------------- report -------------------------------
    // Buckets in mm of seed->target distance. "Far" is defined by the data, not
    // by assertion: the reachable workspace is only tens of mm across.
    const std::vector<double> edges{0.00, 20.0, 40.0, 60.0, 80.0, 1.0E9};

    std::cout << "\n=== convergence vs seed->target distance ===\n"
              << "  tol = " << posTol * 1.0E3 << " mm (planner), gate = 3.000 mm (manager)\n\n";
    std::cout << std::left << std::setw(12) << "dist[mm]"
              << std::right << std::setw(6) << "n"
              << std::setw(9) << "<tol"
              << std::setw(9) << "<3mm"
              << std::setw(11) << "resid_p50"
              << std::setw(11) << "resid_p90"
              << std::setw(11) << "resid_max"
              << std::setw(8) << "iters"
              << std::setw(8) << "restart"
              << std::setw(9) << "t_p50"
              << std::setw(9) << "t_max"
              << std::setw(9) << "plan_ok"
              << "\n";

    auto pct = [](size_t num, size_t den) { return den ? (100.0 * static_cast<double>(num) / static_cast<double>(den)) : 0.0; };

    for (size_t b = 0UL; b + 1UL < edges.size(); ++b)
    {
        std::vector<double> resid, secs;
        size_t n = 0UL, okTol = 0UL, ok3 = 0UL, planOk = 0UL, iters = 0UL, restarts = 0UL;

        for (const auto &t : trials)
        {
            const double d = t.seedDistance * 1.0E3;
            if (d < edges[b] || d >= edges[b + 1UL])
                continue;
            ++n;
            resid.push_back(t.residual * 1.0E3);
            secs.push_back(t.seconds);
            if (t.residual <= posTol) ++okTol;
            if (t.residual < 3.0E-3) ++ok3;
            if (t.plannerWouldAccept) ++planOk;
            iters += t.diag.iterations;
            restarts += t.diag.restarts;
        }
        if (n == 0UL)
            continue;

        std::ostringstream label;
        if (edges[b + 1UL] > 1.0E8) label << edges[b] << "+";
        else label << edges[b] << "-" << edges[b + 1UL];

        std::cout << std::left << std::setw(12) << label.str() << std::right
                  << std::setw(6) << n
                  << std::setw(8) << std::fixed << std::setprecision(1) << pct(okTol, n) << "%"
                  << std::setw(8) << pct(ok3, n) << "%"
                  << std::setw(11) << std::setprecision(3) << percentile(resid, 0.50)
                  << std::setw(11) << percentile(resid, 0.90)
                  << std::setw(11) << percentile(resid, 1.00)
                  << std::setw(8) << std::setprecision(0) << static_cast<double>(iters) / static_cast<double>(n)
                  << std::setw(8) << std::setprecision(2) << static_cast<double>(restarts) / static_cast<double>(n)
                  << std::setw(9) << std::setprecision(2) << percentile(secs, 0.50)
                  << std::setw(9) << percentile(secs, 1.00)
                  << std::setw(8) << std::setprecision(1) << pct(planOk, n) << "%"
                  << "\n";
    }

    // ---- diagnostics that decide between the competing failure hypotheses ----
    size_t nFail = 0UL, failNonMono = 0UL, failClamped = 0UL, budgetExhausted = 0UL;
    size_t rejected = 0UL, rejectedButConverged = 0UL, boundsFail = 0UL, validFail = 0UL;
    double worstJinv = 0.00, worstJinvFail = 0.00;
    std::vector<double> nonMonoFrac, clampFrac;

    for (const auto &t : trials)
    {
        worstJinv = std::max(worstJinv, t.diag.maxJinvNorm);
        if (t.diag.iterations)
        {
            nonMonoFrac.push_back(100.0 * static_cast<double>(t.diag.nonMonotonicSteps) / static_cast<double>(t.diag.iterations));
            clampFrac.push_back(100.0 * static_cast<double>(t.diag.clampedSteps) / static_cast<double>(t.diag.iterations));
        }
        if (t.residual > posTol)
        {
            ++nFail;
            if (t.diag.nonMonotonicSteps) ++failNonMono;
            if (t.diag.clampedSteps) ++failClamped;
            if (t.diag.iterations >= 3000UL) ++budgetExhausted;
            worstJinvFail = std::max(worstJinvFail, t.diag.maxJinvNorm);
        }
        if (!t.plannerWouldAccept)
        {
            ++rejected;
            if (t.residual <= posTol) ++rejectedButConverged;
            if (!t.boundsOk) ++boundsFail;
            if (!t.validOk) ++validFail;
        }
    }

    std::cout << "\n=== failure-mode diagnostics (n = " << trials.size() << ") ===\n"
              << std::fixed << std::setprecision(2);
    std::cout << "  misses (resid > tol)              : " << nFail << " (" << pct(nFail, trials.size()) << "%)\n"
              << "  ... that ever went uphill         : " << failNonMono << " (" << pct(failNonMono, nFail) << "% of misses)  [H4]\n"
              << "  ... that ever hit a joint clamp   : " << failClamped << " (" << pct(failClamped, nFail) << "% of misses)  [H6]\n"
              << "  ... that exhausted the 3000 budget: " << budgetExhausted << " (" << pct(budgetExhausted, nFail) << "% of misses)\n"
              << "  uphill steps, median share of run : " << percentile(nonMonoFrac, 0.50) << "%          [H4]\n"
              << "  clamped steps, median share of run: " << percentile(clampFrac, 0.50) << "%          [H6]\n"
              << "  max ||J^+||_F over all trials     : " << std::scientific << worstJinv << "  [H3]\n"
              << "  max ||J^+||_F among misses        : " << worstJinvFail << "  [H3]\n"
              << std::fixed;
    std::cout << "\n  PLANNER REJECTION (setGoalState would throw):\n"
              << "  returned q rejected              : " << rejected << " (" << pct(rejected, trials.size()) << "%)\n"
              << "  ... despite converging           : " << rejectedButConverged << "   [F1: converged but unplannable]\n"
              << "  ... failing satisfiesBounds      : " << boundsFail << "\n"
              << "  ... failing isValid              : " << validFail << "\n";

    // Which specific constraint does the returned configuration violate? This is
    // what distinguishes F1 (posCTRL's beta2 cap is looser than the planner's)
    // from a generic "solution is out of bounds".
    size_t vB2Hi = 0UL, vB1Hi = 0UL, vB2Lo = 0UL, vB1Lo = 0UL, vOrder = 0UL, vAngle = 0UL;
    double worstB2Excess = 0.00;
    for (const auto &t : trials)
    {
        if (t.plannerWouldAccept)
            continue;
        const auto &q = t.qOut;
        if (q[1UL] > ub[1UL]) { ++vB2Hi; worstB2Excess = std::max(worstB2Excess, q[1UL] - ub[1UL]); }
        if (q[0UL] > ub[0UL]) ++vB1Hi;
        if (q[1UL] < lb[1UL]) ++vB2Lo;
        if (q[0UL] < lb[0UL]) ++vB1Lo;
        if (q[0UL] > q[1UL] - clr) ++vOrder;
        if (std::fabs(q[3UL] - q[2UL]) > M_PI) ++vAngle;
    }
    std::cout << "\n  WHICH CONSTRAINT (of the " << rejected << " rejected):\n"
              << std::setprecision(4)
              << "  beta2 > ub[1] (" << ub[1UL] << ")        : " << vB2Hi
              << "   <-- F1: posCTRL caps beta2 at -clr = " << -clr << ", the planner at " << ub[1UL] << "\n"
              << "  beta1 > ub[0] (" << ub[0UL] << ")        : " << vB1Hi << "\n"
              << "  beta2 < lb[1]                     : " << vB2Lo << "\n"
              << "  beta1 < lb[0]                     : " << vB1Lo << "\n"
              << "  beta1 > beta2 - clr (ordering)    : " << vOrder << "\n"
              << "  |a2 - a1| > pi                    : " << vAngle << "\n"
              << std::setprecision(3)
              << "  worst beta2 overshoot past ub[1]  : " << worstB2Excess * 1.0E3 << " mm\n";

    if (!csvPath.empty())
    {
        std::ofstream csv(csvPath);
        csv << "seed_dist_m,residual_m,seconds,iterations,restarts,clamped_steps,non_monotonic_steps,"
               "max_jinv_norm,initial_error_m,converged,bounds_ok,valid_ok\n";
        csv << std::setprecision(9);
        for (const auto &t : trials)
            csv << t.seedDistance << ',' << t.residual << ',' << t.seconds << ','
                << t.diag.iterations << ',' << t.diag.restarts << ',' << t.diag.clampedSteps << ','
                << t.diag.nonMonotonicSteps << ',' << t.diag.maxJinvNorm << ',' << t.diag.initialError << ','
                << (t.diag.converged ? 1 : 0) << ',' << (t.boundsOk ? 1 : 0) << ',' << (t.validOk ? 1 : 0) << '\n';
        std::cout << "\nwrote " << csvPath << std::endl;
    }

    return 0;
}

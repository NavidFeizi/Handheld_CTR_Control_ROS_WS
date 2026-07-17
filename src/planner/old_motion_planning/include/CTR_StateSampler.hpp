#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSampler.h>

#include <memory>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "PINNs.hpp"
#include "CTR_StateValidityChecker.hpp"

// 2-tube CTR state sampler (Tube 1 innermost, Tube 2 intermediate).
// Tube 3 (outermost) is static and excluded from the planning state.
//
// State layout: [β₁, β₂, α₁, α₂]  (values[0..3])
template <size_t controlInputs>
class CTR_StateSampler : public ompl::base::StateSampler
{
public:
    CTR_StateSampler(const ompl::base::StateSpace *space,
                     CTR_StateValidityChecker<controlInputs> &validityChecker,
                     PINNs<controlInputs> &ctr);

    ~CTR_StateSampler() override = default;

    void sampleUniform(ompl::base::State *state) override;
    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

    // Set per-tube prismatic minimums and stage clearance
    void setParameters(double b1, double b2, double clearance);

    // Set goal revolute angles for rotation-focused sampling bias
    void setGoalAngles(double a1, double a2);

protected:
    ompl::RNG rng_;

private:
    PINNs<controlInputs> &m_ctr;
    CTR_StateValidityChecker<controlInputs> &m_validityChecker;

    double m_beta1_Min, m_beta2_Min;
    double m_alpha1_Min, m_alpha1_max;
    double m_alpha2_Min, m_alpha2_max;
    double m_Clr;

    // Goal revolute angles (for rotation-focused sampling)
    double m_alpha1_goal{0.00}, m_alpha2_goal{0.00};
};

// ============================= Implementation =============================

template <size_t controlInputs>
CTR_StateSampler<controlInputs>::CTR_StateSampler(const ompl::base::StateSpace *space,
                                                   CTR_StateValidityChecker<controlInputs> &validityChecker,
                                                   PINNs<controlInputs> &ctr)
    : ompl::base::StateSampler(space), m_validityChecker(validityChecker), m_ctr(ctr)
{
    m_Clr = this->m_ctr.getStageThickness();

    // Use getInputPosBounds() which properly handles the 4-DoF layout:
    //   lb/ub[0] = beta1,  lb/ub[1] = beta2,
    //   lb/ub[2] = alpha1, lb/ub[3] = alpha2
    const auto [lb, ub] = this->m_ctr.getInputPosBounds();

    m_beta1_Min = lb[0UL];
    m_beta2_Min = lb[1UL];

    m_alpha1_Min = lb[2UL];
    m_alpha1_max = ub[2UL];

    m_alpha2_Min = lb[3UL];
    m_alpha2_max = ub[3UL];
}

template <size_t controlInputs>
void CTR_StateSampler<controlInputs>::sampleUniform(ompl::base::State *state)
{
    ompl::base::RealVectorStateSpace::StateType *sampled_State =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Goal prismatic values from the validity checker
    const double beta1_goal = this->m_validityChecker.getBeta1Goal();
    const double beta2_goal = this->m_validityChecker.getBeta2Goal();

    // ================= Prismatic joint sampling =================
    //
    // Two phases weighted by probability:
    //   Phase 1 (95%): Both β₁ and β₂ evolve toward their goal values.
    //   Phase 2 (5%):  β₂ fixed at goal; β₁ varies freely toward its goal.
    //
    // Ordering constraint β₁ < β₂ (with clearance) is enforced per tube.

    const double u = rng_.uniformReal(0.00, 1.00);
    const size_t phase = (u < 0.95) ? 1 : 2;

    double beta_1, beta_2;

    switch (phase)
    {
    case 1: // Both betas evolve independently toward goals
        beta_1 = rng_.uniformReal(m_beta1_Min, beta1_goal);
        beta_2 = rng_.uniformReal(std::max(m_beta2_Min, beta_1 + m_Clr), beta2_goal);
        break;
    default: // Phase 2: β₂ fixed at goal, β₁ varies
        beta_2 = beta2_goal;
        beta_1 = rng_.uniformReal(m_beta1_Min, beta1_goal);
        break;
    }

    sampled_State->values[0UL] = beta_1;
    sampled_State->values[1UL] = beta_2;

    // ================= Revolute joint sampling =================
    // Compute normalised prismatic progress (average of the two tubes).
    double progressBeta = 0.00;
    if (beta1_goal > m_beta1_Min && beta2_goal > m_beta2_Min)
    {
        const double f1 = (beta_1 - m_beta1_Min) / std::max(1.00E-9, beta1_goal - m_beta1_Min);
        const double f2 = (beta_2 - m_beta2_Min) / std::max(1.00E-9, beta2_goal - m_beta2_Min);
        progressBeta = std::clamp((f1 + f2) / 2.00, 0.00, 1.00);
    }

    // Bias revolute sampling toward goal angles using two complementary modes:
    //   revolveFirst (25%): revolute joints near goal regardless of prismatic
    //     progress. These waypoints enable "rotate-before-deploy" paths.
    //   rotationFocus (33% of remaining, only after 15% deployment): fine-
    //     grained goal-biased revolute sampling once insertion has started.
    const bool goalsSet = (std::fabs(m_alpha1_goal) > 1.00E-9 ||
                           std::fabs(m_alpha2_goal) > 1.00E-9);
    const bool revolveFirst  = goalsSet && (rng_.uniformReal(0.00, 1.00) < 0.25);
    const bool rotationFocus = !revolveFirst && goalsSet &&
                               (progressBeta > 0.15) &&
                               (rng_.uniformReal(0.00, 1.00) < 0.3333);
    constexpr double rotStd = 0.174533; // 10 degrees standard deviation

    if (revolveFirst || rotationFocus)
    {
        auto wrapClamp = [](double v, double lo, double hi) -> double
        {
            while (v >  M_PI) v -= 2.0 * M_PI;
            while (v < -M_PI) v += 2.0 * M_PI;
            return std::min(std::max(v, lo), hi);
        };
        sampled_State->values[2UL] = wrapClamp(rng_.gaussian(m_alpha1_goal, rotStd), m_alpha1_Min, m_alpha1_max);
        sampled_State->values[3UL] = wrapClamp(rng_.gaussian(m_alpha2_goal, rotStd), m_alpha2_Min, m_alpha2_max);
    }
    else
    {
        sampled_State->values[2UL] = rng_.uniformReal(m_alpha1_Min, m_alpha1_max);
        sampled_State->values[3UL] = rng_.uniformReal(m_alpha2_Min, m_alpha2_max);
    }
}

template <size_t controlInputs>
void CTR_StateSampler<controlInputs>::sampleUniformNear(ompl::base::State *state,
                                                         const ompl::base::State *near,
                                                         double distance)
{
    auto       *q  = state->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *qn = near->as<ompl::base::RealVectorStateSpace::StateType>();

    const double b0n = qn->values[0], b1n = qn->values[1];

    const double beta1_goal = this->m_validityChecker.getBeta1Goal();
    const double beta2_goal = this->m_validityChecker.getBeta2Goal();

    // Valid interval for each tube – per-tube clamping preserves tube identity.
    const double lo0 = m_beta1_Min,                            hi0 = std::min(beta1_goal, b1n - m_Clr);
    const double lo1 = std::max(m_beta2_Min, b0n + m_Clr),    hi1 = beta2_goal;

    auto clampPerturb = [&](double center, double lo, double hi) -> double
    {
        if (lo > hi) return (lo + hi) * 0.5;
        const double val = center + rng_.uniformReal(-distance, distance);
        return std::min(std::max(val, lo), hi);
    };

    q->values[0] = clampPerturb(b0n, lo0, hi0);
    q->values[1] = clampPerturb(b1n, lo1, hi1);

    // Revolute joints (indices 2-3): shortest-arc perturbation, clamped to bounds.
    const double alphaMin[2] = {m_alpha1_Min, m_alpha2_Min};
    const double alphaMax[2] = {m_alpha1_max, m_alpha2_max};
    for (size_t i = 2; i < 4; ++i)
    {
        double val = qn->values[i] + rng_.uniformReal(-distance, distance);
        while (val >  M_PI) val -= 2.0 * M_PI;
        while (val < -M_PI) val += 2.0 * M_PI;
        val = std::min(std::max(val, alphaMin[i - 2]), alphaMax[i - 2]);
        q->values[i] = val;
    }
}

template <size_t controlInputs>
void CTR_StateSampler<controlInputs>::sampleGaussian(ompl::base::State *state,
                                                      const ompl::base::State *mean,
                                                      double stdDev)
{
    auto *q  = state->as<ompl::base::RealVectorStateSpace::StateType>();
    auto *qm = mean->as<ompl::base::RealVectorStateSpace::StateType>();

    const double beta1_goal = m_validityChecker.getBeta1Goal();
    const double beta2_goal = m_validityChecker.getBeta2Goal();

    const double b0m = qm->values[0], b1m = qm->values[1];
    const double lo0 = m_beta1_Min,                           hi0 = std::min(beta1_goal, b1m - m_Clr);
    const double lo1 = std::max(m_beta2_Min, b0m + m_Clr),   hi1 = beta2_goal;

    auto clampGauss = [&](double center, double lo, double hi) -> double
    {
        if (lo > hi) return (lo + hi) * 0.5;
        return std::min(std::max(center + rng_.gaussian(0.0, stdDev), lo), hi);
    };

    q->values[0] = clampGauss(b0m, lo0, hi0);
    q->values[1] = clampGauss(b1m, lo1, hi1);

    // Revolute joints (indices 2-3): Gaussian perturbation wrapped to [-π, π].
    const double alphaMin[2] = {m_alpha1_Min, m_alpha2_Min};
    const double alphaMax[2] = {m_alpha1_max, m_alpha2_max};
    for (size_t i = 2; i < controlInputs; ++i)
    {
        double val = qm->values[i] + rng_.gaussian(0.0, stdDev);
        while (val >  M_PI) val -= 2.0 * M_PI;
        while (val < -M_PI) val += 2.0 * M_PI;
        val = std::min(std::max(val, alphaMin[i - 2]), alphaMax[i - 2]);
        q->values[i] = val;
    }
}

template <size_t controlInputs>
void CTR_StateSampler<controlInputs>::setParameters(double b1, double b2, double clearance)
{
    this->m_beta1_Min = b1;
    this->m_beta2_Min = b2;
    this->m_Clr = clearance;
}

template <size_t controlInputs>
void CTR_StateSampler<controlInputs>::setGoalAngles(double a1, double a2)
{
    this->m_alpha1_goal = a1;
    this->m_alpha2_goal = a2;
}


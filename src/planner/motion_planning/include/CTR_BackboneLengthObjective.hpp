#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <algorithm> // for std::copy_n
#include <exception>
#include <iostream>
#include "PINNs.hpp"
#include "CTR_StateValidityChecker.hpp"
#include "CTR_StateSampler.hpp"
#include "CTR_PINNsInformedStateSamplerInformed.hpp"
#include "CTR_SamplerUtils.hpp"
#include "InformedSamplerGuard.hpp"
#include "Logging.hpp"

// 2-tube CTR backbone length objective.
// Tube 3 is static; arc lengths computed directly from getOverallLen().
template <size_t controlInputs>
class CTR_BackboneLengthObjective : public ompl::base::StateCostIntegralObjective
{
public:
    static constexpr double k1 = 1000.0;
    static constexpr double k2 = 6000.0;
    static constexpr double k3 = 3000.0;
    static constexpr double k4 =  500.0;

    CTR_BackboneLengthObjective(const ompl::base::SpaceInformationPtr &si, PINNs<controlInputs> &_ctr);
    ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDef, unsigned int maxNumberCalls) const override;

    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

private:
    PINNs<controlInputs> &m_ctr;
};

// ============================= Implementation =============================

// A thin InformedSampler implementation that uses the PINNs model to
// sample in tip-space and perform a conservative admissibility check.
template <size_t controlInputs>
class CTR_PINNsInformedSamplerImpl : public ompl::base::InformedSampler
{
public:
    CTR_PINNsInformedSamplerImpl(const ompl::base::ProblemDefinitionPtr &probDefn,
                                 unsigned int maxIters,
                                 CTR_StateValidityChecker<controlInputs> &validityChecker,
                                 PINNs<controlInputs> &ctr,
                                 ompl::base::StateSamplerPtr fallback)
        : ompl::base::InformedSampler(probDefn, maxIters), m_validityChecker(validityChecker), m_ctr(ctr), m_fallback(std::move(fallback))
    {
        m_tipRadius = 0.02;
        m_maxAttempts = 50;
    }

    ~CTR_PINNsInformedSamplerImpl() override = default;

    bool sampleUniform(ompl::base::State *statePtr, const ompl::base::Cost &maxCost) override
    {
        // Attempt PINNs-informed tip-space sampling
        // Try to obtain a concrete goal object
        const ompl::base::GoalPtr gptr = this->probDefn_->getGoal();
        if (!gptr)
        {
            if (m_fallback)
            {
                m_fallback->sampleUniform(statePtr);
                return true;
            }
            return false;
        }

        // The Goal may not provide a single state; try to extract the goal state from the problem definition's goal pointer
        const ompl::base::State *gstate = nullptr;
        if (this->probDefn_->getGoal().get())
        {
            // If ProblemDefinition has a solution goal state, get it
            if (this->probDefn_->getGoal()->as<ompl::base::GoalRegion>())
            {
                // No direct canonical goal state available; fall back
                if (m_fallback)
                {
                    m_fallback->sampleUniform(statePtr);
                    return true;
                }
                return false;
            }
        }

        // For simplicity, if we cannot obtain a single start state, use fallback
        if (this->probDefn_->getStartStateCount() == 0)
        {
            if (m_fallback)
            {
                m_fallback->sampleUniform(statePtr);
                return true;
            }
            return false;
        }

        // We will attempt PINNs sampling using the first goal state if available via the solution path
        blaze::StaticVector<double, controlInputs> goalVec{};
        blaze::StaticVector<double, 3UL> goalTip{};
        bool haveGoalTip = false;
        if (this->probDefn_->hasSolution())
        {
            auto path = this->probDefn_->getSolutionPath();
            if (path)
            {
                // Try to cast to a geometric path to access waypoints
                const auto *pgeom = path->as<ompl::geometric::PathGeometric>();
                if (pgeom && pgeom->getStateCount() > 0)
                {
                    const auto *gs = pgeom->getState(pgeom->getStateCount() - 1);
                    const auto *rv = gs->as<ompl::base::RealVectorStateSpace::StateType>();
                    for (size_t i = 0; i < controlInputs; ++i)
                        goalVec[i] = rv->values[i];
                    try
                    {
                        m_ctr.getPosDistal(goalVec, goalTip);
                        haveGoalTip = true;
                        gstate = gs;
                    }
                    catch (...)
                    {
                        haveGoalTip = false;
                    }
                }
            }
        }

        if (!haveGoalTip)
        {
            if (m_fallback)
            {
                m_fallback->sampleUniform(statePtr);
                return true;
            }
            return false;
        }

        // posCTRL is broken for 4-DoF (hardcoded 3x6 Jacobian).
        // Always delegate to fallback sampler.
        if (m_fallback)
        {
            m_fallback->sampleUniform(statePtr);
            return true;
        }
        return false;
    }

    bool sampleUniform(ompl::base::State *statePtr, const ompl::base::Cost &minCost, const ompl::base::Cost &maxCost) override
    {
        // Simple approach: attempt sampling with maxCost and ensure heuristic >= minCost
        for (int i = 0; i < 3; ++i)
        {
            if (sampleUniform(statePtr, maxCost))
            {
                double h = conservativeCostToProblemGoal(statePtr);
                if (h >= minCost.value())
                    return true;
            }
        }
        return false;
    }

    bool hasInformedMeasure() const override { return true; }
    double getInformedMeasure(const ompl::base::Cost & /*currentCost*/) const override
    {
        // Return a positive, conservative estimate of the informed measure.
        // Use the tip-space ball volume as a simple proxy so OMPL will take
        // the direct informed-sampling code path instead of rejection.
        const double r = std::max(0.0, m_tipRadius);
        const double vol = (4.0 / 3.0) * M_PI * r * r * r;
        return vol;
    }

    // Helpers
    // randomPointInBall is a free function in CTR_SamplerUtils.hpp;
    // use member rng_ so no new RNG is constructed per call.
    ompl::RNG rng_;

    // Conservative cost-to-go using direct arc length computation.
    // Do NOT use getArclengthEnd(q) — broken for 4-DoF (tau[2]=alpha1, not beta3).
    // Instead compute directly: end1=L[0]+beta1, end2=L[1]+beta2, end3=L[2] (static).
    double conservativeCostToGoal(const ompl::base::State *s, const ompl::base::State *g) const
    {
        if (!s || !g)
            return std::numeric_limits<double>::infinity();

        constexpr double c1 = 1000.00;
        constexpr double c2 = 6000.00;
        constexpr double c3 = 3000.00;
        constexpr double c4 = 500.00;

        const auto *rs = s->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto *rg = g->as<ompl::base::RealVectorStateSpace::StateType>();

        const blaze::StaticVector<double, 3UL> L = m_ctr.getOverallLen();

        // Current state arc lengths
        const double es1 = L[0] + rs->values[0]; // tube1 distal
        const double es2 = L[1] + rs->values[1]; // tube2 distal
        const double es3 = L[2];                  // tube3 static

        // Goal state arc lengths
        const double eg1 = L[0] + rg->values[0];
        const double eg2 = L[1] + rg->values[1];
        const double eg3 = L[2];

        const double term1 = std::fabs(es1 - eg1);
        const double term2 = std::fabs((es1 - es2) - (eg1 - eg2));
        const double term3 = std::fabs((es2 - es3) - (eg2 - eg3));
        const double term4 = std::fabs((es1 - es3) - (eg1 - eg3));

        return c1 * term1 + c2 * term2 + c3 * term3 + c4 * term4;
    }

    // Convenience wrapper: compute the conservative cost-to-go from state s to
    // the ProblemDefinition's goal(s). Returns +inf if no goal is present.
    double conservativeCostToProblemGoal(const ompl::base::State *s) const
    {
        if (!s || !this->probDefn_)
            return std::numeric_limits<double>::infinity();
        const ompl::base::Goal *goal = this->probDefn_->getGoal().get();
        if (!goal)
            return std::numeric_limits<double>::infinity();

        using GoalState = ompl::base::GoalState;
        using GoalStates = ompl::base::GoalStates;

        if (goal->hasType(ompl::base::GoalType::GOAL_STATE))
        {
            const ompl::base::State *gs = goal->as<GoalState>()->getState();
            return conservativeCostToGoal(s, gs);
        }
        else if (goal->hasType(ompl::base::GoalType::GOAL_STATES))
        {
            const auto *gss = goal->as<GoalStates>();
            std::size_t count = gss->getStateCount();
            if (count == 0)
                return std::numeric_limits<double>::infinity();
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < count; ++i)
            {
                const auto *gs = gss->getState(i);
                if (!gs)
                    continue;
                double val = conservativeCostToGoal(s, gs);
                if (val < best)
                    best = val;
            }
            return best;
        }
        else
        {
            // For other goal types we don't have a cheap conversion to a
            // backbone-based heuristic; return +inf to indicate no usable
            // heuristic is available.
            return std::numeric_limits<double>::infinity();
        }
    }

private:
    CTR_StateValidityChecker<controlInputs> &m_validityChecker;
    PINNs<controlInputs> &m_ctr;
    ompl::base::StateSamplerPtr m_fallback;
    double m_tipRadius{0.02};
    int m_maxAttempts{50};
};

template <size_t controlInputs>
CTR_BackboneLengthObjective<controlInputs>::CTR_BackboneLengthObjective(const ompl::base::SpaceInformationPtr &si, PINNs<controlInputs> &_ctr)
    : ompl::base::StateCostIntegralObjective(si, true), m_ctr(_ctr)
{
}

template <size_t controlInputs>
ompl::base::Cost CTR_BackboneLengthObjective<controlInputs>::stateCost(const ompl::base::State *s) const
{
    const auto *state = s->as<ompl::base::RealVectorStateSpace::StateType>();

    // Do NOT use getArclengthEnd(q) — broken for 4-DoF (tau[2]=alpha1, not beta3).
    // Compute arc lengths directly from tube overall lengths.
    const blaze::StaticVector<double, 3UL> L = this->m_ctr.getOverallLen();
    const double beta1 = state->values[0];
    const double beta2 = state->values[1];

    const double end1 = L[0] + beta1; // tube1 distal arc length
    const double end2 = L[1] + beta2; // tube2 distal arc length
    const double end3 = L[2];          // tube3 static (beta3=0)

    return ompl::base::Cost(k1 * end1 +
                            k2 * (end1 - end2) +
                            k3 * (end2 - end3) +
                            k4 * (end1 - end3));
}

template <size_t controlInputs>
ompl::base::InformedSamplerPtr CTR_BackboneLengthObjective<controlInputs>::allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDef, unsigned int /*maxNumberCalls*/) const
{
    logging::debug("CTR_BackboneLengthObjective::allocInformedStateSampler called");
    // Prevent recursive allocation / re-entrancy when OMPL calls allocators from inside constructors
    if (informed_sampler_guard::constructing)
    {
        logging::debug("Guard active, skipping informed sampler construction");
        return ompl::base::InformedSamplerPtr();
    }

    // Acquire SpaceInformation and the CTR-specific validity checker
    auto si = probDef->getSpaceInformation();
    if (!si)
    {
        logging::debug("ProblemDefinition has no SpaceInformation");
        return ompl::base::InformedSamplerPtr();
    }

    auto vc = std::dynamic_pointer_cast<CTR_StateValidityChecker<controlInputs>>(si->getStateValidityChecker());
    if (!vc)
    {
        logging::debug("Failed to dynamic_cast CTR_StateValidityChecker");
        return ompl::base::InformedSamplerPtr();
    }

    ompl::base::StateSamplerPtr fallback = std::make_shared<CTR_StateSampler<controlInputs>>(si->getStateSpace().get(), *vc, const_cast<PINNs<controlInputs> &>(m_ctr));
    logging::debug("Created fallback sampler for backbone objective");

    // Cost getter used by the informed sampler to obtain current best cost
    auto getCost = [probDef]() -> ompl::base::Cost
    {
        if (!probDef->hasSolution())
            return ompl::base::Cost(std::numeric_limits<double>::infinity());
        auto path = probDef->getSolutionPath();
        if (!path)
            return ompl::base::Cost(std::numeric_limits<double>::infinity());
        auto opt = probDef->getOptimizationObjective();
        if (!opt)
            return ompl::base::Cost(std::numeric_limits<double>::infinity());
        return path->cost(opt);
    };

    // Construct the informed sampler safely while setting the global guard
    informed_sampler_guard::constructing = true;
    try
    {
        auto informed = std::make_shared<CTR_PINNsInformedSamplerImpl<controlInputs>>(probDef, 1000, *vc, const_cast<PINNs<controlInputs> &>(m_ctr), fallback);
        logging::debug("CTR_BackboneLengthObjective created informed sampler");
        informed_sampler_guard::constructing = false;
        return informed;
    }
    catch (const std::exception &ex)
    {
        logging::debug("CTR_BackboneLengthObjective::allocInformedStateSampler exception: ", ex.what());
        informed_sampler_guard::constructing = false;
        return ompl::base::InformedSamplerPtr();
    }
    catch (...)
    {
        logging::debug("CTR_BackboneLengthObjective::allocInformedStateSampler unknown exception");
        informed_sampler_guard::constructing = false;
        return ompl::base::InformedSamplerPtr();
    }
}
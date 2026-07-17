#pragma once

#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

#include <memory>
#include <cmath>

#include "CTR_SamplerUtils.hpp"
#include "PINNs.hpp"
#include "CTR_StateValidityChecker.hpp"

// A PINNs-informed state sampler installed via setStateSamplerAllocator.
// Biases samples toward the goal tip using the PINNs IK model (posCTRL).
// Falls back to the provided fallback sampler when IK fails or with
// probability (1 - m_bias). Inherits StateSampler directly; OMPL planners
// obtain their informed (cost-bounded) samples from the objective's
// allocInformedStateSampler, so the InformedStateSampler interface is not
// needed here and the InformedSampler cost-bounded virtuals do not apply.

template <size_t controlInputs>
class CTR_PINNsInformedStateSampler : public ompl::base::StateSampler
{
public:
    CTR_PINNsInformedStateSampler(const ompl::base::StateSpace *space,
                                  CTR_StateValidityChecker<controlInputs> &validityChecker,
                                  PINNs<controlInputs> &ctr,
                                  ompl::base::StateSamplerPtr fallback)
        : ompl::base::StateSampler(space), m_validityChecker(validityChecker), m_ctr(ctr), m_fallback(std::move(fallback))
    {
        m_bias = 0.50;
        m_tipRadius = 0.005;
        m_hasGoal = false;
    }

    ~CTR_PINNsInformedStateSampler() override = default;

    // Set the probability of attempting an IK-based informed sample (vs. fallback)
    void setBias(double b) { m_bias = std::min(std::max(b, 0.0), 1.0); }

    // Register the external tip force so the goal tip is computed on the
    // loaded (deflected) robot. Recomputes the goal tip if a goal is set.
    void setExternalForce(const blaze::StaticVector<double, 3UL> &force)
    {
        m_force = force;
        if (m_hasGoal)
            recomputeGoalTip();
    }

    // Provide the goal state (the raw State pointer). Planner should call
    // this when goal is known so we can compute the goal tip via PINNs.
    void setGoalState(const ompl::base::State *goal)
    {
        if (!goal)
        {
            m_hasGoal = false;
            return;
        }
        m_hasGoal = true;
        const auto *rv = goal->as<ompl::base::RealVectorStateSpace::StateType>();
        for (size_t i = 0; i < controlInputs; ++i)
            m_goalVec[i] = rv->values[i];
        recomputeGoalTip();
    }

    // Set radius (meters) used when sampling in tip-space
    void setTipRadius(double r) { m_tipRadius = std::max(0.0, r); }

    // OMPL calls this for every sample request. With probability m_bias attempt
    // IK-based sampling near the goal tip; otherwise delegate to fallback.
    void sampleUniform(ompl::base::State *state) override
    {
        const double u = rng_.uniformReal(0.0, 1.0);
        if (u > m_bias || !m_hasGoal)
        {
            if (m_fallback)
                m_fallback->sampleUniform(state);
            return;
        }

        // posCTRL is broken for 4-DoF (hardcoded 3x6 Jacobian).
        // Always delegate to fallback sampler.
        if (m_fallback)
            m_fallback->sampleUniform(state);
    }

    // Allow runtime tuning of the maximum number of informed attempts
    void setMaxAttempts(int n) { m_maxAttempts = std::max(1, n); }

    // Fallback implementations: delegate to fallback sampler to keep behavior
    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override
    {
        if (m_fallback)
            m_fallback->sampleUniformNear(state, near, distance);
    }

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override
    {
        if (m_fallback)
            m_fallback->sampleGaussian(state, mean, stdDev);
    }

private:
    void recomputeGoalTip()
    {
        try
        {
            m_ctr.getPosDistal(m_goalVec, m_force, m_goalTip);
        }
        catch (...) // be conservative
        {
            m_hasGoal = false;
        }
    }

    CTR_StateValidityChecker<controlInputs> &m_validityChecker;
    PINNs<controlInputs> &m_ctr;
    ompl::base::StateSamplerPtr m_fallback;
    ompl::RNG rng_;
    int m_maxAttempts{50};

    bool m_hasGoal{false};
    blaze::StaticVector<double, controlInputs> m_goalVec{};
    blaze::StaticVector<double, 3UL> m_goalTip{};
    blaze::StaticVector<double, 3UL> m_force{};

    double m_bias;      // probability of attempting IK-based informed sample
    double m_tipRadius; // meters — radius of tip-space sampling ball
};

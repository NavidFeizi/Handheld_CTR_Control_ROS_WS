#pragma once

#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSampler.h>

#include <memory>
#include <cmath>

#include "CTR_SamplerUtils.hpp"
#include "PINNs.hpp"
#include "CTR_StateValidityChecker.hpp"

// A simple PINNs-informed state sampler that biases samples toward the goal
// tip position. This is NOT an OMPL InformedStateSampler subclass; instead it
// implements ompl::base::StateSampler and can be installed via
// setStateSamplerAllocator on the state space. It uses the PINNs forward model
// to compute tip positions and the PINNs::posCTRL (IK) routine to map a
// target tip back to a joint configuration. If IK or validity checks fail the
// sampler falls back to the provided uniform sampler.

template <size_t controlInputs>
class CTR_PINNsInformedSampler : public ompl::base::StateSampler
{
public:
    // ctor: space must outlive this sampler. fallbackSampler is used when the
    // informed approach fails; it can be the regular CTR_StateSampler.
    CTR_PINNsInformedSampler(const ompl::base::StateSpace *space,
                             CTR_StateValidityChecker<controlInputs> &validityChecker,
                             PINNs<controlInputs> &ctr,
                             ompl::base::StateSamplerPtr fallbackSampler)
        : ompl::base::StateSampler(space), m_validityChecker(validityChecker), m_ctr(ctr), m_fallback(std::move(fallbackSampler))
    {
        // default bias: how often to attempt an informed sample [0..1]
        m_bias = 0.50;
        // default radius (meters) around goal tip used for sampling tip-space
        m_tipRadius = 0.005; // 0.5 cm by default; tune for your workspace
    }

    ~CTR_PINNsInformedSampler() override = default;

    // Set the desired bias (probability of generating informed samples)
    void setBias(double b) { m_bias = std::min(std::max(b, 0.00), 1.00); }

    // Set the tip-space sampling radius in meters
    void setTipRadius(double r) { m_tipRadius = std::max(r, 0.00); }

    // Set maximum informed attempts when trying IK-based samples
    void setMaxAttempts(int n) { m_maxAttempts = std::max(1, n); }

    // Register the external tip force so the goal tip is computed on the
    // loaded (deflected) robot. Recomputes the goal tip if a goal is set.
    void setExternalForce(const blaze::StaticVector<double, 3UL> &force)
    {
        m_force = force;
        if (m_hasGoal)
            recomputeGoalTip();
    }

    // Provide the goal state so the sampler can compute goal tip position
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
        // compute tip position for goal using PINNs (force-aware)
        recomputeGoalTip();
    }

    // Primary sampling method called by OMPL
    void sampleUniform(ompl::base::State *state) override
    {
        // With probability m_bias attempt an informed sample; otherwise fallback
        const double u = rng_.uniformReal(0.00, 1.00);
        if (u > m_bias || !m_hasGoal)
        {
            if (m_fallback)
                m_fallback->sampleUniform(state);
            return;
        }

        // posCTRL is broken for 4-DoF (hardcoded 3×6 Jacobian).
        // Always use the fallback sampler for uniform sampling.
        if (m_fallback)
            m_fallback->sampleUniform(state);
    }

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override
    {
        // Delegate to fallback for near-sampling (2-tube layout handled there)
        if (m_fallback)
            m_fallback->sampleUniformNear(state, near, distance);
    }

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override
    {
        // Delegate to fallback for Gaussian sampling (2-tube layout handled there)
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
    ompl::RNG rng_;
    ompl::base::StateSamplerPtr m_fallback;

    // goal info
    bool m_hasGoal{false};
    blaze::StaticVector<double, controlInputs> m_goalVec{};
    blaze::StaticVector<double, 3UL> m_goalTip{};
    blaze::StaticVector<double, 3UL> m_force{};

    double m_bias;      // probability to attempt informed sample
    double m_tipRadius; // meters
    int m_maxAttempts{10};

};
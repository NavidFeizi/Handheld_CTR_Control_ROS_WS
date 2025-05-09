#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
// #include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSampler.h>

#include "CTR.hpp"
#include "CTR_StateValidityChecker.hpp"

class CTR_StateSampler : public ompl::base::StateSampler
{
public:
    // // default constructor
    // CTR_StateSampler() = delete;

    // overloaded constructor
    CTR_StateSampler(const ompl::base::StateSpace *space, CTR_StateValidityChecker& validityChecker, CTR& ctr);

    // destructor
    ~CTR_StateSampler() override = default;

    // // copy constructor
    // CTR_StateSampler(const CTR_StateSampler &rhs);

    // // move constructor
    // CTR_StateSampler(CTR_StateSampler &&rhs);

    // // copy assignment operator
    // CTR_StateSampler &operator=(const CTR_StateSampler &rhs);

    // // move assignment operator
    // CTR_StateSampler &operator=(CTR_StateSampler &&rhs) noexcept;

    void sampleUniform(ompl::base::State *state) override;

    // Add these overrides:
    void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, double distance) override 
    {
        throw ompl::Exception("CTR_StateSampler", "sampleUniformNear not implemented");
    }

    void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override 
    {
        throw ompl::Exception("CTR_StateSampler", "sampleGaussian not implemented");
    }

    void setParameters(double b1, double b2, double b3, double clearance);

    void setGoalBetas(double b1, double b2, double b3);

protected:
    ompl::RNG rng_;
    
private:
    CTR& m_ctr;
    CTR_StateValidityChecker& m_validityChecker;
    double m_beta1_Min, m_beta2_Min, m_beta3_Min;
    double m_beta1_goal, m_beta2_goal, m_beta3_goal;
    double m_Clr; // minimum clearance between linear actuators
};
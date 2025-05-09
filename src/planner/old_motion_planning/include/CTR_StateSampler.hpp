#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ValidStateSampler.h>

#include "CTR.hpp"

class CTR_StateSampler : public ompl::base::ValidStateSampler
{
public:
    // // default constructor
    // CTR_StateSampler() = delete;

    // overloaded constructor
    CTR_StateSampler(const ompl::base::SpaceInformation *si);

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

    bool sample(ompl::base::State *state) override;

    // We don't need this in the example below.
    bool sampleNear(ompl::base::State* /*state*/, const ompl::base::State* /*near*/, const double /*distance*/) override;

    void setParameters(double b1, double b2, double b3, double clearance);

    void setGoalBetas(double b1, double b2, double b3);

protected:
    ompl::RNG rng_;

    private:
    double m_beta1_Min, m_beta2_Min, m_beta3_Min;
    double m_beta1_goal, m_beta2_goal, m_beta3_goal;
    double m_Clr; // minimum clearance between linear actuators
};
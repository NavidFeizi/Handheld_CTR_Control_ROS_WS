#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

// including the CTR class to compute illegal configurations and collisions with the anatomy
#include "CTR.hpp"
#include <iostream>

class CTR_StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    CTR_StateValidityChecker() = delete;

    CTR_StateValidityChecker(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<CTR> _ctr);

    bool isValid(const ompl::base::State *state) const override;

    double clearance();

    // New methods to set/get the final/goal betas
    void setGoalBetas(double beta1, double beta2, double beta3);

    double getBeta1Goal() const;
    
    double getBeta2Goal() const;
    
    double getBeta3Goal() const;

private:
    std::shared_ptr<CTR> ctr;
    double m_beta1_goal, m_beta2_goal, m_beta3_goal; 
};
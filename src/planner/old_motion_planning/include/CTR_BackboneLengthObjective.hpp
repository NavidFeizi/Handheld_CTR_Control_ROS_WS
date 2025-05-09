#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "CTR.hpp"

class CTR_BackboneLengthObjective : public ompl::base::StateCostIntegralObjective
{
public:
    CTR_BackboneLengthObjective(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<CTR> _ctr);

    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

private:
    std::shared_ptr<CTR> m_ctr;
};
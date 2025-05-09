#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <array>

class CTR_RevoluteJointObjective : public ompl::base::StateCostIntegralObjective
{
public:
    CTR_RevoluteJointObjective(const ompl::base::SpaceInformationPtr &si, const std::array<double, 3UL> &goal);

    ompl::base::Cost stateCost(const ompl::base::State *state) const override;

private:
    std::array<double, 3UL> m_goal;
};
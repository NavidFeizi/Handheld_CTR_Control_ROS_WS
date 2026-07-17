#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <limits>

// Make CTR_StateSampler templated to match PINNs
template <size_t controlInputs>
class CTR_ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
    CTR_ClearanceObjective(const ompl::base::SpaceInformationPtr &si);

    ompl::base::Cost stateCost(const ompl::base::State *s) const override;
};

// ============================= Implementation =============================

template <size_t controlInputs>
CTR_ClearanceObjective<controlInputs>::CTR_ClearanceObjective(const ompl::base::SpaceInformationPtr &si)
    : ompl::base::StateCostIntegralObjective(si, true)
{
}

template <size_t controlInputs>
ompl::base::Cost CTR_ClearanceObjective<controlInputs>::stateCost(const ompl::base::State *s) const
{
    const double clearance = si_->getStateValidityChecker()->clearance(s);
    constexpr double eps = 1.00E-9;
    return ompl::base::Cost(1.00 / (clearance + eps));
}

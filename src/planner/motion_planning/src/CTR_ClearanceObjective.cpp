#include "CTR_ClearanceObjective.hpp"

CTR_ClearanceObjective::CTR_ClearanceObjective(const ompl::base::SpaceInformationPtr& si) : ompl::base::StateCostIntegralObjective(si, true)
{}

ompl::base::Cost CTR_ClearanceObjective::stateCost(const ompl::base::State* s) const
{
    return ompl::base::Cost(1.00 / (si_->getStateValidityChecker()->clearance(s) +
             std::numeric_limits<double>::min()));
}
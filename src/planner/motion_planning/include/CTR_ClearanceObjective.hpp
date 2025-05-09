#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <limits>

class CTR_ClearanceObjective : public ompl::base::StateCostIntegralObjective
 {
 public:
     CTR_ClearanceObjective(const ompl::base::SpaceInformationPtr& si);
  
     ompl::base::Cost stateCost(const ompl::base::State* s) const override;
 };
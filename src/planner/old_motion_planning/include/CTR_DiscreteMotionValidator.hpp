#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/DiscreteMotionValidator.h>

// including the CTR class to compute illegal configurations and collisions with the anatomy
#include "CTR.hpp"

class CTR_DiscreteMotionValidator : public ompl::base::DiscreteMotionValidator
{
public:
    CTR_DiscreteMotionValidator(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<CTR> _ctr);

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    void setGoalBetas(double b1, double b2, double b3);

private:
    std::shared_ptr<CTR> ctr;
    double beta1_goal, beta2_goal, beta3_goal;
    mutable bool goalBetasSet = false;
    
    bool checkDeploymentPhases(const blaze::StaticVector<double, 6UL> &q) const;
};
// Discrete motion validator for CTR: checks interpolated states along an edge
// for mechanical ordering constraints (beta sequence) and (future) anatomical
// collisions. Uses SpaceInformation's state allocation utilities and reuses a
// single temporary state for efficiency.

#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/util/Console.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include "PINNs.hpp"

// 2-tube CTR discrete motion validator.
// State layout: [β₁, β₂, α₁, α₂]  (indices 0-3)
// Tube 3 (outermost) is static and excluded from the planning state.
template <size_t controlInputs>
class CTR_DiscreteMotionValidator : public ompl::base::DiscreteMotionValidator
{
public:
    CTR_DiscreteMotionValidator(const ompl::base::SpaceInformationPtr &si, PINNs<controlInputs> &_ctr);

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    void setGoalBetas(double b1, double b2);

private:
    PINNs<controlInputs> &m_ctr;
    double m_Clr{0.0};
    double beta1_goal{0.0}, beta2_goal{0.0};
    mutable bool goalBetasSet{false};
    static constexpr double goalTolerance{1.00E-6};
};

// ============================= Implementation =============================

template <size_t controlInputs>
CTR_DiscreteMotionValidator<controlInputs>::CTR_DiscreteMotionValidator(const ompl::base::SpaceInformationPtr &si, PINNs<controlInputs> &_ctr)
    : ompl::base::DiscreteMotionValidator(si), m_ctr(_ctr)
{
    m_Clr = _ctr.getStageThickness();
}

template <size_t controlInputs>
bool CTR_DiscreteMotionValidator<controlInputs>::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    if (!goalBetasSet)
        OMPL_WARN("CTR_DiscreteMotionValidator: checkMotion called before setGoalBetas(). "
                  "Deployment-phase monotonicity will not be enforced. "
                  "Call Planner::setGoalState() before solving.");

    if (!si_->isValid(s1) || !si_->isValid(s2))
        return false;

    if (si_->getStateSpace()->equalStates(s1, s2))
        return true;

    const auto nd = static_cast<unsigned int>(si_->getStateSpace()->validSegmentCount(s1, s2));

    auto *tmp = si_->getStateSpace()->allocState();

    bool beta2Reached = false;

    for (unsigned int i = 1; i < nd; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(nd);
        si_->getStateSpace()->interpolate(s1, s2, t, tmp);

        const auto *rv = tmp->as<ompl::base::RealVectorStateSpace::StateType>();
        const double b1 = rv->values[0UL];
        const double b2 = rv->values[1UL];

        // Mechanical ordering: β₁ < β₂ with clearance
        if (b1 > b2 - m_Clr)
        {
            si_->getStateSpace()->freeState(tmp);
            return false;
        }

        // Deployment-phase monotonicity: once β₂ reaches its goal, forbid retraction.
        if (goalBetasSet)
        {
            if (!beta2Reached && std::fabs(b2 - beta2_goal) <= goalTolerance)
                beta2Reached = true;
            if (beta2Reached && b2 < beta2_goal - goalTolerance)
            {
                si_->getStateSpace()->freeState(tmp);
                return false;
            }
        }

        if (!si_->isValid(tmp))
        {
            si_->getStateSpace()->freeState(tmp);
            return false;
        }
    }

    si_->getStateSpace()->freeState(tmp);
    return true;
}

template <size_t controlInputs>
void CTR_DiscreteMotionValidator<controlInputs>::setGoalBetas(double b1, double b2)
{
    beta1_goal = b1;
    beta2_goal = b2;
    goalBetasSet = true;
}
#pragma once

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

// including the CTR class to compute illegal configurations and collisions with the anatomy
#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
#include <iostream>

// 2-tube CTR state validity checker.
// State layout: [β₁, β₂, α₁, α₂]  (indices 0-3)
// Tube 3 (outermost) is static and excluded from the planning state.
template<size_t controlInputs>
class CTR_StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    CTR_StateValidityChecker() = delete;

    CTR_StateValidityChecker(const ompl::base::SpaceInformationPtr &si, PINNs<controlInputs>& _ctr);

    bool isValid(const ompl::base::State *state) const override;

    double clearance(const ompl::base::State* state) const override;

    // Set / get the goal prismatic values used by the sampler
    void setGoalBetas(double beta1, double beta2);

    double getBeta1Goal() const;

    double getBeta2Goal() const;

private:
    double m_Clr;                    // minimum clearance between adjacent linear actuators
    double m_beta1_min, m_beta1_max; // joint limits for Tube 1 prismatic
    double m_beta2_min, m_beta2_max; // joint limits for Tube 2 prismatic
    double m_beta1_goal, m_beta2_goal;
};

// ============================= Implementation =============================

template<size_t controlInputs>
CTR_StateValidityChecker<controlInputs>::CTR_StateValidityChecker(const ompl::base::SpaceInformationPtr &si, PINNs<controlInputs>& _ctr)
    : ompl::base::StateValidityChecker(si)
{
    // Use getInputPosBounds() which correctly handles the 4-DoF layout
    // [beta1, beta2, alpha1, alpha2]
    const auto [lb, ub] = _ctr.getInputPosBounds();
    m_beta1_min = lb[0UL];
    m_beta1_max = ub[0UL];
    m_beta2_min = lb[1UL];
    m_beta2_max = ub[1UL];
    m_Clr = _ctr.getStageThickness();

    // Initialise goal betas to the global joint maxima so that samplers
    // produce valid states even if setGoalBetas() has not been called yet.
    m_beta1_goal = m_beta1_max;
    m_beta2_goal = m_beta2_max;
}

template<size_t controlInputs>
bool CTR_StateValidityChecker<controlInputs>::isValid(const ompl::base::State *state) const
{
    const ompl::base::RealVectorStateSpace::StateType *candidate_State =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    // State layout: values[0]=β₁, values[1]=β₂, values[2]=α₁, values[3]=α₂
    const double beta_1 = candidate_State->values[0UL];
    const double beta_2 = candidate_State->values[1UL];

    const double alpha1 = candidate_State->values[2UL];
    const double alpha2 = candidate_State->values[3UL];

    // β₁ ordering: β₁_min ≤ β₁ ≤ min(β₁_max, β₂ − clr)
    const bool conditionTb1 = (beta_1 >= m_beta1_min) &&
                               (beta_1 <= std::min(m_beta1_max, beta_2 - m_Clr));

    // β₂ ordering: max(β₁ + clr, β₂_min) ≤ β₂ ≤ β₂_max
    const bool conditionTb2 = (beta_2 >= std::max(beta_1 + m_Clr, m_beta2_min)) &&
                               (beta_2 <= m_beta2_max);

    const bool conditionAngle = std::fabs(alpha2 - alpha1) <= M_PI;

    return conditionTb1 && conditionTb2 && conditionAngle;
}

template<size_t controlInputs>
double CTR_StateValidityChecker<controlInputs>::clearance(const ompl::base::State* /*state*/) const
{
    return 0.05;
}

template<size_t controlInputs>
void CTR_StateValidityChecker<controlInputs>::setGoalBetas(double beta1, double beta2)
{
    m_beta1_goal = beta1;
    m_beta2_goal = beta2;
}

template<size_t controlInputs>
double CTR_StateValidityChecker<controlInputs>::getBeta1Goal() const { return m_beta1_goal; }

template<size_t controlInputs>
double CTR_StateValidityChecker<controlInputs>::getBeta2Goal() const { return m_beta2_goal; }

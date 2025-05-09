#include "CTR_StateValidityChecker.hpp"

CTR_StateValidityChecker::CTR_StateValidityChecker(const ompl::base::SpaceInformationPtr &si, CTR& _ctr) : ompl::base::StateValidityChecker(si) 
{
    this->m_lengthStraight = _ctr.getStraightLengthsOfTubes();
    this->m_Clr = _ctr.getStageThickness();
}

bool CTR_StateValidityChecker::isValid(const ompl::base::State *state) const
{
    // std::cout << "CTR_StateValidityChecker called!" << std::endl;
    bool validState = true;
    const ompl::base::RealVectorStateSpace::StateType *candidate_State = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // retrieving the joint values of the linear actuators
    const double beta_1 = candidate_State->values[0UL];
    const double beta_2 = candidate_State->values[1UL];
    const double beta_3 = candidate_State->values[2UL];

    // >> checking for illegal configurations of the robot (linear actuator constraints)

    // (beta_1 >= -ls_1) AND (beta_1 <= beta_2 - clr)
    bool conditionTb1 = ((beta_1 >= -m_lengthStraight[0UL]) && (beta_1 <= beta_2 - m_Clr)) ? true : false;

    // (beta_2 >= -ls_2) AND (beta_2 <= beta_3 - clr)
    bool conditionTb2 = ((beta_2 >= -m_lengthStraight[1UL]) && (beta_2 <= beta_3 - m_Clr)) ? true : false;

    // (beta_3 >= -ls_3) AND (beta_3 <= 0.00 - clr)
    bool conditionTb3 = ((beta_3 >= -m_lengthStraight[2UL]) && (beta_3 <= 0.00)) ? true : false;

    // If the sampled state results in illegal configuration of the CTR, reject it!
    if (!(conditionTb1 && conditionTb2 && conditionTb3))
    {
        // std::cout << "Drawed actuation values outside the allowable range!" << std::endl;
        validState = false;
    }

    // std::cout << "Valid state: " << validState << std::endl;

    // returns the result of the validity check
    return validState; // Replace with collision check result
}

// function that computes the clearance (minimum distance) between the CTR and the anatomy
double CTR_StateValidityChecker::clearance()
{
    // return this->ctr->computeAnatomicalDistances();
    return 0.05;
}

void CTR_StateValidityChecker::setGoalBetas(double beta1, double beta2, double beta3)
{
    m_beta1_goal = beta1;
    m_beta2_goal = beta2;
    m_beta3_goal = beta3;
}

double CTR_StateValidityChecker::getBeta1Goal() const { return m_beta1_goal; }
double CTR_StateValidityChecker::getBeta2Goal() const { return m_beta2_goal; }
double CTR_StateValidityChecker::getBeta3Goal() const { return m_beta3_goal; }
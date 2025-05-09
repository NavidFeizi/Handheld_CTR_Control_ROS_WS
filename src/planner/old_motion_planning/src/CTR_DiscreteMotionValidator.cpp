#include "CTR_DiscreteMotionValidator.hpp"

CTR_DiscreteMotionValidator::CTR_DiscreteMotionValidator(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<CTR> _ctr) : ompl::base::DiscreteMotionValidator(si), ctr(_ctr) {}

bool CTR_DiscreteMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    if (!this->goalBetasSet)
        return false;

    // std::cout << "Entered MY motion validator routine..." << std::endl;

    static constexpr size_t numSteps = 4UL; // number of intermediate steps between the two configurations

    const blaze::StaticVector<double, numSteps> alpha = blaze::linspace(numSteps, 0.0000, 1.0000);

    const ompl::base::RealVectorStateSpace::StateType *State_S1 = s1->as<ompl::base::RealVectorStateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType *State_S2 = s2->as<ompl::base::RealVectorStateSpace::StateType>();

    // initial state
    const blaze::StaticVector<double, 6UL> q1 = {State_S1->values[0UL], State_S1->values[1UL], State_S1->values[2UL], State_S1->values[3UL], State_S1->values[4UL], State_S1->values[5UL]};
    // final state
    const blaze::StaticVector<double, 6UL> q2 = {State_S2->values[0UL], State_S2->values[1UL], State_S2->values[2UL], State_S2->values[3UL], State_S2->values[4UL], State_S2->values[5UL]};
    // intermediate state
    blaze::StaticVector<double, 6UL> q_previous, q_current;

    // vector of initial guesses
    blaze::StaticVector<double, 5UL> initialGuesses;
    size_t i = 0UL;
    bool validConfiguration = true;
    bool validSequence = true;
    bool tube3_done = false, tube2_done = false;
    const double clr = ctr->getStageThickness();

    // Interpolate and check every step
    while ((i < numSteps) && validConfiguration && validSequence)
    {
        q_current = alpha[i] * q2 + (1.00 - alpha[i]) * q1;

        // // actuates the CTR to the corresponding sampled state
        // this->ctr->actuate_CTR(initialGuesses, q_current);

        // // sending the backbone points to collision library
        // this->ctr->sendCTRBackbonePoints();

        // // sending the backbone points to collision library
        // validConfiguration = (this->ctr->computeAnatomicalCollisions() > 0) ? false : true;

        if ((q_current[0UL] > q_current[1UL] - clr) || (q_current[1UL] > q_current[2UL] - clr))
            validSequence = false;

        // Check if previous tubes remain fixed after their goals are reached
        if (i > 0)
        {
            if (q_previous[2UL] == beta3_goal && q_current[2UL] != beta3_goal)
                validSequence = false;
            if (q_previous[1UL] == beta2_goal && q_current[1UL] != beta2_goal)
                validSequence = false;
        }

        q_previous = q_current;

        i++;
    }

    return validConfiguration && validSequence;
}

void CTR_DiscreteMotionValidator::setGoalBetas(double b1, double b2, double b3)
{
    this->beta1_goal = b1;
    this->beta2_goal = b2;
    this->beta3_goal = b3;
    this->goalBetasSet = true;
}
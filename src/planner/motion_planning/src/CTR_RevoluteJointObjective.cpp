#include "CTR_RevoluteJointObjective.hpp"

CTR_RevoluteJointObjective::CTR_RevoluteJointObjective(const ompl::base::SpaceInformationPtr &si, const std::array<double, 3UL> &goal) : ompl::base::StateCostIntegralObjective(si, true), m_goal(goal) {}

ompl::base::Cost CTR_RevoluteJointObjective::stateCost(const ompl::base::State *state) const
{
    const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Penalize deviations for joints 3,4,5 (rotational)
    constexpr double c1 = 5000.00;
    constexpr double c2 = 3000.00;
    constexpr double c3 = 1000.00;

    // std::cout << "Revolute goal: [" << m_goal[0UL] << ", " << m_goal[1UL] << ", " << m_goal[2UL] << "]" << std::endl;

    double cost = c1 * std::abs(pos->values[3UL] - m_goal[0UL]) + c2 * std::abs(pos->values[4UL] - m_goal[1UL]) + c3 * std::abs(pos->values[5UL] - m_goal[2UL]);

    return ompl::base::Cost(cost);
}
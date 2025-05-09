#include "CTR_BackboneLengthObjective.hpp"

CTR_BackboneLengthObjective::CTR_BackboneLengthObjective(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<CTR> _ctr) : ompl::base::StateCostIntegralObjective(si, true), m_ctr(_ctr)
{}

ompl::base::Cost CTR_BackboneLengthObjective::stateCost(const ompl::base::State *s) const
{
    const ompl::base::RealVectorStateSpace::StateType *state = s->as<ompl::base::RealVectorStateSpace::StateType>();

    const blaze::StaticVector<double, 6UL> q = {state->values[0UL],
                                                state->values[1UL],
                                                state->values[2UL],
                                                state->values[3UL],
                                                state->values[4UL],
                                                state->values[5UL]};

    // std::cout << "q: " << blaze::trans(q);

    blaze::StaticVector<double, 5UL> initialGuess;

    m_ctr->actuate_CTR(initialGuess, q);

    // weights (cost) associated to lengths of sections in CTR backbone
    constexpr double c1 = 1000.00; // length tube 1
    constexpr double c2 = 3000.00; // length between distal ends of tube 1 & 2
    constexpr double c3 = 1500.00; // length between distal ends of tube 2 & 3
    constexpr double c4 = 5000.00; // length between distal ends of tube 1 & 3

    // section lengths in the CTR
    const blaze::StaticVector<double, 3UL> distEnds = this->m_ctr->getDistalEnds();

    // std::cout << "Distal ends: " << blaze::trans(distEnds) << std::endl;

    // return ompl::base::Cost( c1 * distEnds[0UL] );

    return ompl::base::Cost(c1 * distEnds[0UL] +                   // length tube 1
                            c2 * (distEnds[0UL] - distEnds[1UL]) + // length between distal ends of tube 1 & 2
                            c3 * (distEnds[1UL] - distEnds[2UL]) + // length between distal ends of tube 2 & 3
                            c4 * (distEnds[0UL] - distEnds[2UL])   // length between distal ends of tube 1 & 3
    );
}
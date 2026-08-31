#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <array>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <cmath>

// 2-tube CTR revolute joint objective
template <size_t controlInputs>
class CTR_RevoluteJointObjective : public ompl::base::StateCostIntegralObjective
{
public:
    // Per-joint penalty weights used in stateCost() and the cost-to-go heuristic.
    static constexpr double c1 = 2000.0; // inner tube revolute weight
    static constexpr double c2 = 1000.0; // intermediate tube revolute weight

    // Default progress-scaling parameters for the rear-loaded penalty curve.
    static constexpr double defaultProgressAlpha = 200.0;
    static constexpr double defaultProgressGamma =   1.1;

    CTR_RevoluteJointObjective(const ompl::base::SpaceInformationPtr &si, const std::array<double, 2UL> &goal);

    ompl::base::Cost stateCost(const ompl::base::State *state) const override;

    // Configure prismatic start/goal for progress estimation.
    void setPrismaticStartGoal(const std::array<double, 2UL> &startPrism, const std::array<double, 2UL> &goalPrism)
    {
        m_prismaticStart = startPrism;
        m_prismaticGoal = goalPrism;
        m_havePrismaticBounds = true;
    }

    void setProgressScalingParameters(double alpha, double gamma)
    {
        m_progressAlpha = std::max(0.00, alpha);
        m_progressGamma = std::max(0.00, gamma);
    }

private:
    std::array<double, 2UL> m_goal;
    std::array<double, 2UL> m_prismaticStart{0.00, 0.00};
    std::array<double, 2UL> m_prismaticGoal{0.00, 0.00};
    bool m_havePrismaticBounds{false};

    double m_progressAlpha{defaultProgressAlpha};
    double m_progressGamma{defaultProgressGamma};

    // 2D prismatic progress using indices 0 and 1
    double computePrismaticProgress(const ompl::base::State *state) const
    {
        const auto *rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        const double q0 = rv->values[0], q1 = rv->values[1];
        const double sx = m_prismaticStart[0], sy = m_prismaticStart[1];
        const double gx = m_prismaticGoal[0],  gy = m_prismaticGoal[1];
        const double vx = gx - sx, vy = gy - sy;
        const double denom = vx * vx + vy * vy;
        if (denom <= 1.00E-12)
        {
            const double distGoal2 = gx * gx + gy * gy;
            const double distCur2  = q0 * q0 + q1 * q1;
            if (distGoal2 <= 1.00E-12) return 0.00;
            return std::min(1.00, std::max(0.00, std::sqrt(distCur2 / distGoal2)));
        }
        const double dot = (q0 - sx) * vx + (q1 - sy) * vy;
        return std::min(1.00, std::max(0.00, dot / denom));
    }
};

// ============================= Implementation =============================

// Provide a cost-to-go heuristic so informed planners can use it. The
// heuristic is an admissible underestimate: the weighted absolute
// difference in revolute joints between the current state and the goal
// state(s). It is cheap to compute and expressed in the same units as
// stateCost().
template <size_t controlInputs>
static ompl::base::Cost revoluteCostToGoalHelper(const ompl::base::State *state, const ompl::base::Goal *goal, const std::array<double, 2UL> &weights)
{
    using namespace ompl::base;
    if (!state || !goal)
        return Cost(0.00);

    // Raw |a - b|: joint values are absolute motor angles, so the plain
    // difference is the true remaining rotation. A wrapped diff reported a
    // state a full physical turn from the goal as "aligned", flattening the
    // cost-to-go exactly for goals across the state-space seam.
    auto angleDiff = [](double a, double b)
    {
        return std::fabs(a - b);
    };

    // Read revolute joints at indices 2 and 3 (4-DoF: [β₁, β₂, α₁, α₂])
    auto getAngles = [](const State *s) -> blaze::StaticVector<double, 2UL>
    {
        blaze::StaticVector<double, 2UL> angs{0.00, 0.00};
        if (!s) return angs;
        const auto *rv = s->as<RealVectorStateSpace::StateType>();
        angs[0UL] = rv->values[2UL];
        angs[1UL] = rv->values[3UL];
        return angs;
    };

    if (goal->hasType(GoalType::GOAL_STATE))
    {
        const GoalState *gs = goal->as<GoalState>();
        const State *gstate = gs->getState();
        if (!gstate) return Cost(0.00);
        auto ang_cur = getAngles(state);
        auto ang_g   = getAngles(gstate);
        return Cost(weights[0] * angleDiff(ang_cur[0], ang_g[0]) +
                    weights[1] * angleDiff(ang_cur[1], ang_g[1]));
    }

    if (goal->hasType(GoalType::GOAL_STATES))
    {
        const GoalStates *gss = goal->as<GoalStates>();
        std::size_t count = gss->getStateCount();
        if (count == 0) return Cost(0.0);
        double best = std::numeric_limits<double>::infinity();
        auto ang_cur = getAngles(state);
        for (std::size_t i = 0; i < count; ++i)
        {
            const State *gs = gss->getState(i);
            if (!gs) continue;
            auto ang_g = getAngles(gs);
            double cost = weights[0] * angleDiff(ang_cur[0], ang_g[0]) +
                          weights[1] * angleDiff(ang_cur[1], ang_g[1]);
            if (cost < best) best = cost;
        }
        return Cost(best == std::numeric_limits<double>::infinity() ? 0.0 : best);
    }

    return Cost(0.00);
}

// Set a cost-to-go heuristic using the same per-joint weights as stateCost.
// (These used to be {1000, 500} -- half of stateCost's {c1, c2} -- so the
// heuristic and the integral cost disagreed by a factor of 2 in the same units.)
template <size_t controlInputs>
struct _RevoluteWeightsHolder
{
    static constexpr std::array<double, 2UL> weights = {
        CTR_RevoluteJointObjective<controlInputs>::c1,
        CTR_RevoluteJointObjective<controlInputs>::c2};
};

template <size_t controlInputs>
const std::array<double, 2UL> _RevoluteWeightsHolder<controlInputs>::weights;

template <size_t controlInputs>
CTR_RevoluteJointObjective<controlInputs>::CTR_RevoluteJointObjective(const ompl::base::SpaceInformationPtr &si, const std::array<double, 2UL> &goal)
    : ompl::base::StateCostIntegralObjective(si, true), m_goal(goal)
{
    auto weights = _RevoluteWeightsHolder<controlInputs>::weights;
    this->setCostToGoHeuristic([weights](const ompl::base::State *s, const ompl::base::Goal *g) -> ompl::base::Cost
                               { return revoluteCostToGoalHelper<controlInputs>(s, g, weights); });
}

template <size_t controlInputs>
ompl::base::Cost CTR_RevoluteJointObjective<controlInputs>::stateCost(const ompl::base::State *state) const
{
    const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Raw |a - b| -- absolute motor angles; see revoluteCostToGoalHelper.
    auto angleDiff = [](double a, double b)
    {
        return std::fabs(a - b);
    };

    // Revolute joints are at indices 2 (α₁) and 3 (α₂) in the 4-DoF state.
    const double cost = c1 * angleDiff(pos->values[2UL], m_goal[0UL]) +
                        c2 * angleDiff(pos->values[3UL], m_goal[1UL]);

    return ompl::base::Cost(cost);
}
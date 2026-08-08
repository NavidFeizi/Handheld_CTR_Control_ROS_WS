#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>

#include <blaze/Math.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"

// Follow-the-leader (FTL) deployment objective for the CTR.
//
// Penalizes the volume swept by the backbone between consecutive planner
// states. For an edge (q1 -> q2) the cost is the discrete counterpart of
//
//     J_FTL(q1, q2) = w_ftl * ∫ ||p(s; q2, f_ext) − p(s; q1, f_ext)||² ds
//
// where p(s; q, f) is the backbone curve predicted by the PINN under the
// external tip force f_ext, and node correspondence is by normalized arc
// length. A perfect follow-the-leader deployment (new sections appended at
// the tip while the already-deployed backbone stays in place) drives this
// integral to zero; lateral sweeps of the deployed backbone are heavily
// penalized. Because the shapes are evaluated on the LOADED robot, the
// external force and moment field enters the planning optimization directly,
// not just the forward simulation.
template <size_t controlInputs>
class CTR_FollowTheLeaderObjective : public ompl::base::OptimizationObjective
{
public:
    using JointVector = blaze::StaticVector<double, controlInputs>;

    // Weight converting the swept-volume integral (m²·m) to objective units.
    // Chosen so a full 90 mm deployment with ~5 mm lateral sweep is on the
    // same order as the backbone/revolute terms of the composite objective.
    static constexpr double w_ftl = 5.00E6;

    CTR_FollowTheLeaderObjective(const ompl::base::SpaceInformationPtr &si,
                                 PINNs<controlInputs> &ctr,
                                 const blaze::StaticVector<double, 3UL> &externalForce)
        : ompl::base::OptimizationObjective(si), m_ctr(ctr), m_force(externalForce)
    {
        description_ = "CTR follow-the-leader swept-volume objective";

        // Informed-sampling heuristic: lower-bound the remaining swept volume
        // by the squared tip displacement to the goal spread over one shape
        // node spacing. Cheap, force-aware, and conservative in practice.
        this->setCostToGoHeuristic([this](const ompl::base::State *s, const ompl::base::Goal *g) -> ompl::base::Cost
                                   { return this->tipDistanceHeuristic(s, g); });
    }

    // Update the external tip force. Cached shapes belong to the previous
    // load case and must be discarded.
    void setExternalForce(const blaze::StaticVector<double, 3UL> &force)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_force = force;
        m_shapeCache.clear();
    }

    [[nodiscard]] blaze::StaticVector<double, 3UL> getExternalForce() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_force;
    }

    // FTL is inherently an edge (motion) cost; states carry no cost themselves.
    ompl::base::Cost stateCost(const ompl::base::State * /*s*/) const override
    {
        return this->identityCost();
    }

    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        return ompl::base::Cost(w_ftl * this->sweptCost(stateToTau(s1), stateToTau(s2)));
    }

    // Discrete swept-volume proxy between two configurations, evaluated on the
    // loaded robot:  Σ_i ||p_i(q2, f_ext) − p_i(q1, f_ext)||² Δs
    // with the i-th of N nodes at equal normalized arc length on each backbone.
    double sweptCost(const JointVector &q1, const JointVector &q2) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        const auto shape1 = shapeForLocked(q1);
        const auto shape2 = shapeForLocked(q2);
        if (!shape1 || !shape2)
            return 0.00; // PINN failure: do not poison the cost landscape

        const std::size_t N = shape1->rows();
        if (N < 2UL || shape2->rows() != N)
            return 0.00;

        // Arc-length measure: average deployed length of the two configurations.
        const blaze::StaticVector<double, 3UL> L = m_ctr.getOverallLen();
        const double len1 = L[0UL] + q1[0UL];
        const double len2 = L[0UL] + q2[0UL];
        const double ds = 0.50 * (len1 + len2) / static_cast<double>(N - 1UL);

        double sum = 0.00;
        for (std::size_t i = 0UL; i < N; ++i)
        {
            const double dx = (*shape2)(i, 0UL) - (*shape1)(i, 0UL);
            const double dy = (*shape2)(i, 1UL) - (*shape1)(i, 1UL);
            const double dz = (*shape2)(i, 2UL) - (*shape1)(i, 2UL);
            sum += dx * dx + dy * dy + dz * dz;
        }

        return sum * ds;
    }

private:
    using ShapeMatrix = blaze::DynamicMatrix<double, blaze::rowMajor>;
    using ShapePtr = std::shared_ptr<const ShapeMatrix>;
    using CacheKey = std::array<std::int64_t, controlInputs>;

    struct CacheKeyHash
    {
        std::size_t operator()(const CacheKey &k) const noexcept
        {
            std::size_t h = 0UL;
            for (const auto v : k)
                h ^= std::hash<std::int64_t>{}(v) + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
            return h;
        }
    };

    static JointVector stateToTau(const ompl::base::State *s)
    {
        JointVector q(0.00);
        if (!s)
            return q;
        const auto *rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
        for (std::size_t i = 0UL; i < controlInputs; ++i)
            q[i] = rv->values[i];
        return q;
    }

    static CacheKey quantize(const JointVector &q)
    {
        // 1e-7 m / 1e-7 rad resolution — far below planner step sizes.
        CacheKey key{};
        for (std::size_t i = 0UL; i < controlInputs; ++i)
            key[i] = static_cast<std::int64_t>(std::llround(q[i] * 1.00E7));
        return key;
    }

    // Must be called with m_mutex held (PINN inference buffers are shared).
    ShapePtr shapeForLocked(const JointVector &q) const
    {
        const CacheKey key = quantize(q);
        auto it = m_shapeCache.find(key);
        if (it != m_shapeCache.end())
            return it->second;

        auto shape = std::make_shared<ShapeMatrix>(m_ctr.getNumNodes(), 3UL);
        try
        {
            m_ctr.getShape(q, m_force, *shape);
        }
        catch (...)
        {
            return ShapePtr();
        }

        // Bound memory: drop the whole cache when it grows too large. Planner
        // edge evaluations are strongly local, so refill cost is negligible.
        if (m_shapeCache.size() >= kMaxCacheEntries)
            m_shapeCache.clear();
        m_shapeCache.emplace(key, shape);
        return shape;
    }

    ompl::base::Cost tipDistanceHeuristic(const ompl::base::State *s, const ompl::base::Goal *goal) const
    {
        using namespace ompl::base;
        if (!s || !goal)
            return this->identityCost();

        auto tipOf = [this](const State *st, blaze::StaticVector<double, 3UL> &tip) -> bool
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            try
            {
                m_ctr.getPosDistal(stateToTau(st), m_force, tip);
                return true;
            }
            catch (...)
            {
                return false;
            }
        };

        blaze::StaticVector<double, 3UL> tipCur;
        if (!tipOf(s, tipCur))
            return this->identityCost();

        double bestD2 = std::numeric_limits<double>::infinity();
        if (goal->hasType(GoalType::GOAL_STATE))
        {
            blaze::StaticVector<double, 3UL> tipGoal;
            const State *gs = goal->as<GoalState>()->getState();
            if (gs && tipOf(gs, tipGoal))
                bestD2 = blaze::sqrNorm(tipCur - tipGoal);
        }
        else if (goal->hasType(GoalType::GOAL_STATES))
        {
            const auto *gss = goal->as<GoalStates>();
            for (std::size_t i = 0UL; i < gss->getStateCount(); ++i)
            {
                blaze::StaticVector<double, 3UL> tipGoal;
                if (gss->getState(i) && tipOf(gss->getState(i), tipGoal))
                    bestD2 = std::min(bestD2, blaze::sqrNorm(tipCur - tipGoal));
            }
        }

        if (!std::isfinite(bestD2))
            return this->identityCost();

        // One node spacing as the arc-length measure keeps this a lower bound.
        const blaze::StaticVector<double, 3UL> L = m_ctr.getOverallLen();
        const double ds = L[0UL] / static_cast<double>(std::max<std::size_t>(2UL, m_ctr.getNumNodes()) - 1UL);
        return ompl::base::Cost(w_ftl * bestD2 * ds);
    }

    static constexpr std::size_t kMaxCacheEntries = 20000UL;

    PINNs<controlInputs> &m_ctr;
    blaze::StaticVector<double, 3UL> m_force;
    mutable std::mutex m_mutex;
    mutable std::unordered_map<CacheKey, ShapePtr, CacheKeyHash> m_shapeCache;
};

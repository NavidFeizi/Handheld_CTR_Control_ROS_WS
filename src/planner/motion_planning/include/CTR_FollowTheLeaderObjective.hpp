#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <blaze/Math.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "FtlSweptArea.hpp"
#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"

// Follow-the-leader (FTL) deployment objective for the CTR.
//
// Penalizes the lateral area swept by the backbone between consecutive planner
// states. For an edge (q1 -> q2) the cost is the discrete counterpart of
//
//     J_FTL(q1, q2) = w_ftl * ∫_0^ℓ ||(I − t tᵀ) (p(s; q2, f_ext) − p(s; q1, f_ext))|| ds
//
// where p(s; q, f) is the backbone curve predicted by the PINN under the
// external tip force f_ext, s is the ABSOLUTE arc length from the fixed base,
// ℓ the shorter of the two deployed lengths and t the unit backbone tangent.
// Only the component normal to the backbone is penalized: a perfect
// follow-the-leader deployment (new sections appended at the tip while the
// already-deployed backbone stays in place) and tubes sliding along their
// own curve cost zero. Units m²; linear in the displacement, so the cost of a
// motion does not depend on how finely it is segmented. Because the shapes
// are evaluated on the LOADED robot, the external force and moment field
// enters the planning optimization directly, not just the forward simulation.
template <size_t controlInputs>
class CTR_FollowTheLeaderObjective : public ompl::base::OptimizationObjective
{
public:
    using JointVector = blaze::StaticVector<double, controlInputs>;

    // Weight converting the lateral swept area (m²) to objective units.
    static constexpr double w_ftl = 5.00E6;

    CTR_FollowTheLeaderObjective(const ompl::base::SpaceInformationPtr &si,
                                 PINNs<controlInputs> &ctr,
                                 const blaze::StaticVector<double, 3UL> &externalForce)
        : ompl::base::OptimizationObjective(si), m_ctr(ctr), m_force(externalForce)
    {
        description_ = "CTR follow-the-leader lateral swept-area objective";
        // No cost-to-go heuristic: a pure FTL deployment reaches a tip target
        // at zero cost, so the only admissible lower bound is zero (OMPL default).
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

    // Lateral swept area between two configurations, evaluated on the loaded
    // robot (see header comment and ftl::lateralSweptArea).
    double sweptCost(const JointVector &q1, const JointVector &q2) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        const auto shape1 = shapeForLocked(q1);
        const auto shape2 = shapeForLocked(q2);
        if (!shape1 || !shape2)
            return 0.00; // PINN failure: do not poison the cost landscape

        // getShape samples each backbone on linspace(0, L1 + β1, N)
        const blaze::StaticVector<double, 3UL> L = m_ctr.getOverallLen();
        return ftl::lateralSweptArea(*shape1, L[0UL] + q1[0UL], *shape2, L[0UL] + q2[0UL]);
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

    static constexpr std::size_t kMaxCacheEntries = 20000UL;

    PINNs<controlInputs> &m_ctr;
    blaze::StaticVector<double, 3UL> m_force;
    mutable std::mutex m_mutex;
    mutable std::unordered_map<CacheKey, ShapePtr, CacheKeyHash> m_shapeCache;
};

#ifndef DEPLOYMENT_SCHEDULE_HPP_
#define DEPLOYMENT_SCHEDULE_HPP_

// Deployment-schedule candidate generation and scoring, extracted from
// Planner.hpp so they are testable without OMPL, Torch, or a live FTL
// objective. buildDeploymentCandidates is purely geometric;
// scheduleSweptCost takes the pairwise swept-cost as a std::function
// (the Planner injects m_ftlObjective->sweptCost).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

#include <blaze/Math.h>

namespace deployment_schedule
{

template <size_t controlInputs>
using JointVector = blaze::StaticVector<double, controlInputs>;

template <size_t controlInputs>
using Waypoints = std::vector<JointVector<controlInputs>>;

template <size_t controlInputs>
struct Candidate
{
    const char *name;
    Waypoints<controlInputs> wps;
};

// Builds the four candidate prismatic deployment schedules from q_from to
// q_to (β in the first two coordinates, α values held from q_from), each
// discretized with increments <= safeStep. All schedules are monotone in β.
template <size_t controlInputs>
std::vector<Candidate<controlInputs>> buildDeploymentCandidates(
    const JointVector<controlInputs> &q_from, const JointVector<controlInputs> &q_to, const double safeStep)
{
    using WaypointsT = Waypoints<controlInputs>;

    // Per-tube prismatic displacements (2 tubes).
    std::array<double, 2> disp;
    for (size_t i = 0; i < 2; ++i)
        disp[i] = q_to[i] - q_from[i];

    // Sort tube indices by ascending |displacement| to find stopping order.
    std::array<size_t, 2> stopOrder = {0, 1};
    std::sort(stopOrder.begin(), stopOrder.end(),
              [&](size_t a, size_t b) { return std::abs(disp[a]) < std::abs(disp[b]); });

    // Synchronized: all unfinished tubes advance at equal linear velocity; the
    // tube with the least remaining travel stops first (zero relative
    // inter-tube velocity during co-deployment).
    auto makeSynchronized = [&]() -> WaypointsT
    {
        WaypointsT wps;
        wps.push_back(q_from);
        JointVector<controlInputs> q_cur(q_from);
        double accumulated = 0.0; // |displacement| already applied to active tubes

        for (size_t step = 0; step < 2; ++step)
        {
            const size_t stopIdx = stopOrder[step];
            const double subDelta = std::abs(disp[stopIdx]) - accumulated; // remaining travel this sub-phase

            if (subDelta < 1.0e-9)
            {
                // Tube has negligible remaining travel — snap and skip this sub-phase.
                q_cur[stopIdx] = q_to[stopIdx];
                accumulated = std::abs(disp[stopIdx]);
                continue;
            }

            // Subdivide this sub-phase into fine equal increments of size ≤ safeStep.
            const size_t nIncrements = static_cast<size_t>(std::ceil(subDelta / safeStep));
            const double inc = subDelta / static_cast<double>(nIncrements); // exact equal increment

            for (size_t n = 1; n <= nIncrements; ++n)
            {
                for (size_t k = step; k < 2; ++k)
                {
                    const size_t idx = stopOrder[k];
                    const double sign = (disp[idx] >= 0.0) ? 1.0 : -1.0;
                    q_cur[idx] += sign * inc;
                }
                if (n == nIncrements)
                {
                    // Snap to exact goal to prevent floating-point drift.
                    q_cur[stopIdx] = q_to[stopIdx];
                }
                wps.push_back(q_cur);
            }

            accumulated = std::abs(disp[stopIdx]);
        }
        return wps;
    };

    // Sequential: deploy tube `first` fully, then the other tube.
    auto makeSequential = [&](size_t first) -> WaypointsT
    {
        WaypointsT wps;
        wps.push_back(q_from);
        JointVector<controlInputs> q_cur(q_from);
        const std::array<size_t, 2> order = {first, 1 - first};
        for (const size_t tube : order)
        {
            const double delta = std::abs(disp[tube]);
            if (delta < 1.0e-9)
            {
                q_cur[tube] = q_to[tube];
                continue;
            }
            const size_t nIncrements = static_cast<size_t>(std::ceil(delta / safeStep));
            const double inc = delta / static_cast<double>(nIncrements);
            const double sign = (disp[tube] >= 0.0) ? 1.0 : -1.0;
            for (size_t n = 1; n <= nIncrements; ++n)
            {
                q_cur[tube] += sign * inc;
                if (n == nIncrements)
                    q_cur[tube] = q_to[tube];
                wps.push_back(q_cur);
            }
        }
        return wps;
    };

    // Proportional: tube velocities scaled so both reach the goal together.
    auto makeProportional = [&]() -> WaypointsT
    {
        WaypointsT wps;
        wps.push_back(q_from);
        const double maxDisp = std::max(std::abs(disp[0]), std::abs(disp[1]));
        if (maxDisp < 1.0e-9)
            return wps;
        const size_t nIncrements = static_cast<size_t>(std::ceil(maxDisp / safeStep));
        JointVector<controlInputs> q_cur(q_from);
        for (size_t n = 1; n <= nIncrements; ++n)
        {
            const double t = static_cast<double>(n) / static_cast<double>(nIncrements);
            for (size_t i = 0; i < 2; ++i)
                q_cur[i] = q_from[i] + t * disp[i];
            wps.push_back(q_cur);
        }
        return wps;
    };

    std::vector<Candidate<controlInputs>> candidates;
    candidates.push_back({"synchronized (least-travel stops first)", makeSynchronized()});
    candidates.push_back({"sequential (beta1 first)", makeSequential(0)});
    candidates.push_back({"sequential (beta2 first)", makeSequential(1)});
    candidates.push_back({"proportional rates", makeProportional()});
    return candidates;
}

// Swept cost of a schedule: sweptCost(a, b) summed over ~40 coarse segments
// so all candidates are compared at the same discretization.
template <size_t controlInputs>
double scheduleSweptCost(
    const Waypoints<controlInputs> &wps,
    const std::function<double(const JointVector<controlInputs> &, const JointVector<controlInputs> &)> &sweptCost)
{
    if (wps.size() < 2 || !sweptCost)
        return 0.0;
    const size_t stride = std::max<size_t>(1, wps.size() / 40);
    double cost = 0.0;
    size_t prev = 0;
    for (size_t i = stride; i < wps.size(); i += stride)
    {
        cost += sweptCost(wps[prev], wps[i]);
        prev = i;
    }
    if (prev != wps.size() - 1)
        cost += sweptCost(wps[prev], wps.back());
    return cost;
}

}  // namespace deployment_schedule

#endif  // DEPLOYMENT_SCHEDULE_HPP_

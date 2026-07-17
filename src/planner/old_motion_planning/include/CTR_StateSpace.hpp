#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <cmath>

// 4-DOF state space for a 2-tube Concentric Tube Robot (CTR).
//
// Tube 3 (outermost) is static and unactuated; only Tube 1 (innermost)
// and Tube 2 (intermediate) are actuated.
//
// State layout:
//   values[0] = β₁  (prismatic joint, Tube 1)
//   values[1] = β₂  (prismatic joint, Tube 2)
//   values[2] = α₁  (revolute joint,  Tube 1)
//   values[3] = α₂  (revolute joint,  Tube 2)
//
// Dimensions 0-1 are prismatic joints: standard Euclidean metric and
// linear interpolation.
// Dimensions 2-3 are revolute joints: shortest-path angular distance and
// interpolation, so the planner correctly handles wrap-around at ±π.
class CTRStateSpace : public ompl::base::RealVectorStateSpace
{
public:
    CTRStateSpace() : ompl::base::RealVectorStateSpace(4) {}

    // Distance: Euclidean for prismatic dims, shortest-path for revolute dims.
    double distance(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        const auto *rv1 = s1->as<StateType>();
        const auto *rv2 = s2->as<StateType>();
        double sum = 0.0;

        // Prismatic (dims 0-1): standard Euclidean
        for (unsigned int i = 0; i < 2; ++i)
        {
            const double d = rv1->values[i] - rv2->values[i];
            sum += d * d;
        }

        // Revolute (dims 2-3): wrap difference to (-π, π]
        for (unsigned int i = 2; i < 4; ++i)
        {
            double d = rv1->values[i] - rv2->values[i];
            while (d >  M_PI) d -= 2.0 * M_PI;
            while (d < -M_PI) d += 2.0 * M_PI;
            sum += d * d;
        }

        return std::sqrt(sum);
    }

    // Interpolation: linear for prismatic, shortest-arc for revolute.
    void interpolate(const ompl::base::State *from, const ompl::base::State *to,
                     double t, ompl::base::State *state) const override
    {
        const auto *f = from->as<StateType>();
        const auto *g = to->as<StateType>();
        auto       *r = state->as<StateType>();

        // Prismatic: linear interpolation
        for (unsigned int i = 0; i < 2; ++i)
            r->values[i] = f->values[i] + t * (g->values[i] - f->values[i]);

        // Revolute: shortest-arc interpolation
        for (unsigned int i = 2; i < 4; ++i)
        {
            double diff = g->values[i] - f->values[i];
            while (diff >  M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            r->values[i] = f->values[i] + t * diff;
        }
    }
};

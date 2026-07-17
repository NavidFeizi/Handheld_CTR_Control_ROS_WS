#pragma once

#include <ompl/util/RandomNumbers.h>
#include <blaze/Math.h>
#include <cmath>

// Samples a point uniformly at random inside a 3D ball of the given radius
// centered at `center` using rejection sampling (max 30 attempts).
// Falls back to `center` if all attempts miss the ball.
//
// Takes the caller's RNG by reference so no new RNG is constructed on each
// call (avoids repeated seed initialization overhead in hot sampling loops).
inline void randomPointInBall(ompl::RNG &rng,
                               const blaze::StaticVector<double, 3UL> &center,
                               double radius,
                               blaze::StaticVector<double, 3UL> &out)
{
    const double r2 = radius * radius;
    for (int attempt = 0; attempt < 30; ++attempt)
    {
        const double rx = rng.uniformReal(-radius, radius);
        const double ry = rng.uniformReal(-radius, radius);
        const double rz = rng.uniformReal(-radius, radius);
        if (rx * rx + ry * ry + rz * rz <= r2)
        {
            out[0] = center[0] + rx;
            out[1] = center[1] + ry;
            out[2] = center[2] + rz;
            return;
        }
    }
    out = center;
}

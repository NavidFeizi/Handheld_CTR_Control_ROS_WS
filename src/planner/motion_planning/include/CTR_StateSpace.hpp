#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>

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
// Deliberately a PLAIN RealVectorStateSpace, with NO angular wrap in distance()
// or interpolate(). This used to override both with shortest-arc semantics
// "so the planner correctly handles wrap-around at ±π" -- it did the opposite:
// the box bounds stayed a hard interval, so the metric reported a state on the
// far side of a bound seam as "adjacent" while every interpolated edge toward
// it was pushed into the bound wall and rejected. The tree was systematically
// pulled into a wall it could never cross, exactly for goals across the seam.
//
// The correct topology here IS the box: joint values on the wire are absolute
// motor angles (the drives travel ±2.5π on α₁ and ±1.5π on α₂ within the
// dataset box), so a 2π-shifted representation is a genuinely different motor
// state a full physical turn away. Euclidean distance is the true travel cost;
// 2π-equivalence is handled where it belongs -- when the goal representative is
// chosen (ctr_kinematics_pinn::nearestGoalRepresentative), not in the metric.
class CTRStateSpace : public ompl::base::RealVectorStateSpace
{
public:
    CTRStateSpace() : ompl::base::RealVectorStateSpace(4) {}
};

// joint_box.hpp
#pragma once

#include <algorithm>
#include <cmath>

// Absolute joint box for the 4-DoF CTR from a model exposing the trained
// envelopes. Mirrors planner/motion_planning/include/Planner.hpp:378-395.
template <class Model, class Vec4>
void absoluteJointBox(const Model &model, Vec4 &q_min, Vec4 &q_max)
{
  const auto pr = model.getPrismaticJointRanges(); // [b1min,b1max,b2min,b2max] absolute
  const auto rr = model.getRevoluteJointRanges();  // [a1min,a1max,a2min,a2max] absolute
  q_min = {pr[0], pr[2], std::max(rr[0], -2.0 * M_PI), rr[2]};
  q_max = {pr[1], pr[3], std::min(rr[1],  2.0 * M_PI), rr[3]};
}

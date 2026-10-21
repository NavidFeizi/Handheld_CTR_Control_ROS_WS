#ifndef CTR_COMMON__PLANNER_PROTOCOL_HPP_
#define CTR_COMMON__PLANNER_PROTOCOL_HPP_

// Response messages of the planner/command service that the manager must
// recognise exactly; shared so both sides compare against the same literal.

namespace ctr_common::planner_protocol
{

/// replanDeployment was not run because the planning force equals the force
/// the active plan was computed with; the current plan stays valid.
inline constexpr char kReplanForceUnchanged[] = "replanDeployment skipped: planning force unchanged";

}  // namespace ctr_common::planner_protocol

#endif  // CTR_COMMON__PLANNER_PROTOCOL_HPP_

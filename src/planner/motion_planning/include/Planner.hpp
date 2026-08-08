#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>

// NEW PLANNERS THAT I'M INCLUDING NOW
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>
#include <ompl/geometric/planners/rrt/TRRTstar.h>
#include <ompl/geometric/planners/rrt/ATRRT.h>

#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>
// For handling/writing/reading files
#include <fstream>
#include <thread>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <array>
#include <algorithm>
#include <blaze/Math.h>
#include <cmath>
#include <filesystem>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
// For accessing information about the CTR object for whom we're planning
#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
#include "DeploymentSchedule.hpp"

// My own classes that I implemented
#include "CTR_StateSpace.hpp"
#include "CTR_StateSampler.hpp"
#include "CTR_StateValidityChecker.hpp"
#include "CTR_RevoluteJointObjective.hpp"
#include "CTR_BackboneLengthObjective.hpp"
#include "CTR_ClearanceObjective.hpp"
#include "CTR_FollowTheLeaderObjective.hpp"
#include "CTR_DiscreteMotionValidator.hpp"
#include "CTR_PINNsInformedSampler.hpp"
#include "CTR_PINNsInformedStateSamplerInformed.hpp"
#include "InformedSamplerGuard.hpp"
#include "Logging.hpp"

// Helper: wrap angular difference to [-pi, pi]
static inline double shortestAngleDiff(double a, double b)
{
	double d = a - b;
	while (d > M_PI)
		d -= 2.00 * M_PI;
	while (d < -M_PI)
		d += 2.00 * M_PI;
	return d;
}

// Centralized composite parameters shared between objective and diagnostics
struct CompositeParams
{
	static constexpr double w_backbone = 3.00;
	static constexpr double w_revolute = 15.00;
	static constexpr double k_coupling = 1.50;

	// Flat (non-directional) angle-error penalty. The exponential front-loading
	// has been removed because it penalised early-p states with revolute error —
	// exactly the states that must exist on a "rotate-first" path. Keeping a
	// constant weight preserves the error signal without fighting the rear-loaded
	// revolute objective.
	static constexpr double w_mismatch = 225.00;

	// Soft rear gate: penalise any remaining angle error once prismatic is past
	// p_gate. Much stronger than the old value so it creates a real barrier.
	static constexpr double p_gate = 0.15; // 0.25
	static constexpr double angle_gate_thresh = 0.0175; // ~1 degree
	static constexpr double w_gate = 200.0;

	// Ordering constraint: revolute joints must be fully aligned by the time
	// prismatic progress reaches revolute_target_fraction.  Required alignment
	// at progress p is  req(p) = min(1, p / revolute_target_fraction).
	// When the actual alignment is below req(p) a quadratic penalty fires.
	// At p = 0 the required alignment is 0, so the start state is never penalised.
	static constexpr double revolute_target_fraction = 0.25;
	static constexpr double w_ordering = 500000.0;
	static constexpr double ordering_power = 2.0;

	// Transition-based ordering weight for CompositeStateCostIntegral::motionCost().
	// For an edge advancing prismatic progress by Δp while normalised revolute error
	// is ε, the penalty is  w_transition · Δp · ε².
	// Summed over a fully mis-ordered ("prismatic first") path this totals w_transition;
	// for a "revolute first" path it is ≈ 0 (Δp ≈ 0 during the rotation phase).
	// Choose a value comfortably above the extra backbone-integral cost of keeping
	// prismatic fixed while rotating (~1–3 M for typical joint ranges here).
	static constexpr double w_transition = 5000000.0;

	// Per-state ordering penalty weight.  Added to stateCost() as  w_ordering_state * p * ε²
	// where p is prismatic progress (0→1) and ε is normalised revolute angle error (0→1).
	//
	// Why this works where motionCost() alone doesn't:
	// motionCost() penalties are invisible to the cost-to-go heuristic and informed sampler.
	// Putting the penalty in stateCost() makes simultaneous-path tree nodes genuinely more
	// expensive in cumulative cost, so RRT* rewires them away and the informed sampler's
	// ellipsoid naturally focuses on the revolute-first region.
	//
	// Correctly orders all path types:
	//   Revolute-first: p≈0 during rotation → penalty≈0.  Prismatic-only after: ε≈0 → penalty≈0.
	//   Simultaneous: integral ≈ w_ordering_state * L / 12 (L = arc length ≈ 4.14).
	//
	// Tuning: increase to demand earlier revolute convergence.
	static constexpr double w_ordering_state = 10000000.00;
};

// Close any remaining open scope introduced by local classes/lambdas above.
// (Balancing brace added to fix a missing closing brace introduced during edits.)

// Cost-to-go heuristic suitable for state-space path-length optimization.
// Assumes states are RealVectorStateSpace states and indices:
//  0..2 -> prismatic (linear), 3..5 -> revolute (angles)
static inline ompl::base::Cost ctrStateSpaceCostToGo(const ompl::base::State *s1, const ompl::base::State *s2)
{
	using StateType = ompl::base::RealVectorStateSpace::StateType;
	auto *x = s1->as<StateType>();
	auto *y = s2->as<StateType>();
	if (!x || !y)
		return ompl::base::Cost(0.00);

	double sum = 0.00;
	// prismatic indices (0,1)
	for (std::size_t i = 0; i < 2; ++i)
	{
		const double d = x->values[i] - y->values[i];
		sum += d * d;
	}
	// revolute indices (2,3)
	for (std::size_t i = 2; i < 4; ++i)
	{
		const double d = shortestAngleDiff(x->values[i], y->values[i]);
		sum += d * d;
	}
	return ompl::base::Cost(std::sqrt(sum));
}

// Cost-to-go adapter that matches OMPL's CostToGoHeuristic signature:
// Cost(const State*, const Goal*)
static inline ompl::base::Cost ctrCostToGoal(const ompl::base::State *state, const ompl::base::Goal *goal)
{
	using namespace ompl::base;

	if (!goal)
		return Cost(0.00);

	// If the goal is a single state, use that state directly
	if (goal->hasType(GoalType::GOAL_STATE))
	{
		const GoalState *gs = goal->as<GoalState>();
		const State *gstate = gs->getState();
		if (!gstate)
			return Cost(0.00);
		return ctrStateSpaceCostToGo(state, gstate);
	}

	// If the goal is a set of states, return the minimum cost-to-go to any of them
	if (goal->hasType(GoalType::GOAL_STATES))
	{
		const GoalStates *gss = goal->as<GoalStates>();
		std::size_t count = gss->getStateCount();
		if (count == 0)
			return Cost(0.00);
		double best = std::numeric_limits<double>::infinity();
		for (std::size_t i = 0; i < count; ++i)
		{
			const State *gs = gss->getState(i);
			if (!gs)
				continue;
			double val = ctrStateSpaceCostToGo(state, gs).value();
			if (val < best)
				best = val;
		}
		return Cost(best == std::numeric_limits<double>::infinity() ? 0.00 : best);
	}

	return Cost(0.00);
}

template <std::size_t controlInputs>
class Planner
{
public:
	using JointVector = blaze::StaticVector<double, controlInputs>;

	enum class optimalPlanner
	{
		PLANNER_AITSTAR,
		PLANNER_BFMTSTAR,
		PLANNER_BITSTAR,
		PLANNER_CFOREST,
		PLANNER_FMTSTAR,
		PLANNER_INF_RRTSTAR,
		PLANNER_PRMSTAR,
		PLANNER_RRTSTAR,
		PLANNER_SORRTSTAR,
		PLANNER_PRM,
		PLANNER_SPARS,
		PLANNER_RRT,
		PLANNER_RRT_CONNECT,
		PLANNER_RRT_SHARP,
		PLANNER_RLRT,
		PLANNER_AORRTC,
		PLANNER_TRRTSTAR,
		PLANNER_ATRRT
	};

	enum class planningObjective
	{
		OBJECTIVE_PATH_CLEARANCE,
		OBJECTIVE_PATH_LENGTH,
		OBJECTIVE_REVJOINTS_AND_BACKBONE,
		OBJECTIVE_PATH_LENGTH_COST2GO,
		OBJECTIVE_THRESHOLD_PATH_LENGTH,
		OBJECTIVE_WEIGHTED_COMBO,
		OBJECTIVE_BACKBONE_LENGTH,
		OBJECTIVE_REVJOINTS_ONLY,  // pure revolute-error objective; used in Phase 1 of planTwoPhase()
		OBJECTIVE_FOLLOW_THE_LEADER,   // swept-volume (FTL) edge cost only, evaluated on the loaded robot
		OBJECTIVE_REVJOINTS_AND_FTL    // revolute error + FTL swept volume; force-aware Phase 1 objective
	};

	explicit Planner(PINNs<controlInputs> &CTR_model);

	// Registers the external tip force/contact load with the planner and
	// propagates it to every force-aware component (FTL objective, informed
	// samplers, heuristics). Call BEFORE solveInverseKinematics()/plan().
	void setCTR_externalForce(const blaze::StaticVector<double, 3UL> &force);

	// Planner-mediated inverse kinematics: routes the task-space target through
	// the kinematics solver with the planner's registered external force so the
	// resulting goal configuration is consistent with the loaded (deflected)
	// robot. Returns true if the tip error is within posTol.
	bool solveInverseKinematics(JointVector &q, const blaze::StaticVector<double, 3UL> &targetTip, double posTol);
	bool resetStartState(const JointVector &q0);
	bool resetGoalState(const JointVector &qf);
	bool resetStartAndGoalStates(const JointVector &q0, const JointVector &qf);
	bool setStartState(const JointVector &q_0);
	bool setGoalState(const JointVector &q_f);
	bool plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, double rangeOverride = 0.0);

	// Two-phase planner that structurally enforces revolute-before-prismatic ordering.
	//
	// Phase 1: plan start → (prismatic_start, revolute_goal) using OBJECTIVE_REVJOINTS_ONLY.
	//          Revolute joints converge fully while prismatic joints remain at their start values.
	// Phase 2: plan (prismatic_start, revolute_goal) → goal using OBJECTIVE_BACKBONE_LENGTH.
	//          Prismatic joints deploy with revolute already at target.
	//
	// The two paths are concatenated and exposed via writeSolutionToFile / analyzeSolution
	// exactly as if plan() had been called once.
	// phase2LinearStep: maximum prismatic increment (metres) per waypoint in Phase 2.
	// Smaller values produce finer motion resolution at the cost of a larger path.
	bool planTwoPhase(double runTime, optimalPlanner plannerType, double phase2RangeFactor = 0.2, double phase2LinearStep = 1.0e-3);

	// Analytic re-schedule of the prismatic deployment (Phase 2 only) from
	// q_start to q_goal under the currently registered external force — call
	// setCTR_externalForce() first. No OMPL solve is performed, so this is fast
	// enough for mid-deployment replanning.
	//
	// The revolute joints of q_start and q_goal must agree within alphaTol
	// (radians): the schedules hold alpha constant at the goal values, and a
	// mismatch beyond alphaTol is rejected (returns false) rather than silently
	// rotating deployed tubes — that requires a full planTwoPhase().
	//
	// On success the winning schedule becomes the solution path in the problem
	// definition, so writeSolutionToFile()/analyzeSolution() work unchanged. On
	// failure the solution paths are cleared and the planner status is set
	// non-EXACT, so a stale previous solution can never be re-exported.
	// Must not run concurrently with plan()/replan()/planTwoPhase().
	bool planDeployment(const JointVector &q_start, const JointVector &q_goal, double linearStep = 1.0e-3, double alphaTol = 0.05);

	// Diagnostics for the most recent analytic deployment selection
	// (planTwoPhase Phase 2 or planDeployment).
	const std::string &lastDeploymentScheduleName() const { return m_lastScheduleName; }
	double lastDeploymentCost() const { return m_lastScheduleCost; }

	bool replan(double runTime);
	ompl::base::PlannerStatus getPlannerStatus() const { return m_solved; }
	void writeSolutionToFile(const std::string &outputFile);
	void analyzeSolution(bool dumpCSV, const std::string &csvPath);
	void cleanup();

	ompl::base::OptimizationObjectivePtr getRevoluteJointObjective(const std::array<double, 2UL> &goal);
	ompl::base::OptimizationObjectivePtr getPathLengthObjective();
	ompl::base::OptimizationObjectivePtr getThresholdPathLengthObjctive();
	ompl::base::OptimizationObjectivePtr getClearanceObjective();
	ompl::base::OptimizationObjectivePtr RevoluteJointsAndBackboneLengthObjective(const std::array<double, 2UL> &goal);
	ompl::base::OptimizationObjectivePtr getBalancedObjective();
	ompl::base::OptimizationObjectivePtr getPathLengthObjWithCostToGo();
	ompl::base::OptimizationObjectivePtr getBackboneLengthObjective();
	ompl::base::OptimizationObjectivePtr getFollowTheLeaderObjective();
	ompl::base::OptimizationObjectivePtr getRevoluteJointsAndFTLObjective(const std::array<double, 2UL> &goal);

private:
	ompl::base::PlannerPtr allocatePlanner(optimalPlanner plannerType);
	ompl::base::OptimizationObjectivePtr allocateObjective(planningObjective objectiveType);

	// ----- Phase-2 deployment-schedule machinery (shared by planTwoPhase and planDeployment) -----
	using Waypoints = std::vector<JointVector>;
	struct DeploymentCandidate
	{
		const char *name;
		Waypoints wps;
	};
	// Builds the four candidate prismatic schedules from q_from to q_to.
	// q_from supplies the starting β values and the α values held throughout;
	// q_to supplies the goal β values.
	std::vector<DeploymentCandidate> buildDeploymentCandidates(const JointVector &q_from, const JointVector &q_to, double safeStep) const;
	bool scheduleIsValid(const Waypoints &wps) const;
	double scheduleSweptCost(const Waypoints &wps) const;
	// Validity-filters and FTL-scores the candidates; lowest cost wins. Returns
	// nullptr when none is valid and fallbackToSynchronized is false; with the
	// fallback enabled, the synchronized schedule is returned unvalidated
	// (planTwoPhase's historical behavior).
	const Waypoints *selectBestSchedule(const std::vector<DeploymentCandidate> &candidates, bool fallbackToSynchronized,
	                                    const char *&bestNameOut, double &bestCostOut) const;

	PINNs<controlInputs> &m_CTR_model;
	blaze::StaticVector<double, 3UL> m_externalForce;
	ompl::base::StateSpacePtr m_space;
	std::shared_ptr<ompl::base::RealVectorBounds> m_coordBound;
	ompl::base::SpaceInformationPtr m_si;
	ompl::base::StateValidityCheckerPtr m_stateValidityChecker;
	ompl::base::ProblemDefinitionPtr m_pdef;
	ompl::base::PlannerPtr m_optimizingPlanner;
	ompl::base::PlannerStatus m_solved = ompl::base::PlannerStatus::UNKNOWN;
	std::shared_ptr<ompl::base::ScopedState<>> m_startState;
	std::shared_ptr<ompl::base::ScopedState<>> m_goalState;
	ompl::base::StateSamplerPtr m_informedSampler;
	std::shared_ptr<CTR_StateSampler<controlInputs>> m_fallbackSampler;
	std::shared_ptr<CTR_FollowTheLeaderObjective<controlInputs>> m_ftlObjective;
	double m_informedTipRadius = 0.005;
	unsigned int m_informedMaxAttempts = 1000U;
	std::string m_lastScheduleName;
	double m_lastScheduleCost = 0.0;
};

template <size_t controlInputs>
Planner<controlInputs>::Planner(PINNs<controlInputs> &CTR_model)
	: m_externalForce(0.0), m_CTR_model(CTR_model)
{
	// CTRStateSpace uses Euclidean distance for prismatic dims and
	// shortest-path angular distance/interpolation for revolute dims,
	// so the planner correctly handles angle wrap-around.
	m_space = std::make_shared<CTRStateSpace>();

	m_coordBound = std::make_shared<ompl::base::RealVectorBounds>(controlInputs);

	// Use getInputPosBounds() which properly handles the 4-DoF layout:
	//   lb/ub[0] = beta1, lb/ub[1] = beta2, lb/ub[2] = alpha1, lb/ub[3] = alpha2
	// Do NOT use getPrismaticJointRanges() / getRevoluteJointRanges() —
	// both are broken for controlInputs=4 (hardcoded 6-element initializer lists).
	const auto [lb, ub] = CTR_model.getInputPosBounds();

	// beta_1
	m_coordBound->setLow(0, lb[0UL]);
	m_coordBound->setHigh(0, ub[0UL]);
	// beta_2
	m_coordBound->setLow(1, lb[1UL]);
	m_coordBound->setHigh(1, ub[1UL]);
	// alpha_1
	m_coordBound->setLow(2, lb[2UL]);
	m_coordBound->setHigh(2, ub[2UL]);
	// alpha_2
	m_coordBound->setLow(3, lb[3UL]);
	m_coordBound->setHigh(3, ub[3UL]);

	// setting the space bounds for the linear (prismatic) CTR joints
	m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(*m_coordBound);

	// Construct a space information instance for this compound state space
	m_si = std::make_shared<ompl::base::SpaceInformation>(m_space);

	// Set the object used to check which states in the space are valid
	m_stateValidityChecker = std::make_shared<CTR_StateValidityChecker<controlInputs>>(this->m_si, m_CTR_model);

	// Create the state validity checker with the CTR instance
	m_si->setStateValidityChecker(m_stateValidityChecker);

	// Enable discrete motion validator for edge checking (deployment monotonicity & ordering)
	m_si->setMotionValidator(std::make_shared<CTR_DiscreteMotionValidator<controlInputs>>(m_si, m_CTR_model));
	// Defines the segment granularity for the discrete motion validator.
	// getMaximumExtent() is dominated by the revolute joints (≈ sqrt(3*(2π)²) ≈ 10.88)
	// so the segment length = fraction × extent. Using 3e-4 keeps the segment length
	// at the same scale as defaultRange (also 3e-4 × extent), ensuring each planned
	// motion receives at least one intermediate validity check from
	// CTR_DiscreteMotionValidator (monotonicity, β ordering, etc.).
	// A value of 3e-3 would make segments 10× longer than the range,
	// meaning validSegmentCount = 1 always and intermediate checks never fire.
	m_si->getStateSpace()->setLongestValidSegmentFraction(3.00e-4); //(3e-4);

	// setup the space information from the Real state space
	this->m_si->setup();

	// setting up the pointer to the start state
	this->m_startState = std::make_shared<ompl::base::ScopedState<>>(m_space);

	// setting up the pointer to the goal state
	this->m_goalState = std::make_shared<ompl::base::ScopedState<>>(m_space);

	// create an instance of a planning problem
	this->m_pdef = std::make_shared<ompl::base::ProblemDefinition>(this->m_si);

	// Follow-the-leader swept-volume objective. Created once and shared by all
	// planning objectives so the external force is registered in one place.
	this->m_ftlObjective = std::make_shared<CTR_FollowTheLeaderObjective<controlInputs>>(this->m_si, m_CTR_model, m_externalForce);

	// external force acting on the distal-end of the CTR (propagates to the
	// FTL objective and any force-aware samplers)
	this->setCTR_externalForce(m_externalForce);
}

template <size_t controlInputs>
bool Planner<controlInputs>::resetStartState(const blaze::StaticVector<double, controlInputs> &q0)
{
	// Update our internal start‐state storage
	auto *stored = m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>();

	for (size_t i = 0; i < controlInputs; ++i)
		stored->values[i] = q0[i];

	// Validate start state
	if (!m_stateValidityChecker->isValid(stored))
		throw std::runtime_error("New start state is invalid!");

	// Clear out the old start, add the new one
	m_pdef->clearStartStates();

	// ProblemDefinition copies the provided state; no need to clone
	m_pdef->addStartState(stored);

	// Reset planner’s internal query & paths
	if (m_optimizingPlanner)
	{
		m_optimizingPlanner->clearQuery();
		m_pdef->clearSolutionPaths();
		m_optimizingPlanner->setProblemDefinition(m_pdef);

		// Re‐setup so everything is internally consistent
		if (!m_optimizingPlanner->isSetup())
    		m_optimizingPlanner->setup();
	}

	std::cout << "Start state has been updated!\n";
	return true;
}

template <size_t controlInputs>
bool Planner<controlInputs>::resetGoalState(const blaze::StaticVector<double, controlInputs> &qf)
{
	if (!m_goalState || !m_goalState->get())
		throw std::runtime_error("resetGoalState() called before a goal state was allocated. Re-run plan() or construct a new Planner after cleanup().");
	if (!m_pdef)
		throw std::runtime_error("resetGoalState() called after the problem definition was cleared. Rebuild the planner before resetting goals.");
	if (!m_stateValidityChecker)
		throw std::runtime_error("resetGoalState() requires an active state validity checker. Did you call cleanup()?");

	auto *stored = m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
	for (size_t i = 0; i < controlInputs; ++i)
		stored->values[i] = qf[i];

	if (!m_si->satisfiesBounds(stored))
		throw std::runtime_error("New goal state violates state space bounds (check alpha values)!");

	if (!m_stateValidityChecker->isValid(stored))
		throw std::runtime_error("New goal state is invalid!");

	// Replace existing goal with the updated state
	m_pdef->clearGoal();
	// Use a practical joint-space tolerance so the planner can declare an
	// exact solution when it gets close to the goal (default epsilon is ~1e-16,
	// which is impossible to satisfy exactly in floating-point arithmetic).
	m_pdef->setGoalState(stored, 1.00E-3);

	// Update helpers that rely on the goal betas/alphas
	if (m_si)
	{
		if (auto motionValidator = std::dynamic_pointer_cast<CTR_DiscreteMotionValidator<controlInputs>>(m_si->getMotionValidator()))
			motionValidator->setGoalBetas(qf[0UL], qf[1UL]);
	}

	if (auto validityChecker = std::dynamic_pointer_cast<CTR_StateValidityChecker<controlInputs>>(m_si ? m_si->getStateValidityChecker() : ompl::base::StateValidityCheckerPtr()))
		validityChecker->setGoalBetas(qf[0UL], qf[1UL]);

	if (this->m_informedSampler)
	{
		if (m_fallbackSampler)
			m_fallbackSampler->setGoalAngles(qf[2UL], qf[3UL]);

		if (auto inf = std::dynamic_pointer_cast<CTR_PINNsInformedSampler<controlInputs>>(this->m_informedSampler))
			inf->setGoalState(m_goalState->get());
		else if (auto inf2 = std::dynamic_pointer_cast<CTR_PINNsInformedStateSampler<controlInputs>>(this->m_informedSampler))
			inf2->setGoalState(m_goalState->get());
	}

	if (this->m_optimizingPlanner)
	{
		this->m_optimizingPlanner->clearQuery();
		this->m_pdef->clearSolutionPaths();
		this->m_optimizingPlanner->setProblemDefinition(this->m_pdef);
		if (!m_optimizingPlanner->isSetup())
    		m_optimizingPlanner->setup();
	}

	std::cout << "Goal state has been updated!\n";
	return true;
}

template <size_t controlInputs>
bool Planner<controlInputs>::resetStartAndGoalStates(const JointVector &q0, const JointVector &qf)
{
	bool startSet = this->resetStartState(q0);
	bool goalSet = this->resetGoalState(qf);
	return startSet && goalSet;
}

template <size_t controlInputs>
bool Planner<controlInputs>::setStartState(const blaze::StaticVector<double, controlInputs> &q_0)
{
	// setting the start state for the CTR robot
	auto *state = m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
	for (size_t i = 0; i < controlInputs; ++i)
		state->values[i] = q_0[i]; // Simplified assignment loop

	if (m_stateValidityChecker->isValid(state))
	{
		this->m_pdef->clearStartStates();
		// ProblemDefinition copies the provided state; do not clone to avoid leaks
		this->m_pdef->addStartState(state);

		std::cout << "Start state has been updated!" << std::endl;
		return true;
	}
	else
	{
		throw std::runtime_error("Start State is invalid!");
		return false;
	}
}

template <size_t controlInputs>
bool Planner<controlInputs>::setGoalState(const blaze::StaticVector<double, controlInputs> &q_f)
{
	// setting the start state for the CTR robot
	auto *state = m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
	for (size_t i = 0; i < controlInputs; ++i)
		state->values[i] = q_f[i]; // Simplified assignment

	if (!m_si->satisfiesBounds(state))
		throw std::runtime_error("Goal state violates state space bounds (check alpha values)!");

	if (m_stateValidityChecker->isValid(state))
	{
		m_pdef->clearGoal();
		// ProblemDefinition copies the provided state; do not clone to avoid leaks
		// Use a practical joint-space tolerance so the planner can declare an
		// exact solution when it gets close to the goal.
		m_pdef->setGoalState(state, 1.00E-3);
		std::cout << "Goal state has been updated!" << std::endl;

		// Cast the motion validator and set goal betas:
		auto motion_validator = std::dynamic_pointer_cast<CTR_DiscreteMotionValidator<controlInputs>>(m_si->getMotionValidator());
		// Cast the state sampler validator and set goal betas:
		auto validity_checker = std::dynamic_pointer_cast<CTR_StateValidityChecker<controlInputs>>(m_si->getStateValidityChecker());

		if (motion_validator && validity_checker)
		{
			motion_validator->setGoalBetas(q_f[0UL], q_f[1UL]);
			validity_checker->setGoalBetas(q_f[0UL], q_f[1UL]);
		}

		// Also propagate goal revolute angles to CTR_StateSampler for rotation-focused sampling bias
		if (m_fallbackSampler)
			m_fallbackSampler->setGoalAngles(q_f[2UL], q_f[3UL]);

		// If we have created an informed sampler earlier, update its goal so it
		// can compute the goal tip position for informed sampling.
		if (this->m_informedSampler)
		{
			// Try updating both possible informed sampler implementations
			auto inf = std::dynamic_pointer_cast<CTR_PINNsInformedSampler<controlInputs>>(this->m_informedSampler);
			if (inf)
			{
				inf->setGoalState(m_goalState->get());
			}
			else
			{
				auto inf2 = std::dynamic_pointer_cast<CTR_PINNsInformedStateSampler<controlInputs>>(this->m_informedSampler);
				if (inf2)
				{
					inf2->setGoalState(m_goalState->get());
				}
			}
		}

		// Reinitialize planner internal query/state now that the goal changed
		if (this->m_optimizingPlanner)
		{
			this->m_optimizingPlanner->clearQuery();
			this->m_pdef->clearSolutionPaths();
			this->m_optimizingPlanner->setProblemDefinition(this->m_pdef);
			if (!this->m_optimizingPlanner->isSetup())
				this->m_optimizingPlanner->setup();
		}

		return true;
	}
	else
	{
		throw std::runtime_error("Goal State is invalid!");
		return false;
	}
}

template <size_t controlInputs>
bool Planner<controlInputs>::plan(const double runTime, optimalPlanner plannerType, planningObjective objectiveType, const double rangeOverride)
{
	// Basic sanity: ensure start and goal are present before planning
	if (!this->m_pdef || this->m_pdef->getStartStateCount() == 0 || !this->m_pdef->getGoal())
		throw std::runtime_error("Start or goal not set before calling plan()");

	// Guard: feasibility-only planners find a path but completely ignore the
	// optimization objective. Warn loudly so the mismatch is not silent.
	const bool isFeasibilityPlanner =
		plannerType == optimalPlanner::PLANNER_RRT         ||
		plannerType == optimalPlanner::PLANNER_RRT_CONNECT ||
		plannerType == optimalPlanner::PLANNER_RLRT        ||
		plannerType == optimalPlanner::PLANNER_PRM         ||
		plannerType == optimalPlanner::PLANNER_AORRTC;
	const bool wantsOptimization = objectiveType != planningObjective::OBJECTIVE_PATH_LENGTH;
	if (isFeasibilityPlanner && wantsOptimization)
		std::cerr << "[Planner] WARNING: a feasibility-only planner was selected with an "
				  << "optimization objective. The objective will be ignored. "
				  << "Use PLANNER_RRTSTAR, PLANNER_INF_RRTSTAR, PLANNER_BITSTAR, or "
				  << "PLANNER_AITSTAR for optimal planning.\n";

	// Create the optimization objective specified by the objectiveType argument
	this->m_pdef->setOptimizationObjective(this->allocateObjective(objectiveType));

	// Choose whether to enable the informed sampler for this planning run.
	// By default enable informed sampling for planners that benefit from it
	// (RRT*, InformedRRT*, CForest, BIT*, AIT*). For sampling-based planners
	// that do not use informed sampling (PRM, RRT, PRM variants) we fall back
	// to the regular CTR_StateSampler.
	auto useInformed = false;
	switch (plannerType)
	{
	case optimalPlanner::PLANNER_INF_RRTSTAR:
	case optimalPlanner::PLANNER_RRTSTAR:
	case optimalPlanner::PLANNER_SORRTSTAR:
	case optimalPlanner::PLANNER_BITSTAR:
	case optimalPlanner::PLANNER_AITSTAR:
	case optimalPlanner::PLANNER_CFOREST:
		useInformed = true;
		break;
	default:
		break;
	}
	// Ensure the chosen optimization objective actually exposes a cost-to-go
	// heuristic; informed sampling requires a cost-to-go to be available.
	if (useInformed)
	{
		auto opt = this->m_pdef->getOptimizationObjective();
		if (!opt || !opt->hasCostToGoHeuristic())
		{
			useInformed = false;
			std::cout << "Informed sampler disabled: current objective has no cost-to-go heuristic." << std::endl;
		}
	}

	// Re-install a state sampler allocator appropriate for this planner.
	// Simpler allocator: construct a CTR fallback sampler and, if informed
	// sampling is enabled, wrap it with our PINNs-based informed-state
	// sampler. This avoids complex planner-level queries to
	// OptimizationObjective::allocInformedStateSampler which previously
	// introduced parsing/templating issues during compilation.
	this->m_space->setStateSamplerAllocator([this, useInformed](const ompl::base::StateSpace *space) -> ompl::base::StateSamplerPtr
											{
			auto validityChecker = std::dynamic_pointer_cast<CTR_StateValidityChecker<controlInputs>>(m_stateValidityChecker);
			if (!validityChecker)
				throw std::runtime_error("Expected CTR_StateValidityChecker in Planner::plan() sampler allocator");

			// Fallback uniform/CTR sampler
			auto fallback = std::make_shared<CTR_StateSampler<controlInputs>>(space, *validityChecker, m_CTR_model);

			// Extract parameters from Planner's bounds and validity checker
			const double beta1_min = m_coordBound->low[0UL];
			const double beta2_min = m_coordBound->low[1UL];
			const double clearance = m_CTR_model.getStageThickness();

			// Set parameters on the fallback sampler
			fallback->setParameters(beta1_min, beta2_min, clearance);
			this->m_fallbackSampler = fallback;

			if (!useInformed)
			{
				this->m_informedSampler = fallback;
				return fallback;
			}

			// Avoid recursion during informed sampler construction
			auto informed = std::make_shared<CTR_PINNsInformedStateSampler<controlInputs>>(space, *validityChecker, m_CTR_model, fallback);

			// Register the external force before the goal so the goal tip is
			// computed on the loaded robot.
			informed->setExternalForce(this->m_externalForce);

			if (this->m_goalState && this->m_goalState->get())
			{
				informed->setGoalState(this->m_goalState->get());
			}

			informed->setTipRadius(this->m_informedTipRadius);
			informed->setMaxAttempts(this->m_informedMaxAttempts);

			this->m_informedSampler = informed;
			return informed; });

	// Re-setup the SpaceInformation so the new allocator is used to create samplers
	this->m_si->setup();

	// If we plan to use an informed sampler, estimate how useful the
	// optimization objective's cost-to-go heuristic is by sampling a few
	// states (via the fallback sampler) and comparing the objective's
	// cost-to-go to the conservative state-space baseline (ctrCostToGoal).
	// If the heuristic only provides a very small improvement we lower the
	// informed-sampler bias to avoid expensive PINNs/IK attempts.
	if (useInformed && this->m_informedSampler)
	{
		auto inf = std::dynamic_pointer_cast<CTR_PINNsInformedSampler<controlInputs>>(this->m_informedSampler);
		auto inf2 = !inf ? std::dynamic_pointer_cast<CTR_PINNsInformedStateSampler<controlInputs>>(this->m_informedSampler) : nullptr;
		if (inf || inf2)
		{
			auto opt = this->m_pdef->getOptimizationObjective();
			auto goalPtr = this->m_pdef->getGoal().get();
			if (opt && goalPtr)
			{
				// Create a temporary fallback sampler to draw unbiased samples
				auto validityChecker = std::dynamic_pointer_cast<CTR_StateValidityChecker<controlInputs>>(m_stateValidityChecker);
				if (!validityChecker)
					throw std::runtime_error("Expected CTR_StateValidityChecker when creating temporary fallback sampler");
				auto fallback = std::make_shared<CTR_StateSampler<controlInputs>>(this->m_space.get(), *validityChecker, m_CTR_model);
				// set same parameters as allocator would
				const double beta1_min = m_coordBound->low[0UL];
				const double beta2_min = m_coordBound->low[1UL];
				const double clearance = m_CTR_model.getStageThickness();
				fallback->setParameters(beta1_min, beta2_min, clearance);

				// sample a small number of states and compute relative improvement
				constexpr std::size_t N = 8;
				double sumImprovement = 0.00;
				std::size_t counted = 0;
				for (std::size_t i = 0; i < N; ++i)
				{
					auto *s = this->m_space->allocState();
					try
					{
						fallback->sampleUniform(s);
						double baseline = ctrCostToGoal(s, goalPtr).value();
						double objVal = 0.00;
						try
						{
							objVal = this->m_pdef->getOptimizationObjective()->costToGo(s, goalPtr).value();
						}
						catch (...)
						{
							objVal = baseline;
						}

						if (baseline > 1.00E-12)
						{
							double improvement = std::max(0.00, (baseline - objVal) / baseline);
							sumImprovement += improvement;
							++counted;
						}
					}
					catch (...)
					{
					}
					this->m_space->freeState(s);
				}

				double avgImprovement = (counted > 0) ? (sumImprovement / static_cast<double>(counted)) : 0.00;

				// Map average improvement to a practical bias value: weak heuristics -> low bias
				double newBias = 0.50; // default
				if (avgImprovement < 0.05)
					newBias = 0.10; // very weak
				else if (avgImprovement < 0.15)
					newBias = 0.25; // weak
				else
					newBias = 0.50; // reasonable

				if (inf)  inf->setBias(newBias);
				if (inf2) inf2->setBias(newBias);
				logging::debug("Adjusted informed-sampler bias to ", newBias, " (avg heuristic improvement=", avgImprovement, ")");
			}
		}
	}

	// Construct the optimal planner specified the plannerType argument.
	this->m_optimizingPlanner = this->allocatePlanner(plannerType);

	/*
							____ SETTING THE PLANNER'S RANGE ____

		1. The range typically refers to the maximum distance a planner is allowed to extend in the configuration space while attempting to connect two points
		2. Choosing an appropriate range is crucial for the performance of the planner. Too small of a range might result in a planner that explores the configuration space too slowly, while too large of a range might lead to inefficiencies or failure to find solutions. The optimal range often depends on the specific characteristics of the planning problem and the environment. It's usually a parameter that needs to be tuned based on experimentation and domain knowledge.
		3. In the process of randomly selecting states in the state space to attempt to go towards, the algorithm may in fact choose the actual goal state, if it knows it, with some probability. This probability is a real number between 0.0 and 1.0; its value should usually be around 0.05 and should not be too large. It is probably a good idea to use the default value.
	*/

	const double extent = m_space->getMaximumExtent();
	const double defaultRange = 3.00E-4 * extent; // 0.03% of maximum extent
	// When an explicit rangeOverride is provided (e.g. for Phase 2 of planTwoPhase()),
	// use it instead of the full-space defaultRange so the planner takes finer steps.
	const double actualRange = (rangeOverride > 0.0) ? rangeOverride : defaultRange;

	// Diagnostic prints to help debug rewiring radius computation
	logging::debug("state space dimension = ", m_space->getDimension());
	logging::debug("state space maximum extent = ", extent);
	logging::debug("defaultRange (scaled extent) = ", defaultRange);
	logging::debug("actualRange (applied to planner) = ", actualRange);
	if (m_coordBound)
	{
		std::ostringstream oss;
		oss << "coord bounds: ";
		for (std::size_t i = 0; i < m_coordBound->low.size(); ++i)
		{
			oss << "[" << m_coordBound->low[i] << "," << m_coordBound->high[i] << "] ";
		}
		logging::debug(oss.str());
	}

	switch (plannerType)
	{
	case optimalPlanner::PLANNER_RRTSTAR:
		// range represents the maximum length of a motion to be added in the tree of motions
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		this->m_optimizingPlanner->params()["delay_collision_checking"] = true;
		this->m_optimizingPlanner->params()["rewire_factor"] = 1.50; // Recommended value: 1.0 to 2.0.
		this->m_optimizingPlanner->params()["use_k_nearest"] = false;
		// 100 attempts per iteration is sufficient even with a rejecting custom sampler;
		// 5000 would make every planning step block on sampling before extending the tree.
		this->m_optimizingPlanner->params()["number_sampling_attempts"] = 100;
		this->m_optimizingPlanner->params()["tree_pruning"] = false;
		this->m_optimizingPlanner->params()["prune_threshold"] = 0.00;
		// informed_sampling is handled by our custom setStateSamplerAllocator;
		// enabling it here would make RRT* call allocInformedStateSampler() in
		// addition to the custom allocator, causing conflicting sampling paths.
		this->m_optimizingPlanner->params()["new_state_rejection"] = false;
		this->m_optimizingPlanner->params()["use_admissible_heuristic"] = true;
		break;

	case optimalPlanner::PLANNER_RLRT:
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		this->m_optimizingPlanner->params()["keep_last_valid"] = false;
		break;

	case optimalPlanner::PLANNER_RRT:
		// range represents the maximum length of a motion to be added in the tree of motions
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		this->m_optimizingPlanner->params()["intermediate_states"] = true;
		break;

	case optimalPlanner::PLANNER_RRT_CONNECT:
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["intermediate_states"] = true;
		break;

	case optimalPlanner::PLANNER_INF_RRTSTAR:
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		this->m_optimizingPlanner->params()["delay_collision_checking"] = true;
		this->m_optimizingPlanner->params()["rewire_factor"] = 1.50; // Recommended value: 1.0 to 2.0.
		this->m_optimizingPlanner->params()["use_k_nearest"] = false;
		this->m_optimizingPlanner->params()["number_sampling_attempts"] = 100;
		this->m_optimizingPlanner->params()["prune_threshold"] = 0.005;
		break;

	case optimalPlanner::PLANNER_SORRTSTAR:
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		this->m_optimizingPlanner->params()["delay_collision_checking"] = true;
		this->m_optimizingPlanner->params()["rewire_factor"] = 1.75; // Recommended value: 1.0 to 2.0.
		this->m_optimizingPlanner->params()["use_k_nearest"] = false;
		this->m_optimizingPlanner->params()["number_sampling_attempts"] = 100;
		this->m_optimizingPlanner->params()["prune_threshold"] = 0.005;
		break;

	case optimalPlanner::PLANNER_BITSTAR:
		// The number of samples to generate on each batch. OMPL default is 100.
		// 10000 would delay the first solution by 100×; 300 gives adequate 6D coverage
		// while still finding an initial path in reasonable time.
		this->m_optimizingPlanner->params()["samples_per_batch"] = 300;
		// The factor by which to increase the connection radius.
		this->m_optimizingPlanner->params()["rewire_factor"] = 1.75;
		// Whether to use k-nearest neighbors or a radius search.
		this->m_optimizingPlanner->params()["use_k_nearest"] = false;
		// The fraction of the solution cost at which to prune states.
		this->m_optimizingPlanner->params()["use_graph_pruning"] = true;
		this->m_optimizingPlanner->params()["drop_unconnected_samples_on_prune"] = true;
		this->m_optimizingPlanner->params()["delay_rewiring_to_first_solution"] = false;
		this->m_optimizingPlanner->params()["find_approximate_solutions"] = true;
		this->m_optimizingPlanner->params()["prune_threshold_as_fractional_cost_change"] = true;
		this->m_optimizingPlanner->params()["stop_on_each_solution_improvement"] = false;
		this->m_optimizingPlanner->params()["use_just_in_time_sampling"] = true;
		break;

	case optimalPlanner::PLANNER_AITSTAR:
		// The number of samples to generate on each batch.
		this->m_optimizingPlanner->params()["samples_per_batch"] = 1000;
		// The factor by which to increase the connection radius.
		this->m_optimizingPlanner->params()["rewire_factor"] = 1.75;
		// Whether to use k-nearest neighbors or a radius search.
		this->m_optimizingPlanner->params()["use_k_nearest"] = false;
		break;

	case optimalPlanner::PLANNER_BFMTSTAR:
	{
		auto &params = this->m_optimizingPlanner->params();
		// num_samples: total number of uniformly drawn states in each forward/backward frontier expansion.
		params["num_samples"] = 2000;
		// radius_multiplier: scales the connection radius relative to the theoretical optimum ( > 1 widens connectivity ).
		params["radius_multiplier"] = 1.50;
		// balanced: alternate between forward/backward trees to maintain similar frontier sizes.
		params["balanced"] = true;
		// cache_cc: cache collision-check queries so repeated edge evaluations are cheap.
		params["cache_cc"] = true;
		// extended_fmt: enable the BFMT* extension that lazily rewires when cheaper edges appear.
		params["extended_fmt"] = true;
		// heuristics: use cost-to-go heuristics to sort the wavefront and focus expansions.
		params["heuristics"] = true;
		// nearest_k: switch between radius-based (false) and k-nearest (true) neighbor selection.
		params["nearest_k"] = false;
		// optimality: keep asymptotic optimality guarantees (set false to get faster but sub-optimal results).
		params["optimality"] = true;
		break;
	}

	case optimalPlanner::PLANNER_CFOREST:
	{
		// Configure a CForest of RRT* planners. Create per-thread RRT* instances
		// and configure each instance with the problem-specific defaultRange and
		// a small set of tuned parameters so all threads behave consistently.

		auto cforest = std::make_shared<ompl::geometric::CForest>(m_si);

		// IMPORTANT: set the problem definition on the CForest before
		// creating planner instances so that addPlannerInstanceInternal
		// can propagate the ProblemDefinition to each created planner.
		cforest->setProblemDefinition(this->m_pdef);

		// Number of threads: at least 2, otherwise use hardware concurrency
		const unsigned numThreads = std::max(2u, std::thread::hardware_concurrency());

		// Use the templated helper to create the planner instances with the
		// correct wrapped SpaceInformation. Then retrieve each created
		// instance and configure it.
		cforest->addPlannerInstances<ompl::geometric::InformedRRTstar>(numThreads); // InformedRRTstar

		for (unsigned i = 0; i < numThreads; ++i)
		{
			auto &plannerPtr = cforest->getPlannerInstance(i);
			auto rrt = std::dynamic_pointer_cast<ompl::geometric::InformedRRTstar>(plannerPtr);
			if (!rrt)
				continue;

			// Configure commonly used RRT* parameters (match RRT*/InformedRRT* tuning above)
			rrt->params()["range"] = actualRange;
			rrt->params()["goal_bias"] = 0.005;
			rrt->params()["delay_collision_checking"] = true;
			rrt->params()["rewire_factor"] = 1.75; // Recommended value: 1.0 to 2.0.
			rrt->params()["use_k_nearest"] = false;
			rrt->params()["number_sampling_attempts"] = 100;
			rrt->params()["ordered_sampling"] = false;
			rrt->params()["ordering_batch_size"] = 1;
			rrt->params()["prune_threshold"] = 0.005;
		}

		cforest->setNumThreads(numThreads);
		// Focus search enabled: when any thread finds a better solution it narrows
		// all other threads' sampling to the new prolate-hyperspheroid. Disabling
		// this makes CForest behave as N independent RRT* instances with no
		// inter-thread cooperation, which defeats CForest's main advantage.
		cforest->setFocusSearch(true);
		this->m_optimizingPlanner = cforest;
		break;
	}

	case optimalPlanner::PLANNER_PRMSTAR:
		// Limit k-nearest connections to prevent near-complete O(n²) graph construction.
		// PRM* guarantees asymptotic optimality through rewiring, not dense connectivity,
		// so a moderate k (same as plain PRM) is appropriate.
		this->m_optimizingPlanner->params()["max_nearest_neighbors"] = 15;
		break;

	case optimalPlanner::PLANNER_PRM:
		// k = 15 is a reasonable k-nearest default for a 6D space; 5000 would
		// produce a near-complete graph and make the roadmap construction O(n²).
		this->m_optimizingPlanner->params()["max_nearest_neighbors"] = 15;
		break;

	case optimalPlanner::PLANNER_AORRTC:
	{
		// AORRTC does not register a "range" param via declareParam, so
		// params()["range"] is silently ignored. Call setRange() directly.
		auto aorrtc = std::dynamic_pointer_cast<ompl::geometric::AORRTC>(this->m_optimizingPlanner);
		if (aorrtc)
			aorrtc->setRange(actualRange);
		break;
	}

	case optimalPlanner::PLANNER_TRRTSTAR:
		// TRRTstar: asymptotically optimal variant of T-RRT.
		// Uses transition-based rejection (temperature / cost) on top of RRT*.
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		break;

	case optimalPlanner::PLANNER_ATRRT:
		// ATRRT: anytime asymptotically optimal T-RRT.
		// Continuously improves solution quality within the allotted time.
		this->m_optimizingPlanner->params()["range"] = actualRange;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.05;
		break;

	default:
		throw std::runtime_error("Planner not supported!");
	}

	// Set the problem instance for the specified planner to solve
	this->m_optimizingPlanner->setProblemDefinition(this->m_pdef);
	if (!m_optimizingPlanner->isSetup())
		m_optimizingPlanner->setup();

	// Print the planner properties/settings after it has been fully configured
	if (this->m_optimizingPlanner)
		this->m_optimizingPlanner->printProperties(std::cout);

	m_optimizingPlanner->printSettings(std::cout);

	// attempt to solve the planning problem in the given runtime
	// Debug: print start/goal/objective costs to diagnose zero-target issues
	{
		auto opt = this->m_pdef->getOptimizationObjective();
		const ompl::base::Goal *goal = this->m_pdef->getGoal().get();
		if (opt && goal)
		{
			try
			{
				const ompl::base::State *start = nullptr;
				try
				{
					start = this->m_pdef->getStartState(0);
				}
				catch (...)
				{
					start = nullptr;
				}
				// If goal is a GoalState, extract the underlying state for stateCost
				const ompl::base::State *goalState = nullptr;
				if (goal->hasType(ompl::base::GoalType::GOAL_STATE))
				{
					goalState = goal->as<ompl::base::GoalState>()->getState();
				}
				logging::debug("objective present.");
				if (start)
					logging::debug("start state cost = ", opt->stateCost(start).value());
				if (goalState)
					logging::debug("goal state cost = ", opt->stateCost(goalState).value());
				try
				{
					double ctg = opt->costToGo(start, goal).value();
					logging::debug("costToGo(start,goal) = ", ctg);
				}
				catch (const std::exception &e)
				{
					logging::debug("costToGo threw: ", e.what());
				}
			}
			catch (const std::exception &e)
			{
				logging::debug("exception while computing costs: ", e.what());
			}
		}
	}

	// Ensure any previously recorded solution paths (e.g., trivial goal-only
	// paths) are cleared so the optimizer does not treat an initial zero-cost
	// "solution" as the current best and thereby make the informed subset
	// empty. This avoids the planner seeking an impossible improvement (cost<0).
	if (this->m_pdef)
		this->m_pdef->clearSolutionPaths();

	this->m_solved = this->m_optimizingPlanner->solve(runTime);

	if (this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
		std::cout << "Exact solution found." << std::endl;
	else if (this->m_solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
		std::cout << "Approximate solution found." << std::endl;
	else
		std::cout << "No solution found." << std::endl;

	return this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
}

template <size_t controlInputs>
bool Planner<controlInputs>::replan(const double runTime)
{
	this->m_solved = this->m_optimizingPlanner->solve(runTime);

	if (this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
		std::cout << "Exact solution found." << std::endl;
	else if (this->m_solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
		std::cout << "Approximate solution found." << std::endl;
	else
		std::cout << "No solution found." << std::endl;

	return this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
}

template <size_t controlInputs>
bool Planner<controlInputs>::planTwoPhase(const double runTime, optimalPlanner plannerType, const double phase2RangeFactor, const double phase2LinearStep)
{
	// -----------------------------------------------------------------------
	// Two-phase planning: enforces revolute-before-prismatic ordering
	// structurally, bypassing the cost-landscape local-minimum problem.
	//
	// Why cost-function penalties fail here:
	//   The simultaneous path and the revolute-first path have nearly identical
	//   Euclidean length in 6D (prismatic displacement << revolute displacement),
	//   so RRT*'s nearest-neighbor queries always return simultaneous-path nodes.
	//   The optimizer never visits the revolute-first region and therefore never
	//   finds the lower-cost path, regardless of penalty magnitude.
	//
	// This method solves the problem structurally:
	//   Phase 1: start  -> waypoint = (beta_start, alpha_goal)
	//            Only revolute joints need to move; prismatic stays at start.
	//            Objective: CTR_RevoluteJointObjective (pure revolute error).
	//   Phase 2: waypoint -> goal = (beta_goal, alpha_goal)
	//            Only prismatic joints need to move; revolute already at target.
	//            Objective: CTR_BackboneLengthObjective.
	// -----------------------------------------------------------------------

	if (!this->m_startState || !this->m_goalState ||
	    !this->m_startState->get() || !this->m_goalState->get())
		throw std::runtime_error("planTwoPhase(): start/goal not set.");

	using StateType = ompl::base::RealVectorStateSpace::StateType;

	const auto *sst = this->m_startState->get()->as<StateType>();
	const auto *gst = this->m_goalState->get()->as<StateType>();

	// Build the intermediate waypoint: prismatic = start, revolute = goal.
	blaze::StaticVector<double, controlInputs> q_wp;
	for (size_t i = 0; i < 2; ++i) q_wp[i] = sst->values[i];   // beta_start
	for (size_t i = 2; i < controlInputs; ++i) q_wp[i] = gst->values[i];  // alpha_goal

	std::cout << "[planTwoPhase] Phase 1: rotating revolute joints to goal angles (prismatic fixed)." << std::endl;

	// ----- Phase 1 -----
	// Temporarily redirect goal to the waypoint.
	blaze::StaticVector<double, controlInputs> q_goal_orig;
	for (size_t i = 0; i < controlInputs; ++i)
		q_goal_orig[i] = gst->values[i];

	// Restrict prismatic (β) bounds to a tight window around β_start.
	// The window MUST be intersected with the dataset joint bounds: the start
	// configuration typically sits exactly on the dataset lower bounds (fully
	// retracted), and an unclamped window [β_start − w, β_start + w] would make
	// almost every prismatic sample invalid, starving the Phase 1 tree and
	// making RRTConnect time out.
	const auto [lb_ds, ub_ds] = m_CTR_model.getInputPosBounds();
	const double betaWindow = 3.0 * (3.0e-4 * m_space->getMaximumExtent()); // 3× defaultRange
	for (size_t i = 0; i < 2; ++i)
	{
		m_coordBound->setLow(i,  std::max(sst->values[i] - betaWindow, lb_ds[i]));
		m_coordBound->setHigh(i, std::min(sst->values[i] + betaWindow, ub_ds[i]));
	}
	m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(*m_coordBound);

	this->resetGoalState(q_wp);

	// Feasibility planners (RRTConnect & co.) ignore objectives, so keep the
	// cheap pure-revolute objective there. Optimizing planners get the
	// force-aware composite: revolute error + FTL swept volume, so the
	// rotation path itself minimizes backbone sweep under f_ext.
	const bool phase1Feasibility =
		plannerType == optimalPlanner::PLANNER_RRT         ||
		plannerType == optimalPlanner::PLANNER_RRT_CONNECT ||
		plannerType == optimalPlanner::PLANNER_RLRT        ||
		plannerType == optimalPlanner::PLANNER_PRM         ||
		plannerType == optimalPlanner::PLANNER_AORRTC;
	const planningObjective phase1Objective = phase1Feasibility
		? planningObjective::OBJECTIVE_REVJOINTS_ONLY
		: planningObjective::OBJECTIVE_REVJOINTS_AND_FTL;

	this->plan(runTime * 0.5, plannerType, phase1Objective);

	ompl::geometric::PathGeometric phase1path(this->m_si);
	if (this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION ||
	    this->m_solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
	{
		auto rawPath = std::static_pointer_cast<ompl::geometric::PathGeometric>(
		    this->m_pdef->getSolutionPath());
		if (rawPath)
			phase1path = *rawPath;
		else
			std::cerr << "[planTwoPhase] Phase 1: no path in pdef after solve.\n";
	}
	else
	{
		std::cerr << "[planTwoPhase] Phase 1 failed to find a solution. "
		          << "Falling back to single-phase planning (warm-starting from Phase 1 tree).\n";
		// Restore full prismatic bounds so the combined objective can explore the whole space.
		const auto [lb_pr, ub_pr] = m_CTR_model.getInputPosBounds();
		for (size_t i = 0; i < 2; ++i)
		{
			m_coordBound->setLow(i,  lb_pr[i]);
			m_coordBound->setHigh(i, ub_pr[i]);
		}
		m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(*m_coordBound);
		m_si->setup();
		// Restore original goal (also re-links the planner to the updated pdef).
		this->resetGoalState(q_goal_orig);
		// Swap to the combined objective; replan() will inherit it via m_pdef.
		this->m_pdef->setOptimizationObjective(this->allocateObjective(planningObjective::OBJECTIVE_REVJOINTS_AND_BACKBONE));
		// Warm-start: reuse the Phase 1 tree instead of discarding it.
		return this->replan(runTime);
	}

	std::cout << "[planTwoPhase] Phase 2: deploying prismatic joints (revolute at goal)." << std::endl;

	// ----- Phase 2: Synchronized deployment (equal linear velocity per active tube) -----
	// All active tubes advance at the same linear velocity. When the tube with the
	// smallest remaining displacement reaches its goal it stops; the others continue.
	// This produces exactly zero relative velocity between co-deploying tubes.
	//
	// Stopping order is determined by sorting tubes in ascending |displacement|:
	//   step 0 → tube with least travel stops first  (typically tube 3, outermost)
	//   step 1 → tube with next-least travel stops   (typically tube 2, intermediate)
	//   step 2 → tube with most travel stops last    (typically tube 1, innermost)
	//
	// The path is constructed analytically — no planner is needed for Phase 2.

	// Restore full prismatic bounds (required for path interpolation and for any
	// subsequent plan() / replan() calls after planTwoPhase() returns).
	const auto [lb_full, ub_full] = m_CTR_model.getInputPosBounds();
	for (size_t i = 0; i < 2; ++i)
	{
		m_coordBound->setLow(i,  lb_full[i]);
		m_coordBound->setHigh(i, ub_full[i]);
	}
	m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(*m_coordBound);

	// Restore original goal in pdef (also propagates to motion validator / validity checker).
	this->resetGoalState(q_goal_orig);

	const double safeStep = std::max(phase2LinearStep, 1.0e-6); // guard against zero/negative

	// ----- Phase 2 as an optimization over deployment schedules -----
	// Each candidate is a monotone prismatic profile from the waypoint to the
	// goal. The winner minimizes the follow-the-leader swept-volume cost
	// evaluated by the PINN on the LOADED robot (external tip force f_ext
	// applied), so tip loads / tissue contact shape the chosen deployment
	// rather than only the forward simulation.
	//
	// The synchronized fallback preserves the historical behavior: an already
	// committed two-phase plan degrades to the clearance-preserving schedule
	// rather than failing outright.
	const std::vector<DeploymentCandidate> candidates = this->buildDeploymentCandidates(q_wp, q_goal_orig, safeStep);
	const char *bestName = "";
	double bestCost = std::numeric_limits<double>::infinity();
	const Waypoints *bestSchedule = this->selectBestSchedule(candidates, true, bestName, bestCost);

	this->m_lastScheduleName = bestName;
	this->m_lastScheduleCost = bestCost;

	// Helper: allocate a state from q, append a copy to the path, then free it.
	ompl::geometric::PathGeometric phase2path(this->m_si);
	auto appendState = [&](const blaze::StaticVector<double, controlInputs> &q)
	{
		auto *s = this->m_space->allocState();
		auto *rv = s->as<StateType>();
		for (size_t i = 0; i < controlInputs; ++i)
			rv->values[i] = q[i];
		phase2path.append(s);
		this->m_space->freeState(s);
	};

	for (const auto &q : *bestSchedule)
		appendState(q);

	std::cout << "[planTwoPhase] Phase 2 (FTL-optimal deployment: " << bestName << "): "
	          << phase2path.getStateCount() << " states (step=" << safeStep * 1.0e3
	          << " mm, FTL swept cost=" << bestCost << ")." << std::endl;

	// ----- Concatenate -----
	if (phase1path.getStateCount() > 0)
	{
		phase1path.append(phase2path);
		this->m_pdef->clearSolutionPaths();
		this->m_pdef->addSolutionPath(
		    std::make_shared<ompl::geometric::PathGeometric>(phase1path));
		this->m_solved = ompl::base::PlannerStatus::EXACT_SOLUTION;
		std::cout << "[planTwoPhase] Combined path: "
		          << phase1path.getStateCount() << " states." << std::endl;
	}
	else
	{
		// Phase 1 produced no states — use the Phase 2 path alone.
		this->m_pdef->clearSolutionPaths();
		this->m_pdef->addSolutionPath(
		    std::make_shared<ompl::geometric::PathGeometric>(phase2path));
		this->m_solved = ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
	}

	// Restore m_startState to original start so analyzeSolution() works correctly.
	{
		auto *stState = this->m_startState->get()->as<StateType>();
		for (size_t i = 0; i < controlInputs; ++i)
			stState->values[i] = sst->values[i];
	}
	// Ensure the prismatic bounds are in their original state for any subsequent calls.
	{
		const auto [lb_r, ub_r] = m_CTR_model.getInputPosBounds();
		for (size_t i = 0; i < 2; ++i)
		{
			m_coordBound->setLow(i,  lb_r[i]);
			m_coordBound->setHigh(i, ub_r[i]);
		}
		m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(*m_coordBound);
	}

	return this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
}

// Builds the four candidate prismatic deployment schedules from q_from to q_to.
// q_from supplies the starting β values and the α values held for every state;
// q_to supplies the goal β values. All schedules are monotone in β toward the goal.
template <size_t controlInputs>
std::vector<typename Planner<controlInputs>::DeploymentCandidate>
Planner<controlInputs>::buildDeploymentCandidates(const JointVector &q_from, const JointVector &q_to, const double safeStep) const
{
	// Geometry lives in DeploymentSchedule.hpp (pure, unit-tested); this
	// wrapper only maps the extracted candidate type onto the Planner's.
	auto raw = deployment_schedule::buildDeploymentCandidates<controlInputs>(q_from, q_to, safeStep);
	std::vector<DeploymentCandidate> candidates;
	candidates.reserve(raw.size());
	for (auto &cand : raw)
		candidates.push_back({cand.name, std::move(cand.wps)});
	return candidates;
}

// A schedule is admissible only if every waypoint satisfies the tube
// ordering / clearance constraints.
template <size_t controlInputs>
bool Planner<controlInputs>::scheduleIsValid(const Waypoints &wps) const
{
	using StateType = ompl::base::RealVectorStateSpace::StateType;
	auto *s = this->m_space->allocState();
	auto *rv = s->as<StateType>();
	bool ok = true;
	for (const auto &q : wps)
	{
		for (size_t i = 0; i < controlInputs; ++i)
			rv->values[i] = q[i];
		if (!this->m_stateValidityChecker->isValid(s))
		{
			ok = false;
			break;
		}
	}
	this->m_space->freeState(s);
	return ok;
}

// FTL swept-volume cost of a schedule, evaluated with f_ext. All
// candidates are compared at the same coarse discretization (~40 segments)
// so PINN inference stays cheap and the comparison is fair.
template <size_t controlInputs>
double Planner<controlInputs>::scheduleSweptCost(const Waypoints &wps) const
{
	if (!this->m_ftlObjective)
		return 0.0;
	return deployment_schedule::scheduleSweptCost<controlInputs>(
	    wps, [this](const JointVector &a, const JointVector &b)
	    { return this->m_ftlObjective->sweptCost(a, b); });
}

template <size_t controlInputs>
const typename Planner<controlInputs>::Waypoints *
Planner<controlInputs>::selectBestSchedule(const std::vector<DeploymentCandidate> &candidates, const bool fallbackToSynchronized,
                                           const char *&bestNameOut, double &bestCostOut) const
{
	const Waypoints *bestSchedule = nullptr;
	bestNameOut = "";
	bestCostOut = std::numeric_limits<double>::infinity();
	for (const auto &cand : candidates)
	{
		if (!this->scheduleIsValid(cand.wps))
		{
			std::cout << "[planTwoPhase] Phase 2 candidate '" << cand.name
			          << "' violates tube-ordering constraints; skipped." << std::endl;
			continue;
		}
		const double cost = this->scheduleSweptCost(cand.wps);
		std::cout << "[planTwoPhase] Phase 2 candidate '" << cand.name
		          << "': FTL swept cost = " << cost << " (with f_ext)." << std::endl;
		if (cost < bestCostOut)
		{
			bestCostOut = cost;
			bestSchedule = &cand.wps;
			bestNameOut = cand.name;
		}
	}

	if (!bestSchedule && fallbackToSynchronized)
	{
		// All alternatives infeasible — fall back to the synchronized schedule,
		// which preserves the clearance invariant by construction.
		bestSchedule = &candidates.front().wps;
		bestNameOut = candidates.front().name;
		std::cerr << "[planTwoPhase] WARNING: no valid Phase 2 candidate; using synchronized fallback." << std::endl;
	}
	return bestSchedule;
}

template <size_t controlInputs>
bool Planner<controlInputs>::planDeployment(const JointVector &q_start, const JointVector &q_goal, const double linearStep, const double alphaTol)
{
	using StateType = ompl::base::RealVectorStateSpace::StateType;

	// The deployment schedules hold alpha constant at the goal values, so the
	// current revolute angles must already agree with the goal within alphaTol;
	// correcting a larger mismatch would rotate deployed tubes and requires a
	// full planTwoPhase().
	for (size_t i = 2; i < controlInputs; ++i)
	{
		const double alphaErr = std::abs(shortestAngleDiff(q_start[i], q_goal[i]));
		if (alphaErr > alphaTol)
		{
			std::cerr << "[planDeployment] Revolute joint " << i - 1 << " is " << alphaErr
			          << " rad away from its goal (tolerance " << alphaTol
			          << " rad); a full planTwoPhase() is required." << std::endl;
			this->m_pdef->clearSolutionPaths();
			this->m_solved = ompl::base::PlannerStatus::ABORT;
			return false;
		}
	}

	// Hold alpha at the goal values for the whole schedule; the ≤ alphaTol
	// correction rides on the first commanded waypoint.
	JointVector q_eff(q_start);
	for (size_t i = 2; i < controlInputs; ++i)
		q_eff[i] = q_goal[i];

	// Start validity (β ordering/clearance + |α₂ − α₁| ≤ π). State-space bounds
	// are deliberately not checked, matching resetStartState(): live joint
	// feedback may sit marginally outside the dataset bounds.
	{
		auto *s = this->m_space->allocState();
		auto *rv = s->as<StateType>();
		for (size_t i = 0; i < controlInputs; ++i)
			rv->values[i] = q_eff[i];
		const bool startValid = this->m_stateValidityChecker->isValid(s);
		this->m_space->freeState(s);
		if (!startValid)
		{
			std::cerr << "[planDeployment] Current configuration violates tube-ordering constraints; cannot re-schedule." << std::endl;
			this->m_pdef->clearSolutionPaths();
			this->m_solved = ompl::base::PlannerStatus::INVALID_START;
			return false;
		}
	}

	const double safeStep = std::max(linearStep, 1.0e-6);

	Waypoints degenerateSchedule;
	std::vector<DeploymentCandidate> candidates;
	const Waypoints *bestSchedule = nullptr;
	const char *bestName = "already at goal";
	double bestCost = 0.0;

	if (std::abs(q_goal[0] - q_eff[0]) < 1.0e-9 && std::abs(q_goal[1] - q_eff[1]) < 1.0e-9)
	{
		// Prismatic joints are already at the goal — single-state path.
		degenerateSchedule.push_back(q_eff);
		bestSchedule = &degenerateSchedule;
	}
	else
	{
		candidates = this->buildDeploymentCandidates(q_eff, q_goal, safeStep);
		// No synchronized fallback here (unlike planTwoPhase): commanding a
		// constraint-violating schedule mid-deployment is worse than keeping the
		// current plan, so the caller falls back to "continue the old plan".
		bestSchedule = this->selectBestSchedule(candidates, false, bestName, bestCost);
		if (!bestSchedule)
		{
			std::cerr << "[planDeployment] No valid deployment schedule from the current configuration." << std::endl;
			this->m_pdef->clearSolutionPaths();
			this->m_solved = ompl::base::PlannerStatus::ABORT;
			return false;
		}
	}

	// Keep the problem definition consistent with the re-scheduled segment so
	// writeSolutionToFile()/analyzeSolution() and later plan calls see coherent
	// endpoints. q_eff passed the same validity check setStartState() applies;
	// q_goal was already accepted by setGoalState() when the plan was created.
	this->setStartState(q_eff);
	this->setGoalState(q_goal);

	ompl::geometric::PathGeometric schedulePath(this->m_si);
	{
		auto *s = this->m_space->allocState();
		auto *rv = s->as<StateType>();
		for (const auto &q : *bestSchedule)
		{
			for (size_t i = 0; i < controlInputs; ++i)
				rv->values[i] = q[i];
			schedulePath.append(s); // append() copies the state
		}
		this->m_space->freeState(s);
	}

	this->m_pdef->clearSolutionPaths();
	this->m_pdef->addSolutionPath(std::make_shared<ompl::geometric::PathGeometric>(schedulePath));
	this->m_solved = ompl::base::PlannerStatus::EXACT_SOLUTION;
	this->m_lastScheduleName = bestName;
	this->m_lastScheduleCost = bestCost;

	std::cout << "[planDeployment] Re-scheduled deployment (" << bestName << "): "
	          << schedulePath.getStateCount() << " states (step=" << safeStep * 1.0e3
	          << " mm, FTL swept cost=" << bestCost << ")." << std::endl;

	return true;
}

template <size_t controlInputs>
void Planner<controlInputs>::writeSolutionToFile(const std::string &outputFile)
{
	if ((this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION) || (this->m_solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION))
	{
		if (!outputFile.empty())
		{
			const std::filesystem::path fileName = outputFile;

			// Ensure the target directory exists
			if (fileName.has_parent_path() && !std::filesystem::exists(fileName.parent_path()))
			{
				std::filesystem::create_directories(fileName.parent_path());
			}

			// Atomic publish: write to a temp file, then rename over the target.
			// The master node polls this CSV; rename() is atomic on POSIX, so it
			// can never observe a half-written plan.
			const std::filesystem::path tmpName = fileName.string() + ".tmp";
			std::ofstream outFile(tmpName, std::ios::out);

			if (outFile.is_open())
			{
				// Set precision for the entire file
				outFile << std::fixed << std::setprecision(10);

				auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(this->m_pdef->getSolutionPath());
				if (!path)
					throw std::runtime_error("No solution path available to write.");

				// Densify the path to the motion validator's resolution so the
				// written CSV has finely-spaced waypoints suitable for execution.
				path->interpolate();

				// Loop through the states and write each state as a line in CSV format
				for (const auto &state : path->getStates())
				{
					// Assuming your state is of a type that can be converted to a string with comma-separated values
					auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
					if (pos)
					{
						for (size_t i = 0UL; i < controlInputs; ++i)
						{
							if (i > 0)
								outFile << ",";
							outFile << pos->values[i];
						}
						outFile << std::endl;
					}
					else
					{
						throw std::runtime_error("Error: Unexpected state type.");
					}
				}

				outFile.close();
				std::filesystem::rename(tmpName, fileName);
				// Inform the user that the file was successfully written
				std::cout << "Motion plan successfully written to: "
						  << std::filesystem::absolute(fileName) << std::endl;
			}
			else
			{
				throw std::runtime_error("Error: Could not save surgical planning to file: " + fileName.string());
			}
		}
	}
	else
		std::cout << "No solution found." << std::endl;
}

template <size_t controlInputs>
void Planner<controlInputs>::analyzeSolution(bool dumpCSV, const std::string &csvPath)
{
	using StateType = ompl::base::RealVectorStateSpace::StateType;

	if (!(this->m_solved == ompl::base::PlannerStatus::EXACT_SOLUTION || this->m_solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION))
	{
		if (logging::verbose)
			std::cout << "Skipping diagnostics: planner has no solution." << std::endl;
		return;
	}

	auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(this->m_pdef->getSolutionPath());
	if (!path || path->getStateCount() == 0)
	{
		if (logging::verbose)
			std::cout << "No states in solution path to analyze." << std::endl;
		return;
	}

	// Pull start/goal prismatic for progress; goal revolute for angle diffs
	if (!this->m_startState || !this->m_goalState || !this->m_startState->get() || !this->m_goalState->get())
	{
		if (logging::verbose)
			std::cout << "Start/goal states unavailable for analysis." << std::endl;
		return;
	}
	const auto *sstate = this->m_startState->get()->as<StateType>();
	const auto *gstate = this->m_goalState->get()->as<StateType>();
	if (!sstate || !gstate)
	{
		if (logging::verbose)
			std::cout << "Start/goal states unavailable for analysis." << std::endl;
		return;
	}

	const bool printDiagnostics = logging::verbose;
	if (printDiagnostics)
	{
		std::cout << std::fixed << std::setprecision(6);
		std::cout << "Diagnostics: idx | path_frac | p | rev_base | scale | rev_scaled | rev_weighted | backbone_raw | prismatic_weighted | backbone_coupled | mismatch | gating | alignment_progress | req_alignment | deficit | ordering_penalty | composite | max_angle_err_deg | ftl_swept" << std::endl;
	}

	auto clamp01 = [](double x) { return std::max(0.0, std::min(1.0, x)); };

	// Prismatic start->goal vector and denom for projection progress p (2D: beta1, beta2)
	const double sx = sstate->values[0UL], sy = sstate->values[1UL];
	const double gx = gstate->values[0UL], gy = gstate->values[1UL];
	const double vx = gx - sx, vy = gy - sy;
	const double prismaticDenom = vx * vx + vy * vy;

	// Revolute goal angles (4-DoF: indices 2 and 3)
	const double a1g = gstate->values[2UL], a2g = gstate->values[3UL];

	const std::size_t N = path->getStateCount();
	std::vector<double> cumulativeLengths(N, 0.0);
	double totalLength = 0.0;
	if (this->m_si)
	{
		for (std::size_t i = 1; i < N; ++i)
		{
			double seg = this->m_si->distance(path->getState(i - 1), path->getState(i));
			totalLength += seg;
			cumulativeLengths[i] = totalLength;
		}
	}
	const double invTotalLength = (totalLength > 1.0e-9) ? 1.0 / totalLength : 0.0;

	// Constants from CTR_RevoluteJointObjective — use static constexpr class members
	// to avoid duplicating values that would silently drift out of sync.
	constexpr double c1    = CTR_RevoluteJointObjective<controlInputs>::c1;
	constexpr double c2    = CTR_RevoluteJointObjective<controlInputs>::c2;

	auto shortest = [](double a, double b) {
		double d = a - b;
		while (d > M_PI)
			d -= 2.0 * M_PI;
		while (d < -M_PI)
			d += 2.0 * M_PI;
		return std::fabs(d);
	};

	// Backbone weights from CTR_BackboneLengthObjective — use static constexpr class members.
	constexpr double k1 = CTR_BackboneLengthObjective<controlInputs>::k1;
	constexpr double k2 = CTR_BackboneLengthObjective<controlInputs>::k2;
	constexpr double k3 = CTR_BackboneLengthObjective<controlInputs>::k3;
	constexpr double k4 = CTR_BackboneLengthObjective<controlInputs>::k4;

	// Composite parameters (synchronized with CompositeStateCostIntegral)
	const double w_backbone   = CompositeParams::w_backbone;
	const double w_revolute   = CompositeParams::w_revolute;
	const double k_coupling   = CompositeParams::k_coupling;
	const double w_mismatch   = CompositeParams::w_mismatch;
	const double p_gate       = CompositeParams::p_gate;
	const double angle_gate_thresh = CompositeParams::angle_gate_thresh;
	const double w_gate       = CompositeParams::w_gate;
	const double revolute_target_fraction = CompositeParams::revolute_target_fraction;
	const double w_ordering   = CompositeParams::w_ordering;
	const double ordering_power = CompositeParams::ordering_power;

	// CSV output
	std::ofstream csv;
	if (dumpCSV)
	{
		const std::filesystem::path out = csvPath;
		if (out.has_parent_path() && !std::filesystem::exists(out.parent_path()))
			std::filesystem::create_directories(out.parent_path());
		csv.open(out, std::ios::out);
		if (csv.is_open())
		{
			csv << std::fixed << std::setprecision(10);
			csv << "index,path_fraction,p,rev_base,scale,rev_scaled,rev_weighted,backbone_raw,prismatic_weighted,backbone_coupled,mismatch,gating,alignment_progress,lag_raw,lag,composite,max_angle_err_deg,ftl_swept\n";
		}
	}

	std::size_t idx_first_small_err = N; // where max angle error < 1 deg
	double firstSmallErrFrac = -1.0;
	double firstSmallErrLength = 0.0;
	const double err_thresh = M_PI / 180.0; // 1 deg in rad
	double initialMaxErrDeg = -1.0;

	for (std::size_t i = 0; i < N; ++i)
	{
		const auto *st = path->getState(i)->as<StateType>();
		const double pathFrac = (totalLength > 1.0e-9) ? cumulativeLengths[i] * invTotalLength : ((N > 1) ? static_cast<double>(i) / static_cast<double>(N - 1) : 1.0);
		// Progress p (2D prismatic)
		double p = 0.0;
		if (prismaticDenom > 1.0e-12)
		{
			const double wx = st->values[0UL] - sx;
			const double wy = st->values[1UL] - sy;
			p = clamp01((wx * vx + wy * vy) / prismaticDenom);
		}
		else
		{
			// Fallback: normalize distance from origin to goal in 2D prismatic
			const double dgoal2 = gx * gx + gy * gy;
			const double dcur2 = st->values[0UL] * st->values[0UL] + st->values[1UL] * st->values[1UL];
			p = (dgoal2 > 1.00E-12) ? clamp01(std::sqrt(dcur2 / dgoal2)) : 0.0;
		}

		// Revolute contributions (4-DoF: angles at indices 2 and 3)
		const double d1 = shortest(st->values[2UL], a1g);
		const double d2 = shortest(st->values[3UL], a2g);
		const double rev_base = c1 * d1 + c2 * d2;
		// Progress scaling removed from revolute objective; ordering is enforced by
		// the composite's motionCost() override.  Scale is identically 1.
		constexpr double scale = 1.0;
		const double rev_scaled = rev_base;
		const double revoluteWeighted = w_revolute * rev_scaled;

		// Backbone cost — compute directly from getOverallLen() (not get_s_ends which is broken for 4-DoF)
		const blaze::StaticVector<double, 3UL> L = this->m_CTR_model.getOverallLen();
		const double end1 = L[0] + st->values[0]; // tube1 distal
		const double end2 = L[1] + st->values[1]; // tube2 distal
		const double end3 = L[2];                   // tube3 static
		const double backbone_raw = k1 * end1 + k2 * (end1 - end2) + k3 * (end2 - end3) + k4 * (end1 - end3);
		const double prismaticWeighted = w_backbone * backbone_raw;

		const double max_err_deg = 180.0 / M_PI * std::max(d1, d2);
		if (initialMaxErrDeg < 0.0)
			initialMaxErrDeg = max_err_deg > 1.00E-9 ? max_err_deg : 1.0;
		double normAngleErr = max_err_deg / initialMaxErrDeg;
		normAngleErr = std::clamp(normAngleErr, 0.0, 1.0);

		const double backbone_coupled = w_backbone * backbone_raw * (1.0 + k_coupling * normAngleErr);
		const double mismatch = w_mismatch * normAngleErr;
		double gating = 0.0;
		if (p > p_gate && normAngleErr > angle_gate_thresh)
			gating += w_gate * (p - p_gate) * (normAngleErr - angle_gate_thresh);
		const double alignmentProgress = std::clamp(1.0 - normAngleErr, 0.0, 1.0);
		const double reqAlignment = std::min(1.0, p / std::max(1.0e-9, revolute_target_fraction));
		const double deficit = std::max(0.0, reqAlignment - alignmentProgress);
		const double lagPenalty = w_ordering * std::pow(deficit, ordering_power);

		// Gating and lag penalty are diagnostic-only.  The active per-state ordering
		// penalty is w_ordering_state * p * ε².
		const double w_os = CompositeParams::w_ordering_state;
		const double orderingStatePenalty = w_os * p * normAngleErr * normAngleErr;
		const double composite = backbone_coupled + revoluteWeighted + mismatch + orderingStatePenalty;

		// FTL swept-volume of the incoming edge, evaluated on the loaded robot
		// (f_ext applied). Zero for the first state.
		double ftl_swept = 0.0;
		if (i > 0 && this->m_ftlObjective)
		{
			const auto *stPrev = path->getState(i - 1)->as<StateType>();
			blaze::StaticVector<double, controlInputs> q_prev, q_curr;
			for (size_t k = 0; k < controlInputs; ++k)
			{
				q_prev[k] = stPrev->values[k];
				q_curr[k] = st->values[k];
			}
			ftl_swept = this->m_ftlObjective->sweptCost(q_prev, q_curr);
		}

		if (idx_first_small_err == N && std::max(d1, d2) < err_thresh)
		{
			idx_first_small_err = i;
			firstSmallErrFrac = pathFrac;
			firstSmallErrLength = cumulativeLengths[i];
		}

		// Print every state; for long paths this is verbose but intentional for diagnostics
		if (printDiagnostics)
		{
			std::cout << i << " | " << pathFrac << " | " << p << " | " << rev_base << " | " << scale << " | " << rev_scaled << " | " << revoluteWeighted << " | " << backbone_raw << " | " << prismaticWeighted << " | " << backbone_coupled << " | " << mismatch << " | " << gating << " | " << alignmentProgress << " | " << reqAlignment << " | " << deficit << " | " << lagPenalty << " | " << composite << " | " << max_err_deg << " | " << ftl_swept << std::endl;
		}

		if (csv.is_open())
			csv << i << "," << pathFrac << "," << p << "," << rev_base << "," << scale << "," << rev_scaled << "," << revoluteWeighted << "," << backbone_raw << "," << prismaticWeighted << "," << backbone_coupled << "," << mismatch << "," << gating << "," << alignmentProgress << "," << reqAlignment << "," << deficit << "," << lagPenalty << "," << composite << "," << max_err_deg << "," << ftl_swept << "\n";
	}

	if (csv.is_open())
	{
		csv.close();
		logging::debug("Diagnostics CSV written.");
	}

	if (printDiagnostics)
	{
		if (idx_first_small_err < N)
		{
			double frac = (firstSmallErrFrac >= 0.0) ? firstSmallErrFrac : ((N > 1) ? static_cast<double>(idx_first_small_err) / static_cast<double>(N - 1) : 1.0);
			double arcLen = firstSmallErrLength;
			std::cout << "Revolute angle error < 1 deg achieved at state index " << idx_first_small_err
				  << " (" << std::setprecision(3) << 100.0 * frac << "% of cumulative path length";
			if (totalLength > 1.0e-9)
				std::cout << ", arc length = " << std::setprecision(4) << arcLen << " (state-space units))" << std::endl;
			else
				std::cout << ")" << std::endl;
		}
		else
		{
			std::cout << "Revolute angle error did not drop below 1 deg along the path." << std::endl;
		}
	}
}

template <size_t controlInputs>
void Planner<controlInputs>::cleanup()
{
	// Clear planner-specific state first to release any references to ProblemDefinition/SpaceInformation
	if (this->m_optimizingPlanner)
	{
		try
		{
			this->m_optimizingPlanner->clear();
		}
		catch (...)
		{
		}
		this->m_optimizingPlanner.reset();
	}

	// Clear problem definition states and paths (frees cloned start/goal states)
	if (this->m_pdef)
	{
		try
		{
			this->m_pdef->clearSolutionPaths();
			this->m_pdef->clearStartStates();
			this->m_pdef->clearGoal();
		}
		catch (...)
		{
		}
		this->m_pdef.reset();
	}

	// Reset samplers, objectives, and validity/motion validators
	this->m_informedSampler.reset();
	this->m_ftlObjective.reset();
	this->m_stateValidityChecker.reset();

	if (this->m_si)
	{
		try
		{
			this->m_si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr());
			this->m_si->setMotionValidator(ompl::base::MotionValidatorPtr());
		}
		catch (...)
		{
		}
		this->m_si.reset();
	}

	// Reset space and bounds
	this->m_space.reset();
	this->m_coordBound.reset();

	// Reset scoped states to release any underlying allocations
	this->m_startState.reset();
	this->m_goalState.reset();
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getRevoluteJointObjective(const std::array<double, 2UL> &goal)
{
	auto obj = std::make_shared<CTR_RevoluteJointObjective<controlInputs>>(this->m_si, goal);

	try
	{
		if (this->m_startState && this->m_goalState && this->m_startState->get() && this->m_goalState->get())
		{
			const auto *sstate = this->m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
			const auto *gstate = this->m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
			std::array<double, 2UL> startPrism{sstate->values[0UL], sstate->values[1UL]};
			std::array<double, 2UL> goalPrism{gstate->values[0UL], gstate->values[1UL]};
			obj->setPrismaticStartGoal(startPrism, goalPrism);
		}
	}
	catch (...)
	{ /* non-fatal: leave objective without prismatic bounds */
	}

	return obj;
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getPathLengthObjective()
{
	return std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si);
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getThresholdPathLengthObjctive()
{
	// Return a plain path-length optimization objective without a cost threshold.
	// Setting an infinite cost threshold causes planners to consider any finite
	// solution as meeting the threshold immediately which leads to early
	// termination. Leave thresholds out to allow continued optimization until
	// the termination condition (time, etc.) is met.
	return std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si);
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getClearanceObjective()
{
	return std::make_shared<CTR_ClearanceObjective<controlInputs>>(this->m_si);
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::RevoluteJointsAndBackboneLengthObjective(const std::array<double, 2UL> &goal)
{
	auto backboneLengthObj(std::make_shared<CTR_BackboneLengthObjective<controlInputs>>(this->m_si, this->m_CTR_model));
	auto revoluteJointObj(std::make_shared<CTR_RevoluteJointObjective<controlInputs>>(this->m_si, goal));

	// Create a MultiOptimizationObjective subclass that forwards
	// allocInformedStateSampler() to its component objectives. OMPL's
	// default MultiOptimizationObjective does not forward this call,
	// which causes informed planners to fall back to rejection
	// sampling even when a component objective provides a direct
	// InformedSampler (e.g., the backbone-length objective).
	class MultiOptWithInformed : public ompl::base::MultiOptimizationObjective
	{
	public:
		using ompl::base::MultiOptimizationObjective::MultiOptimizationObjective;

		ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDef,
																 unsigned int maxNumberCalls) const override
		{
				logging::debug("MultiOptWithInformed::allocInformedStateSampler called");
			// Iterate component objectives and return the first non-null
			// informed sampler provided by a component. This keeps behavior
			// simple and leverages any specialized informed sampling
			// implemented by component objectives.
			for (unsigned int i = 0; i < this->getObjectiveCount(); ++i)
			{
				const auto &comp = this->getObjective(i);
				if (!comp)
					continue;
				try
				{
						std::ostringstream oss;
						oss << "querying component " << i << " for informed sampler";
						logging::debug(oss.str());
					auto inf = comp->allocInformedStateSampler(probDef, maxNumberCalls);
					if (inf)
					{
							logging::debug("component ", i, " returned non-null informed sampler");
						return inf;
					}
					else
					{
							logging::debug("component ", i, " returned null informed sampler");
					}
				}
				catch (...)
				{
						logging::debug("component ", i, " allocInformedStateSampler threw");
					// Ignore component failures and try next
				}
			}
			return ompl::base::InformedSamplerPtr();
		}
	};

	// Instead of using a MultiOptimizationObjective (which OMPL's informed
	// sampling path doesn't treat as a StateCostIntegralObjective), create
	// a composite StateCostIntegralObjective that represents the weighted
	// sum of the backbone-length and revolute-joint objectives. This class
	// will expose a proper allocInformedStateSampler() (forwarding to the
	// backbone objective) so informed planners can use direct informed
	// sampling.
	class CompositeStateCostIntegral : public ompl::base::StateCostIntegralObjective
	{
	public:
		CompositeStateCostIntegral(const ompl::base::SpaceInformationPtr &si,
							   const ompl::base::OptimizationObjectivePtr &backbone,
							   const ompl::base::OptimizationObjectivePtr &revolute,
							   const ompl::base::OptimizationObjectivePtr &ftl,
							   double wb,
							   double wr,
							   const std::array<double,2UL> &goalAngles,
							   const std::array<double,2UL> &prismStart,
							   const std::array<double,2UL> &prismGoal)
			: ompl::base::StateCostIntegralObjective(si, true), m_backbone(backbone), m_revolute(revolute), m_ftl(ftl), w_backbone(wb), w_revolute(wr), m_si(si), m_goalAngles(goalAngles), m_prismStart(prismStart), m_prismGoal(prismGoal)
		{
			// Pre-compute initial max angle error (degrees) for normalization
			m_initialMaxAngleErrDeg = computeMaxAngleErrDeg(prismStart[0], prismStart[1], goalAngles);
			if (m_initialMaxAngleErrDeg < 1.00E-9)
				m_initialMaxAngleErrDeg = 1.00; // avoid divide by zero
		}

		ompl::base::Cost stateCost(const ompl::base::State *s) const override
		{
			// Raw component costs
			double backboneRaw = (m_backbone) ? m_backbone->stateCost(s).value() : 0.0;
			double revoluteScaled = (m_revolute) ? m_revolute->stateCost(s).value() : 0.0;

			// Current max angle error (degrees) for normalization
			const auto *rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
			double maxErrDeg = computeMaxAngleErrDeg(rv->values[2], rv->values[3], m_goalAngles);
			double normAngleErr = maxErrDeg / m_initialMaxAngleErrDeg;
			if (normAngleErr < 0.0) normAngleErr = 0.0;
			if (normAngleErr > 1.0) normAngleErr = 1.0;

			// Prismatic progress p ∈ [0,1] for the ordering penalty below.
			const double p = computeProgress(rv);

			// Backbone amplified when revolute error is high.
			const double backboneCoupled = w_backbone * backboneRaw * (1.0 + k_coupling * normAngleErr);
			// Flat per-state angle-error penalty: smooth gradient toward the goal revolute angles.
			const double mismatch = w_mismatch * normAngleErr;
			// Per-state ordering penalty: w * p * ε²
			// Zero when p≈0 (revolute-first waypoints at start of deployment) or ε≈0 (revolute done).
			// Non-zero for simultaneous-path intermediate states where both p>0 and ε>0.
			// Appears in the cost-to-come of every tree node, so the informed sampler naturally
			// focuses on revolute-first regions (p≈0 nodes have lower cumulative cost).
			const double orderingStatePenalty = w_ordering_state * p * normAngleErr * normAngleErr;
			return ompl::base::Cost(backboneCoupled + w_revolute * revoluteScaled + mismatch + orderingStatePenalty);
		}

		ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
		{
			// Base: trapezoidal integral of stateCost along the segment.
			const ompl::base::Cost base = ompl::base::StateCostIntegralObjective::motionCost(s1, s2);

			// Transition ordering penalty: fires when prismatic deployment advances while
			// revolute error remains.  Direction-aware — only penalises "prismatic-first"
			// edges, not "revolute-first" ones.
			//
			// IMPORTANT: use normAngleErr at s1 (the SOURCE of the edge), not s2.
			// Using s2 fails because any edge whose destination is the goal has
			// normAngleErr(s2) = 0, making the penalty zero for the direct start→goal
			// edge regardless of how much prismatic progress it advances.
			// Using s1 asks "was revolute already converged before this prismatic step?"
			// which is exactly the ordering constraint.
			//
			// Revolute-first path: Δp ≈ 0 during rotation → total penalty ≈ 0.
			// Direct start→goal edge: Δp=1, normAngleErr(s1=start)=1 → penalty = w_transition.
			const auto *r1 = s1->as<ompl::base::RealVectorStateSpace::StateType>();
			const auto *r2 = s2->as<ompl::base::RealVectorStateSpace::StateType>();
			const double delta_p = std::max(0.0, computeProgress(r2) - computeProgress(r1));
			const double maxErrDeg1 = computeMaxAngleErrDeg(
				r1->values[2], r1->values[3], m_goalAngles);
			const double normAngleErr1 = std::clamp(maxErrDeg1 / m_initialMaxAngleErrDeg, 0.0, 1.0);

			// Follow-the-leader swept-volume term: penalizes edges whose backbone
			// (evaluated on the loaded robot, i.e. with f_ext applied) sweeps a
			// large volume. This is what makes the optimizer prefer near-FTL
			// deployments and makes the external force part of the optimization.
			const double ftlCost = (m_ftl) ? m_ftl->motionCost(s1, s2).value() : 0.0;

			return ompl::base::Cost(base.value() + w_transition * delta_p * normAngleErr1 * normAngleErr1 + ftlCost);
		}

		bool hasCostToGoHeuristic() const
		{
			return (m_backbone && m_backbone->hasCostToGoHeuristic()) || (m_revolute && m_revolute->hasCostToGoHeuristic());
		}

		ompl::base::Cost costToGo(const ompl::base::State *s, const ompl::base::Goal *g) const
		{
			double sum = 0.0;
			if (m_backbone && m_backbone->hasCostToGoHeuristic())
				sum += w_backbone * m_backbone->costToGo(s, g).value();
			if (m_revolute && m_revolute->hasCostToGoHeuristic())
				sum += w_revolute * m_revolute->costToGo(s, g).value();
			return ompl::base::Cost(sum);
		}

		ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDef,
																 unsigned int maxNumberCalls) const override
		{
			logging::debug("CompositeStateCostIntegral::allocInformedStateSampler called");

			auto tryAlloc = [probDef, maxNumberCalls](const char *label, const ompl::base::OptimizationObjectivePtr &obj) -> ompl::base::InformedSamplerPtr
			{
				logging::debug("tryAlloc(", label, ") obj=", (obj ? "set" : "null"));
				if (!obj)
					return ompl::base::InformedSamplerPtr();
				try
				{
					logging::debug("invoking component sampler for ", label);
					return obj->allocInformedStateSampler(probDef, maxNumberCalls);
				}
				catch (...)
				{
					return ompl::base::InformedSamplerPtr();
				}
			};

			if (auto backboneSampler = tryAlloc("backbone", m_backbone))
				return backboneSampler;
			if (auto revoluteSampler = tryAlloc("revolute", m_revolute))
				return revoluteSampler;

			return ompl::base::InformedSamplerPtr();
		}

	private:
		// Helpers
		static double shortestDiff(double a, double b)
		{
			double d = a - b;
			while (d > M_PI) d -= 2.0 * M_PI;
			while (d < -M_PI) d += 2.0 * M_PI;
			return std::fabs(d);
		}
		static double computeMaxAngleErrDeg(double a1, double a2, const std::array<double,2UL> &goalAng)
		{
			double e1 = shortestDiff(a1, goalAng[0]);
			double e2 = shortestDiff(a2, goalAng[1]);
			return 180.0 / M_PI * std::max(e1, e2);
		}
		double computeProgress(const ompl::base::RealVectorStateSpace::StateType *rv) const
		{
			// Projection of current prismatic onto start->goal (2D: beta1, beta2)
			double sx = m_prismStart[0], sy = m_prismStart[1];
			double gx = m_prismGoal[0], gy = m_prismGoal[1];
			double vx = gx - sx, vy = gy - sy;
			double denom = vx*vx + vy*vy;
			if (denom <= 1.00E-12) return 0.0;
			double wx = rv->values[0] - sx;
			double wy = rv->values[1] - sy;
			double p = (wx*vx + wy*vy) / denom;
			if (p < 0.0) p = 0.0; if (p > 1.0) p = 1.0;
			return p;
		}
		// Component objectives
		ompl::base::OptimizationObjectivePtr m_backbone;
		ompl::base::OptimizationObjectivePtr m_revolute;
		ompl::base::OptimizationObjectivePtr m_ftl;
		// Weights (retuned)
		double w_backbone{CompositeParams::w_backbone};
		double w_revolute{CompositeParams::w_revolute};
		// Coupling & mismatch parameters
		double k_coupling{CompositeParams::k_coupling};
		double w_mismatch{CompositeParams::w_mismatch};
		// Rear gate parameters
		double p_gate{CompositeParams::p_gate};
		double angle_gate_thresh{CompositeParams::angle_gate_thresh};
		double w_gate{CompositeParams::w_gate};
		// Ordering penalty parameters
		double revolute_target_fraction{CompositeParams::revolute_target_fraction};
		double w_ordering{CompositeParams::w_ordering};
		double ordering_power{CompositeParams::ordering_power};
		double w_transition{CompositeParams::w_transition};
		double w_ordering_state{CompositeParams::w_ordering_state};
		// Geometry / model context
		ompl::base::SpaceInformationPtr m_si;
		// Goal & progress baseline
		std::array<double,2UL> m_goalAngles{};
		std::array<double,2UL> m_prismStart{};
		std::array<double,2UL> m_prismGoal{};
		double m_initialMaxAngleErrDeg{1.0};
	};

	// Gather goal angles & prismatic start/goal for coupling and progress (4-DoF: 2 revolute, 2 prismatic)
	std::array<double,2UL> goalAngles = {goal[0], goal[1]};
	std::array<double,2UL> prismStart{0.0,0.0};
	std::array<double,2UL> prismGoal{0.0,0.0};
	if (this->m_startState && this->m_goalState && this->m_startState->get() && this->m_goalState->get())
	{
		auto *ss = this->m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
		auto *gs = this->m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>();
		prismStart = {ss->values[0], ss->values[1]};
		prismGoal  = {gs->values[0], gs->values[1]};
	}

	// Ensure the revolute objective has access to prismatic progress for rear-loaded scaling
	revoluteJointObj->setPrismaticStartGoal(prismStart, prismGoal);

	// Create and return the composite objective (weighted sum with coupling + rear-loaded mismatch
	// + gating + force-aware follow-the-leader swept volume)
	auto composite = std::make_shared<CompositeStateCostIntegral>(this->m_si, backboneLengthObj, revoluteJointObj, this->getFollowTheLeaderObjective(), CompositeParams::w_backbone, CompositeParams::w_revolute, goalAngles, prismStart, prismGoal);

	// Provide the same conservative combined cost-to-go heuristic as before
	composite->setCostToGoHeuristic([this, composite, backboneLengthObj, revoluteJointObj](const ompl::base::State *s, const ompl::base::Goal *g) -> ompl::base::Cost
									{
		double sum = 0.0;
		// backbone heuristic (scaled by new w_backbone)
		const auto &compB = backboneLengthObj;
		if (compB && compB->hasCostToGoHeuristic())
			sum += CompositeParams::w_backbone * compB->costToGo(s, g).value();
		// revolute heuristic (scaled by new w_revolute; coupling/mismatch/gating excluded for admissibility)
		const auto &compR = revoluteJointObj;
		if (compR && compR->hasCostToGoHeuristic())
			sum += CompositeParams::w_revolute * compR->costToGo(s, g).value();
		return ompl::base::Cost(sum); });

	return ompl::base::OptimizationObjectivePtr(composite);
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getBalancedObjective()
{
	auto lengthObj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si));
	auto clearObj(std::make_shared<CTR_ClearanceObjective<controlInputs>>(this->m_si));

	auto opt(std::make_shared<ompl::base::MultiOptimizationObjective>(this->m_si));
	opt->addObjective(lengthObj, 10.00);
	opt->addObjective(clearObj, 1.00);

	// Provide a conservative combined cost-to-go: weighted sum of available
	// component heuristics. If a component lacks a heuristic, its contribution
	// is treated as zero (admissible underestimate).
	opt->setCostToGoHeuristic([opt](const ompl::base::State *s, const ompl::base::Goal *g)
								  -> ompl::base::Cost
							  {
		double sum = 0.0;
		for (std::size_t i = 0; i < opt->getObjectiveCount(); ++i)
		{
			const auto &comp = opt->getObjective(static_cast<unsigned int>(i));
			double w = opt->getObjectiveWeight(static_cast<unsigned int>(i));
			if (comp && comp->hasCostToGoHeuristic())
			{
				sum += w * comp->costToGo(s, g).value();
			}
		}
		return ompl::base::Cost(sum); });

	return ompl::base::OptimizationObjectivePtr(opt);
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getPathLengthObjWithCostToGo()
{
	auto obj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si));
	// Use a state-space admissible cost-to-go for the CTR state vector.
	// This provides a tighter, valid heuristic for informed sampling.
	obj->setCostToGoHeuristic(&ctrCostToGoal);

	return obj;
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getBackboneLengthObjective()
{
	auto obj = std::make_shared<CTR_BackboneLengthObjective<controlInputs>>(this->m_si, this->m_CTR_model);

	// Provide a PINNs-backed cost-to-go heuristic. We compute two heuristics:
	//  1) the existing conservative state-space heuristic (ctrCostToGoal), which
	//     is known to be admissible; and
	//  2) a tip-position Euclidean distance computed by the PINNs forward model
	//     (potentially much tighter). To preserve admissibility we return the
	//     minimum of these two values (min <= ctrStateSpace => still admissible).
	obj->setCostToGoHeuristic([this](const ompl::base::State *state, const ompl::base::Goal *goal) -> ompl::base::Cost
							  {
		using namespace ompl::base;

		// conservative admissible baseline (backbone-based, same units as the objective)
		auto backboneBaseline = [this](const ompl::base::State *s, const ompl::base::Goal *goal) -> ompl::base::Cost
		{
			using namespace ompl::base;
			if (!goal || !s)
				return Cost(std::numeric_limits<double>::infinity());

			// weights used in CTR_BackboneLengthObjective
			constexpr double c1 = 1000.00;
			constexpr double c2 = 6000.00;
			constexpr double c3 = 3000.00;
			constexpr double c4 = 500.00;

			// Helper to convert an ompl RealVector state to a blaze vector
			auto stateToTauLocal = [this](const State *s)
			{
				blaze::StaticVector<double, controlInputs> q;
				if (!s)
					return q; // zero-initialized
				const auto *rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
				std::copy_n(rv->values, controlInputs, q.begin());
				return q;
			};

			double best = std::numeric_limits<double>::infinity();

			if (goal->hasType(GoalType::GOAL_STATE))
			{
				const GoalState *gs = goal->as<GoalState>();
				const State *gstate = gs->getState();
				if (!gstate)
					return Cost(std::numeric_limits<double>::infinity());

				const auto q_s = stateToTauLocal(s);
				const auto q_g = stateToTauLocal(gstate);
				const blaze::StaticVector<double, 3UL> Ls = this->m_CTR_model.getOverallLen();
				const double s1 = Ls[0] + q_s[0], s2 = Ls[1] + q_s[1], s3 = Ls[2];
				const double g1 = Ls[0] + q_g[0], g2 = Ls[1] + q_g[1], g3 = Ls[2];

				const double term1 = std::fabs(s1 - g1);
				const double term2 = std::fabs((s1 - s2) - (g1 - g2));
				const double term3 = std::fabs((s2 - s3) - (g2 - g3));
				const double term4 = std::fabs((s1 - s3) - (g1 - g3));

				best = c1 * term1 + c2 * term2 + c3 * term3 + c4 * term4;
			}
			else if (goal->hasType(GoalType::GOAL_STATES))
			{
				const GoalStates *gss = goal->as<GoalStates>();
				std::size_t count = gss->getStateCount();
				for (std::size_t i = 0; i < count; ++i)
				{
					const State *gs = gss->getState(i);
					if (!gs)
						continue;
					const auto q_s = stateToTauLocal(s);
					const auto q_g = stateToTauLocal(gs);
					const blaze::StaticVector<double, 3UL> Lg = this->m_CTR_model.getOverallLen();
					const double s1 = Lg[0] + q_s[0], s2 = Lg[1] + q_s[1], s3 = Lg[2];
					const double g1 = Lg[0] + q_g[0], g2 = Lg[1] + q_g[1], g3 = Lg[2];

					const double term1 = std::fabs(s1 - g1);
					const double term2 = std::fabs((s1 - s2) - (g1 - g2));
					const double term3 = std::fabs((s2 - s3) - (g2 - g3));
					const double term4 = std::fabs((s1 - s3) - (g1 - g3));

					const double val = c1 * term1 + c2 * term2 + c3 * term3 + c4 * term4;
					if (val < best)
						best = val;
				}
			}
			else
			{
				return Cost(std::numeric_limits<double>::infinity());
			}

			return Cost(best);
		};
		const Cost baseline = backboneBaseline(state, goal);

		// Helper to convert an ompl RealVector state to a blaze vector
		auto stateToTau = [this](const State *s)
		{
			blaze::StaticVector<double, controlInputs> q;
			if (!s)
				return q; // zero-initialized
			const auto *rv = s->as<ompl::base::RealVectorStateSpace::StateType>();
			std::copy_n(rv->values, controlInputs, q.begin());
			return q;
		};

		// Compute PINNs tip-distance heuristic to the goal. For single/sets of
		// goal states take the minimum tip distance. For other goal types fall
		// back to baseline's value.
		double pinn_min = std::numeric_limits<double>::infinity();

		if (!goal)
			return baseline;

		// current tip (evaluated on the loaded robot: f_ext applied)
		blaze::StaticVector<double, 3UL> tip_cur;
		const auto q_cur = stateToTau(state);
		try
		{
			this->m_CTR_model.getPosDistal(q_cur, this->m_externalForce, tip_cur);
		}
		catch (...) // be conservative: if PINNs fails, return baseline
		{
			return baseline;
		}

		if (goal->hasType(GoalType::GOAL_STATE))
		{
			const GoalState *gs = goal->as<GoalState>();
			const State *gstate = gs->getState();
			if (gstate)
			{
				const auto qg = stateToTau(gstate);
				blaze::StaticVector<double, 3UL> tip_goal;
				try
				{
					this->m_CTR_model.getPosDistal(qg, this->m_externalForce, tip_goal);
				}
				catch (...)
				{
					return baseline;
				}
				const double d = blaze::norm(tip_cur - tip_goal);
				pinn_min = std::min(pinn_min, d);
			}
		}
		else if (goal->hasType(GoalType::GOAL_STATES))
		{
			const GoalStates *gss = goal->as<GoalStates>();
			std::size_t count = gss->getStateCount();
			for (std::size_t i = 0; i < count; ++i)
			{
				const State *gs = gss->getState(i);
				if (!gs)
					continue;
				const auto qg = stateToTau(gs);
				blaze::StaticVector<double, 3UL> tip_goal;
				try
				{
					this->m_CTR_model.getPosDistal(qg, this->m_externalForce, tip_goal);
				}
				catch (...)
				{
					continue; // skip problematic goal states
				}
				const double d = blaze::norm(tip_cur - tip_goal);
				if (d < pinn_min)
					pinn_min = d;
			}
		}
		else
		{
			// For regions or sampleable regions we don't have an easy PINNs-based
			// heuristic, so fall back to baseline.
			return baseline;
		}

		if (pinn_min == std::numeric_limits<double>::infinity())
			return baseline;

		// Convert PINNs tip-distance (meters) to a conservative objective-unit
		// lower bound by multiplying by a small per-meter cost factor. We use
		// the smallest weight as a conservative conversion so units match and
		// admissibility is preserved.
		constexpr double per_meter_lower_bound = std::min(std::min(1000.0, 6000.0), std::min(3000.0, 500.0));
		const double pinn_converted = pinn_min * per_meter_lower_bound;

		// Return the minimum of the backbone baseline and the converted PINNs
		// heuristic to preserve admissibility while allowing tighter PINNs
		// guidance when available.
		const double best = std::min(baseline.value(), pinn_converted);
		return Cost(best); });

	return obj;
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getFollowTheLeaderObjective()
{
	// The FTL objective is created once in the constructor and shared so that
	// setCTR_externalForce() has a single propagation point and shape caches
	// are reused across objectives and phases.
	return this->m_ftlObjective;
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::getRevoluteJointsAndFTLObjective(const std::array<double, 2UL> &goal)
{
	// Revolute-error state cost + follow-the-leader swept-volume edge cost.
	// Used in Phase 1 of planTwoPhase() with optimizing planners: rotating the
	// pre-curved tubes sweeps the backbone laterally, and the magnitude of
	// that sweep depends on the external tip force, so the FTL term makes the
	// rotation phase force-aware instead of purely joint-space.
	class RevoluteFTLObjective : public ompl::base::OptimizationObjective
	{
	public:
		RevoluteFTLObjective(const ompl::base::SpaceInformationPtr &si,
							 const ompl::base::OptimizationObjectivePtr &revolute,
							 const ompl::base::OptimizationObjectivePtr &ftl)
			: ompl::base::OptimizationObjective(si), m_revolute(revolute), m_ftl(ftl)
		{
			description_ = "revolute error + follow-the-leader swept volume";
			if (m_revolute && m_revolute->hasCostToGoHeuristic())
				this->setCostToGoHeuristic([rev = m_revolute](const ompl::base::State *s, const ompl::base::Goal *g) -> ompl::base::Cost
										   { return rev->costToGo(s, g); });
		}

		ompl::base::Cost stateCost(const ompl::base::State *s) const override
		{
			return m_revolute ? m_revolute->stateCost(s) : this->identityCost();
		}

		ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
		{
			double cost = m_revolute ? m_revolute->motionCost(s1, s2).value() : 0.0;
			if (m_ftl)
				cost += m_ftl->motionCost(s1, s2).value();
			return ompl::base::Cost(cost);
		}

	private:
		ompl::base::OptimizationObjectivePtr m_revolute;
		ompl::base::OptimizationObjectivePtr m_ftl;
	};

	auto revolute = this->getRevoluteJointObjective(goal);
	return std::make_shared<RevoluteFTLObjective>(this->m_si, revolute, this->getFollowTheLeaderObjective());
}

template <size_t controlInputs>
ompl::base::PlannerPtr Planner<controlInputs>::allocatePlanner(optimalPlanner plannerType)
{
	switch (plannerType)
	{
	case optimalPlanner::PLANNER_AITSTAR:
	{
		return std::make_shared<ompl::geometric::AITstar>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_BFMTSTAR:
	{
		return std::make_shared<ompl::geometric::BFMT>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_BITSTAR:
	{
		return std::make_shared<ompl::geometric::BITstar>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_CFOREST:
	{
		return std::make_shared<ompl::geometric::CForest>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_FMTSTAR:
	{
		return std::make_shared<ompl::geometric::FMT>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_INF_RRTSTAR:
	{
		return std::make_shared<ompl::geometric::InformedRRTstar>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_PRMSTAR:
	{
		return std::make_shared<ompl::geometric::PRMstar>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_RRTSTAR:
	{
		return std::make_shared<ompl::geometric::RRTstar>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_SORRTSTAR:
	{
		return std::make_shared<ompl::geometric::SORRTstar>(this->m_si);
		break;
	}
	// NEW PLANNERS
	case optimalPlanner::PLANNER_PRM:
	{
		return std::make_shared<ompl::geometric::PRM>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_SPARS:
	{
		return std::make_shared<ompl::geometric::SPARS>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_RRT:
	{
		return std::make_shared<ompl::geometric::RRT>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_RRT_CONNECT:
	{
		return std::make_shared<ompl::geometric::RRTConnect>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_RRT_SHARP:
	{
		return std::make_shared<ompl::geometric::RRTsharp>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_RLRT:
	{
		return std::make_shared<ompl::geometric::RLRT>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_AORRTC:
	{
		return std::make_shared<ompl::geometric::AORRTC>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_TRRTSTAR:
	{
		return std::make_shared<ompl::geometric::TRRTstar>(this->m_si);
		break;
	}
	case optimalPlanner::PLANNER_ATRRT:
	{
		return std::make_shared<ompl::geometric::ATRRT>(this->m_si);
		break;
	}
	default:
	{
		OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
		return ompl::base::PlannerPtr(); // Address compiler warning re: no return value.
		break;
	}
	}
}

template <size_t controlInputs>
ompl::base::OptimizationObjectivePtr Planner<controlInputs>::allocateObjective(planningObjective objectiveType)
{
	switch (objectiveType)
	{
	case planningObjective::OBJECTIVE_PATH_CLEARANCE:
		return getClearanceObjective();
		break;
	case planningObjective::OBJECTIVE_PATH_LENGTH:
		return getPathLengthObjective();
		break;
	case planningObjective::OBJECTIVE_REVJOINTS_AND_BACKBONE:
	{
		std::array<double, 2UL> revJointGoals = {
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[2UL],
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[3UL]};
		return RevoluteJointsAndBackboneLengthObjective(revJointGoals);
		break;
	}
	case planningObjective::OBJECTIVE_PATH_LENGTH_COST2GO:
		return getPathLengthObjWithCostToGo();
		break;
	case planningObjective::OBJECTIVE_THRESHOLD_PATH_LENGTH:
		return getThresholdPathLengthObjctive();
		break;
	case planningObjective::OBJECTIVE_WEIGHTED_COMBO:
		return getBalancedObjective();
		break;
	case planningObjective::OBJECTIVE_BACKBONE_LENGTH:
		return getBackboneLengthObjective();
		break;
	case planningObjective::OBJECTIVE_REVJOINTS_ONLY:
	{
		std::array<double, 2UL> revJointGoals = {
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[2UL],
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[3UL]};
		return getRevoluteJointObjective(revJointGoals);
		break;
	}
	case planningObjective::OBJECTIVE_FOLLOW_THE_LEADER:
		return getFollowTheLeaderObjective();
		break;
	case planningObjective::OBJECTIVE_REVJOINTS_AND_FTL:
	{
		std::array<double, 2UL> revJointGoals = {
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[2UL],
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[3UL]};
		return getRevoluteJointsAndFTLObjective(revJointGoals);
		break;
	}
	default:
		OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
		return ompl::base::OptimizationObjectivePtr();
		break;
	}
}

template <size_t controlInputs>
void Planner<controlInputs>::setCTR_externalForce(const blaze::StaticVector<double, 3UL>& force)
{
	this->m_externalForce = force;

	// Propagate to the FTL objective: its swept-volume costs are evaluated on
	// the loaded robot, so a stale force would silently mis-cost every edge.
	if (this->m_ftlObjective)
		this->m_ftlObjective->setExternalForce(force);

	// Propagate to any live informed samplers so goal-tip biasing uses the
	// deflected tip position.
	if (this->m_informedSampler)
	{
		if (auto inf = std::dynamic_pointer_cast<CTR_PINNsInformedSampler<controlInputs>>(this->m_informedSampler))
			inf->setExternalForce(force);
		else if (auto inf2 = std::dynamic_pointer_cast<CTR_PINNsInformedStateSampler<controlInputs>>(this->m_informedSampler))
			inf2->setExternalForce(force);
	}
}

template <size_t controlInputs>
bool Planner<controlInputs>::solveInverseKinematics(JointVector &q, const blaze::StaticVector<double, 3UL> &targetTip, const double posTol)
{
	// IK on the loaded robot: Jacobian and forward model both see f_ext.
	this->m_CTR_model.posCTRL(q, targetTip, posTol, this->m_externalForce);

	blaze::StaticVector<double, 3UL> tip;
	this->m_CTR_model.getPosDistal(q, this->m_externalForce, tip);
	return blaze::norm(targetTip - tip) <= posTol;
}
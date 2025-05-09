#pragma once

#include <ompl/base/SpaceInformation.h>
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
#include <iostream>
// For accessing information about the CTR object for whom we're planning
#include "CTR.hpp"

// My own classes that I implemented
#include "CTR_StateSampler.hpp"
#include "CTR_StateValidityChecker.hpp"
#include "CTR_RevoluteJointObjective.hpp"

class Planner
{
public:
	// An enum of supported optimal planners, alphabetical order
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
		// new ones
		PLANNER_PRM,
		PLANNER_SPARS,
		PLANNER_RRT,
		PLANNER_RRT_CONNECT,
		PLANNER_RRT_SHARP,
		PLANNER_RLRT
	};

	// An enum of the supported optimization objectives, alphabetical order
	enum class planningObjective
	{
		OBJECTIVE_PATHCLEARANCE,
		OBJECTIVE_PATHLENGTH,
		OBJECTIVE_REVJOINTSANDPATHLENGTH,
		OBJECTIVE_PATHLENGTH_COST2GO,
		OBJECTIVE_THRESHOLDPATHLENGTH,
		OBJECTIVE_WEIGHTEDCOMBO,
		OBJECTIVE_BACKBONE_LENGTH,
		OBJECTIVE_FOLLOW_THE_LEADER
	};

	// default constructor ==> deleted
	Planner() = delete;

	// overloaded constructor
	Planner(CTR& _CTR_StateValidator, CTR& _CTR_MotionValidator, CTR& _CTR_ObjectiveFunction);

	// class destructor
	~Planner() = default;

	// setter method for the starting CTR configuration
	bool setStartState(const blaze::StaticVector<double, 6UL> &q_0);

	// setter method for the goal CTR configuration
	bool setGoalState(const blaze::StaticVector<double, 6UL> &q_f);

	// method to effectively start the robot planning
	void plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, const std::string &tmpDir, const std::string &outputFile);

	ompl::base::OptimizationObjectivePtr getRevoluteJointObjective(const std::array<double, 3UL>& goal);
	
	ompl::base::OptimizationObjectivePtr getPathLengthObjective();

	ompl::base::OptimizationObjectivePtr getThresholdPathLengthObjctive();

	ompl::base::OptimizationObjectivePtr getClearanceObjective();

	ompl::base::OptimizationObjectivePtr RevoluteJointsAndBackboneLengthObjective(const std::array<double, 3UL>& goal);

	ompl::base::OptimizationObjectivePtr getBalancedObjective();

	ompl::base::OptimizationObjectivePtr getPathLengthObjWithCostToGo();

	ompl::base::OptimizationObjectivePtr getBackboneLengthObjective();

	ompl::base::PlannerPtr allocatePlanner(optimalPlanner plannerType);

	ompl::base::OptimizationObjectivePtr allocateObjective(planningObjective objectiveType);

private:
	// CTR objects to be used during state validity and motion validation checks
	CTR m_CTR_StateValidator, m_CTR_MotionValidator, m_CTR_ObjectiveFunction;  

	// construct the state space we are planning in
	std::shared_ptr<ompl::base::RealVectorStateSpace> m_space;

	// pointer to real vector used to set the boundaries of each entry of the state space vector
	std::shared_ptr<ompl::base::RealVectorBounds> m_coordBound;

	// construct an instance of  space information from this state space
	ompl::base::SpaceInformationPtr m_si;

	// // constrict an instance of a custom state sampler
	// ompl::base::ValidStateSamplerPtr m_stateSampler;
	// std::shared_ptr<CTR_StateSampler> m_stateSampler;

	// constructs an instance of state validity checker for sampling
	ompl::base::StateValidityCheckerPtr m_stateValidityChecker;

	// create a problem instance
	ompl::base::ProblemDefinitionPtr m_pdef;

	// Planner instance
	ompl::base::PlannerPtr m_optimizingPlanner;

	// start state
	ompl::base::ScopedStatePtr m_startState;

	// goal state
	ompl::base::ScopedStatePtr m_goalState;
};
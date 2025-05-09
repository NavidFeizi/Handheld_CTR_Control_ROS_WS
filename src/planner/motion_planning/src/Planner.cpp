#include "Planner.hpp"
#include "CTR_BackboneLengthObjective.hpp"
#include "CTR_ClearanceObjective.hpp"
#include "CTR_DiscreteMotionValidator.hpp"
#include "CTR_StateSampler.hpp"
#include "CTR_StateValidityChecker.hpp"

#include <blaze/Math.h>
#include <filesystem>

// Overloaded Constructor
Planner::Planner(CTR &_CTR_StateValidator, CTR &_CTR_MotionValidator, CTR &_CTR_ObjectiveFunction) : m_CTR_StateValidator(_CTR_StateValidator), m_CTR_MotionValidator(_CTR_MotionValidator), m_CTR_ObjectiveFunction(_CTR_ObjectiveFunction)
{
	// dimmension of the state space --> 6 DoF CTR
	const int dim = 6;
	// defining a state space over which the planning will take place
	m_space = std::make_shared<ompl::base::RealVectorStateSpace>(dim);

	m_coordBound = std::make_shared<ompl::base::RealVectorBounds>(dim);

	// ___ setting bounds for the linear joints ___

	auto Tb = m_CTR_StateValidator.getTubes();

	// beta_1
	m_coordBound->setLow(0, -Tb[0UL]->getStraightLen());
	m_coordBound->setHigh(0, 0.00);
	// beta_2
	m_coordBound->setLow(1, -Tb[1UL]->getStraightLen());
	m_coordBound->setHigh(1, 0.00);
	// beta_3
	m_coordBound->setLow(2, -Tb[2UL]->getStraightLen());
	m_coordBound->setHigh(2, 0.00);
	// alpha_1
	m_coordBound->setLow(3, -M_PI);
	m_coordBound->setHigh(3, M_PI);
	// alpha_2
	m_coordBound->setLow(4, -M_PI);
	m_coordBound->setHigh(4, M_PI);
	// alpha_3
	m_coordBound->setLow(5, -M_PI);
	m_coordBound->setHigh(5, M_PI);

	// setting the space bounds for the linear (prismatic) CTR joints
	m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(*m_coordBound);

	// Construct a space information instance for this compound state space
	m_si = std::make_shared<ompl::base::SpaceInformation>(m_space);

	// Set the object used to check which states in the space are valid
	m_stateValidityChecker = std::make_shared<CTR_StateValidityChecker>(this->m_si, this->m_CTR_StateValidator);

	// Create the state validity checker with your CTR instance
	m_si->setStateValidityChecker(m_stateValidityChecker);

	// Inside Planner constructor
	this->m_space->setStateSamplerAllocator([&](const ompl::base::StateSpace *space)
											{
		auto validityChecker = std::dynamic_pointer_cast<CTR_StateValidityChecker>(m_stateValidityChecker);
		auto sampler = std::make_shared<CTR_StateSampler>(space, *validityChecker, m_CTR_StateValidator);
    
		// Extract parameters from Planner's bounds and validity checker
		const double beta1_min = m_coordBound->low[0UL];
		const double beta2_min = m_coordBound->low[1UL];
		const double beta3_min = m_coordBound->low[2UL];
		const double clearance = m_CTR_StateValidator.getStageThickness(); // Or m_stateValidityChecker->clearance()
		
		// Set parameters on the sampler
		sampler->setParameters(beta1_min, beta2_min, beta3_min, clearance);
		
		return sampler; });

	// Create the motion validator with your CTR instance
	m_si->setMotionValidator(std::make_shared<CTR_DiscreteMotionValidator>(this->m_si, this->m_CTR_MotionValidator));

	// setup the space information from the Real state space
	this->m_si->setup();

	// setting up the pointer to the start state
	this->m_startState = std::make_shared<ompl::base::ScopedState<>>(m_space);

	// setting up the pointer to the goal state
	this->m_goalState = std::make_shared<ompl::base::ScopedState<>>(m_space);

	// create an instance of a planning problem
	this->m_pdef = std::make_shared<ompl::base::ProblemDefinition>(this->m_si);
}

bool Planner::setStartState(const blaze::StaticVector<double, 6UL> &q_0)
{
	// setting the start state for the CTR robot
	m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0UL] = q_0[0UL];
	m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1UL] = q_0[1UL];
	m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[2UL] = q_0[2UL];
	// values for the angular (revolute) joints
	m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[3UL] = q_0[3UL];
	m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[4UL] = q_0[4UL];
	m_startState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[5UL] = q_0[5UL];

	// checking if the candidate start state is a valid one
	if (this->m_stateValidityChecker->isValid(m_startState->get()))
	{
		this->m_pdef->clearStartStates();
		this->m_pdef->addStartState(m_startState->get());
		std::cout << "Start state has been updated!" << std::endl;
		return true;
	}
	else
	{
		throw std::runtime_error("Start State is invalid!");
		return false;
	}
}

bool Planner::setGoalState(const blaze::StaticVector<double, 6UL> &q_f)
{
	// setting the start state for the CTR robot
	m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[0UL] = q_f[0UL];
	m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[1UL] = q_f[1UL];
	m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[2UL] = q_f[2UL];
	// values for the angular (revolute) joints
	m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[3UL] = q_f[3UL];
	m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[4UL] = q_f[4UL];
	m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[5UL] = q_f[5UL];

	// checking if the candidate start state is a valid one
	if (this->m_stateValidityChecker->isValid(m_goalState->get()))
	{
		this->m_pdef->clearGoal();
		this->m_pdef->setGoalState(m_goalState->get());
		std::cout << "Goal state has been updated!" << std::endl;

		// Cast the motion validator and set goal betas:
		auto motion_validator = std::dynamic_pointer_cast<CTR_DiscreteMotionValidator>(m_si->getMotionValidator());
		// Cast the state sampler validator and set goal betas:
		auto validity_checker = std::dynamic_pointer_cast<CTR_StateValidityChecker>(m_si->getStateValidityChecker());

		if (motion_validator && validity_checker)
		{
			motion_validator->setGoalBetas(q_f[0UL], q_f[1UL], q_f[2UL]);
			validity_checker->setGoalBetas(q_f[0UL], q_f[1UL], q_f[2UL]);
		}

		return true;
	}
	else
	{
		throw std::runtime_error("Goal State is invalid!");
		return false;
	}
}

void Planner::plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, const std::string &dir, const std::string &outputFile)
{
	// // passing the start and goal states to the planning problem definition
	// this->m_pdef->setStartAndGoalStates(m_startState->get(), m_goalState->get());

	// Create the optimization objective specified by the objectiveType argument
	this->m_pdef->setOptimizationObjective(this->allocateObjective(objectiveType));

	// Construct the optimal planner specified the plannerType argument.
	this->m_optimizingPlanner = this->allocatePlanner(plannerType);

	this->m_optimizingPlanner->printProperties(std::cout);

	/*
							____ SETTING THE PLANNER'S RANGE ____

		1. The range typically refers to the maximum distance a planner is allowed to extend in the configuration space while attempting to connect two points
		2. Choosing an appropriate range is crucial for the performance of the planner. Too small of a range might result in a planner that explores the configuration space too slowly, while too large of a range might lead to inefficiencies or failure to find solutions. The optimal range often depends on the specific characteristics of the planning problem and the environment. It's usually a parameter that needs to be tuned based on experimentation and domain knowledge.
		3. In the process of randomly selecting states in the state space to attempt to go towards, the algorithm may in fact choose the actual goal state, if it knows it, with some probability. This probability is a real number between 0.0 and 1.0; its value should usually be around 0.05 and should not be too large. It is probably a good idea to use the default value.
	*/

	switch (plannerType)
	{
	case optimalPlanner::PLANNER_RRTSTAR:
		// range represents the maximum length of a motion to be added in the tree of motions
		this->m_optimizingPlanner->params()["range"] = 0.001;		// 
		this->m_optimizingPlanner->params()["goal_bias"] = 0.015;
		this->m_optimizingPlanner->params()["delay_collision_checking"] = true;
		this->m_optimizingPlanner->params()["rewire_factor"] = 2.00; // Recommended value: 1.0 to 2.0 times the range value.
		this->m_optimizingPlanner->params()["use_k_nearest"] = true;
		this->m_optimizingPlanner->params()["tree_pruning"] = true;
		this->m_optimizingPlanner->params()["number_sampling_attempts"] = 1000;
		this->m_optimizingPlanner->params()["prune_threshold"] = 0.001;
		this->m_optimizingPlanner->params()["informed_sampling"] = false;
		this->m_optimizingPlanner->params()["new_state_rejection"] = false;
		break;
	
	case optimalPlanner::PLANNER_RLRT:
		this->m_optimizingPlanner->params()["range"] = 0.001;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.015;
		this->m_optimizingPlanner->params()["keep_last_valid"] = false;
		break;

	case optimalPlanner::PLANNER_RRT:
		// range represents the maximum length of a motion to be added in the tree of motions
		this->m_optimizingPlanner->params()["range"] = 0.001;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.015;
		this->m_optimizingPlanner->params()["intermediate_states"] = true;
		break;
	
	case optimalPlanner::PLANNER_RRT_CONNECT:
		this->m_optimizingPlanner->params()["range"] = 0.001;
		this->m_optimizingPlanner->params()["intermediate_states"] = true;
		break;

	case optimalPlanner::PLANNER_INF_RRTSTAR:
		this->m_optimizingPlanner->params()["range"] = 0.001;
		this->m_optimizingPlanner->params()["goal_bias"] = 0.025;
		this->m_optimizingPlanner->params()["delay_collision_checking"] = true;
		this->m_optimizingPlanner->params()["number_sampling_attempts"] = 1000;
		// 0: random sampling | 1: ordered sampling
		this->m_optimizingPlanner->params()["ordered_sampling"] = false;
		// small values (0.001 - 0.01) prunes more conservatively. Moderate (0.01 - 0.1)
		this->m_optimizingPlanner->params()["prune_threshold"] = 0.001;
		this->m_optimizingPlanner->params()["rewire_factor"] = 1.75;
		this->m_optimizingPlanner->params()["use_k_nearest"] = true;
		break;

	case optimalPlanner::PLANNER_CFOREST:
		this->m_optimizingPlanner->params()["focus_search"] = true;
		this->m_optimizingPlanner->params()["num_threads"] = 4;
		break;

	case optimalPlanner::PLANNER_PRMSTAR:
		// this->m_optimizingPlanner->params()["range"] = 0.001;
		// this->m_optimizingPlanner->params()["goal_bias"] = 0.025;
		// this->m_optimizingPlanner->params()["max_nearest_neighbors"] = 20;
		// this->m_optimizingPlanner->params()["num_samples"] = 3000;
		break;

	case optimalPlanner::PLANNER_PRM:
		this->m_optimizingPlanner->params()["max_nearest_neighbors"] = 5000;
		// this->m_optimizingPlanner->params()["max_neighbors"] = 10;
		// this->m_optimizingPlanner->params()["connection_attempts"] = 10;

		// this->m_optimizingPlanner->params()["num_samples"] = 1000;
		break;

	default:
		throw std::runtime_error("Planner not supported!");
	}

	// Set the problem instance for the specified planner to solve
	this->m_optimizingPlanner->setProblemDefinition(this->m_pdef);
	this->m_optimizingPlanner->setup();

	m_optimizingPlanner->printSettings(std::cout);

	// attempt to solve the planning problem in the given runtime
	ompl::base::PlannerStatus solved = this->m_optimizingPlanner->solve(runTime);

	// if ((solved == ompl::base::PlannerStatus::EXACT_SOLUTION) || (solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION))
	if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
	{
		// Output the length of the path found
		std::cout
			<< this->m_optimizingPlanner->getName()
			<< " found a solution of length "
			<< this->m_pdef->getSolutionPath()->length()
			<< " with an optimization objective value of "
			<< this->m_pdef->getSolutionPath()->cost(this->m_pdef->getOptimizationObjective()) << std::endl;

		// printing the solution to the console
		// std::static_pointer_cast<ompl::geometric::PathGeometric>(this->m_pdef->getSolutionPath())->print(std::cout);

		// If a filename was specified, output the path as a matrix to
		// that file for visualization
		if (!outputFile.empty())
		{
			const std::filesystem::path tmpDir = dir; // "../../OutputFiles";
			const std::filesystem::path fileName = tmpDir / outputFile;

			// Ensure the directory exists
			if (!std::filesystem::exists(tmpDir))
			{
				std::filesystem::create_directories(tmpDir);
			}

			std::ofstream outFile(fileName, std::ios::out | std::ios::trunc);

			if (outFile.is_open())
			{
				// Set precision for the entire file
				outFile << std::fixed << std::setprecision(10);

				auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(this->m_pdef->getSolutionPath());

				// Loop through the states and write each state as a line in CSV format
				for (const auto &state : path->getStates())
				{
					// Assuming your state is of a type that can be converted to a string with comma-separated values
					auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
					if (pos)
					{
						for (size_t i = 0UL; i < 6UL; ++i)
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

ompl::base::OptimizationObjectivePtr Planner::getRevoluteJointObjective(const std::array<double, 3UL>& goal)
{
	return std::make_shared<CTR_RevoluteJointObjective>(this->m_si, goal);
}

ompl::base::OptimizationObjectivePtr Planner::getPathLengthObjective()
{
	return std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si);
}

ompl::base::OptimizationObjectivePtr Planner::getThresholdPathLengthObjctive()
{
	auto obj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si));
	obj->setCostThreshold(ompl::base::Cost(std::numeric_limits<double>::min())); // return immediately after find a solution
	return obj;
}

ompl::base::OptimizationObjectivePtr Planner::getClearanceObjective()
{
	return std::make_shared<CTR_ClearanceObjective>(this->m_si);
}

ompl::base::OptimizationObjectivePtr Planner::RevoluteJointsAndBackboneLengthObjective(const std::array<double, 3UL>& goal)
{
	auto backboneLengthObj(std::make_shared<CTR_BackboneLengthObjective>(this->m_si, this->m_CTR_ObjectiveFunction));
	auto revoluteJointObj(std::make_shared<CTR_RevoluteJointObjective>(this->m_si, goal));

	auto opt(std::make_shared<ompl::base::MultiOptimizationObjective>(this->m_si));
	opt->addObjective(backboneLengthObj, 1.00);
	opt->addObjective(revoluteJointObj, 100.00);

	return ompl::base::OptimizationObjectivePtr(opt);
}

ompl::base::OptimizationObjectivePtr Planner::getBalancedObjective()
{
	auto lengthObj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si));
	auto clearObj(std::make_shared<CTR_ClearanceObjective>(this->m_si));

	auto opt(std::make_shared<ompl::base::MultiOptimizationObjective>(this->m_si));
	opt->addObjective(lengthObj, 10.00);
	opt->addObjective(clearObj, 1.00);

	return ompl::base::OptimizationObjectivePtr(opt);
}

ompl::base::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo()
{
	auto obj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(this->m_si));
	obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
	return obj;
}

ompl::base::OptimizationObjectivePtr Planner::getBackboneLengthObjective()
{
	return std::make_shared<CTR_BackboneLengthObjective>(this->m_si, this->m_CTR_ObjectiveFunction);
}

ompl::base::PlannerPtr Planner::allocatePlanner(optimalPlanner plannerType)
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
	default:
	{
		OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
		return ompl::base::PlannerPtr(); // Address compiler warning re: no return value.
		break;
	}
	}
}

ompl::base::OptimizationObjectivePtr Planner::allocateObjective(planningObjective objectiveType)
{
	switch (objectiveType)
	{
	case planningObjective::OBJECTIVE_PATHCLEARANCE:
		return getClearanceObjective();
		break;
	case planningObjective::OBJECTIVE_PATHLENGTH:
		return getPathLengthObjective();
		break;
	case planningObjective::OBJECTIVE_REVJOINTSANDPATHLENGTH:
	{
		std::array<double, 3UL> revJointGoals = {
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[3UL],
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[4UL],
			m_goalState->get()->as<ompl::base::RealVectorStateSpace::StateType>()->values[5UL]
		};
		return getRevoluteJointObjective(revJointGoals);
		break;
	}
	case planningObjective::OBJECTIVE_PATHLENGTH_COST2GO:
		return getPathLengthObjWithCostToGo();
		break;
	case planningObjective::OBJECTIVE_THRESHOLDPATHLENGTH:
		return getThresholdPathLengthObjctive();
		break;
	case planningObjective::OBJECTIVE_WEIGHTEDCOMBO:
		return getBalancedObjective();
		break;
	case planningObjective::OBJECTIVE_BACKBONE_LENGTH:
		return getBackboneLengthObjective();
		break;
	default:
		OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
		return ompl::base::OptimizationObjectivePtr();
		break;
	}
}
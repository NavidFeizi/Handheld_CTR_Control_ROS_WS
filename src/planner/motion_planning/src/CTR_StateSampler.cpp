#include "CTR_StateSampler.hpp"
#include <memory>
#include <cmath>
#include <iostream>

// overloaded constructor
CTR_StateSampler::CTR_StateSampler(const ompl::base::StateSpace *space, CTR_StateValidityChecker& validityChecker, CTR& ctr) : ompl::base::StateSampler(space), m_validityChecker(validityChecker), m_ctr(ctr)
{
   m_Clr = this->m_ctr.getStageThickness();  ;
   // setting the minimum allowable actuation value for the linear joints
   m_beta1_Min = 0.00;
   m_beta2_Min = 0.00;
   m_beta3_Min = 0.00;

   std::cout << "Sampler: CTR_StateSampler" << std::endl;
}

// // Copy Constructor
// CTR_StateSampler::CTR_StateSampler(const CTR_StateSampler &other)
//     : ValidStateSampler(other.si_)
// {
//    // Copy over any member variables if needed
//    this->rng_ = other.rng_;
//    this->beta1_Min = other.m_beta1_Min;
//    this->beta2_Min = other.m_beta2_Min;
//    this->beta3_Min = other.m_beta3_Min;
//    this->m_Clr = other.m_Clr;
// }


void CTR_StateSampler::sampleUniform(ompl::base::State *state)
{
//    std::cout << "CTR_StateSampler called!" << std::endl;

   ompl::base::RealVectorStateSpace::StateType *sampled_State = state->as<ompl::base::RealVectorStateSpace::StateType>();

   // Get beta goals directly from the validity checker
   const double beta1_goal = this->m_validityChecker.getBeta1Goal();
   const double beta2_goal = this->m_validityChecker.getBeta2Goal();
   const double beta3_goal = this->m_validityChecker.getBeta3Goal();
   
   // ===============================================================================

   // Randomly select a phase to sample from
   double u = rng_.uniformReal(0.00, 1.00);

   // decide phase with desired weights:
   size_t phase;

   if (u < 0.80)         phase = 1; // phase 1: 80% selected of the time
   else if (u < 0.975)   phase = 2; // phase 2: 17.5% selected of the time
   else                  phase = 3; // phase 3: 2.5% selected of the time

   double beta_1, beta_2, beta_3;

   switch (phase) 
   {
      // sampling the linear joints ensuring that beta_1 < beta_2 < beta_3
       case 1: // Phase 1: All betas evolve independently
           beta_1 = rng_.uniformReal(m_beta1_Min, beta1_goal); // Clamp to valid range
           beta_2 = rng_.uniformReal(std::max(m_beta2_Min, beta_1 + m_Clr), beta2_goal);
           beta_3 = rng_.uniformReal(std::max(m_beta3_Min, beta_2 + m_Clr), beta3_goal);
           break;
       case 2: // Phase 2: Beta3 fixed, beta1, beta2 evolve independently
           beta_3 = beta3_goal;
           beta_2 = rng_.uniformReal(m_beta2_Min, beta2_goal);
           beta_1 = rng_.uniformReal(m_beta1_Min, std::min(beta1_goal, beta_2 - m_Clr));           
           break;
       case 3: // Phase 3: Beta3, beta2 fixed, beta1 varies
           beta_3 = beta3_goal;
           beta_2 = beta2_goal;
           beta_1 = rng_.uniformReal(m_beta1_Min, beta1_goal);
           break;
   }

   // ===============================================================================

   sampled_State->values[0UL] = beta_1;
   sampled_State->values[1UL] = beta_2;
   sampled_State->values[2UL] = beta_3; 

   // sampling the revolute joints in the interval: [-pi, pi]
   sampled_State->values[3UL] = rng_.uniformReal(-M_PI, M_PI);
   sampled_State->values[4UL] = rng_.uniformReal(-M_PI, M_PI);
   sampled_State->values[5UL] = rng_.uniformReal(-M_PI, M_PI);
}

void CTR_StateSampler::setParameters(double b1, double b2, double b3, double clearance)
{
   this->m_beta1_Min = b1;
   this->m_beta2_Min = b2;
   this->m_beta3_Min = b3;
   this->m_Clr = clearance;
}

void CTR_StateSampler::setGoalBetas(double b1, double b2, double b3)
{
    this->m_beta1_goal = b1;
    this->m_beta2_goal = b2;
    this->m_beta3_goal = b3;
}
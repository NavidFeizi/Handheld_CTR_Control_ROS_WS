//
//

#include "TendonDriven.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/jointspace.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

void print_tip(double t, blaze::StaticVector<double, 3UL> position, blaze::StaticVector<double, 2UL> q);

class CatheterSim : public rclcpp::Node
{
public:
  CatheterSim() : Node("catheter_sim"), count_(0)
  {
    m_sample_time = 1E-3;         // time infinitesimal [sec]
    dt_publisher_ = 5E-3; // time infinitesimal [sec]
    setupParametersAndInitializeRobot();
    setupROSInterfaces();
    // node initialization time
    rclcpp::Time now = this->get_clock()->now();
    m_t_init = static_cast<double>(now.nanoseconds()) / 1E9;
  }

  ~CatheterSim()
  {
    if (robot_)
    {
      robot_->resetStaticSimulation();
      robot_->resetDynamicSimulation();
    }
  }

private:
  void setupParametersAndInitializeRobot()
  {
    /* Simulation params */
    double E = 19.16795366474113845825195E9; // Young's modulus [GPa] 3.39794625E9;
    double nu = 0.35;                        // Poisson's ratio --> for Nitinol: [0.30, 0.50]
    double G = E / (2 * (1.00 + nu));        // Shear modulus [GPa]
    double radius = 4.6769660928E-4;         // radius of center beam [m]
    double mass = 2.99447132638E-3;          // total mass of the sheath [kg]
    double Length = 0.064;                   // total length of the sheath [m]
    double basePlateToMotor = 0.00;          // distance between base plate and motor [m]
    double tendonOffset = 1.25E-3;           // offset distance between the tendons and backbone [m]
    double alpha = 0.00;                     // backward differentiation parameter BDF-alpha ==> alpha in [-0.5, 0.00]
    double dampingBendTwist = 1.25E-6;       // viscous damping for bending & twist
    double dampingShearExt = 2.00E-5;        // viscous damping for shear & extension
    double airDragCoeff = 4.00E-5;           // viscous air drag coeffic

    const double distalMass = 00.00E-3;                 // 30 grams weight at the end of the catheter
    const double probeMass = 340.00E-6;                 // 339.00E-6;					// mass of the ultrasound probe at the distal end of the catheter 0.339 gram
    const double g = -9.81;                             // acceleration og gravity [m/s/s]
    const double weight = (probeMass + distalMass) * g; // weight [Newtons]
    blaze::StaticVector<double, 3UL> distalForce = {weight, 0.00, 0.00};

    /****************************************************************************************************
     * 		INSTANTIATING A 4-TENDON CATHETER OBJECT WITH 200 DISCRETIZED POINTS ALONG ITS BACKBONE		*
     *****************************************************************************************************/
    // asserts if alpha parameter for implicit time differentiation is valid
    auto isWithinRange = [](double alpha) -> bool
    {
      if (alpha >= -0.50 && alpha <= 0.00)
      {
        return true;
      }
      else
      {
        std::cerr << "Execution Interrupted ==> BDF-alpha parameter does not lie within allowed interval [-0.50, 0.00]!" << std::endl;
        exit(1);
      }
    };

    isWithinRange(alpha);
    // instantiating a tendon-driven robot (4 tendons, 200 discretized arc-length points)
    robot_ = std::make_unique<TendonDriven<backboneDiscretizedPoints_, numberOfTendons_>>(E, G, radius, mass, Length, alpha, m_sample_time, tendonOffset, dampingBendTwist, dampingShearExt, airDragCoeff);
    robot_->setExternalDistalForce(distalForce);
    // setting the actuation input at time t = 0.00 second
    q = blaze::StaticVector<double, numberOfTendons_>(0.0);
    robot_->setTendonActuation(q);
    bool convergence;
    // solve the static problem -> initial condition for the dynamics problem
    convergence = robot_->solveBVP<mathOp::cosseratEquations::STATIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
    if (!convergence)
    {
      initGuess_ *= 0.75;
      convergence = robot_->solveBVP<mathOp::cosseratEquations::STATIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
    }
    if (!convergence)
    {
      initGuess_ *= 0.25;
      convergence = robot_->solveBVP<mathOp::cosseratEquations::STATIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
      if (!convergence)
        std::cout << "Static model still hasn't converged yet!" << std::endl;
    }

    // set the parameters for the implicit differentiation for time discretization of PDEs
    robot_->setTimeDiscretization();
  }

  void setupROSInterfaces()
  {
    //// Publisher, Subscribers, and Timers
    auto sim_sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    auto publish_sample_time = std::chrono::milliseconds(static_cast<int>(m_sample_time * 1e3));
    // publisher to publish model outputs
    m_publisher_control = this->create_publisher<interfaces::msg::Taskspace>("Target", 10);
    // initialize timer to call simulation step and publish states
    // Create callback groups
    m_callback_group_sim = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_pub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create wall timers with different callback groups
    // sim_timer_ = this->create_wall_timer(sim_sample_time, std::bind(&CatheterSim::simulationStep, this), callback_group_sim_);
    // publish_timer_ = this->create_wall_timer(publish_sample_time, std::bind(&CatheterSim::publishStates, this), callback_group_publish_);

    // subscriber to receive actuation input
    auto subs_options_ = rclcpp::SubscriptionOptions();
    subs_options_.callback_group = m_callback_group_sub;
    m_subscription_target = this->create_subscription<interfaces::msg::Jointspace>(
        "JointsActuation_target", rclcpp::QoS(10), std::bind(&CatheterSim::update_current_tip, this, _1), subs_options_);
  }

  // step one time step
  void simulationStep()
  {
    // lines bellow are for wight release
    // if (counter == 0)
    // {
    //   const double probeMass = 340.00E-6;                 // 339.00E-6;					// mass of the ultrasound probe at the distal end of the catheter 0.339 gram
    //   const double g = -9.81;                             // acceleration og gravity [m/s/s]
    //   // release weight from the catheter's distal end
    //   blaze::StaticVector<double, 3UL> distalForce = {probeMass * g, 0.00, 0.00};
    //   robot->setExternalDistalForce(distalForce);
    // }

    bool convergence;
    robot_->setTendonActuation(q);
    // solve the static problem
    convergence = robot_->solveBVP<mathOp::cosseratEquations::DYNAMIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
    if (!convergence)
    {
      initGuess_ = 0.00;
      convergence = robot_->solveBVP<mathOp::cosseratEquations::DYNAMIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
    }

    // record tip position
    distalPositions_ = robot_->getTipPosition();
    states_ = std::get<0>(robot_->getTrainingData());
    // actuationInputs = std::get<1>(robot->getTrainingData());
    // advances to the next discretized time instance
    robot_->stepTime();
  }

  void lqr_loop()
  {
    auto message = interfaces::msg::Taskspace();
    // get current time
    rclcpp::Time now = this->get_clock()->now();
    double t = static_cast<double>(now.nanoseconds()) / 1E9;
    double sim_time = t - m_t_init;

    // publish tip states
    message.p[0] = distalPositions_[0]; // position_x
    message.p[1] = distalPositions_[1]; // position_y
    message.p[2] = distalPositions_[2]; // position_z
    message.h[0] = 0.0;                 // orientation quat
    message.h[1] = 0.0;                 // orientation quat
    message.h[2] = 0.0;                 // orientation quat
    message.h[3] = 0.0;                 // orientation quat
    message.q[0] = states_[13];         // velocity_x
    message.q[1] = states_[14];         // velocity_y
    message.q[2] = states_[15];         // velocity_z
    message.w[0] = states_[16];         // angular velocity
    message.w[1] = states_[17];         // angular velocity
    message.w[2] = states_[18];         // angular velocity
    m_publisher_control->publish(message);
    print_tip(sim_time, distalPositions_, q);
  }

  // updates tendon actuation from controller/tester node
  void update_current_tip(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    // only two tendons are actuated in the model, could be adjusted accordingly
    q[0] = msg->tension[0];
    q[1] = msg->tension[2];
    // q[2] = msg->tension[2];
    // q[3] = msg->tension[3];

    // temp
    CatheterSim::simulationStep();
    CatheterSim::lqr_loop();

  }

  // Member variables
  static const size_t backboneDiscretizedPoints_ = 32UL;                                                                    // defining the spatial resolution
  static const size_t numberOfTendons_ = 2UL;                                                                               // number of tendons present in the sheath
  std::unique_ptr<TendonDriven<backboneDiscretizedPoints_, numberOfTendons_>> robot_;                                       // Robot object
  blaze::StaticVector<double, 6UL> initGuess_;                                                                              // initial guess for the BVP problem
  blaze::StaticVector<double, 3UL> distalPositions_ = blaze::StaticVector<double, 3UL>(0.0);                                // distal position
  blaze::StaticVector<double, 19UL + numberOfTendons_> states_ = blaze::StaticVector<double, 19UL + numberOfTendons_>(0.0); // full states_
  blaze::StaticVector<double, numberOfTendons_> q = blaze::StaticVector<double, numberOfTendons_>(0.0);                     // tendons pull
  rclcpp::TimerBase::SharedPtr m_sim_timer;                                                                                  // timer object to step simulation
  rclcpp::TimerBase::SharedPtr m_publish_timer;                                                                              // timer object to states publisher
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sim;                                                                     // callback grounp for running simulation callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub1;                                                                 // callback grounp for running publisher callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub;                                                                     // callback grounp for running subscriber callback function on separate thread
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_control;                                                      // publisher object
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target;                                               // subscriber object
  size_t count_;
  double m_t_init = 0.0;
  double m_sample_time, dt_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CatheterSim>());
  rclcpp::shutdown();
  return 0;
}

void print_tip(double t, blaze::StaticVector<double, 3UL> position, blaze::StaticVector<double, 2UL> q)
{
  auto printWithSpaceIfPositive = [](double value)
  {
    if (value >= 0)
    {
      std::cout << " " << value;
    }
    else
    {
      std::cout << value;
    }
  };

  std::cout << "\r" << std::dec << std::fixed << std::setprecision(4) << "t:" << t << " [s] |"
            << "  ";
  std::cout << "X:";
  std::cout << std::fixed << std::setprecision(4);
  printWithSpaceIfPositive(position[0]);
  std::cout << "  ";
  std::cout << "Y:";
  std::cout << std::fixed << std::setprecision(4);
  printWithSpaceIfPositive(position[1]);
  std::cout << "  ";
  std::cout << "Z:";
  std::cout << std::fixed << std::setprecision(4);
  printWithSpaceIfPositive(position[2]);
  std::cout << " [m]";
  std::cout << " | ";
  std::cout << "q0:";
  std::cout << std::fixed << std::setprecision(2);
  printWithSpaceIfPositive(q[0]);
  std::cout << "  ";
  std::cout << "q1:";
  std::cout << std::fixed << std::setprecision(2);
  printWithSpaceIfPositive(q[1]);
  std::cout << " [N]";
  std::cout << std::endl;
}

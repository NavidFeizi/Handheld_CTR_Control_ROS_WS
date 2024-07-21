
#include "predictor.hpp"
#include <chrono>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <boost/tokenizer.hpp>
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
    // Set default sample_time and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("sample_time", 1E-3);
    m_sample_time = this->get_parameter("sample_time").as_double();

    setupParametersAndInitializeRobot();
    setupROSInterfaces();

    // node initialization time
    rclcpp::Time now = this->get_clock()->now();
    m_t_init = static_cast<double>(now.nanoseconds()) / 1E9;
  }

  ~CatheterSim()
  {
  }

private:
  void setupParametersAndInitializeRobot()
  {
    std::cout << "Setting up parameters and initializing the model..." << std::endl;

    /* Simulation params */
    float fc = 50;
    m_predictor = std::make_unique<Predictor>(static_cast<float>(m_sample_time), 1, fc);

    // initialize x with the first velua in the trajectory
    m_X = {-1.780, 0.000, 63.971, 0.000};
    std::cout << "Initial X: " << m_X << std::endl;
    m_Y = m_predictor->encode(m_X);
  }

  void setupROSInterfaces()
  {
    //// Publisher, Subscribers, and Timers
    auto sim_sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    // auto publish_sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    // publisher to publish model outputs
    m_publisher_control = this->create_publisher<interfaces::msg::Taskspace>("Target", 10);
    // initialize timer to call simulation step and publish states
    // Create callback groups
    m_callback_group_sim = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_pub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create wall timers with different callback groups
    m_sim_timer = this->create_wall_timer(sim_sample_time, std::bind(&CatheterSim::simulationStep, this), m_callback_group_sim);
    // m_publish_timer = this->create_wall_timer(publish_sample_time, std::bind(&CatheterSim::publish_output, this), m_callback_group_pub1);

    // subscriber to receive actuation input
    auto subs_options_ = rclcpp::SubscriptionOptions();
    subs_options_.callback_group = m_callback_group_sub;
    m_subscription_target = this->create_subscription<interfaces::msg::Jointspace>(
        "JointsActuation_target", rclcpp::QoS(10), std::bind(&CatheterSim::update_inputs, this, _1), subs_options_);
  }

  // step one time step
  void simulationStep()
  {
    // std::cout << "Checkpint 2" << std::endl;
    t0 = std::chrono::high_resolution_clock::now();
    m_predictor->update_parameters();
    m_predictor->koopman_step(m_U);

    m_Y = m_predictor->get_lifted_states();
    m_X = m_predictor->decode(m_Y);
    t2 = std::chrono::high_resolution_clock::now();

    auto elapsed_5 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t0);
    // std::cout << "Checkpint 3" << std::endl;

    std::cout << std::fixed << std::setprecision(2);
    std::cout
        << "elapsed: " << elapsed_5.count() * 1e-3 << " [ms] | "
        << "q0: " << m_U[0] << " [mm] | "
        << "x1: " << m_X[0] << "  "
        << "x2: " << m_X[1] << "  "
        << "x3: " << m_X[2] << "  "
        << "x4: " << m_X[3] << " [mm] | "
        << std::endl;

    CatheterSim::publish_output();
  }

  void publish_output()
  {
    auto msg = interfaces::msg::Taskspace();
    // get current time
    rclcpp::Time now = this->get_clock()->now();
    double t = static_cast<double>(now.nanoseconds()) / 1E9;
    double sim_time = t - m_t_init;

    blaze::StaticVector<float, statesSize> m_X_dot = (m_X - m_X_prev) / m_sample_time;
    m_X_prev = m_X;

    // publish tip states
    msg.p[0] = m_X[0] * 1e-3;     // position_x
    msg.p[1] = 0.0;               // position_y
    msg.p[2] = m_X[2] * 1e-3;     // position_z
    msg.h[0] = 0.0;               // orientation quat
    msg.h[1] = 0.0;               // orientation quat
    msg.h[2] = 0.0;               // orientation quat
    msg.h[3] = 0.0;               // orientation quat
    msg.q[0] = m_X[1] * 1e-3;     // velocity_x
    // msg.q[1] = m_X_dot[0] * 1e-3; // velocity_x
    msg.q[2] = m_X[3] * 1e-3;     // velocity_z
    msg.w[0] = 0.0;               // angular velocity
    msg.w[1] = 0.0;               // angular velocity
    msg.w[2] = 0.0;               // angular velocity
    m_publisher_control->publish(msg);
    // print_tip(sim_time, distalPositions_, q);
  }

  // updates tendon actuation from controller/tester node
  void update_inputs(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    // only two tendons are actuated in the model, could be adjusted accordingly
    m_U[0] = msg->position[0] * 1e3;
    // q[1] = msg->position[2];
    // q[2] = msg->tension[2];
    // q[3] = msg->tension[3];

    // CatheterSim::simulationStep();
    // CatheterSim::publishStates();
  }

  // Member variables
  static const size_t statesSize = 4UL;
  static const size_t liftedStatesSize = 2UL;
  static const size_t inputsSize = 1UL;

  std::unique_ptr<Predictor> m_predictor;

  blaze::StaticVector<float, statesSize> m_X, m_X_prev = blaze::StaticVector<float, statesSize>(0.0);   // states
  blaze::StaticVector<float, inputsSize> m_U = blaze::StaticVector<float, inputsSize>(0.0);             // tendons displacement
  blaze::StaticVector<float, liftedStatesSize> m_Y = blaze::StaticVector<float, liftedStatesSize>(0.0); // liftes states

  rclcpp::TimerBase::SharedPtr m_sim_timer;                                           // timer object to step simulation
  rclcpp::TimerBase::SharedPtr m_publish_timer;                                       // timer object to states publisher
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sim;                              // callback grounp for running simulation callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub1;                             // callback grounp for running publisher callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub;                              // callback grounp for running subscriber callback function on separate thread
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_control;       // publisher object
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target; // subscriber object

  size_t count_;
  double m_t_init = 0.0;
  double m_sample_time;

  std::chrono::time_point<std::chrono::high_resolution_clock> t0;
  std::chrono::time_point<std::chrono::high_resolution_clock> t1;
  std::chrono::time_point<std::chrono::high_resolution_clock> t2;
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

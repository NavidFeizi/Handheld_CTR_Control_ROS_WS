
#include <chrono>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <string>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/target.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

void print_tip(double t, blaze::StaticVector<double, 3UL> position, blaze::StaticVector<double, 2UL> q);

class Controller : public rclcpp::Node
{
public:
  Controller() : Node("ctr_control"), count_(0)
  {
    Controller::declare_parameters();
    Controller::setup_and_initialize_controller();
    Controller::setup_ros_interfaces();

    // node initialization time
    rclcpp::Time now = this->get_clock()->now();
    m_t_init = static_cast<double>(now.nanoseconds()) / 1E9;
  }

  ~Controller()
  {
  }

private:
  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    // Set default parameters and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("control_sample_time", 4E-3);
    m_control_sample_time = this->get_parameter("control_sample_time").as_double();
  }

  /**
   * @brief Setup and initialize the controller.
   */
  void setup_and_initialize_controller()
  {
    std::cout << "Setting up controller..." << std::endl;
    // TODO: Add controller initialization code here.
  }

  /**
   * @brief Setup ROS interfaces, including publishers, subscribers, and services.
   */
  void setup_ros_interfaces()
  {
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1e6));

    // Publisher to publish control signal
    m_publisher_control = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation", 10);

    // Create callback groups
    m_callback_group_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub1;
    m_subscription_base_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_base_tool", rclcpp::QoS(10), std::bind(&Controller::update_current_tip_in_phantom, this, _1), subs_options_1);

    // Subscriber to receive ModelOutput feedback
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    subs_options_2.callback_group = m_callback_group_sub2;
    m_subscription_phantom_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_phantom_tool", rclcpp::QoS(10), std::bind(&Controller::update_current_tip_in_phantom, this, _1), subs_options_2);

    // Service to receive target
    m_service_target = this->create_service<interfaces::srv::Target>(
        "set_target", std::bind(&Controller::set_target_callback, this, _1, _2));
  }

  /**
   * @brief Update current tool (tip) position in the phantom frame.
   */
  void update_current_tip_in_phantom(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool = {msg->p[0] * 1e3,
                   msg->q[0] * 1e3,
                   msg->p[2] * 1e3,
                   msg->q[2] * 1e3};
    m_flag_new_feedback = true;
  }

  /**
   * @brief Update current tool (tip) position in the base frame.
   */
  void update_current_tip_in_base(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool = {msg->p[0] * 1e3,
                   msg->q[0] * 1e3,
                   msg->p[2] * 1e3,
                   msg->q[2] * 1e3};
    m_flag_new_feedback = true;
  }

  /**
   * @brief Service callback to set the target.
   */
  void set_target_callback(const std::shared_ptr<interfaces::srv::Target::Request> request,
                           std::shared_ptr<interfaces::srv::Target::Response> response)
  {
    // Store the received target values
    m_target_position = {request->target[0], request->target[1], request->target[2]};

    // Call the control loop
    control_loop();

    response->success = true;
  }

  /**
   * @brief Main control loop to calculate the control and publish the joint space values.
   */
  void control_loop()
  {
    auto t0 = std::chrono::high_resolution_clock::now();

    // TODO: Add control loop logic here

    // Placeholder joint space values
    interfaces::msg::Jointspace msg;
    msg.position[0] = 0.0;
    msg.position[1] = 0.0;
    msg.position[2] = 0.0;
    msg.position[3] = 0.0;

    // Publish the control signal
    m_publisher_control->publish(msg);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    std::cout << std::fixed << std::setprecision(5); // Set the precision for the entire stream
    std::cout << "elapsed: " << elapsed.count() * 1e-3 << " [ms]" << std::endl;

    return;
  }

  // Member variables
  size_t count_;
  double m_t_init = 0.0;
  double m_control_sample_time;
  bool m_flag_new_feedback = true;

  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;

  blaze::StaticVector<double, 3UL> m_base_tool, m_phantom_tool, m_target_position = blaze::StaticVector<double, 3UL>(0.0);

  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;                                  // Callback group for running subscriber callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub2;                                  // Callback group for running subscriber callback function on separate thread
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_control;           // Publisher object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_base_tool;    // Subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_phantom_tool; // Subscriber object
  rclcpp::Service<interfaces::srv::Target>::SharedPtr m_service_target;                    // Service object for setting target
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
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
  printWithSpace

      // updates current tool (tip) position in phantom frame
      void
      update_current_tip_in_phantom(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool = {msg->p[0] * 1e3,
                   msg->q[0] * 1e3,
                   msg->p[2] * 1e3,
                   msg->q[2] * 1e3};
    m_flag_new_feedback = true;
  }

  // updates current tool (tip) position in ctr frame
  void update_current_tip_in_base(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool = {msg->p[0] * 1e3,
                   msg->q[0] * 1e3,
                   msg->p[2] * 1e3,
                   msg->q[2] * 1e3};
    m_flag_new_feedback = true;
  }

  // Service callback to set the target
  void set_target_callback(const std::shared_ptr<interfaces::srv::SetTarget::Request> request,
                           std::shared_ptr<interfaces::srv::SetTarget::Response> response)
  {
    // Store the received target values
    m_target_position = {request->target[0], request->target[1], request->target[2]};

    // Call the control loop
    Controller::control_loop();

    response->success = true;
  }

  void contol_loop()
  {
    auto t0 = std::chrono::high_resolution_clock::now();

    // **** write your control loop code here *** //

    msg.position[0] = 0.0;
    msg.position[1] = 0.0;
    msg.position[2] = 0.0;
    msg.position[3] = 0.0;

    m_publisher_control->publish(msg);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    std::cout << std::fixed << std::setprecision(5); // Set the precision for the entire stream
    std::cout
        << "elapsed: " << elapsed.count() * 1e-3 << " [ms] | "
        << std::endl;

    return;
  }

  // Member variables
  size_t count_;
  double m_t_init = 0.0;
  double m_control_sample_time;
  bool m_flag_new_feedback = true;

  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;

  blaze::StaticVector<double, 3UL> m_base_tool, m_phantom_tool, m_target_position = blaze::StaticVector<double, 3UL>(0.0);

  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;                                  // callback grounp for running publisher callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub2;                                  // callback grounp for running subscriber callback function on separate thread
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_control;           // publisher object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_base_tool;    // subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_phantom_tool; // subscriber object
  rclcpp::Service<interfaces::srv::SetTarget>::SharedPtr m_service_target;                 // service object for setting target

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
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

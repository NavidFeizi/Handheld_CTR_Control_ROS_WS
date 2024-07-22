#include <chrono>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <string>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/action/target.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

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
    m_t_init = static_cast<double>(now.nanoseconds()) / 1.00E9;
  }

  ~Controller()
  {
  }

private:
  // Member variables
  size_t count_;
  double m_t_init = 0.00;
  double m_control_sample_time;
  bool m_flag_new_feedback = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  blaze::StaticVector<double, 3UL> m_base_tool, m_phantom_tool, m_target_position = blaze::StaticVector<double, 3UL>(0.00);
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;                                  // Callback group for running subscriber callback function on separate thread
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub2;                                  // Callback group for running subscriber callback function on separate thread
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_control;           // Publisher object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_base_tool;    // Subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_phantom_tool; // Subscriber object
  // rclcpp::Service<interfaces::srv::Target>::SharedPtr m_service_target;                    // Service object for setting target
  rclcpp_action::Server<interfaces::action::Target>::SharedPtr m_action_server;

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    // Set default parameters and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("control_sample_time", 4.00E-3);
    m_control_sample_time = this->get_parameter("control_sample_time").as_double();
  }

  /**
   * @brief Setup and initialize the controller.
   */
  void setup_and_initialize_controller()
  {
    std::cout << "Setting up controller..." << std::endl;
    // TODO: Add controller initialization code here.

    // This is my change ==> Filipe C. Pedrosa
  }

  /**
   * @brief Setup ROS interfaces, including publishers, subscribers, and services.
   */
  void setup_ros_interfaces()
  {
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1.00E6));

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

    // Action Server Setup
    m_action_server = rclcpp_action::create_server<interfaces::action::Target>(
        this, "set_target",
        std::bind(&Controller::handle_goal, this, _1, _2),
        std::bind(&Controller::handle_cancel, this, _1),
        std::bind(&Controller::handle_accepted, this, _1));
  }

  /**
   * @brief Update current tool (tip) position in the phantom frame.
   */
  void update_current_tip_in_phantom(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool = {msg->p[0UL] * 1.00E3,
                   msg->q[0UL] * 1.00E3,
                   msg->p[2UL] * 1.00E3,
                   msg->q[2UL] * 1.00E3};
    m_flag_new_feedback = true;
  }

  /**
   * @brief Update current tool (tip) position in the base frame.
   */
  void update_current_tip_in_base(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool = {msg->p[0UL] * 1.00E3,
                   msg->q[0UL] * 1.00E3,
                   msg->p[2UL] * 1.00E3,
                   msg->q[2UL] * 1.00E3};
    m_flag_new_feedback = true;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const interfaces::action::Target::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    std::thread{std::bind(&Controller::set_target_callback, this, std::placeholders::_1), goal_handle}.detach();
  }

  /**
   * @brief Service callback to set the target.
   */
  void set_target_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    m_target_position = {goal->target_pose[0UL], goal->target_pose[1UL], goal->target_pose[2UL]};

    auto result = std::make_shared<interfaces::action::Target::Result>();

    // Call the control loop
    Controller::control_loop();

    result->success = true; // Indicate success in the result
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal processed successfully.");
  }

  /**
   * @brief Main control loop to calculate the control and publish the joint space values.
   */
  void control_loop()
  {
    const auto t0 = std::chrono::high_resolution_clock::now();

    // TODO: Add control loop logic here

    // Placeholder joint space values
    interfaces::msg::Jointspace msg;
    msg.position[0UL] = 0.00;
    msg.position[1UL] = 0.00;
    msg.position[2UL] = 0.00;
    msg.position[3UL] = 0.00;

    // Publish the control signal
    m_publisher_control->publish(msg);

    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    std::cout << std::fixed << std::setprecision(5); // Set the precision for the entire stream
    std::cout << "elapsed: " << elapsed.count() * 1.00E-3 << " [ms]" << std::endl;

    return;
  }
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

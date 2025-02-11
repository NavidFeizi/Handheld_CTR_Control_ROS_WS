#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/homing.hpp"
#include "interfaces/action/jointstarget.hpp"
#include "interfaces/srv/jointstarget.hpp" // Adjust the path to match your package and service name

#include "Robot.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
void print_robot_status(double t,
                        const blaze::StaticVector<double, 6UL> &position,
                        const blaze::StaticVector<double, 6UL> &velocity,
                        const blaze::StaticVector<double, 6UL> &current);

class RobotNode : public rclcpp::Node
{
public:
  RobotNode() : Node("ctr_robot"), m_count(0)
  {
    // declare_parameters();
    m_flag_use_target_action = true;
    setup_and_initialize_robot();
    setup_ros_interfaces();
    // setup_parameter_callback();
  }

private:
  // Declare ROS parameters
  void declare_parameters()
  {
    declare_parameter<double>("Kp", 4.60);
    declare_parameter<double>("Ki", 2.60);
    m_kp = get_parameter("Kp").as_double();
    m_ki = get_parameter("Ki").as_double();
  }

  // Setup and initialize the robot
  void setup_and_initialize_robot()
  {
    // int8_t operation_mode = static_cast<int8_t>(0xFE); // 0x01, 0x03, 0xFE
    int operation_mode = 1;
    bool position_limit = false;
    constexpr blaze::StaticVector<double, 4UL> max_acc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> max_vel = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
    int sample_time = 50;                                                                                                                    //[ms]

    m_robot = std::make_unique<CTRobot>(sample_time, operation_mode, max_acc, max_vel, position_limit);

    m_robot->Start_Thread();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (m_robot->Get_Controller_Switch_Status())
    {
      m_robot->Enable_Operation(true);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    m_robot->set_target_position({0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    m_robot->Wait_until_reach();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Setup ROS interfaces including publishers, subscribers, services, and timers
  void setup_ros_interfaces()
  {
    // Create callback groups to ensure mutually exclusive callbacks
    m_callback_group_pub1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_homing = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_watchdog_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target joint configurations
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub1;
    m_subscription_target = create_subscription<interfaces::msg::Jointspace>(
        "JointsActuation", 10, std::bind(&RobotNode::target_callback, this, _1), subs_options_1);

    // Publisher to broadcast the current joint configurations
    m_publisher_robot = create_publisher<interfaces::msg::Jointspace>("JointSpaceStatus", 10);

    // Low-level control loop timer
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1.00E6));
    m_control_loop_timer = create_wall_timer(control_sample_time, std::bind(&RobotNode::position_command_loop, this), m_callback_group_pub1);

    // Timer to read joint configurations periodically
    m_read_robot_timer = create_wall_timer(50ms, std::bind(&RobotNode::joints_config_callback, this), m_callback_group_read);

    // Initialize homing service using the custom service type
    m_homing_service = this->create_service<interfaces::srv::Homing>(
        "homing", std::bind(&RobotNode::manual_callback, this, _1, _2));

    // Initialize the watchdog timer
    m_watchdog_timer_target = this->create_wall_timer(2000ms, std::bind(&RobotNode::check_target_publisher_alive, this), m_callback_group_watchdog_2);

    m_target_service = this->create_service<interfaces::srv::Jointstarget>(
        "joints_space/target", std::bind(&RobotNode::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    // // Action Server Setup
    // m_action_server = rclcpp_action::create_server<interfaces::action::Jointstarget>(
    //     this, "joints_target",
    //     std::bind(&RobotNode::handle_goal, this, _1, _2),
    //     std::bind(&RobotNode::handle_cancel, this, _1),
    //     std::bind(&RobotNode::handle_accepted, this, _1));
  }

  // Setup ROS parameter callback function to handle dynamic parameter (Kp, Ki) updates
  void setup_parameter_callback()
  {
    auto param_callback = [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "Kp")
        {
          m_kp = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Parameter 'Kp' set to: %f", m_kp);
        }
        else if (parameter.get_name() == "Ki")
        {
          m_ki = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Parameter 'Ki' set to: %f", m_ki);
        }
        else
        {
          result.successful = false;
          result.reason = "Unknown parameter";
          RCLCPP_WARN(get_logger(), "Unknown parameter '%s'", parameter.get_name().c_str());
        }
      }

      return result;
    };

    m_param_callback_handle = add_on_set_parameters_callback(param_callback);
  }

  // Timer callback function to read the current joint configurations and publish them
  void joints_config_callback()
  {
    auto msg = interfaces::msg::Jointspace();

    m_robot->Get_PosVelCur(&m_x, &m_x_dot, &m_c);

    msg.position[0UL] = m_x[0UL];
    msg.position[1UL] = m_x[1UL];
    msg.position[2UL] = m_x[2UL];
    msg.position[3UL] = m_x[3UL];

    msg.velocity[0UL] = m_x_dot[0UL];
    msg.velocity[1UL] = m_x_dot[1UL];
    msg.velocity[2UL] = m_x_dot[2UL];
    msg.velocity[3UL] = m_x_dot[3UL];

    m_publisher_robot->publish(msg);
  }

  // Timer callback function for joint space control loop
  void joint_space_control_callback()
  {
    blaze::StaticVector<double, 4L> q_dot_command, x_des, x_dot_des;

    if (!m_flag_manual)
    {
      x_des = m_x_des;
      x_dot_des = m_x_dot_des;
      joint_space_control_step(x_des, x_dot_des, q_dot_command);
      // m_robot->Set_Target_Velocity(q_dot_command);
    }
  }

  // Perform a control step in joint space
  void joint_space_control_step(const blaze::StaticVector<double, 4UL> x_des, const blaze::StaticVector<double, 4UL> x_dot_des, blaze::StaticVector<double, 4UL> &q_dot_command)
  {
    m_x_error = x_des - m_x;
    m_x_error_int = m_x_error_int + m_x_error * m_control_sample_time;
    q_dot_command = m_x_error * m_kp + m_x_error_int * m_ki + x_dot_des;
  }

  // Set the target position in the robot - Depreciated
  void position_command_loop()
  {
    m_robot->set_target_position(m_x_des);
  }

  // Perform system identification - Depreciated
  void system_identification_loop()
  {
    m_robot->Set_Target_Velocity(m_x_dot_des);
    std::cout << "\r" << std::dec << std::fixed << std::setprecision(3) << "q2_dot:" << m_x_dot_des[2UL] << std::endl;
  }

  // Subscription callback function to updates the target joint positions and velocities
  void target_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    if (!m_flag_manual && !m_flag_use_target_action)
    {
      m_targpublisher_alive_tmep = true;
      m_x_des = blaze::StaticVector<double, 4UL>(0.00);

      m_x_des[0UL] = msg->position[0UL];
      m_x_des[1UL] = msg->position[1UL];
      m_x_des[2UL] = msg->position[2UL];
      m_x_des[3UL] = msg->position[3UL];
    }

    // RCLCPP_INFO(this->get_logger(), "New Target");
  }

  //
  void handle_service_request(
      const std::shared_ptr<interfaces::srv::Jointstarget::Request> request,
      std::shared_ptr<interfaces::srv::Jointstarget::Response> response)
  {
    if (!m_flag_manual && m_flag_use_target_action)
    {
      m_x_des[0UL] = request->position[0UL];
      m_x_des[1UL] = request->position[1UL];
      m_x_des[2UL] = request->position[2UL];
      m_x_des[3UL] = request->position[3UL];
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Target is not accepted in manual mode");
    }

    RCLCPP_INFO(this->get_logger(), "Joints target received: R1: %0.1f [deg]  T1: %0.1f [mm]  R2: %0.1f [deg]  T2: %0.1f [mm]",
                m_x_des[0UL] * 360 / M_PI, m_x_des[1UL] * 1e3, m_x_des[2UL] * 360 / M_PI, m_x_des[3UL] * 1e3);

    RCLCPP_INFO(this->get_logger(), "Waiting");
    rclcpp::sleep_for(200ms);
    m_robot->Wait_until_reach();
    RCLCPP_INFO(this->get_logger(), "Joints target reached");
    response->success = true;
    response->message = "Joints target reached!";
    
  }

  // //
  // rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const interfaces::action::Jointstarget::Goal> goal)
  // {
  //   // RCLCPP_INFO(this->get_logger(), "Target request received ");
  //   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  // }

  // //
  // rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Jointstarget>> goal_handle)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  //   return rclcpp_action::CancelResponse::ACCEPT;
  // }

  // //
  // void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Jointstarget>> goal_handle)
  // {
  //   std::thread{std::bind(&RobotNode::set_action_target_callback, this, std::placeholders::_1), goal_handle}.detach();
  // }

  // //
  // void set_action_target_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Jointstarget>> &goal_handle)
  // {
  //   const auto goal = goal_handle->get_goal();

  //   if (!m_flag_manual && m_flag_use_target_action)
  //   {
  //     m_x_des[0UL] = goal->position[0UL];
  //     m_x_des[1UL] = goal->position[1UL];
  //     m_x_des[2UL] = goal->position[2UL];
  //     m_x_des[3UL] = goal->position[3UL];
  //   }
  //   else
  //   {
  //     RCLCPP_INFO(this->get_logger(), "Target is not accepten in manual mode");
  //   }

  //   RCLCPP_INFO(this->get_logger(), "Joints target received: R1: %0.1f [deg]  T1: %0.1f [mm]  R2: %0.1f [deg]  T2: %0.1f [mm]",
  //               m_x_des[0UL] * 360 / M_PI, m_x_des[1UL] * 1e3, m_x_des[2UL] * 360 / M_PI, m_x_des[3UL] * 1e3);

  //   auto result = std::make_shared<interfaces::action::Jointstarget::Result>();

  //   // Call the control loop
  //   rclcpp::sleep_for(200ms);
  //   m_robot->Wait_until_reach();
  //   // rclcpp::sleep_for(2000ms);

  //   result->success = true; // Indicate success in the result
  //   goal_handle->succeed(result);
  //   RCLCPP_INFO(this->get_logger(), "Target reached successfully");
  // }

  // Subscription callback function updates the current catheter tip status using EMTracker topic
  void current_tool_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_catheter = {msg->p[1UL] * 1.00E3, -msg->p[0UL] * 1.00E3, msg->p[2UL] * 1.00E3};
    m_emtracker_alive_tmep = true;
  }

  // Service callback to perform the homing procedure
  void manual_callback(const std::shared_ptr<interfaces::srv::Homing::Request> request, std::shared_ptr<interfaces::srv::Homing::Response> response)
  {
    if (request->command == "ON")
    {
      m_flag_manual = true;
      response->position[0UL] = m_x[0UL];
      response->position[1UL] = m_x[1UL];
      response->position[2UL] = m_x[2UL];
      response->position[3UL] = m_x[3UL];
      response->success = true;
      response->message = "Manual mode activated";
      RCLCPP_INFO(this->get_logger(), "Manual mode activated");
    }
    else if (request->command == "OFF")
    {
      m_flag_manual = false;
      response->success = true;
      response->message = "Manual mode OFF";
      RCLCPP_INFO(this->get_logger(), "Manual mode deactivated");
    }
    else if (request->command == "move")
    {
      if (m_flag_manual)
      {
        m_x_des[0UL] = request->target[0UL];
        m_x_des[1UL] = request->target[1UL];
        m_x_des[2UL] = request->target[2UL];
        m_x_des[3UL] = request->target[3UL];
        response->success = true;
        response->message = "Moving";
      }
      else
      {
        response->success = false;
        response->message = "Manual mode is not deactive";
      }
    }
    else if (request->command == "set_home")
    {
      if (m_flag_manual)
      {
        m_robot->Set_Zero_Position(blaze::StaticVector<double, 4UL>(0.00));
        m_x_des = blaze::StaticVector<double, 4UL>(0.00);
        response->success = true;
        response->message = "Home is set";
        response->position[0UL] = m_x[0UL];
        response->position[1UL] = m_x[1UL];
        response->position[2UL] = m_x[2UL];
        response->position[3UL] = m_x[3UL];
      }
      else
      {
        response->success = false;
        response->message = "Manual mode is not deactive";
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid command");
      response->success = false;
      response->message = "Invalid command";
    }
  }

  // Timer callback function to check if the EMTracker is still alive
  void check_target_publisher_alive()
  {
    // Set the flag to false; it will be set to true if current_catheter_tip_callback is called
    if (m_targpublisher_alive_tmep)
    {
      if (!m_targpublisher_alive)
      {
        m_targpublisher_alive = true;
        RCLCPP_WARN(get_logger(), "publisher_node is alive");
      }
    }
    else
    {
      if (m_targpublisher_alive)
      {
        m_targpublisher_alive = false;
        // m_x_des = blaze::StaticVector<double, 4UL>(0.00);
        // m_x_dot_des = blaze::StaticVector<double, 4UL>(0.00);
        RCLCPP_WARN(get_logger(), "publisher_node is dead");
      }
    }
    m_targpublisher_alive_tmep = false;
  }

  // Wait in milliseconds
  void wait(int milliseconds)
  {
    rclcpp::Rate rate(1000.00 / milliseconds);
    rate.sleep();
  }

  size_t m_count;
  // const double m_control_sample_time = 0.0025; //[s]
  const double m_control_sample_time = 0.050; //[s]
  double m_kp, m_ki = 0.00;
  bool m_flag_manual, m_flag_use_target_action = false;
  bool m_emtracker_alive, m_emtracker_alive_tmep, m_targpublisher_alive, m_targpublisher_alive_tmep = false;
  std::unique_ptr<CTRobot> m_robot;

  rclcpp::TimerBase::SharedPtr m_watchdog_timer_target;
  rclcpp::TimerBase::SharedPtr m_read_robot_timer;
  rclcpp::TimerBase::SharedPtr m_control_loop_timer;
  rclcpp::TimerBase::SharedPtr m_position_control_timer;
  rclcpp::TimerBase::SharedPtr m_system_identification_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_robot;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target;
  rclcpp::Service<interfaces::srv::Homing>::SharedPtr m_homing_service;
  // rclcpp_action::Server<interfaces::action::Jointstarget>::SharedPtr m_action_server;
  rclcpp::Service<interfaces::srv::Jointstarget>::SharedPtr m_target_service;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_homing;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_2;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  blaze::StaticVector<double, 4UL> m_x_abs;
  blaze::StaticVector<double, 4UL> m_x;
  blaze::StaticVector<double, 4UL> m_x_dot;
  blaze::StaticVector<int, 4UL> m_c;
  blaze::StaticVector<double, 4UL> m_x_des; // in SI units
  blaze::StaticVector<double, 4UL> m_x_error;
  blaze::StaticVector<double, 4UL> m_x_error_int;
  blaze::StaticVector<double, 4UL> m_x_error_dot;
  blaze::StaticVector<double, 4UL> m_x_dot_forward;
  blaze::StaticVector<double, 4UL> m_x_dot_des;
  blaze::StaticVector<double, 4UL> m_x_des_home;
  blaze::StaticVector<double, 3UL> m_x_catheter;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

void print_robot_status(double t, const blaze::StaticVector<double, 6UL> &position, const blaze::StaticVector<double, 6UL> &velocity, const blaze::StaticVector<double, 6UL> &current)
{
  auto print_with_space_if_positive = [](double value) -> void
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

  std::cout << "\r" << std::dec << std::fixed << std::setprecision(3) << "Robot => Time: " << t << "[s]" << std::endl;
  std::cout << "Units: [mm], [rad], [mm/s], [rad/s], [A]" << std::endl;
  for (size_t i = 0; i < 6; i++)
  {
    std::cout << "Node " << i + 1 << " =>  ";
    std::cout << "Pos: ";
    std::cout << std::fixed << std::setprecision(5);
    print_with_space_if_positive(position[i] * 1.00E3);
    std::cout << "     Vel: ";
    print_with_space_if_positive(velocity[i] * 1.00E3);
    std::cout << "     Current: ";
    print_with_space_if_positive(current[i]);
    std::cout << " \n";
  }
  std::cout << std::endl;
}
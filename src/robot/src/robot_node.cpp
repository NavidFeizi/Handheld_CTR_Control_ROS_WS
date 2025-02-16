#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "interfaces/msg/status.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/config.hpp"
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
    OpMode operation_mode = OpMode::VelocityProfile;
    bool position_limit = false;
    int sample_time = 50; //[ms]

    m_robot = std::make_unique<CTRobot>(sample_time, operation_mode, position_limit);
    m_robot->startFiber();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (m_robot->getSwitchStatus())
    {
      // m_robot->Enable_Operation(true);
    }
    // m_robot->Set_Target_Position({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // if (m_robot->Get_Controller_Switch_Status())
    // {
    //   m_robot->Enable_Operation(true);
    // }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // m_robot->set_target_position({0.0, 0.0, 0.0, 0.0});
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // m_robot->Wait_until_reach();
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Setup ROS interfaces including publishers, subscribers, services, and timers
  void setup_ros_interfaces()
  {
    // Create callback groups to ensure mutually exclusive callbacks
    m_callback_group_pub1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_homing = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_read_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_watchdog_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target joint configurations
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub1;
    m_subscription_target = create_subscription<interfaces::msg::Jointspace>("joints_space/target", 10, std::bind(&RobotNode::target_callback, this, _1), subs_options_1);

    // Publisher to broadcast the robot status
    m_publisher_status = create_publisher<interfaces::msg::Status>("robot_status", 10);
    // Publisher to broadcast the current joint configurations
    m_publisher_joints = create_publisher<interfaces::msg::Jointspace>("/joint_space/feedback", 10);
    // configing service 
    m_homing_service = this->create_service<interfaces::srv::Config>("config", std::bind(&RobotNode::manual_callback, this, _1, _2));
    //
    m_target_service = this->create_service<interfaces::srv::Jointstarget>("joints_space/target", std::bind(&RobotNode::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));

    // Low-level control loop timer
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1.00E6));
    m_control_loop_timer = create_wall_timer(control_sample_time, std::bind(&RobotNode::velocity_command_loop, this), m_callback_group_pub1);
    // Timer to read joint configurations periodically
    m_joints_config_timer = create_wall_timer(50ms, std::bind(&RobotNode::joints_config_callback, this), m_callback_group_read);
    // Timer to read robot status periodically
    m_read_robot_timer = create_wall_timer(100ms, std::bind(&RobotNode::robot_status_callback, this), m_callback_group_read_2);
    // Initialize the watchdog timer
    m_watchdog_timer_target = this->create_wall_timer(2000ms, std::bind(&RobotNode::check_target_publisher_alive, this), m_callback_group_watchdog_2);

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
  void robot_status_callback()
  {
    auto msg = interfaces::msg::Status();
    blaze::StaticVector<bool, 4UL> status;
    m_robot->getEnableStatus(status);
    msg.enable[0] = status[0];
    msg.enable[1] = status[1];
    msg.enable[2] = status[2];
    msg.enable[3] = status[3];
    m_robot->getEncoderStatus(status);
    msg.encoder[0] = status[0];
    msg.encoder[1] = status[1];
    msg.encoder[2] = status[2];
    msg.encoder[3] = status[3];
    m_robot->getReachedStatus(status);
    msg.reached[0] = status[0];
    msg.reached[1] = status[1];
    msg.reached[2] = status[2];
    msg.reached[3] = status[3];
    if (abs((m_x[1] - m_pos_prewrench[1]) < 0.002) && (abs(m_x[3] - m_pos_prewrench[3]) < 0.002))
      m_ready_to_proceed = true;
    else
      m_ready_to_proceed = false;
    if (abs((m_x[1] - m_pos_wrench[1]) < 0.002) && (abs(m_x[3] - m_pos_wrench[3]) < 0.002))
      m_ready_to_unlock = true;
    else
      m_ready_to_unlock = false;
    msg.proceed_ready = m_ready_to_proceed;
    msg.unlock_ready = m_ready_to_unlock;
    msg.unlocked = m_unlocked;
    msg.locked = m_locked;
    m_publisher_status->publish(msg);
  }

  // Timer callback function to read the current joint configurations and publish them
  void joints_config_callback()
  {
    auto msg = interfaces::msg::Jointspace();
    m_robot->getPosVelCur(m_x, m_x_dot, m_c);
    msg.position[0UL] = m_x[0UL];
    msg.position[1UL] = m_x[1UL];
    msg.position[2UL] = m_x[2UL];
    msg.position[3UL] = m_x[3UL];
    msg.velocity[0UL] = m_x_dot[0UL];
    msg.velocity[1UL] = m_x_dot[1UL];
    msg.velocity[2UL] = m_x_dot[2UL];
    msg.velocity[3UL] = m_x_dot[3UL];
    msg.current[0UL] = m_c[0UL];
    msg.current[1UL] = m_c[1UL];
    msg.current[2UL] = m_c[2UL];
    msg.current[3UL] = m_c[3UL];
    m_publisher_joints->publish(msg);
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
    m_robot->setTargetPos(m_x_des);
  }

  // Set the target position in the robot - Depreciated
  void velocity_command_loop()
  {
    // m_robot->set_target_velocity(m_x_dot_des);
  }

  // // Perform system identification - Depreciated
  // void system_identification_loop()
  // {
  //   m_robot->set_target_velocity(m_x_dot_des);
  //   std::cout << "\r" << std::dec << std::fixed << std::setprecision(3) << "q2_dot:" << m_x_dot_des[2UL] << std::endl;
  // }

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
    m_robot->waitUntilReach();
    RCLCPP_INFO(this->get_logger(), "Joints target reached");
    response->success = true;
    response->message = "Joints target reached!";
  }

  // Subscription callback function updates the current catheter tip status using EMTracker topic
  void current_tool_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_catheter = {msg->p[1UL] * 1.00E3, -msg->p[0UL] * 1.00E3, msg->p[2UL] * 1.00E3};
    m_emtracker_alive_tmep = true;
  }

  // Service callback to perform the homing procedure
  void manual_callback(const std::shared_ptr<interfaces::srv::Config::Request> request, std::shared_ptr<interfaces::srv::Config::Response> response)
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

    else if (request->command == "enable")
    {
      m_robot->enableOperation(true);
      // m_robot->Set_Zero_Position(blaze::StaticVector<double, 4UL>(0.00));
      response->success = true;
      response->message = "enabled";
      RCLCPP_INFO(this->get_logger(), "enabled");
    }

    else if (request->command == "disable")
    {
      m_robot->enableOperation(false);
      m_flag_manual = false;
      response->success = true;
      response->message = "disabled";
      RCLCPP_INFO(this->get_logger(), "disabled");
    }

    else if (request->command == "find")
    {
      RCLCPP_INFO(this->get_logger(), "finding encoders");
      m_robot->findTransEncoders();

      response->success = true;
      response->message = "encoders found";
      RCLCPP_INFO(this->get_logger(), "encoders found");
    }

    else if (request->command == "prepare_position_wrench")
    {
      RCLCPP_INFO(this->get_logger(), "preparing to position_wrench");

      blaze::StaticVector<bool, 4UL> status;
      m_robot->getEncoderStatus(status); // updates encoders set flag
      if (true)                          // (status[1] * status[3])                                           // check if linear encoders are set
      {
        blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
        blaze::StaticVector<double, 4UL> max_vel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00};   // [deg/s] and [mm/s]
        blaze::StaticVector<double, 4UL> negative = {200.0, 400.0, 200.0, 400.0};
        blaze::StaticVector<double, 4UL> positive = {200.0, 400.0, 200.0, 400.0};

        m_robot->setOperationMode(OpMode::PositionProfile);
        m_robot->setProfileParams(max_vel, max_dcc, max_dcc);
        m_robot->setMaxTorque(negative, positive);
        m_robot->enableOperation(true);

        // m_robot->set_target_position(m_pos_prewrench);
        m_robot->setTargetPos({m_x[0], m_pos_prewrench[1], m_x[2], m_pos_prewrench[3]});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        response->success = true;
        response->message = "ready to position wrench";
      }
      else
      {
        response->success = false;
        response->message = "encoder must be found before positioning wrench";
      }
    }

    else if (request->command == "position_wrench")
    {
      blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
      blaze::StaticVector<double, 4UL> max_vel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00};   // [deg/s] and [mm/s]
      blaze::StaticVector<double, 4UL> negative = {400.0, 250.0, 400.0, 250.0};
      blaze::StaticVector<double, 4UL> positive = {400.0, 250.0, 400.0, 250.0};
      blaze::StaticVector<double, 4UL> temp;

      RCLCPP_INFO(this->get_logger(), "positioning wrench");
      blaze::StaticVector<bool, 4UL> reach;
      blaze::StaticVector<bool, 4UL> status;
      m_robot->getEncoderStatus(status); // updates encoders set flag
      if (true)                          // (status[1] * status[3])                                           // check if linear encoders are set
      {
        m_robot->setOperationMode(OpMode::PositionProfile);
        m_robot->setProfileParams(max_vel, max_dcc, max_dcc);
        m_robot->setMaxTorque(negative, positive);
        m_robot->enableOperation(true);

        m_robot->setTargetPos({m_x[0], m_pos_wrench[1] + 0.006, m_x[2], m_pos_wrench[3] - 0.004});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_robot->getReachedStatus(reach);
        while (!(reach[1] && reach[3]))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          m_robot->getReachedStatus(reach);
        }
        RCLCPP_INFO(this->get_logger(), "[Master] wrench position step one done");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        temp[0] = m_x[0] - 5.0 * M_PI;
        temp[1] = m_pos_wrench[1];
        temp[2] = m_x[2] + 5.0 * M_PI;
        temp[3] = m_pos_wrench[3];
        int counter = 1;

        m_robot->setTargetPos(temp);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        m_robot->getReachedStatus(reach);
        while (!(reach[1] && reach[3]))
        {
          if (counter == 1)
          {
            temp[0] = m_x[0] - 5.0 * M_PI;
            temp[2] = m_x[2] + 5.0 * M_PI;
            m_robot->setTargetPos(temp);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          }
          else
          {
            temp[0] = m_x[0] + 5.0 * M_PI;
            temp[2] = m_x[2] - 5.0 * M_PI;
            m_robot->setTargetPos(temp);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
          }
          counter = counter * -1;

          m_robot->getReachedStatus(reach);
        }
        m_robot->setTargetPos(m_x);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "[Master] wrench position step two done");

        temp = m_x;
        temp[0] = temp[0] - 5.0 * M_PI;
        temp[2] = temp[2] + 5.0 * M_PI;
        m_robot->setTargetPos(temp);
        while (!(m_c[0] < -390 && m_c[2] > 390))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        m_robot->setTargetPos(m_x);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "[Master] wrench position step three done");

        response->success = true;
        response->message = "wrench positioned";
      }
      else
      {
        response->success = false;
        response->message = "wrench positioning error";
      }
    }

    else if (request->command == "unlock")
    {
      RCLCPP_INFO(this->get_logger(), "unlocking");
      blaze::StaticVector<bool, 4UL> reach;
      if (true) // (status[1] * status[3])                                           // check if linear encoders are set
      {
        blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
        blaze::StaticVector<double, 4UL> max_vel = {100.00 * M_PI / 20.00, 0.50 / 1000.00, 100.00 * M_PI / 20.00, 0.50 / 1000.00};     // [deg/s] and [mm/s]
        blaze::StaticVector<double, 4UL> negative = {1000.0, 220.0, 1000.0, 220.0};
        blaze::StaticVector<double, 4UL> positive = {1000.0, 220.0, 1000.0, 220.0};

        m_robot->setOperationMode(OpMode::PositionProfile);
        m_robot->setProfileParams(max_vel, max_dcc, max_dcc);
        m_robot->setMaxTorque(negative, positive);
        m_robot->enableOperation(true);

        // m_robot->set_target_position(m_pos_prewrench);
        m_robot->setTargetPos({m_x[0] + 4 * M_PI, m_x[1] + 0.0015, m_x[2] - 4 * M_PI, m_x[3] - 0.0015}); // *********** the parameters must be adjusted based on screw lead *********
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        m_robot->getReachedStatus(reach);
        while (!(reach[0] && reach[1] && reach[2] && reach[3]))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          m_robot->getReachedStatus(reach);
        }
        m_unlocked = true;
        response->success = true;
        response->message = "unlocked";
      }
      else
      {
        response->success = false;
        response->message = "unlocking error";
      }
    }

    else if (request->command == "lock")
    {
      RCLCPP_INFO(this->get_logger(), "locking");
      blaze::StaticVector<bool, 4UL> reach;
      if (true) // (status[1] * status[3])                                           // check if linear encoders are set
      {
        blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
        blaze::StaticVector<double, 4UL> max_vel = {100.00 * M_PI / 20.00, 0.50 / 1000.00, 100.00 * M_PI / 20.00, 0.50 / 1000.00};     // [deg/s] and [mm/s]
        blaze::StaticVector<double, 4UL> negative = {400.0, 300.0, 400.0, 300.0};
        blaze::StaticVector<double, 4UL> positive = {400.0, 300.0, 400.0, 300.0};

        m_robot->setOperationMode(OpMode::PositionProfile);
        m_robot->setProfileParams(max_vel, max_dcc, max_dcc);
        m_robot->setMaxTorque(negative, positive);
        m_robot->enableOperation(true);

        // m_robot->set_target_position(m_pos_prewrench);
        m_robot->setTargetPos({m_x[0] - 10 * M_PI, m_x[1] - 0.0015, m_x[2] + 10 * M_PI, m_x[3] + 0.0015}); // *********** the parameters must be adjusted based on screw lead *********
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        m_robot->getReachedStatus(reach);
        while (!(reach[1] && reach[3] && m_c[0] < -390 && m_c[2] > 390))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          m_robot->getReachedStatus(reach);
        }
        m_robot->setTargetPos(m_x);

        m_unlocked = false;
        m_locked = true;
        response->success = true;
        response->message = "locked";
      }
      else
      {
        response->success = false;
        response->message = "locking error";
      }
    }

    else if (request->command == "move")
    {
      if (m_flag_manual)
      {
        m_x_dot_des[0UL] = request->target[0UL];
        m_x_dot_des[1UL] = request->target[1UL];
        m_x_dot_des[2UL] = request->target[2UL];
        m_x_dot_des[3UL] = request->target[3UL];
        response->success = true;
        if (m_x_dot_des[0] == 0 && m_x_dot_des[1] == 0 && m_x_dot_des[2] == 0 && m_x_dot_des[3] == 0)
        {
          response->message = "Stopped";
        }
        else
        {
          response->message = "Moving";
        }
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
        // m_robot->Set_Zero_Position(blaze::StaticVector<double, 4UL>(0.00));
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
      RCLCPP_WARN(this->get_logger(), "Invalid command: ", request->command);
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
  bool m_ready_to_proceed, m_ready_to_unlock, m_unlocked, m_locked = false;
  std::unique_ptr<CTRobot> m_robot;

  const blaze::StaticVector<double, 4> m_pos_prewrench = {0.0, 0.085, 0.0, 0.128};
  const blaze::StaticVector<double, 4> m_pos_wrench = {0.0, 0.0750, 0.0, 0.1335};

  rclcpp::TimerBase::SharedPtr m_watchdog_timer_target;
  rclcpp::TimerBase::SharedPtr m_read_robot_timer;
  rclcpp::TimerBase::SharedPtr m_joints_config_timer;
  rclcpp::TimerBase::SharedPtr m_control_loop_timer;
  rclcpp::TimerBase::SharedPtr m_position_control_timer;
  rclcpp::TimerBase::SharedPtr m_system_identification_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_joints;
  rclcpp::Publisher<interfaces::msg::Status>::SharedPtr m_publisher_status;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target;
  rclcpp::Service<interfaces::srv::Config>::SharedPtr m_homing_service;
  // rclcpp_action::Server<interfaces::action::Jointstarget>::SharedPtr m_action_server;
  rclcpp::Service<interfaces::srv::Jointstarget>::SharedPtr m_target_service;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_homing;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read_2;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_2;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  blaze::StaticVector<double, 4UL> m_x_abs;
  blaze::StaticVector<double, 4UL> m_x;
  blaze::StaticVector<double, 4UL> m_x_dot;
  blaze::StaticVector<double, 4UL> m_c;
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
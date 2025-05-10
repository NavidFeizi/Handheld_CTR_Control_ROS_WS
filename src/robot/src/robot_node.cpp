#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "interfaces/msg/status.hpp"
#include "interfaces/msg/interface.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/srv/jointstarget.hpp"

#include "Robot.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// Declare your enum class
enum class CtrlMode : int
{
  Config = 0x00,
  Manual = 0x01,
  Position = 0x02,
  Velocity = 0x03,
};

class RobotNode : public rclcpp::Node, public CTRobot
{

public:
  RobotNode() : rclcpp::Node("ctr_robot")
  {
    m_flag_use_target_action = false;
    m_trans_limit = true;
    m_encoders_set = {1, 1, 1, 1}; /// temoporarly for development
    // declare_parameters();
    initRosInterfaces();
    startRobotCommunication(s_sample_time);

    worker_thread_ = std::thread([this]()
                                 {
      while (true)
      {
        std::unique_lock<std::mutex> lock(m_task_mutex);
        m_task_cv.wait(lock, [this]() -> bool { return this->m_flag_task_ready.load(); });
    
        m_cancel_flag = false;
        m_flag_task_ready = false;  // Reset immediately
    
        auto task = m_current_task;
        lock.unlock();  // Let the task run without holding the mutex
    
        if (task) task();
      } });
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

  // Setup ROS interfaces including publishers, subscribers, services, and timers
  void initRosInterfaces()
  {
    // Create callback groups to ensure mutually exclusive callbacks
    m_cbGroup1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup4 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup5 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup6 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup7 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup8 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_callback_group_watchdog_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target joint configurations
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    auto subs_options_3 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_cbGroup2;
    subs_options_2.callback_group = m_cbGroup7;
    subs_options_3.callback_group = m_cbGroup8;

    m_subscription_target = create_subscription<interfaces::msg::Jointspace>("joint_space/target", 10, std::bind(&RobotNode::jointSpaceTarget_callback, this, _1), subs_options_1);
    // subscriber to read vecloity command for manual joint space control mode
    m_subscription_manual_vel = create_subscription<interfaces::msg::Jointspace>("joint_space/manual_vel", 20, std::bind(&RobotNode::jointSpaceManualVel_callback, this, _1), subs_options_2);
    // subscriber to em tracker tool-in-robot reading
    m_subscription_tool = create_subscription<interfaces::msg::Taskspace>("task_space/feedback/base_tool", 20, std::bind(&RobotNode::current_tool_callback, this, _1), subs_options_3);

    // Publisher to broadcast the robot status
    m_publisher_status = create_publisher<interfaces::msg::Status>("robot_status", 10);
    // Publisher to broadcast the robot interface status
    m_publisher_interface = create_publisher<interfaces::msg::Interface>("manual_interface", 10);
    // Publisher to broadcast the current joint configurations
    m_publisher_joints = create_publisher<interfaces::msg::Jointspace>("joint_space/feedback", 10);
    // configing service
    m_config_service = create_service<interfaces::srv::Config>("robot_config", std::bind(&RobotNode::configService_callback, this, _1, _2));
    // enable/disbale service
    m_enable_service = create_service<interfaces::srv::Config>("robot_enable", std::bind(&RobotNode::enableService_callback, this, _1, _2));

    // Low-level control loop timer
    auto command_sample_time = std::chrono::milliseconds(static_cast<int>(50));
    m_control_loop_timer = create_wall_timer(command_sample_time, std::bind(&RobotNode::targetCommand_timerCallback, this), m_cbGroup1);
    // Timer to read joint configurations periodically
    m_joints_config_timer = create_wall_timer(10ms, std::bind(&RobotNode::jointsConfig_timerCallback, this), m_cbGroup3);
    // Timer to read robot status periodically
    m_read_robot_timer = create_wall_timer(10ms, std::bind(&RobotNode::robotStatus_timerCallback, this), m_cbGroup3);
    // Initialize a timer to check emtracker lifecycle
    m_watchdog_timer_emt = create_wall_timer(100ms, std::bind(&RobotNode::check_emtracker_alive_timerCallback, this), m_callback_group_watchdog_1);

    // Initialize the watchdog timer
    // m_watchdog_timer_target = this->create_wall_timer(2000ms, std::bind(&RobotNode::check_target_publisher_alive, this), m_cbGroup6);
  }

  // Setup ROS parameter callback function to handle dynamic parameter (Kp, Ki) updates - ** TEMP ** - needs to be further developed
  void rosParameterUpdate_callback()
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
  void robotStatus_timerCallback()
  {
    auto msg = interfaces::msg::Status();

    blaze::StaticVector<bool, 4UL> reached_status;
    blaze::StaticVector<bool, 4UL> encoder_status;
    blaze::StaticVector<bool, 4UL> en_status;
    en_status = getEnableStatus();
    encoder_status = getEncoderStatus();
    reached_status = getReachedStatus();
    getPosLimit(m_minCurrentPosLimit, m_maxCurrentPosLimit);

    if ((abs(m_x[1] - s_pos_preEngage[1]) < 0.001) && (abs(m_x[3] - s_pos_preEngage[3]) < 0.001))
      m_flag_readyToEngage = true;
    else
      m_flag_readyToEngage = false;
    if ((abs(m_x[1] - s_pos_engage[1]) < 0.003) && (abs(m_x[3] - s_pos_engage[3]) < 0.002))
      m_flagEngaged = true;
    else
      m_flagEngaged = false;

    m_head_attached = !m_digital_input[3][18];

    for (int i = 0; i < 4; i++)
    {
      msg.enable[i] = en_status[i];
      // msg.encoder[i] = encoder_status[i];
      msg.encoder[i] = m_encoders_set[i];
      msg.reached[i] = reached_status[i];
      msg.min_pos_limit[i] = m_minCurrentPosLimit[i];
      msg.max_pos_limit[i] = m_maxCurrentPosLimit[i];
    }

    msg.head_attached = m_head_attached;
    msg.trans_limit_en = m_trans_limit;
    msg.control_mode = static_cast<int>(m_mode);

    msg.ready_to_engage = m_flag_readyToEngage;
    msg.engaged = m_flagEngaged;
    msg.locked = m_locked;

    m_publisher_status->publish(msg);

    // update interface
    auto interface_msg = interfaces::msg::Interface();
    m_interface_key[0] = !m_digital_input[0][17];
    m_interface_key[1] = !m_digital_input[0][18];
    m_interface_key[2] = !m_digital_input[1][17];
    m_interface_key[3] = !m_digital_input[1][18];
    m_interface_key[4] = !m_digital_input[2][17];
    m_interface_key[5] = !m_digital_input[2][18];
    m_interface_key[6] = !m_digital_input[3][17];
    for (int i = 0; i < 7; i++)
    {
      interface_msg.interface_key[i] = m_interface_key[i];
    }
    m_publisher_interface->publish(interface_msg);
  }

  // Timer callback function to read the current joint configurations and publish them
  void jointsConfig_timerCallback()
  {
    auto msg = interfaces::msg::Jointspace();
    getPos(m_x);
    getVel(m_xdot);
    m_current = getCurrent();

    getTemperature(m_cpu_temp, m_driver_temp);
    getDigitalIn(m_digital_input);

    minDynamicPosLimit[1] = std::max(s_minStaticLimitAll[1], m_x[3] - s_linear_stage_max_clearance);
    maxDynamicPosLimit[1] = std::min(s_maxStaticLimitAll[1], m_x[3] - s_linear_stage_min_clearance);
    minDynamicPosLimit[3] = std::max(s_minStaticLimitAll[3], m_x[1] + s_linear_stage_min_clearance);
    maxDynamicPosLimit[3] = std::min(s_maxStaticLimitAll[3], m_x[1] + s_linear_stage_max_clearance);

    msg.position[0UL] = m_x[0UL];
    msg.position[1UL] = m_x[1UL];
    msg.position[2UL] = m_x[2UL];
    msg.position[3UL] = m_x[3UL];
    msg.velocity[0UL] = m_xdot[0UL];
    msg.velocity[1UL] = m_xdot[1UL];
    msg.velocity[2UL] = m_xdot[2UL];
    msg.velocity[3UL] = m_xdot[3UL];
    msg.current[0UL] = m_current[0UL];
    msg.current[1UL] = m_current[1UL];
    msg.current[2UL] = m_current[2UL];
    msg.current[3UL] = m_current[3UL];
    m_publisher_joints->publish(msg);
  }

  // Timer callback function for joint space control loop
  void jointSpaceControl_callback()
  {
    blaze::StaticVector<double, 4L> q_dot_command, x_des, x_dot_des;

    if (!m_flag_manual)
    {
      x_des = m_x_des;
      x_dot_des = m_xdot_des;

      // joint_space_control_step
      m_x_error = x_des - m_x;
      m_x_error_int = m_x_error_int + m_x_error * s_sample_time * 1e-3;
      q_dot_command = m_x_error * m_kp + m_x_error_int * m_ki + x_dot_des;

      // joint_space_control_step(x_des, x_dot_des, q_dot_command);
      // Set_Target_Velocity(q_dot_command);
    }
  }

  // Set the target position/velocity in the robot - Depreciated
  void targetCommand_timerCallback()
  {
    switch (m_mode)
    {
    case CtrlMode::Manual:
      setTargetVel(m_xdot_manual);
      break;
    case CtrlMode::Velocity:
      setTargetVel(m_xdot_des);
      break;
    case CtrlMode::Position:
      setTargetPos(m_x_des);
      std::cout << "m_x_des sent: " << blaze::trans(m_x_des) << std::endl;
      break;
    }
    if (m_trans_limit)
    {
      setPosLimit(minDynamicPosLimit, maxDynamicPosLimit);
    }
    else
    {
      setPosLimit(minDynamicPosLimitInf, maxDynamicPosLimitInf);
    }
  }

  // Subscription callback function to updates the target joint positions and velocities
  void jointSpaceTarget_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    
    if (!m_flag_manual && !m_flag_use_target_action)
    {
      // m_targpublisher_alive_tmep = true;
      m_x_des = blaze::StaticVector<double, 4UL>(0.00);

      switch (m_mode)
      {
      case CtrlMode::Velocity:
        m_xdot_des[0UL] = msg->velocity[0UL];
        m_xdot_des[1UL] = msg->velocity[1UL];
        m_xdot_des[2UL] = msg->velocity[2UL];
        m_xdot_des[3UL] = msg->velocity[3UL];
        std::cout << "Vel target received:" << blaze::trans(m_xdot_des) << std::endl;
        break;
      case CtrlMode::Position:
        m_x_des[0UL] = msg->position[0UL];
        m_x_des[1UL] = msg->position[1UL];
        m_x_des[2UL] = msg->position[2UL];
        m_x_des[3UL] = msg->position[3UL];
        std::cout << "Pos target received:" << blaze::trans(m_x_des) << std::endl;
        break;
      }

    }

    
    // std::cout << "CtrlMode:" << m_mode << std::endl;
  }

  // Subscription callback function updates the current catheter tip status using EMTracker topic
  void current_tool_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_tip = {msg->p[0UL], msg->p[1UL], msg->p[2UL]};
  }

  // Subscription callback function to updates the target joint positions and velocities
  void jointSpaceManualVel_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    if (m_mode == CtrlMode::Manual)
    {
      m_xdot_manual[0UL] = msg->velocity[0UL];
      m_xdot_manual[1UL] = msg->velocity[1UL];
      m_xdot_manual[2UL] = msg->velocity[2UL];
      m_xdot_manual[3UL] = msg->velocity[3UL];
    }
  }

  // Service callback to triger tasks, enable, and control mode section
  void enableService_callback(const std::shared_ptr<interfaces::srv::Config::Request> request,
                              std::shared_ptr<interfaces::srv::Config::Response> response)
  {
    if (request->command == "toggleEnable")
    {
      m_cancel_flag = true; // Signal cancellation to any running task
      auto toggle_thread_ = std::thread(&RobotNode::toggleEnable, this);
      if (toggle_thread_.joinable())
        toggle_thread_.join();
    }
    else if (request->command == "disable")
    {
      m_cancel_flag = true; // Signal cancellation to any running task
      auto toggle_thread_ = std::thread(&RobotNode::disable, this);
      toggle_thread_.join();
    }
    else
    {
      response->success = false;
      response->message = "Invalid command";
    }
    response->success = true;
  }

  // Service callback to triger tasks, enable, and control mode section
  void configService_callback(const std::shared_ptr<interfaces::srv::Config::Request> request,
                              std::shared_ptr<interfaces::srv::Config::Response> response)
  {
    std::future<std::string> result_future;

    if (request->command == "toggleEnable")
    {
      m_cancel_flag = true; // Signal cancellation to any running task
      auto toggle_thread_ = std::thread(&RobotNode::toggleEnable, this);
      if (toggle_thread_.joinable())
        toggle_thread_.join();
    }
    else if (request->command == "disable")
    {
      m_cancel_flag = true; // Signal cancellation to any running task
      auto toggle_thread_ = std::thread(&RobotNode::disable, this);
      toggle_thread_.join();
    }
    else if (request->command == "setCtrlMode")
    {
      setCtrlMode(static_cast<CtrlMode>(request->value));
      return;
    }
    else if (request->command == "setTransLimMode")
    {
      setTranLim(request->value);
      return;
    }
    else if (request->command == "findLinearHome")
    {
      result_future = dispatchTask([this]()
                                   { return findLinearHome(); });
    }
    else if (request->command == "engageCollets")
    {
      result_future = dispatchTask([this]()
                                   { return engageCollets(); });
    }
    else if (request->command == "disengageCollets")
    {
      result_future = dispatchTask([this]()
                                   { return disengageCollets(); });
    }
    else if (request->command == "lockCollets")
    {
      result_future = dispatchTask([this]()
                                   { return lockCollets(); });
    }
    else if (request->command == "unlockCollets")
    {
      result_future = dispatchTask([this]()
                                   { return unlockCollets(); });
    }
    else if (request->command == "findRotaryHome")
    {
      result_future = dispatchTask([this]()
                                   { return findRotaryHome(); });
    }
    else if (request->command == "goHome")
    {
      result_future = dispatchTask([this]()
                                   { return goHome(); });
    }
    else if (request->command == "findRotaryHomeAndGoHome")
    {
      result_future = dispatchTask([this]()
                                   { return findRotaryHomeAndGoHome(); });
    }
    else if (request->command == "engageAndUnlock")
    {
      result_future = dispatchTask([this]()
                                   { return engageUnlock(); });
    }
    else if (request->command == "lockAndDisengage")
    {
      result_future = dispatchTask([this]()
                                   { return lockDisengage(); });
    }
    else
    {
      response->success = false;
      response->message = "Invalid command";
      return;
    }

    response->success = true;
    response->message = "Task received";

    // // Wait for result from task
    // if (result_future.valid()) {
    //   std::string result = result_future.get();  // Block only this service thread
    //   response->success = (result == "OK");
    //   response->message = result;
    // }
  }

  //
  void handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
  {
    // Get the result of the future object
    auto response = future.get();

    if (response->success)
    {
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
    }
  }

  // Timer callback function to check the lifecycle of the em tracker node
  void check_emtracker_alive_timerCallback()
  {
    // Set the flag to false; it will be set to true if current_catheter_tip_callback is called
    if (m_subscription_tool->get_publisher_count() != 0 && !m_emtracker_alive)
    {
      m_emtracker_alive = true;
      m_logger->info("[RobotNode] EM Tracker_node is alive");
    }
    else if (m_subscription_tool->get_publisher_count() == 0 && m_emtracker_alive)
    {
      m_emtracker_alive = false;
      m_logger->warn("[RobotNode] EM Tracker_node is dead");
    }
  }

  // Timer callback function to check the life cycle of the publisher node
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

  // dispatch robot config tasks on a dedicated thread
  void dispatchTask_old(std::function<void()> task)
  {
    std::lock_guard<std::mutex> lock(m_task_mutex);
    m_cancel_flag = true; // Signal cancellation to any running task
    // Wait a moment for running task to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //
    m_current_task = std::move(task);
    m_flag_task_ready = true;
    m_task_cv.notify_one();
  }

  // dispatch robot config tasks on a dedicated thread
  std::future<std::string> dispatchTask(std::function<std::string()> task)
  {
    std::lock_guard<std::mutex> lock(m_task_mutex);
    m_cancel_flag = true; // Signal cancellation to any running task
    // Wait a moment for running task to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Promise for task results
    auto promise = std::make_shared<std::promise<std::string>>();
    std::future<std::string> results_future = promise->get_future();

    m_current_task = [task, promise]()
    {
      std::string result = task(); // Execuse the task
      promise->set_value(result);  // return result
    };

    m_flag_task_ready = true;
    m_task_cv.notify_one();

    return results_future;
  }

  // =============================================== Robot Config/Setup Tasks ====================================== //
  // set robot control mode to config mode
  void switchToConfigMode()
  {
    setTargetVel({0.0, 0.0, 0.0, 0.0});
    m_trans_limit = false; // disable translation limits
    m_mode = CtrlMode::Config;
    m_logger->info("[RobotNode] Selected control mode: Config");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // set robot control mode
  void setCtrlMode(const CtrlMode mode)
  {
    if (mode == CtrlMode::Config)
    {
      switchToConfigMode();
    }
    else if (mode == CtrlMode::Manual)
    {
      setTargetVel({0.0, 0.0, 0.0, 0.0});
      blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
      blaze::StaticVector<double, 4UL> max_vel = {100.00 * M_PI / 180.00, 10.00 / 1000.00, 100.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
      blaze::StaticVector<double, 4UL> negative = {500.0, 500.0, 500.0, 500.0};
      blaze::StaticVector<double, 4UL> positive = {500.0, 500.0, 500.0, 500.0};
      setOperationMode(OpMode::VelocityProfile);
      setProfileParams(max_vel, max_dcc, max_dcc);
      setMaxTorque(negative, positive);
      m_mode = CtrlMode::Manual;
      m_logger->info("[RobotNode] Selected Mode: Manual");
    }
    else if (mode == CtrlMode::Position)
    {
      setTargetVel({0.0, 0.0, 0.0, 0.0});
      m_x_des = m_x;
      m_trans_limit = true; // enable translation limits
      setOperationMode(OpMode::PositionProfile);
      m_mode = CtrlMode::Position;
      m_logger->info("[RobotNode] Selected Mode: Position");
    }
    else if (mode == CtrlMode::Velocity)
    {
      setTargetVel({0.0, 0.0, 0.0, 0.0});
      m_trans_limit = true; // enable translation limits
      setOperationMode(OpMode::VelocityProfile);
      m_mode = CtrlMode::Velocity;
      m_logger->info("[RobotNode] Selected Mode: Velocity");
    }
  }

  // enable/disable hardcoded translation limits for stages
  void setTranLim(int enable)
  {
    if (m_mode == CtrlMode::Manual)
    {
      if (enable)
      {
        m_trans_limit = true;
        m_logger->info("[RobotNode] Translation Limit ON");
      }
      else
      {
        m_trans_limit = false;
        m_logger->info("[RobotNode] Translation Limit OFF");
      }
    }
    else
    {
      m_logger->info("[RobotNode] setTranLim() is not allowed in Non-Manual modes");
    }
  }

  // enable or distable the robot
  void toggleEnable()
  {
    blaze::StaticVector<bool, 4UL> en_status = getEnableStatus();
    if (en_status[0] || en_status[1] || en_status[2] || en_status[3])
      enableOperation(false);
    else
      enableOperation(true);
  }

  // enable or distable the robot
  void disable()
  {
    enableOperation(false);
  }

  // lock the couplings and move the the linear statges to the proximal mechanical limit of the robot to home the linear encoders
  std::string findLinearHome()
  {
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] findLinearHome() task terminated");
        return true;
      }
      return false;
    };

    bool flag_inr_encoder_set = false;
    bool flag_mdl_encoder_set = false;
    blaze::StaticVector<double, 4UL> current;
    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {300.0, 300.0, 300.0, 300.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {300.0, 300.0, 300.0, 300.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {20.00 * M_PI / 180.00, 2.00 / 1000.00, 20.00 * M_PI / 180.00, 2.00 / 1000.00};     // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = {50.00 * M_PI / 180.00, 5.00 / 1000.00, 50.00 * M_PI / 180.00, 5.00 / 1000.00};     // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxVel = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
    constexpr double current_thresh = 10.0;
    constexpr double inr_thresh = -1.0 * (maxTorqueNegative[1] - current_thresh);
    constexpr double mdl_thresh = -1.0 * (maxTorqueNegative[3] - current_thresh);
    blaze::StaticVector<double, 4UL> target_vel = {-0.0, -0.0055, -0.0, -0.005};

    // set control mode and parameters
    m_logger->info("[RobotNode] Finding linear home...");
    switchToConfigMode();
    setOperationMode(OpMode::VelocityProfile);
    setMaxTorque(maxTorqueNegative, maxTorquePositive);
    setProfileParams(maxVel, maxAcc, maxDcc);
    // move linear joints with constant velocity
    setTargetVel(target_vel);
    enableOperation(true);
    // monitor linear joint currents to detect mechanical limits
    while (!flag_inr_encoder_set || !flag_mdl_encoder_set)
    {
      if (check_cancel())
        return "canceled";
      current = getCurrent();
      if (current[1] <= inr_thresh && !flag_inr_encoder_set)
      {
        m_logger->info("[RobotNode] inner carriage hit mechanical limit");
        target_vel[1] = 0.0;
        setTargetVel(target_vel);
        flag_inr_encoder_set = true;
      }
      if (current[3] <= mdl_thresh && !flag_mdl_encoder_set)
      {
        m_logger->info("[RobotNode] middle carriage hit mechanical limit");
        target_vel[3] = 0.0;
        setTargetVel(target_vel);
        flag_mdl_encoder_set = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // rotate rotary joints to make sure couplings are engaged
    setTargetVel({2.0, 0.0, 2.0, 0.0});
    sleep_thread_cancelable(4000);
    if (check_cancel())
      return "canceled";
    setTargetVel({0.0, 0.0, 0.0, 0.0});
    // set encoders
    setEncoders({0.0, s_pos_inr_prox_stop, 0.0, s_pos_mdl_prox_stop});
    sleep_thread_cancelable(500);
    if (check_cancel())
      return "canceled";
    enableOperation(false);
    setTargetPos(m_x);
    m_encoders_set[1] = true;
    m_encoders_set[3] = true;
    m_logger->info("[RobotNode] linear joints encoders found");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string res;
    res = "OK";
    return res;
  }

  // disengage the collets
  std::string disengageCollets()
  {
    if (!(m_encoders_set[1] && m_encoders_set[3])) // check if linear encoders are set
    {
      m_logger->warn("[RobotNode] Linear encoders must be set - disEngageCollets() task terminated");
      return "terminated";
    }
    if (!m_flagEngaged) // check if the collets are engaged
    {
      m_logger->warn("[RobotNode] collets are not engaged - disEngageCollets() task terminated");
      return "terminated";
    }
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] disEngageCollets() task terminated");
        return true;
      }
      return false;
    };

    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {200.0, 400.0, 200.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {200.0, 400.0, 200.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {50.00 * M_PI / 180.00, 5.00 / 1000.00, 50.00 * M_PI / 180.00, 5.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]

    m_logger->info("[RobotNode] Disengaging collets...");
    switchToConfigMode();
    setOperationMode(OpMode::PositionProfile);
    setProfileParams(maxVel, maxAcc, maxDcc);
    setMaxTorque(maxTorqueNegative, maxTorquePositive);
    enableOperation(true);
    if (check_cancel())
      return "canceled";
    setTargetPos({m_x[0], s_pos_preEngage[1], m_x[2], s_pos_preEngage[3]});
    waitUntilTransReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    m_logger->info("[RobotNode] Collets disengaged");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string res;
    res = "OK";
    return res;
  }

  // engage the collets
  std::string engageCollets()
  {
    if (!(m_encoders_set[1] && m_encoders_set[3])) // check if linear encoders are set
    {
      m_logger->warn("[RobotNode] Linear encoders must be set - engageCollets() task terminated");
      return "terminated";
    }
    if (m_flagEngaged) // check if the collets are engaged
    {
      m_logger->warn("[RobotNode] collets are already engaged - disEngageCollets() task terminated");
      return "terminated";
    }
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] engageCollets() task terminated");
        return true;
      }
      return false;
    };

    blaze::StaticVector<double, 4UL> maxTorqueNegative = {200.0, 400.0, 200.0, 400.0};
    blaze::StaticVector<double, 4UL> maxTorquePositive = {200.0, 400.0, 200.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {50.00 * M_PI / 180.00, 5.00 / 1000.00, 50.00 * M_PI / 180.00, 5.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
    constexpr double current_thresh = 10.0;
    blaze::StaticVector<double, 4UL> targetPosTemp;

    m_logger->info("[RobotNode] Engaging collets...");
    switchToConfigMode();
    setOperationMode(OpMode::PositionProfile);
    setProfileParams(maxVel, maxAcc, maxDcc);

    // if the stages are not in pre-engae mode, move to pre-engage mode first
    if (!m_flag_readyToEngage)
    {
      m_logger->info("[RobotNode] Moving to pre-engage location");
      setMaxTorque(maxTorqueNegative, maxTorquePositive);
      enableOperation(true);
      targetPosTemp = {m_x[0], s_pos_preEngage[1], m_x[2], s_pos_preEngage[3]};
      setTargetPos(targetPosTemp);
      waitUntilTransReach(m_cancel_flag);
      if (check_cancel())
        return "canceled";
    }

    // if the stages are in pre-engae mode, engage collets
    if (m_flag_readyToEngage)
    {
      maxTorqueNegative = {350.0, 250.0, 350.0, 250.0};
      maxTorquePositive = {350.0, 250.0, 350.0, 250.0};
      setMaxTorque(maxTorqueNegative, maxTorquePositive);
      enableOperation(true);

      // attemp to engage middle collet - step one - setting translational joint
      m_logger->info("[RobotNode] Engaging middle collet");
      targetPosTemp = {m_x[0], s_pos_preEngage[1], m_x[2], s_pos_engage[3]};
      setTargetPos(targetPosTemp);
      sleep_thread_cancelable(1500);
      if (check_cancel())
        return "canceled";
      // if it doesn't each the target, back of, rotate, and try again.
      while (!getReachedStatus()[3])
      {
        if (check_cancel())
          return "canceled";
        targetPosTemp[3] = s_pos_engage[3] - 0.004;
        setTargetPos(targetPosTemp); // back off for 4 mm
        sleep_thread_cancelable(1000);
        targetPosTemp[2] = m_x[2] + 0.07 * M_PI;
        setTargetPos(targetPosTemp); // rotate
        sleep_thread_cancelable(1000);
        targetPosTemp[2] = m_x[2];
        targetPosTemp[3] = s_pos_engage[3];
        setTargetPos(targetPosTemp); // attemp to engage middle collet again
        sleep_thread_cancelable(1500);
      }
      targetPosTemp = m_x;
      setTargetPos(targetPosTemp);
      m_logger->debug("Engaing middle coller step one done");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // attemp to engage middle collet - step two - slightly locking the collet so always continue from same config
      targetPosTemp[2] = targetPosTemp[2] + 20.0 * M_PI; // rotate collet to lock
      setTargetPos(targetPosTemp);
      // monitor current
      while (!(m_current[2] > (maxTorquePositive[2] - current_thresh)))
      {
        if (check_cancel())
          return "canceled";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      setTargetPos(m_x);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      m_logger->info("[RobotNode] Middle collet engaged");

      // attemp to engage inner collet - step one - setting translational joint
      m_logger->info("[RobotNode] Engaging the inner collet");
      targetPosTemp = {m_x[0], s_pos_engage[1], m_x[2], s_pos_engage[3]};
      setTargetPos(targetPosTemp);
      sleep_thread_cancelable(2000);
      if (check_cancel())
        return "canceled";
      // if it doesn't each the target, back of, rotate, and try again.
      while (!getReachedStatus()[1])
      {
        if (check_cancel())
          return "canceled";
        targetPosTemp[1] = s_pos_engage[1] - 0.004;
        setTargetPos(targetPosTemp);
        sleep_thread_cancelable(1000);
        targetPosTemp[0] = m_x[0] + 0.07 * M_PI;
        setTargetPos(targetPosTemp);
        sleep_thread_cancelable(1000);
        targetPosTemp[0] = m_x[0];
        targetPosTemp[1] = s_pos_engage[1];
        setTargetPos(targetPosTemp);
        sleep_thread_cancelable(1500);
      }
      targetPosTemp = m_x;
      setTargetPos(targetPosTemp);
      m_logger->debug("Engaing inner coller step one done");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // attemp to engage middle collet - step two - slightly locking the collet so always continue from same config
      targetPosTemp = m_x;
      targetPosTemp[0] = targetPosTemp[0] + 20.0 * M_PI;
      setTargetPos(targetPosTemp);
      while (!(m_current[0] > (maxTorquePositive[0] - current_thresh)))
      {
        if (check_cancel())
          return "canceled";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      setTargetPos(m_x);
      m_logger->info("[RobotNode] Inner collet engaged");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      m_logger->info("[RobotNode] Both collets engaged");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string res;
    res = "OK";
    return res;
  }

  // lock collets
  std::string lockCollets()
  {
    if (!m_flagEngaged) // check if collets are engaged
    {
      m_logger->warn("[RobotNode] Collets must be engaged before locking - lockCollets() task terminated");
      return "terminated";
    }
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] lockCollets() task terminated");
        return true;
      }
      return false;
    };

    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {400.0, 300.0, 400.0, 300.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {400.0, 300.0, 400.0, 300.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {100.00 * M_PI / 20.00, 0.50 / 1000.00, 100.00 * M_PI / 20.00, 0.50 / 1000.00}; // [deg/s] and [mm/s]
    constexpr blaze::StaticVector<double, 4UL> motion = {15 * M_PI, 0.0010, 15 * M_PI, 0.0005};                                         // *********** the parameters must be adjusted based on screw lead *********
    blaze::StaticVector<bool, 4UL> reach;
    constexpr double current_thresh = 10.0;
    constexpr double inr_thresh = (maxTorquePositive[0] - current_thresh);
    constexpr double mdl_thresh = (maxTorquePositive[2] - current_thresh);

    m_logger->info("[RobotNode] Locking collets... ");
    switchToConfigMode();
    setOperationMode(OpMode::PositionProfile);
    setProfileParams(maxVel, maxAcc, maxDcc);
    setMaxTorque(maxTorqueNegative, maxTorquePositive);
    enableOperation(true);
    setTargetPos(m_x + motion);
    sleep_thread_cancelable(500);
    if (check_cancel())
      return "canceled";
    reach = getReachedStatus();
    while (!(m_current[0] > inr_thresh) || !reach[1] || !(m_current[2] > mdl_thresh) || !reach[3])
    {
      if (check_cancel())
        return "canceled";
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    setTargetPos(m_x);
    m_logger->info("[RobotNode] Collets locked");
    m_locked = 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string res;
    res = "OK";
    return res;
  }

  // unlock collets
  std::string unlockCollets()
  {
    if (!m_flagEngaged) // check if collets are engaged
    {
      m_logger->warn("[RobotNode] Collets must be engaged before locking - unlockCollets() task terminated");
      return "terminated";
    }
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] unlockCollets() task terminated");
        return true;
      }
      return false;
    };

    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {1000.0, 220.0, 1000.0, 220.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {1000.0, 220.0, 1000.0, 220.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {100.00 * M_PI / 20.00, 0.50 / 1000.00, 100.00 * M_PI / 20.00, 0.50 / 1000.00}; // [deg/s] and [mm/s]
    constexpr blaze::StaticVector<double, 4UL> motion = {-4 * M_PI, -0.0010, -4 * M_PI, -0.0005};                                       // *********** the parameters must be adjusted based on screw lead *********
    blaze::StaticVector<bool, 4UL> reach;
    constexpr double current_thresh = 10.0;
    constexpr double inr_thresh = (maxTorquePositive[0] - current_thresh);
    constexpr double mdl_thresh = (maxTorquePositive[2] - current_thresh);

    m_logger->info("[RobotNode] Unlocking collets...");
    switchToConfigMode();
    setOperationMode(OpMode::PositionProfile);
    setProfileParams(maxVel, maxAcc, maxDcc);
    setMaxTorque(maxTorqueNegative, maxTorquePositive);
    enableOperation(true);
    setTargetPos(m_x + motion);
    sleep_thread_cancelable(500);
    reach = getReachedStatus();
    while (!reach[0] || !reach[1] || !reach[2] || !reach[3])
    {
      if (check_cancel())
        return "canceled";
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    m_logger->info("[RobotNode] Collets unlocked");
    m_locked = -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string res;
    res = "OK";
    return res;
  }

  // find the home location of the rotary encoders thorugh reading the tip position while doing independent roation of tubes
  std::string findRotaryHome()
  {
    if (!(m_encoders_set[1] && m_encoders_set[3])) // check if linear encoders are set
    {
      m_logger->warn("[RobotNode] Linear encoders must be set - findRotaryHome() terminated");
      return "termianted";
    }
    if (!m_emtracker_alive) // check if the em tracker node is alive
    {
      m_logger->warn("[RobotNode] emtracker must be alive for automated rotary homing");
      return "termianted";
    }
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] findRotaryHome() task terminated");
        return true;
      }
      return false;
    };

    auto find_min = [](const std::vector<std::pair<double, double>> &data,
                       double &best_input,
                       double &min_output)
    {
      min_output = std::numeric_limits<double>::max();
      for (const auto &[input, output] : data)
      {
        if (output < min_output)
        {
          min_output = output;
          best_input = input;
        }
      }
    };

    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {400.0, 400.0, 400.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {400.0, 400.0, 400.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]

    blaze::StaticVector<double, 4UL> targetPosTemp;
    blaze::StaticVector<bool, 4UL> reach;

    std::vector<std::pair<double, double>> data;
    std::pair<double, double> row;
    double min_output = 0.0;
    double best_input = 0.0;

    m_logger->info("[RobotNode] Finding rotary joints encoder...");
    switchToConfigMode();
    setOperationMode(OpMode::PositionProfile);
    setProfileParams(maxVel, maxAcc, maxDcc);
    setMaxTorque(maxTorqueNegative, maxTorquePositive);
    enableOperation(true);

    // moving to pre-engage postion. This is the most extended positon of the tubes
    targetPosTemp = {m_x[0], s_pos_preEngage[1], m_x[2], s_pos_preEngage[3]};
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    // rotate the middle tube for 2PI while recoding tip Y position
    m_logger->info("[RobotNode] Middle tube coarse rotation...");
    targetPosTemp = {m_x[0], s_pos_preEngage[1], m_x[2] + 2 * M_PI, s_pos_preEngage[3]};
    setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[2], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    // find the config leading to minimum Y and move to that config
    find_min(data, best_input, min_output);
    m_logger->debug("[RobotNode] Minimum Y for middle tube: q = {:.2f} | tip y = {:.5f}", best_input, min_output);
    targetPosTemp[2] = best_input;
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    // rotate the inner tube for 2PI while recoding tip Y position
    m_logger->info("[RobotNode] Inner tube coarse rotation...");
    targetPosTemp = {m_x[0] + 2 * M_PI, s_pos_preEngage[1], m_x[2], s_pos_preEngage[3]};
    setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[0], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    // find the config leading to minimum Y and move to that config
    find_min(data, best_input, min_output);
    m_logger->debug("[RobotNode] Minimum Y for inner tube: q = {:.2f} | tip y = {:.5f}", best_input, min_output);
    targetPosTemp[0] = best_input;
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    // now repeat the process for double sided shorter range of motion for fine tuning
    // first for the inner tube
    targetPosTemp[0] += M_PI / 2;
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    // moving in negative direction and recording tip position
    m_logger->info("[RobotNode] Inner tube fine positive rotation...");
    targetPosTemp[0] -= M_PI;
    setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[0], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    find_min(data, best_input, min_output);
    double input = best_input;
    // moving in positive direction and recording tip position
    m_logger->info("[RobotNode] Inner tube fine negative rotation...");
    targetPosTemp[0] += M_PI;
    setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[0], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    find_min(data, best_input, min_output);
    input += best_input;
    input /= 2; // computing the averagte of minimum config for both sides
    m_logger->debug("[RobotNode] Minimum Y for inner tube: q = {:.2f}", input);
    // computing the averagte of minimum config for both sides
    targetPosTemp[0] = input;
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    m_logger->info("[RobotNode] Inner tube is in minimum Y posoition");

    // then for the middle tube
    m_logger->debug("[RobotNode] Round 2 - repeat for fine tunnig");
    targetPosTemp[2] += M_PI / 3;
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    // moving in negative direction and recording tip position
    m_logger->info("[RobotNode] Middle tube fine negative rotation...");
    targetPosTemp[2] -= 2 * M_PI / 3;
    setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[2], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    find_min(data, best_input, min_output);
    input = best_input;
    // moving in positive direction and recording tip position
    m_logger->info("[RobotNode] Middle tube fine positive rotation...");
    targetPosTemp[2] += 2 * M_PI / 3;
    setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[2], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = getReachedStatus();
    }
    find_min(data, best_input, min_output);
    input += best_input;
    input /= 2; // computing the averagte of minimum config for both sides
    m_logger->debug("[RobotNode] Minimum Y for inner tube: q = {:.2f}", input);
    // computing the averagte of minimum config for both sides
    targetPosTemp[2] = input;
    setTargetPos(targetPosTemp);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    m_logger->info("[RobotNode] Middle tube is in minimum Y posoition");

    setEncoders({0.0, m_x[1], 0.0, m_x[3]});
    sleep_thread_cancelable(500);
    if (check_cancel())
      return "canceled";
    enableOperation(false);
    setTargetPos(m_x);
    m_encoders_set[0] = true;
    m_encoders_set[2] = true;
    m_logger->info("[RobotNode] Rotary joints encoders found");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string res;
    res = "OK";
    return res;
  }

  // move all joints to home position (flush tubes with zero rotation)
  std::string goHome()
  {
    if (!m_encoders_set[0] || !m_encoders_set[1] || !m_encoders_set[2] || !m_encoders_set[3]) // check if linear encoders are set
    {
      m_logger->warn("[RobotNode] All encoders must be set - goHome() task terminated");
      return "terminated";
    }
    auto check_cancel = [&]()
    {
      if (m_cancel_flag.load())
      {
        m_logger->warn("[RobotNode] goHome() task terminated");
        return true;
      }
      return false;
    };

    m_logger->info("Going to Home...");
    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {400.0, 400.0, 400.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {400.0, 400.0, 400.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {50.00 * M_PI / 180.00, 5.00 / 1000.00, 50.00 * M_PI / 180.00, 5.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]

    switchToConfigMode();
    // m_trans_limit = true; // enable translation limits
    setOperationMode(OpMode::PositionProfile);
    setProfileParams(maxVel, maxAcc, maxDcc);
    setMaxTorque(maxTorqueNegative, maxTorquePositive);
    enableOperation(true);
    setTargetPos(s_home_pos);
    waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    m_logger->info("[RobotNode] Homed");

    std::string res;
    res = "OK";
    return res;
  }

  // move all joints to home position (flush tubes with zero rotation)
  std::string engageUnlock()
  {
    m_logger->info("[RobotNode] Engage collets and unlock");
    engageCollets();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    unlockCollets();

    std::string res;
    res = "OK";
    return res;
  }

  // move all joints to home position (flush tubes with zero rotation)
  std::string lockDisengage()
  {
    m_logger->info("[RobotNode] Lock collets and disengage");
    lockCollets();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    disengageCollets();

    std::string res;
    res = "OK";
    return res;
  }

  // move all joints to home position (flush tubes with zero rotation)
  std::string findRotaryHomeAndGoHome()
  {
    m_logger->info("[RobotNode] Find rotary home and go home");
    findRotaryHome();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    goHome();

    std::string res;
    res = "OK";
    return res;
  }


  // Wait in milliseconds
  void sleep_thread_cancelable(const int milliseconds)
  {
    for (int i = 0; i < floor(milliseconds / 10); i++)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (m_cancel_flag.load())
        return;
    }
  }

  static constexpr double s_inr_active_length = 0.216; // need to be adjusted based on the tube set
  static constexpr double s_mdl_active_length = 0.132; // need to be adjusted based on the tube set
  static constexpr double s_otr_active_length = 0.060; // need to be adjusted based on the tube set

  static constexpr int s_sample_time = 20; // [ms]

  static constexpr blaze::StaticVector<double, 4> s_pos_preEngage = {0.0, -0.0660, 0.0, -0.0360};
  static constexpr blaze::StaticVector<double, 4> s_pos_engage = {0.0, -0.0560, 0.0, -0.0290};
  static constexpr double s_pos_inr_prox_stop = -0.1670; // need to be adjusted based on the robot design
  static constexpr double s_pos_mdl_prox_stop = -0.1230; // need to be adjusted based on the robot design
  static constexpr double s_linear_stage_min_clearance = 0.030;
  static constexpr double s_linear_stage_max_clearance = s_inr_active_length - s_mdl_active_length;
  static constexpr blaze::StaticVector<double, 4> s_home_pos = {0.0, s_otr_active_length - s_inr_active_length, 0.0, s_otr_active_length - s_mdl_active_length};
  static constexpr blaze::StaticVector<double, 4> s_minStaticLimitAll = {-20 * M_PI, s_home_pos[1], 20 * M_PI, s_home_pos[3]};
  static constexpr blaze::StaticVector<double, 4> s_maxStaticLimitAll = {20 * M_PI, s_pos_preEngage[1], 20 * M_PI, s_pos_preEngage[3]};

  blaze::StaticVector<double, 4> minDynamicPosLimitInf = {-40 * M_PI, -0.50, -40 * M_PI, -0.50}; // for when the limit is off
  blaze::StaticVector<double, 4> maxDynamicPosLimitInf = {40 * M_PI, 0.50, 40 * M_PI, 0.50};     // for when the limit is off
  blaze::StaticVector<double, 4> minDynamicPosLimit = {-10 * M_PI, 0.001, -10 * M_PI, 0.040};    // for when the limit is on
  blaze::StaticVector<double, 4> maxDynamicPosLimit = {10 * M_PI, 0.088, 10 * M_PI, 0.129};      // for when the limit is on

  bool m_flag_manual, m_flag_use_target_action, m_flag_enabled, m_trans_limit = false;
  bool m_emtracker_alive, m_targpublisher_alive, m_targpublisher_alive_tmep = false;
  bool m_flag_readyToEngage, m_flagEngaged, m_head_attached = false;
  int m_locked = 0;
  double m_kp, m_ki = 0.00;

  std::array<bool, 7> m_interface_key = {0, 0, 0, 0, 0, 0, 0};
  std::array<bool, 4> m_encoders_set = {0, 0, 0, 0};

  CtrlMode m_mode; // controller mode (manual, velocity, position)

  rclcpp::TimerBase::SharedPtr m_watchdog_timer_emt;
  rclcpp::TimerBase::SharedPtr m_watchdog_timer_target;
  rclcpp::TimerBase::SharedPtr m_read_robot_timer;
  rclcpp::TimerBase::SharedPtr m_joints_config_timer;
  rclcpp::TimerBase::SharedPtr m_control_loop_timer;
  rclcpp::TimerBase::SharedPtr m_position_control_timer;
  rclcpp::TimerBase::SharedPtr m_system_identification_timer;
  rclcpp::TimerBase::SharedPtr m_read_key_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_joints;
  rclcpp::Publisher<interfaces::msg::Status>::SharedPtr m_publisher_status;
  rclcpp::Publisher<interfaces::msg::Interface>::SharedPtr m_publisher_interface;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_manual_vel;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_tool;
  rclcpp::Service<interfaces::srv::Config>::SharedPtr m_config_service;
  rclcpp::Service<interfaces::srv::Config>::SharedPtr m_enable_service;

  rclcpp::CallbackGroup::SharedPtr m_cbGroup1, m_cbGroup2, m_cbGroup3, m_cbGroup4, m_cbGroup5, m_cbGroup6, m_cbGroup7, m_cbGroup8;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_watchdog_1;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  size_t publisher_count;

  blaze::StaticVector<double, 4UL> m_x, m_x_des, m_x_error, m_x_abs;                                // in SI units
  blaze::StaticVector<double, 4UL> m_xdot, m_xdot_manual, m_xdot_des, m_xdot_error, m_xdot_forward; // in SI units
  blaze::StaticVector<double, 4UL> m_x_error_int;                                                   // in SI units
  blaze::StaticVector<double, 4UL> m_current;
  blaze::StaticVector<double, 4UL> m_minCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 4UL> m_maxCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 3UL> m_x_tip;

  blaze::StaticVector<int32_t, 4UL> m_cpu_temp;
  blaze::StaticVector<int32_t, 4UL> m_driver_temp;
  blaze::StaticVector<std::bitset<32>, 4UL> m_digital_input;

  blaze::StaticVector<double, 4UL> m_maxTorqueNegative;
  blaze::StaticVector<double, 4UL> m_maxTorquePositive;
  blaze::StaticVector<double, 4UL> m_maxDcc; // [deg/s^2] and [mm/s^2]
  blaze::StaticVector<double, 4UL> m_maxVel; // [deg/s] and [mm/s]
  blaze::StaticVector<double, 4UL> m_maxAcc; // [deg/s^2] and [mm/s^2]

  std::thread worker_thread_;
  std::mutex m_task_mutex;
  std::condition_variable m_task_cv;
  std::function<void()> m_current_task;
  std::atomic<bool> m_cancel_flag{false};
  std::atomic<bool> m_flag_task_ready{false};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 7);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

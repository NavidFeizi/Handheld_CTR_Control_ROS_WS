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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ctr_robot_driver/Robot.hpp"
#include <mutex>
#include <rcutils/logging.h>
#include <spdlog/sinks/base_sink.h>
#include "ctr_common/package_log.hpp"
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

enum class CtrlMode : int
{
  Config = 0x00,
  Manual = 0x01,
  Position = 0x02,
  Velocity = 0x03,
};
// Human-readable control-mode name for diagnostic messages.
static const char *ctrlModeName(CtrlMode mode)
{
  switch (mode)
  {
  case CtrlMode::Config:
    return "Config";
  case CtrlMode::Manual:
    return "Manual";
  case CtrlMode::Position:
    return "Position";
  case CtrlMode::Velocity:
    return "Velocity";
  }
  return "Unknown";
}

namespace
{
// Tees every record written through m_logger (spdlog) into the same package
// log file installPackageFileLog() sets up for rcutils/RCLCPP_* records, so a
// single <log_root>/robot/<...>_ctr_robot_<pid>.log has both.
class PackageLogSink : public spdlog::sinks::base_sink<std::mutex>
{
protected:
  void sink_it_(const spdlog::details::log_msg &msg) override
  {
    int severity = RCUTILS_LOG_SEVERITY_INFO;
    switch (msg.level)
    {
    case spdlog::level::trace:
    case spdlog::level::debug:
      severity = RCUTILS_LOG_SEVERITY_DEBUG;
      break;
    case spdlog::level::info:
      severity = RCUTILS_LOG_SEVERITY_INFO;
      break;
    case spdlog::level::warn:
      severity = RCUTILS_LOG_SEVERITY_WARN;
      break;
    case spdlog::level::err:
      severity = RCUTILS_LOG_SEVERITY_ERROR;
      break;
    case spdlog::level::critical:
      severity = RCUTILS_LOG_SEVERITY_FATAL;
      break;
    default:
      break;
    }
    ctr_common::writePackageLog(severity, "robot_node.spdlog",
                                std::string(msg.payload.data(), msg.payload.size()));
  }
  void flush_() override {}
};
} // namespace

class RobotNode : public rclcpp::Node
{

public:
  RobotNode() : rclcpp::Node("ctr_robot")
  {
    ctr_common::installPackageFileLog(*this, "robot");
    m_logger->sinks().push_back(std::make_shared<PackageLogSink>());

    m_flag_use_target_action = false;
    m_trans_limit = true;
    m_encoders_set = {1, 1, 1, 1}; /// temoporarly for development
    declare_parameters();

    RCLCPP_INFO(get_logger(),
                "[lifecycle] static position limits (wire [a1,b1,a2,b2]): min=[%.4f, %.4f, %.4f, %.4f] "
                "max=[%.4f, %.4f, %.4f, %.4f] rad/m",
                k_minStaticLimitAll[0], k_minStaticLimitAll[1], k_minStaticLimitAll[2], k_minStaticLimitAll[3],
                k_maxStaticLimitAll[0], k_maxStaticLimitAll[1], k_maxStaticLimitAll[2], k_maxStaticLimitAll[3]);
    RCLCPP_INFO(get_logger(),
                "[lifecycle] stage clearances: rotary=[%.4f, %.4f] rad, linear=[%.4f, %.4f] m",
                k_rotary_stage_min_clearance, k_rotary_stage_max_clearance,
                k_linear_stage_min_clearance, k_linear_stage_max_clearance);

    // Runtime paths for the CANopen driver (formerly compile-time macros).
    CTRobot::RuntimePaths paths;
    paths.canopen_dir = declare_parameter<std::string>("canopen_dir", "");
    if (paths.canopen_dir.empty())
    {
      paths.canopen_dir =
          ament_index_cpp::get_package_share_directory("ctr_robot_driver") + "/canopen";
    }
    paths.can_interface = declare_parameter<std::string>("can_interface", "can0");
    paths.encoder_memory_dir = declare_parameter<std::string>("encoder_memory_dir", "");
    paths.boot_max_attempts = static_cast<int>(declare_parameter<int64_t>("boot_max_attempts", 10));
    paths.boot_timeout_s = declare_parameter<double>("boot_timeout_s", 5.0);

    RCLCPP_INFO(get_logger(),
                "[lifecycle] CANopen runtime paths: can_interface='%s' canopen_dir='%s' "
                "encoder_memory_dir='%s' boot_max_attempts=%d boot_timeout_s=%.2f",
                paths.can_interface.c_str(), paths.canopen_dir.c_str(),
                paths.encoder_memory_dir.c_str(), paths.boot_max_attempts, paths.boot_timeout_s);

    auto robot = std::make_unique<CTRobot>();
    robot->setRuntimePaths(paths);
    m_robot = std::move(robot);

    m_robot->setMaxVel(m_maxVel);
    m_robot->setMaxAcc(m_maxAcc);
    initRosInterfaces();
    // Hardware bring-up happens in initHardware(), called from main() before
    // the executor spins — same ordering as before, but construction can no
    // longer deadlock on absent hardware.

    worker_thread_ = std::thread([this]()
                                 {

                                  while (!m_stop_worker.load())
        {
            std::unique_lock<std::mutex> lock(m_task_mutex);
            m_task_cv.wait(lock, [this]() {
                return m_flag_task_ready.load() || m_stop_worker.load();
            });

            if (m_stop_worker.load()) break;

            m_cancel_flag = false;
            m_flag_task_ready = false;

            auto task = m_current_task;
            lock.unlock();

            if (task) task();
        } });
  }

  // Bring up the CAN hardware. Called from main() BEFORE the executor spins.
  bool initHardware()
  {
    if (!m_robot->connect(k_sample_time))
    {
      RCLCPP_FATAL(get_logger(), "[lifecycle] CANopen bring-up failed after retries - node stays up, hardware unavailable");
      return false;
    }
    RCLCPP_INFO(get_logger(), "[lifecycle] CANopen bring-up succeeded - hardware connected");
    return true;
  }

  ~RobotNode() override
  {
    m_stop_worker.store(true);
    m_flag_task_ready.store(true);
    m_task_cv.notify_one();
    if (worker_thread_.joinable())
      worker_thread_.join();
  }

private:
  // Declare ROS parameters
  void declare_parameters()
  {
    // Declare motion limit parameters as vectors
    declare_parameter<std::vector<double>>("maxVel", {3.0, 0.012, 3.0, 0.012});
    declare_parameter<std::vector<double>>("maxAcc", {10.0, 0.10, 10.0, 0.10});

    std::vector<double> maxVelVec = get_parameter("maxVel").as_double_array();
    std::vector<double> maxAccVec = get_parameter("maxAcc").as_double_array();

    // Clamp values to physical limits
    for (size_t i = 0; i < 4; ++i)
    {
      if (maxVelVec[i] > k_velocityPhysicalLimit[i])
      {
        m_logger->warn("[lifecycle] [RobotNode] maxVel[{}] = {:.4f} exceeds physical limit {:.4f}, clamping to limit",
                       i, maxVelVec[i], k_velocityPhysicalLimit[i]);
        maxVelVec[i] = k_velocityPhysicalLimit[i];
      }

      if (maxAccVec[i] > k_accelerationPhysicalLimit[i])
      {
        m_logger->warn("[lifecycle] [RobotNode] maxAcc[{}] = {:.4f} exceeds physical limit {:.4f}, clamping to limit",
                       i, maxAccVec[i], k_accelerationPhysicalLimit[i]);
        maxAccVec[i] = k_accelerationPhysicalLimit[i];
      }
    }

    // Copy to the blaze::StaticVector members
    for (size_t i = 0; i < 4; ++i)
    {
      m_maxVel[i] = maxVelVec[i];
      m_maxAcc[i] = maxAccVec[i];
    }

    m_logger->info("[lifecycle] [RobotNode] Max Velocities set to: [{}, {}, {}, {}]",
                   m_maxVel[0], m_maxVel[1], m_maxVel[2], m_maxVel[3]);
    declare_parameter<double>("Kp", 30.0);
    declare_parameter<double>("Ki", 5.0);
    m_kp = get_parameter("Kp").as_double();
    m_ki = get_parameter("Ki").as_double();

    RCLCPP_INFO(get_logger(),
                "[lifecycle] max accel/vel (wire [a1,b1,a2,b2]): maxVel=[%.4f, %.4f, %.4f, %.4f] "
                "maxAcc=[%.4f, %.4f, %.4f, %.4f] Kp=%.3f Ki=%.3f",
                m_maxVel[0], m_maxVel[1], m_maxVel[2], m_maxVel[3],
                m_maxAcc[0], m_maxAcc[1], m_maxAcc[2], m_maxAcc[3], m_kp, m_ki);
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
    auto command_sample_time = std::chrono::milliseconds(static_cast<int>(10));
    m_control_loop_timer = create_wall_timer(command_sample_time, std::bind(&RobotNode::targetCommand_timerCallback, this), m_cbGroup1);
    // Timer to read joint configurations periodically
    m_joints_config_timer = create_wall_timer(10ms, std::bind(&RobotNode::jointsConfig_timerCallback, this), m_cbGroup3);
    // Timer to read robot status periodically
    m_read_robot_timer = create_wall_timer(10ms, std::bind(&RobotNode::robotStatus_timerCallback, this), m_cbGroup3);
    // Initialize a timer to check emtracker lifecycle
    m_watchdog_timer_emt = create_wall_timer(100ms, std::bind(&RobotNode::check_emtracker_alive_timerCallback, this), m_callback_group_watchdog_1);
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
    if (!m_robot->isConnected())
      return; // hardware not (yet) up
    auto msg = interfaces::msg::Status();

    blaze::StaticVector<bool, 4UL> reached_status;
    blaze::StaticVector<bool, 4UL> encoder_status;
    blaze::StaticVector<bool, 4UL> en_status;
    blaze::StaticVector<bool, 4UL> en_fault_status;
    en_status = m_robot->getEnableStatus();
    en_fault_status = m_robot->getEnableFaultStatus();
    encoder_status = m_robot->getEncoderStatus();
    reached_status = m_robot->getReachedStatus();
    m_robot->getPosLimit(m_minCurrentPosLimit, m_maxCurrentPosLimit);

    // Edge-triggered: this callback runs on a timer, so level-triggered
    // logging would bury the log under a repeat every period.
    for (int i = 0; i < 4; i++)
    {
      if (en_fault_status[i] && !m_enable_fault_prev[i])
        RCLCPP_ERROR(get_logger(),
                     "[procedure] Joint %d (%s) failed the CiA-402 enable transition - that joint is NOT under control",
                     i, k_joint_names[i]);
      else if (!en_fault_status[i] && m_enable_fault_prev[i])
        RCLCPP_INFO(get_logger(), "[procedure] Joint %d (%s) CiA-402 enable fault cleared", i, k_joint_names[i]);
      m_enable_fault_prev[i] = en_fault_status[i];
    }

    if (m_mode == CtrlMode::Position)
    {
      blaze::StaticVector<double, 4UL> x_des_snapshot;
      {
        std::lock_guard<std::mutex> lock(m_x_des_log_mutex);
        x_des_snapshot = m_x_des_log_snapshot;
      }
      const bool all_reached = reached_status[0] && reached_status[1] && reached_status[2] && reached_status[3];
      if (all_reached != m_reached_prev)
      {
        RCLCPP_INFO(get_logger(),
                    "[procedure] reached status changed to %s (wire [a1,b1,a2,b2]): x_des=[%.5f, %.5f, %.5f, %.5f] x=[%.5f, %.5f, %.5f, %.5f]",
                    all_reached ? "REACHED" : "NOT REACHED",
                    x_des_snapshot[0], x_des_snapshot[1], x_des_snapshot[2], x_des_snapshot[3],
                    m_x[0], m_x[1], m_x[2], m_x[3]);
        m_reached_prev = all_reached;
        m_not_reached_since = all_reached ? std::chrono::steady_clock::time_point{} : std::chrono::steady_clock::now();
        m_not_reached_target = x_des_snapshot;
      }
      else if (!all_reached && x_des_snapshot == m_not_reached_target)
      {
        const auto stuck_for = std::chrono::steady_clock::now() - m_not_reached_since;
        if (stuck_for > std::chrono::seconds(10))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "[procedure] target not reached for %.1f s (wire [a1,b1,a2,b2]): x_des=[%.5f, %.5f, %.5f, %.5f] x=[%.5f, %.5f, %.5f, %.5f]",
                               std::chrono::duration<double>(stuck_for).count(),
                               x_des_snapshot[0], x_des_snapshot[1], x_des_snapshot[2], x_des_snapshot[3],
                               m_x[0], m_x[1], m_x[2], m_x[3]);
        }
      }
      else if (!all_reached)
      {
        // target changed while still not reached - restart the not-reached timer against the new target
        m_not_reached_since = std::chrono::steady_clock::now();
        m_not_reached_target = x_des_snapshot;
      }
    }

    if ((abs(m_x[1] - k_pos_preEngage[1]) < 0.001) && (abs(m_x[3] - k_pos_preEngage[3]) < 0.001))
      m_flag_readyToEngage = true;
    else
      m_flag_readyToEngage = false;
    if ((abs(m_x[1] - k_pos_engage[1]) < 0.003) && (abs(m_x[3] - k_pos_engage[3]) < 0.002))
      m_flagEngaged = true;
    else
      m_flagEngaged = false;

    m_head_attached = m_digital_input[3][18];

    for (int i = 0; i < 4; i++)
    {
      msg.enable[i] = en_status[i];
      msg.enable_fault[i] = en_fault_status[i];
      // msg.encoder[i] = encoder_status[i];
      msg.encoder[i] = m_encoders_set[i];
      msg.reached[i] = reached_status[i];
      msg.min_pos_limit[i] = m_minCurrentPosLimit[i];
      msg.max_pos_limit[i] = m_maxCurrentPosLimit[i];
      msg.cpu_temp[i] = m_cpu_temp[i];
      msg.winding_temp[i] = m_winding_temp[i];
    }

    msg.head_attached = m_head_attached;
    msg.trans_limit_en = m_trans_limit;
    msg.control_mode = static_cast<int>(m_mode);

    msg.ready_to_engage = m_flag_readyToEngage;
    msg.engaged = m_flagEngaged;
    msg.locked = m_locked;

    msg.procedure = m_procedure;

    m_publisher_status->publish(msg);

    // update interface
    auto interface_msg = interfaces::msg::Interface();
    m_interface_key[0] = m_digital_input[0][17];
    m_interface_key[1] = m_digital_input[0][18];
    m_interface_key[2] = m_digital_input[1][17];
    m_interface_key[3] = m_digital_input[1][18];
    m_interface_key[4] = m_digital_input[2][17];
    m_interface_key[5] = m_digital_input[2][18];
    m_interface_key[6] = m_digital_input[3][17];
    for (int i = 0; i < 7; i++)
    {
      interface_msg.interface_key[i] = m_interface_key[i];
    }
    m_publisher_interface->publish(interface_msg);
  }

  // Timer callback function to read the current joint configurations and publish them
  void jointsConfig_timerCallback()
  {
    if (!m_robot->isConnected())
      return; // hardware not (yet) up
    auto msg = interfaces::msg::Jointspace();
    m_robot->getPos(m_x);
    m_robot->getVel(m_xdot);
    m_current = m_robot->getCurrent();

    m_robot->getTemperature(m_cpu_temp, m_winding_temp);
    m_robot->getDigitalIn(m_digital_input);

    minDynamicPosLimit[0] = std::max(k_minStaticLimitAll[0], m_x[2] - k_rotary_stage_max_clearance);
    maxDynamicPosLimit[0] = std::min(k_maxStaticLimitAll[0], m_x[2] - k_rotary_stage_min_clearance);
    minDynamicPosLimit[2] = std::max(k_minStaticLimitAll[2], m_x[0] + k_rotary_stage_min_clearance);
    maxDynamicPosLimit[2] = std::min(k_maxStaticLimitAll[2], m_x[0] + k_rotary_stage_max_clearance);

    minDynamicPosLimit[1] = std::max(k_minStaticLimitAll[1], m_x[3] - k_linear_stage_max_clearance);
    maxDynamicPosLimit[1] = std::min(k_maxStaticLimitAll[1], m_x[3] - k_linear_stage_min_clearance);
    minDynamicPosLimit[3] = std::max(k_minStaticLimitAll[3], m_x[1] + k_linear_stage_min_clearance);
    maxDynamicPosLimit[3] = std::min(k_maxStaticLimitAll[3], m_x[1] + k_linear_stage_max_clearance);

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
      m_x_error_int = m_x_error_int + m_x_error * k_sample_time * 1e-3;
      q_dot_command = m_x_error * m_kp + m_x_error_int * m_ki + x_dot_des;

      // joint_space_control_step(x_des, x_dot_des, q_dot_command);
      // Set_Target_Velocity(q_dot_command);
    }
  }

  // Set the target position/velocity in the robot - Depreciated
  void targetCommand_timerCallback()
  {
    if (!m_robot->isConnected())
      return; // hardware not (yet) up
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                          "[cmd] control loop: mode=%s x_des=[%.5f, %.5f, %.5f, %.5f] "
                          "limits=[%.5f, %.5f, %.5f, %.5f]..[%.5f, %.5f, %.5f, %.5f] trans_limit=%d",
                          ctrlModeName(m_mode), m_x_des[0], m_x_des[1], m_x_des[2], m_x_des[3],
                          minDynamicPosLimit[0], minDynamicPosLimit[1], minDynamicPosLimit[2], minDynamicPosLimit[3],
                          maxDynamicPosLimit[0], maxDynamicPosLimit[1], maxDynamicPosLimit[2], maxDynamicPosLimit[3],
                          m_trans_limit);
    switch (m_mode)
    {
    case CtrlMode::Manual:
      m_robot->setTargetVel(m_xdot_manual);
      break;
    case CtrlMode::Velocity:
      m_robot->setTargetVel(m_xdot_des);
      break;
    case CtrlMode::Position:
      m_robot->setTargetPos(m_x_des);
      // std::cout << "m_x_des sent: " << blaze::trans(m_x_des) << std::endl;
      break;
    }
    if (m_trans_limit)
    {
      m_robot->setPosLimit(minDynamicPosLimit, maxDynamicPosLimit);
    }
    else
    {
      m_robot->setPosLimit(minDynamicPosLimitInf, maxDynamicPosLimitInf);
    }
  }

  // Subscription callback function to updates the target joint positions and velocities
  void jointSpaceTarget_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    if (m_last_target_rx.time_since_epoch().count() != 0 && now - m_last_target_rx > std::chrono::seconds(1))
    {
      RCLCPP_INFO(get_logger(), "[cmd] joint_space/target resumed after %.3f s of silence",
                  std::chrono::duration<double>(now - m_last_target_rx).count());
    }
    m_last_target_rx = now;

    RCLCPP_DEBUG(get_logger(),
                 "[cmd] target received (wire [a1,b1,a2,b2]): pos=[%.5f, %.5f, %.5f, %.5f] vel=[%.5f, %.5f, %.5f, %.5f]",
                 msg->position[0UL], msg->position[1UL], msg->position[2UL], msg->position[3UL],
                 msg->velocity[0UL], msg->velocity[1UL], msg->velocity[2UL], msg->velocity[3UL]);

    if (!m_flag_manual && !m_flag_use_target_action)
    {
      // m_targpublisher_alive_tmep = true;
      m_xdot_des = blaze::StaticVector<double, 4UL>(0.00);
      m_x_des = m_x;

      switch (m_mode)
      {
      case CtrlMode::Velocity:
        m_xdot_des[0UL] = msg->velocity[0UL];
        m_xdot_des[1UL] = msg->velocity[1UL];
        m_xdot_des[2UL] = msg->velocity[2UL];
        m_xdot_des[3UL] = msg->velocity[3UL];
        break;
      case CtrlMode::Position:
        m_x_des[0UL] = msg->position[0UL];
        m_x_des[1UL] = msg->position[1UL];
        m_x_des[2UL] = msg->position[2UL];
        m_x_des[3UL] = msg->position[3UL];
        if (m_trans_limit)
        {
          std::ostringstream violations;
          violations << std::fixed << std::setprecision(5);
          for (int i = 0; i < 4; ++i)
          {
            if (m_x_des[i] < minDynamicPosLimit[i] || m_x_des[i] > maxDynamicPosLimit[i])
            {
              if (violations.tellp() > 0)
                violations << "; ";
              violations << k_joint_names[i] << "=" << m_x_des[i]
                         << " [" << minDynamicPosLimit[i] << ", " << maxDynamicPosLimit[i] << "]";
            }
          }
          if (violations.tellp() > 0)
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[cmd] target outside dynamic limits - drive will clamp: %s",
                                 violations.str().c_str());
          }
        }
        break;
      default:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[cmd] target dropped: control mode is %s (not Position/Velocity)",
                             ctrlModeName(m_mode));
        break;
      }
      {
        std::lock_guard<std::mutex> lock(m_x_des_log_mutex);
        m_x_des_log_snapshot = m_x_des;
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[cmd] target dropped: m_flag_manual=%d m_flag_use_target_action=%d",
                           m_flag_manual, m_flag_use_target_action);
    }
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
  void enableService_callback(const std::shared_ptr<interfaces::srv::Config::Request> request, std::shared_ptr<interfaces::srv::Config::Response> response)
  {
    RCLCPP_INFO(get_logger(), "[comm] robot_enable request: command='%s' value=%lld", request->command.c_str(), static_cast<long long>(request->value));
    if (request->command == "toggleEnable")
    {
      m_cancel_flag = true; // Signal cancellation to any running task
      // auto toggle_thread_ = std::thread(&RobotNode::toggleEnable, this);
      // toggle_thread_.join();
      RobotNode::toggleEnable();
      response->success = true;
      response->message = "Toggled enable";
    }
    else if (request->command == "disable")
    {
      m_cancel_flag = true; // Signal cancellation to any running task
      // auto toggle_thread_ = std::thread(&RobotNode::disable, this);
      // toggle_thread_.join();
      RobotNode::disable();
      response->success = true;
      response->message = "Disabled";
    }
    else
    {
      response->success = false;
      response->message = "Invalid command";
      RCLCPP_WARN(get_logger(), "[comm] robot_enable rejected: command='%s' is not recognised", request->command.c_str());
    }
    RCLCPP_INFO(get_logger(), "[comm] robot_enable response: success=%d message='%s'", response->success, response->message.c_str());
  }

  // Service callback to triger tasks, enable, and control mode section
  void configService_callback(const std::shared_ptr<interfaces::srv::Config::Request> request, std::shared_ptr<interfaces::srv::Config::Response> response)
  {
    std::future<std::string> result_future;

    RCLCPP_INFO(get_logger(), "[comm] robot_config request: command='%s' value=%lld", request->command.c_str(), static_cast<long long>(request->value));
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
      RCLCPP_INFO(get_logger(), "[comm] robot_config '%s' applied synchronously (no response payload)", request->command.c_str());
      return;
    }
    else if (request->command == "setTransLimMode")
    {
      setTranLim(request->value);
      RCLCPP_INFO(get_logger(), "[comm] robot_config '%s' applied synchronously (no response payload)", request->command.c_str());
      return;
    }
    else if (request->command == "findLinearHome")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return findLinearHome(); });
    }
    else if (request->command == "engageCollets")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return engageCollets(); });
    }
    else if (request->command == "disengageCollets")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return disengageCollets(); });
    }
    else if (request->command == "lockCollets")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return lockCollets(); });
    }
    else if (request->command == "unlockCollets")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return unlockCollets(); });
    }
    else if (request->command == "findRotaryHome")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return findRotaryHome(); });
    }
    else if (request->command == "goHome")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return goHome(); });
    }
    else if (request->command == "findRotaryHomeAndGoHome")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return findRotaryHomeAndGoHome(); });
    }
    else if (request->command == "startProcedure")
    {
      startProcedure();
    }
    else if (request->command == "endProcedure")
    {
      endProcedure();
    }
    else if (request->command == "engageAndUnlock")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return engageUnlock(); });
    }
    else if (request->command == "lockAndDisengage")
    {
      result_future = dispatchTask(request->command, [this]()
                                   { return lockDisengage(); });
    }
    else
    {
      response->success = false;
      response->message = "Invalid command";
      RCLCPP_WARN(get_logger(), "[comm] robot_config rejected: command='%s' is not recognised", request->command.c_str());
      return;
    }
    response->success = true;
    response->message = "Task received";
    RCLCPP_INFO(get_logger(), "[comm] robot_config response: success=%d message='%s'", response->success, response->message.c_str());
  }

  // Handle the service response future
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
  std::future<std::string> dispatchTask(const std::string &name, std::function<std::string()> task)
  {
    std::lock_guard<std::mutex> lock(m_task_mutex);
    m_cancel_flag = true; // Signal cancellation to any running task
    // Wait a moment for running task to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Promise for task results
    auto promise = std::make_shared<std::promise<std::string>>();
    std::future<std::string> results_future = promise->get_future();

    RCLCPP_INFO(get_logger(), "[procedure] task '%s' dispatched", name.c_str());
    const auto dispatch_time = std::chrono::steady_clock::now();

    m_current_task = [this, task, promise, name, dispatch_time]()
    {
      std::string result = task(); // Execuse the task
      const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - dispatch_time)
                                    .count();
      RCLCPP_INFO(get_logger(), "[procedure] task '%s' completed: result='%s' (%ld ms)",
                  name.c_str(), result.c_str(), duration_ms);
      promise->set_value(result); // return result
    };

    m_flag_task_ready = true;
    m_task_cv.notify_one();

    return results_future;
  }

  // =============================================== Robot Config/Setup Tasks ====================================== //
  // set robot control mode to config mode
  int switchToConfigMode()
  {
    if (m_procedure)
    {
      m_logger->info("[RobotNode] Config mode is not allowed once procedure started");
      return 1;
    }
    m_robot->setTargetVel({0.0, 0.0, 0.0, 0.0});
    m_trans_limit = false; // disable translation limits
    m_mode = CtrlMode::Config;
    m_logger->info("[RobotNode] Selected control mode: Config");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
  }

  // set robot control mode
  void setCtrlMode(const CtrlMode mode)
  {
    const CtrlMode prev_mode = m_mode;

    if (mode == CtrlMode::Config)
    {
      switchToConfigMode();
    }
    else if (mode == CtrlMode::Manual)
    {
      m_robot->setTargetVel({0.0, 0.0, 0.0, 0.0});
      blaze::StaticVector<double, 4UL> negative = {1000.0, 1000.0, 1000.0, 1000.0};
      blaze::StaticVector<double, 4UL> positive = {1000.0, 1000.0, 1000.0, 1000.0};
      m_robot->setOperationMode(OpMode::VelocityProfile);
      m_robot->setProfileParams(m_maxVel, m_maxAcc, m_maxAcc);
      m_robot->setMaxTorque(negative, positive);
      m_mode = CtrlMode::Manual;
      m_logger->info("[RobotNode] Selected Mode: Manual");
    }
    else if (mode == CtrlMode::Position)
    {
      m_robot->setTargetVel({0.0, 0.0, 0.0, 0.0});
      m_x_des = m_x;
      m_trans_limit = true; // enable translation limits
      m_robot->setOperationMode(OpMode::PositionProfile);
      m_robot->setProfileParams(m_maxVel, m_maxAcc, m_maxAcc);
      m_mode = CtrlMode::Position;
      m_logger->info("[RobotNode] Selected Mode: Position");
      m_procedure = true;
      m_logger->info("[RobotNode] Procedure Started");
    }
    else if (mode == CtrlMode::Velocity)
    {
      m_robot->setTargetVel({0.0, 0.0, 0.0, 0.0});
      m_trans_limit = true; // enable translation limits
      m_robot->setOperationMode(OpMode::VelocityProfile);
      m_robot->setProfileParams(m_maxVel, m_maxAcc, m_maxAcc);
      m_mode = CtrlMode::Velocity;
      m_logger->info("[RobotNode] Selected Mode: Velocity");
      m_procedure = true;
      m_logger->info("[RobotNode] Procedure Started");
    }
    RCLCPP_INFO(get_logger(), "[procedure] control mode: %s -> %s (procedure=%d)",
                ctrlModeName(prev_mode), ctrlModeName(m_mode), m_procedure);
  }

  // enable/disable hardcoded translation limits for stages
  void setTranLim(int enable)
  {
    if (m_mode == CtrlMode::Manual)
    {
      if (enable)
      {
        m_trans_limit = true;
        m_logger->info("[RobotNode] Joints Position Limits ON");
      }
      else
      {
        m_trans_limit = false;
        m_logger->info("[RobotNode] Joints Position Limits OFF");
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
    blaze::StaticVector<bool, 4UL> en_status = m_robot->getEnableStatus();
    const bool was_enabled = en_status[0] || en_status[1] || en_status[2] || en_status[3];
    if (was_enabled)
    {
      m_robot->enableOperation(false);
      endProcedure();
    }
    else
    {
      m_robot->enableOperation(true);
    }
    RCLCPP_INFO(get_logger(), "[procedure] toggleEnable: enabled %d -> %d (mode=%s, procedure=%d)",
                was_enabled, !was_enabled, ctrlModeName(m_mode), m_procedure);
  }

  // enable or distable the robot
  void disable()
  {
    m_robot->enableOperation(false);
    endProcedure();
    RCLCPP_INFO(get_logger(), "[procedure] disable: operation disabled (mode=%s, procedure=%d)",
                ctrlModeName(m_mode), m_procedure);
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
    if (switchToConfigMode())
      return "canceled";
    m_robot->setOperationMode(OpMode::VelocityProfile);
    m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);
    // move linear joints with constant velocity
    m_robot->setTargetVel(target_vel);
    m_robot->enableOperation(true);
    // monitor linear joint currents to detect mechanical limits
    while (!flag_inr_encoder_set || !flag_mdl_encoder_set)
    {
      if (check_cancel())
        return "canceled";
      current = m_robot->getCurrent();
      if (current[1] <= inr_thresh && !flag_inr_encoder_set)
      {
        m_logger->info("[RobotNode] inner carriage hit mechanical limit");
        target_vel[1] = 0.0;
        m_robot->setTargetVel(target_vel);
        flag_inr_encoder_set = true;
      }
      if (current[3] <= mdl_thresh && !flag_mdl_encoder_set)
      {
        m_logger->info("[RobotNode] middle carriage hit mechanical limit");
        target_vel[3] = 0.0;
        m_robot->setTargetVel(target_vel);
        flag_mdl_encoder_set = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // rotate rotary joints to make sure couplings are engaged
    m_robot->setTargetVel({2.0, 0.0, 2.0, 0.0});
    sleep_thread_cancelable(4000);
    if (check_cancel())
      return "canceled";
    m_robot->setTargetVel({0.0, 0.0, 0.0, 0.0});
    // set encoders
    m_robot->setEncoders({0.0, k_pos_inr_prox_stop, 0.0, k_pos_mdl_prox_stop});
    sleep_thread_cancelable(500);
    if (check_cancel())
      return "canceled";
    m_robot->enableOperation(false);
    m_robot->setTargetPos(m_x);
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
    if (switchToConfigMode())
      return "canceled";
    m_robot->setOperationMode(OpMode::PositionProfile);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);
    m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
    m_robot->enableOperation(true);
    if (check_cancel())
      return "canceled";
    m_robot->setTargetPos({m_x[0], k_pos_preEngage[1], m_x[2], k_pos_preEngage[3]});
    m_robot->waitUntilTransReach(m_cancel_flag);
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
    if (switchToConfigMode())
      return "canceled";
    m_robot->setOperationMode(OpMode::PositionProfile);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);

    // if the stages are not in pre-engae mode, move to pre-engage mode first
    if (!m_flag_readyToEngage)
    {
      m_logger->info("[RobotNode] Moving to pre-engage location");
      m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
      m_robot->enableOperation(true);
      targetPosTemp = {m_x[0], k_pos_preEngage[1], m_x[2], k_pos_preEngage[3]};
      m_robot->setTargetPos(targetPosTemp);
      m_robot->waitUntilTransReach(m_cancel_flag);
      if (check_cancel())
        return "canceled";
    }

    // if the stages are in pre-engae mode, engage collets
    if (m_flag_readyToEngage)
    {
      maxTorqueNegative = {350.0, 250.0, 350.0, 250.0};
      maxTorquePositive = {350.0, 250.0, 350.0, 250.0};
      m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
      m_robot->enableOperation(true);

      // attemp to engage middle collet - step one - setting translational joint
      m_logger->info("[RobotNode] Engaging middle collet");
      targetPosTemp = {m_x[0], k_pos_preEngage[1], m_x[2], k_pos_engage[3]};
      m_robot->setTargetPos(targetPosTemp);
      sleep_thread_cancelable(1500);
      if (check_cancel())
        return "canceled";
      // if it doesn't each the target, back of, rotate, and try again.
      while (!m_robot->getReachedStatus()[3])
      {
        if (check_cancel())
          return "canceled";
        targetPosTemp[3] = k_pos_engage[3] - 0.004;
        m_robot->setTargetPos(targetPosTemp); // back off for 4 mm
        sleep_thread_cancelable(1000);
        targetPosTemp[2] = m_x[2] + 0.07 * M_PI;
        m_robot->setTargetPos(targetPosTemp); // rotate
        sleep_thread_cancelable(1000);
        targetPosTemp[2] = m_x[2];
        targetPosTemp[3] = k_pos_engage[3];
        m_robot->setTargetPos(targetPosTemp); // attemp to engage middle collet again
        sleep_thread_cancelable(1500);
      }
      targetPosTemp = m_x;
      m_robot->setTargetPos(targetPosTemp);
      m_logger->debug("Engaing middle coller step one done");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // attemp to engage middle collet - step two - slightly locking the collet so always continue from same config
      targetPosTemp[2] = targetPosTemp[2] + 20.0 * M_PI; // rotate collet to lock
      m_robot->setTargetPos(targetPosTemp);
      // monitor current
      while (!(m_current[2] > (maxTorquePositive[2] - current_thresh)))
      {
        if (check_cancel())
          return "canceled";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      m_robot->setTargetPos(m_x);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      m_logger->info("[RobotNode] Middle collet engaged");

      // attemp to engage inner collet - step one - setting translational joint
      m_logger->info("[RobotNode] Engaging the inner collet");
      targetPosTemp = {m_x[0], k_pos_engage[1], m_x[2], k_pos_engage[3]};
      m_robot->setTargetPos(targetPosTemp);
      sleep_thread_cancelable(2000);
      if (check_cancel())
        return "canceled";
      // if it doesn't each the target, back of, rotate, and try again.
      while (!m_robot->getReachedStatus()[1])
      {
        if (check_cancel())
          return "canceled";
        targetPosTemp[1] = k_pos_engage[1] - 0.004;
        m_robot->setTargetPos(targetPosTemp);
        sleep_thread_cancelable(1000);
        targetPosTemp[0] = m_x[0] + 0.07 * M_PI;
        m_robot->setTargetPos(targetPosTemp);
        sleep_thread_cancelable(1000);
        targetPosTemp[0] = m_x[0];
        targetPosTemp[1] = k_pos_engage[1];
        m_robot->setTargetPos(targetPosTemp);
        sleep_thread_cancelable(1500);
      }
      targetPosTemp = m_x;
      m_robot->setTargetPos(targetPosTemp);
      m_logger->debug("Engaing inner coller step one done");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // attemp to engage middle collet - step two - slightly locking the collet so always continue from same config
      targetPosTemp = m_x;
      targetPosTemp[0] = targetPosTemp[0] + 20.0 * M_PI;
      m_robot->setTargetPos(targetPosTemp);
      while (!(m_current[0] > (maxTorquePositive[0] - current_thresh)))
      {
        if (check_cancel())
          return "canceled";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      m_robot->setTargetPos(m_x);
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
    if (switchToConfigMode())
      return "canceled";
    m_robot->setOperationMode(OpMode::PositionProfile);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);
    m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
    m_robot->enableOperation(true);
    m_robot->setTargetPos(m_x + motion);
    sleep_thread_cancelable(500);
    if (check_cancel())
      return "canceled";
    reach = m_robot->getReachedStatus();
    while (!(m_current[0] > inr_thresh) || !reach[1] || !(m_current[2] > mdl_thresh) || !reach[3])
    {
      if (check_cancel())
        return "canceled";
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    m_robot->setTargetPos(m_x);
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
    if (switchToConfigMode())
      return "canceled";
    m_robot->setOperationMode(OpMode::PositionProfile);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);
    m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
    m_robot->enableOperation(true);
    m_robot->setTargetPos(m_x + motion);
    sleep_thread_cancelable(500);
    reach = m_robot->getReachedStatus();
    while (!reach[0] || !reach[1] || !reach[2] || !reach[3])
    {
      if (check_cancel())
        return "canceled";
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
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
    if (switchToConfigMode())
      return "canceled";
    m_robot->setOperationMode(OpMode::PositionProfile);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);
    m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
    m_robot->enableOperation(true);

    // moving to pre-engage postion. This is the most extended positon of the tubes
    targetPosTemp = {m_x[0], k_pos_preEngage[1], m_x[2], k_pos_preEngage[3]};
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    // rotate the middle tube for 2PI while recoding tip Y position
    m_logger->info("[RobotNode] Middle tube coarse rotation...");
    targetPosTemp = {m_x[0], k_pos_preEngage[1], m_x[2] + 2 * M_PI, k_pos_preEngage[3]};
    m_robot->setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = m_robot->getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[2], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    // find the config leading to minimum Y and move to that config
    find_min(data, best_input, min_output);
    m_logger->debug("[RobotNode] Minimum Y for middle tube: q = {:.2f} | tip y = {:.5f}", best_input, min_output);
    targetPosTemp[2] = best_input;
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    // rotate the inner tube for 2PI while recoding tip Y position
    m_logger->info("[RobotNode] Inner tube coarse rotation...");
    targetPosTemp = {m_x[0] + 2 * M_PI, k_pos_preEngage[1], m_x[2], k_pos_preEngage[3]};
    m_robot->setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = m_robot->getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[0], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    // find the config leading to minimum Y and move to that config
    find_min(data, best_input, min_output);
    m_logger->debug("[RobotNode] Minimum Y for inner tube: q = {:.2f} | tip y = {:.5f}", best_input, min_output);
    targetPosTemp[0] = best_input;
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";

    // now repeat the process for double sided shorter range of motion for fine tuning
    // first for the inner tube
    targetPosTemp[0] += M_PI / 2;
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    // moving in negative direction and recording tip position
    m_logger->info("[RobotNode] Inner tube fine positive rotation...");
    targetPosTemp[0] -= M_PI;
    m_robot->setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = m_robot->getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[0], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    find_min(data, best_input, min_output);
    double input = best_input;
    // moving in positive direction and recording tip position
    m_logger->info("[RobotNode] Inner tube fine negative rotation...");
    targetPosTemp[0] += M_PI;
    m_robot->setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = m_robot->getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[0], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    find_min(data, best_input, min_output);
    input += best_input;
    input /= 2; // computing the averagte of minimum config for both sides
    m_logger->debug("[RobotNode] Minimum Y for inner tube: q = {:.2f}", input);
    // computing the averagte of minimum config for both sides
    targetPosTemp[0] = input;
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    m_logger->info("[RobotNode] Inner tube is in minimum Y posoition");

    // then for the middle tube
    m_logger->debug("[RobotNode] Round 2 - repeat for fine tunnig");
    targetPosTemp[2] += M_PI / 3;
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    // moving in negative direction and recording tip position
    m_logger->info("[RobotNode] Middle tube fine negative rotation...");
    targetPosTemp[2] -= 2 * M_PI / 3;
    m_robot->setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = m_robot->getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[2], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    find_min(data, best_input, min_output);
    input = best_input;
    // moving in positive direction and recording tip position
    m_logger->info("[RobotNode] Middle tube fine positive rotation...");
    targetPosTemp[2] += 2 * M_PI / 3;
    m_robot->setTargetPos(targetPosTemp);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reach = m_robot->getReachedStatus();
    data.clear();
    while (!(reach[0] && reach[1] && reach[2] && reach[3]))
    {
      if (check_cancel())
        return "canceled";
      data.push_back({m_x[2], m_x_tip[1]});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      reach = m_robot->getReachedStatus();
    }
    find_min(data, best_input, min_output);
    input += best_input;
    input /= 2; // computing the averagte of minimum config for both sides
    m_logger->debug("[RobotNode] Minimum Y for inner tube: q = {:.2f}", input);
    // computing the averagte of minimum config for both sides
    targetPosTemp[2] = input;
    m_robot->setTargetPos(targetPosTemp);
    m_robot->waitUntilReach(m_cancel_flag);
    if (check_cancel())
      return "canceled";
    m_logger->info("[RobotNode] Middle tube is in minimum Y posoition");

    m_robot->setEncoders({0.0, m_x[1], 0.0, m_x[3]});
    sleep_thread_cancelable(500);
    if (check_cancel())
      return "canceled";
    m_robot->enableOperation(false);
    m_robot->setTargetPos(m_x);
    m_encoders_set[0] = true;
    m_encoders_set[2] = true;
    m_logger->info("[RobotNode] Rotary joints encoders found");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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

    m_logger->info("[RobotNode] Going to Home...");
    constexpr blaze::StaticVector<double, 4UL> maxTorqueNegative = {400.0, 400.0, 400.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxTorquePositive = {400.0, 400.0, 400.0, 400.0};
    constexpr blaze::StaticVector<double, 4UL> maxAcc = {50.00 * M_PI / 180.00, 5.00 / 1000.00, 50.00 * M_PI / 180.00, 5.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
    constexpr blaze::StaticVector<double, 4UL> maxDcc = maxAcc;
    constexpr blaze::StaticVector<double, 4UL> maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]

    switchToConfigMode();
    // m_trans_limit = true; // enable translation limits
    m_robot->setOperationMode(OpMode::PositionProfile);
    m_robot->setProfileParams(maxVel, maxAcc, maxDcc);
    m_robot->setMaxTorque(maxTorqueNegative, maxTorquePositive);
    m_robot->enableOperation(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    m_robot->setTargetPos(k_home_pos + k_home_pos_margin);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    m_robot->waitUntilReach(m_cancel_flag);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    m_logger->info("[RobotNode] Triggering homing");
    goHome();

    std::string res;
    res = "OK";
    return res;
  }

  // set the m_procedure flag to false
  std::string startProcedure()
  {
    m_logger->info("[procedure] [RobotNode] Setting control mode to Position Control");
    setCtrlMode(static_cast<CtrlMode>(CtrlMode::Position));
    std::string res;
    res = "OK";
    return res;
  }

  // set the m_procedure flag to false
  void endProcedure()
  {
    const bool was_running = m_procedure;
    m_procedure = false;
    m_logger->info("[procedure] [RobotNode] Procedure Ended (procedure {} -> false, mode={})",
                   was_running, ctrlModeName(m_mode));
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

  // tube related constants - may need to be adjusted if the tube set changes
  static constexpr double k_inr_active_length = 0.216; // need to be adjusted based on the tube set
  static constexpr double k_mdl_active_length = 0.132; // need to be adjusted based on the tube set
  static constexpr double k_otr_active_length = 0.060; // need to be adjusted based on the tube set

  static constexpr blaze::StaticVector<double, 4> minDynamicPosLimitInf = {-10 * M_PI, -0.50, -10 * M_PI, -0.50}; // for when the limit is off
  static constexpr blaze::StaticVector<double, 4> maxDynamicPosLimitInf = {10 * M_PI, 0.50, 10 * M_PI, 0.50};     // for when the limit is off
  blaze::StaticVector<double, 4> minDynamicPosLimit = {-2.0 * M_PI, 0.001, -1.5 * M_PI, 0.040};                     // for when the limit is on
  blaze::StaticVector<double, 4> maxDynamicPosLimit = {2.0 * M_PI, 0.088, 1.5 * M_PI, 0.129};                       // for when the limit is on

  // robot mechanics related constants -  may need to be adjusted if the robot design changes
  static constexpr blaze::StaticVector<double, 4UL> k_velocityPhysicalLimit = {3.0, 0.0125, 3.0, 0.0125};
  static constexpr blaze::StaticVector<double, 4UL> k_accelerationPhysicalLimit = {10.0, 0.10, 10.0, 0.10};
  static constexpr blaze::StaticVector<double, 4> k_pos_preEngage = {0.0, -0.0640, 0.0, -0.0340};
  static constexpr blaze::StaticVector<double, 4> k_pos_engage = {0.0, -0.0560, 0.0, -0.0290};
  static constexpr double k_pos_inr_prox_stop = -0.1670;
  // static constexpr double k_pos_mdl_prox_stop = -0.1230; // old design
  static constexpr double k_pos_mdl_prox_stop = -0.1130;
  static constexpr double k_linear_stage_min_clearance = 0.030;

  // driven constants
  static constexpr double k_linear_stage_max_clearance = k_inr_active_length - k_mdl_active_length;
  static constexpr double k_rotary_stage_min_clearance = -2.0 * M_PI; // relative limit between the rotary joints
  static constexpr double k_rotary_stage_max_clearance = 2.0 * M_PI;  // relative limit between the rotary joints
  static constexpr blaze::StaticVector<double, 4> k_home_pos_margin = {0.0, 0.0002, 0.0, 0.0001};
  static constexpr blaze::StaticVector<double, 4> k_home_pos = {0.0, k_otr_active_length - k_inr_active_length, 0.0, k_otr_active_length - k_mdl_active_length};
  static constexpr blaze::StaticVector<double, 4> k_minStaticLimitAll = {-3 * M_PI, k_home_pos[1], -1.5 * M_PI, k_home_pos[3]};
  static constexpr blaze::StaticVector<double, 4> k_maxStaticLimitAll = {3 * M_PI, k_pos_preEngage[1], 1.5 * M_PI, k_pos_preEngage[3]};

  // constant
  static constexpr int k_sample_time = 20; // [ms]
  // wire order [alpha1, beta1, alpha2, beta2]; used for operator-facing messages
  static constexpr const char *k_joint_names[4] = {
      "inner tube rotation", "inner tube translation",
      "middle tube rotation", "middle tube translation"};

  // member variables
  std::atomic<bool> m_stop_worker{false};
  std::array<bool, 4> m_enable_fault_prev = {0, 0, 0, 0}; // for edge-triggered fault logging
  // Every flag is initialised explicitly: `bool a, b, c = false;` only
  // initialises c. m_flag_manual gates every joint_space/target message in
  // jointSpaceTarget_callback, so an indeterminate (non-zero) value silently
  // dropped all manager position targets ("Start Procedure does nothing").
  bool m_flag_manual = false, m_flag_use_target_action = false, m_trans_limit = false;
  bool m_emtracker_alive = false, m_targpublisher_alive = false, m_targpublisher_alive_tmep = false;
  bool m_flag_readyToEngage = false, m_flagEngaged = false, m_head_attached = false;
  int m_locked = 0;
  double m_kp = 0.00, m_ki = 0.00;
  bool m_procedure = false;
  std::array<bool, 7> m_interface_key = {0, 0, 0, 0, 0, 0, 0};
  std::array<bool, 4> m_encoders_set = {0, 0, 0, 0};
  CtrlMode m_mode = CtrlMode::Config; // controller mode (manual, velocity, position); Config issues no targets
  bool m_reached_prev = true;                                 // for edge-triggered reached-status logging in Position mode
  std::chrono::steady_clock::time_point m_not_reached_since{}; // start of current not-reached streak
  blaze::StaticVector<double, 4UL> m_not_reached_target;       // target in effect when the not-reached streak started
  std::chrono::steady_clock::time_point m_last_target_rx{};    // last time jointSpaceTarget_callback saw a message
  std::mutex m_x_des_log_mutex;                                  // guards m_x_des_log_snapshot (written in m_cbGroup2, read in m_cbGroup3)
  blaze::StaticVector<double, 4UL> m_x_des_log_snapshot;          // diagnostics-only copy of m_x_des for the status timer
  size_t publisher_count;
  blaze::StaticVector<double, 4UL> m_x, m_x_des, m_x_error;           // in SI units
  blaze::StaticVector<double, 4UL> m_xdot, m_xdot_manual, m_xdot_des; // in SI units
  blaze::StaticVector<double, 4UL> m_x_error_int;                     // in SI units
  blaze::StaticVector<double, 4UL> m_current;
  blaze::StaticVector<double, 4UL> m_minCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 4UL> m_maxCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 3UL> m_x_tip;
  blaze::StaticVector<int32_t, 4UL> m_cpu_temp;
  blaze::StaticVector<int32_t, 4UL> m_winding_temp;
  blaze::StaticVector<std::bitset<32>, 4UL> m_digital_input;
  blaze::StaticVector<double, 4UL> m_maxTorqueNegative;
  blaze::StaticVector<double, 4UL> m_maxTorquePositive;
  blaze::StaticVector<double, 4UL> m_maxDcc; // [deg/s^2] and [mm/s^2]
  blaze::StaticVector<double, 4UL> m_maxVel; // [deg/s] and [mm/s]
  blaze::StaticVector<double, 4UL> m_maxAcc; // [deg/s^2] and [mm/s^2]

  std::unique_ptr<ICtrJointGroup> m_robot; // hardware seam (CTRobot on the real robot)
  // node-side spdlog sink (was inherited from CTRobot before the seam)
  std::shared_ptr<spdlog::logger> m_logger = spdlog::default_logger();
  std::thread worker_thread_;
  std::mutex m_task_mutex;
  std::condition_variable m_task_cv;
  std::function<void()> m_current_task;
  std::atomic<bool> m_cancel_flag{false};
  std::atomic<bool> m_flag_task_ready{false};

  rclcpp::TimerBase::SharedPtr m_watchdog_timer_emt;
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
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNode>();
  node->initHardware();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 7);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

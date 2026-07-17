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
#include "interfaces/msg/force.hpp"
#include "interfaces/action/target.hpp"
#include "interfaces/action/jointstarget.hpp"
#include "interfaces/srv/transformation.hpp"
#include "interfaces/srv/jointstarget.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <fstream>
#include <filesystem>

#include "ctr_pinn_inference.hpp"
#include <iostream>
#include <limits>
#include <chrono>
#include <fstream>
#include <string>
#include <filesystem>
#include <boost/tokenizer.hpp>

#include <limits>

// #include <chrono>
#include <future> // needed in order to invoke async "execute functions asynchronously"
#include <boost/tokenizer.hpp>
#include <functional>
#include <blaze/Math.h>

#include <memory.h>
#include <numeric>

#include <thread>

#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

template <size_t N, bool SO = blaze::rowMajor>
std_msgs::msg::Float64MultiArray blazeToMultiArrayMsg(const blaze::StaticMatrix<double, N, 3UL, SO>& mat);

template <bool SO = blaze::rowMajor>
std_msgs::msg::Float64MultiArray blazeToMultiArrayMsg(const blaze::DynamicMatrix<double, SO>& mat);

template <size_t N>
void clampJointPositions(blaze::StaticVector<double, N> &q, const blaze::StaticVector<double, N> &q_min, const blaze::StaticVector<double, N> &q_max, const rclcpp::Logger &logger);


class RobotSimNode : public rclcpp::Node
{
private:
  // Member variables
  const std::string m_packageName = "robot";
  static constexpr size_t N = 4UL; // number of joints (keep it 6 although the robot is 4DoF)
  static constexpr size_t M = 3UL; // outputs (x,y,z)

  std::string m_model_name;
  std::size_t m_backbonePoints;
  std::shared_ptr<PINNs<N>> m_ctr_pinns;

  // static constexpr std::size_t backbonePoints = 30UL;
  // std::shared_ptr<PINNs<backbonePoints, N>> m_ctr_pinns;

  double m_sample_time;
  blaze::StaticVector<double, N> m_q, m_q0;
  blaze::StaticVector<double, N> m_q_min, m_q_max; // Joint limits
  blaze::StaticVector<double, 3> m_wf;         
  blaze::StaticVector<double, 4> m_qdot_des(0.0);

  // Ros interfaces
  rclcpp::CallbackGroup::SharedPtr m_callback_group_0;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_1;                   
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_q; // Subscriber object
  rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_subscription_f; // Subscriber object
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_service_reset_q;
  rclcpp::TimerBase::SharedPtr m_fk_timer;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_tube_1, m_publisher_tube_2, m_publisher_tube_3;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_simout;

public:
  // default class constructor
  RobotSimNode() : Node("forward_kinematics_node")
  {
    RobotSimNode::declare_parameters();
    RobotSimNode::setup_ros_interfaces();
    RobotSimNode::init_pinns_model();
    RCLCPP_INFO(this->get_logger(), "Forward Kinenamtics Node Initialized");
  }

  // class destructor
  ~RobotSimNode()
  {
  }

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    declare_parameter<double>("sample_time", 25E-3);
    m_sample_time = get_parameter("sample_time").as_double();
    if (m_sample_time <= 0.0)
    {
      RCLCPP_ERROR(get_logger(), "sample_time must be > 0 (got %f)", m_sample_time);
      throw std::runtime_error("Invalid sample_time parameter");
    }

    declare_parameter<int>("num_backbone", 50);
    int backbonePoints = get_parameter("num_backbone").as_int();
    if (backbonePoints <= 1)
    {
      RCLCPP_ERROR(get_logger(), "num_backbone must be > 0 (got %d)", backbonePoints);
      throw std::runtime_error("Invalid num_backbone parameter");
    }
    m_backbonePoints = static_cast<std::size_t>(backbonePoints);

    declare_parameter<std::string>("model_name", "ctr_8x91_0.18_tanh_9K_9K_50K_v3");
    m_model_name = get_parameter("model_name").as_string();

    declare_parameter<std::vector<double>>("q0", {-156.00E-3, -72.00E-3, 0.0, 0.0});
    std::vector<double> q0Vec = get_parameter("q0").as_double_array();
    if (q0Vec.size() != N)
    {
      RCLCPP_ERROR(get_logger(), "q0 must have exactly %zu elements, got %zu", N, q0Vec.size());
      throw std::runtime_error("Invalid q0 parameter size");
    }
    for (size_t i = 0; i < N; ++i)
    {
      m_q0[i] = q0Vec[i];
    }
    m_q = m_q0;
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

    // 
    m_subscription_target = create_subscription<interfaces::msg::Jointspace>("joint_space/target", 10, std::bind(&RobotSimNode::jointSpaceTarget_callback, this, _1), subs_options_1);

    // Publisher to broadcast the robot status
    m_publisher_status = create_publisher<interfaces::msg::Status>("robot_status", 10);
    // Publisher to broadcast the robot interface status
    m_publisher_interface = create_publisher<interfaces::msg::Interface>("manual_interface", 10);
    // Publisher to broadcast the current joint configurations
    m_publisher_joints = create_publisher<interfaces::msg::Jointspace>("joint_space/feedback", 10);

    // Low-level control loop timer
    auto command_sample_time = std::chrono::milliseconds(static_cast<int>(10));
    m_control_loop_timer = create_wall_timer(command_sample_time, std::bind(&RobotSimNode::targetCommand_timerCallback, this), m_cbGroup1);
    // Timer to read joint configurations periodically
    m_joints_config_timer = create_wall_timer(10ms, std::bind(&RobotSimNode::jointsConfig_timerCallback, this), m_cbGroup3);
    // Timer to read robot status periodically
    m_read_robot_timer = create_wall_timer(10ms, std::bind(&RobotSimNode::robotStatus_timerCallback, this), m_cbGroup3);
    // Initialize a timer to check emtracker lifecycle
    m_watchdog_timer_emt = create_wall_timer(100ms, std::bind(&RobotSimNode::check_emtracker_alive_timerCallback, this), m_callback_group_watchdog_1);

    m_service_reset_q = this->create_service<std_srvs::srv::Trigger>("set_q_to_q0", std::bind(&RobotSimNode::setQToQ0Callback, this, _1, _2));

    // Subscriber to receive current tip force
    auto subs_current_f = rclcpp::SubscriptionOptions();
    subs_current_f.callback_group = m_callback_group_1;
    m_subscription_f = create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&RobotSimNode::updateExternalForce, this, _1), subs_current_f);    
 
    // timer to run the forward kin and publish shape
    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_fk_timer = create_wall_timer(sample_time, std::bind(&RobotSimNode::actuate_timer_callback, this), m_callback_group_0);

    // Publishers for broadcasting the task space position
    m_publisher_simout = create_publisher<interfaces::msg::Taskspace>("task_space/sim_out", 10);

    // publishers to send the backbone shape
    m_publisher_tube_1 = create_publisher<std_msgs::msg::Float64MultiArray>("shape/tube_1", 10);
    m_publisher_tube_2 = create_publisher<std_msgs::msg::Float64MultiArray>("shape/tube_2", 10);
    m_publisher_tube_3 = create_publisher<std_msgs::msg::Float64MultiArray>("shape/tube_3", 10);
  }

  /// initialize robot
  void init_pinns_model()
  {
    if (m_model_name.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Model name is not set!");
      throw std::runtime_error("Model name parameter is required");
    }

    if (m_backbonePoints <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Backbone points must be greater than 0!");
      throw std::runtime_error("Invalid backbone points parameter");
    }

    // Initialize the PINN model
    RCLCPP_INFO(this->get_logger(), "Model: %s, Backbone Points: %zu", m_model_name.c_str(), m_backbonePoints);
    m_ctr_pinns = std::make_shared<PINNs<N>>(m_model_name, 1UL, m_backbonePoints);

    std::tie(m_q_min, m_q_max) = m_ctr_pinns->getInputPosBounds();

    // warm up the model
    blaze::StaticVector<double, M> x;
    m_ctr_pinns->getPosDistal(m_q, m_wf, x);
    m_ctr_pinns->getPosDistal(m_q, m_wf, x);

    RCLCPP_INFO(this->get_logger(), "PINN FK initialized");
  }

  // Subscription callback function to updates the target joint positions and velocities
  void jointSpaceTarget_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    m_qdot_des[0UL] = msg->velocity[0UL];
    m_qdot_des[1UL] = msg->velocity[1UL];
    m_qdot_des[2UL] = msg->velocity[2UL];
    m_qdot_des[3UL] = msg->velocity[3UL];
    m_logger->info("[RobotSimNode] Vel target received: [{:.4f}, {:.4f}, {:.4f}, {:.4f}]",
                    m_qdot_des[0], m_qdot_des[1], m_qdot_des[2], m_qdot_des[3]);
  }

  /// Service callback to reset simulated joints to initial value q0
  void setQToQ0Callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    m_q = m_q0;
    response->success = true;
    response->message = "m_q was set to m_q0";
    RCLCPP_INFO(this->get_logger(), "Service set_q_to_q0 called: m_q reset to q0");
  }

  /// @brief Update disterbance distal force (m_wf = f)
  void updateExternalForce(const interfaces::msg::Force::ConstSharedPtr &msg)
  {
    m_wf[0] = msg->x;
    m_wf[1] = msg->y;
    m_wf[2] = msg->z;
    RCLCPP_DEBUG(this->get_logger(), "u: [%.3f, %.3f, %.3f] N", m_wf[0UL], m_wf[1UL], m_wf[2UL]);
  }

  /// actuate the model to the current joints config and get the shape
  void actuate_timer_callback()
  {
    blaze::StaticVector<double, 7UL> x;
    blaze::StaticVector<double, 4UL> q_dot;

    // Clamp desired velocity to limits
    blaze::StaticVector<double, N> xdot_des_max = {0.012, 0.012, 3.0, 3.0};  // Example lower limit in m/s

    for (size_t i = 0; i < N; ++i) 
    {
      q_dot[i] = std::clamp(m_qdot_des[i], -xdot_des_max[i], xdot_des_max[i]);
    }

    m_q += q_dot * m_sample_time; 

    // clamp joint positions within limits
    clampJointPositions(m_q, m_q_min, m_q_max, this->get_logger());
    
    // forward kinematics
    m_ctr_pinns->getPosDistal(m_q, m_wf, x);

    // publish end-effector position
    interfaces::msg::Taskspace simout_msg;
    simout_msg.p[0] = x[0];;
    simout_msg.p[1] = x[1];;
    simout_msg.p[2] = x[2];;
    m_publisher_simout->publish(simout_msg);

    const auto [Tb1, Tb2, Tb3] = m_ctr_pinns->getAllTubesShape(m_q, m_wf);

    // convert unit to mm for Slicer
    blaze::DynamicMatrix<double, blaze::rowMajor> Tb1_mm = Tb1 * 1.0e3;
    blaze::DynamicMatrix<double, blaze::rowMajor> Tb2_mm = Tb2 * 1.0e3;
    blaze::DynamicMatrix<double, blaze::rowMajor> Tb3_mm = Tb3 * 1.0e3;

    std::cout << "Tb1: \n" << Tb1_mm << std::endl;

    std_msgs::msg::Float64MultiArray msg_1 = blazeToMultiArrayMsg(Tb1_mm);
    std_msgs::msg::Float64MultiArray msg_2 = blazeToMultiArrayMsg(Tb2_mm);
    std_msgs::msg::Float64MultiArray msg_3 = blazeToMultiArrayMsg(Tb3_mm);

    // blaze::StaticMatrix<double, m_backbonePoints, 3UL> Tb1_mm = Tb1 * 1.0e3;
    // blaze::StaticMatrix<double, m_backbonePoints, 3UL> Tb2_mm = Tb2 * 1.0e3;
    // blaze::StaticMatrix<double, m_backbonePoints, 3UL> Tb3_mm = Tb3 * 1.0e3;

    // std::cout << "Tb1: \n" << Tb1_mm << std::endl;

    // std_msgs::msg::Float64MultiArray msg_1 = blazeToMultiArrayMsg<m_backbonePoints>(Tb1_mm);
    // std_msgs::msg::Float64MultiArray msg_2 = blazeToMultiArrayMsg<m_backbonePoints>(Tb2_mm);
    // std_msgs::msg::Float64MultiArray msg_3 = blazeToMultiArrayMsg<m_backbonePoints>(Tb3_mm);

    m_publisher_tube_1->publish(msg_1);
    m_publisher_tube_2->publish(msg_2);
    m_publisher_tube_3->publish(msg_3);

    // RCLCPP_INFO(this->get_logger(), "Published shape");
  }
  
};

/// @brief Convert a Blaze StaticMatrix to a ROS Float64MultiArray message
  template <size_t N, bool SO>
  std_msgs::msg::Float64MultiArray blazeToMultiArrayMsg(const blaze::StaticMatrix<double, N, 3UL, SO>& mat)
  {
    std_msgs::msg::Float64MultiArray msg;

    constexpr size_t rows = N;
    constexpr size_t cols = 3UL;

    // MultiArray layout: row-major flattening
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label  = "rows";
    msg.layout.dim[0].size   = rows;
    msg.layout.dim[0].stride = cols;      // next row starts after 'cols' items
    msg.layout.dim[1].label  = "cols";
    msg.layout.dim[1].size   = cols;
    msg.layout.dim[1].stride = 1;         // next col is next item

    msg.data.resize(rows * cols);

    // Flatten row-major; works for either storage order SO
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        msg.data[i * cols + j] = mat(i, j) * 1.0e-3;

    return msg;
  }

/// @brief Convert a Blaze DynamicMatrix to a ROS Float64MultiArray message
  template <bool SO>
  std_msgs::msg::Float64MultiArray blazeToMultiArrayMsg(const blaze::DynamicMatrix<double, SO>& mat)
  {
    std_msgs::msg::Float64MultiArray msg;

    const size_t rows = mat.rows();
    const size_t cols = mat.columns();

    // MultiArray layout: row-major flattening
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label  = "rows";
    msg.layout.dim[0].size   = rows;
    msg.layout.dim[0].stride = cols;      // next row starts after 'cols' items
    msg.layout.dim[1].label  = "cols";
    msg.layout.dim[1].size   = cols;
    msg.layout.dim[1].stride = 1;         // next col is next item

    msg.data.resize(rows * cols);

    // Flatten row-major; works for either storage order SO
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        msg.data[i * cols + j] = mat(i, j) * 1.0e-3;

    return msg;
  }

/// @brief Clamp joint positions with consideration of coupled joints
template <size_t N>
void clampJointPositions(blaze::StaticVector<double, N> &q, const blaze::StaticVector<double, N> &q_min, const blaze::StaticVector<double, N> &q_max, const rclcpp::Logger &logger)
{
  blaze::StaticVector<double, N> q_max_eff = q_max;
  blaze::StaticVector<double, N> q_min_eff = q_min;

  if (N == 6)
  {
    q_max_eff[1] += q[2];
    q_min_eff[1] += q[2];
    q_max_eff[0] += q[1];
    q_min_eff[0] += q[1];
    q_max_eff[4] += q[5];
    q_min_eff[4] += q[5];
    q_max_eff[3] += q[4];
    q_min_eff[3] += q[4];
  }
  else if (N == 4)
  {
    q_max_eff[0] += q[1];
    q_min_eff[0] += q[1];
    q_max_eff[2] += q[3];
    q_min_eff[2] += q[3];
  }
  else
  {
    RCLCPP_WARN(logger, "clampJointPositions: No coupling handling for N=%zu", N);
  }

  for (size_t i = 0; i < q.size(); ++i)
  {
    double original = q[i];
    q[i] = std::clamp(q[i], q_min_eff[i], q_max_eff[i]);
    if (q[i] != original)
    {
      // RCLCPP_WARN(logger, "q clamping at joint %zu: %.6f -> %.6f", i, original, q[i]);
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotSimNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

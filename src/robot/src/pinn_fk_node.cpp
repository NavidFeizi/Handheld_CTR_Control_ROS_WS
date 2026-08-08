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
#include "interfaces/srv/transformation.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <fstream>
#include <filesystem>

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "ctr_common/runtime_paths.hpp"
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

class ForwardKinNode : public rclcpp::Node
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
  std::mutex m_state_mutex; // guards m_q / m_wf (subs write, FK timer reads)
  blaze::StaticVector<double, N> m_q;
  blaze::StaticVector<double, N> m_q_min, m_q_max; // Joint limits
  blaze::StaticVector<double, 3> m_wf;              

  // Ros interfaces
  rclcpp::CallbackGroup::SharedPtr m_callback_group_0;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_1;                   
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_q; // Subscriber object
  rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_subscription_f; // Subscriber object
  rclcpp::TimerBase::SharedPtr m_fk_timer;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_tube_1, m_publisher_tube_2, m_publisher_tube_3;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_simout;

public:
  // default class constructor
  ForwardKinNode() : Node("forward_kinematics_node")
  {
    ForwardKinNode::declare_parameters();
    ForwardKinNode::setup_ros_interfaces();
    ForwardKinNode::init_pinns_model();
    RCLCPP_INFO(this->get_logger(), "Forward Kinenamtics Node Initialized");
  }

  // class destructor
  ~ForwardKinNode()
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

    declare_parameter<std::vector<double>>("q0", {-100.00E-3, -55.00E-3, 0.0, 0.0});
    std::vector<double> q0Vec = get_parameter("q0").as_double_array();
    if (q0Vec.size() != N)
    {
      RCLCPP_ERROR(get_logger(), "q0 must have exactly %zu elements, got %zu", N, q0Vec.size());
      throw std::runtime_error("Invalid q0 parameter size");
    }
    for (size_t i = 0; i < N; ++i)
    {
      m_q[i] = q0Vec[i];
    }
  }

  // Setup ROS interfaces, including publishers, subscribers, and services.
  void setup_ros_interfaces()
  {
    m_callback_group_0 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    // timer to run the forward kin and publish shape
    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_fk_timer = create_wall_timer(sample_time, std::bind(&ForwardKinNode::actuate_timer_callback, this), m_callback_group_0);

    // Subscriber to receive current q
    auto subs_current_q = rclcpp::SubscriptionOptions();
    subs_current_q.callback_group = m_callback_group_1;
    m_subscription_q = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&ForwardKinNode::updateJointsConfig, this, _1), subs_current_q);

    // Subscriber to receive current tip force
    auto subs_current_f = rclcpp::SubscriptionOptions();
    subs_current_f.callback_group = m_callback_group_1;
    m_subscription_f = create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&ForwardKinNode::updateExternalForce, this, _1), subs_current_f);

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
    m_ctr_pinns = std::make_shared<PINNs<N>>(ctr_common::resolveModelsDir(*this).string(), m_model_name, 1UL, m_backbonePoints);

    std::tie(m_q_min, m_q_max) = m_ctr_pinns->getInputPosBounds();

    // warm up the model
    blaze::StaticVector<double, M> x;
    m_ctr_pinns->getPosDistal(m_q, m_wf, x);
    m_ctr_pinns->getPosDistal(m_q, m_wf, x);

    RCLCPP_INFO(this->get_logger(), "PINN FK initialized");
  }

  /// Update current joints configuration
  void updateJointsConfig(const interfaces::msg::Jointspace::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_q = ctr_common::wireToPhysics4(msg->position);
  }

  /// @brief Update disterbance distal force (m_wf = f)
  void updateExternalForce(const interfaces::msg::Force::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_wf[0] = msg->x;
    m_wf[1] = msg->y;
    m_wf[2] = msg->z;
    RCLCPP_DEBUG(this->get_logger(), "u: [%.3f, %.3f, %.3f] N", m_wf[0UL], m_wf[1UL], m_wf[2UL]);
  }

  /// actuate the model to the current joints config and get the shape
  void actuate_timer_callback()
  {
    blaze::StaticVector<double, M> x;

    // Snapshot the shared state (subscriptions write it from another executor
    // thread), then clamp the LOCAL copy — the feedback state itself is never
    // mutated by this timer.
    blaze::StaticVector<double, N> q;
    blaze::StaticVector<double, 3> wf;
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      q = m_q;
      wf = m_wf;
    }
    ctr_common::clampJointPositions(q, m_q_min, m_q_max, this->get_logger());
    
    // just for test
    // // Generate time-varying force input
    // // Use node timer to generate time-varying force signal
    // auto now = this->get_clock()->now();
    // double t = now.seconds();
    
    // const double sigmoid_sharpness = 50.0;
    // const double wf_mag = 0.2 / (1.0 + std::exp(-sigmoid_sharpness * (t - 0.5)));
    // const double freq_xy = 0.25;
    // const double freq_z = 0.1;
    // const double angle = 2.0 * M_PI * freq_xy * t;

    // m_wf[0] = wf_mag * std::cos(angle);
    // m_wf[1] = wf_mag * std::sin(angle);
    // m_wf[2] = wf_mag * 0.2 * std::sin(2.0 * M_PI * freq_z * t);

    // forward kinematics
    m_ctr_pinns->getPosDistal(q, wf, x);

    // publish end-effector position
    interfaces::msg::Taskspace simout_msg;
    simout_msg.p[0] = x[0];;
    simout_msg.p[1] = x[1];;
    simout_msg.p[2] = x[2];;
    m_publisher_simout->publish(simout_msg);

    const auto [Tb1, Tb2, Tb3] = m_ctr_pinns->getAllTubesShape(q, wf);

    // shapes stay in metres on the wire; the IGTL bridge converts to mm for Slicer
    std_msgs::msg::Float64MultiArray msg_1 = blazeToMultiArrayMsg(Tb1);
    std_msgs::msg::Float64MultiArray msg_2 = blazeToMultiArrayMsg(Tb2);
    std_msgs::msg::Float64MultiArray msg_3 = blazeToMultiArrayMsg(Tb3);

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
        msg.data[i * cols + j] = mat(i, j);

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
        msg.data[i * cols + j] = mat(i, j);

    return msg;
  }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardKinNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

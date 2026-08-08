#include <chrono>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>
#include <array>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "std_msgs/msg/float64.hpp"
#include "interfaces/srv/recording.hpp"
#include "interfaces/msg/force.hpp"

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "ctr_common/runtime_paths.hpp"
#include "mpc.hpp"
#include "mpc.tpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> circular_ref_trajectory(double t, double dt, bool vary_in_horizon);

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> arc270_ref_trajectory(double t, double dt, bool vary_in_horizon);

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> infinity_ref_trajectory(double t, double dt, bool vary_in_horizon);

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> W_ref_trajectory(double t, double dt, bool vary_in_horizon);

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> H_ref_trajectory(double t, double dt, bool vary_in_horizon);

class MpcNode : public rclcpp::Node
{
protected:
  static constexpr size_t N = 4UL; // joints (beta1, beta2, beta3, alpha1, alpha2, alpha3)
  static constexpr size_t m = 3UL; // outputs (x,y,z)
  static constexpr size_t h = 15L; // horizon

private:
  // MPC model
  std::shared_ptr<PINNs<N>> m_mpc_model;
  std::string m_model_name;

  // MPC
  // const blaze::StaticVector<double, N> m_margin_q = blaze::StaticVector<double, N>({1e-3, 1e-3, 1e-3, 2e-2, 2e-2, 2e-2}); // Joints position limits safety margin
  const blaze::StaticVector<double, N> m_margin_q = blaze::StaticVector<double, N>({1e-4, 1e-4, 1e-2, 1e-2}); // Joints position limits safety margin
  std::shared_ptr<MPC<h, N, m>> m_mpc;
  double m_sample_time;                            // MPC loop sample time
  double m_Q;                                      // Task space weights
  double m_R_u;                                    //  input weights on u
  double m_R_du;                                   //  input weights on du
  blaze::StaticVector<double, N> m_q_scale;        // Joints scaling factors
  blaze::StaticVector<double, N> m_uMax;           // Control input (u = q_dot) limit
  blaze::StaticVector<double, N> m_u_dotMax;       // du limit
  blaze::StaticVector<double, N> m_q_min, m_q_max; // Joints position limits
  blaze::StaticVector<double, N> m_q;              // current robot joints position
  blaze::StaticVector<double, m> m_x;              // current end effector positoin
  blaze::StaticVector<double, m> m_ref;            // reference in task space
  const blaze::StaticVector<double, m> k_x_scale = blaze::StaticVector<double, m>({1.0, 1.0, 1.0});

  blaze::StaticVector<double, 3> m_wf; // external distal force estimate

  double m_error_c = 0.0; // error compensation gain
  double m_t;             // time tracker

  rclcpp::TimerBase::SharedPtr m_control_timer;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_mpc;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_publisher_time;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_control;              // publisher object
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joint_feedback; // subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_target;          // subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_task_feedback;   // subscriber object

  rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_subscription_f; // Subscriber object

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_reference; // publisher object
  rclcpp::Client<interfaces::srv::Recording>::SharedPtr m_record_client;

  mutable std::mutex m_state_mutex;

  // Kalman filter state for joints
  blaze::StaticVector<double, N> m_q_hat{0.0};  // filtered joint position
  blaze::StaticVector<double, N> m_P{1e-4};     // covariance per joint (variance)
  blaze::StaticVector<double, N> m_u_last{0.0}; // last commanded joint velocity
  // Simple noise parameters (tune!)
  double m_q_proc_var = 1e-2; // process noise variance (model: integration drift)
  double m_q_meas_var = 1.0;  // measurement noise variance (encoder noise)

public:
  MpcNode() : Node("mpc")
  {
    this->initNodeParameters();
    this->setupDynamicParameterUpdates();
    this->setupRosInterfaces();
    this->initMPC();

    m_t = 0.0;

    // in constructor, after you have a reasonable m_q:
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_q_hat = m_q; // start filter at measured joints
    }
    m_P = 1e-4; // same variance for all joints initially
    m_u_last = 0.0;

    // Start Recording
    auto request = std::make_shared<interfaces::srv::Recording::Request>();
    request->command = "start"; // Set the desired command
    request->duration = 0.0;    // Set the desired duration
    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Recording>::SharedFuture;
    auto response_received_callback = std::bind(&MpcNode::handle_record_response, this, std::placeholders::_1);
    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

private:
  /// @brief Function to declare and initialize ROS parameters - parameters values can be set from the launch file
  void initNodeParameters()
  {
    declare_parameter<double>("sample_time", 25E-3);
    m_sample_time = get_parameter("sample_time").as_double();

    declare_parameter<std::string>("model_name", "ctr_8x91_0.18_tanh_9K_9K_50K_v3");
    m_model_name = get_parameter("model_name").as_string();

    declare_parameter<std::vector<double>>("q0", {-0.156, -0.072, 0.0, 0.0});
    std::vector<double> q0Vec = get_parameter("q0").as_double_array();

    declare_parameter<std::vector<double>>("u_max", {0.012, 0.012, 3.0, 3.0});
    std::vector<double> uMaxVec = get_parameter("u_max").as_double_array();

    declare_parameter<std::vector<double>>("u_dot_max", {0.10, 0.10, 10.0, 10.0});
    std::vector<double> u_dotMaxVec = get_parameter("u_dot_max").as_double_array();

    declare_parameter<std::vector<double>>("q_scale", {0.0, 0.0, 0.0, 0.0});
    std::vector<double> qScaleVec = get_parameter("q_scale").as_double_array();

    declare_parameter<double>("R_u", {0.0});
    double Ru = get_parameter("R_u").as_double();

    declare_parameter<double>("R_du", {0.5});
    double Rdu = get_parameter("R_du").as_double();

    declare_parameter<double>("Q", {1000.0});
    double Q = get_parameter("Q").as_double();

    declare_parameter<double>("error_c", {0.0});
    double error_c = get_parameter("error_c").as_double();

    declare_parameter<double>("q_proc_var", {1e-2});
    m_q_proc_var = get_parameter("q_proc_var").as_double();

    declare_parameter<double>("q_meas_var", {1.0});
    m_q_meas_var = get_parameter("q_meas_var").as_double();

    for (size_t i = 0; i < N; ++i)
    {
      m_q[i] = q0Vec[i];
      m_uMax[i] = uMaxVec[i];
      m_u_dotMax[i] = u_dotMaxVec[i];
      m_q_scale[i] = qScaleVec[i];
    }

    m_R_u = Ru;
    m_R_du = Rdu;
    m_Q = Q;

    m_error_c = error_c;
  }

  /// @brief Function to setup dynamic parameter update callback
  void setupDynamicParameterUpdates()
  {
    auto param_callback =
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        const std::string &name = parameter.get_name();
        const auto type = parameter.get_type();

        if (name == "Q")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.successful = false;
            result.reason = name + " must be a double.";
            continue;
          }
          auto value = parameter.as_double();
          m_Q = value;

          auto Q = k_x_scale * m_Q;
          auto R_u = blaze::pow(m_q_scale, 2.0) * m_R_u;
          auto R_du = blaze::pow(m_q_scale, 2.0) * m_R_du;
          m_mpc->updateWeights(Q, 2.0 * Q, R_u, R_du);
        }

        else if (name == "R_u")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.successful = false;
            result.reason = name + " must be a double.";
            continue;
          }
          auto value = parameter.as_double();
          m_R_u = value;

          auto Q = k_x_scale * m_Q;
          auto R_u = blaze::pow(m_q_scale, 2.0) * m_R_u;
          auto R_du = blaze::pow(m_q_scale, 2.0) * m_R_du;
          m_mpc->updateWeights(Q, 2.0 * Q, R_u, R_du);
        }

        else if (name == "R_du")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.successful = false;
            result.reason = name + " must be a double.";
            continue;
          }
          auto value = parameter.as_double();
          m_R_du = value;

          auto Q = k_x_scale * m_Q;
          auto R_u = blaze::pow(m_q_scale, 2.0) * m_R_u;
          auto R_du = blaze::pow(m_q_scale, 2.0) * m_R_du;
          m_mpc->updateWeights(Q, 2.0 * Q, R_u, R_du);
        }

        else if (name == "q_scale")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
          {
            result.successful = false;
            result.reason = name + " must be a double array.";
            continue;
          }

          auto values = parameter.as_double_array();
          if (values.size() != N)
          {
            result.successful = false;
            result.reason = name + " must contain " + std::to_string(N) + " elements.";
            continue;
          }
          for (size_t i = 0; i < N; ++i)
          {
            m_q_scale[i] = values[i];
          }

          auto Q = k_x_scale * m_Q;
          auto R_u = blaze::pow(m_q_scale, 2.0) * m_R_u;
          auto R_du = blaze::pow(m_q_scale, 2.0) * m_R_du;
          m_mpc->updateWeights(Q, 2.0 * Q, R_u, R_du);
        }

        else if (name == "u_max")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
          {
            result.successful = false;
            result.reason = name + " must be a double array.";
            continue;
          }

          auto values = parameter.as_double_array();
          if (values.size() != N)
          {
            result.successful = false;
            result.reason = name + " must contain " + std::to_string(N) + " elements.";
            continue;
          }

          for (size_t i = 0; i < N; ++i)
          {
            m_uMax[i] = values[i];
          }

          auto uMin = m_uMax * -1.0;
          auto u_dotMin = m_u_dotMax * -1.0;
          auto m_margin_u = m_uMax * 0.02;
          auto m_margin_du = m_u_dotMax * 0.0;
          m_mpc->setJointsLimits(m_q_min, m_q_max, uMin, m_uMax, u_dotMin, m_u_dotMax, m_margin_q, m_margin_u, m_margin_du);
        }

        else if (name == "u_dot_max")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
          {
            result.successful = false;
            result.reason = name + " must be a double array.";
            continue;
          }

          auto values = parameter.as_double_array();
          if (values.size() != N)
          {
            result.successful = false;
            result.reason = name + " must contain " + std::to_string(N) + " elements.";
            continue;
          }

          for (size_t i = 0; i < N; ++i)
          {
            m_u_dotMax[i] = values[i];
          }

          auto uMin = m_uMax * -1.0;
          auto u_dotMin = m_u_dotMax * -1.0;
          auto m_margin_u = m_uMax * 0.02;
          auto m_margin_du = m_u_dotMax * 0.0;
          m_mpc->setJointsLimits(m_q_min, m_q_max, uMin, m_uMax, u_dotMin, m_u_dotMax, m_margin_q, m_margin_u, m_margin_du);
        }

        else if (name == "error_c")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.successful = false;
            result.reason = name + " must be a double.";
            continue;
          }
          auto value = parameter.as_double();
          m_error_c = value;
        }

        else if (name == "q_proc_var")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.successful = false;
            result.reason = name + " must be a double.";
            continue;
          }
          auto value = parameter.as_double();
          m_q_proc_var = value;
          RCLCPP_INFO(this->get_logger(), "Kalman filter process variance updated: q_proc_var = %f", m_q_proc_var);
        }

        else if (name == "q_meas_var")
        {
          if (type != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.successful = false;
            result.reason = name + " must be a double.";
            continue;
          }
          auto value = parameter.as_double();
          m_q_meas_var = value;
          RCLCPP_INFO(this->get_logger(), "Kalman filter measurement variance updated: q_meas_var = %f", m_q_meas_var);
        }

        else
        {
          result.successful = false;
          result.reason = "Parameter " + name + " not recognized.";
        }
      };
      return result;
    };
    m_param_callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  /// @brief setup ROS Publisher, Subscribers, and Timers
  void setupRosInterfaces()
  {
    // Create callback groups
    m_callback_group_mpc = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // subscriber to receive target
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub1;
    m_subscription_target = this->create_subscription<interfaces::msg::Taskspace>(
        "task_space/target", rclcpp::QoS(10), std::bind(&MpcNode::updateTarget, this, _1), subs_options_1);

    // subscriber to receive Model Output feedback
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    subs_options_2.callback_group = m_callback_group_sub1;
    m_subscription_task_feedback = this->create_subscription<interfaces::msg::Taskspace>(
        // "task_space/sim_out", rclcpp::QoS(10), std::bind(&MpcNode::updateTipPosition, this, _1), subs_options_2);
        "/task_space/feedback/base_tool", rclcpp::QoS(10), std::bind(&MpcNode::updateTipPosition, this, _1), subs_options_2);

    // subscriber to receive Jointspace feedback
    auto subs_options_3 = rclcpp::SubscriptionOptions();
    subs_options_3.callback_group = m_callback_group_sub1;
    m_subscription_joint_feedback = this->create_subscription<interfaces::msg::Jointspace>(
        "joint_space/feedback", rclcpp::QoS(10), std::bind(&MpcNode::updateJointsPosition, this, _1), subs_options_3);

    // Subscriber to receive current tip force
    auto subs_current_f = rclcpp::SubscriptionOptions();
    subs_current_f.callback_group = m_callback_group_sub1;
    m_subscription_f = this->create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&MpcNode::updateExternalForce, this, _1), subs_current_f);

    // publisher to publish control signal
    m_publisher_control = this->create_publisher<interfaces::msg::Jointspace>("joint_space/target", 10);

    // publisher to publish computation time
    m_publisher_time = this->create_publisher<std_msgs::msg::Float64>("mpc/computation_time", 10);

    // Create wall timers with different callback groups
    RCLCPP_INFO(this->get_logger(), "MPC sample_time: %.6f s", m_sample_time);
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_control_timer = this->create_wall_timer(control_sample_time, std::bind(&MpcNode::mpcStep, this), m_callback_group_mpc);

    // // publisher to publish reference signal
    m_publisher_reference = this->create_publisher<interfaces::msg::Taskspace>("/task_space/target", 10);

    // Recorder service
    m_record_client = this->create_client<interfaces::srv::Recording>("recording");
    // while (!m_record_client->wait_for_service(std::chrono::seconds(1)))
    // {
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //         return;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "Record service not available, waiting again...");
    // }
  }

  /// @brief Initialize MPC
  void initMPC()
  {
    if (m_model_name.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Model name is not set!");
      throw std::runtime_error("Model name parameter is required");
    }

    const size_t backbonePoints = 10UL;
    RCLCPP_INFO(this->get_logger(), "Model: %s", m_model_name.c_str());

    // instantiate MPC model object
    m_mpc_model = std::make_shared<PINNs<N>>(ctr_common::resolveModelsDir(*this).string(), m_model_name, h, backbonePoints);

    // instantiate MPC object with disturbance force input
    m_mpc = std::make_shared<MPC<h, N, m>>(
        m_sample_time,
        [this](const blaze::StaticVector<double, N> &q, const blaze::StaticVector<double, m> &wf) -> blaze::StaticVector<double, m>
        {
          blaze::StaticVector<double, m, blaze::columnVector> y;
          this->m_mpc_model->getPosDistal(q, wf, y);
          return y;
        },
        [this](const blaze::StaticVector<double, N> &q, const blaze::StaticVector<double, m> &wf) -> blaze::StaticMatrix<double, m, N, blaze::columnMajor>
        {
          blaze::StaticMatrix<double, m, N, blaze::columnMajor> Jac;
          this->m_mpc_model->jacobian(q, wf, Jac);
          return Jac;
        });

    blaze::StaticVector<double, m> x;
    blaze::StaticMatrix<double, h, m, blaze::columnMajor> x_ref(0.0);

    auto q_bound = m_mpc_model->getInputPosBounds();
    m_q_min = std::get<0UL>(q_bound);
    m_q_max = std::get<1UL>(q_bound);

    m_q_min[2] *= 1.0;
    m_q_max[2] *= 1.0;


    RCLCPP_INFO(this->get_logger(),
                "q_min: [%.4f, %.4f, %.4f, %.4f]",
                m_q_min[0], m_q_min[1], m_q_min[2], m_q_min[3]);
    RCLCPP_INFO(this->get_logger(),
                "q_max: [%.4f, %.4f, %.4f, %.4f]",
                m_q_max[0], m_q_max[1], m_q_max[2], m_q_max[3]);

    

    auto uMin = m_uMax * -1.0;
    auto u_dotMin = m_u_dotMax * -1.0;
    auto m_margin_u = m_uMax * 0.02;
    auto m_margin_du = m_u_dotMax * 0.0;
    m_mpc->setJointsLimits(m_q_min, m_q_max, uMin, m_uMax, u_dotMin, m_u_dotMax, m_margin_q, m_margin_u, m_margin_du);

    auto Q = k_x_scale * m_Q;
    auto R_u = blaze::pow(m_q_scale, 2.0) * m_R_u;
    auto R_du = blaze::pow(m_q_scale, 2.0) * m_R_du;
    m_mpc->updateWeights(Q, 2.0 * Q, R_u, R_du);

    m_ref = blaze::StaticVector<double, m>({0.00, 0.00, 0.140});

    RCLCPP_INFO(this->get_logger(), "MPC initialized");
  }

  /// @brief update current robot end effector position
  void updateTipPosition(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    if (msg->p.size() == m) // check if not none
    {
      bool all_valid = true;
      for (size_t i = 0; i < m; ++i)
      {
        if (std::isnan(msg->p[i]) || std::isinf(msg->p[i]))
        {
          all_valid = false;
          break;
        }
      }

      if (all_valid)
      {
        for (size_t i = 0; i < m; ++i)
        {
          m_x[i] = msg->p[i];
        }
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "x: %0.2f, %0.2f, %0.2f", m_x[0], m_x[1], m_x[2]);
  }

  /// @brief update current robot joints position
  void updateJointsPosition(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_q = ctr_common::wireToPhysics4(msg->position);

    RCLCPP_DEBUG(this->get_logger(), "q: %0.2f, %0.2f, %0.2f, %0.2f", m_q[0], m_q[1], m_q[2], m_q[3]);
  }

  /// @brief update target position in task space
  void updateTarget(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_ref = blaze::StaticVector<double, m>({msg->p[0], msg->p[1], msg->p[2]});
    RCLCPP_DEBUG(this->get_logger(), "ref: %0.2f, %0.2f, %0.2f", m_ref[0], m_ref[1], m_ref[2]);
  }

  /// @brief Update disterbance distal force (m_wf = f)
  void updateExternalForce(const interfaces::msg::Force::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_wf[0] = msg->x;
    m_wf[1] = msg->y;
    m_wf[2] = msg->z;

    // m_wf[0] = std::clamp(msg->x, -0.25, 0.25);
    // m_wf[1] = std::clamp(msg->y, -0.25, 0.25);
    // m_wf[2] = std::clamp(msg->z, -0.25, 0.25);

    // kill the force for testing
    // m_wf *= 0.0; // Temp
    RCLCPP_DEBUG(this->get_logger(), "u: [%.3f, %.3f, %.3f] N", m_wf[0UL], m_wf[1UL], m_wf[2UL]);
  }

  /// @brief MPC control step
  void mpcStep()
  {
    blaze::StaticMatrix<double, h, m, blaze::columnMajor> ref_h;
    blaze::StaticVector<double, N> q;
    blaze::StaticVector<double, N> u_apply;
    blaze::StaticVector<double, m> x_e(0.0);
    blaze::StaticVector<double, m> x_mpc;

    auto t0 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      q = m_q;
    }

    blaze::StaticVector<double, N> q_meas;
    blaze::StaticVector<double, 3> wf;
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      q_meas = m_q; // raw noisy joints from robot
      wf = m_wf;
    }
    // Kalman filter: fuse integrated model (using m_u_last) with measurement
    q = kalmanUpdateJoints(q_meas, m_u_last, m_sample_time);

    this->m_mpc_model->getPosDistal(q, wf, x_mpc);
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      x_e = (m_x - x_mpc);
      std::cout << std::fixed << std::setprecision(4);
      // std::cout << "x_e: " << x_e[0] << ", " << x_e[1] << ", " << x_e[2] << std::endl;

      // ref_h = circular_ref_trajectory<h, m>(m_t, m_sample_time, true);
      // ref_h = W_ref_trajectory<h, m>(m_t, m_sample_time, true);
      ref_h = infinity_ref_trajectory<h, m>(m_t, m_sample_time, true);
      // ref_h = H_ref_trajectory<h, m>(m_t, m_sample_time, true);
      // ref_h = arc270_ref_trajectory<h, m>(m_t, m_sample_time, true);
      
      m_t += m_sample_time;

      auto msg = interfaces::msg::Taskspace();
      msg.p[0] = ref_h(0, 0);
      msg.p[1] = ref_h(0, 1);
      msg.p[2] = ref_h(0, 2);
      m_publisher_reference->publish(msg);

      // // closed the mpc loop
      // for (size_t i = 0; i < h; ++i)
      // {
      //   ref_h(i, 0) -= x_e[0] * m_error_c;
      //   ref_h(i, 1) -= x_e[1] * m_error_c;
      //   ref_h(i, 2) -= x_e[2] * m_error_c;
      // }
    }

    m_mpc->step(q, wf, ref_h, u_apply);

    m_u_last = u_apply;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    // publishd control signal
    auto msg = interfaces::msg::Jointspace();
    // for (size_t i = 0; i < N; ++i)
    // {
    //   msg.velocity[i] = u_apply[i];
    // }

    msg.velocity[0] = u_apply[2]; // alpha_1
    msg.velocity[1] = u_apply[0]; // beta_1
    msg.velocity[2] = u_apply[3]; // alpha_2
    msg.velocity[3] = u_apply[1]; // beta_2

    m_publisher_control->publish(msg);

    // publish loop time
    auto time_msg = std_msgs::msg::Float64();
    time_msg.data = static_cast<double>(elapsed.count()) * 1.0E-3;
    m_publisher_time->publish(time_msg);

    RCLCPP_DEBUG(this->get_logger(), "q: %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f - x: %0.3f, %0.3f, %0.3f - u: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f - eps: %0.3f [ms]", q[0], q[1], q[2], q[3], q[4], q[5], m_x[0], m_x[1], m_x[2], u_apply[0], u_apply[1], u_apply[2], u_apply[3], u_apply[4], u_apply[5], static_cast<double>(elapsed.count()) * 1e-3);
  }

  /// @brief 1D per-joint Kalman filter: fuse integrated velocity and noisy measurement.
  /// @param q_meas  measured joints (m_q)
  /// @param u_cmd   last commanded velocity (m_u_last)
  /// @param dt      sample time
  /// @return        filtered joint estimate
  blaze::StaticVector<double, N> kalmanUpdateJoints(const blaze::StaticVector<double, N> &q_meas, const blaze::StaticVector<double, N> &u_cmd, double dt)
  {
    blaze::StaticVector<double, N> q_new;

    for (size_t i = 0; i < N; ++i)
    {
      // --- Predict step ---
      // x_k^- = x_{k-1} + dt * u_{k-1}
      double x_pred = m_q_hat[i] + dt * u_cmd[i];

      // P_k^- = P_{k-1} + Q
      double P_pred = m_P[i] + m_q_proc_var;

      // --- Update step ---
      // K_k = P_k^- / (P_k^- + R)
      double S = P_pred + m_q_meas_var;
      double K = (S > 0.0) ? (P_pred / S) : 0.0;

      // x_k = x_k^- + K (z_k - x_k^-)
      double z = q_meas[i];
      double x_upd = x_pred + K * (z - x_pred);

      // P_k = (1 - K) P_k^-
      double P_upd = (1.0 - K) * P_pred;

      // Store
      q_new[i] = x_upd;
      m_q_hat[i] = x_upd;
      m_P[i] = P_upd;
    }

    return q_new;
  }

  void handle_record_response(rclcpp::Client<interfaces::srv::Recording>::SharedFuture future)
  {
    auto response = future.get();
    if (response->success)
    {
      RCLCPP_INFO(this->get_logger(), "Recording started successfully: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", response->message.c_str());
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> circular_ref_trajectory(double t, double dt, bool vary_in_horizon)
{
  blaze::StaticMatrix<double, h, m, blaze::columnMajor> ref_traj(0.0);

  // // Center position
  // const double x0 = 0.00, y0 = -0.00, z0 = 0.140;

  // // Params
  // const double r = 0.040;     // [m]
  // const double A_r = 0.015;   // [m]
  // const double A_z = 0.015;   // [m]
  // const double omega = 0.25;  // [rad/s]
  // const double omega_r = 2.0; // [rad/s]
  // const double omega_z = 1.5; // [rad/s]

  // Center position
  const double x0 = 0.00, y0 = -0.00, z0 = 0.125;

  // Params
  const double r = 0.022;           // [m]
  const double A_r = 0.005;         // [m]
  const double A_z = 0.000;         // [m]
  const double omega = 0.25 * 0.25;  // [rad/s]
  const double omega_r = 1.5 * 0.25; // [rad/s]
  const double omega_z = 1.1 * 0.25; // [rad/s]

  const double theta0 = omega * t;
  const double theta0_r = omega_r * t;
  const double theta0_z = omega_z * t;

  if (vary_in_horizon)
  {
    for (size_t k = 1; k <= h; ++k)
    {
      const double th = theta0 + static_cast<double>(k) * omega * dt;
      const double th_r = theta0_r + static_cast<double>(k) * omega_r * dt;
      const double th_z = theta0_z + static_cast<double>(k) * omega_z * dt;

      // theta = -1.25*pi + 0.5*(1 + sin(-pi/2 + th)) * pi * 1.5
      const double theta =
          -1.25 * M_PI +
          0.5 * (1.0 + std::sin(-M_PI / 2.0 + th)) * M_PI * 1.5;

      const double rad = r + A_r * std::sin(th_r);
      const size_t row = k - 1;

      ref_traj(row, 0) = x0 + rad * std::cos(theta);
      ref_traj(row, 1) = y0 + rad * std::sin(theta);
      ref_traj(row, 2) = z0 - A_z * std::sin(th_z); // note the "-1 *" in Python
    }
  }
  else
  {
    // Hold the same target over the horizon (use k = 0 values)
    const double th = theta0;
    const double th_r = theta0_r;
    const double th_z = theta0_z;

    const double theta =
        -1.25 * M_PI +
        0.5 * (1.0 + std::sin(-M_PI / 2.0 + th)) * M_PI * 1.5;

    const double rad = r + A_r * std::sin(th_r);

    const double x = x0 + rad * std::cos(theta);
    const double y = y0 + rad * std::sin(theta);
    const double z = z0 - A_z * std::sin(th_z);

    for (size_t row = 0; row < h; ++row)
    {
      ref_traj(row, 0) = x;
      ref_traj(row, 1) = y;
      ref_traj(row, 2) = z;
    }
  }

  return ref_traj;
}

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> arc270_ref_trajectory(double t, double dt, bool vary_in_horizon)
{
  blaze::StaticMatrix<double, h, m, blaze::columnMajor> ref_traj(0.0);

  // Center position
  const double x0 = 0.00, y0 = -0.00, z0 = 0.130;

  // Radius [m] - set this value as needed.
  const double R = 0.020;

  // One full out-and-back cycle every 60 seconds.
  const double cycle_sec = 60.0;
  const double arc_span = 1.5 * M_PI;      // 270 deg

  const double theta_start = -arc_span/2 - M_PI/2; // sweep start angle

  const auto point_on_arc = [&](double t_now) -> blaze::StaticVector<double, 3>
  {
    const double phase = std::fmod(t_now / cycle_sec, 1.0);
    const double phase01 = (phase < 0.0) ? (phase + 1.0) : phase;
    // Triangular phase: 0->1 on first half, then 1->0 on second half.
    const double forward_back_progress = (phase01 <= 0.5) ? (2.0 * phase01) : (2.0 * (1.0 - phase01));
    const double theta = theta_start + arc_span * forward_back_progress;
    return blaze::StaticVector<double, 3>{x0 + R * std::cos(theta), y0 + R * std::sin(theta), z0};
  };

  if (vary_in_horizon)
  {
    for (size_t k = 1; k <= h; ++k)
    {
      const double t_k = t + static_cast<double>(k) * dt;
      const auto p = point_on_arc(t_k);
      const size_t row = k - 1;
      ref_traj(row, 0) = p[0];
      ref_traj(row, 1) = p[1];
      ref_traj(row, 2) = p[2];
    }
  }
  else
  {
    const auto p = point_on_arc(t);
    for (size_t row = 0; row < h; ++row)
    {
      ref_traj(row, 0) = p[0];
      ref_traj(row, 1) = p[1];
      ref_traj(row, 2) = p[2];
    }
  }

  return ref_traj;
}

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> infinity_ref_trajectory(double t, double dt, bool vary_in_horizon)
{
  blaze::StaticMatrix<double, h, m, blaze::columnMajor> ref_traj(0.0);

  // Center position
  const double x0 = 0.000, y0 = -0.013, z0 = 0.130;
  t = t + 0.5; // phase shift to start at a specific point on the curve

  // Infinity shape (lemniscate of Gerono) parameters
  const double A_xy = 0.024; // size of the infinity in x-y [m]
  // const double A_z = -0.012;      // small oscillation in z [m]
  const double A_z = -0.00;    // small oscillation in z [m]
  const double omega_xy = 0.1; // angular speed for x-y [rad/s]
  const double omega_z = 0.2;  // angular speed for z [rad/s]

  const double theta0_xy = omega_xy * t;
  const double theta0_z = omega_z * t;

  if (vary_in_horizon)
  {
    // Move along the infinity curve across the horizon
    for (size_t k = 1; k <= h; ++k)
    {
      const double th_xy = theta0_xy + static_cast<double>(k) * omega_xy * dt;
      const double th_z = theta0_z + static_cast<double>(k) * omega_z * dt;

      // Lemniscate of Gerono in x-y:
      // x = A * sin(θ)
      // y = A * sin(θ) * cos(θ)
      const double x = x0 + A_xy * std::sin(th_xy);
      const double y = y0 + A_xy * std::sin(th_xy) * std::cos(th_xy);
      const double z = z0 + A_z * std::sin(th_z);

      const size_t row = k - 1;
      ref_traj(row, 0) = x;
      ref_traj(row, 1) = y;
      ref_traj(row, 2) = z;
    }
  }
  else
  {
    // Hold the same target (at current time t) over the whole horizon
    const double th_xy = theta0_xy;
    const double th_z = theta0_z;

    const double x = x0 + A_xy * std::sin(th_xy);
    const double y = y0 + A_xy * std::sin(th_xy) * std::cos(th_xy);
    const double z = z0 + A_z * std::sin(th_z);

    for (size_t row = 0; row < h; ++row)
    {
      ref_traj(row, 0) = x;
      ref_traj(row, 1) = y;
      ref_traj(row, 2) = z;
    }
  }

  return ref_traj;
}

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> W_ref_trajectory(double t, double dt, bool vary_in_horizon)
{
  blaze::StaticMatrix<double, h, m, blaze::columnMajor> ref_traj(0.0);
  double width = 0.050;
  double outer_height = 0.040;
  double inner_height = 0.015;
  double inner_offset_down = 0.010;
  double stroke = 0.005;

  // Center and base z (matched to the Python implementation)
  const double x0 = 0.00;
  const double y0 = -0.010;
  const double z0 = 0.132;

  // One full loop every 60 seconds
  const double omega = 1.0 / 60.0;

  // Symmetric W centerline points
  const double x_left = x0 - width / 2.0;
  const double x_lmid = x0 - width / 4.0;
  const double x_center = x0;
  const double x_rmid = x0 + width / 4.0;
  const double x_right = x0 + width / 2.0;

  const double outer_center = y0;
  const double y_outer_top = outer_center + outer_height / 2.0;

  const double inner_center = y0 - inner_offset_down;
  const double y_inner_top = inner_center + inner_height / 2.0;
  const double y_inner_bot = inner_center - inner_height / 2.0;

  using Pt2 = std::array<double, 2>;
  const std::array<Pt2, 5> centerline = {
      Pt2{x_left, y_outer_top},
      Pt2{x_lmid, y_inner_bot},
      Pt2{x_center, y_inner_top},
      Pt2{x_rmid, y_inner_bot},
      Pt2{x_right, y_outer_top}};

  const auto add = [](const Pt2 &a, const Pt2 &b) -> Pt2
  { return Pt2{a[0] + b[0], a[1] + b[1]}; };
  const auto sub = [](const Pt2 &a, const Pt2 &b) -> Pt2
  { return Pt2{a[0] - b[0], a[1] - b[1]}; };
  const auto mul = [](const Pt2 &a, double s) -> Pt2
  { return Pt2{a[0] * s, a[1] * s}; };
  const auto norm = [](const Pt2 &a) -> double
  { return std::sqrt(a[0] * a[0] + a[1] * a[1]); };
  const auto cross2 = [](const Pt2 &a, const Pt2 &b) -> double
  { return a[0] * b[1] - a[1] * b[0]; };

  const double half_stroke = 0.5 * stroke;
  std::array<Pt2, 4> tangents{};
  std::array<Pt2, 4> normals{}; // left normals

  for (size_t i = 0; i < 4; ++i)
  {
    Pt2 d = sub(centerline[i + 1], centerline[i]);
    const double len = std::max(norm(d), 1e-12);
    tangents[i] = mul(d, 1.0 / len);
    normals[i] = Pt2{-tangents[i][1], tangents[i][0]};
  }

  const auto line_intersection = [&](const Pt2 &p, const Pt2 &r, const Pt2 &q, const Pt2 &s) -> Pt2
  {
    const double rxs = cross2(r, s);
    if (std::abs(rxs) < 1e-12)
    {
      return mul(add(p, q), 0.5);
    }
    const Pt2 qp = sub(q, p);
    const double tpar = cross2(qp, s) / rxs;
    return add(p, mul(r, tpar));
  };

  const Pt2 start_left = add(centerline[0], mul(normals[0], half_stroke));
  const Pt2 start_right = sub(centerline[0], mul(normals[0], half_stroke));
  const Pt2 end_left = add(centerline[4], mul(normals[3], half_stroke));
  const Pt2 end_right = sub(centerline[4], mul(normals[3], half_stroke));

  Pt2 left_j1 = line_intersection(add(centerline[1], mul(normals[0], half_stroke)), tangents[0],
                                  add(centerline[1], mul(normals[1], half_stroke)), tangents[1]);
  Pt2 left_j2 = line_intersection(add(centerline[2], mul(normals[1], half_stroke)), tangents[1],
                                  add(centerline[2], mul(normals[2], half_stroke)), tangents[2]);
  Pt2 left_j3 = line_intersection(add(centerline[3], mul(normals[2], half_stroke)), tangents[2],
                                  add(centerline[3], mul(normals[3], half_stroke)), tangents[3]);

  Pt2 right_j1 = line_intersection(sub(centerline[1], mul(normals[0], half_stroke)), tangents[0],
                                   sub(centerline[1], mul(normals[1], half_stroke)), tangents[1]);
  Pt2 right_j2 = line_intersection(sub(centerline[2], mul(normals[1], half_stroke)), tangents[1],
                                   sub(centerline[2], mul(normals[2], half_stroke)), tangents[2]);
  Pt2 right_j3 = line_intersection(sub(centerline[3], mul(normals[2], half_stroke)), tangents[2],
                                   sub(centerline[3], mul(normals[3], half_stroke)), tangents[3]);

  std::vector<Pt2> vertices;
  vertices.reserve(11);
  vertices.push_back(start_left);
  vertices.push_back(left_j1);
  vertices.push_back(left_j2);
  vertices.push_back(left_j3);
  vertices.push_back(end_left);
  vertices.push_back(end_right);
  vertices.push_back(right_j3);
  vertices.push_back(right_j2);
  vertices.push_back(right_j1);
  vertices.push_back(start_right);
  vertices.push_back(start_left); // close loop

  std::vector<double> seg_lengths;
  seg_lengths.reserve(vertices.size() - 1);
  std::vector<double> cum_lengths(vertices.size(), 0.0);
  double total_length = 0.0;
  for (size_t i = 0; i + 1 < vertices.size(); ++i)
  {
    const double len = norm(sub(vertices[i + 1], vertices[i]));
    seg_lengths.push_back(len);
    total_length += len;
    cum_lengths[i + 1] = total_length;
  }

  const auto point_on_W = [&](double phi) -> blaze::StaticVector<double, 3>
  {
    phi = std::fmod(phi, 1.0);
    if (phi < 0.0)
      phi += 1.0;
    const double d = phi * total_length;

    size_t i = 0;
    for (; i + 1 < cum_lengths.size(); ++i)
    {
      if (d <= cum_lengths[i + 1])
        break;
    }
    i = std::min(i, seg_lengths.size() - 1);

    const Pt2 &seg_start = vertices[i];
    const Pt2 &seg_end = vertices[i + 1];
    const double seg_len = seg_lengths[i];

    Pt2 xy = seg_start;
    if (seg_len > 1e-9)
    {
      const double u = (d - cum_lengths[i]) / seg_len;
      xy = add(seg_start, mul(sub(seg_end, seg_start), u));
    }
    return blaze::StaticVector<double, 3>{xy[0], xy[1], z0};
  };

  if (vary_in_horizon)
  {
    for (size_t k = 1; k <= h; ++k)
    {
      const double t_k = t + static_cast<double>(k) * dt;
      const double phi_k = std::fmod(omega * t_k, 1.0);
      const auto p = point_on_W(phi_k);
      const size_t row = k - 1;
      ref_traj(row, 0) = p[0];
      ref_traj(row, 1) = p[1];
      ref_traj(row, 2) = p[2];
    }
  }
  else
  {
    const double phi = std::fmod(omega * t, 1.0);
    const auto p = point_on_W(phi);
    for (size_t row = 0; row < h; ++row)
    {
      ref_traj(row, 0) = p[0];
      ref_traj(row, 1) = p[1];
      ref_traj(row, 2) = p[2];
    }
  }

  return ref_traj;
}

template <size_t h, size_t m>
blaze::StaticMatrix<double, h, m, blaze::columnMajor> H_ref_trajectory(double t, double dt, bool vary_in_horizon)
{
  blaze::StaticMatrix<double, h, m, blaze::columnMajor> ref_traj(0.0);

  const double x0 = 0.00;
  const double y0 = -0.009;
  const double z0 = 0.130;

  const double H_height = 0.036;
  const double H_width = 0.030;
  const double stroke = 0.007;
  const double bar_offset_down = 0.002;

  const double omega = 1.0 / 60.0;

  const double y_bot = y0 - H_height / 2.0;
  const double y_top = y0 + H_height / 2.0;
  const double y_bar_center = y0 - bar_offset_down;
  const double y_bar_bot = y_bar_center - stroke / 2.0;
  const double y_bar_top = y_bar_center + stroke / 2.0;

  const double xL_center = x0 - H_width / 2.0;
  const double xR_center = x0 + H_width / 2.0;

  const double xL_outer = xL_center - stroke / 2.0;
  const double xL_inner = xL_center + stroke / 2.0;
  const double xR_inner = xR_center - stroke / 2.0;
  const double xR_outer = xR_center + stroke / 2.0;

  using Pt2 = std::array<double, 2>;
  const std::array<Pt2, 13> vertices = {
      Pt2{xL_outer, y_bot},
      Pt2{xL_outer, y_top},
      Pt2{xL_inner, y_top},
      Pt2{xL_inner, y_bar_top},
      Pt2{xR_inner, y_bar_top},
      Pt2{xR_inner, y_top},
      Pt2{xR_outer, y_top},
      Pt2{xR_outer, y_bot},
      Pt2{xR_inner, y_bot},
      Pt2{xR_inner, y_bar_bot},
      Pt2{xL_inner, y_bar_bot},
      Pt2{xL_inner, y_bot},
      Pt2{xL_outer, y_bot}};

  const auto sub = [](const Pt2 &a, const Pt2 &b) -> Pt2
  { return Pt2{a[0] - b[0], a[1] - b[1]}; };
  const auto add = [](const Pt2 &a, const Pt2 &b) -> Pt2
  { return Pt2{a[0] + b[0], a[1] + b[1]}; };
  const auto mul = [](const Pt2 &a, double s) -> Pt2
  { return Pt2{a[0] * s, a[1] * s}; };
  const auto norm = [](const Pt2 &a) -> double
  { return std::sqrt(a[0] * a[0] + a[1] * a[1]); };

  std::array<double, vertices.size() - 1> seg_lengths{};
  std::array<double, vertices.size()> cum_lengths{};
  double total_length = 0.0;
  cum_lengths[0] = 0.0;

  for (size_t i = 0; i + 1 < vertices.size(); ++i)
  {
    seg_lengths[i] = norm(sub(vertices[i + 1], vertices[i]));
    total_length += seg_lengths[i];
    cum_lengths[i + 1] = total_length;
  }

  const auto point_on_H = [&](double phi) -> blaze::StaticVector<double, 3>
  {
    phi = std::fmod(phi, 1.0);
    if (phi < 0.0)
      phi += 1.0;

    const double d = phi * total_length;
    auto upper = std::upper_bound(cum_lengths.begin(), cum_lengths.end(), d);
    size_t i = static_cast<size_t>(std::distance(cum_lengths.begin(), upper));
    i = (i == 0) ? 0 : i - 1;
    i = std::min(i, seg_lengths.size() - 1);

    const Pt2 &seg_start = vertices[i];
    const Pt2 &seg_end = vertices[i + 1];
    const double seg_len = seg_lengths[i];

    Pt2 xy = seg_start;
    if (seg_len > 1e-9)
    {
      const double u = (d - cum_lengths[i]) / seg_len;
      xy = add(seg_start, mul(sub(seg_end, seg_start), u));
    }

    return blaze::StaticVector<double, 3>{xy[0], xy[1], z0};
  };

  if (vary_in_horizon)
  {
    for (size_t k = 1; k <= h; ++k)
    {
      const double t_k = t + static_cast<double>(k) * dt;
      const double phi_k = std::fmod(omega * t_k, 1.0);
      const auto p = point_on_H(phi_k);
      const size_t row = k - 1;
      ref_traj(row, 0) = p[0];
      ref_traj(row, 1) = p[1];
      ref_traj(row, 2) = p[2];
    }
  }
  else
  {
    const double phi = std::fmod(omega * t, 1.0);
    const auto p = point_on_H(phi);
    for (size_t row = 0; row < h; ++row)
    {
      ref_traj(row, 0) = p[0];
      ref_traj(row, 1) = p[1];
      ref_traj(row, 2) = p[2];
    }
  }

  return ref_traj;
}



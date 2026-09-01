#include <chrono>
#include <blaze/Blaze.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/force.hpp"
#include "interfaces/msg/ekf_residual.hpp"

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
#include "ctr_common/finite_guard.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "robot/quat_utils.hpp"
using namespace robot_quat;
#include "ctr_common/runtime_paths.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


static constexpr bool use_orientation = true;
static constexpr bool exclude_roll = true;

class KalmanFilterNode : public rclcpp::Node
{
protected:
  static constexpr size_t nu = 4UL; // joints (beta1, beta2, beta3, alpha1, alpha2, alpha3)
  static constexpr size_t nx = 3UL; // states F (x,y,z)
  static constexpr size_t ny = 7UL; // measurements if 3 (x,y,z) if 7 (x,y,z) + (quaternion)

private:
  double m_sample_time; // EKF loop sample time

  // EKF model
  const std::string m_packageName = "robot";
  std::shared_ptr<PINNs<nu>> m_pinn;
  std::string m_model_name;

  // EKF
  blaze::StaticVector<double, nx> m_Q_vec; // Task space weights
  blaze::StaticVector<double, ny> m_x;     // Tip position
  blaze::StaticVector<double, nu> m_q;     // Joints config

  blaze::StaticMatrix<double, 6UL, 6UL> m_R; // measurement noise covariance (top-left block used when !exclude_roll; 5x5 when exclude_roll)

  double m_f_dot;
  double m_force_threshold = 0.50; // N
  bool m_first_measurement_received = false;
  // The tip measurement is not the only prerequisite for a correction: the
  // JOINTS have to be real too. Bringup starts the EM tracker 14 s before the
  // robot nodes, so tip poses always arrive first and every early correction
  // used to run on the q0 seed rather than on hardware feedback.
  bool m_first_joint_received = false;

  blaze::StaticVector<double, nx> m_x_pred;
  blaze::StaticMatrix<double, nx, nx> m_A;
  blaze::StaticMatrix<double, nx, nx> m_P; // state covariance
  blaze::StaticMatrix<double, nx, nx> m_Q; // process noise covariance

  // Dataset-native joint window for the FK queries, same source and same
  // relative-beta1 convention pinn_fk_node uses. Without this the EKF queried
  // the network outside its trained box whenever feedback left the dataset
  // range, which is extrapolation dressed up as a prediction.
  blaze::StaticVector<double, nu> m_q_min, m_q_max;

  // Last force estimate that was finite, so a rejected update never publishes a
  // non-finite value downstream (pinn_fk, the planner and the MPC all feed this
  // straight into the PINN as a network input).
  blaze::StaticVector<double, nx> m_force_last_good{0.0, 0.0, 0.0};

  rclcpp::TimerBase::SharedPtr m_control_timer;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_EKF;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_publisher_time;
  rclcpp::Publisher<interfaces::msg::EKFResidual>::SharedPtr m_publisher_residual_error;
  rclcpp::Publisher<interfaces::msg::Force>::SharedPtr m_publisher_observer;                  // publisher object
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joint_feedback; // subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_task_feedback;   // subscriber object
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  mutable std::mutex m_state_mutex;

public:
  KalmanFilterNode() : Node("kalman_filter_node")
  {
    KalmanFilterNode::initNodeParameters();
    KalmanFilterNode::setupDynamicParameterUpdates();
    KalmanFilterNode::setupRosInterfaces();
    KalmanFilterNode::initKalmanFilter();
  }

private:
  /// @brief Function to declare and initialize ROS parameters - parameters values can be set from the launch file
  void initNodeParameters()
  {
    declare_parameter<double>("sample_time", 25E-3);
    m_sample_time = get_parameter("sample_time").as_double();

    declare_parameter<std::string>("model_name", "ctr_8x91_0.18_tanh_9K_9K_50K_v3");
    m_model_name = get_parameter("model_name").as_string();

    declare_parameter<double>("f_dot", 0.2); // N/s
    m_f_dot = get_parameter("f_dot").as_double();

    declare_parameter<std::vector<double>>("R", {2.85e-09, 1.58e-08, 3.94e-09, 1.0e-6, 1.0e-6, 1.0e-6});
    std::vector<double> R_flat = get_parameter("R").as_double_array();

    if (R_flat.size() == 6UL)
    {
      RCLCPP_INFO(this->get_logger(), "Using diagonal R measurement noise (6 elements)");
      blaze::StaticVector<double, 6UL> R_vec;
      reset(R_vec);
      for (size_t i = 0; i < R_vec.size(); ++i)
      {
        if (i < R_flat.size())
        {
          R_vec[i] = R_flat[i];
        }
      }

      // If we receive 6 orientation terms but run x-y only orientation residual, ignore roll term.
      if constexpr (exclude_roll)
      {
        R_vec[5UL] = 0.0;
      }
      updateMeasurementNoiseCovariance(R_vec);
    }
    else if (R_flat.size() == 36UL)
    {
      RCLCPP_INFO(this->get_logger(), "Using full 6x6 R measurement noise matrix (36 elements)");
      blaze::StaticMatrix<double, 6UL, 6UL> R;
      reset(R);
      // Convert flat vector to blaze matrix (row-major)
      for (size_t i = 0; i < 6UL; ++i)
      {
        for (size_t j = 0; j < 6UL; ++j)
        {
          R(i, j) = R_flat[i * 6UL + j];
        }
      }

      if constexpr (exclude_roll)
      {
        // Drop roll channel in x-y orientation mode.
        for (size_t i = 0; i < 6UL; ++i)
        {
          R(5UL, i) = 0.0;
          R(i, 5UL) = 0.0;
        }
      }

      updateMeasurementNoiseCovariance(R);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "R must have %zu elements (diagonal) or %zu elements (full matrix 6x6), got %zu", 6UL, 36UL, R_flat.size());
      throw std::runtime_error("Invalid R dimension");
    }

    // Physics order [beta1, beta2, alpha1, alpha2]; same default as
    // pinn_fk_node's q0. The old all-zero seed was INFEASIBLE (beta2 = 0 is
    // outside the trained [-0.072, -0.034], and beta1 - beta2 = 0 is outside
    // [-0.084, -0.030]), so the first FK queries of every run were made deep in
    // extrapolation. Only the tip measurement was gated, and bringup gives the
    // EM tracker a 14 s head start, so those queries always happened.
    declare_parameter<std::vector<double>>("q0", {-0.100, -0.055, 0.0, 0.0});
    const std::vector<double> q0 = get_parameter("q0").as_double_array();
    if (q0.size() != nu)
    {
      RCLCPP_ERROR(this->get_logger(), "q0 must have %zu elements, got %zu", nu, q0.size());
      throw std::runtime_error("Invalid q0 dimension");
    }
    if (!ctr_common::allFinite(q0))
    {
      RCLCPP_ERROR(this->get_logger(), "q0 must be finite");
      throw std::runtime_error("Non-finite q0");
    }

    // Initialize with NaNs so EKF can skip update until first valid measurement.
    for (size_t i = 0; i < ny; ++i)
    {
      m_x[i] = std::numeric_limits<double>::quiet_NaN();
    }
    for (size_t i = 0; i < nu; ++i)
    {
      m_q[i] = q0[i];
    }
    for (size_t i = 0; i < nx; ++i)
    {
      m_x_pred[i] = 0.0;
    }
  }

  /// @brief setup ROS Publisher, Subscribers, and Timers
  void setupRosInterfaces()
  {
    // Create callback groups
    m_callback_group_EKF = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // subscriber to receive Model Output feedback
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    subs_options_2.callback_group = m_callback_group_sub1;
    m_subscription_task_feedback = this->create_subscription<interfaces::msg::Taskspace>(
        "/task_space/feedback/base_tool", rclcpp::QoS(10), std::bind(&KalmanFilterNode::updateTipPosition, this, _1), subs_options_2);

    // subscriber to receive Jointspace feedback
    auto subs_options_3 = rclcpp::SubscriptionOptions();
    subs_options_3.callback_group = m_callback_group_sub1;
    m_subscription_joint_feedback = this->create_subscription<interfaces::msg::Jointspace>(
        "joint_space/feedback", rclcpp::QoS(10), std::bind(&KalmanFilterNode::updateJointsPosition, this, _1), subs_options_3);

    // publisher to publish estiamted tip force
    m_publisher_observer = this->create_publisher<interfaces::msg::Force>("task_space/force_estimate", 10);

    // publisher to publish computation time
    m_publisher_time = this->create_publisher<std_msgs::msg::Float64>("EKF/computation_time", 10);

    // publisher to publish EKF residual error
    m_publisher_residual_error = this->create_publisher<interfaces::msg::EKFResidual>("EKF/residual_error", 10);

    // Create wall timers with different callback groups
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_control_timer = this->create_wall_timer(control_sample_time, std::bind(&KalmanFilterNode::EKFStep, this), m_callback_group_EKF);
  }

  /// @brief Initialize Extended Kalman Filter
  void initKalmanFilter()
  {
    if (m_model_name.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Model name is not set!");
      throw std::runtime_error("Model name parameter is required");
    }

    const size_t backbonePoints = 20UL;
    RCLCPP_INFO(this->get_logger(), "Model: %s", m_model_name.c_str());

    // instantiate EKF model object
    m_pinn = std::make_shared<PINNs<nu>>(ctr_common::resolveModelsDir(*this).string(), m_model_name, 1UL, backbonePoints);

    // Dataset-native ranges, exactly as pinn_fk_node resolves them:
    // clampJointPositions() re-applies the tube coupling by shifting beta1's
    // window by the live beta2, so it needs the RELATIVE window that
    // getDatasetInputRanges() returns, not the absolute box from
    // getInputPosBounds().
    std::tie(m_q_min, m_q_max) = m_pinn->getDatasetInputRanges();

    const double alpha = 1.0;
    m_A = alpha * blaze::IdentityMatrix<double>(nx);

    double sigma_df = m_f_dot * m_sample_time;
    m_Q_vec = blaze::StaticVector<double, nx>(1.0) * sigma_df * sigma_df;
    updateProcessNoiseCovariance(m_Q_vec);

    // Initialize error covariance matrix
    m_P = blaze::IdentityMatrix<double>(nx) * 1e-4;
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
        if (name == "f_dot")
        {
          m_f_dot = parameter.as_double();
          double sigma_df = m_f_dot * m_sample_time;
          m_Q_vec = blaze::StaticVector<double, nx>(1.0) * sigma_df * sigma_df;
          updateProcessNoiseCovariance(m_Q_vec);
          RCLCPP_INFO(this->get_logger(), "Updated f_dot: %f N/s", m_f_dot);
        }
      }

      return result;
    };

    m_param_callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  /// @brief update current robot end effector position
  void updateTipPosition(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    for (size_t i = 0; i < 3UL; ++i)
    {
      m_x[i] = msg->p[i]; // convert from mm to meters
    }
    for (size_t i = 0; i < 4UL; ++i)
    {
      m_x[3UL + i] = msg->h[i]; // quaternion
    }
    // Was a zero-norm test, which is blind to the case that actually occurs:
    // EMTracker::ToolData2QuatTransform writes a NaN quaternion when the tool or
    // robot sensor frame is missing, and `NaN < 1e-12` is false. quatIsUsable()
    // tests finiteness AND norm. The measurement is stored either way -- EKFStep
    // decides per channel what to do with it -- so this stays a warning, not a
    // filter.
    if (!ctr_common::quatIsUsable(msg->h))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Tip orientation measurement is unusable (non-finite or zero-norm) "
                           "[%.4f, %.4f, %.4f, %.4f] - falling back to the position-only correction",
                           msg->h[0], msg->h[1], msg->h[2], msg->h[3]);
    }
    if (!ctr_common::allFinite(msg->p))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Tip position measurement is non-finite [%.4f, %.4f, %.4f] - skipping the EKF correction",
                           msg->p[0], msg->p[1], msg->p[2]);
    }
    m_first_measurement_received = true;
    RCLCPP_DEBUG(this->get_logger(), "x: %0.4f, %0.4f, %0.4f, q: %0.4f, %0.4f, %0.4f, %0.4f", m_x[0], m_x[1], m_x[2], m_x[3], m_x[4], m_x[5], m_x[6]);
  }

  /// @brief update current robot joints position
  void updateJointsPosition(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    if (!ctr_common::allFinite(msg->position))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Non-finite joint feedback - holding the previous joint vector. "
                           "A NaN here would silently poison every FK query.");
      return;
    }

    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_q = ctr_common::wireToPhysics4(msg->position);
    m_first_joint_received = true;

    RCLCPP_DEBUG(this->get_logger(), "q: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", m_q[0], m_q[1], m_q[2], m_q[3], m_q[4], m_q[5]);
  }

  /// @brief Update process noise covariance matrices
  void updateProcessNoiseCovariance(blaze::StaticVector<double, 3UL> Q_diag)
  {
    for (size_t i = 0; i < nx; ++i)
    {
      m_Q(i, i) = Q_diag[i];
    }
    RCLCPP_INFO(this->get_logger(), "Updated process noise covariance:");
    logMatrix("Q", m_Q);
  }

  /// @brief Update measurement noise covariance matrices
  template <size_t N>
  void updateMeasurementNoiseCovariance(blaze::StaticVector<double, N> R_diag)
  {
    static_assert(N == 3UL || N == 6UL, "R_diag must have size 3 (position only) or 6 (position + full orientation)");

    // --- Update R ---
    reset(m_R);
    for (size_t i = 0; i < N; ++i)
    {
      m_R(i, i) = R_diag[i];
    }

    RCLCPP_INFO(this->get_logger(), "Updated measurement noise covariance:");
    logMatrix("R", m_R);
  }

  /// @brief Update measurement noise covariance matrices
  template <size_t N>
  void updateMeasurementNoiseCovariance(const blaze::StaticMatrix<double, N, N> &R)
  {
    static_assert(N == 3UL || N == 6UL, "R must be 3x3 (position only) or 6x6 (position + full orientation)");

    // --- Update R ---
    reset(m_R);

    for (size_t i = 0; i < N; ++i)
    {
      for (size_t j = 0; j < N; ++j)
      {
        m_R(i, j) = R(i, j);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Updated measurement noise covariance:");
    logMatrix("R", m_R);
  }

  /// @brief Finite-difference Jacobian of measurement model w.r.t. force
  void jacobian_wrt_wf_fd(const blaze::StaticVector<double, nu> &q, const blaze::StaticVector<double, nx> &wf, blaze::StaticMatrix<double, 6UL, nx> &H, double eps = 1e-5)
  {
    // Initialize Jacobian to zero
    H = blaze::StaticMatrix<double, 6UL, nx>(0.0);

    // Nominal prediction
    blaze::StaticVector<double, ny> z0;
    this->m_pinn->getPosDistal(q, wf, z0);

    blaze::StaticVector<double, 3UL> pos0 = blaze::subvector(z0, 0UL, 3UL);
    blaze::StaticVector<double, 4UL> quat0 = blaze::subvector(z0, 3UL, 4UL);

    // Normalize quaternion
    double norm_q0 = blaze::norm(quat0);
    if (norm_q0 > 1e-10)
      quat0 /= norm_q0;

    for (size_t i = 0; i < nx; ++i)
    {
      blaze::StaticVector<double, nx> wf_plus = wf;
      blaze::StaticVector<double, nx> wf_minus = wf;

      wf_plus[i] += eps;
      wf_minus[i] -= eps;

      // Forward and backward perturbations
      blaze::StaticVector<double, ny> z_plus, z_minus;
      this->m_pinn->getPosDistal(q, wf_plus, z_plus);
      this->m_pinn->getPosDistal(q, wf_minus, z_minus);

      // Extract positions
      blaze::StaticVector<double, 3UL> pos_plus = blaze::subvector(z_plus, 0UL, 3UL);
      blaze::StaticVector<double, 3UL> pos_minus = blaze::subvector(z_minus, 0UL, 3UL);

      // Extract and normalize quaternions
      blaze::StaticVector<double, 4UL> quat_plus = blaze::subvector(z_plus, 3UL, 4UL);
      blaze::StaticVector<double, 4UL> quat_minus = blaze::subvector(z_minus, 3UL, 4UL);

      double norm_qp = blaze::norm(quat_plus);
      double norm_qm = blaze::norm(quat_minus);
      if (norm_qp > 1e-10)
        quat_plus /= norm_qp;
      if (norm_qm > 1e-10)
        quat_minus /= norm_qm;

      // Position derivative
      blaze::StaticVector<double, 3UL> dpos = (pos_plus - pos_minus) / (2.0 * eps);

      // Orientation derivative via quaternion error
      blaze::StaticVector<double, 4UL> quat_err_plus = quat_multiply(quat_inverse(quat0), quat_plus);
      blaze::StaticVector<double, 4UL> quat_err_minus = quat_multiply(quat_inverse(quat0), quat_minus);

      blaze::StaticVector<double, 3UL> rot_plus = quat_to_rotvec(quat_err_plus);
      blaze::StaticVector<double, 3UL> rot_minus = quat_to_rotvec(quat_err_minus);

      blaze::StaticVector<double, 3UL> drot = (rot_plus - rot_minus) / (2.0 * eps);

      // Assign to Jacobian (element-by-element)
      for (size_t j = 0; j < 3UL; ++j)
      {
        H(j, i) = dpos[j];
      }
      for (size_t j = 0; j < 3UL; ++j)
      {
        H(3UL + j, i) = drot[j];
      }
    }
  }

  /// @brief Finite-difference Jacobian of measurement model w.r.t. force using only orientation x-y components
  void jacobian_wrt_wf_fd_xy(const blaze::StaticVector<double, nu> &q, const blaze::StaticVector<double, nx> &wf, blaze::StaticMatrix<double, 5UL, nx> &H, double eps = 1e-5)
  {
    H = blaze::StaticMatrix<double, 5UL, nx>(0.0);

    blaze::StaticVector<double, ny> z0;
    this->m_pinn->getPosDistal(q, wf, z0);

    blaze::StaticVector<double, 4UL> quat0 = blaze::subvector(z0, 3UL, 4UL);
    const double norm_q0 = blaze::norm(quat0);
    if (norm_q0 > 1e-10)
    {
      quat0 /= norm_q0;
    }

    for (size_t i = 0; i < nx; ++i)
    {
      blaze::StaticVector<double, nx> wf_plus = wf;
      blaze::StaticVector<double, nx> wf_minus = wf;
      wf_plus[i] += eps;
      wf_minus[i] -= eps;

      blaze::StaticVector<double, ny> z_plus, z_minus;
      this->m_pinn->getPosDistal(q, wf_plus, z_plus);
      this->m_pinn->getPosDistal(q, wf_minus, z_minus);

      const blaze::StaticVector<double, 3UL> pos_plus = blaze::subvector(z_plus, 0UL, 3UL);
      const blaze::StaticVector<double, 3UL> pos_minus = blaze::subvector(z_minus, 0UL, 3UL);
      const blaze::StaticVector<double, 3UL> dpos = (pos_plus - pos_minus) / (2.0 * eps);

      blaze::StaticVector<double, 4UL> quat_plus = blaze::subvector(z_plus, 3UL, 4UL);
      blaze::StaticVector<double, 4UL> quat_minus = blaze::subvector(z_minus, 3UL, 4UL);

      const double norm_qp = blaze::norm(quat_plus);
      const double norm_qm = blaze::norm(quat_minus);
      if (norm_qp > 1e-10)
      {
        quat_plus /= norm_qp;
      }
      if (norm_qm > 1e-10)
      {
        quat_minus /= norm_qm;
      }

      // Keep perturbations in the same hemisphere to avoid branch jumps in finite differences.
      if (blaze::dot(quat_plus, quat0) < 0.0)
      {
        quat_plus = -quat_plus;
      }
      if (blaze::dot(quat_minus, quat0) < 0.0)
      {
        quat_minus = -quat_minus;
      }

      blaze::StaticVector<double, 4UL> q_err_plus = quat_multiply(quat_inverse(quat0), quat_plus);
      blaze::StaticVector<double, 4UL> q_err_minus = quat_multiply(quat_inverse(quat0), quat_minus);

      blaze::StaticVector<double, 3UL> rot_plus = quat_to_rotvec(q_err_plus);
      blaze::StaticVector<double, 3UL> rot_minus = quat_to_rotvec(q_err_minus);
      const blaze::StaticVector<double, 3UL> drot = (rot_plus - rot_minus) / (2.0 * eps);

      H(0UL, i) = dpos[0UL];
      H(1UL, i) = dpos[1UL];
      H(2UL, i) = dpos[2UL];
      H(3UL, i) = drot[0UL];
      H(4UL, i) = drot[1UL];
    }
  }

  // ---------------------------------------------------------------------------
  // Measurement validity, PER CHANNEL.
  //
  // This replaced an all-or-nothing measurementIsMissing() that returned true
  // only when all SEVEN components were non-finite. That is precisely the hole
  // the 2026-09-01 incident went through: the EM tracker's translation is
  // NaN-masked by a Butterworth filter but its rotation was not, so a missing
  // sensor frame produced a finite position beside a NaN quaternion. One finite
  // element made the whole vector "present", the NaN entered the correction, and
  // the force state was NaN for the rest of the session.
  // ---------------------------------------------------------------------------

  /// @brief True when the position channel of a measurement can be used.
  bool positionIsUsable(const blaze::StaticVector<double, ny> &z) const
  {
    return ctr_common::allFinite(blaze::subvector(z, 0UL, 3UL));
  }

  /// @brief True when the orientation channel of a measurement can be used.
  ///        Checked against BOTH the measurement and the prediction, because
  ///        quat_inverse() is applied to the prediction: a degenerate predicted
  ///        quaternion would divide by ~0 there.
  bool orientationIsUsable(const blaze::StaticVector<double, ny> &z_meas,
                           const blaze::StaticVector<double, ny> &z_pred) const
  {
    return ctr_common::quatIsUsable(blaze::subvector(z_meas, 3UL, 4UL)) &&
           ctr_common::quatIsUsable(blaze::subvector(z_pred, 3UL, 4UL));
  }

  /// @brief Common Kalman update: innovation covariance, damped solve, gain, and
  ///        a GUARDED commit.
  ///
  /// The commit guard is what makes a NaN self-limiting. The new state and
  /// covariance are computed into locals and written back only when both are
  /// finite; otherwise the filter keeps its previous state and this cycle is
  /// simply lost. Guarding here rather than at the inputs also covers the two
  /// failure modes no input check can see: a non-finite H out of the
  /// finite-difference Jacobian, and a blaze::solve() on a singular S.
  ///
  /// Before this, `m_x_pred = m_x_pred + L * r` was committed unconditionally,
  /// m_P went non-finite with it, and since m_A is the identity both reproduced
  /// the NaN forever with no reset path.
  template <size_t nr, bool SO>
  bool applyCorrection(const blaze::StaticVector<double, nr> &r,
                       const blaze::StaticMatrix<double, nr, nx, SO> &H,
                       const blaze::StaticMatrix<double, nr, nr> &R)
  {
    blaze::StaticMatrix<double, nr, nr> S = H * m_P * blaze::trans(H) + R;

    // Damping
    const double lambda = 1e-6;
    for (size_t i = 0; i < nr; ++i)
    {
      S(i, i) += lambda;
    }

    // Solve instead of explicit inverse
    const blaze::StaticMatrix<double, nx, nr> PHt = m_P * blaze::trans(H);
    const blaze::StaticMatrix<double, nr, nx> Kt = blaze::solve(S, blaze::trans(PHt));

    // Kalman gain
    const blaze::StaticMatrix<double, nx, nr> L = blaze::trans(Kt);

    // --- 2b) State estimation measurement correction ---
    const blaze::StaticVector<double, nx> x_new = m_x_pred + L * r;

    // --- 2c) Estimation-error covariance measurement update (Joseph form) ---
    //
    // `temp` is materialised into a concrete matrix rather than left as `auto`.
    // Blaze expression templates capture their operands by reference, so
    // `auto temp = IdentityMatrix<double>(nx) - L * H;` binds a reference to the
    // IdentityMatrix temporary and then uses it in a later statement, after that
    // temporary has died. The three call sites this replaced all did that.
    const blaze::StaticMatrix<double, nx, nx> temp =
        blaze::IdentityMatrix<double>(nx) - L * H;
    const blaze::StaticMatrix<double, nx, nx> P_new =
        temp * m_P * blaze::trans(temp) + L * R * blaze::trans(L);

    if (!ctr_common::allFinite(x_new) || !ctr_common::allFinite(P_new))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "EKF correction rejected: the update is not finite (residual finite: %d, "
                           "gain finite: %d). Holding the previous force estimate and covariance.",
                           static_cast<int>(ctr_common::allFinite(r)),
                           static_cast<int>(ctr_common::allFinite(L)));
      return false;
    }

    m_x_pred = x_new;
    m_P = P_new;
    return true;
  }

  /// @brief Position-only correction (3 rows). Also the fallback whenever the
  ///        orientation channel is unusable, which keeps the force estimate
  ///        tracking through a dropped EM frame instead of losing the session.
  bool correctPositionOnly(const blaze::StaticVector<double, nu> &q,
                           const blaze::StaticVector<double, ny> &z_meas,
                           const blaze::StaticVector<double, ny> &z_pred,
                           blaze::StaticVector<double, 3UL> &pos_residual)
  {
    blaze::StaticVector<double, 3UL> r;                         // residual
    blaze::StaticMatrix<double, 3UL, nx, blaze::columnMajor> H; // measurement Jacobian
    const blaze::StaticMatrix<double, 3UL, 3UL> R = blaze::submatrix(m_R, 0UL, 0UL, 3UL, 3UL);

    // Position residual
    r = blaze::subvector(z_meas, 0UL, 3UL) - blaze::subvector(z_pred, 0UL, 3UL);
    for (size_t i = 0; i < 3UL; ++i)
    {
      pos_residual[i] = r[i];
    }

    this->m_pinn->jacobian_wrt_force(q, m_x_pred, H);

    return applyCorrection(r, H, R);
  }

  /// @brief Full correction using position and orientation. Callers must have
  ///        established orientationIsUsable() first, so the normalisations below
  ///        cannot divide by zero and quat_inverse() cannot blow up.
  bool correctPositionAndOrientation(const blaze::StaticVector<double, nu> &q,
                                     const blaze::StaticVector<double, ny> &z_meas,
                                     const blaze::StaticVector<double, ny> &z_pred,
                                     blaze::StaticVector<double, 3UL> &pos_residual,
                                     blaze::StaticVector<double, 3UL> &rot_residual_rad)
  {
    blaze::StaticVector<double, 4UL> quat_meas = blaze::subvector(z_meas, 3UL, 4UL);
    blaze::StaticVector<double, 4UL> quat_pred = blaze::subvector(z_pred, 3UL, 4UL);

    // Normalize. Unconditional: quatIsUsable() guarantees norm >= 1e-6.
    quat_meas /= blaze::norm(quat_meas);
    quat_pred /= blaze::norm(quat_pred);

    // Align measurement sign to prediction to avoid antipodal jumps.
    if (blaze::dot(quat_meas, quat_pred) < 0.0)
    {
      quat_meas = -quat_meas;
    }

    const blaze::StaticVector<double, 4UL> quat_err = quat_multiply(quat_inverse(quat_pred), quat_meas);
    const blaze::StaticVector<double, 3UL> r_rot = quat_to_rotvec(quat_err);

    if constexpr (!exclude_roll)
    {
      blaze::StaticVector<double, 6UL> r;            // residual
      blaze::StaticMatrix<double, 6UL, nx> H;        // measurement Jacobian
      blaze::StaticMatrix<double, 6UL, 6UL> R = m_R; // measurement noise covariance

      blaze::subvector(r, 0UL, 3UL) = blaze::subvector(z_meas, 0UL, 3UL) - blaze::subvector(z_pred, 0UL, 3UL);
      blaze::subvector(r, 3UL, 3UL) = r_rot;
      for (size_t i = 0; i < 3UL; ++i)
      {
        pos_residual[i] = r[i];
        rot_residual_rad[i] = r_rot[i];
      }

      jacobian_wrt_wf_fd(q, m_x_pred, H);

      return applyCorrection(r, H, R);
    }
    else
    {
      blaze::StaticVector<double, 5UL> r;      // residual [pos(3), rotvec_xy(2)]
      blaze::StaticMatrix<double, 5UL, nx> H;  // measurement Jacobian
      blaze::StaticMatrix<double, 5UL, 5UL> R; // active R block (position + orientation x-y)

      reset(R);
      for (size_t i = 0; i < 5UL; ++i)
      {
        for (size_t j = 0; j < 5UL; ++j)
        {
          R(i, j) = m_R(i, j);
        }
      }

      blaze::subvector(r, 0UL, 3UL) = blaze::subvector(z_meas, 0UL, 3UL) - blaze::subvector(z_pred, 0UL, 3UL);
      r[3UL] = r_rot[0UL];
      r[4UL] = r_rot[1UL];
      for (size_t i = 0; i < 3UL; ++i)
      {
        pos_residual[i] = r[i];
        rot_residual_rad[i] = r_rot[i];
      }

      jacobian_wrt_wf_fd_xy(q, m_x_pred, H);

      return applyCorrection(r, H, R);
    }
  }

  /// @brief EKF step
  void EKFStep()
  {

    auto t0 = std::chrono::high_resolution_clock::now();

    blaze::StaticVector<double, nu> q;                 // current joint positions
    blaze::StaticVector<double, ny> z_meas;            // current measurements
    blaze::StaticVector<double, ny> z_pred;            // predicted measurements
    const blaze::StaticMatrix<double, nx, nx> Q = m_Q; // local snapshot of process noise covariance
    blaze::StaticVector<double, 3UL> pos_residual;
    blaze::StaticVector<double, 3UL> rot_residual_rad;
    bool first_measurement_received = false;
    bool first_joint_received = false;

    for (size_t i = 0; i < 3UL; ++i)
    {
      pos_residual[i] = std::numeric_limits<double>::quiet_NaN();
      rot_residual_rad[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // make sure to copy the current measurement and input to avoid race conditions
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      q = m_q;
      z_meas = m_x;
      first_measurement_received = m_first_measurement_received;
      first_joint_received = m_first_joint_received;
    }

    // Clamp the LOCAL copy into the dataset range before any FK query, exactly
    // as pinn_fk_node does. The two nodes evaluate the same network and must not
    // disagree about what a legal query is; without this the EKF extrapolated
    // whenever feedback left the trained box.
    const blaze::StaticVector<double, nu> q_raw = q;
    ctr_common::clampJointPositions(q, m_q_min, m_q_max, this->get_logger());
    if (blaze::maxNorm(q - q_raw) > 1e-9)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "EKF FK input clamped to the dataset range [b1, b2, a1, a2]: "
                           "[%.4f, %.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f, %.4f]",
                           q_raw[0], q_raw[1], q_raw[2], q_raw[3], q[0], q[1], q[2], q[3]);
    }

    // 1a) States prediction time update (Gauss–Markov)
    m_x_pred = m_A * m_x_pred; // TODO: may need to remove the alpha scaling

    // 1b) Predict-error covariance time update
    m_P = m_A * m_P * blaze::trans(m_A) + Q;

    // 1c) Prediction system output
    this->m_pinn->getPosDistal(q, m_x_pred, z_pred);

    // Correction, gated PER CHANNEL so a dropped EM frame degrades one channel
    // for one cycle instead of poisoning the filter for the whole session.
    if (!first_joint_received)
    {
      // The prediction is still running on the q0 seed: bringup starts the EM
      // tracker 14 s ahead of the robot nodes, so tip poses arrive long before
      // any joint feedback. Correcting here would fit the force to a
      // configuration the robot is not in.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for joint_space/feedback before correcting - the force estimate stays at its "
                           "initial value until the robot node publishes joint feedback.");
    }
    else if (first_measurement_received && positionIsUsable(z_meas))
    {
      if constexpr (use_orientation)
      {
        if (orientationIsUsable(z_meas, z_pred))
        {
          correctPositionAndOrientation(q, z_meas, z_pred, pos_residual, rot_residual_rad);
        }
        else
        {
          // Position is good, orientation is not. Previously this ingested the
          // NaN quaternion and latched the filter; now it degrades gracefully.
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Orientation channel unusable this cycle - applying the position-only correction.");
          correctPositionOnly(q, z_meas, z_pred, pos_residual);
        }
      }
      else
      {
        correctPositionOnly(q, z_meas, z_pred, pos_residual);
      }
    }

    // Clip the estimate to the force magnitude the model was trained on
    // (dataset f_max = 0.5 N). Clipping the VECTOR NORM, not each component:
    // per-component clipping at 0.5 admitted ||f|| up to 0.5*sqrt(3) = 0.87 N,
    // outside the trained box, and the resulting direction was not preserved.
    //
    // Note this can no longer scrub a non-finite value, and never could: the old
    // `if (magnitude > m_force_threshold)` is false for NaN. Finiteness is
    // enforced at the commit in applyCorrection() and again before publishing.
    const double force_norm = blaze::norm(m_x_pred);
    if (std::isfinite(force_norm) && force_norm > m_force_threshold)
    {
      m_x_pred *= (m_force_threshold / force_norm);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    // publish force estimate
    //
    // Last line of defence. Every consumer of this topic feeds the value
    // straight into the PINN as a network input (pinn_fk, the planner's IK and
    // FTL cost, the MPC's QP), so a single non-finite sample here becomes a
    // NaN tip pose, a NaN IK residual that passes every `<` threshold, and NaN
    // joint velocity commands. Publish the last good value instead.
    if (ctr_common::allFinite(m_x_pred))
    {
      m_force_last_good = m_x_pred;
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Force estimate is not finite - publishing the last good value "
                            "[%.4f, %.4f, %.4f] N. The filter state is corrupt; restart ekf_node.",
                            m_force_last_good[0], m_force_last_good[1], m_force_last_good[2]);
    }

    auto msg = interfaces::msg::Force();
    msg.x = m_force_last_good[0];
    msg.y = m_force_last_good[1];
    msg.z = m_force_last_good[2];
    msg.magnitude = blaze::norm(m_force_last_good);
    m_publisher_observer->publish(msg);

    // publish residual error
    auto residual_msg = interfaces::msg::EKFResidual();
    residual_msg.x = std::numeric_limits<double>::quiet_NaN();
    residual_msg.y = std::numeric_limits<double>::quiet_NaN();
    residual_msg.z = std::numeric_limits<double>::quiet_NaN();
    residual_msg.pos_mag = std::numeric_limits<double>::quiet_NaN();
    residual_msg.theta_x = std::numeric_limits<double>::quiet_NaN();
    residual_msg.theta_y = std::numeric_limits<double>::quiet_NaN();
    residual_msg.theta_z = std::numeric_limits<double>::quiet_NaN();
    residual_msg.orientation_mag = std::numeric_limits<double>::quiet_NaN();

    bool pos_finite = true;
    bool rot_finite = true;
    for (size_t i = 0; i < 3UL; ++i)
    {
      pos_finite = pos_finite && std::isfinite(pos_residual[i]);
      rot_finite = rot_finite && std::isfinite(rot_residual_rad[i]);
    }

    residual_msg.x = pos_residual[0UL];
    residual_msg.y = pos_residual[1UL];
    residual_msg.z = pos_residual[2UL];
    residual_msg.theta_x = rot_residual_rad[0UL] * (180.0 / M_PI);
    residual_msg.theta_y = rot_residual_rad[1UL] * (180.0 / M_PI);
    residual_msg.theta_z = rot_residual_rad[2UL] * (180.0 / M_PI);

    if (pos_finite)
    {
      residual_msg.pos_mag = blaze::norm(pos_residual);
    }
    if (rot_finite)
    {
      residual_msg.orientation_mag = blaze::norm(rot_residual_rad) * (180.0 / M_PI);
    }
    m_publisher_residual_error->publish(residual_msg);

    // publish loop time
    auto time_msg = std_msgs::msg::Float64();
    time_msg.data = static_cast<double>(elapsed.count()) * 1.0E-3;
    m_publisher_time->publish(time_msg);

    // RCLCPP_INFO(this->get_logger(), "wf: %0.3f, %0.3f, %0.3f - eps: %0.3f [ms]", m_x_pred[0], m_x_pred[1], m_x_pred[2], static_cast<double>(elapsed.count()) * 1e-3);
  }

  template <size_t R, size_t C>
  void logMatrix(const std::string &name, const blaze::StaticMatrix<double, R, C> &M)
  {
    std::ostringstream oss;
    oss << name << " =\n"
        << M;
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }
};

// Quaternion multiplication: computes Hamilton product of q and r | Format: [w, x, y, z]
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilterNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

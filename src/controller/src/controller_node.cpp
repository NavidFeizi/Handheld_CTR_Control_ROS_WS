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
#include "interfaces/action/jointstarget.hpp"
#include "CTR.hpp"
#include <memory.h>
#include <numeric>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Controller : public rclcpp::Node
{
public:
  // default class constructor
  Controller() : Node("ctr_control"), count_(0)
  {
    Controller::declare_parameters();
    Controller::setup_ros_interfaces();

    // node initialization time
    rclcpp::Time now = this->get_clock()->now();
    m_t_init = static_cast<double>(now.nanoseconds()) / 1.00E9;

    /*
     ===================================================================================
     =========================== initializing the CTR object ===========================
     ===================================================================================
    */

    constexpr double inf = std::numeric_limits<double>::infinity();

    //  # # # # # # # # ---- Properties of Nitinol Tubes ---- # # # # # # # #
    // Young's modulus GPa
    constexpr double E1 = 30.00E9;
    constexpr double E2 = 30.00E9;
    constexpr double E3 = 74.9122E9;
    // Poisson's ratio
    constexpr double nu = 0.300;
    // Shear modulus
    constexpr double G1 = E1 / (2.00 * (1.00 + nu));
    constexpr double G2 = E2 / (2.00 * (1.00 + nu));
    constexpr double G3 = E3 / (2.00 * (1.00 + nu));

    // Precurvature radii for the tubes
    constexpr double R1 = 41.00E-3; // (4.1cm curvature radius)
    constexpr double R2 = 95.00E-3; // (9.5 cm curvature radius)
    constexpr double R3 = inf;      // (infinite curvature radius)

    // -- ** -- Precurvature vectors (for curved portions of the tubes) -- ** -- [u_x* u_y* 0]
    constexpr blaze::StaticVector<double, 3UL> u1 = {1.00 / R1, 0.00, 0.00};
    constexpr blaze::StaticVector<double, 3UL> u2 = {1.00 / R2, 0.00, 0.00};
    constexpr blaze::StaticVector<double, 3UL> u3 = {1.00 / R3, 0.00, 0.00};

    // --** --Lengths of the tubes' straight sections (meters) -- ** --
    constexpr blaze::StaticVector<double, 3UL> ls = {151.00E-3, 61.00E-3, 62.00E-3};

    // --** --Lengths of the tubes' curved sections (meters) -- ** --
    constexpr blaze::StaticVector<double, 3UL> lc = {58.00E-3, 78.00E-3, 0.00};

    // --** --Outer and Inner diameters of the tubes (meters)--** --
    constexpr blaze::StaticVector<double, 3UL> ID = {0.737E-3, 0.965E-3, 1.1448E-3};
    constexpr blaze::StaticVector<double, 3UL> OD = {0.940E-3, 1.372E-3, 2.045E-3};

    // # # # # # ---- Instantiating the three Tube objects ---- # # # # #
    std::shared_ptr<Tube> T1 = std::make_shared<Tube>(OD[0UL], ID[0UL], E1, G1, ls[0UL], lc[0UL], u1); // innermost tube
    std::shared_ptr<Tube> T2 = std::make_shared<Tube>(OD[1UL], ID[1UL], E2, G2, ls[1UL], lc[1UL], u2); // intermediate tube
    std::shared_ptr<Tube> T3 = std::make_shared<Tube>(OD[2UL], ID[2UL], E3, G3, ls[2UL], lc[2UL], u3); // outermost tube

    // instantiating an array of smart pointers to CTR component tubes
    std::array<std::shared_ptr<Tube>, 3UL> Tb = {T1, T2, T3};

    // home position of the handheld CTR
    this->m_q = {-109.00E-3, -39.00E-3, 0.00, 0.00, 0.00, 0.00};

    // Determining the accuracy of BVP solutions
    constexpr double Tol = 1.00E-6;

    // iniritalizing the vector of initial guesses
    this->m_initGuess = 0.00;

    // instantiating the CTR object for control
    this->m_robot = std::make_shared<CTR>(Tb,
                                          this->m_q,
                                          Tol,
                                          mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON,
                                          this->m_linearActuatorThickness);

    // updating the EM alignment of the CTR, as it won't be identical to the surgical plan
    rclcpp::sleep_for(2s);
    Controller::updateExperimentalEMPose();
  }

  // class destructor
  ~Controller() = default;

  /**
   * @brief Reads the EM sensor data and updates current the pose of the handheld CTR in the EM space
   */
  void updateExperimentalEMPose()
  {
    blaze::StaticMatrix<double, 4UL, 4UL, blaze::columnMajor> H;
    H(3UL, 3UL) = 1.00;

    // getting the orientation of the handheld CTR
    blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> R;
    mathOp::getSO3(m_rot_phantom_base, R);

    // std::cout << "m_rot_phantom_base" << m_rot_phantom_base << std::endl;
    // std::cout << "m_tran_phantom_base" << m_tran_phantom_base << std::endl;

    blaze::submatrix<0UL, 0UL, 3UL, 3UL>(H) = R;

    // getting the position of the handheld CTR
    // blaze::submatrix<0UL, 3UL, 3UL, 1UL>(H) = m_tran_phantom_base;
    blaze::submatrix<0UL, 3UL, 3UL, 1UL>(H);
    H(0, 3) = m_tran_phantom_base[0];
    H(1, 3) = m_tran_phantom_base[1];
    H(2, 3) = m_tran_phantom_base[2];

    this->m_H = blaze::inv(H);

    // accounting for the 10 mm retraction of the CTR
    static constexpr blaze::StaticVector<double, 3UL> retraction{0.00, 0.00, 10.00E-3};
    // blaze::submatrix<0UL, 3UL, 3UL, 1UL>(this->m_H) += retraction;
    m_H(0, 3) += retraction[0];
    m_H(1, 3) += retraction[1];
    m_H(2, 3) += retraction[2];

    // std::cout << "m_H" << m_H << std::endl;

    m_robot->setHomogeneousTransformation(this->m_H);
  }

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    // Set default parameters and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("control_sample_time", 4.00E-3);
    m_control_sample_time = this->get_parameter("control_sample_time").as_double();
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
    m_callback_group_sub_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub_1;
    m_subscription_base_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_base_tool", rclcpp::QoS(10), std::bind(&Controller::update_current_tip_in_base, this, _1), subs_options_1);

    // Subscriber to receive ModelOutput feedback
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    subs_options_2.callback_group = m_callback_group_sub_2;
    m_subscription_phantom_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_phantom_tool", rclcpp::QoS(10), std::bind(&Controller::update_current_tip_in_phantom, this, _1), subs_options_2);

    // Subscriber to receive ModelOutput feedback
    auto subs_options_3 = rclcpp::SubscriptionOptions();
    subs_options_3.callback_group = m_callback_group_sub_3;
    m_subscription_phantom_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_phantom_base", rclcpp::QoS(10), std::bind(&Controller::update_current_base_in_phantom, this, _1), subs_options_3);

    // Action Server Setup
    m_action_server = rclcpp_action::create_server<interfaces::action::Target>(
        this, "navigate_to_target",
        std::bind(&Controller::handle_goal, this, _1, _2),
        std::bind(&Controller::handle_cancel, this, _1),
        std::bind(&Controller::handle_accepted, this, _1));

    // Target action
    m_action_client = rclcpp_action::create_client<interfaces::action::Jointstarget>(this, "joints_target");
    if (!m_action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Joint target action server not available");
    }
  }

  /**
   * @brief Update current tool (tip) position in the phantom frame.
   */
  void update_current_tip_in_phantom(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_tran_endEffector = {msg->p[0UL],
                          msg->p[1UL],
                          msg->p[2UL]};
  }

  /**
   * @brief Update current tool (tip) position in the base frame.
   */
  void update_current_tip_in_base(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_tran_base_endEffector = {msg->p[0UL],
                               msg->p[1UL],
                               msg->p[2UL]};
  }

  /**
   * @brief Update current tool (tip) position in the phantom frame.
   */
  void update_current_base_in_phantom(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_tran_phantom_base = {msg->p[0UL],
                           msg->p[1UL],
                           msg->p[2UL]};
    m_rot_phantom_base = {msg->h[0UL],
                          msg->h[1UL],
                          msg->h[2UL],
                          msg->h[3UL]};
  }

  //
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const interfaces::action::Target::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Target request received ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  //
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  //
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    std::thread{std::bind(&Controller::set_target_callback, this, std::placeholders::_1), goal_handle}.detach();
  }

  /**
   * @brief Service callback to set the target.
   */
  void set_target_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> &goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    m_target_position = {goal->target_pose[0UL], goal->target_pose[1UL], goal->target_pose[2UL]};
    m_calyxPosition = {goal->calyx_pose[0UL], goal->calyx_pose[1UL], goal->calyx_pose[2UL]};
    RCLCPP_INFO(this->get_logger(), "Task target received: x: %0.3f   y: %0.3f  z: %0.3f [mm]", m_target_position[0] * 1e3, m_target_position[1] * 1e3, m_target_position[2] * 1e3);
    Controller::updateExperimentalEMPose();

    auto result = std::make_shared<interfaces::action::Target::Result>();

    // Call the control loop
    Controller::control_loop();

    result->success = true; // Indicate success in the result
    goal_handle->succeed(result);
    // RCLCPP_INFO(this->get_logger(), "Target reached successfully");
  }

  /**
   * @brief Main control loop to calculate the control and publish the joint space values.
   */
  void control_loop()
  {
    // the targets on the CSV file are in the CT-Scan's coord frame. Need to convert to CTR coord frame for control
    const blaze::StaticVector<double, 4UL> auxVec = {this->m_target_position[0UL], this->m_target_position[1UL], this->m_target_position[2UL], 1.00};
    const blaze::StaticVector<double, 4UL> convertedTarget = this->m_H * auxVec;

    const auto t0 = std::chrono::high_resolution_clock::now();

    std::cout << "Target: " << blaze::trans(convertedTarget);

    this->m_robot->posCTRL(this->m_initGuess, blaze::subvector<0UL, 3UL>(convertedTarget), this->m_pos_tol);

    std::cout << "TipPos: " << blaze::trans(m_robot->getTipPos()) << std::endl;

    this->m_q = this->m_robot->getConfiguration();

    blaze::StaticVector<double, 4UL> posOffsets = {0.0, -147.0E-3, 0.0, -77.0E-3};
    blaze::StaticVector<double, 4UL> pose_in_robot_system;
    // Placeholder joint space values
    interfaces::msg::Jointspace msg;
    pose_in_robot_system[0UL] = m_q[3UL] - posOffsets[0UL]; // inner tube --> rotation
    pose_in_robot_system[1UL] = m_q[0UL] - posOffsets[1UL]; // inner tube --> translation
    pose_in_robot_system[2UL] = m_q[4UL] - posOffsets[2UL]; // middle tube --> rotation
    pose_in_robot_system[3UL] = m_q[1UL] - posOffsets[3UL]; // middle tube --> translation

    msg.position[0UL] = pose_in_robot_system[0UL]; // inner tube --> rotation
    msg.position[1UL] = pose_in_robot_system[1UL]; // inner tube --> translation
    msg.position[2UL] = pose_in_robot_system[2UL]; // middle tube --> rotation
    msg.position[3UL] = pose_in_robot_system[3UL]; // middle tube --> translation

    // Publish the control signal
    // m_publisher_control->publish(msg);

    Controller::request_target_action(pose_in_robot_system);

    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);

    RCLCPP_INFO(this->get_logger(), "joints target sent to robot node: R_in:%0.3f [deg]  T_in:%0.3f [mm]  R_mid:%0.3f [deg]  T_mid: %0.3f [mm] | elapsed: %0.3f [ms]",
                msg.position[0UL] * 360 / M_PI, msg.position[1UL] * 1e3, msg.position[2UL] * 360 / M_PI, msg.position[3UL] * 1e3, elapsed.count() * 1.00E-3);

    // Wait for action to complete
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]()
               { return action_done_; });
    }

    // Check if the action was successful
    if (!action_success_)
    {
      RCLCPP_ERROR(this->get_logger(), "Control loop aborted due to action failure.");
      return;
    }

    return;
  }

  //
  void request_target_action(blaze::StaticVector<double, 4UL> target)
  {
    auto goal_msg = interfaces::action::Jointstarget::Goal();
    goal_msg.position[0UL] = target[0UL]; // inner tube --> rotation
    goal_msg.position[1UL] = target[1UL]; // inner tube --> translation
    goal_msg.position[2UL] = target[2UL]; // middle tube --> rotation
    goal_msg.position[3UL] = target[3UL]; // middle tube --> translation

    auto send_goal_options = rclcpp_action::Client<interfaces::action::Jointstarget>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&Controller::action_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Controller::action_result_callback, this, _1);

    m_action_client->async_send_goal(goal_msg, send_goal_options);
    // RCLCPP_INFO(this->get_logger(), "Joints target sent");
  }

  //
  void action_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<interfaces::action::Jointstarget>> goal_handle, const std::shared_ptr<const interfaces::action::Jointstarget::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "feedback callback called");
  }

  //
  void action_result_callback(const rclcpp_action::ClientGoalHandle<interfaces::action::Jointstarget>::WrappedResult &result)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(this->get_logger(), "Target reached successfully");
        action_success_ = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach target");
        action_success_ = false;
      }
      action_done_ = true;
    }
    cv_.notify_one();
  }

private:
  // Member variables
  size_t count_;
  double m_t_init = 0.00;
  double m_control_sample_time;
  bool m_flag_new_feedback = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  blaze::StaticVector<double, 3UL> m_tran_endEffector, m_tran_base_endEffector, m_tran_phantom_base, m_target_position, m_calyxPosition;
  blaze::StaticVector<double, 4UL> m_rot_phantom_base;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub_1, m_callback_group_sub_2, m_callback_group_sub_3; // Callback group for running subscriber callback function on separate thread
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_control;                           // Publisher object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_base_tool;                    // Subscriber object
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_phantom_tool;                 // Subscriber object
  rclcpp_action::Server<interfaces::action::Target>::SharedPtr m_action_server;
  rclcpp_action::Client<interfaces::action::Jointstarget>::SharedPtr m_action_client;

  std::condition_variable cv_;
  std::mutex mutex_;
  bool action_done_ = false;
  bool action_success_ = false;

  /*
     =============================================================================================
     =========================== Setting the CTR object as data member ===========================
     =============================================================================================
  */
  std::shared_ptr<CTR> m_robot;
  blaze::StaticVector<double, 5UL> m_initGuess;      // initial guess for the solution of the BVP
  blaze::StaticVector<double, 6UL> m_q;              // joint values of the CTR
  const double m_linearActuatorThickness = 30.00E-3; // thickness of the linear actuator stages --> collision avoidance
  const double m_pos_tol = 1.00E-3;
  // Selected surgical plan
  blaze::HybridMatrix<double, 4UL, 4UL, blaze::columnMajor> m_H;
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

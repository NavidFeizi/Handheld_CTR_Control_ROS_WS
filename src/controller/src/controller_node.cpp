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
#include "interfaces/srv/transformation.hpp"
#include "interfaces/srv/jointstarget.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <filesystem>

#include "CTR.hpp"
#include <memory.h>
#include <numeric>

#include <thread>

#include <mutex>
#include <condition_variable>

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

    std::string package_name = "controller";
    std::string workspace_directory = ament_index_cpp::get_package_share_directory(package_name);
    auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    std::string folder_address = workspace_directory + "/../../../../Output_Files/" + ss.str();
    std::filesystem::create_directories(folder_address);

    m_output_file.open(folder_address + "/" + "Results.csv");
    if (!m_output_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Robot.csv");
      return;
    }
    m_output_file << "tgt_x,tgt_y,tgt_z,e_x,e_y,e_z" << "\n";

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
    this->m_CTR_robot = std::make_shared<CTR>(Tb,
                                          this->m_q,
                                          Tol,
                                          mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON,
                                          this->m_linearActuatorThickness);
    
    // Homeogeneous transformation: CT-SCAN -->> CTR coordinate frame
    m_H = {{0.3282276588, 0.3156448091, 0.8903004878, -0.1685793917},
           {0.2948837319, 0.8611736556, -0.4140332349, 0.1204215318},
           {-0.8973907670, 0.3984322897, 0.1895824935, 0.0950790488},
           {0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000}};

    this->request_base_transformation();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    m_H = blaze::inv(m_ct_ctr_tranformation); // update H with the measurement from the setup

    std::cout << "m_H: \n"
              << m_H << std::endl;
    this->m_CTR_robot->setHomogeneousTransformation(m_H);

    constexpr blaze::StaticVector<double, 6UL> init_conf = {-0.083473,-0.0516,0,-0.647814,-1.49396,0};   // need to be corrected and grab it from the manager
    this->m_CTR_robot->actuate_CTR(this->m_initGuess, init_conf);     

    // // updating the EM alignment of the CTR, as it won't be identical to the surgical plan
    // rclcpp::sleep_for(2s);
    // Controller::updateExperimentalEMPose();
  }

  // class destructor
  ~Controller()
  {
    m_output_file.close();
  }

  // Reads the EM sensor data and updates current pose of the handheld CTR in the EM space
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

    // this->m_H = blaze::inv(H);

    // // accounting for the 10 mm retraction of the CTR
    // static constexpr blaze::StaticVector<double, 3UL> retraction{0.00, 0.00, 20.00E-3};
    // // blaze::submatrix<0UL, 3UL, 3UL, 1UL>(this->m_H) += retraction;
    // m_H(0, 3) += retraction[0];
    // m_H(1, 3) += retraction[1];
    // m_H(2, 3) += retraction[2];

    // std::cout << "m_H" << m_H << std::endl;

    // m_robot->setHomogeneousTransformation(this->m_H);
  }

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    // Set default parameters and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("control_sample_time", 4.00E-3);
    m_control_sample_time = this->get_parameter("control_sample_time").as_double();
  }

  // Setup ROS interfaces, including publishers, subscribers, and services.
  void setup_ros_interfaces()
  {
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1.00E6));
    // Create callback groups
    m_callback_group_sub_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_sub_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_callback_group_sub_1;
    m_subscription_base_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_base_tool", rclcpp::QoS(10), std::bind(&Controller::update_end_effector, this, _1), subs_options_1);

    // Subscriber to receive ModelOutput feedback
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    subs_options_2.callback_group = m_callback_group_sub_2;
    m_subscription_phantom_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_phantom_tool", rclcpp::QoS(10), std::bind(&Controller::update_current_tip_in_phantom_callback, this, _1), subs_options_2);

    // Subscriber to receive ModelOutput feedback
    auto subs_options_3 = rclcpp::SubscriptionOptions();
    subs_options_3.callback_group = m_callback_group_sub_3;
    m_subscription_phantom_tool = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_phantom_base", rclcpp::QoS(10), std::bind(&Controller::update_current_base_in_phantom_callback, this, _1), subs_options_3);

    // Service client to receive the measured tranformation of the CTR frame in the CT frame
    m_tranformation_client = this->create_client<interfaces::srv::Transformation>("get_transformation");

    // Task space Action server
    m_action_server = rclcpp_action::create_server<interfaces::action::Target>(
        this, "navigate_to_target",
        std::bind(&Controller::handle_goal, this, _1, _2),
        std::bind(&Controller::handle_cancel, this, _1),
        std::bind(&Controller::handle_accepted, this, _1));

    // Joint space target Action client
    m_target_service = this->create_client<interfaces::srv::Jointstarget>("joint_space/target");

    // Publisher to publish control signal - depreciated and updated to Joins space target Action
    m_publisher_control = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation", 10);
  }

  // Update current tool (tip) position in the phantom frame.
  void update_current_tip_in_phantom_callback(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_tran_endEffector = {msg->p[0UL],
                          msg->p[1UL],
                          msg->p[2UL]};
  }

  // Update current tool (tip) position in the base frame.
  void update_end_effector(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_tran_base_endEffector = {msg->p[0UL],
                               msg->p[1UL],
                               msg->p[2UL]};
  }

  // Update current tool (tip) position in the phantom frame.
  void update_current_base_in_phantom_callback(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_tran_phantom_base = {msg->p[0UL],
                           msg->p[1UL],
                           msg->p[2UL]};
    m_rot_phantom_base = {msg->h[0UL],
                          msg->h[1UL],
                          msg->h[2UL],
                          msg->h[3UL]};
  }

  // request CTR base (robot) tranformarion in the CT frame from the EMtracker node -  this is a blocking process
  void request_base_transformation()
  {
    auto request = std::make_shared<interfaces::srv::Transformation::Request>();
    // Optionally, you can set parameters in the request if needed

    while (!m_tranformation_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
    }

    auto result_future = m_tranformation_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result_future.get();
      // Handle response directly here if not using a separate callback
      handle_base_tranformation_response(response);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

  // handle the recived service response
  void handle_base_tranformation_response(std::shared_ptr<interfaces::srv::Transformation::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received transformation matrix:");
    // assuming m_ct_ctr_tranformation is defined and accessible within this scope.
    m_ct_ctr_tranformation(0, 0) = response->transformation_matrix[0];
    m_ct_ctr_tranformation(0, 1) = response->transformation_matrix[1];
    m_ct_ctr_tranformation(0, 2) = response->transformation_matrix[2];
    m_ct_ctr_tranformation(0, 3) = response->transformation_matrix[3];
    m_ct_ctr_tranformation(1, 0) = response->transformation_matrix[4];
    m_ct_ctr_tranformation(1, 1) = response->transformation_matrix[5];
    m_ct_ctr_tranformation(1, 2) = response->transformation_matrix[6];
    m_ct_ctr_tranformation(1, 3) = response->transformation_matrix[7];
    m_ct_ctr_tranformation(2, 0) = response->transformation_matrix[8];
    m_ct_ctr_tranformation(2, 1) = response->transformation_matrix[9];
    m_ct_ctr_tranformation(2, 2) = response->transformation_matrix[10];
    m_ct_ctr_tranformation(2, 3) = response->transformation_matrix[11];
    m_ct_ctr_tranformation(3, 0) = 0.0;
    m_ct_ctr_tranformation(3, 1) = 0.0;
    m_ct_ctr_tranformation(3, 2) = 0.0;
    m_ct_ctr_tranformation(3, 3) = 1.0;
  }

  // Task space Action server
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const interfaces::action::Target::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Target request received ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Task space Action server
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Task space Action server
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> goal_handle)
  {
    std::thread{std::bind(&Controller::set_target_callback, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Action service callback to set the target.
  void set_target_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Target>> &goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    m_target_position = {goal->target_pose[0UL], goal->target_pose[1UL], goal->target_pose[2UL]};
    m_calyxPosition = {goal->calyx_pose[0UL], goal->calyx_pose[1UL], goal->calyx_pose[2UL]};
    RCLCPP_INFO(this->get_logger(), "\n Task target received: x: %0.3f   y: %0.3f  z: %0.3f [mm]",
                m_target_position[0] * 1e3, m_target_position[1] * 1e3, m_target_position[2] * 1e3);
    // Controller::updateExperimentalEMPose();

    auto result = std::make_shared<interfaces::action::Target::Result>();

    // Call the control loop for n times
    blaze::StaticVector<double, 3UL> error;
    for (int i = 0; i < 10; i++)
    {
      // introduces a delay in case there's a NAN in the EM data
      while (!blaze::isfinite(this->m_target_position))
      {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
      error = Controller::control_loop();
    }

    m_output_file << std::fixed << std::setprecision(4)
                  << m_target_position[0] << ',' << m_target_position[1] << ',' << m_target_position[2] << ','
                  << error[0] << ',' << error[1] << ',' << error[2] << '\n';

    result->success = true; // Indicate success in the result
    goal_handle->succeed(result);
    // RCLCPP_INFO(this->get_logger(), "Target reached successfully");
  }

  // brief Main control loop to calculate the control and publish the joint space values.
  blaze::StaticVector<double, 3UL> control_loop()
  {
    // the targets on the CSV file are in the CT-Scan's coord frame. Need to convert to CTR coord frame for control
    const blaze::StaticVector<double, 4UL> auxVec = {this->m_target_position[0UL], this->m_target_position[1UL], this->m_target_position[2UL], 1.00};
    const blaze::StaticVector<double, 4UL> convertedTarget = this->m_H * auxVec;
    const blaze::StaticVector<double, 3UL> m_target_position_slicer = {convertedTarget[0UL], convertedTarget[1UL], convertedTarget[2UL]};

    const blaze::StaticVector<double, 4UL> auxVecCalyx = {this->m_calyxPosition[0UL], this->m_calyxPosition[1UL], this->m_calyxPosition[2UL], 1.00};
    const blaze::StaticVector<double, 4UL> convertedCalyx = this->m_H * auxVecCalyx;
    const blaze::StaticVector<double, 3UL> m_calyx_position_slicer = {convertedCalyx[0UL], convertedCalyx[1UL], convertedCalyx[2UL]};

    const auto t0 = std::chrono::high_resolution_clock::now();

    this->m_CTR_robot->broadcastTargetToSlicer(m_target_position_slicer);
    this->m_CTR_robot->broadcastCalyxToSlicer(m_calyx_position_slicer);

    this->m_CTR_robot->posCTRL(this->m_initGuess, blaze::subvector<0UL, 3UL>(convertedTarget), this->m_pos_tol, m_tran_base_endEffector);
    this->m_CTR_robot->broadcastShapeToSlicer();



    this->m_q = this->m_CTR_robot->getConfiguration();
    RCLCPP_INFO(this->get_logger(), "m_q: q[0]:%0.6f  q[1]:%0.6f  q[2]:%0.6f  q[3]: %0.6f  q[4]: %0.6f  q[5]: %0.6f",
               m_q[0UL], m_q[1UL], m_q[2UL], m_q[3UL], m_q[4UL], m_q[5UL]);

    constexpr blaze::StaticVector<double, 4UL> posOffsets = {0.0, -147.0E-3, 0.0, -77.0E-3};
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

    Controller::send_request(pose_in_robot_system);

    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
    RCLCPP_INFO(this->get_logger(), "joints target sent to robot node: R_in:%0.3f [deg]  T_in:%0.3f [mm]  R_mid:%0.3f [deg]  T_mid: %0.3f [mm] | elapsed: %0.3f [ms]",
                msg.position[0UL] * 360 * M_1_PI, msg.position[1UL] * 1e3, msg.position[2UL] * 360 * M_1_PI, msg.position[3UL] * 1e3, elapsed.count() * 1.00E-3);

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    blaze::StaticVector<double, 3UL> pos_error = blaze::subvector<0UL, 3UL>(convertedTarget) - m_tran_base_endEffector;
    RCLCPP_INFO(this->get_logger(), "EMTtip: x: %0.5f   y: %0.5f  z: %0.5f [mm]", m_tran_base_endEffector[0] * 1e3, m_tran_base_endEffector[1] * 1e3, m_tran_base_endEffector[2] * 1e3);
    RCLCPP_INFO(this->get_logger(), "Target: x: %0.5f   y: %0.5f  z: %0.5f [mm]", convertedTarget[0] * 1e3, convertedTarget[1] * 1e3, convertedTarget[2] * 1e3);
    RCLCPP_INFO(this->get_logger(), "TipPos: x: %0.5f   y: %0.5f  z: %0.5f [mm]", m_CTR_robot->getTipPos()[0] * 1e3, m_CTR_robot->getTipPos()[1] * 1e3, m_CTR_robot->getTipPos()[2] * 1e3);
    RCLCPP_INFO(this->get_logger(), "Error:  x: %0.5f   y: %0.5f  z: %0.5f [mm]", pos_error[0] * 1e3, pos_error[1] * 1e3, pos_error[2] * 1e3);
    RCLCPP_INFO(this->get_logger(), "Error: %0.5f [mm]", blaze::norm(pos_error) * 1e3);
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");
    
    // // Wait for action to complete
    // {
    //   std::unique_lock<std::mutex> lock(mutex_);
    //   cv_.wait(lock, [this]()
    //            { return action_done_; });
    // }

    // // Check if the action was successful
    // if (!action_success_)
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Control loop aborted due to action failure.");
    //   return pos_error;
    // }

    return pos_error;
  }

  void send_request(const blaze::StaticVector<double, 4UL> &target)
  {
    while (!m_target_service->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the service to appear...");
    }
    auto request = std::make_shared<interfaces::srv::Jointstarget::Request>();
    for (size_t i = 0; i < target.size(); ++i)
    {
      request->position[i] = target[i];
    }

    // Sending request and non-blocking wait for the future
    auto future_result = m_target_service->async_send_request(request);

    future_result.wait(); // Explicitly wait on the future without spinning

    if (future_result.valid() && future_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto response = future_result.get();
      if (response->success)
      {
        RCLCPP_INFO(this->get_logger(), "Response from server: %s", response->message.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to run robot: %s", response->message.c_str());
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive response from service");
    }
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
  // rclcpp_action::Client<interfaces::action::Jointstarget>::SharedPtr m_action_client;
  rclcpp::Client<interfaces::srv::Jointstarget>::SharedPtr m_target_service;
  rclcpp::Client<interfaces::srv::Transformation>::SharedPtr m_tranformation_client;

  std::ofstream m_output_file;

  std::condition_variable cv_;
  std::mutex mutex_;
  bool action_done_ = false;
  bool action_success_ = false;

  /*
     =============================================================================================
     =========================== Setting the CTR object as data member ===========================
     =============================================================================================
  */
  std::shared_ptr<CTR> m_CTR_robot;
  blaze::StaticVector<double, 5UL> m_initGuess;      // initial guess for the solution of the BVP
  blaze::StaticVector<double, 6UL> m_q;              // joint values of the CTR
  const double m_linearActuatorThickness = 30.00E-3; // thickness of the linear actuator stages --> collision avoidance
  const double m_pos_tol = 1.00E-3;
  // Selected surgical plan
  // blaze::HybridMatrix<double, 4UL, 4UL, blaze::columnMajor> m_H;
  blaze::StaticMatrix<double, 4UL, 4UL> m_H, m_ct_ctr_tranformation;
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

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
#include "interfaces/action/target.hpp"
#include "interfaces/action/jointstarget.hpp"
#include "interfaces/srv/transformation.hpp"
#include "interfaces/srv/jointstarget.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_eigen/tf2_eigen.hpp"

#include <fstream>
#include <filesystem>

#include "CTR.hpp"
#include "Planner.hpp"
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

class PathPlannerNode : public rclcpp::Node
{
public:
  // default class constructor
  PathPlannerNode() : Node("path_planner"), count_(0)
  {
    PathPlannerNode::declare_parameters();
    PathPlannerNode::setup_ros_interfaces();
    PathPlannerNode::init_planner();
  }

  // class destructor
  ~PathPlannerNode()
  {
    m_output_file.close();
  }

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    std::string workspace_directory = ament_index_cpp::get_package_share_directory(m_packageName);
    std::string output_dir = workspace_directory + "/../../../../Output_Files/path_data";
    this->declare_parameter<std::string>("temp_dir", output_dir);
    m_tempDir = this->get_parameter("temp_dir").as_string();
  }

  // Setup ROS interfaces, including publishers, subscribers, and services.
  void setup_ros_interfaces()
  {
    // auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_control_sample_time * 1.00E6));

    m_callback_group_tf2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    /// listener to tf2 transormation messages
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf2_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    m_tf2_timer = create_wall_timer(50ms, std::bind(&PathPlannerNode::tf2_receive_timer_callback, this), m_callback_group_tf2);

    // Subscriber to receive current q
    m_callback_group_sub_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subs_current_q = rclcpp::SubscriptionOptions();
    subs_current_q.callback_group = m_callback_group_sub_1;
    m_subscription_q = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&PathPlannerNode::updateCurrentQ, this, _1), subs_current_q);

    // configing service
    m_manual_target_service = create_service<std_srvs::srv::Trigger>("manual_target", std::bind(&PathPlannerNode::manualTarget_callback, this, _1, _2));

    // publisher to send the path
    m_publisher_path = create_publisher<std_msgs::msg::Float64MultiArray>("task_space/path", 10);

    // // Joint space target Action client
    // m_target_service = create_client<interfaces::srv::Jointstarget>("joint_space/target");

  }

  //
  void init_planner()
  {
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
    constexpr blaze::StaticVector<double, 3UL> ls = {158.00E-3, 77.00E-3, 60.00E-3};

    // --** --Lengths of the tubes' curved sections (meters) -- ** --
    constexpr blaze::StaticVector<double, 3UL> lc = {58.00E-3, 55.00E-3, 0.00};

    // --** --Outer and Inner diameters of the tubes (meters)--** --
    constexpr blaze::StaticVector<double, 3UL> ID = {0.737E-3, 0.965E-3, 1.1448E-3};
    constexpr blaze::StaticVector<double, 3UL> OD = {0.940E-3, 1.372E-3, 2.045E-3};

    // # # # # # ---- Instantiating the three Tube objects ---- # # # # #
    std::shared_ptr<Tube> T1 = std::make_shared<Tube>(OD[0UL], ID[0UL], E1, G1, ls[0UL], lc[0UL], u1); // innermost tube
    std::shared_ptr<Tube> T2 = std::make_shared<Tube>(OD[1UL], ID[1UL], E2, G2, ls[1UL], lc[1UL], u2); // intermediate tube
    std::shared_ptr<Tube> T3 = std::make_shared<Tube>(OD[2UL], ID[2UL], E3, G3, ls[2UL], lc[2UL], u3); // outermost tube

    std::cout << "k1: " << T1->getK(0) << std::endl
              << "k2: " << T2->getK(0) << std::endl;

    // instantiating an array of smart pointers to CTR component tubes
    std::array<std::shared_ptr<Tube>, 3UL> Tb = {T1, T2, T3};

    // initial joint actuation values "home position" - q = [Beta Alpha]
    blaze::StaticVector<double, 3UL> Beta_0 = {-150.00E-3, -78.00E-3, 0.00}; // +50.00E-3
    blaze::StaticVector<double, 3UL> Alpha_0 = {mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00)};

    blaze::StaticVector<double, 6UL> q_0;
    blaze::subvector<0UL, 3UL>(q_0) = Beta_0;
    blaze::subvector<3UL, 3UL>(q_0) = Alpha_0;

    // Determining the accuracy of BVP solutions
    double Tol = 1.00E-6;

    // tolerance for position control
    double pos_tol = 5.00E-4;

    // clearance between linear actuator stages
    double Clr = 30.00E-3;
    
    // Method for solving the BVP Problem
    // 1: Newton-Raphson
    // 2: Levenberg-Marquardt
    // 3: Powell's Dog-Leg
    // 4: Modified Newton-Raphson (globally convergent)
    // 5: Broyden
    // # # # # # ---- Instantiating the CTR object ---- # # # # #
    CTR CTR_robot = CTR(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);

    CTR CTR_StateValidator = CTR(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);
    CTR CTR_MotionValidator = CTR(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);
    CTR CTR_ObjectiveFunction = CTR(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);

    CTR_robot.actuate_CTR(this->m_initGuess, q_0);
    std::cout << "Checkpoint_5" << std::endl;
    // instantiating an Planner object for defining a motion plan to deploy the CTR into the anatomy

    m_motionPlan = std::make_shared<Planner>(CTR_StateValidator, CTR_MotionValidator, CTR_ObjectiveFunction);
    std::cout << "Checkpoint_6" << std::endl;

    // instantiating the CTR object for inverse kinematics
    this->m_CTR_robot_IK = std::make_shared<CTR>(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);
    this->m_CTR_robot_IK->actuate_CTR(this->m_initGuess, q_0);

    // instantiating the CTR object for task trajectory generation
    this->m_CTR_robot_TT = std::make_shared<CTR>(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);
    this->m_CTR_robot_TT->actuate_CTR(this->m_initGuess, q_0);
  }

  // Update current tool (tip) position in the base frame.
  void updateCurrentQ(const interfaces::msg::Jointspace::ConstSharedPtr &msg)
  {
    m_current_q[0UL] = msg->position[1UL];
    m_current_q[1UL] = msg->position[3UL];
    m_current_q[2UL] = 0.0;
    m_current_q[3UL] = msg->position[0UL];
    m_current_q[4UL] = msg->position[2UL];
    m_current_q[5UL] = 0.0;
  }

  /// listen to ROS2 tf2 message
  void tf2_receive_timer_callback()
  {
    geometry_msgs::msg::TransformStamped tf2_tran;
    auto frameNames = m_tf_buffer->getAllFrameNames();

    for (auto sourceFrame : frameNames)

      if (sourceFrame == "probe")
      {
        std::string targetFrame = "robot_base";
        try
        {
          tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
          // std::cout << "X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              targetFrame.c_str(), sourceFrame.c_str(), ex.what());
          return;
        }
        Eigen::Matrix4d eigen_trans = tf2::transformToEigen(tf2_tran).matrix();
        m_manual_target[0] = eigen_trans(0, 3);
        m_manual_target[1] = eigen_trans(1, 3);
        m_manual_target[2] = eigen_trans(2, 3);

        // RCLCPP_INFO(this->get_logger(), "IGTL EM tf2 sent");
      }
  }

  // Service callback to triget tasks, enable, and control mode section
  void manualTarget_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // ToDo: add a timeout

    // initial state: initial robot configuration prior to the deployment ==> CTR steered to renal calyx
    blaze::StaticVector<double, 6UL> q_initial, q_final;

    // initial joint actuation values "home position" - q = [Beta Alpha]
    blaze::StaticVector<double, 3UL> Beta_0 = {-150.00E-3, -73.00E-3, 0.00};
    blaze::StaticVector<double, 3UL> Alpha_0 = {mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00)};

    blaze::StaticVector<double, 6UL> q_0;
    blaze::subvector<0UL, 3UL>(q_0) = Beta_0;
    blaze::subvector<3UL, 3UL>(q_0) = Alpha_0;

    // initial configuration
    q_initial = q_0;

    std::cout << "q_initial: " << blaze::trans(q_initial) << std::endl;
    // run IK to compute q_final
    std::cout << "manual_target: x: " << m_manual_target[0] * 1e3 << " |  " << "y: " << m_manual_target[1] * 1e3 << " |  " << "z: " << m_manual_target[2] * 1e3 << std::endl;

    std::cout << "Running IK" << std::endl;
    constexpr double pos_tol = 1.00E-3;
    m_CTR_robot_IK->posCTRL(m_initGuess_IK, m_manual_target, pos_tol);
    q_final = m_CTR_robot_IK->getConfiguration();
    // std::cout << "Running IK" << std::endl;

    // // initial joint actuation values "home position" - q = [Beta Alpha]
    // blaze::StaticVector<double, 3UL> Beta_0 = {-150.00E-3, -73.00E-3, 0.00}; // +50.00E-3
    // blaze::StaticVector<double, 3UL> Alpha_0 = {mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00)};

    // blaze::StaticVector<double, 6UL> q_0;
    // blaze::subvector<0UL, 3UL>(q_0) = Beta_0;
    // blaze::subvector<3UL, 3UL>(q_0) = Alpha_0;
    // q_initial = q_0;

    // final configuration
    // q_final = {-0.0949115, -0.0497543, 0.00, 0.227169, -0.318651, 0.00}; // Lower-Pole Access
    // q_final = {-0.083473, -0.051600, 0.00, -0.647814, -1.493957, 0.00}; // Mid-Pole Access
    // q_final = {-0.0949115, -0.0497543, 0.00, 0.227169, -0.318651, 0.00}; // Upper-Pole Access
    q_final = {-0.0629115, -0.0317543, 0.00, 0.227169, -1.6851, 0.00}; // Upper-Pole Access

    // setting the initial state: initial configuration of the robot
    m_motionPlan->setStartState(q_initial);
    // setting the goal state: distal end steered to renal calyx
    m_motionPlan->setGoalState(q_final);

    std::cout << "Start state: " << blaze::trans(q_initial)
              << "Goal state: " << blaze::trans(q_final) << std::endl;

    // setting up the planning problem and its definitions
    constexpr double runTime = 20.00; // Planning time in seconds (2 min)

    const std::string plannedPathFile("plannedPath.csv");

    auto start = std::chrono::high_resolution_clock::now();
    m_motionPlan->plan(runTime, Planner::optimalPlanner::PLANNER_RRT, Planner::planningObjective::OBJECTIVE_BACKBONE_LENGTH, m_tempDir, plannedPathFile);
    // motionPlan.plan(runTime, Planner::optimalPlanner::PLANNER_PRM, Planner::planningObjective::OBJECTIVE_BACKBONE_LENGTH, plannedPathFile);
    auto end = std::chrono::high_resolution_clock::now();
	  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "Finished planning!! - Saving plan in: " << plannedPathFile << std::endl;
    std::cout << "Planning time: " << elapsed * 1.00E-3 << " seconds" << std::endl;
    // std::cout << "Finished planning!!" << std::endl;

    genTaskTraj();

    response->success = true;
  }

  void genTaskTraj()
  {
    // initial guess for the BVP
    blaze::StaticVector<double, 5UL> initGuess;
    blaze::HybridMatrix<double, 15000UL, 6UL, blaze::columnMajor> JointValues; // Sequence of actuation values

    // reading the planned path from the CSV file
    readFromCSV(JointValues, m_tempDir, "plannedPath");

    // total number of joint values in the motion plan
    const size_t totalRows = JointValues.rows();
    // stores the current joint values to be actuated
    blaze::StaticVector<double, 6UL> q;
    // amount of time in milliseconds elapsed between consecutive CTR configurations
    constexpr size_t milliseconds = 50UL;

    bool convergence = false;

    
    std_msgs::msg::Float64MultiArray msg;
    // 3 rows × n cols → flattened into row-major
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = totalRows;
    msg.layout.dim[0].stride = totalRows * 3;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = 3;
    msg.layout.dim[1].stride = 3;

    msg.data.resize(totalRows * 3);

    for (size_t row = 0UL; row < totalRows; row += 1)
    {
      std::cout << "iteration " << row + 1 << " of " << totalRows << "\r";
      // clear the output buffer
      std::cout.flush();

      q[0UL] = JointValues(row, 0UL);
      q[1UL] = JointValues(row, 1UL);
      q[2UL] = JointValues(row, 2UL);
      q[3UL] = JointValues(row, 3UL);
      q[4UL] = JointValues(row, 4UL);
      q[5UL] = JointValues(row, 5UL);

      // initGuess = 0.00;
      convergence = m_CTR_robot_TT->actuate_CTR(initGuess, q);

      if (!convergence)
        initGuess = 0.00;

      blaze::StaticVector<double, 3> tipPos = m_CTR_robot_TT->getTipPos();

      for (size_t j = 0; j < 3; ++j)
      {
        msg.data[row * 3 + j] = tipPos[j]; // row-major order
      }

      // Introduce a delay of 10 milliseconds (10,000 microseconds) using usleep
      usleep(5000);

      // m_CTR_robot_TT->broadcastShapeToSlicer();
    }

    m_publisher_path->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published path matrix");

    std::cout << "Task Trajectory Generation Finish!!" << std::endl;
  }

  // function that reads relevant clinical data from CSV files for each case
  template <typename MatrixType>
  MatrixType readFromCSV(MatrixType &Mat, const std::string &dir, const std::string &fileName)
  {
    // Construct file path and name
    const std::filesystem::path filePath = dir; //("../../Output_Files/");
    const std::filesystem::path file = filePath / (fileName + ".csv");

    // Ensure the directory exists
    std::filesystem::create_directories(filePath);

    // Open the CSV file
    std::ifstream CSV_file(file, std::ifstream::in);
    if (!CSV_file.is_open())
    {
      throw std::runtime_error("Error opening the CSV file: " + file.string());
    }

    typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

    std::vector<std::string> vec;
    std::string line;

    size_t row = 0UL, col = 0UL;
    double value;

    while (std::getline(CSV_file, line))
    {
      Tokenizer tokenizer(line);
      col = 0UL;

      for (Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
      {
        value = std::stod(*it);
        Mat(row, col) = value;
        ++col;
      }
      ++row;
    }

    CSV_file.close();
    Mat.resize(row, col, true);

    return Mat;
  }

private:
  // Member variables
  const std::string m_packageName = "planner";

  size_t count_;
  std::string m_tempDir;
  double m_t_init = 0.00;
  double m_sample_time;
  bool m_flag_new_feedback = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  blaze::StaticVector<double, 3UL> m_target_position, m_calyxPosition;
  blaze::StaticVector<double, 6UL> m_current_q;
  blaze::StaticVector<double, 4UL> m_rot_phantom_base;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub_1, m_callback_group_sub_2, m_callback_group_sub_3; // Callback group for running subscriber callback function on separate thread
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_q;                           // Subscriber object
  rclcpp::Client<interfaces::srv::Jointstarget>::SharedPtr m_target_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_manual_target_service;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_tf2;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_path;
  rclcpp::TimerBase::SharedPtr m_tf2_timer;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf2_listener;

  std::ofstream m_output_file;

  std::condition_variable cv_;
  std::mutex mutex_;

  blaze::StaticVector<double, 3> m_manual_target;

  std::shared_ptr<CTR> m_CTR_robot_IK;
  std::shared_ptr<CTR> m_CTR_robot_TT;
  std::shared_ptr<Planner> m_motionPlan;
  // Planner m_motionPlan;
  std::shared_ptr<CTR> m_CTR_StateValidator;
  std::shared_ptr<CTR> m_CTR_MotionValidator;
  std::shared_ptr<CTR> m_CTR_ObjectiveFunction;

  blaze::StaticVector<double, 5UL> m_initGuess;      // initial guess for the solution of the BVP
  blaze::StaticVector<double, 5UL> m_initGuess_IK;   // initial guess for the solution of the BVP
  blaze::StaticVector<double, 6UL> m_q;              // joint values of the CTR
  const double m_linearActuatorThickness = 30.00E-3; // thickness of the linear actuator stages --> collision avoidance
  const double m_pos_tol = 1.00E-3;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

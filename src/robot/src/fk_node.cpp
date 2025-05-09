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
#include "std_msgs/msg/float64_multi_array.hpp"

#include <fstream>
#include <filesystem>

#include "CTR.hpp"
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

class ForwardKinNode : public rclcpp::Node
{
public:
  // default class constructor
  ForwardKinNode() : Node("forward_kinematics_node"), count_(0)
  {
    ForwardKinNode::declare_parameters();
    ForwardKinNode::setup_ros_interfaces();
    ForwardKinNode::init_robot();
  }

  // class destructor
  ~ForwardKinNode()
  {
  }

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    declare_parameter<double>("sample_time", 100E-3);
    m_sample_time = get_parameter("sample_time").as_double();
  }

  // Setup ROS interfaces, including publishers, subscribers, and services.
  void setup_ros_interfaces()
  {
    // timer to run the forward kin and publish shape
    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_callback_group_0 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_fk_timer = create_wall_timer(sample_time, std::bind(&ForwardKinNode::actuate_timer_callback, this), m_callback_group_0);

    // Subscriber to receive current q
    m_callback_group_sub_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subs_current_q = rclcpp::SubscriptionOptions();
    subs_current_q.callback_group = m_callback_group_sub_1;
    m_subscription_q = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&ForwardKinNode::updateCurrentQ, this, _1), subs_current_q);

    // publisher to send the path
    m_publisher_tube_1 = create_publisher<std_msgs::msg::Float64MultiArray>("shape/tube_1", 10);
    m_publisher_tube_2 = create_publisher<std_msgs::msg::Float64MultiArray>("shape/tube_2", 10);
    m_publisher_tube_3 = create_publisher<std_msgs::msg::Float64MultiArray>("shape/tube_3", 10);
  }

  /// initialize robot
  void init_robot()
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
    blaze::StaticVector<double, 3UL> Beta_0 = {-150.00E-3, -73.00E-3, 0.00}; // +50.00E-3
    blaze::StaticVector<double, 3UL> Alpha_0 = {mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00), mathOp::deg2Rad(0.00)};

    // blaze::StaticVector<double, 6UL> q_0;
    blaze::subvector<0UL, 3UL>(m_current_q) = Beta_0;
    blaze::subvector<3UL, 3UL>(m_current_q) = Alpha_0;

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
    this->m_CTR_robot = std::make_shared<CTR>(Tb, m_current_q, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, Clr);
    this->m_CTR_robot->actuate_CTR(this->m_initGuess, m_current_q);
  }

  /// Update current joints configuration
  void updateCurrentQ(const interfaces::msg::Jointspace::ConstSharedPtr &msg)
  {
    m_current_q[0UL] = msg->position[1UL];
    m_current_q[1UL] = msg->position[3UL];
    m_current_q[2UL] = 0.0;
    m_current_q[3UL] = msg->position[0UL];
    m_current_q[4UL] = msg->position[2UL];
    m_current_q[5UL] = 0.0;
    // std::cout << "current q:" << blaze::trans(m_current_q) << std::endl;
  }

  /// actuate the model to the current joints config and get the shape
  void actuate_timer_callback()
  {
    bool convergence = false;

    convergence = m_CTR_robot->actuate_CTR(m_initGuess, m_current_q);
    if (!convergence)
      m_initGuess = 0.00;

    // blaze::StaticVector<double, 3> temp = m_CTR_robot->getTipPos();
    // std::cout << "current tip:" << blaze::trans(temp) << std::endl;
    auto [Tb1, Tb2, Tb3] = m_CTR_robot->getTubeShapes();   // units are in [mm]

    // std::cout << "Tb1: \n" << blaze::trans(Tb1) << std::endl;

    std_msgs::msg::Float64MultiArray msg_1 = blazeToMultiArrayMsg(Tb1 * 1e-3);
    std_msgs::msg::Float64MultiArray msg_2 = blazeToMultiArrayMsg(Tb2 * 1e-3);
    std_msgs::msg::Float64MultiArray msg_3 = blazeToMultiArrayMsg(Tb3 * 1e-3);
    m_publisher_tube_1->publish(msg_1);
    m_publisher_tube_2->publish(msg_2);
    m_publisher_tube_3->publish(msg_3);

    // RCLCPP_INFO(this->get_logger(), "Published shape");
  }

  ///
  std_msgs::msg::Float64MultiArray blazeToMultiArrayMsg(const blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> &mat)
  {
    std_msgs::msg::Float64MultiArray msg;

    auto mat_row_major = blaze::trans(mat);
    size_t rows = mat_row_major.rows();
    size_t cols = mat_row_major.columns();

    // 3 rows × n cols → flattened into row-major
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = rows;
    msg.layout.dim[0].stride = rows * cols;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = cols;
    msg.layout.dim[1].stride = cols;

    msg.data.resize(rows * cols);

    for (size_t i = 0; i < rows; ++i)
    {
      for (size_t j = 0; j < cols; ++j)
      {
        msg.data[i * cols + j] = mat_row_major(i, j); // row-major order
      }
    }

    return msg;
  }

private:
  // Member variables
  const std::string m_packageName = "robot";

  size_t count_;
  double m_sample_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  blaze::StaticVector<double, 6UL> m_current_q;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub_1;                       // Callback groups
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_q; // Subscriber object

  rclcpp::CallbackGroup::SharedPtr m_callback_group_0;
  rclcpp::TimerBase::SharedPtr m_fk_timer;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_tube_1, m_publisher_tube_2, m_publisher_tube_3;

  std::shared_ptr<CTR> m_CTR_robot;
  blaze::StaticVector<double, 5UL> m_initGuess; // initial guess for the solution of the BVP
};

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

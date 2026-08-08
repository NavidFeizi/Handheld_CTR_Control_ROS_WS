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
#include "interfaces/srv/config.hpp"
#include "interfaces/srv/planner.hpp"
#include "interfaces/msg/force.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_eigen/tf2_eigen.hpp"

#include <fstream>
#include <filesystem>

#include "PINNs.hpp"
#include "Planner.hpp"
#include <limits>

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

std::string package_name = "planner";
std::string PACKAGE_SHARE_DIR = ament_index_cpp::get_package_share_directory(package_name);

class PathPlannerNode : public rclcpp::Node
{
  // Compile-time CTR sizing — declared first so they are visible in the
  // member-function parameter types further down (parameter lists are not part
  // of the complete-class context).
  const std::string kModelName = "ctr_8x91_0.18_tanh_9K_9K_50K_FP64";
  const static size_t kBackbonePoints = 150UL; // number of discretized backbone points for the planning CTR
  const static size_t kControlInputs = 4UL;    // number of control inputs (actuated tubes)
  const static size_t kBatch = 1UL;

public:
  // default class constructor
  PathPlannerNode() : Node("path_planner"), m_ctr_pinn(kModelName, kBatch, kBackbonePoints), m_motionPlan(m_ctr_pinn)
  {
    PathPlannerNode::declare_parameters();
    PathPlannerNode::setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "Path Planner Node has been initialized.");
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
    std::string output_dir = workspace_directory + "/../../../../Shared_Files";
    this->declare_parameter<std::string>("temp_dir", output_dir);
    m_tempDir = this->get_parameter("temp_dir").as_string();
    if (!std::filesystem::exists(m_tempDir))
    {
      std::filesystem::create_directories(m_tempDir);
    }
  }

  // Setup ROS interfaces, including publishers, subscribers, and services.
  void setup_ros_interfaces()
  {
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

    // Subscriber to receive current q
    m_callback_group_sub_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subs_current_tip = rclcpp::SubscriptionOptions();
    subs_current_tip.callback_group = m_callback_group_sub_2;
    m_subscription_tip = create_subscription<interfaces::msg::Taskspace>("task_space/feedback/base_tool", 10, std::bind(&PathPlannerNode::updateCurrentX, this, _1), subs_current_q);

    // Subscriber to receive the EKF external tip-force estimate
    m_callback_group_sub_3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subs_force = rclcpp::SubscriptionOptions();
    subs_force.callback_group = m_callback_group_sub_3;
    m_subscription_force = create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&PathPlannerNode::updateForceEstimate, this, _1), subs_force);

    // // path planning service
    // m_manual_target_service = create_service<interfaces::srv::Config>("manual_target", std::bind(&PathPlannerNode::planner_callback, this, _1, _2));

    // path planning service
    m_command_service = create_service<interfaces::srv::Planner>("planner/command", std::bind(&PathPlannerNode::plannerService_callback, this, _1, _2));

    // publisher to send the path
    m_publisher_path = create_publisher<std_msgs::msg::Float64MultiArray>("task_space/path", 10);

    // publisher to actuate the robot
    m_publisher_actuate = create_publisher<interfaces::msg::Jointspace>("joint_space/target", 10);
  }

  // Update current joints position.
  void updateCurrentQ(const interfaces::msg::Jointspace::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(m_feedback_mutex);
    m_current_q[0UL] = msg->position[1UL];
    m_current_q[1UL] = msg->position[3UL];
    m_current_q[2UL] = 0.00;
    m_current_q[3UL] = msg->position[0UL];
    m_current_q[4UL] = msg->position[2UL];
    m_current_q[5UL] = 0.00;
    // std::cout << "m_current_q: " << blaze::trans(m_current_q) << std::endl;
  }

  // Update current tool (tip) position in the base frame.
  void updateCurrentX(const interfaces::msg::Taskspace::ConstSharedPtr &msg)
  {
    m_x[0UL] = msg->p[0UL];
    m_x[1UL] = msg->p[1UL];
    m_x[2UL] = msg->p[2UL];
    // std::cout << "m_current_x: " << blaze::trans(m_current_q) << std::endl;
  }

  // Update the EKF external tip-force estimate.
  void updateForceEstimate(const interfaces::msg::Force::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(m_feedback_mutex);
    m_force_est[0UL] = msg->x;
    m_force_est[1UL] = msg->y;
    m_force_est[2UL] = msg->z;
  }

  /// listen to ROS2 tf2 message
  void tf2_receive_timer_callback()
  {
    geometry_msgs::msg::TransformStamped tf2_tran;
    auto frameNames = m_tf_buffer->getAllFrameNames();

    for (auto sourceFrame : frameNames)

      if (sourceFrame == "probe") // "probe", "path_target"
      {
        // RCLCPP_INFO(this->get_logger(), "Target received");
        std::string targetFrame = "robot_base";
        try
        {
          tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
          // RCLCPP_INFO(this->get_logger(), "Target transform updated");
          // std::cout << "Target in Robot -> X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
          // RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
          return;
        }
        Eigen::Matrix4d eigen_trans = tf2::transformToEigen(tf2_tran).matrix();
        m_manual_target[0UL] = eigen_trans(0, 3);
        m_manual_target[1UL] = eigen_trans(1, 3);
        m_manual_target[2UL] = eigen_trans(2, 3);

        // RCLCPP_INFO(this->get_logger(), "IGTL EM tf2 sent");

        try
        {
          tf2_tran = m_tf_buffer->lookupTransform("em_tracker", sourceFrame, tf2::TimePointZero);
          // std::cout << "Target in EM -> X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
          // RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
          return;
        }
      }

      else if (sourceFrame == "ctr_tip") // "probe", "path_target"
      {
        // RCLCPP_INFO(this->get_logger(), "Target received");
        std::string targetFrame = "robot_base";
        try
        {
          tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
          // std::cout << "Target in Robot -> X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
          // RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
          return;
        }
        Eigen::Matrix4d eigen_trans = tf2::transformToEigen(tf2_tran).matrix();
        // m_x[0UL] = eigen_trans(0, 3);
        // m_x[1UL] = eigen_trans(1, 3);
        // m_x[2UL] = eigen_trans(2, 3);

        // RCLCPP_INFO(this->get_logger(), "IGTL EM tf2 sent");

        try
        {
          tf2_tran = m_tf_buffer->lookupTransform("em_tracker", sourceFrame, tf2::TimePointZero);
          // std::cout << "Target in EM -> X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
          // RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", targetFrame.c_str(), sourceFrame.c_str(), ex.what());
          return;
        }
      }
  }

  // Service callback to trigger commanding the robot
  void plannerService_callback(const std::shared_ptr<interfaces::srv::Planner::Request> request, std::shared_ptr<interfaces::srv::Planner::Response> response)
  {
    if (request->command == "generateTrajectory")
    {
      blaze::StaticVector<double, 3UL> target = request->value;
      // std::this_thread::sleep_for(std::chrono::milliseconds(50));
      double error = 0.0;

      // target = {-0.00127, 0.00398, 0.13724};
      // target = {-0.00056, 0.02588, 0.11926};
      // target = {0.00801581, 0.0327939, 0.130476};
      // target = {0.00395, 0.02749, 0.12326};

      try
      {
        // One consistent snapshot of joint feedback and EKF force for the whole request.
        blaze::StaticVector<double, 6UL> q_snapshot;
        blaze::StaticVector<double, 3UL> force;
        {
          std::lock_guard<std::mutex> lock(m_feedback_mutex);
          q_snapshot = m_current_q;
          force = m_force_est;
        }

        // Map the 6-element current configuration [β₁, β₂, β₃, α₁, α₂, α₃] (tube-3
        // entries are static/zero) down to the 4 actuated inputs [β₁, β₂, α₁, α₂].
        blaze::StaticVector<double, 4UL> q_initial = {q_snapshot[0UL], q_snapshot[1UL], q_snapshot[3UL], q_snapshot[4UL]};
        blaze::StaticVector<double, 4UL> q_final = q_initial;

        // Safe here: no solve is in flight (mutually exclusive service group).
        // Propagates to the FTL objective (cache cleared) and informed samplers.
        m_motionPlan.setCTR_externalForce(force);
        RCLCPP_INFO(this->get_logger(), "Planning with force estimate: f = [%.4f, %.4f, %.4f] N", force[0UL], force[1UL], force[2UL]);

        error = inverseKin(target, q_final, force);

        RCLCPP_INFO(this->get_logger(), "Initial config: q = %.4f, %.4f, %.4f, %.4f", q_initial[0UL], q_initial[1UL], q_initial[2UL], q_initial[3UL]);
        RCLCPP_INFO(this->get_logger(), "Final config: q = %.4f, %.4f, %.4f, %.4f", q_final[0UL], q_final[1UL], q_final[2UL], q_final[3UL]);

        bool planning_status = plan(q_initial, q_final, force);
        if (planning_status)
        {
          m_target_last = target;
          response->success = true;
          response->value = error;
          response->message = "Path generated successfully.";
        }
        else
        {
          response->success = false;
          response->value = error;
          response->message = "Planning failed to find a solution.";
          return;
        }
      }
      catch (const std::exception &e)
      {
        std::cout << "Planning error: " << e.what() << '\n';
        response->success = false;
        response->value = error;
        response->message = e.what();
      }
    }
    else if (request->command == "replanDeployment")
    {
      // Response contract for this command:
      //   success = new schedule exported to plannedPath.csv
      //   value   = FTL swept cost of the chosen schedule (NOT an IK error)
      //   message = schedule name or failure reason
      blaze::StaticVector<double, 6UL> q_snapshot;
      blaze::StaticVector<double, 3UL> force;
      {
        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        q_snapshot = m_current_q;
        force = m_force_est;
      }

      if (!m_has_active_plan)
      {
        response->success = false;
        response->value = 0.0;
        response->message = "replanDeployment: no active plan/goal - call generateTrajectory first.";
        return;
      }

      try
      {
        blaze::StaticVector<double, 4UL> q_now = {q_snapshot[0UL], q_snapshot[1UL], q_snapshot[3UL], q_snapshot[4UL]};

        RCLCPP_INFO(this->get_logger(), "Replan request: f = [%.4f, %.4f, %.4f] N (active plan used f = [%.4f, %.4f, %.4f] N), q_now = [%.4f, %.4f, %.4f, %.4f]",
                    force[0UL], force[1UL], force[2UL],
                    m_force_at_plan[0UL], m_force_at_plan[1UL], m_force_at_plan[2UL],
                    q_now[0UL], q_now[1UL], q_now[2UL], q_now[3UL]);

        // Safe here: no solve is in flight (mutually exclusive service group).
        m_motionPlan.setCTR_externalForce(force);

        auto start = std::chrono::high_resolution_clock::now();
        const bool ok = m_motionPlan.planDeployment(q_now, m_q_goal_last);
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (ok)
        {
          m_force_at_plan = force;
          m_motionPlan.writeSolutionToFile(m_tempDir + "/plannedPath.csv");
          publishTaskSpacePath();

          response->success = true;
          response->value = m_motionPlan.lastDeploymentCost();
          response->message = "Deployment replanned: " + m_motionPlan.lastDeploymentScheduleName();
          RCLCPP_INFO(this->get_logger(), "Replanned deployment in %ld ms: %s (FTL cost %.6f)",
                      static_cast<long>(elapsed), m_motionPlan.lastDeploymentScheduleName().c_str(), m_motionPlan.lastDeploymentCost());
        }
        else
        {
          // planDeployment() cleared the solution paths, so nothing stale can be
          // exported; the previous CSV stays on disk and the master keeps
          // executing the old plan.
          response->success = false;
          response->value = 0.0;
          response->message = "replanDeployment failed (alpha mismatch, invalid start, or no valid schedule).";
          RCLCPP_WARN(this->get_logger(), "Deployment replanning failed after %ld ms.", static_cast<long>(elapsed));
        }
      }
      catch (const std::exception &e)
      {
        std::cout << "Replanning error: " << e.what() << '\n';
        response->success = false;
        response->value = 0.0;
        response->message = e.what();
      }
    }
    else
    {
      response->success = false;
      response->value = 0.0;
      response->message = "Invalid planner command";
    }
  }

  // Service callback to triget tasks, enable, and control mode section
  double inverseKin(const blaze::StaticVector<double, 3UL> &target, blaze::StaticVector<double, 4UL> &q, const blaze::StaticVector<double, 3UL> &force)
  {
    // run IK to compute q_final
    constexpr double posTolerance = 5.00E-4;
    blaze::StaticVector<double, 3UL> tipPosition;

    std::cout << "\nRunning IK..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    // force-aware IK: uses the force registered via setCTR_externalForce()
    m_motionPlan.solveInverseKinematics(q, target, posTolerance);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "IK time: " << elapsed * 1.00E-3 << " seconds" << std::endl;

    m_ctr_pinn.getPosDistal(q, force, tipPosition);
    std::cout << "CTR target joints are: q = " << blaze::trans(q)
              << "target: " << blaze::trans(target)
              << "tip position (after IK): " << blaze::trans(tipPosition)
              << "error: " << blaze::norm(target - tipPosition) * 1.00E3 << " mm\n"
              << std::endl;

    return blaze::norm(target - tipPosition);
  }

  // Service callback to triget tasks, enable, and control mode section
  bool plan(const blaze::StaticVector<double, 4UL> &q_initial, const blaze::StaticVector<double, 4UL> &q_final, const blaze::StaticVector<double, 3UL> &force)
  {
    bool planning_status = false;
    // std::cout << "Target: x: " << m_manual_target[0] * 1.00E3 << " |  " << "y: " << m_manual_target[1] * 1.00E3 << " |  " << "z: " << m_manual_target[2] * 1.00E3 << std::endl;

    // setting the initial state: initial configuration of the robot
    m_motionPlan.setStartState(q_initial);
    m_motionPlan.setGoalState(q_final);

    blaze::StaticVector<double, 4UL> scale = {1.00, 1.00, 20.0, 20.0};
    double norm_diff = blaze::norm((m_q_initial_prev - q_initial) / scale);

    std::cout << "\nStart state: " << blaze::trans(q_initial) << "Goal state: " << blaze::trans(q_final) << std::endl;

    // setting up the planning problem and its definitions
    constexpr double runTime = 3.00; // Planning time in seconds (2 min)

    // completely silence OMPL's own logging:
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    const std::string plannedPathFile = m_tempDir + "/plannedPath.csv";
    auto start = std::chrono::high_resolution_clock::now();
    // motionPlan.plan(runTime, Planner::optimalPlanner::PLANNER_RRT, Planner::planningObjective::OBJECTIVE_BACKBONE_LENGTH, plannedPathFile);
    if (true)//(norm_diff > 1.00E-3)
    {
      // m_motionPlan->plan(runTime, Planner::optimalPlanner::PLANNER_RRT, Planner::planningObjective::OBJECTIVE_BACKBONE_LENGTH);
      // m_motionPlan.plan(runTime, Planner<kBackbonePoints, kControlInputs>::optimalPlanner::PLANNER_RRT_CONNECT, Planner<kBackbonePoints, kControlInputs>::planningObjective::OBJECTIVE_REVJOINTSANDPATHLENGTH);
      // planning_status = m_motionPlan.plan(runTime, Planner<kBackbonePoints, kControlInputs>::optimalPlanner::PLANNER_RRT_CONNECT, Planner<kBackbonePoints, kControlInputs>::planningObjective::OBJECTIVE_REVJOINTS_AND_BACKBONE);
      // m_ctr_pinn
      planning_status = m_motionPlan.planTwoPhase(runTime, Planner<kControlInputs>::optimalPlanner::PLANNER_RRT_CONNECT);

      // std::cout << "The first plan" << std::endl;
      m_first_plan = false;
    }
    else
    {
      m_motionPlan.replan(runTime);
      std::cout << "Not the first plan (replan)" << std::endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Planning time: " << elapsed * 1.00E-3 << " seconds" << std::endl;

    m_q_initial_prev = q_initial;

    if (planning_status)
    {
      // Export and publish only real solutions. On failure the previous CSV stays
      // on disk, so the master keeps executing the old plan (fallback contract).
      m_motionPlan.writeSolutionToFile(plannedPathFile);
      std::cout << "Finished planning!! - Saved plan in: " << plannedPathFile << std::endl;

      m_has_active_plan = true;
      m_q_goal_last = q_final;
      m_force_at_plan = force;
      publishTaskSpacePath(); // FK below uses m_force_at_plan — keep after the update
    }

    return planning_status;
  }

  /// Load generated path in joint space, run it through FK to generate task-space path, and publish.
  void publishTaskSpacePath()
  {
    const double step_size = 2e-3;
    std::vector<blaze::StaticVector<double, kControlInputs>> m_q_list;
    blaze::StaticVector<double, 3UL> tipPosition;

    // RCLCPP_INFO(get_logger(), "Checkpoint_0");
    read_path_from_csv(m_q_list, "plannedPath.csv");

    if (m_q_list.size() == 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read planned path or empty CSV");
      std::lock_guard<std::mutex> lock(m_feedback_mutex);
      m_q = {m_current_q[0UL], m_current_q[1UL], m_current_q[3UL], m_current_q[4UL]};
    }
  
    auto m_q_list_adjusted = adjustConfigurationListStepSize(m_q_list, step_size);
    // RCLCPP_INFO(get_logger(), "Checkpoint_1");
    // prepare message format
    size_t pathSize = m_q_list_adjusted.size();
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = pathSize;
    msg.layout.dim[0].stride = pathSize * 3;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = 3;
    msg.layout.dim[1].stride = 3;
    msg.data.resize(pathSize * 3);
    // RCLCPP_INFO(get_logger(), "Checkpoint_2");
    for (size_t i = 0; i < m_q_list_adjusted.size(); ++i)
    {
      // predicted tip under the same force the active plan was computed with
      m_ctr_pinn.getPosDistal(m_q_list_adjusted[i], m_force_at_plan, tipPosition);

      const size_t idx = i * 3UL;
      msg.data[idx] = tipPosition[0UL];
      msg.data[idx + 1] = tipPosition[1UL];
      msg.data[idx + 2] = tipPosition[2UL];
    }
    // RCLCPP_INFO(get_logger(), "Checkpoint_3");

    m_publisher_path->publish(msg);
    RCLCPP_INFO(get_logger(), "Published task-space path with %zu points.", pathSize);

    // readFromCSV(m_JointValues, m_tempDir, "plannedPath");
    // // Read joint space path from CSV
    // if (m_JointValues.rows() == 0)
    // {
    //   RCLCPP_ERROR(get_logger(), "Failed to read planned path or empty CSV");
    //   m_q = m_current_q;
    // }
    // else
    // {
    //   m_q = blaze::trans(blaze::row(m_JointValues, m_JointValues.rows() - 1UL));
    // }

    // // Actuate CTR to current config
    // bool convergence = m_ctr_pinn.actuate_CTR(m_initGuess, m_current_q);
    // if (!convergence)
    // {
    //   RCLCPP_ERROR(get_logger(), "FK failed for initial joint state");
    //   return;
    // }
    // // Get backbone shape
    // const size_t current_number_of_bb_points = m_ctr_pinn.getNumberOfkBackbonePoints() + 1;
    // std::cout << "current_number_of_bb_points: " << current_number_of_bb_points << std::endl;

    // // Actuate CTR to compute FK
    // convergence = m_ctr_pinn.actuate_CTR(m_initGuess, m_q);
    // if (!convergence)
    // {
    //   RCLCPP_ERROR(get_logger(), "FK failed for final joint state");
    //   return;
    // }

    // // Get backbone shape
    // const auto [x, y, z] = m_ctr_pinn.getShape();
    // std::cout << "final_config_number_of_bb_points: " << x.size() << std::endl;

    // std::cout << "Replaned trajectory -- CTR tip position: " << blaze::trans(m_ctr_pinn.getTipPos()) << std::endl;

    // size_t pathSize = x.size() - current_number_of_bb_points;

    // std_msgs::msg::Float64MultiArray msg;
    // msg.layout.dim.resize(2);
    // msg.layout.dim[0].label = "rows";
    // msg.layout.dim[0].size = pathSize;
    // msg.layout.dim[0].stride = pathSize * 3;
    // msg.layout.dim[1].label = "cols";
    // msg.layout.dim[1].size = 3;
    // msg.layout.dim[1].stride = 3;
    // msg.data.resize(pathSize * 3);

    // // Populate message
    // for (size_t i = 0; i < pathSize; ++i)
    // {
    //   const size_t idx = i * 3UL;
    //   msg.data[idx] = x[i + current_number_of_bb_points];
    //   msg.data[idx + 1] = y[i + current_number_of_bb_points];
    //   msg.data[idx + 2] = z[i + current_number_of_bb_points];
    // }

    // m_publisher_path->publish(msg);
  }

  void read_path_from_csv(std::vector<blaze::StaticVector<double, kControlInputs>>& init_q_list,  const std::string& fileName)
  {
      std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
      ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
      std::filesystem::path file_path = ws_dir / "Shared_Files" / fileName;

      std::ifstream file;
      file.open(file_path, std::ifstream::in);
      if (!file.is_open())
      {
          RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
          return;
      }

      init_q_list.clear();
      std::string line;

      // writeSolutionToFile() emits no header; every line is a control-space state.
      // Read file line by line
      while (std::getline(file, line))
      {
          std::istringstream ss(line);
          std::string value;
          std::vector<double> row;

          while (std::getline(ss, value, ','))
          {
              try
              {
                  row.push_back(std::stod(value));
              }
              catch (const std::exception& e)
              {
                  RCLCPP_WARN(get_logger(), "Failed to parse value: %s", value.c_str());
              }
          }

          if (row.size() == kControlInputs)
          {
              blaze::StaticVector<double, kControlInputs> q_point = {row[0], row[1], row[2], row[3]};
              init_q_list.push_back(q_point);
          }
          else
          {
              RCLCPP_WARN(get_logger(), "Line does not contain exactly %zu values: %s", kControlInputs, line.c_str());
          }
      }

      file.close();
      RCLCPP_INFO(get_logger(), "Loaded %zu path points from CSV file.", init_q_list.size());
  }

  std::vector<blaze::StaticVector<double, kControlInputs>> adjustConfigurationListStepSize(const std::vector<blaze::StaticVector<double, kControlInputs>>& q_list_in, double step_size)
  {
      std::vector<blaze::StaticVector<double, kControlInputs>> q_list_out;

      if (q_list_in.empty())
      {
          RCLCPP_WARN(this->get_logger(), "Empty list");
          return q_list_out;
      }

      size_t prev_idx = 0;
      q_list_out.push_back(q_list_in[0]);

      for (size_t i = 1; i < q_list_in.size(); ++i)
      {
          if (std::abs(q_list_in[i][0] - q_list_in[prev_idx][0]) >= step_size)
          {
              prev_idx = i;
              q_list_out.push_back(q_list_in[i]);
          }
      }
      q_list_out.push_back(q_list_in.back());

      return q_list_out;
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

  bool m_first_plan = true;

  size_t count_;
  int m_traj_counter = 0;
  std::string m_tempDir;
  double m_t_init = 0.00;
  double m_sample_time;
  bool m_flag_new_feedback = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  blaze::StaticVector<double, 3UL> m_target_position, m_calyxPosition;
  blaze::StaticVector<double, 6UL> m_current_q;
  blaze::StaticVector<double, 3UL> m_force_est; // latest EKF tip-force estimate [N]
  std::mutex m_feedback_mutex;                  // guards m_current_q and m_force_est (written on subscriber threads, read on the service thread)
  blaze::StaticVector<double, 4UL> m_rot_phantom_base;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_sub_1, m_callback_group_sub_2, m_callback_group_sub_3; // Callback group for running subscriber callback function on separate thread
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_q;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_tip;
  rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_subscription_force;

  // Subscriber object
  rclcpp::Service<interfaces::srv::Config>::SharedPtr m_manual_target_service;
  rclcpp::Service<interfaces::srv::Planner>::SharedPtr m_command_service;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_tf2;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher_path;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_actuate;
  rclcpp::TimerBase::SharedPtr m_tf2_timer;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf2_listener;

  std::ofstream m_output_file;

  std::condition_variable cv_;
  std::mutex mutex_;

  blaze::StaticVector<double, 3> m_manual_target;
  blaze::StaticVector<double, 3> m_x;

  // std::shared_ptr<CTR> m_CTR_robot;
  // // std::shared_ptr<CTR> m_CTR_robot_TT;
  // std::shared_ptr<Planner> m_motionPlan;
  // // Planner m_motionPlan;
  // std::shared_ptr<CTR> m_CTR_StateValidator;
  // std::shared_ptr<CTR> m_CTR_MotionValidator;
  // std::shared_ptr<CTR> m_CTR_ObjectiveFunction;

  blaze::StaticVector<double, 5UL> m_initGuess;      // initial guess for the solution of the BVP
  blaze::StaticVector<double, 4UL> m_q;              // joint values of the CTR
  const double m_linearActuatorThickness = 30.00E-3; // thickness of the linear actuator stages --> collision avoidance
  const double m_pos_tol = 1.00E-3;

  blaze::HybridMatrix<double, 15000UL, 6UL, blaze::columnMajor> m_JointValues; // Sequence of actuation values

  
  PINNs<kControlInputs> m_ctr_pinn;
  Planner<kControlInputs> m_motionPlan;

  blaze::StaticVector<double, 4UL> m_q_initial_prev = {0.00, 0.00, 0.00, 0.00};

  // Context of the last successfully exported plan; "replanDeployment" resumes from it.
  // Only mutated on the service thread (mutually exclusive group) — no locking needed.
  bool m_has_active_plan = false;
  blaze::StaticVector<double, 4UL> m_q_goal_last;   // joint goal of the active plan
  blaze::StaticVector<double, 3UL> m_target_last;   // task-space target of the active plan
  blaze::StaticVector<double, 3UL> m_force_at_plan; // force the active plan was computed with
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

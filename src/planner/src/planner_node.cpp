#include <chrono>
#include <cstdint>
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
#include "interfaces/srv/transformation.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/srv/planner.hpp"
#include "interfaces/msg/force.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_eigen/tf2_eigen.hpp"

#include <fstream>
#include <filesystem>

#include "ctr_common/csv_io.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "ctr_common/runtime_paths.hpp"
#include "ctr_common/package_log.hpp"

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
#include "Planner.hpp"
#include <limits>
#include <cmath>

#include <future> // needed in order to invoke async "execute functions asynchronously"
#include <boost/tokenizer.hpp>
#include <functional>
#include <blaze/Math.h>

#include <memory.h>
#include <numeric>
#include <algorithm>
#include <cctype>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PathPlannerNode : public rclcpp::Node
{
  // Compile-time CTR sizing — declared first so they are visible in the
  // member-function parameter types further down (parameter lists are not part
  // of the complete-class context).
  const static size_t kBackbonePoints = 150UL; // number of discretized backbone points for the planning CTR
  const static size_t kControlInputs = 4UL;    // number of control inputs (actuated tubes)
  const static size_t kBatch = 1UL;
  // IK convergence tolerance used by inverseKin()/solveInverseKinematics(); distinct
  // from the manager's 3 mm plan-acceptance threshold.
  static constexpr double kIkPosTol = 5.00E-4;

  // Builds the PINN from parameters. Called from the member-initializer list,
  // which is safe: the rclcpp::Node base is fully constructed before members.
  static PINNs<kControlInputs> makePinn(rclcpp::Node &node)
  {
    const std::string model_name =
        node.declare_parameter<std::string>("model_name", "ctr_8x91_0.18_tanh_9K_9K_50K_FP64");
    return PINNs<kControlInputs>(ctr_common::resolveModelsDir(node).string(), model_name, kBatch, kBackbonePoints);
  }

public:
  // default class constructor
  PathPlannerNode() : Node("path_planner"), m_ctr_pinn(makePinn(*this)), m_motionPlan(m_ctr_pinn)
  {
    ctr_common::installPackageFileLog(*this, "planner");

    PathPlannerNode::declare_parameters();
    PathPlannerNode::setup_ros_interfaces();

    // [lifecycle] configuration + trained-domain summary, now that the log file is live.
    const std::string model_name_param = this->get_parameter("model_name").as_string();
    const std::string models_dir_param = this->get_parameter("models_dir").as_string();
    const std::string data_root_param = this->get_parameter("data_root").as_string();
    const auto prismaticRanges = m_ctr_pinn.getPrismaticJointRanges(); // [b1min,b1max,b2min,b2max]
    const auto revoluteRanges = m_ctr_pinn.getRevoluteJointRanges();   // [a1min,a1max,a2min,a2max]
    const double stageClearance = m_ctr_pinn.getStageThickness();

    RCLCPP_INFO(this->get_logger(),
                "[lifecycle] model='%s' models_dir='%s' data_root='%s' temp_dir='%s' solve_time=%.3f s",
                model_name_param.c_str(), models_dir_param.c_str(), data_root_param.c_str(),
                m_tempDir.c_str(), m_solve_time);
    RCLCPP_INFO(this->get_logger(),
                "[lifecycle] trained domain (physics order b1,b2,a1,a2): b1=[%.4f, %.4f] m, b2=[%.4f, %.4f] m, "
                "a1=[%.4f, %.4f] rad, a2=[%.4f, %.4f] rad, stage_clearance=%.4f m",
                prismaticRanges[0UL], prismaticRanges[1UL], prismaticRanges[2UL], prismaticRanges[3UL],
                revoluteRanges[0UL], revoluteRanges[1UL], revoluteRanges[2UL], revoluteRanges[3UL],
                stageClearance);
    if (m_solve_time <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "[lifecycle] solve_time=%.3f s is non-positive; OMPL will return with no plan", m_solve_time);
    }

    RCLCPP_INFO(this->get_logger(), "[lifecycle] Path Planner Node has been initialized.");
  }

  // class destructor
  ~PathPlannerNode()
  {
    m_output_file.close();
  }

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    m_solve_time = this->declare_parameter<double>("solve_time", 3.0); // OMPL solve budget [s]

    const std::string output_dir = (ctr_common::resolveDataRoot(*this, m_packageName) / "Shared_Files").string();
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
    m_subscription_tip = create_subscription<interfaces::msg::Taskspace>("task_space/feedback/base_tool", 10, std::bind(&PathPlannerNode::updateCurrentX, this, _1), subs_current_tip);

    // Subscriber to receive the EKF external tip-force estimate
    m_callback_group_sub_3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subs_force = rclcpp::SubscriptionOptions();
    subs_force.callback_group = m_callback_group_sub_3;
    m_subscription_force = create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&PathPlannerNode::updateForceEstimate, this, _1), subs_force);

    // // path planning service
    // m_manual_target_service = create_service<interfaces::srv::Config>("manual_target", std::bind(&PathPlannerNode::planner_callback, this, _1, _2));

    // Path planning service. Concurrency contract: the manager keeps at most
    // ONE outstanding request (m_flag_planning gates the next one), and the
    // whole write(plannedPath.csv) → respond → manager-read sequence is
    // strictly ordered by the service round-trip, so the ~3 s OMPL solve runs
    // inside this callback by design. Do not add a second client without
    // revisiting that ordering.
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
    m_current_q = ctr_common::wireToPhysics6(msg->position);
    m_q_feedback_received = true;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[comm] q feedback (physics b1,b2,b3,a1,a2,a3) = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                           m_current_q[0UL], m_current_q[1UL], m_current_q[2UL],
                           m_current_q[3UL], m_current_q[4UL], m_current_q[5UL]);
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
    const auto requestStart = std::chrono::high_resolution_clock::now();
    const uint64_t seq = ++m_request_seq;
    const double receiptTime = this->get_clock()->now().seconds();

    RCLCPP_INFO(this->get_logger(),
                "[comm] request #%lu command='%s' target=[%.6f, %.6f, %.6f] m received_at=%.3f s",
                static_cast<unsigned long>(seq), request->command.c_str(),
                request->value[0UL], request->value[1UL], request->value[2UL], receiptTime);

    auto logResponse = [this, seq, requestStart](const std::shared_ptr<interfaces::srv::Planner::Response> &resp, const char *reason)
    {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - requestStart).count();
      RCLCPP_INFO(this->get_logger(),
                  "[comm] response #%lu success=%s value=%.6f message=\"%s\" reason=%s elapsed=%.2f ms",
                  static_cast<unsigned long>(seq), resp->success ? "true" : "false", resp->value,
                  resp->message.c_str(), reason, elapsed_ms);
    };

    if (request->command == "generateTrajectory")
    {
      blaze::StaticVector<double, 3UL> target = request->value;
      double error = 0.0;

      const bool targetFinite = std::isfinite(target[0UL]) && std::isfinite(target[1UL]) && std::isfinite(target[2UL]);
      if (!targetFinite)
      {
        // Genuinely invalid input: an IK solve against a non-finite target
        // produces garbage, so this is rejected here rather than fed through.
        RCLCPP_WARN(this->get_logger(), "[comm] invalid target: non-finite value(s) target=[%f, %f, %f]",
                    target[0UL], target[1UL], target[2UL]);
        response->success = false;
        response->value = 0.0;
        response->message = "invalid target: non-finite value(s) in target";
        logResponse(response, "INVALID_TARGET");
        return;
      }

      try
      {
        // One consistent snapshot of joint feedback and EKF force for the whole request.
        blaze::StaticVector<double, 6UL> q_snapshot;
        blaze::StaticVector<double, 3UL> force;
        {
          std::lock_guard<std::mutex> lock(m_feedback_mutex);
          q_snapshot = m_current_q;
          force = m_force_est;
          if (!m_q_feedback_received)
          {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "[comm] no joint_space/feedback received yet; using default q=0 snapshot for request #%lu",
                                  static_cast<unsigned long>(seq));
          }
        }

        // Map the 6-element current configuration [β₁, β₂, β₃, α₁, α₂, α₃] (tube-3
        // entries are static/zero) down to the 4 actuated inputs [β₁, β₂, α₁, α₂].
        blaze::StaticVector<double, 4UL> q_initial = {q_snapshot[0UL], q_snapshot[1UL], q_snapshot[3UL], q_snapshot[4UL]};
        blaze::StaticVector<double, 4UL> q_final = q_initial;

        // Safe here: no solve is in flight (mutually exclusive service group).
        // Propagates to the FTL objective (cache cleared) and informed samplers.
        m_motionPlan.setCTR_externalForce(force);
        RCLCPP_INFO(this->get_logger(), "[plan] force estimate f = [%.4f, %.4f, %.4f] N", force[0UL], force[1UL], force[2UL]);

        const std::string startViolation = m_ctr_pinn.domainViolation(q_initial);
        if (!startViolation.empty())
        {
          RCLCPP_WARN(this->get_logger(), "[pinn] start state outside trained domain: %s", startViolation.c_str());
        }

        error = inverseKin(target, q_final, force);
        const bool ik_converged = error <= kIkPosTol;

        RCLCPP_INFO(this->get_logger(), "[plan] initial q = [%.4f, %.4f, %.4f, %.4f]", q_initial[0UL], q_initial[1UL], q_initial[2UL], q_initial[3UL]);
        RCLCPP_INFO(this->get_logger(), "[plan] final q = [%.4f, %.4f, %.4f, %.4f]", q_final[0UL], q_final[1UL], q_final[2UL], q_final[3UL]);

        bool planning_status = plan(q_initial, q_final, force);
        if (planning_status)
        {
          m_target_last = target;
          response->success = true;
          response->value = error;
          response->message = "Path generated successfully.";
          logResponse(response, ik_converged ? "OK" : "IK_NOT_CONVERGED");
        }
        else
        {
          response->success = false;
          response->value = error;
          response->message = "Planning failed to find a solution.";
          logResponse(response, "PLAN_FAILED");
          return;
        }
      }
      catch (const std::exception &e)
      {
        const std::string reason = classifyPlannerException(e.what());
        RCLCPP_ERROR(this->get_logger(), "[plan] generateTrajectory exception (%s): %s", reason.c_str(), e.what());
        response->success = false;
        response->value = error;
        response->message = e.what();
        logResponse(response, reason.c_str());
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
        if (!m_q_feedback_received)
        {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "[comm] no joint_space/feedback received yet; using default q=0 snapshot for request #%lu",
                                static_cast<unsigned long>(seq));
        }
      }

      if (!m_has_active_plan)
      {
        response->success = false;
        response->value = 0.0;
        response->message = "replanDeployment: no active plan/goal - call generateTrajectory first.";
        logResponse(response, "NO_ACTIVE_PLAN");
        return;
      }

      try
      {
        blaze::StaticVector<double, 4UL> q_now = {q_snapshot[0UL], q_snapshot[1UL], q_snapshot[3UL], q_snapshot[4UL]};

        RCLCPP_INFO(this->get_logger(), "[plan] replan request f = [%.4f, %.4f, %.4f] N (active plan used f = [%.4f, %.4f, %.4f] N), q_now = [%.4f, %.4f, %.4f, %.4f]",
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
          RCLCPP_INFO(this->get_logger(), "[plan] replanned deployment in %ld ms: %s (FTL cost %.6f)",
                      static_cast<long>(elapsed), m_motionPlan.lastDeploymentScheduleName().c_str(), m_motionPlan.lastDeploymentCost());
          logResponse(response, "OK");
        }
        else
        {
          // planDeployment() cleared the solution paths, so nothing stale can be
          // exported; the previous CSV stays on disk and the master keeps
          // executing the old plan.
          response->success = false;
          response->value = 0.0;
          response->message = "replanDeployment failed (alpha mismatch, invalid start, or no valid schedule).";
          RCLCPP_WARN(this->get_logger(), "[plan] deployment replanning failed after %ld ms.", static_cast<long>(elapsed));
          logResponse(response, "PLAN_FAILED");
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "[plan] replanDeployment exception: %s", e.what());
        response->success = false;
        response->value = 0.0;
        response->message = e.what();
        logResponse(response, "EXCEPTION");
      }
    }
    else
    {
      response->success = false;
      response->value = 0.0;
      response->message = "Invalid planner command";
      RCLCPP_WARN(this->get_logger(), "[comm] invalid command '%s'", request->command.c_str());
      logResponse(response, "INVALID_COMMAND");
    }
  }

  // Classifies a Planner.hpp exception message into a stable reason code for
  // [comm]/[plan] log summaries. Case-insensitive substring match on the
  // exception text; never alters response->message.
  static std::string classifyPlannerException(const std::string &what)
  {
    std::string lower = what;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (lower.find("start state") != std::string::npos)
    {
      return "START_INVALID";
    }
    if (lower.find("goal state") != std::string::npos)
    {
      return "GOAL_INVALID";
    }
    return "EXCEPTION";
  }

  // Service callback to triget tasks, enable, and control mode section
  double inverseKin(const blaze::StaticVector<double, 3UL> &target, blaze::StaticVector<double, 4UL> &q, const blaze::StaticVector<double, 3UL> &force)
  {
    blaze::StaticVector<double, 3UL> tipPosition;
    PosCtrlReport report;

    RCLCPP_DEBUG(this->get_logger(),
                 "[ik] seed q=[%.4f, %.4f, %.4f, %.4f] target=[%.4f, %.4f, %.4f] m posTol=%.2e m f=[%.4f, %.4f, %.4f] N",
                 q[0UL], q[1UL], q[2UL], q[3UL], target[0UL], target[1UL], target[2UL], kIkPosTol,
                 force[0UL], force[1UL], force[2UL]);

    // run IK to compute q_final; force-aware: uses the force registered via setCTR_externalForce()
    auto start = std::chrono::high_resolution_clock::now();
    const bool converged = m_motionPlan.solveInverseKinematics(q, target, kIkPosTol, &report);
    auto end = std::chrono::high_resolution_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    m_ctr_pinn.getPosDistal(q, force, tipPosition);
    const double error = blaze::norm(target - tipPosition);

    const char *terminationStr = "MaxIterations";
    switch (report.termination)
    {
      case PosCtrlReport::Termination::AlreadyWithinTolerance: terminationStr = "AlreadyWithinTolerance"; break;
      case PosCtrlReport::Termination::Converged: terminationStr = "Converged"; break;
      case PosCtrlReport::Termination::Stalled: terminationStr = "Stalled"; break;
      case PosCtrlReport::Termination::MaxIterations: terminationStr = "MaxIterations"; break;
    }

    RCLCPP_INFO(this->get_logger(),
                "[ik] q=[%.4f, %.4f, %.4f, %.4f] tip=[%.4f, %.4f, %.4f] error=%.4f mm iterations=%zu "
                "termination=%s initial_error=%.4f mm final_error=%.4f mm limit_hits=%zu elapsed=%.2f ms",
                q[0UL], q[1UL], q[2UL], q[3UL], tipPosition[0UL], tipPosition[1UL], tipPosition[2UL],
                error * 1.00E3, report.iterations, terminationStr,
                report.initialError * 1.00E3, report.finalError * 1.00E3, report.prismaticLimitHits, elapsed_ms);

    if (!converged)
    {
      RCLCPP_WARN(this->get_logger(), "[ik] not converged: error=%.4f mm > posTol=%.4f mm (termination=%s)",
                  error * 1.00E3, kIkPosTol * 1.00E3, terminationStr);
    }

    const std::string solutionViolation = m_ctr_pinn.domainViolation(q);
    if (!solutionViolation.empty())
    {
      RCLCPP_WARN(this->get_logger(), "[pinn] IK solution outside trained domain: %s", solutionViolation.c_str());
    }

    return error;
  }

  // Service callback to triget tasks, enable, and control mode section
  bool plan(const blaze::StaticVector<double, 4UL> &q_initial, const blaze::StaticVector<double, 4UL> &q_final, const blaze::StaticVector<double, 3UL> &force)
  {
    bool planning_status = false;

    // setting the initial state: initial configuration of the robot
    m_motionPlan.setStartState(q_initial);
    m_motionPlan.setGoalState(q_final);

    blaze::StaticVector<double, 4UL> scale = {1.00, 1.00, 20.0, 20.0};
    double norm_diff = blaze::norm((m_q_initial_prev - q_initial) / scale);

    RCLCPP_INFO(this->get_logger(), "[plan] start q=[%.4f, %.4f, %.4f, %.4f] goal q=[%.4f, %.4f, %.4f, %.4f]",
                q_initial[0UL], q_initial[1UL], q_initial[2UL], q_initial[3UL],
                q_final[0UL], q_final[1UL], q_final[2UL], q_final[3UL]);

    // setting up the planning problem and its definitions
    const double runTime = m_solve_time; // Planning time in seconds

    // completely silence OMPL's own logging:
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    const std::string plannedPathFile = m_tempDir + "/plannedPath.csv";
    auto start = std::chrono::high_resolution_clock::now();
    // motionPlan.plan(runTime, Planner::optimalPlanner::PLANNER_RRT, Planner::planningObjective::OBJECTIVE_BACKBONE_LENGTH, plannedPathFile);
    planning_status = m_motionPlan.planTwoPhase(runTime, Planner<kControlInputs>::optimalPlanner::PLANNER_RRT_CONNECT);
    m_first_plan = false;
    auto end = std::chrono::high_resolution_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    m_q_initial_prev = q_initial;

    if (planning_status)
    {
      // Export and publish only real solutions. On failure the previous CSV stays
      // on disk, so the master keeps executing the old plan (fallback contract).
      m_motionPlan.writeSolutionToFile(plannedPathFile);
      RCLCPP_INFO(this->get_logger(), "[plan] planTwoPhase succeeded in %.2f ms, wrote %s", elapsed_ms, plannedPathFile.c_str());

      m_has_active_plan = true;
      m_q_goal_last = q_final;
      m_force_at_plan = force;
      publishTaskSpacePath(); // FK below uses m_force_at_plan — keep after the update
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[plan] planTwoPhase failed after %.2f ms", elapsed_ms);
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
      RCLCPP_ERROR(get_logger(), "[traj] failed to read planned path or empty CSV");
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
    RCLCPP_INFO(get_logger(), "[traj] published task-space path with %zu points.", pathSize);

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
      const std::filesystem::path file_path =
          ctr_common::resolveDataRoot(*this, m_packageName) / "Shared_Files" / fileName;

      const auto rows = ctr_common::csv::readNumericCsv(file_path);
      if (!rows)
      {
          RCLCPP_ERROR(get_logger(), "[traj] failed to open file: %s", file_path.c_str());
          return;
      }

      init_q_list.clear();

      // writeSolutionToFile() emits no header; every line is a control-space state.
      for (const auto &row : *rows)
      {
          if (row.size() == kControlInputs)
          {
              init_q_list.push_back({row[0], row[1], row[2], row[3]});
          }
          else
          {
              RCLCPP_WARN(get_logger(), "[traj] row does not contain exactly %zu values (got %zu)", kControlInputs, row.size());
          }
      }

      RCLCPP_INFO(get_logger(), "[traj] loaded %zu path points from CSV file.", init_q_list.size());
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

private:
  // Member variables
  const std::string m_packageName = "planner";

  bool m_first_plan = true;

  size_t count_;
  int m_traj_counter = 0;
  std::string m_tempDir;
  double m_solve_time = 3.0;
  double m_t_init = 0.00;
  double m_sample_time;
  bool m_flag_new_feedback = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  blaze::StaticVector<double, 3UL> m_target_position, m_calyxPosition;
  blaze::StaticVector<double, 6UL> m_current_q;
  blaze::StaticVector<double, 3UL> m_force_est; // latest EKF tip-force estimate [N]
  std::mutex m_feedback_mutex;                  // guards m_current_q and m_force_est (written on subscriber threads, read on the service thread)
  bool m_q_feedback_received = false;           // true once joint_space/feedback has delivered at least one message
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


  
  PINNs<kControlInputs> m_ctr_pinn;
  Planner<kControlInputs> m_motionPlan;

  blaze::StaticVector<double, 4UL> m_q_initial_prev = {0.00, 0.00, 0.00, 0.00};

  // Context of the last successfully exported plan; "replanDeployment" resumes from it.
  // Only mutated on the service thread (mutually exclusive group) — no locking needed.
  bool m_has_active_plan = false;
  blaze::StaticVector<double, 4UL> m_q_goal_last;   // joint goal of the active plan
  blaze::StaticVector<double, 3UL> m_target_last;   // task-space target of the active plan
  blaze::StaticVector<double, 3UL> m_force_at_plan; // force the active plan was computed with
  uint64_t m_request_seq = 0UL;                     // monotonically increasing planner/command request counter, for [comm] log correlation
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

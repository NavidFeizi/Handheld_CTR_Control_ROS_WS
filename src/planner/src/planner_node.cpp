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
#include "ctr_common/diag_csv.hpp"
#include "ctr_common/finite_guard.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "ctr_common/output_session.hpp"
#include "ctr_common/runtime_paths.hpp"

#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
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

class PathPlannerNode : public rclcpp::Node
{
  // Compile-time CTR sizing — declared first so they are visible in the
  // member-function parameter types further down (parameter lists are not part
  // of the complete-class context).
  const static size_t kBackbonePoints = 150UL; // number of discretized backbone points for the planning CTR
  const static size_t kControlInputs = 4UL;    // number of control inputs (actuated tubes)
  const static size_t kBatch = 1UL;

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
    m_solve_time = this->declare_parameter<double>("solve_time", 3.0); // OMPL solve budget [s]

    // Routes the planning library's logging::debug() lines (state-space bounds,
    // planner range, rewiring diagnostics) to stdout. Read once at startup.
    logging::verbose = this->declare_parameter<bool>("verbose_planner_log", false);

    const std::filesystem::path data_root = ctr_common::resolveDataRoot(*this, m_packageName);
    const std::string output_dir = (data_root / "Shared_Files").string();
    this->declare_parameter<std::string>("temp_dir", output_dir);
    m_tempDir = this->get_parameter("temp_dir").as_string();
    if (!std::filesystem::exists(m_tempDir))
    {
      std::filesystem::create_directories(m_tempDir);
    }

    // One diagnostic record per planning request, structured for offline
    // correlation with the manager's manager_diag.csv (join on wall time).
    const auto diag_dir = ctr_common::makeSessionDir(data_root / "Output_Files" / "diagnostics", "planner");
    m_diag.configure(diag_dir / "planner_diag.csv",
                     "req_id,wall_time,command,target_x,target_y,target_z,target_azimuth,"
                     "fext_x,fext_y,fext_z,"
                     "start_b1,start_b2,start_a1,start_a2,"
                     "ik_b1,ik_b2,ik_a1,ik_a2,goal_repr_a1,goal_repr_a2,"
                     "ik_residual_m,ik_converged,ik_time_s,ik_iterations,ik_restarts,"
                     "ik_clamped_steps,ik_alpha_capped_steps,ik_nonmonotonic_steps,ik_max_jinv_norm,ik_initial_error_m,"
                     "proj_delta_b1,proj_delta_b2,proj_delta_a1,proj_delta_a2,"
                     "max_abs_alpha2_queried,max_abs_alpha_rel_queried,"
                     "dalpha1_travel,dalpha2_travel,goal_alpha2_off_principal,"
                     "plan_time_s,plan_success,message,"
                     "endpoint_err_m,endpoint_tip_x,endpoint_tip_y,endpoint_tip_z");
    RCLCPP_INFO(this->get_logger(), "Planner diagnostics CSV: %s", m_diag.path().c_str());
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
    // Reject and hold the last good value. This force reaches the network as an
    // input via the IK, the FTL swept cost and the informed samplers, so a
    // non-finite sample would make IK a no-op and every candidate cost NaN.
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->z))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Non-finite force estimate [%.3f, %.3f, %.3f] N - holding the last good value",
                           msg->x, msg->y, msg->z);
      return;
    }

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
      double error = 0.0;

      // ---- per-request diagnostic record (written on EVERY exit path) ----
      const size_t req_id = ++m_req_counter;
      IkDiagnostics ik_diag;
      double ik_seconds = 0.0, plan_seconds = 0.0;
      bool planning_status = false;
      std::string outcome_message;
      blaze::StaticVector<double, 4UL> q_initial(0.0), q_final(0.0);
      std::array<double, 4UL> goal_repr{};
      // Hoisted out of the try so the diagnostics row records the f_ext the
      // request actually used. It was console-only before, which made a bad
      // force impossible to correlate with a bad plan after the fact.
      blaze::StaticVector<double, 3UL> force(0.0);

      // Stale-value guard: the diag row is written on every exit path, so an
      // early rejection must not report the previous request's endpoint.
      m_plan_endpoint_err = std::numeric_limits<double>::quiet_NaN();
      m_plan_endpoint_tip = blaze::StaticVector<double, 3UL>(std::numeric_limits<double>::quiet_NaN());

      try
      {
        // One consistent snapshot of joint feedback and EKF force for the whole request.
        blaze::StaticVector<double, 6UL> q_snapshot;
        {
          std::lock_guard<std::mutex> lock(m_feedback_mutex);
          q_snapshot = m_current_q;
          force = m_force_est;
        }

        // Map the 6-element current configuration [β₁, β₂, β₃, α₁, α₂, α₃] (tube-3
        // entries are static/zero) down to the 4 actuated inputs [β₁, β₂, α₁, α₂].
        q_initial = {q_snapshot[0UL], q_snapshot[1UL], q_snapshot[3UL], q_snapshot[4UL]};
        q_final = q_initial;

        // Refuse to plan on inputs that cannot produce a meaningful answer. The
        // force and the joint snapshot are both NETWORK INPUTS: a non-finite
        // value makes every FK call return NaN, and posCTRL's descent gate is
        // `while (dist2Tgt > posTol)`, which is FALSE for NaN -- so IK silently
        // performs zero iterations, returns the seed unchanged, and the planner
        // then "succeeds" on a start-equals-goal path. That is exactly how a
        // deployment completed 57 mm from its target with Success: Yes.
        if (!ctr_common::allFinite(force))
        {
          throw std::runtime_error(
              "EKF force estimate is not finite - refusing to plan (restart ekf_node)");
        }
        if (!ctr_common::allFinite(q_initial))
        {
          throw std::runtime_error(
              "Joint feedback is not finite - refusing to plan");
        }
        if (!ctr_common::allFinite(target))
        {
          throw std::runtime_error(
              "Requested target is not finite - refusing to plan");
        }

        // Safe here: no solve is in flight (mutually exclusive service group).
        // Propagates to the FTL objective (cache cleared) and informed samplers.
        m_motionPlan.setCTR_externalForce(force);
        RCLCPP_INFO(this->get_logger(), "Planning with force estimate: f = [%.4f, %.4f, %.4f] N", force[0UL], force[1UL], force[2UL]);
        RCLCPP_INFO(this->get_logger(), "Request #%zu: target = [%.4f, %.4f, %.4f] m, azimuth atan2(y,x) = %.4f rad",
                    req_id, target[0UL], target[1UL], target[2UL], std::atan2(target[1UL], target[0UL]));

        error = inverseKin(target, q_final, force, ik_diag, ik_seconds);

        // Mirror of the representative selection setGoalState() performs, so the
        // record shows the α pair the planner actually planned toward.
        goal_repr = ctr_kinematics_pinn::nearestGoalRepresentative(
            {q_final[0UL], q_final[1UL], q_final[2UL], q_final[3UL]},
            {q_initial[0UL], q_initial[1UL], q_initial[2UL], q_initial[3UL]},
            m_ctr_pinn.getJointLimits4());

        RCLCPP_INFO(this->get_logger(), "Initial config: q = %.4f, %.4f, %.4f, %.4f", q_initial[0UL], q_initial[1UL], q_initial[2UL], q_initial[3UL]);
        RCLCPP_INFO(this->get_logger(), "Final config: q = %.4f, %.4f, %.4f, %.4f (goal alpha representative: %.4f, %.4f)",
                    q_final[0UL], q_final[1UL], q_final[2UL], q_final[3UL], goal_repr[2UL], goal_repr[3UL]);

        // Reject a goal that IK never actually moved to.
        //
        // setGoalState() validates only box bounds and tube ordering, so a goal
        // identical to the start is perfectly "valid": Phase 1 solves instantly,
        // Phase 2 emits a single waypoint, every FTL candidate scores exactly 0
        // (the `wps.size() < 2` early return, not a measurement), and the plan is
        // reported EXACT_SOLUTION. Nothing downstream could tell that apart from
        // a real plan.
        if (!std::isfinite(error))
        {
          throw std::runtime_error(
              "IK residual is not finite - refusing to emit a plan");
        }
        if (blaze::maxNorm(q_final - q_initial) < 1.0e-9 &&
            blaze::norm(target - m_target_last) > 1.0e-9)
        {
          throw std::runtime_error(
              "IK returned the start configuration unchanged for a moved target - "
              "refusing to emit a no-op plan (residual " + std::to_string(error) + " m)");
        }

        planning_status = plan(target, q_initial, q_final, force, plan_seconds);
        if (planning_status)
        {
          m_target_last = target;
          response->success = true;
          response->value = error;
          response->message = "Path generated successfully.";
          outcome_message = response->message;
        }
        else
        {
          response->success = false;
          response->value = error;
          response->message = "Planning failed to find a solution.";
          outcome_message = response->message;
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", e.what());
        response->success = false;
        // NOTE: 0.0 here means "threw before IK finished", not a perfect solve.
        response->value = error;
        response->message = e.what();
        outcome_message = e.what();
      }

      writePlanDiagRecord(req_id, "generateTrajectory", target, q_initial, q_final, goal_repr,
                          force, error, ik_diag, ik_seconds, plan_seconds, planning_status, outcome_message);
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
        RCLCPP_ERROR(this->get_logger(), "Deployment replanning failed: %s", e.what());
        response->success = false;
        response->value = 0.0;
        response->message = e.what();
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid planner command '%s' - expected 'generateTrajectory' or 'replanDeployment'",
                   request->command.c_str());
      response->success = false;
      response->value = 0.0;
      response->message = "Invalid planner command";
    }
  }

  // Service callback to triget tasks, enable, and control mode section
  double inverseKin(const blaze::StaticVector<double, 3UL> &target, blaze::StaticVector<double, 4UL> &q, const blaze::StaticVector<double, 3UL> &force,
                    IkDiagnostics &diag, double &ikSeconds)
  {
    // run IK to compute q_final. 1 mm, not the tighter 0.5 mm this used to ask
    // for: the extra 0.5 mm is well inside the manager's own 3 mm acceptance gate
    // (k_ik_error_threshold), so a solve that lands between 0.5 and 1 mm is a
    // perfectly plannable target and should be reported as converged rather than
    // ground on until the iteration budget runs out.
    constexpr double posTolerance = 1.00E-3;
    blaze::StaticVector<double, 3UL> tipPosition;

    RCLCPP_INFO(this->get_logger(), "Running IK...");
    auto start = std::chrono::high_resolution_clock::now();
    // force-aware IK: uses the force registered via setCTR_externalForce()
    const bool converged = m_motionPlan.solveInverseKinematics(q, target, posTolerance, diag);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ikSeconds = elapsed * 1.00E-3;
    // The IK wall time is NOT covered by solve_time: it is additive to the
    // service round-trip, against the manager's planner_timeout_s budget.
    RCLCPP_INFO(this->get_logger(), "IK time: %.3f seconds (%zu iterations, %zu restarts, %zu alpha-capped steps, max ||J^+|| = %.3e)",
                ikSeconds, diag.iterations, diag.restarts, diag.alphaCappedSteps, diag.maxJinvNorm);

    m_ctr_pinn.getPosDistal(q, force, tipPosition);
    const double residual = blaze::norm(target - tipPosition);
    RCLCPP_INFO(this->get_logger(),
                "IK result: q = [%.4f, %.4f, %.4f, %.4f], target = [%.4f, %.4f, %.4f], tip after IK = [%.4f, %.4f, %.4f], error = %.3f mm",
                q[0UL], q[1UL], q[2UL], q[3UL], target[0UL], target[1UL], target[2UL],
                tipPosition[0UL], tipPosition[1UL], tipPosition[2UL], residual * 1.00E3);

    // Domain tripwire. Only the RELATIVE term is a live check: posCTRL's
    // wrapAngles() pins α₂ onto the principal branch [-π, π) on every iterate,
    // so maxAbsAlpha2Queried is structurally incapable of exceeding π, let
    // alone 1.5π -- the old α₂ clause could never fire and its silence meant
    // nothing. What CAN leave the trained box is the α₁ − α₂ offset during the
    // descent, and the RESULT: posCTRL returns on the principal branch, but the
    // representative the planner will actually plan toward is shifted by 2πk
    // and has to stay inside α₂'s absolute travel.
    const auto &lim4 = m_ctr_pinn.getJointLimits4();
    if (diag.maxAbsAlphaRelQueried > M_PI + 1.0E-9)
    {
      RCLCPP_WARN(this->get_logger(),
                  "IK queried the network OUTSIDE its trained alpha domain: max|alpha1-alpha2| = %.4f (limit pi). "
                  "FK/Jacobian values there are extrapolation - do not trust this solve.",
                  diag.maxAbsAlphaRelQueried);
    }
    if (!ctr_kinematics_pinn::alphaFeasible(q[2UL], q[3UL], lim4, 1.0E-9))
    {
      RCLCPP_WARN(this->get_logger(),
                  "IK returned an alpha pair outside the trained/travel box: alpha1 = %.4f, alpha2 = %.4f "
                  "(alpha2 limit +/-%.4f, |alpha1 - alpha2| limit pi). setGoalState() will reject this.",
                  q[2UL], q[3UL], lim4.alpha2_absolute[1UL]);
    }

    // posCTRL is best-effort: it returns its closest-seen configuration without
    // signalling failure. Planning still proceeds with that configuration (as it
    // always has), but a miss must be visible in the ROS log -- the manager
    // silently rejects any plan whose residual exceeds its own 3 mm gate
    // (k_ik_error_threshold, manager/include/manager/master_node.hpp).
    if (!converged)
    {
      RCLCPP_WARN(this->get_logger(),
                  "IK did not converge in %.3f s: residual %.3f mm (tol %.3f mm) for target [%.4f, %.4f, %.4f] "
                  "- the manager rejects plans above 3.000 mm",
                  elapsed * 1.00E-3, residual * 1.00E3, posTolerance * 1.00E3,
                  target[0UL], target[1UL], target[2UL]);
    }

    return residual;
  }

  // Service callback to triget tasks, enable, and control mode section
  bool plan(const blaze::StaticVector<double, 3UL> &target,
            const blaze::StaticVector<double, 4UL> &q_initial, const blaze::StaticVector<double, 4UL> &q_final, const blaze::StaticVector<double, 3UL> &force,
            double &planSeconds)
  {
    bool planning_status = false;

    // setting the initial state: initial configuration of the robot
    m_motionPlan.setStartState(q_initial);

    // The goal the planner PLANS TOWARD is the 2πk representative nearest the
    // start, which setGoalState() selects internally. Capture the same value
    // here: m_q_goal_last used to be set from the raw posCTRL output on the
    // principal branch, so once the representative shift fired, the robot was
    // physically executing a goal 2π away from the one stored. planDeployment()
    // compares the live alphas against it with a RAW difference and a 0.05 rad
    // tolerance, saw ~6.283 rad and aborted -- every mid-deployment replan was
    // rejected for the rest of the deployment, silently disabling force-drift
    // correction. Requires the start state to be set first.
    const auto q_goal_planned = m_motionPlan.selectGoalRepresentative(q_final);
    m_motionPlan.setGoalState(q_final);

    RCLCPP_INFO(this->get_logger(), "Start state: [%.4f, %.4f, %.4f, %.4f], Goal state: [%.4f, %.4f, %.4f, %.4f]",
                q_initial[0UL], q_initial[1UL], q_initial[2UL], q_initial[3UL],
                q_goal_planned[0UL], q_goal_planned[1UL], q_goal_planned[2UL], q_goal_planned[3UL]);

    // setting up the planning problem and its definitions
    const double runTime = m_solve_time; // Planning time in seconds

    // OMPL's own warnings/errors are diagnostic gold ("Skipping invalid start
    // state", "Unable to sample any valid states for goal tree") -- LOG_NONE
    // used to hide all of them from the lab logs.
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    const std::string plannedPathFile = m_tempDir + "/plannedPath.csv";
    auto start = std::chrono::high_resolution_clock::now();
    // motionPlan.plan(runTime, Planner::optimalPlanner::PLANNER_RRT, Planner::planningObjective::OBJECTIVE_BACKBONE_LENGTH, plannedPathFile);
    planning_status = m_motionPlan.planTwoPhase(runTime, Planner<kControlInputs>::optimalPlanner::PLANNER_RRT_CONNECT);
    m_first_plan = false;
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    planSeconds = elapsed * 1.00E-3;
    RCLCPP_INFO(this->get_logger(), "Planning time: %.3f seconds", planSeconds);

    m_q_initial_prev = q_initial;

    if (planning_status)
    {
      // Export and publish only real solutions. On failure the previous CSV stays
      // on disk, so the master keeps executing the old plan (fallback contract).
      m_motionPlan.writeSolutionToFile(plannedPathFile);
      RCLCPP_INFO(this->get_logger(), "Finished planning - saved plan in: %s", plannedPathFile.c_str());

      m_has_active_plan = true;
      m_q_goal_last = q_goal_planned;
      m_force_at_plan = force;
      checkPlanEndpoint(target, force);
      publishTaskSpacePath(); // FK below uses m_force_at_plan — keep after the update
    }

    return planning_status;
  }

  // One structured record per generateTrajectory request, on every exit path.
  void writePlanDiagRecord(const size_t req_id, const char *command,
                           const blaze::StaticVector<double, 3UL> &target,
                           const blaze::StaticVector<double, 4UL> &q_initial,
                           const blaze::StaticVector<double, 4UL> &q_final,
                           const std::array<double, 4UL> &goal_repr,
                           const blaze::StaticVector<double, 3UL> &force,
                           const double ik_residual, const IkDiagnostics &d,
                           const double ik_seconds, const double plan_seconds,
                           const bool plan_success, const std::string &message)
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6);
    os << req_id << ',' << ctr_common::currentTimestamp() << ',' << command << ','
       << target[0UL] << ',' << target[1UL] << ',' << target[2UL] << ','
       << std::atan2(target[1UL], target[0UL]) << ','
       << force[0UL] << ',' << force[1UL] << ',' << force[2UL] << ','
       << q_initial[0UL] << ',' << q_initial[1UL] << ',' << q_initial[2UL] << ',' << q_initial[3UL] << ','
       << q_final[0UL] << ',' << q_final[1UL] << ',' << q_final[2UL] << ',' << q_final[3UL] << ','
       << goal_repr[2UL] << ',' << goal_repr[3UL] << ','
       << ik_residual << ',' << (d.converged ? 1 : 0) << ',' << ik_seconds << ','
       << d.iterations << ',' << d.restarts << ','
       << d.clampedSteps << ',' << d.alphaCappedSteps << ',' << d.nonMonotonicSteps << ','
       << d.maxJinvNorm << ',' << d.initialError << ','
       << d.projectionDelta[0UL] << ',' << d.projectionDelta[1UL] << ','
       << d.projectionDelta[2UL] << ',' << d.projectionDelta[3UL] << ','
       << d.maxAbsAlpha2Queried << ',' << d.maxAbsAlphaRelQueried << ','
       << (goal_repr[2UL] - q_initial[2UL]) << ',' << (goal_repr[3UL] - q_initial[3UL]) << ','
       << (std::fabs(q_final[3UL]) > M_PI ? 1 : 0) << ','
       << plan_seconds << ',' << (plan_success ? 1 : 0) << ','
       << '"' << message << '"' << ','
       << m_plan_endpoint_err << ',' << m_plan_endpoint_tip[0UL] << ','
       << m_plan_endpoint_tip[1UL] << ',' << m_plan_endpoint_tip[2UL];
    m_diag.append(os.str());
  }

  /// The end-to-end check nothing else in the pipeline performs.
  ///
  /// Every existing gate is about the GOAL CONFIGURATION: the IK residual is
  /// measured at q_final, setGoalState() validates only box bounds and tube
  /// ordering, and OMPL's own goal threshold is a mixed metres/radians
  /// Euclidean over joint space. Nothing ever asks whether the tip of the path
  /// that was actually written lands on the requested target. When a plan
  /// executes and the tip ends up somewhere else, this is the number that says
  /// whether the plan was wrong or the execution was.
  ///
  /// Reads the CSV back rather than the in-memory path on purpose: the file is
  /// what the manager will execute.
  void checkPlanEndpoint(const blaze::StaticVector<double, 3UL> &target,
                         const blaze::StaticVector<double, 3UL> &force)
  {
    std::vector<blaze::StaticVector<double, kControlInputs>> q_list;
    read_path_from_csv(q_list, "plannedPath.csv");
    if (q_list.empty())
    {
      RCLCPP_ERROR(get_logger(), "Plan endpoint check: plannedPath.csv is empty or unreadable");
      return;
    }

    const auto &q_end = q_list.back();
    m_ctr_pinn.getPosDistal(q_end, force, m_plan_endpoint_tip);
    m_plan_endpoint_err = blaze::norm(target - m_plan_endpoint_tip);

    RCLCPP_INFO(get_logger(),
                "Plan endpoint: q = [%.4f, %.4f, %.4f, %.4f], FK tip = [%.4f, %.4f, %.4f], "
                "target = [%.4f, %.4f, %.4f], |FK(endpoint) - target| = %.3f mm",
                q_end[0UL], q_end[1UL], q_end[2UL], q_end[3UL],
                m_plan_endpoint_tip[0UL], m_plan_endpoint_tip[1UL], m_plan_endpoint_tip[2UL],
                target[0UL], target[1UL], target[2UL], m_plan_endpoint_err * 1.00E3);

    // The path is supposed to terminate exactly at the IK goal, so this should
    // equal the IK residual. A larger value means the written path does not end
    // where the plan was solved for -- a Phase 1/Phase 2 stitching gap, an
    // approximate OMPL solution accepted as exact, or a truncated export.
    if (m_plan_endpoint_err > k_endpoint_warn_m)
    {
      RCLCPP_WARN(get_logger(),
                  "Plan endpoint is %.3f mm from the target (warn above %.3f mm). The manager's "
                  "acceptance gate only sees the IK residual, so this discrepancy would otherwise "
                  "be invisible: the written path does not end where IK solved.",
                  m_plan_endpoint_err * 1.00E3, k_endpoint_warn_m * 1.00E3);
    }
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
      // Must return: falling through published an empty Float64MultiArray, which reads
      // downstream as a valid zero-length path rather than a failure.
      RCLCPP_ERROR(get_logger(), "Failed to read planned path or empty CSV - not publishing a path");
      std::lock_guard<std::mutex> lock(m_feedback_mutex);
      m_q = {m_current_q[0UL], m_current_q[1UL], m_current_q[3UL], m_current_q[4UL]};
      return;
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
          RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
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
              RCLCPP_WARN(get_logger(), "Row does not contain exactly %zu values (got %zu)", kControlInputs, row.size());
          }
      }

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

      // A waypoint is kept when EITHER prismatic or revolute motion since the
      // last kept waypoint is significant.
      //   - Filtering on β₁ alone (the original rule) collapsed every
      //     pure-rotation segment -- the whole Phase 1 rotation -- into a single
      //     commanded step, leaving the α slew entirely unmanaged.
      //   - It also made β₂ travel invisible, so a β₂-dominant deployment
      //     collapsed to one unmanaged jump. Phase 2's "least-travel stops
      //     first" schedule makes β₂-dominant sub-phases routine, so both
      //     prismatic joints have to be measured.
      // Keep this rule identical to manager/csv_path_io.hpp's copy, which is the
      // one that actually drives the robot.
      constexpr double alpha_step = 0.10; // [rad] ≈ 5.7° per commanded step
      size_t prev_idx = 0;
      q_list_out.push_back(q_list_in[0]);

      for (size_t i = 1; i + 1 < q_list_in.size(); ++i)
      {
          const double d_beta  = std::max(std::abs(q_list_in[i][0UL] - q_list_in[prev_idx][0UL]),
                                          std::abs(q_list_in[i][1UL] - q_list_in[prev_idx][1UL]));
          const double d_alpha = std::max(std::abs(q_list_in[i][2UL] - q_list_in[prev_idx][2UL]),
                                          std::abs(q_list_in[i][3UL] - q_list_in[prev_idx][3UL]));
          if (d_beta >= step_size || d_alpha >= alpha_step)
          {
              prev_idx = i;
              q_list_out.push_back(q_list_in[i]);
          }
      }
      // Always end on the final configuration, exactly once. The loop stops
      // before the last element, so this cannot duplicate a waypoint the loop
      // already kept.
      if (q_list_in.size() > 1UL)
      {
          q_list_out.push_back(q_list_in.back());
      }

      // Downsampler telemetry: a large per-step Δα here means a rotation was
      // collapsed and would execute as one unmanaged swing.
      double max_step_alpha = 0.0;
      for (size_t i = 1; i < q_list_out.size(); ++i)
          max_step_alpha = std::max({max_step_alpha,
                                     std::abs(q_list_out[i][2UL] - q_list_out[i - 1UL][2UL]),
                                     std::abs(q_list_out[i][3UL] - q_list_out[i - 1UL][3UL])});
      RCLCPP_INFO(this->get_logger(), "Path downsampled %zu -> %zu waypoints (max per-step dAlpha = %.3f rad)",
                  q_list_in.size(), q_list_out.size(), max_step_alpha);

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

  // Per-request diagnostics (planner_diag.csv, DiagCsv is internally mutex-guarded).
  ctr_common::DiagCsv m_diag;
  size_t m_req_counter = 0UL;

  // Context of the last successfully exported plan; "replanDeployment" resumes from it.
  // Only mutated on the service thread (mutually exclusive group) — no locking needed.
  bool m_has_active_plan = false;
  blaze::StaticVector<double, 4UL> m_q_goal_last;   // joint goal of the active plan, as PLANNED
                                                    // (the 2πk representative, not the raw IK output)
  // Endpoint check, written by checkPlanEndpoint() and read by the diag row.
  static constexpr double k_endpoint_warn_m = 3.00E-3; // the manager's acceptance gate
  blaze::StaticVector<double, 3UL> m_plan_endpoint_tip = blaze::StaticVector<double, 3UL>(std::numeric_limits<double>::quiet_NaN());
  double m_plan_endpoint_err = std::numeric_limits<double>::quiet_NaN();
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

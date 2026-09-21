#include "manager/master_node.hpp"

#include "ctr_common/csv_io.hpp"
#include "manager/csv_path_io.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "ctr_common/output_session.hpp"
#include "ctr_common/runtime_paths.hpp"

using namespace std::chrono_literals;

// ============================================================================
// Constructor
// ============================================================================

MasterNode::MasterNode(QWidget *parent)
    : QWidget(parent), rclcpp::Node("master_node"), m_gui_manager(std::make_unique<QtGuiManager>(this))
{
    std::cout << "Initializing MasterNode..." << std::endl;

    // Replanning tunables and target-list source (defaults preserve the old constexprs)
    m_force_replan_threshold = declare_parameter<double>("force_replan_threshold", m_force_replan_threshold);
    m_replan_cooldown_s = declare_parameter<double>("replan_cooldown_s", m_replan_cooldown_s);
    m_min_remaining_waypoints =
        static_cast<size_t>(declare_parameter<int>("min_remaining_waypoints", static_cast<int>(m_min_remaining_waypoints)));
    m_targets_csv = declare_parameter<std::string>("targets_csv", m_targets_csv);
    m_planner_timeout_s = declare_parameter<double>("planner_timeout_s", m_planner_timeout_s);
    m_plan_retry_cooldown_s = declare_parameter<double>("plan_retry_cooldown_s", m_plan_retry_cooldown_s);

    initDiagCsv();
    m_gui_manager->initializeGui();
    initRosInterfaces();
    onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::Planner));
    // m_high_level_mode = HighLvlCtrMode::Planner;
}

// ============================================================================
// Structured diagnostics (manager_diag.csv)
// ============================================================================
//
// One wide header shared by all event types; fields an event does not carry
// stay empty. Correlate with the planner's planner_diag.csv by wall_time.
void MasterNode::initDiagCsv()
{
    const auto diag_dir = ctr_common::makeSessionDir(
        ctr_common::resolveDataRoot(*this, "manager") / "Output_Files" / "diagnostics", "manager");
    m_diag.configure(diag_dir / "manager_diag.csv",
                     "event,wall_time,xd_x,xd_y,xd_z,target_azimuth,target_theta,"
                     "alpha1,alpha2,gate_diff1,gate_diff2,cmd_alpha,"
                     "success,ik_error,message,"
                     "tip_x,tip_y,tip_z,sim_x,sim_y,sim_z,probe_tip_err,"
                     "waypoints_in,waypoints_out,max_step_alpha");
    RCLCPP_INFO(get_logger(), "Manager diagnostics CSV: %s", m_diag.path().c_str());
}

void MasterNode::diagPreRotate(const Eigen::Vector3d &Xd, const manager_gate::PreRotation &pr)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "pre_rotate," << ctr_common::currentTimestamp() << ','
       << Xd[0] << ',' << Xd[1] << ',' << Xd[2] << ',' << std::atan2(Xd[1], Xd[0]) << ',' << pr.target_theta << ','
       << m_q[0] << ',' << m_q[2] << ',' << pr.gate_diff_1 << ',' << pr.gate_diff_2 << ',' << pr.cmd_alpha
       << ",,,,,,,,,,,,,";
    m_diag.append(os.str());
}

void MasterNode::diagPlanRequest(const Eigen::Vector3d &Xd, const char *mode)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "plan_request," << ctr_common::currentTimestamp() << ','
       << Xd[0] << ',' << Xd[1] << ',' << Xd[2] << ',' << std::atan2(Xd[1], Xd[0]) << ",,"
       << m_q[0] << ',' << m_q[2] << ",,,,,," << mode << ",,,,,,,,,,";
    m_diag.append(os.str());
}

void MasterNode::diagPlanResponse(const bool success, const double ik_error, const std::string &message)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "plan_response," << ctr_common::currentTimestamp() << ",,,,,,,,,,,"
       << (success ? 1 : 0) << ',' << ik_error << ',' << '"' << message << '"' << ",,,,,,,,,,";
    m_diag.append(os.str());
}

void MasterNode::diagPathLoaded(const size_t waypoints_in, const size_t waypoints_out, const double max_step_alpha)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "path_loaded," << ctr_common::currentTimestamp() << ",,,,,,,,,,,,,,,,,,,,,"
       << waypoints_in << ',' << waypoints_out << ',' << max_step_alpha;
    m_diag.append(os.str());
}

// The model-vs-registration discriminator, written when a deployment reaches
// its final waypoint: if the PINN tip (sim) agrees with the EM tip but both
// miss Xd, the target/registration is wrong; if sim and EM disagree, the model
// (or its joint feedback) is wrong in this region.
void MasterNode::diagDeployComplete(const Eigen::Vector3d &Xd, const Eigen::Vector3d &tip, const Eigen::Vector3d &sim)
{
    const double err = (tip - Xd).norm();
    RCLCPP_INFO(get_logger(),
                "Deployment complete: target = [%.4f, %.4f, %.4f], EM tip = [%.4f, %.4f, %.4f], PINN tip = [%.4f, %.4f, %.4f], "
                "|tip - target| = %.4f m, |tip - PINN| = %.4f m",
                Xd[0], Xd[1], Xd[2], tip[0], tip[1], tip[2], sim[0], sim[1], sim[2], err, (tip - sim).norm());
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "deploy_complete," << ctr_common::currentTimestamp() << ','
       << Xd[0] << ',' << Xd[1] << ',' << Xd[2] << ',' << std::atan2(Xd[1], Xd[0]) << ",,,,,,,,,,"
       << tip[0] << ',' << tip[1] << ',' << tip[2] << ',' << sim[0] << ',' << sim[1] << ',' << sim[2] << ','
       << err << ",,,";
    m_diag.append(os.str());
}

// Written when Auto Retract finishes, so a retraction leaves the same kind of
// trace an insertion does. Before this, the whole open-loop retract path logged
// at DEBUG and recorded nothing -- a retraction that stopped partway and one
// that never started were indistinguishable afterwards.
void MasterNode::diagRetractComplete(const bool reached_home, const size_t home_tail_steps,
                                     const Eigen::Vector3d &tip, const Eigen::Vector3d &sim)
{
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6)
        << "retract " << (reached_home ? "reached home" : "stopped at plan start")
        << "; b1=" << m_q[1] << " b2=" << m_q[3] << " a1=" << m_q[0] << " a2=" << m_q[2];

    RCLCPP_INFO(get_logger(),
                "Retraction complete: %s. q = [b1 %.4f, b2 %.4f, a1 %.4f, a2 %.4f], "
                "home = [b1 %.4f, b2 %.4f], home-leg steps = %zu",
                reached_home ? "at home" : "at the plan's start pose (home leg unavailable)",
                m_q[1], m_q[3], m_q[0], m_q[2],
                ctr_common::homePoseCommanded()[1], ctr_common::homePoseCommanded()[3],
                home_tail_steps);

    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "retract_complete," << ctr_common::currentTimestamp() << ",,,,,,"
       << m_q[0] << ',' << m_q[2] << ",,,,"
       << (reached_home ? 1 : 0) << ",," << '"' << msg.str() << '"' << ','
       << tip[0] << ',' << tip[1] << ',' << tip[2] << ',' << sim[0] << ',' << sim[1] << ',' << sim[2]
       << ",,," << home_tail_steps << ',';
    m_diag.append(os.str());
}

// Caller holds m_deploy_mutex.
void MasterNode::beginHomeTail(const blaze::StaticVector<double, 6> &from)
{
    m_q_list_home_tail = manager_csv::buildHomeLeg(from, m_insertion_step);
    m_home_tail_index = 0;
    m_home_tail_active = !m_q_list_home_tail.empty();

    const auto home = ctr_common::homePoseCommanded();
    RCLCPP_INFO(get_logger(),
                "Retract reached the plan's start pose [b1 %.4f, b2 %.4f, a1 %.4f, a2 %.4f]; "
                "continuing to home [b1 %.4f, b2 %.4f, a1 %.4f, a2 %.4f] over %zu steps",
                from[0], from[1], from[3], from[4],
                home[1], home[3], home[0], home[2], m_q_list_home_tail.size());

    // The feasible joint set is convex and both endpoints are inside it, so
    // every interpolated waypoint should be legal. Verify rather than assume:
    // if this ever fires, the plan's start pose was already out of the box and
    // the drives will clip the leg.
    size_t infeasible = 0;
    for (const auto &q : m_q_list_home_tail)
    {
        if (!ctr_kinematics_pinn::isFeasible4({q[0], q[1], q[3], q[4]}, k_joint_limits, 1.0e-6))
        {
            ++infeasible;
        }
    }
    if (infeasible > 0)
    {
        RCLCPP_WARN(get_logger(),
                    "%zu of %zu home-leg waypoints are outside the feasible joint set - "
                    "the drives will clip them and the retraction may stop short",
                    infeasible, m_q_list_home_tail.size());
    }
}

// Caller holds m_deploy_mutex.
void MasterNode::reportReachStall(const bool retracting, const int index, const size_t total)
{
    const double now_s = this->now().seconds();
    const double last_s = m_last_reach_progress_s.load();
    if (last_s <= 0.0)
    {
        m_last_reach_progress_s.store(now_s);
        return;
    }
    const double waiting_s = now_s - last_s;
    if (waiting_s < k_reach_stall_warn_s)
    {
        return;
    }

    // Nothing in this system reports "the drive stopped short": CTRobot issues
    // the target and the drive clips it against its own POSITION_LIMIT with no
    // feedback, so `reached` is the only signal and its absence was silent.
    if (!m_last_commanded_valid)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Auto-%s waiting %.1f s for 'reached' at waypoint %d/%zu; "
                             "no command has been sent yet (joints reached: %d %d %d %d)",
                             retracting ? "retract" : "insert", waiting_s, index + 1, total,
                             static_cast<int>(m_reachedJoints[0]), static_cast<int>(m_reachedJoints[1]),
                             static_cast<int>(m_reachedJoints[2]), static_cast<int>(m_reachedJoints[3]));
        return;
    }

    // m_q is wire order [a1, b1, a2, b2]; the waypoint is physics order.
    const double err_b1 = m_last_commanded_q[0] - m_q[1];
    const double err_b2 = m_last_commanded_q[1] - m_q[3];
    const double err_a1 = m_last_commanded_q[3] - m_q[0];
    const double err_a2 = m_last_commanded_q[4] - m_q[2];
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Auto-%s STALLED: waiting %.1f s for 'reached' at waypoint %d/%zu. "
                         "commanded [b1 %.4f, b2 %.4f, a1 %.4f, a2 %.4f] vs measured "
                         "[b1 %.4f, b2 %.4f, a1 %.4f, a2 %.4f]; error [%.4f, %.4f, %.4f, %.4f]; "
                         "per-joint reached [a1 %d, b1 %d, a2 %d, b2 %d]. A non-zero error on a "
                         "joint that never reports reached means the drive clipped the target at "
                         "its POSITION_LIMIT and stopped short.",
                         retracting ? "retract" : "insert", waiting_s, index + 1, total,
                         m_last_commanded_q[0], m_last_commanded_q[1], m_last_commanded_q[3], m_last_commanded_q[4],
                         m_q[1], m_q[3], m_q[0], m_q[2],
                         err_b1, err_b2, err_a1, err_a2,
                         static_cast<int>(m_reachedJoints[0]), static_cast<int>(m_reachedJoints[1]),
                         static_cast<int>(m_reachedJoints[2]), static_cast<int>(m_reachedJoints[3]));
}

// Caller holds m_deploy_mutex. One place to publish a waypoint, so the stall
// watchdog always knows what was last commanded and every commanded step is
// visible at INFO. The open-loop path used to log this at DEBUG only, which is
// why a retraction that stopped partway left no trace at all.
void MasterNode::sendDeploymentWaypoint(const blaze::StaticVector<double, 6> &q, const char *phase,
                                        const int index_1based, const size_t total)
{
    publish_position(q);
    m_last_commanded_q = q;
    m_last_commanded_valid = true;

    RCLCPP_INFO(this->get_logger(),
                "Sent q [%d/%zu] (%s): b1=%.4f, b2=%.4f, a1=%.4f, a2=%.4f "
                "(live: b1=%.4f, b2=%.4f, a1=%.4f, a2=%.4f)",
                index_1based, total, phase, q[0], q[1], q[3], q[4],
                m_q[1], m_q[3], m_q[0], m_q[2]);
}

// State the endpoint out loud when the operator arms Auto Retract. "Retract"
// reverses the loaded plan and then walks a home leg; how far back the plan
// itself reaches depends on where it was made, so print both distances rather
// than leaving the operator to infer them from the tubes.
void MasterNode::logRetractPlan()
{
    std::lock_guard<std::mutex> lock(m_deploy_mutex);
    const auto home = ctr_common::homePoseCommanded();
    if (m_q_list_adjusted.empty())
    {
        RCLCPP_WARN(get_logger(), "Auto Retract armed with no waypoint list - nothing to reverse");
        return;
    }
    const auto &row0 = m_q_list_adjusted.front();
    // m_q is wire order [a1, b1, a2, b2]; row0 is physics order.
    RCLCPP_INFO(get_logger(),
                "Auto Retract armed at waypoint %d/%zu. Plan start = [b1 %.4f, b2 %.4f]; "
                "live = [b1 %.4f, b2 %.4f]; home = [b1 %.4f, b2 %.4f]. "
                "|live - plan start| = [%.4f, %.4f] m, |plan start - home| = [%.4f, %.4f] m "
                "(the second pair is the extra home leg).",
                m_current_config_index + 1, m_q_list_adjusted.size(),
                row0[0], row0[1], m_q[1], m_q[3], home[1], home[3],
                std::fabs(m_q[1] - row0[0]), std::fabs(m_q[3] - row0[1]),
                std::fabs(row0[0] - home[1]), std::fabs(row0[1] - home[3]));
}

// Closed-loop deployment consumes plannedPath.csv and removes it once the
// stack is fully unwound, so a stale plan is never picked up by the next run.
void MasterNode::deletePlannedPathFile()
{
    const std::filesystem::path file_path =
        ctr_common::resolveDataRoot(*this, "manager") / "Shared_Files" / "plannedPath.csv";
    std::error_code ec;
    if (std::filesystem::exists(file_path, ec) && std::filesystem::remove(file_path, ec))
    {
        RCLCPP_INFO(this->get_logger(), "Deleted plannedPath.csv");
    }
}

// Caller holds m_deploy_mutex.
void MasterNode::finishRetraction(const bool reached_home, const size_t home_tail_steps,
                                  const Eigen::Vector3d &tip, const Eigen::Vector3d &sim)
{
    m_auto_retract = false;
    m_current_config_index = 0;
    m_q_list_adjusted.clear();
    m_q_list_home_tail.clear();
    m_home_tail_index = 0;
    m_home_tail_active = false;
    m_last_commanded_valid = false;
    m_last_reach_progress_s.store(0.0);
    invalidateForceBaseline();
    diagRetractComplete(reached_home, home_tail_steps, tip, sim);
}

// ============================================================================
// Public Methods for GUI Callbacks
// ============================================================================

void MasterNode::sendConfigCommand(const std::string &command, bool use_enable_service)
{
    if (!m_services_ready)
    {
        RCLCPP_WARN(get_logger(), "Ignoring '%s': robot services are not ready yet", command.c_str());
        return;
    }
    auto request = std::make_shared<interfaces::srv::Config::Request>();
    request->command = command;

    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
    auto client = use_enable_service ? m_robot_enable_client : m_robot_config_client;
    auto response_received_callback = std::bind(&MasterNode::handle_service_response, this, std::placeholders::_1);
    (void)client->async_send_request(request, response_received_callback);
}

void MasterNode::handleFreezeButtonClicked(QPushButton *freeze_button)
{
    if (!m_services_ready)
    {
        RCLCPP_WARN(get_logger(), "Ignoring freeze toggle: robot services are not ready yet");
        return;
    }
    m_robot_frozen = !m_robot_frozen;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = m_robot_frozen;

    auto result_callback = [this, freeze_button](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            // This callback runs on a ROS executor thread — widget mutations
            // must be marshalled onto the Qt thread.
            const bool frozen = m_robot_frozen;
            QMetaObject::invokeMethod(this, [freeze_button, frozen]()
                                      {
                freeze_button->setText(frozen ? "Unfreeze Robot" : "Freeze Robot");
                // Red background when unfrozen (showing "Freeze Robot"), default when frozen
                freeze_button->setStyleSheet(frozen ? "" : "background-color: rgb(255, 0, 0); color: white;"); },
                                      Qt::QueuedConnection);
            RCLCPP_INFO(this->get_logger(), "Robot %s", frozen ? "frozen" : "unfrozen");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to %s robot: %s",
                         m_robot_frozen ? "freeze" : "unfreeze", response->message.c_str());
        }
    };

    m_freeze_robot_client->async_send_request(request, result_callback);
}

void MasterNode::handleAutoInsertClicked()
{
    if (m_high_level_mode == HighLvlCtrMode::Deployment)
    {
        m_auto_insert = !m_auto_insert;
        if (m_auto_insert)
        {
            m_auto_retract = false; // Disable auto-retract when auto-insert is enabled
            // Re-seed the stall watchdog, or the idle time since the last step
            // reads as a stall on the very first cycle.
            m_last_reach_progress_s.store(0.0);
        }
        RCLCPP_INFO(this->get_logger(), "Auto-insert mode: %s", m_auto_insert ? "ON" : "OFF");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Auto-insert only available in Deployment mode");
    }
}

void MasterNode::handleAutoRetractClicked()
{
    if (m_high_level_mode == HighLvlCtrMode::Deployment)
    {
        m_auto_retract = !m_auto_retract;
        if (m_auto_retract)
        {
            m_auto_insert = false; // Disable auto-insert when auto-retract is enabled
            m_last_reach_progress_s.store(0.0); // re-seed the stall watchdog
            logRetractPlan();
        }
        RCLCPP_INFO(this->get_logger(), "Auto-retract mode: %s", m_auto_retract ? "ON" : "OFF");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Auto-retract only available in Deployment mode");
    }
}

void MasterNode::handleTestButtonClicked()
{
    if (!m_test_running)
    {
        if (m_procedure == false)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot start test: Procedure mode is not active.");
            return;
        }
        if (m_enabled == false)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot start test: Robot is not enabled.");
            return;
        }
        // Start the automated test
        RCLCPP_INFO(this->get_logger(), "=== Starting Automated Test ===");

        // Load targets from random_interior_points.csv
        read_targets_from_csv(m_test_targets, m_targets_csv);

        if (m_test_targets.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load targets from %s", m_targets_csv.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu test targets", m_test_targets.size());

        // Initialize recording sessions A and B
        auto request_a = std::make_shared<interfaces::srv::Recording::Request>();
        request_a->command = "init";
        request_a->name = "data";
        m_recording_client->async_send_request(request_a);

        auto request_b = std::make_shared<interfaces::srv::Recording::Request>();
        request_b->command = "init";
        request_b->name = "deployed";
        m_recording_client->async_send_request(request_b);

        RCLCPP_INFO(this->get_logger(), "Recording sessions A and B initialized");

        // Initialize test state
        m_test_running = true;
        onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::None)); 
        m_test_state = TestState::SelectingTarget;
        m_current_target_index = 0;
        m_test_state_entry_time = this->now();
        m_target_set = false;

        RCLCPP_INFO(this->get_logger(), "Test initialized. Processing target 1/%zu", m_test_targets.size());
    }
    else
    {
        // Stop the test
        RCLCPP_INFO(this->get_logger(), "=== Stopping Automated Test ===");

        // Close recording sessions
        auto request_a = std::make_shared<interfaces::srv::Recording::Request>();
        request_a->command = "close";
        request_a->name = "data";
        m_recording_client->async_send_request(request_a);

        auto request_b = std::make_shared<interfaces::srv::Recording::Request>();
        request_b->command = "close";
        request_b->name = "deployed";
        m_recording_client->async_send_request(request_b);

        RCLCPP_INFO(this->get_logger(), "Recording sessions A and B closed");

        m_test_running = false;
        m_test_state = TestState::Idle;
        m_auto_insert = false;
        m_auto_retract = false;
    }
}

void MasterNode::handleToggleTargetModeClicked()
{
    m_use_csv_target = !m_use_csv_target;
    
    if (m_use_csv_target)
    {
        // Switching to CSV mode
        RCLCPP_INFO(this->get_logger(), "=== Switching to CSV Target Mode ===");
        
        // Load targets from random_interior_points.csv
        read_targets_from_csv(m_csv_targets, m_targets_csv);
        
        if (m_csv_targets.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load targets from %s", m_targets_csv.c_str());
            m_use_csv_target = false;
            m_gui_manager->getToggleTargetModeButton()->setText("Toggle: Probe Mode");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu CSV targets", m_csv_targets.size());
        
        // Start with first target
        m_csv_target_index = 0;
        {
            // Qt thread; control_loop reads m_Xd under this mutex on an executor thread.
            std::lock_guard<std::mutex> lock(m_feedback_mutex);
            m_Xd = m_csv_targets[m_csv_target_index];
        }
        
        // Publish target to task_space/target
        auto target_msg = interfaces::msg::Taskspace();
        target_msg.p[0] = m_Xd[0];
        target_msg.p[1] = m_Xd[1];
        target_msg.p[2] = m_Xd[2];
        m_pub_task_target->publish(target_msg);
        
        // Enable Next/Previous buttons and update their states
        m_gui_manager->getNextTargetButton()->setEnabled(m_csv_targets.size() > 1);
        m_gui_manager->getPrevTargetButton()->setEnabled(false);  // At first target
        m_gui_manager->getToggleTargetModeButton()->setText("Toggle: CSV Mode");
        
        RCLCPP_INFO(this->get_logger(), "CSV Target 1/%zu: [%.3f, %.3f, %.3f]",
                    m_csv_targets.size(), m_Xd[0], m_Xd[1], m_Xd[2]);
        
        updateCsvTargetDisplay();
    }
    else
    {
        // Switching back to Probe mode
        RCLCPP_INFO(this->get_logger(), "=== Switching to Probe Target Mode ===");
        
        // Disable Next/Previous buttons
        m_gui_manager->getNextTargetButton()->setEnabled(false);
        m_gui_manager->getPrevTargetButton()->setEnabled(false);
        m_gui_manager->getToggleTargetModeButton()->setText("Toggle: Probe Mode");
    }
}

void MasterNode::handleNextTargetClicked()
{
    if (!m_use_csv_target || m_csv_targets.empty())
    {
        return;
    }
    
    if (m_csv_target_index < m_csv_targets.size() - 1)
    {
        m_csv_target_index++;
        {
            // Qt thread; control_loop reads m_Xd under this mutex on an executor thread.
            std::lock_guard<std::mutex> lock(m_feedback_mutex);
            m_Xd = m_csv_targets[m_csv_target_index];
        }
        
        // Publish target to task_space/target
        auto target_msg = interfaces::msg::Taskspace();
        target_msg.p[0] = m_Xd[0];
        target_msg.p[1] = m_Xd[1];
        target_msg.p[2] = m_Xd[2];
        m_pub_task_target->publish(target_msg);
        
        RCLCPP_INFO(this->get_logger(), "CSV Target %zu/%zu: [%.3f, %.3f, %.3f]",
                    m_csv_target_index + 1, m_csv_targets.size(), m_Xd[0], m_Xd[1], m_Xd[2]);
        
        // Update button states
        m_gui_manager->getPrevTargetButton()->setEnabled(true);
        m_gui_manager->getNextTargetButton()->setEnabled(m_csv_target_index < m_csv_targets.size() - 1);
        
        updateCsvTargetDisplay();
    }
}

void MasterNode::handlePrevTargetClicked()
{
    if (!m_use_csv_target || m_csv_targets.empty())
    {
        return;
    }
    
    if (m_csv_target_index > 0)
    {
        m_csv_target_index--;
        {
            // Qt thread; control_loop reads m_Xd under this mutex on an executor thread.
            std::lock_guard<std::mutex> lock(m_feedback_mutex);
            m_Xd = m_csv_targets[m_csv_target_index];
        }
        
        // Publish target to task_space/target
        auto target_msg = interfaces::msg::Taskspace();
        target_msg.p[0] = m_Xd[0];
        target_msg.p[1] = m_Xd[1];
        target_msg.p[2] = m_Xd[2];
        m_pub_task_target->publish(target_msg);
        
        RCLCPP_INFO(this->get_logger(), "CSV Target %zu/%zu: [%.3f, %.3f, %.3f]",
                    m_csv_target_index + 1, m_csv_targets.size(), m_Xd[0], m_Xd[1], m_Xd[2]);
        
        // Update button states
        m_gui_manager->getPrevTargetButton()->setEnabled(m_csv_target_index > 0);
        m_gui_manager->getNextTargetButton()->setEnabled(true);
        
        updateCsvTargetDisplay();
    }
}

void MasterNode::updateCsvTargetDisplay()
{
    if (!m_use_csv_target)
    {
        return;
    }
    
    // Calculate error between current tip and CSV target
    Eigen::Vector3d tip, Xd;
    {
        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        tip = m_tip_position;
        Xd = m_Xd;
    }
    Eigen::Vector3d error = tip - Xd;
    
    // Update row 5 for CSV target
    QTableWidget* emt_table = m_gui_manager->getEmtStatusTable();
    emt_table->setItem(5, 0, new QTableWidgetItem(QString::number(Xd[0], 'f', 3)));
    emt_table->setItem(5, 1, new QTableWidgetItem(QString::number(Xd[1], 'f', 3)));
    emt_table->setItem(5, 2, new QTableWidgetItem(QString::number(Xd[2], 'f', 3)));
    emt_table->setItem(5, 3, new QTableWidgetItem(QString::number(Xd.norm(), 'f', 3)));
    emt_table->setItem(5, 4, new QTableWidgetItem(QString::number(atan2(Xd[1], Xd[0]), 'f', 3)));
    
    // Update row 6 for CSV-Tip error
    emt_table->setItem(6, 0, new QTableWidgetItem(QString::number(error[0], 'f', 3)));
    emt_table->setItem(6, 1, new QTableWidgetItem(QString::number(error[1], 'f', 3)));
    emt_table->setItem(6, 2, new QTableWidgetItem(QString::number(error[2], 'f', 3)));
    emt_table->setItem(6, 3, new QTableWidgetItem(QString::number(error.norm(), 'f', 3)));
}

// ============================================================================
// Public Slots
// ============================================================================

void MasterNode::onClosedLoopToggled(bool checked)
{
    m_closed_loop_enabled = checked;
    if (m_closed_loop_enabled)
    {
        std::lock_guard<std::mutex> lock(m_deploy_mutex); // slot runs on the Qt thread
        m_q_list_actuated.clear();
        m_q_list_actuated.push_back(blaze::StaticVector<double, 6>({m_q[1], m_q[3], 0.0, m_q[0], m_q[2], 0.0}));
    }
    RCLCPP_INFO(this->get_logger(), "Closed-loop deployment: %s", checked ? "ENABLED" : "DISABLED");
}

void MasterNode::onCtrlModeClicked(int id)
{
    m_high_level_mode = static_cast<HighLvlCtrMode>(id);

    // A mode change disarms both auto modes, in BOTH branches. The Deployment
    // branch used to leave m_auto_retract set while Planner cleared it, so a
    // stray press of the handheld trigger (which toggles the mode on a rising
    // edge in manualInterface_callback) aborted an in-progress retraction with
    // nothing in the log but "High-level control mode set to ...".
    const bool was_retracting = m_auto_retract.exchange(false);
    const bool was_inserting = m_auto_insert.exchange(false);
    m_last_reach_progress_s.store(0.0);
    if (was_retracting || was_inserting)
    {
        RCLCPP_WARN(this->get_logger(), "Mode change aborted an active auto-%s at waypoint %d",
                    was_retracting ? "retract" : "insert", m_current_config_index);
    }

    std::string mode_name;
    switch (static_cast<HighLvlCtrMode>(id))
    {
    case HighLvlCtrMode::Planner:
    {
        mode_name = "Planner";
        m_retracting = false;
        // m_gui_manager->getClosedLoopCheckbox()->setEnabled(false);
        break;
    }
    case HighLvlCtrMode::Deployment:
    {
        // read_path_from_csv(m_q_list, "plannedPath.csv");
        // m_current_config_index = 0;
        // m_q_list_adjusted = adjustConfigurationListStepSize(m_q_list, m_insertion_step);
        mode_name = m_closed_loop_enabled ? "Deployment (Closed-Loop)" : "Deployment";
        m_retracting = false;
        // m_gui_manager->getClosedLoopCheckbox()->setEnabled(true);
        break;
    }
    default:
    {
        mode_name = "Unknown";
        break;
    }
    }
    RCLCPP_INFO(this->get_logger(), "High-level control mode set to %s", mode_name.c_str());
}

// ============================================================================
// ROS2 Interface and Callbacks
// ============================================================================

void MasterNode::initRosInterfaces()
{
    //------ robot node interfaces ------//
    m_subscription_joints = create_subscription<interfaces::msg::Jointspace>(
        "joint_space/feedback", 10, std::bind(&MasterNode::jointsConfig_timerCallback, this, std::placeholders::_1));
    m_subscription_status = create_subscription<interfaces::msg::Status>(
        "robot_status", 10, std::bind(&MasterNode::robotStatus_callback, this, std::placeholders::_1));
    m_subscription_interface = create_subscription<interfaces::msg::Interface>(
        "manual_interface", 10, std::bind(&MasterNode::manualInterface_callback, this, std::placeholders::_1));
    m_subscription_sim_out = create_subscription<interfaces::msg::Taskspace>(
        "task_space/sim_out", 10, std::bind(&MasterNode::updateSimout, this, std::placeholders::_1));
    m_sub_force = create_subscription<interfaces::msg::Force>(
        "task_space/force_estimate", 10, std::bind(&MasterNode::updateForceEstimate, this, std::placeholders::_1));

    m_publisher_manual_vel = create_publisher<interfaces::msg::Jointspace>("joint_space/manual_vel", 10);
    m_pub_joint_targ = create_publisher<interfaces::msg::Jointspace>("joint_space/target", 10);
    m_pub_task_target = create_publisher<interfaces::msg::Taskspace>("task_space/target", 10);

    m_robot_config_client = create_client<interfaces::srv::Config>("robot_config");
    m_robot_enable_client = create_client<interfaces::srv::Config>("robot_enable");
    m_planner_client = create_client<interfaces::srv::Planner>("planner/command");
    m_freeze_robot_client = create_client<std_srvs::srv::SetBool>("freeze_robot");
    m_recording_client = create_client<interfaces::srv::Recording>("recording");

    // Deferred readiness: the old ctor-blocking wait_for_service loops made
    // the GUI unstartable until the whole stack was up (hence launch-file
    // delay gymnastics). A 500 ms timer polls instead; service users are
    // gated on m_services_ready / service_is_ready().
    m_readiness_timer = create_wall_timer(500ms, [this]()
                                          {
        const bool ready = m_robot_config_client->service_is_ready() &&
                           m_robot_enable_client->service_is_ready() &&
                           m_planner_client->service_is_ready() &&
                           m_freeze_robot_client->service_is_ready() &&
                           m_recording_client->service_is_ready();
        if (ready && !m_services_ready.exchange(true))
            RCLCPP_INFO(get_logger(), "All robot/planner/recorder services are ready");
        else if (!ready && m_services_ready.exchange(false))
            RCLCPP_WARN(get_logger(), "A required service went away - controls gated until it returns"); });

    //------ EMtracker node interfaces ------//
    m_callback_group_tf2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf2_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    m_tf2_timer = this->create_wall_timer(10ms, std::bind(&MasterNode::tf2_receive_timer_callback, this), m_callback_group_tf2);

    // Create control timer
    double m_sample_time = 0.100;
    m_callback_group_pub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto control_sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_control_timer = this->create_wall_timer(control_sample_time, std::bind(&MasterNode::control_loop, this), m_callback_group_pub1);
}

void MasterNode::jointsConfig_timerCallback(const interfaces::msg::Jointspace::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_feedback_mutex);
    for (int i = 0; i < 4; i++)
    {
        m_q[i] = msg->position[i];
        m_qdot[i] = msg->velocity[i];
        m_current[i] = msg->current[i];
    }
}

void MasterNode::robotStatus_callback(const interfaces::msg::Status::SharedPtr msg)
{
    m_enabled = msg->enable[0] * msg->enable[1] * msg->enable[2] * msg->enable[3];
    m_reached = msg->reached[0] * msg->reached[1] * msg->reached[2] * msg->reached[3];
    m_encoder = msg->encoder[0] * msg->encoder[1] * msg->encoder[2] * msg->encoder[3];
    m_procedure = msg->procedure;
    m_ready_to_engage = msg->ready_to_engage;
    m_engaged = msg->engaged;
    m_locked = msg->locked;
    m_head_attached = msg->head_attached;
    m_trans_limit = msg->trans_limit_en;
    m_ctrl_mode = static_cast<CtrlMode>(msg->control_mode);

    for (int i = 0; i < 4; i++)
    {
        m_minCurrentPosLimit[i] = msg->min_pos_limit[i];
        m_maxCurrentPosLimit[i] = msg->max_pos_limit[i];
        m_enabledJoints[i] = msg->enable[i];
        m_encoderJoints[i] = msg->encoder[i];
        m_reachedJoints[i] = msg->reached[i];

        // A joint that never reached Operation Enabled will not track the
        // deployment waypoints; say so rather than letting the loop run on.
        if (msg->enable_fault[i] && !m_enableFaultPrev[i])
            RCLCPP_ERROR(get_logger(),
                         "Joint %d reports a drive enable fault - it is NOT under control", i);
        m_enableFaultPrev[i] = msg->enable_fault[i];
    }

    emit robotStatusUpdated(m_enabled, m_procedure, m_head_attached, m_engaged, m_locked);
    emit update_enable_button_Text(m_enabled ? "Disable" : "Enable");
}

void MasterNode::manualInterface_callback(const interfaces::msg::Interface::SharedPtr msg)
{
    for (int i = 0; i < 7; i++)
    {
        m_interface_key_prev[i] = m_interface_key[i].load();
        m_interface_key[i] = msg->interface_key[i];
    }

    // Rising edge detection on trigger button
    size_t trigger_idx = 6;
    if (!m_interface_key_prev[trigger_idx] && m_interface_key[trigger_idx])
    {
        if (m_high_level_mode == HighLvlCtrMode::Planner)
        {
            onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::Deployment));
        }
        else if (m_high_level_mode == HighLvlCtrMode::Deployment)
        {
            onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::Planner));
        }
    }
}

void MasterNode::updateSimout(const interfaces::msg::Taskspace::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_feedback_mutex);
    for (int i = 0; i < 3; i++)
    {
        m_Xsim[i] = msg->p[i];
    }
}

void MasterNode::updateForceEstimate(const interfaces::msg::Force::SharedPtr msg)
{
    // Reject and hold the last good value. A non-finite force here does real
    // damage rather than just displaying wrong: the drift test below is
    // `df <= threshold`, which is FALSE for NaN, so every cycle would look like
    // a large drift and fire a replan until the attempt limit permanently
    // suppressed replanning.
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->z))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Non-finite force estimate [%.3f, %.3f, %.3f] N - holding the last good value",
                             msg->x, msg->y, msg->z);
        return;
    }

    std::lock_guard<std::mutex> lock(m_force_mutex);
    m_f_est = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void MasterNode::tf2_receive_timer_callback()
{
    // The two frames are looked up independently. A missing `probe` (no probe sensor
    // tracked - see EMtracker_node's get_probe_transform_in_em guard) used to return
    // early and also skip the tip update below, freezing the whole Cartesian readout and
    // breaking CSV-target mode, which needs no probe at all.
    const std::string targetFrame = "robot_base";

    const auto lookup = [this, &targetFrame](const char *sourceFrame, Eigen::Matrix4d &out,
                                             std::string &error)
    {
        try
        {
            out = tf2::transformToEigen(
                      m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero))
                      .matrix();
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            error = ex.what();
            return false;
        }
    };

    std::string tip_error, probe_error;
    const bool tip_valid = lookup("ctr_tip", m_trans_tip, tip_error);
    const bool probe_valid = lookup("probe", m_trans_probe, probe_error);

    // Logged from two distinct call sites so each frame gets its own throttle window -
    // sharing one would let a ctr_tip failure mask a probe failure entirely. This is a
    // 10 ms timer, so at INFO these flooded ~100 lines/s and buried everything else.
    if (!tip_valid)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Could not transform robot_base to ctr_tip: %s", tip_error.c_str());
    }
    if (!probe_valid)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Could not transform robot_base to probe: %s - probe targeting "
                             "is unavailable; CSV target mode still works", probe_error.c_str());
    }

    // Update positions
    if (probe_valid && !m_test_running && !m_use_csv_target)
    {
        Eigen::Vector3d Xd;
        {
            std::lock_guard<std::mutex> lock(m_feedback_mutex);
            m_Xd = m_trans_probe.block<3, 1>(0, 3); // target tip position
            Xd = m_Xd;
        }

        // Publish target position
        auto target_msg = interfaces::msg::Taskspace();
        target_msg.p[0] = Xd[0];
        target_msg.p[1] = Xd[1];
        target_msg.p[2] = Xd[2];
        m_pub_task_target->publish(target_msg);
    }

    if (tip_valid)
    {
        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        m_X = m_trans_tip.block<3, 1>(0, 3); // Current tip position
        m_tip_position = m_X;                // Store for CSV target error calculation
    }

    // Update CSV target display if in CSV mode (this timer runs on a ROS
    // executor thread — widget updates must run on the Qt thread)
    if (m_use_csv_target)
    {
        QMetaObject::invokeMethod(this, [this]() { updateCsvTargetDisplay(); }, Qt::QueuedConnection);
    }

    emit emtUpdated(m_trans_tip(0, 3), m_trans_tip(1, 3), m_trans_tip(2, 3),
                    m_trans_probe(0, 3), m_trans_probe(1, 3), m_trans_probe(2, 3),
                    m_Xsim(0), m_Xsim(1), m_Xsim(2));
}

void MasterNode::handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
{
    auto response = future.get();
    if (!response->success)
    {
        // Log errors if needed
    }
}

void MasterNode::handle_planner_response(const rclcpp::Client<interfaces::srv::Planner>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Planner response - Success: %s", response->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "                 - Message: %s", response->message.c_str());
    RCLCPP_INFO(this->get_logger(), "                 - IK error: %.4f", response->value);
    diagPlanResponse(response->success, response->value, response->message);

    m_planner_success = response->success;
    m_planner_ik_error = response->value;

    // In closed-loop mode, bypass error check before first insertion to ensure initial plan exists
    // bool bypass_error_check = m_closed_loop_enabled && m_q_list_actuated.empty();
    bool bypass_error_check = false;

    RCLCPP_INFO(this->get_logger(), "Planner bypass_error_check=%s", bypass_error_check ? "true" : "false");
    
    if (!(m_planner_success || bypass_error_check))
    {
        // The planner's own message names the gate that failed (start state, goal
        // state, IK, or no solution) - carry it through rather than dropping it.
        armPlanRetry(response->message);
    }
    else if (!(m_planner_ik_error < k_ik_error_threshold))
    {
        // Accept-polarity on purpose. Written as `>= threshold` this gate PASSED a
        // non-finite IK error, because every comparison against NaN is false: the
        // planner reported success, the GUI printed "nan" in plain black text, and
        // a no-op plan was loaded and deployed 57 mm from its target. Phrasing it
        // as "not provably good enough" rejects NaN as well as a large error.
        //
        // The scripted-test gate in updateTestStateMachine() is written the same
        // way for the same reason; the two used to disagree on NaN.
        armPlanRetry("IK error " + std::to_string(m_planner_ik_error.load()) + " m is not below the " +
                     std::to_string(k_ik_error_threshold) + " m limit");
        m_planner_success = false;
    }
    else if (loadPlannedPath())
    {
        m_plan_retry_armed = false;
        std::lock_guard<std::mutex> lock(m_force_mutex);
        m_f_at_plan = m_f_pending;
        m_f_at_plan_valid = true;
        m_replan_attempts = 0;
        m_replan_backoff_s = m_replan_cooldown_s;
    }
    else
    {
        armPlanRetry("could not load plannedPath.csv - path will be empty");
        m_planner_success = false;
    }

    std::this_thread::sleep_for(10ms);

    m_flag_planning = false;
    emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);
}

// Replan responses are handled separately from full-plan responses: `value`
// carries the FTL swept cost of the chosen schedule (not an IK error), so the
// k_ik_error_threshold gate must not apply, and a rejected replan must not
// overwrite the full-plan status consumed by the GUI and the test state machine.
void MasterNode::handle_replan_response(const rclcpp::Client<interfaces::srv::Planner>::SharedFuture future)
{
    auto response = future.get();

    if (response->success && loadPlannedPath(/*is_replan=*/true))
    {
        std::lock_guard<std::mutex> lock(m_force_mutex);
        m_f_at_plan = m_f_pending;
        m_f_at_plan_valid = true;
        m_replan_attempts = 0;
        m_replan_backoff_s = m_replan_cooldown_s;
        RCLCPP_INFO(this->get_logger(), "Replan accepted: %s (FTL cost %.6f) - resuming deployment on the new schedule",
                    response->message.c_str(), response->value);
    }
    else
    {
        // Old list, index, and force baseline are retained: deployment resumes
        // on the previous plan. Each rejection doubles the wait (capped); after
        // k_max_replan_attempts, maybeRequestDeploymentReplan() suppresses
        // further requests until the drift recovers (hysteresis re-arm).
        m_replan_attempts++;
        m_replan_backoff_s = std::min(m_replan_backoff_s * 2.0, k_replan_backoff_cap_s);
        RCLCPP_WARN(this->get_logger(),
                    "Replan rejected (%s) - continuing previous plan (attempt %d/%d, next in >= %.1f s)",
                    response->message.c_str(), m_replan_attempts, k_max_replan_attempts, m_replan_backoff_s);
    }

    m_flag_planning = false;
    emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);
}

// ============================================================================
// Control Functions
// ============================================================================

// Force-drift detector for open-loop deployment: when the EKF force estimate
// has drifted from the value the active plan was computed with, pause the
// deployment (m_flag_planning) and ask the planner to re-schedule the
// remaining prismatic deployment from the current configuration.
void MasterNode::invalidateForceBaseline()
{
    std::lock_guard<std::mutex> lock(m_force_mutex);
    m_f_at_plan_valid = false;
}

void MasterNode::maybeRequestDeploymentReplan()
{
    if (m_flag_planning || m_retracting)
        return;
    if (!(m_interface_key[k_forward_button_idx] || m_auto_insert))
        return; // only while actively inserting
    if (!getReachStatus())
        return; // same settle gate as the insertion step

    int index;
    size_t remaining;
    {
        std::lock_guard<std::mutex> lock(m_deploy_mutex);
        if (m_q_list_adjusted.empty() || m_current_config_index < 1)
            return; // deployment not started yet
        index = m_current_config_index;
        remaining = m_q_list_adjusted.size() - 1 - static_cast<size_t>(m_current_config_index);
    }
    if (remaining < m_min_remaining_waypoints)
        return;

    double df;
    {
        std::lock_guard<std::mutex> lock(m_force_mutex);
        if (!m_f_at_plan_valid)
        {
            // Invalidated by a full retraction; only a fresh accepted plan restores it.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "Force-drift replanning is inactive: no plan force baseline "
                                 "(replan a path to re-establish it)");
            return;
        }
        df = (m_f_est - m_f_at_plan).norm();

        // Hysteresis re-arm: once the drift recovers to half the trigger
        // threshold, a previous string of rejections is forgiven.
        if (m_replan_attempts >= k_max_replan_attempts && df <= 0.5 * m_force_replan_threshold)
        {
            m_replan_attempts = 0;
            m_replan_backoff_s = m_replan_cooldown_s;
            RCLCPP_INFO(this->get_logger(), "Force drift recovered (%.3f N) - replanning re-armed", df);
        }
    }
    // A non-finite drift is not evidence of drift. Guard explicitly, because
    // `df <= threshold` is false for NaN and would fall through to request a
    // replan on every cycle.
    if (!std::isfinite(df))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Force drift is not finite - not requesting a replan");
        return;
    }
    if (df <= m_force_replan_threshold)
        return;
    {
        std::lock_guard<std::mutex> lock(m_force_mutex);
        if (m_replan_attempts >= k_max_replan_attempts)
        {
            // Suppressed until hysteresis re-arm or an accepted plan. Silent until now,
            // which is the one replan state an operator most needs to see.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "Force-drift replanning suppressed after %d consecutive "
                                 "rejections (|df| = %.3f N); it re-arms when the drift falls "
                                 "below %.3f N or a plan is accepted",
                                 m_replan_attempts, df, 0.5 * m_force_replan_threshold);
            return;
        }
        if ((this->now() - m_last_replan_request_time).seconds() < m_replan_backoff_s)
            return;
        m_f_pending = m_f_est;
    }

    m_flag_planning = true; // pauses every deployment branch until the response arrives
    m_planner_request_time_s = this->now().seconds();
    m_last_replan_request_time = this->now();
    m_replan_count++;

    auto request = std::make_shared<interfaces::srv::Planner::Request>();
    request->command = "replanDeployment";

    auto response_callback = std::bind(&MasterNode::handle_replan_response, this, std::placeholders::_1);
    (void)m_planner_client->async_send_request(request, response_callback);

    emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);
    RCLCPP_INFO(this->get_logger(), "Force drift |df| = %.3f N at waypoint %d (%zu remaining) - pausing deployment to replan (#%d)",
                df, index, remaining, m_replan_count);

    if (m_test_running)
    {
        captureRecordingFrame("data"); // event marker; force columns are already recorded
    }
}

// Names the services the readiness timer is still waiting on, so a dead control loop
// points at the node that has not come up rather than just going quiet.
std::string MasterNode::missingServicesDescription() const
{
    std::string missing;
    const auto note = [&missing](const char *name)
    {
        if (!missing.empty())
            missing += ", ";
        missing += name;
    };

    if (!m_robot_config_client->service_is_ready())
        note("robot_config (ctr_robot)");
    if (!m_robot_enable_client->service_is_ready())
        note("robot_enable (ctr_robot)");
    if (!m_planner_client->service_is_ready())
        note("planner/command (planner)");
    if (!m_freeze_robot_client->service_is_ready())
        note("freeze_robot (emtracker)");
    if (!m_recording_client->service_is_ready())
        note("recording (record)");

    return missing.empty() ? std::string("none") : missing;
}

// A rejected plan must not look like a satisfied gate. control_loop latches m_Xd_prev at
// request time (that is what stops a 100 ms request storm while a solve is in flight), so
// without this the same target is never asked for again.
void MasterNode::armPlanRetry(const std::string &reason)
{
    RCLCPP_ERROR(get_logger(), "Plan rejected: %s - retrying the same target in %.1f s",
                 reason.c_str(), m_plan_retry_cooldown_s);
    m_plan_retry_after_s = this->now().seconds() + m_plan_retry_cooldown_s;
    m_plan_retry_armed = true;
}

bool MasterNode::planRetryDue() const
{
    return m_plan_retry_armed && this->now().seconds() >= m_plan_retry_after_s.load();
}

// Called from the open-loop Deployment branch whenever a cycle sends no waypoint.
void MasterNode::reportDeploymentGate()
{
    if (m_flag_planning)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Deployment idle: a plan or replan request is still outstanding");
        return;
    }

    size_t total = 0;
    int index = 0;
    {
        std::lock_guard<std::mutex> lock(m_deploy_mutex);
        total = m_q_list_adjusted.size();
        index = m_current_config_index;
    }

    if (total == 0)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Deployment idle: no waypoint list. Switch to the 'Select Target' "
                             "radio and let the planner run first - the Deployment branch never "
                             "requests a plan by itself.");
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Deployment idle: holding at waypoint %d/%zu. Hold an insert/retract "
                         "button on the robot, or click Auto Insert / Auto Retract.",
                         index + 1, total);
}

// Called from the Planner-mode branch whenever a cycle ends without a plan request.
// Reports the first gate that is closed, in the same order control_loop tests them.
void MasterNode::reportPlannerGate(double tube_1_theta_diff, double tube_2_theta_diff,
                                   bool target_changed, bool q_changed)
{
    constexpr auto kDeg = 180.0 / M_PI;

    if (tube_1_theta_diff > k_theta_threshold || tube_2_theta_diff > k_theta_threshold)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Planner idle: rotating tubes toward the target bearing "
                             "(off by %.1f deg and %.1f deg; both must be <= %.1f deg). "
                             "If they are not moving, check the drives are enabled.",
                             tube_1_theta_diff * kDeg, tube_2_theta_diff * kDeg,
                             k_theta_threshold * kDeg);
        return;
    }

    if (!m_reached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Planner idle: joints have not reached their targets "
                             "(reached = [%d %d %d %d], wire order [a1 b1 a2 b2])",
                             static_cast<int>(m_reachedJoints[0]), static_cast<int>(m_reachedJoints[1]),
                             static_cast<int>(m_reachedJoints[2]), static_cast<int>(m_reachedJoints[3]));
        return;
    }

    if (m_flag_planning)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Planner idle: a plan request is still outstanding");
        return;
    }

    if (!(target_changed || q_changed))
    {
        if (m_plan_retry_armed)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Planner idle: last plan was rejected - retrying the same target "
                                 "in %.1f s",
                                 std::max(0.0, m_plan_retry_after_s.load() - this->now().seconds()));
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Planner idle: target unchanged - move the probe more than %.0f mm "
                             "(or use Next/Previous in CSV mode) to request a new plan",
                             k_target_threshold * 1e3);
    }
}

void MasterNode::control_loop()
{
    if (!m_services_ready)
    {
        // Gates the whole loop, including deployment and the auto test - not just planning.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Control loop idle: waiting on services: %s",
                             missingServicesDescription().c_str());
        return; // robot/planner/recorder services not up yet
    }

    // async_send_request never times out. Without this, a planner that dies or hangs
    // mid-solve leaves m_flag_planning raised forever and blocks every branch below.
    if (m_flag_planning)
    {
        const double outstanding_s = this->now().seconds() - m_planner_request_time_s.load();
        if (outstanding_s > m_planner_timeout_s)
        {
            RCLCPP_ERROR(get_logger(),
                         "No planner response after %.1f s (limit %.1f s) - abandoning the request. "
                         "Check that the planner node is still alive.",
                         outstanding_s, m_planner_timeout_s);
            m_flag_planning = false;
            m_planner_success = false;
            // Same reasoning as a rejected response: the target is already latched, so
            // without a retry this target is never asked for again.
            armPlanRetry("planner did not respond within " + std::to_string(m_planner_timeout_s) + " s");
            emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);
        }
    }

    // Update automated test state machine
    updateTestStateMachine();

    // Snapshot the cross-thread feedback state once per cycle
    Eigen::Vector3d X, Xd, Xsim;
    {
        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        X = m_X;
        Xd = m_Xd;
        Xsim = m_Xsim;
    }

    if (m_procedure && m_high_level_mode == HighLvlCtrMode::Planner)
    {
        // Pre-rotation gate. The gate diffs are BEARING misalignments (wrapped:
        // a direction, not a travel), while the command is the travel-true
        // nearest 2π-representative of the bearing inside the trained α box --
        // see manager/bearing_gate.hpp for why the old raw-diff gate and
        // wrapped-command pair made +y targets unplannable.
        const auto pr = manager_gate::computePreRotation(Xd[0], Xd[1], m_q[0], m_q[2], k_alpha_limits);
        const double target_theta = pr.target_theta;

        double tube_1_theta_diff = pr.gate_diff_1;
        double tube_2_theta_diff = pr.gate_diff_2;
        bool target_changed = (Xd - m_Xd_prev).norm() > k_target_threshold;
        bool q_changed = blaze::norm((m_q - m_q_prev) / k_input_scale) > k_q_threshold;

        // If tube bearings differ from the target bearing, command the rotation
        if (tube_1_theta_diff > k_theta_threshold || tube_2_theta_diff > k_theta_threshold)
        {
            blaze::StaticVector<double, 6> q = blaze::StaticVector<double, 6>({m_q[1], m_q[3], 0.0, pr.cmd_alpha, pr.cmd_alpha, 0.0});
            publish_position(q);
            // Record the first command toward a new bearing (edge-triggered so a
            // 100 Hz loop does not flood the CSV while the tubes slew).
            if (std::abs(pr.cmd_alpha - m_last_prerotate_cmd) > 1e-6)
            {
                m_last_prerotate_cmd = pr.cmd_alpha;
                RCLCPP_INFO(get_logger(),
                            "Pre-rotating tubes: bearing %.4f rad -> command alpha = %.4f rad "
                            "(current a1 = %.4f, a2 = %.4f; bearing off by %.1f/%.1f deg)",
                            target_theta, pr.cmd_alpha, m_q[0], m_q[2],
                            tube_1_theta_diff * 180.0 / M_PI, tube_2_theta_diff * 180.0 / M_PI);
                diagPreRotate(Xd, pr);
            }
            reportPlannerGate(tube_1_theta_diff, tube_2_theta_diff, target_changed, q_changed);
        }
        // If target position changed significantly, call planner to generate new path
        else
        {
            // std::cout << "m_reached: " << m_reached << ", !m_flag_planning: " << (!m_flag_planning) << ", target_changed: " << target_changed << ", q_changed: " << q_changed << std::endl;
            if (m_reached && (!m_flag_planning) && (target_changed || q_changed || planRetryDue()))
            {
                m_plan_retry_armed = false;
                {
                    std::lock_guard<std::mutex> lock(m_force_mutex);
                    m_f_pending = m_f_est; // baseline promoted to m_f_at_plan when the plan is accepted
                }

                auto request = std::make_shared<interfaces::srv::Planner::Request>();
                request->command = "generateTrajectory";
                request->value[0] = Xd[0];
                request->value[1] = Xd[1];
                request->value[2] = Xd[2];

                auto response_callback = std::bind(&MasterNode::handle_planner_response, this, std::placeholders::_1);
                auto future_result = m_planner_client->async_send_request(request, response_callback);

                m_Xd_prev = Xd;
                m_q_prev = m_q;
                m_flag_planning = true;
                m_planner_request_time_s = this->now().seconds();
                m_flag_planner_updated = true;
                emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);
                RCLCPP_INFO(this->get_logger(), "Planner called: target = [%.4f, %.4f, %.4f] (azimuth %.4f rad)",
                            Xd[0], Xd[1], Xd[2], std::atan2(Xd[1], Xd[0]));
                diagPlanRequest(Xd, "planner_mode");
            }
            else
            {
                reportPlannerGate(tube_1_theta_diff, tube_2_theta_diff, target_changed, q_changed);
            }
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Deployment && !m_closed_loop_enabled)
    {
        maybeRequestDeploymentReplan();

        if (!m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert))
        {
            m_retracting = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_home_tail_active)
            {
                // The robot is partway along the home leg, so it is not at any
                // plan waypoint and the index means nothing. Inserting from here
                // would command an arbitrary jump back onto the old path.
                m_auto_insert = false;
                RCLCPP_WARN(this->get_logger(),
                            "Cannot insert: a retract-to-home leg is in progress (step %zu/%zu) and "
                            "the robot is no longer at a plan waypoint. Let the retraction finish, "
                            "then plan again.",
                            m_home_tail_index, m_q_list_home_tail.size());
                return;
            }
            if (m_q_list_adjusted.empty())
            {
                // Must return: the index arithmetic below computes size() - 1 on an empty
                // vector, which underflows before being narrowed to int.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "Adjusted deployment list is empty; cannot send positions. "
                                     "Plan a path in 'Select Target' mode first.");
                return;
            }

            if (!getReachStatus())
            {
                reportReachStall(false, m_current_config_index, m_q_list_adjusted.size());
                return;
            }

            noteReachProgress();
            m_current_config_index += 1;
            if (m_current_config_index < static_cast<int>(m_q_list_adjusted.size()))
            {
                // Capture frame in session A during auto-insertion
                if (m_test_running && m_auto_insert)
                {
                    captureRecordingFrame("data");
                }

                sendDeploymentWaypoint(m_q_list_adjusted[m_current_config_index], "insert",
                                       m_current_config_index + 1, m_q_list_adjusted.size());
            }
            else
            {
                m_auto_insert = false;
                m_current_config_index = static_cast<int>(m_q_list_adjusted.size()) - 1;
                RCLCPP_INFO(this->get_logger(), "Max deployment index reached: %d", m_current_config_index);
                if (!m_deploy_complete_logged)
                {
                    m_deploy_complete_logged = true;
                    diagDeployComplete(Xd, X, Xsim);
                }
            }
        }
        else if (!m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract))
        {
            m_retracting = true;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_q_list_adjusted.empty() && !m_home_tail_active)
            {
                // Must return: the index arithmetic below computes size() - 1 on an empty
                // vector, which underflows before being narrowed to int.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "Adjusted deployment list is empty; cannot send positions. "
                                     "Plan a path in 'Select Target' mode first.");
                return;
            }

            if (!getReachStatus())
            {
                reportReachStall(true,
                                 m_home_tail_active ? static_cast<int>(m_home_tail_index)
                                                    : m_current_config_index,
                                 m_home_tail_active ? m_q_list_home_tail.size()
                                                    : m_q_list_adjusted.size());
                return;
            }

            noteReachProgress();

            // Second stage: the reversed plan is exhausted, walk the home leg.
            // Reversing the plan only reaches m_q_list_adjusted[0], which is the
            // pose the robot was in when the plan was made -- not the mechanical
            // home, and after an accepted replan not even the deployment start.
            if (m_home_tail_active)
            {
                if (m_home_tail_index < m_q_list_home_tail.size())
                {
                    sendDeploymentWaypoint(m_q_list_home_tail[m_home_tail_index], "retract-home",
                                           static_cast<int>(m_home_tail_index) + 1,
                                           m_q_list_home_tail.size());
                    ++m_home_tail_index;
                }
                else
                {
                    finishRetraction(true, m_q_list_home_tail.size(), X, Xsim);
                }
                return;
            }

            m_current_config_index--;
            if (m_current_config_index >= 0)
            {
                sendDeploymentWaypoint(m_q_list_adjusted[m_current_config_index], "retract",
                                       m_current_config_index + 1, m_q_list_adjusted.size());
            }
            else
            {
                m_current_config_index = 0;
                beginHomeTail(m_q_list_adjusted.front());
                if (!m_home_tail_active)
                {
                    // Already at home to within one step; nothing left to walk.
                    finishRetraction(true, 0, X, Xsim);
                }
            }
        }
        else
        {
            m_last_reach_progress_s.store(0.0);
            reportDeploymentGate();
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Deployment && m_closed_loop_enabled)
    {
        Eigen::Vector3d Xsim_error(0.0, 0.0, 0.0);
        Xsim_error = X - Xsim;
        Eigen::Vector3d Xd_adj = Xd - Xsim_error;
        bool target_changed = (Xd_adj - m_Xd_adj_prev).norm() > k_target_threshold;
        bool q_changed = blaze::norm((m_q - m_q_prev) / k_input_scale) > k_q_threshold;
        double Xe = (X - Xd).norm();

        // Call planner if target changed significantly
        if (m_reached && !m_flag_planning && (target_changed || q_changed || planRetryDue()) &&
            !m_retracting && (Xe > (k_target_threshold / 2)))
        {
            m_plan_retry_armed = false;
            m_flag_planning = true;
            m_planner_request_time_s = this->now().seconds();
            emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);

            {
                std::lock_guard<std::mutex> lock(m_force_mutex);
                m_f_pending = m_f_est; // baseline promoted to m_f_at_plan when the plan is accepted
            }

            auto request = std::make_shared<interfaces::srv::Planner::Request>();
            request->command = "generateTrajectory";
            request->value[0] = Xd_adj[0];
            request->value[1] = Xd_adj[1];
            request->value[2] = Xd_adj[2];

            auto response_callback = std::bind(&MasterNode::handle_planner_response, this, std::placeholders::_1);
            auto future_result = m_planner_client->async_send_request(request, response_callback);
            RCLCPP_INFO(this->get_logger(), "Planner called (closed loop): target = [%.4f, %.4f, %.4f] (azimuth %.4f rad)",
                        Xd_adj[0], Xd_adj[1], Xd_adj[2], std::atan2(Xd_adj[1], Xd_adj[0]));
            diagPlanRequest(Xd_adj, "closed_loop");

            m_Xd_adj_prev = Xd_adj;
            m_q_prev = m_q;

            std::this_thread::sleep_for(10ms);
        }

        // Handle manual and auto insertion
        if (!m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert))
        {
            m_retracting = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_home_tail_active)
            {
                // The robot is partway along the home leg, so it is not at any
                // plan waypoint and the index means nothing. Inserting from here
                // would command an arbitrary jump back onto the old path.
                m_auto_insert = false;
                RCLCPP_WARN(this->get_logger(),
                            "Cannot insert: a retract-to-home leg is in progress (step %zu/%zu) and "
                            "the robot is no longer at a plan waypoint. Let the retraction finish, "
                            "then plan again.",
                            m_home_tail_index, m_q_list_home_tail.size());
                return;
            }
            if (m_q_list_adjusted.empty())
            {
                // Must return: the index arithmetic below computes size() - 1 on an empty
                // vector, which underflows before being narrowed to int.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "Adjusted deployment list is empty; cannot send positions. "
                                     "Plan a path in 'Select Target' mode first.");
                return;
            }

            if (!getReachStatus())
            {
                reportReachStall(false, m_current_config_index, m_q_list_adjusted.size());
                return;
            }

            noteReachProgress();
            if (m_q_list_actuated.empty())
            {
                blaze::StaticVector<double, 6> q = blaze::StaticVector<double, 6>({m_q[1], m_q[3], 0.0, m_q[0], m_q[2], 0.0});
                m_q_list_actuated.push_back(q);
            }

            m_current_config_index += 1;
            if (m_current_config_index < static_cast<int>(m_q_list_adjusted.size()))
            {
                // Capture frame in session A during auto-insertion
                if (m_test_running && m_auto_insert)
                {
                    captureRecordingFrame("data");
                }

                sendDeploymentWaypoint(m_q_list_adjusted[m_current_config_index], "insert",
                                       m_current_config_index + 1, m_q_list_adjusted.size());

                m_q_list_actuated.push_back(m_q_list_adjusted[m_current_config_index]);
            }
            else
            {
                m_auto_insert = false;
                m_q_list_adjusted.clear();
                invalidateForceBaseline();
                m_current_config_index = 0;
                RCLCPP_INFO(this->get_logger(), "Max deployment index reached: %d", m_current_config_index);
                if (!m_deploy_complete_logged)
                {
                    m_deploy_complete_logged = true;
                    diagDeployComplete(Xd, X, Xsim);
                }
            }
        }
        // Handle manual and auto retraction
        else if (!m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract))
        {
            m_retracting = true;
            m_auto_insert = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);

            if (!getReachStatus())
            {
                reportReachStall(true,
                                 m_home_tail_active ? static_cast<int>(m_home_tail_index) : 0,
                                 m_home_tail_active ? m_q_list_home_tail.size() : m_q_list_actuated.size());
                return;
            }

            noteReachProgress();

            // Same two-stage retraction as the open-loop branch: unwind the
            // commanded stack first, then walk the home leg. m_q_list_actuated[0]
            // is only the pose the first insert step started from, which is not
            // the mechanical home.
            if (m_home_tail_active)
            {
                if (m_home_tail_index < m_q_list_home_tail.size())
                {
                    sendDeploymentWaypoint(m_q_list_home_tail[m_home_tail_index], "retract-home",
                                           static_cast<int>(m_home_tail_index) + 1,
                                           m_q_list_home_tail.size());
                    ++m_home_tail_index;
                }
                else
                {
                    const size_t steps = m_q_list_home_tail.size();
                    deletePlannedPathFile();
                    m_q_list_actuated.clear();
                    finishRetraction(true, steps, X, Xsim);
                }
                return;
            }

            if (m_q_list_actuated.size() > 1)
            {
                m_q_list_actuated.pop_back();
                sendDeploymentWaypoint(m_q_list_actuated.back(), "retract",
                                       static_cast<int>(m_q_list_actuated.size()),
                                       m_q_list_actuated.size() + 1);
            }
            else if (!m_q_list_actuated.empty())
            {
                beginHomeTail(m_q_list_actuated.front());
                if (!m_home_tail_active)
                {
                    deletePlannedPathFile();
                    m_q_list_actuated.clear();
                    finishRetraction(true, 0, X, Xsim);
                }
            }
            else
            {
                // Nothing was ever commanded, so there is nothing to unwind.
                // Do NOT "finish" here: the physical retract button can be held
                // down, and finishing on every cycle would spam the diag file.
                m_auto_retract = false;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "Nothing to retract: no waypoint has been commanded yet. "
                                     "Use the robot GUI's 'Go to Home' to move to the home pose.");
            }
        }
    }
    else if (!m_procedure)
    {
        // Every branch above needs m_procedure, which mirrors robot_status.procedure and
        // is set only by the robot's startProcedure (control mode -> Position).
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Control loop idle: robot is not in Procedure - press 'Enable' "
                             "then 'Start Procedure' (Robot info table shows Procedure = ON)");
    }
    else
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Control loop idle: high-level mode is None - click the "
                             "'Select Target' or 'Deployment' radio to re-arm it");
    }
}

void MasterNode::publish_position(const blaze::StaticVector<double, 6> &q)
{
    interfaces::msg::Jointspace msg;
    msg.position = ctr_common::physicsToWire(q);
    m_pub_joint_targ->publish(msg);
}

// ============================================================================
// Utility Functions
// ============================================================================

// Load the planner's CSV output and swap it in as the active deployment list.
//
// `is_replan` is load-bearing. "replanDeployment" re-plans only the REMAINING
// tail, from the robot's current (already partly inserted) configuration, so
// its row 0 is not the deployment start. Resetting m_current_config_index to 0
// for a replan -- which this function used to do unconditionally, for both
// callers -- redefined "fully retracted" as "back to wherever the replan
// happened". Auto Retract then completed partway in, cleared the list and
// parked. Worse, a replan response landing while retract was armed left the
// next tick decrementing 0 to -1, ending the retraction without moving at all.
//
// So a replan SPLICES: the already-executed prefix is kept and the new tail is
// appended behind it, leaving a list that still reaches back to the true start.
bool MasterNode::loadPlannedPath(const bool is_replan)
{
    std::lock_guard<std::mutex> lock(m_deploy_mutex);
    if (read_path_from_csv(m_q_list, "plannedPath.csv"))
    {
        // Log the file mtime so a stale plan (planner wrote nothing new) is
        // visible in the logs when diagnosing deployment behaviour.
        std::error_code ec;
        const auto csv_path = ctr_common::resolveDataRoot(*this, "manager") / "Shared_Files" / "plannedPath.csv";
        const auto mtime = std::filesystem::last_write_time(csv_path, ec);
        if (!ec)
        {
            const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::filesystem::file_time_type::clock::now() - mtime)
                                 .count();
            RCLCPP_INFO(this->get_logger(), "plannedPath.csv loaded (written %.1f s ago)", age / 1000.0);
        }

        auto tail = adjustConfigurationListStepSize(m_q_list, m_insertion_step);

        const bool can_splice = is_replan && !m_q_list_adjusted.empty() &&
                                m_current_config_index >= 0 &&
                                m_current_config_index < static_cast<int>(m_q_list_adjusted.size());
        if (can_splice)
        {
            const size_t prefix_size = static_cast<size_t>(m_current_config_index) + 1UL;
            std::vector<blaze::StaticVector<double, 6>> spliced;
            spliced.reserve(prefix_size + tail.size());
            spliced.insert(spliced.end(), m_q_list_adjusted.begin(),
                           m_q_list_adjusted.begin() + static_cast<long>(prefix_size));
            spliced.insert(spliced.end(), tail.begin(), tail.end());
            m_q_list_adjusted = std::move(spliced);
            // The index still points at the last executed waypoint, so the next
            // insertion step advances onto the new tail's first entry.
            RCLCPP_INFO(this->get_logger(),
                        "Replan spliced onto the executed path: %zu already-executed waypoints + "
                        "%zu new = %zu total, resuming at index %d",
                        prefix_size, tail.size(), m_q_list_adjusted.size(), m_current_config_index);
        }
        else
        {
            if (is_replan)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Replan could not be spliced (list size %zu, index %d) - "
                            "retraction will only reach this replan's start pose",
                            m_q_list_adjusted.size(), m_current_config_index);
            }
            m_q_list_adjusted = std::move(tail);
            m_current_config_index = 0;
        }
        m_deploy_complete_logged = false;
        // A freshly loaded path supersedes any half-walked home leg.
        m_q_list_home_tail.clear();
        m_home_tail_index = 0;
        m_home_tail_active = false;
        m_last_commanded_valid = false;

        // Downsampler telemetry: a per-step Δα near 2π means a rotation phase
        // was collapsed and would execute as one unmanaged full turn.
        const double max_step_alpha = manager_csv::maxAlphaStep(m_q_list_adjusted);
        RCLCPP_INFO(this->get_logger(), "Deployment list %s %zu -> %zu waypoints (max per-step dAlpha = %.3f rad)",
                    is_replan ? "replanned/spliced" : "downsampled",
                    m_q_list.size(), m_q_list_adjusted.size(), max_step_alpha);
        diagPathLoaded(m_q_list.size(), m_q_list_adjusted.size(), max_step_alpha);
        return true;
    }
    return false;
}

bool MasterNode::read_path_from_csv(std::vector<blaze::StaticVector<double, 6>> &init_q_list, const std::string &fileName)
{
    const std::filesystem::path file_path =
        ctr_common::resolveDataRoot(*this, "manager") / "Shared_Files" / fileName;

    const auto rows = ctr_common::csv::readNumericCsv(file_path);
    if (!rows)
    {
        RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
        return false;
    }

    // Legacy 6-column layout or the planner's 4-column layout; see
    // manager_csv::parsePathRows for the expansion semantics.
    size_t bad_rows = 0;
    init_q_list = manager_csv::parsePathRows(*rows, &bad_rows);
    if (bad_rows > 0)
    {
        RCLCPP_WARN(get_logger(), "%zu rows did not contain 4 or 6 values and were skipped", bad_rows);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu path points from CSV file.", init_q_list.size());
    return true;
}

void MasterNode::read_targets_from_csv(std::vector<Eigen::Vector3d> &target_list, const std::string &fileName)
{
    const std::filesystem::path file_path =
        ctr_common::resolveDataRoot(*this, "manager") / "Input_Files" / fileName;

    // Clear first: returning early on a failed read used to leave the caller holding the
    // previously loaded targets, which then looked like a successful reload.
    target_list.clear();

    const auto rows = ctr_common::csv::readNumericCsv(file_path);
    if (!rows)
    {
        RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    // Header line (x,y,z) is non-numeric and dropped by the parser.
    for (const auto &row : *rows)
    {
        if (row.size() == 3)
        {
            target_list.emplace_back(row[0], row[1], row[2]);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Row does not contain exactly 3 values (x,y,z), got %zu", row.size());
        }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu task-space targets from %s", target_list.size(), fileName.c_str());
}

bool MasterNode::getReachStatus()
{
    return m_reached;
}

std::vector<blaze::StaticVector<double, 6>> MasterNode::adjustConfigurationListStepSize(const std::vector<blaze::StaticVector<double, 6>> &q_list_in, double step_size)
{
    if (q_list_in.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty list");
    }
    return manager_csv::adjustConfigurationListStepSize(q_list_in, step_size);
}

void MasterNode::log_position(Eigen::Vector3d position, std::string prefix)
{
    std::ostringstream oss;
    auto print_with_space_if_positive = [](double value)
    {
        std::ostringstream tmp;
        tmp << std::fixed << std::setprecision(3);
        if (value >= 0)
        {
            tmp << " " << value;
        }
        else
        {
            tmp << value;
        }
        return tmp.str();
    };

    oss << '[' << prefix << "] "
        << "X:" << print_with_space_if_positive(position[0]) << "  "
        << "Y:" << print_with_space_if_positive(position[1]) << "  "
        << "Z:" << print_with_space_if_positive(position[2]) << " [m]";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
}

void MasterNode::captureRecordingFrame(const std::string &session_name)
{
    auto request = std::make_shared<interfaces::srv::Recording::Request>();
    request->command = "capture";
    request->name = session_name;
    request->duration = 0; // Not used for capture command

    m_recording_client->async_send_request(request);
}

void MasterNode::updateTestStateMachine()
{
    if (!m_test_running)
        return;

    auto current_time = this->now();
    auto time_in_state = (current_time - m_test_state_entry_time).seconds();

    switch (m_test_state)
    {
    case TestState::Idle:
        // Do nothing
        break;

    case TestState::SelectingTarget:
    {
        // Switch to Planner mode by clicking the radio button
        if (m_high_level_mode != HighLvlCtrMode::Planner)
        {
            {
                std::lock_guard<std::mutex> lock(m_feedback_mutex);
                m_Xd = m_test_targets[m_current_target_index];
            }
            RCLCPP_INFO(get_logger(), "[Test] Target %zu/%zu selected",
                        m_current_target_index + 1, m_test_targets.size());
            log_position(m_Xd, "Target");
            m_target_set = true;

            // Publish target position
            auto target_msg = interfaces::msg::Taskspace();
            target_msg.p[0] = m_Xd[0];
            target_msg.p[1] = m_Xd[1];
            target_msg.p[2] = m_Xd[2];
            m_pub_task_target->publish(target_msg);

            onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::Planner));
            // control_loop runs on a ROS executor thread — marshal the widget call
            QMetaObject::invokeMethod(this, [this]()
                                      { m_gui_manager->getPlannerRadioButton()->setChecked(true); },
                                      Qt::QueuedConnection);
            RCLCPP_INFO(get_logger(), "[Test] Switched to Planner mode");
        }

        // Set target position (only once when first entering this state)
        if (m_current_target_index < m_test_targets.size())
        {
            m_test_state = TestState::WaitingForPlanning;
            m_test_state_entry_time = current_time;
            RCLCPP_INFO(get_logger(), "[Test] Waiting for planning to complete...");
            // }
        }
        else
        {
            // All targets processed
            m_test_state = TestState::Complete;
            m_test_state_entry_time = current_time;
        }
        break;
    }

    case TestState::WaitingForPlanning:
    {
        // Wait for planner to finish and reach target orientation
        if (!m_flag_planning && m_reached && time_in_state > 2.0)
        {
            // bool bypass_error_check = m_closed_loop_enabled && m_q_list_actuated.empty();
            bool bypass_error_check = false;

            // Accept-polarity `<`, matching handle_planner_response(). Keep it that
            // way: a non-finite IK error must fail this test, and inverting it to
            // `>= threshold` would silently let one through.
            if ((m_planner_success || bypass_error_check) && m_planner_ik_error < k_ik_error_threshold && m_flag_planner_updated)
            {
                RCLCPP_INFO(get_logger(), "[Test] Planning successful, switching to Deployment mode");
                m_test_state = TestState::SwitchingToDeployment;
                m_test_state_entry_time = current_time;
                m_flag_planner_updated = false;
            }
            else if (time_in_state > 10.0)
            {
                RCLCPP_WARN(get_logger(), "[Test] Planning timeout or failed, skipping to next target");
                m_high_level_mode = HighLvlCtrMode::None;
                m_current_target_index++;
                m_test_state = TestState::SelectingTarget;
                m_test_state_entry_time = current_time;
                m_target_set = false;
            }
        }
        break;
    }

    case TestState::SwitchingToDeployment:
    {
        // Switch to Deployment mode by clicking the radio button
        if (m_high_level_mode != HighLvlCtrMode::Deployment)
        {
            onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::Deployment));
            // control_loop runs on a ROS executor thread — marshal the widget call
            QMetaObject::invokeMethod(this, [this]()
                                      { m_gui_manager->getDeploymentRadioButton()->setChecked(true); },
                                      Qt::QueuedConnection);
            RCLCPP_INFO(get_logger(), "[Test] Switched to Deployment mode");
        }

        // Wait for mode switch to complete
        if (time_in_state > 1.0 && m_reached)
        {
            m_test_state = TestState::Inserting;
            m_test_state_entry_time = current_time;
            m_auto_insert = true;
            RCLCPP_INFO(get_logger(), "[Test] Starting auto-insertion...");
        }
        break;
    }

    case TestState::Inserting:
    {
        // Auto-insert is already enabled, just monitor for completion
        if (!m_auto_insert || time_in_state > 1.0)
        {
            m_test_state = TestState::WaitingInsertionComplete;
            m_test_state_entry_time = current_time;
            RCLCPP_INFO(get_logger(), "[Test] Waiting for insertion to complete...");
        }
        break;
    }

    case TestState::WaitingInsertionComplete:
    {
        // Check if insertion is complete (reached end of trajectory)
        // if (m_current_config_index >= static_cast<int>(m_q_list_adjusted.size()) - 1 && m_reached)
        if (!m_auto_insert && m_reached)
        {
            RCLCPP_INFO(get_logger(), "[Test] Insertion complete, starting retraction...");

            // Capture frame in session B when insertion is complete
            captureRecordingFrame("deployed");
            RCLCPP_INFO(get_logger(), "[Test] Captured recording frame in session B");

            m_test_state = TestState::Retracting;
            m_test_state_entry_time = current_time;
            m_auto_retract = true;
            // m_auto_insert = false;
        }
        else if (time_in_state > 60.0)
        {
            RCLCPP_WARN(get_logger(), "[Test] Insertion timeout, proceeding to retraction");
            m_test_state = TestState::Retracting;
            m_test_state_entry_time = current_time;
            m_auto_retract = true;
            m_auto_insert = false;
        }
        break;
    }

    case TestState::Retracting:
    {
        // Auto-retract is already enabled, just monitor
        if (!m_auto_retract || time_in_state > 1.0)
        {
            m_test_state = TestState::WaitingRetractionComplete;
            m_test_state_entry_time = current_time;
            RCLCPP_INFO(get_logger(), "[Test] Waiting for retraction to complete...");
        }
        break;
    }

    case TestState::WaitingRetractionComplete:
    {
        // Check if retraction is complete (back to start)
        if (!m_auto_retract && m_reached)
        {
            RCLCPP_INFO(get_logger(), "[Test] Retraction complete!");

            // Move to next target
            m_current_target_index++;

            if (m_current_target_index < m_test_targets.size())
            {
                RCLCPP_INFO(get_logger(), "[Test] Proceeding to target %zu/%zu",
                            m_current_target_index + 1, m_test_targets.size());
                m_test_state = TestState::SelectingTarget;
                m_test_state_entry_time = current_time;
                m_target_set = false;
            }
            else
            {
                m_test_state = TestState::Complete;
                m_test_state_entry_time = current_time;
            }
        }
        else if (time_in_state > k_test_retraction_timeout_s)
        {
            // This abandons a retraction MID-MOVE, leaving the tubes wherever
            // they happen to be, so the budget has to cover the worst case: the
            // whole reversed plan plus the home leg, at one 2 mm step per
            // settle. Raised from 60 s when the home leg was added.
            RCLCPP_WARN(get_logger(),
                        "[Test] Retraction timeout after %.0f s at waypoint %d (home leg %s) - "
                        "skipping to next target with the tubes still deployed",
                        time_in_state, m_current_config_index,
                        m_home_tail_active ? "in progress" : "not started");
            m_auto_retract = false;
            {
                std::lock_guard<std::mutex> lock(m_deploy_mutex);
                m_q_list_adjusted.clear();
                m_q_list_actuated.clear();
                m_q_list_home_tail.clear();
                m_home_tail_index = 0;
                m_home_tail_active = false;
                m_last_commanded_valid = false;
            }
            invalidateForceBaseline();
            m_current_target_index++;
            m_test_state = TestState::SelectingTarget;
            m_test_state_entry_time = current_time;
            m_target_set = false;
        }
        break;
    }

    case TestState::Complete:
    {
        RCLCPP_INFO(get_logger(), "=== Automated Test Complete! ===");
        RCLCPP_INFO(get_logger(), "Processed %zu targets", m_test_targets.size());
        m_test_running = false;
        m_test_state = TestState::Idle;
        m_auto_insert = false;
        m_auto_retract = false;
        {
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            m_q_list_adjusted.clear();
            m_q_list_actuated.clear();
        }
        invalidateForceBaseline();
        break;
    }
    }
}

// ============================================================================
// Main Function
// ============================================================================

#include "manager/master_node.moc"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<MasterNode>();
    node->show();
    node->setFocus();

    std::thread ros_spin_thread([&]()
                                {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 7);
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown(); });

    int result = app.exec();
    rclcpp::shutdown();
    ros_spin_thread.join();
    return result;
}

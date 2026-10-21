#include "manager/master_node.hpp"

#include "ctr_common/csv_io.hpp"
#include "manager/csv_path_io.hpp"
#include "manager/deployment_path.hpp"
#include "ctr_common/joint_conventions.hpp"
#include "ctr_common/runtime_paths.hpp"
#include "ctr_common/package_log.hpp"
#include "ctr_common/path_downsample.hpp"
#include "ctr_common/planner_protocol.hpp"

#include <algorithm>
#include <sstream>

using namespace std::chrono_literals;

// ============================================================================
// Constructor
// ============================================================================

MasterNode::MasterNode(QWidget *parent)
    : QWidget(parent), rclcpp::Node("master_node"), m_gui_manager(std::make_unique<QtGuiManager>(this))
{
    ctr_common::installPackageFileLog(*this, "manager");

    std::cout << "Initializing MasterNode..." << std::endl;

    // Replanning tunables and target-list source (defaults preserve the old constexprs)
    m_force_replan_threshold = declare_parameter<double>("force_replan_threshold", m_force_replan_threshold);
    m_replan_cooldown_s = declare_parameter<double>("replan_cooldown_s", m_replan_cooldown_s);
    m_min_remaining_waypoints =
        static_cast<size_t>(declare_parameter<int>("min_remaining_waypoints", static_cast<int>(m_min_remaining_waypoints)));
    m_targets_csv = declare_parameter<std::string>("targets_csv", m_targets_csv);
    m_planner_timeout_s = declare_parameter<double>("planner_timeout_s", m_planner_timeout_s);

    const auto data_root = ctr_common::resolveDataRoot(*this, "manager");
    RCLCPP_INFO(get_logger(),
                "[lifecycle] master_node starting: force_replan_threshold=%.3f N replan_cooldown_s=%.2f s "
                "min_remaining_waypoints=%zu targets_csv=%s planner_timeout_s=%.1f s data_root=%s",
                m_force_replan_threshold, m_replan_cooldown_s, m_min_remaining_waypoints,
                m_targets_csv.c_str(), m_planner_timeout_s, data_root.c_str());

    m_gui_manager->initializeGui();
    initRosInterfaces();
    onCtrlModeClicked(static_cast<int>(HighLvlCtrMode::Planner));
    // m_high_level_mode = HighLvlCtrMode::Planner;
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

    auto client = use_enable_service ? m_robot_enable_client : m_robot_config_client;
    RCLCPP_INFO(get_logger(), "[comm] sending %s command '%s'",
                use_enable_service ? "robot_enable" : "robot_config", command.c_str());
    (void)client->async_send_request(
        request, [this, command](rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
        { handle_service_response(command, future); });
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
        m_Xd = m_csv_targets[m_csv_target_index];
        
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
        m_Xd = m_csv_targets[m_csv_target_index];
        
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
        m_Xd = m_csv_targets[m_csv_target_index];
        
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

    std::string mode_name;
    switch (static_cast<HighLvlCtrMode>(id))
    {
    case HighLvlCtrMode::Planner:
    {
        mode_name = "Planner";
        m_auto_insert = false;
        m_auto_retract = false;
        m_retracting = false;
        // m_gui_manager->getClosedLoopCheckbox()->setEnabled(false);
        break;
    }
    case HighLvlCtrMode::Deployment:
    {
        mode_name = m_closed_loop_enabled ? "Deployment (Closed-Loop)" : "Deployment";
        m_auto_insert = false;
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
        const bool config_ready = m_robot_config_client->service_is_ready();
        const bool enable_ready = m_robot_enable_client->service_is_ready();
        const bool planner_ready = m_planner_client->service_is_ready();
        const bool freeze_ready = m_freeze_robot_client->service_is_ready();
        const bool recording_ready = m_recording_client->service_is_ready();
        const bool ready = config_ready && enable_ready && planner_ready && freeze_ready && recording_ready;
        if (ready && !m_services_ready.exchange(true))
            RCLCPP_INFO(get_logger(), "[lifecycle] All robot/planner/recorder services are ready");
        else if (!ready && m_services_ready.exchange(false))
            RCLCPP_WARN(get_logger(), "[lifecycle] A required service went away - controls gated until it returns");

        if (!ready)
        {
            std::string missing;
            if (!config_ready) missing += "robot_config,";
            if (!enable_ready) missing += "robot_enable,";
            if (!planner_ready) missing += "planner/command,";
            if (!freeze_ready) missing += "freeze_robot,";
            if (!recording_ready) missing += "recording,";
            if (!missing.empty())
                missing.pop_back();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[lifecycle] waiting for services: %s", missing.c_str());
        } });

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

    if (m_procedure != m_procedure_log_prev)
    {
        RCLCPP_INFO(get_logger(), "[comm] procedure %s", m_procedure ? "ACTIVE" : "inactive");
        m_procedure_log_prev = m_procedure;
    }
    if (m_enabled != m_enabled_log_prev)
    {
        RCLCPP_INFO(get_logger(), "[comm] robot enable: %s", m_enabled ? "ENABLED" : "disabled");
        m_enabled_log_prev = m_enabled;
    }
    if (m_reached != m_reached_log_prev)
    {
        RCLCPP_INFO(get_logger(), "[comm] all-joints reached: %s", m_reached ? "true" : "false");
        m_reached_log_prev = m_reached;
    }
    {
        const int ctrl_mode_int = static_cast<int>(m_ctrl_mode);
        if (ctrl_mode_int != m_control_mode_log_prev)
        {
            RCLCPP_INFO(get_logger(), "[comm] control_mode changed: %d", ctrl_mode_int);
            m_control_mode_log_prev = ctrl_mode_int;
        }
    }
    if (m_trans_limit != m_trans_limit_prev)
    {
        RCLCPP_INFO(get_logger(), "[comm] trans_limit_en: %s", m_trans_limit ? "true" : "false");
        m_trans_limit_prev = m_trans_limit;
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
    std::lock_guard<std::mutex> lock(m_force_mutex);
    m_f_est = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void MasterNode::tf2_receive_timer_callback()
{
    geometry_msgs::msg::TransformStamped tf2_tran;
    std::string targetFrame = "robot_base";
    std::string sourceFrame = "ctr_tip";

    try
    {
        tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[comm] Could not transform %s to %s: %s",
                              targetFrame.c_str(), sourceFrame.c_str(), ex.what());
        m_tf2_unavailable_prev = true;
        return;
    }
    m_trans_tip = tf2::transformToEigen(tf2_tran).matrix();

    sourceFrame = "probe";
    try
    {
        tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[comm] Could not transform %s to %s: %s",
                              targetFrame.c_str(), sourceFrame.c_str(), ex.what());
        m_tf2_unavailable_prev = true;
        return;
    }
    m_trans_probe = tf2::transformToEigen(tf2_tran).matrix();

    if (m_tf2_unavailable_prev)
    {
        RCLCPP_INFO(this->get_logger(), "[comm] tip/probe transforms available again");
        m_tf2_unavailable_prev = false;
    }

    // Update positions
    if (!m_test_running && !m_use_csv_target)
    {
        {
            std::lock_guard<std::mutex> lock(m_feedback_mutex);
            m_Xd = m_trans_probe.block<3, 1>(0, 3); // target tip position
        }

        // Publish target position
        auto target_msg = interfaces::msg::Taskspace();
        target_msg.p[0] = m_Xd[0];
        target_msg.p[1] = m_Xd[1];
        target_msg.p[2] = m_Xd[2];
        m_pub_task_target->publish(target_msg);
    }

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

void MasterNode::handle_service_response(const std::string &command, const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
{
    auto response = future.get();
    if (response->success)
    {
        RCLCPP_DEBUG(this->get_logger(), "[comm] '%s' succeeded: %s", command.c_str(), response->message.c_str());
        if (command == "goHome" || command == "findRotaryHomeAndGoHome" || command == "findLinearHome")
        {
            resetDeploymentState(command);
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[comm] '%s' failed: %s", command.c_str(), response->message.c_str());
    }
}

// Single emit point for the GUI planner-status row; the message is written by the
// service-response executor thread and read here from any thread.
void MasterNode::emitPlannerStatus()
{
    std::string message;
    {
        std::lock_guard<std::mutex> lock(m_planner_msg_mutex);
        message = m_planner_message;
    }
    emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error, QString::fromStdString(message));
}

void MasterNode::setPlannerMessage(const std::string &message)
{
    std::lock_guard<std::mutex> lock(m_planner_msg_mutex);
    m_planner_message = message;
}

void MasterNode::handle_planner_response(const rclcpp::Client<interfaces::srv::Planner>::SharedFuture future, const bool closed_loop)
{
    auto response = future.get();
    const double round_trip_ms = (this->now().nanoseconds() - m_planning_started_ns.load()) / 1.0e6;

    RCLCPP_INFO(this->get_logger(),
                "[plan] generateTrajectory response #%d: success=%s message=\"%s\" ik_error=%.3f mm round_trip=%.1f ms",
                m_planner_request_seq.load(), response->success ? "true" : "false", response->message.c_str(),
                response->value * 1000.0, round_trip_ms);

    m_planner_success = response->success;
    m_planner_ik_error = response->value;

    auto fmt2 = [](double v)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << v;
        return oss.str();
    };

    if (response->success)
    {
        if (loadPlannedPath())
        {
            {
                std::lock_guard<std::mutex> lock(m_force_mutex);
                m_f_at_plan = m_f_pending;
                m_f_at_plan_valid = true;
                m_replan_attempts = 0;
                m_replan_backoff_s = m_replan_cooldown_s;
            }
            size_t waypoints = 0;
            {
                std::lock_guard<std::mutex> lock(m_deploy_mutex);
                waypoints = m_q_list_adjusted.size();
            }
            setPlannerMessage("accepted: ik_error=" + fmt2(response->value * 1000.0) + " mm, " + std::to_string(waypoints) + " waypoints - " + response->message);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[plan] Failed to load plannedPath.csv - path will be empty");
            m_planner_success = false;
            setPlannerMessage("accepted by planner but plannedPath.csv failed to load");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "[plan] plan rejected: planner reported failure: %s", response->message.c_str());
        setPlannerMessage("rejected: " + response->message);

        // Select Target (open-loop) rejection: the previous plan targets a point the
        // operator has moved away from. Unless an insertion along it is outstanding (it
        // is then needed for retract), discard it so Auto Insert cannot drive to the old
        // target, and clear its drawn path. A closed-loop re-plan rejection keeps the
        // active plan: it is often transient and is re-requested as q or the target moves.
        // m_plan_needed stays false: it would re-request the same rejected target.
        bool discarded = false;
        if (!closed_loop)
        {
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (!manager_deploy::insertionOutstanding(m_q_list_adjusted.size(), m_current_config_index, m_q_list_actuated.size()))
            {
                m_q_list_adjusted.clear();
                m_q_list_actuated.clear();
                m_current_config_index = 0;
                m_waypoint_wait_index = -1;
                discarded = true;
            }
        }
        if (discarded)
        {
            invalidateForceBaseline();
            RCLCPP_INFO(this->get_logger(), "[plan] rejected target: previous unexecuted plan discarded and displayed path cleared");
            auto request = std::make_shared<interfaces::srv::Planner::Request>();
            request->command = "clearPath";
            (void)m_planner_client->async_send_request(
                request, [this](rclcpp::Client<interfaces::srv::Planner>::SharedFuture clear_future)
                {
                    RCLCPP_INFO(this->get_logger(), "[plan] clearPath response: success=%s",
                                clear_future.get()->success ? "true" : "false");
                });
        }
    }

    std::this_thread::sleep_for(10ms);

    m_flag_planning = false;
    emitPlannerStatus();
}

// Replan responses are handled separately from full-plan responses: `value`
// carries the FTL swept cost of the chosen schedule, not an IK error, and a
// rejected replan must not overwrite the full-plan status consumed by the GUI
// and the test state machine. On acceptance the replanned tail is spliced
// onto the executed prefix (spliceReplannedPath), not swapped in wholesale,
// so retract still replays the waypoints the robot actually passed through.
void MasterNode::handle_replan_response(const rclcpp::Client<interfaces::srv::Planner>::SharedFuture future)
{
    auto response = future.get();
    const double round_trip_ms = (this->now().nanoseconds() - m_planning_started_ns.load()) / 1.0e6;

    if (!response->success && response->message == ctr_common::planner_protocol::kReplanForceUnchanged)
    {
        // Not a rejection: the planner would have replanned under the same force as the
        // active plan. Re-baseline the drift so the same EKF delta does not re-trigger.
        {
            std::lock_guard<std::mutex> lock(m_force_mutex);
            m_f_at_plan = m_f_pending;
            m_f_at_plan_valid = true;
            m_replan_attempts = 0;
            m_replan_backoff_s = m_replan_cooldown_s;
        }
        RCLCPP_INFO(this->get_logger(),
                    "[plan] replanDeployment response #%d: skipped (planning force unchanged) - continuing current plan",
                    m_planner_request_seq.load());
        setPlannerMessage("replan skipped: planning force unchanged");
    }
    else if (response->success && spliceReplannedPath())
    {
        std::lock_guard<std::mutex> lock(m_force_mutex);
        m_f_at_plan = m_f_pending;
        m_f_at_plan_valid = true;
        m_replan_attempts = 0;
        m_replan_backoff_s = m_replan_cooldown_s;
        RCLCPP_INFO(this->get_logger(),
                    "[plan] replanDeployment response #%d: accepted: %s (FTL cost %.6f, round_trip=%.1f ms) - resuming deployment on the new schedule",
                    m_planner_request_seq.load(), response->message.c_str(), response->value, round_trip_ms);
        setPlannerMessage("replan accepted: " + response->message);
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
                    "[plan] replanDeployment response #%d: rejected (%s, round_trip=%.1f ms) - continuing previous plan (attempt %d/%d, next in >= %.1f s)",
                    m_planner_request_seq.load(), response->message.c_str(), round_trip_ms, m_replan_attempts, k_max_replan_attempts, m_replan_backoff_s);
        setPlannerMessage("replan rejected: " + response->message);
    }

    m_flag_planning = false;
    emitPlannerStatus();
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
            return;
        df = (m_f_est - m_f_at_plan).norm();

        // Hysteresis re-arm: once the drift recovers to half the trigger
        // threshold, a previous string of rejections is forgiven.
        if (m_replan_attempts >= k_max_replan_attempts && df <= 0.5 * m_force_replan_threshold)
        {
            m_replan_attempts = 0;
            m_replan_backoff_s = m_replan_cooldown_s;
            RCLCPP_INFO(this->get_logger(), "[comm] Force drift recovered (%.3f N) - replanning re-armed", df);
        }
    }
    if (df <= m_force_replan_threshold)
        return;
    {
        std::lock_guard<std::mutex> lock(m_force_mutex);
        if (m_replan_attempts >= k_max_replan_attempts)
            return; // suppressed until hysteresis re-arm or an accepted plan
        if ((this->now() - m_last_replan_request_time).seconds() < m_replan_backoff_s)
            return;
        m_f_pending = m_f_est;
    }

    m_flag_planning = true; // pauses every deployment branch until the response arrives
    m_last_replan_request_time = this->now();
    m_replan_count++;
    const int seq = m_planner_request_seq.fetch_add(1) + 1;
    const int64_t send_ns = m_last_replan_request_time.nanoseconds();
    m_planning_started_ns.store(send_ns);
    m_planning_timeout_logged = false;

    auto request = std::make_shared<interfaces::srv::Planner::Request>();
    request->command = "replanDeployment";

    auto response_callback = std::bind(&MasterNode::handle_replan_response, this, std::placeholders::_1);
    (void)m_planner_client->async_send_request(request, response_callback);

    emitPlannerStatus();
    RCLCPP_INFO(this->get_logger(),
                "[comm] planner/command replanDeployment request #%d sent: |df|=%.3f N at waypoint %d (%zu remaining) - pausing deployment to replan (#%d)",
                seq, df, index, remaining, m_replan_count);

    if (m_test_running)
    {
        captureRecordingFrame("data"); // event marker; force columns are already recorded
    }
}

void MasterNode::control_loop()
{
    if (!m_services_ready)
        return; // robot/planner/recorder services not up yet

    // Diagnostic-only watchdog: a stuck planner/command request (generateTrajectory
    // or replanDeployment) never resets or retries anything here, it only makes a
    // silently-hung request visible in the log.
    if (m_flag_planning && !m_planning_timeout_logged)
    {
        const double elapsed = (this->now().nanoseconds() - m_planning_started_ns.load()) / 1.0e9;
        if (elapsed > m_planner_timeout_s)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "[comm] planner/command response timeout: request #%d outstanding for %.1f s (timeout=%.1f s)",
                         m_planner_request_seq.load(), elapsed, m_planner_timeout_s);
            m_planning_timeout_logged = true;
        }
    }

    // Update automated test state machine
    updateTestStateMachine();

    // Snapshot the cross-thread feedback state once per cycle
    Eigen::Vector3d X, Xd;
    blaze::StaticVector<double, 4UL> q_snapshot;
    {
        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        X = m_X;
        Xd = m_Xd;
        q_snapshot = m_q;
    }

    // Diagnostic-only: a tube-rotation-alignment episode only spans a single
    // continuous run of the Planner branch below; leaving it (Deployment, None,
    // or procedure dropping out) must start a fresh episode next time around.
    if (m_high_level_mode != HighLvlCtrMode::Planner && !m_planner_branch_reason_prev.empty())
    {
        m_planner_branch_reason_prev.clear();
        m_alignment_start_diff = 0.0;
    }

    // Select Target must not re-target while an executed insertion is outstanding:
    // retract replays it back to its start; Go Home discards it.
    bool planner_insertion_outstanding = false;
    if (m_procedure && m_high_level_mode == HighLvlCtrMode::Planner)
    {
        std::lock_guard<std::mutex> lock(m_deploy_mutex);
        planner_insertion_outstanding = manager_deploy::insertionOutstanding(m_q_list_adjusted.size(), m_current_config_index, m_q_list_actuated.size());
    }

    if (m_high_level_mode == HighLvlCtrMode::Planner && !m_procedure)
    {
        if (m_planner_branch_reason_prev != "procedure inactive")
        {
            RCLCPP_INFO(get_logger(), "[procedure] not calling planner: procedure inactive");
            m_planner_branch_reason_prev = "procedure inactive";
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Planner && planner_insertion_outstanding)
    {
        const std::string reason = "insertion outstanding: retract to the plan start or Go Home first";
        if (m_planner_branch_reason_prev != reason)
        {
            RCLCPP_INFO(get_logger(), "[procedure] not calling planner: %s", reason.c_str());
            m_planner_branch_reason_prev = reason;
            setPlannerMessage(reason);
            emitPlannerStatus();
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Planner)
    {
        // Compute target angle in x-y plane
        double target_theta = atan2(Xd[1], Xd[0]) + M_PI / 2.0;

        // Normalize target_theta to be between -π and π
        if (target_theta > M_PI)
        {
            target_theta -= 2.0 * M_PI;
        }
        else if (target_theta < -M_PI)
        {
            target_theta += 2.0 * M_PI;
        }

        // std::cout << "planner target: " << Xd.transpose() << ", theta: " << target_theta << std::endl;

        double tube_1_theta_diff = std::abs(target_theta - m_q[0]);
        double tube_2_theta_diff = std::abs(target_theta - m_q[2]);
        bool target_changed = (Xd - m_Xd_prev).norm() > k_target_threshold;
        bool q_changed = blaze::norm((m_q - m_q_prev) / k_input_scale) > k_q_threshold;

        // If tube angles differ from target, command robot to updated angles
        if (tube_1_theta_diff > k_theta_threshold || tube_2_theta_diff > k_theta_threshold)
        {
            blaze::StaticVector<double, 6> q = blaze::StaticVector<double, 6>({m_q[1], m_q[3], 0.0, target_theta, target_theta, 0.0});
            publish_position(q);
            // std::cout << "tube_1_theta_diff: " << tube_1_theta_diff << ", tube_2_theta_diff: " << tube_2_theta_diff << std::endl;

            // Diagnostics: rotation-alignment is the step that silently stalled in
            // the field (m_flag_manual gating joint_space/target) - make its
            // progress and convergence visible.
            const double max_diff = std::max(tube_1_theta_diff, tube_2_theta_diff);
            if (m_planner_branch_reason_prev != "aligning tubes")
            {
                m_alignment_start_time = this->now();
                m_alignment_start_diff = max_diff;
                RCLCPP_INFO(get_logger(),
                            "[procedure] not calling planner: aligning tubes: theta_target=%.4f alpha1=%.4f alpha2=%.4f threshold=%.4f",
                            target_theta, q_snapshot[0], q_snapshot[2], k_theta_threshold);
                m_planner_branch_reason_prev = "aligning tubes";
            }
            const double elapsed = (this->now() - m_alignment_start_time).seconds();
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                                 "[procedure] aligning tubes: elapsed=%.1f s theta_target=%.4f alpha1=%.4f alpha2=%.4f diff1=%.4f diff2=%.4f threshold=%.4f",
                                 elapsed, target_theta, q_snapshot[0], q_snapshot[2], tube_1_theta_diff, tube_2_theta_diff, k_theta_threshold);
            if (elapsed > 10.0 && max_diff >= 0.9 * m_alignment_start_diff)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                     "[procedure] tube alignment not converging after %.1f s: diff1=%.4f diff2=%.4f (started at %.4f) threshold=%.4f - robot may not be rotating",
                                     elapsed, tube_1_theta_diff, tube_2_theta_diff, m_alignment_start_diff, k_theta_threshold);
            }
        }
        // If target position changed significantly, call planner to generate new path
        else
        {
            // std::cout << "m_reached: " << m_reached << ", !m_flag_planning: " << (!m_flag_planning) << ", target_changed: " << target_changed << ", q_changed: " << q_changed << std::endl;
            if (m_reached && (!m_flag_planning) && (target_changed || q_changed || m_plan_needed))
            {
                {
                    std::lock_guard<std::mutex> lock(m_force_mutex);
                    m_f_pending = m_f_est; // baseline promoted to m_f_at_plan when the plan is accepted
                }

                // Capture the request's identity/send-time BEFORE async_send_request:
                // the response can arrive on another executor thread before this
                // function returns.
                const int seq = m_planner_request_seq.fetch_add(1) + 1;
                const int64_t send_ns = this->now().nanoseconds();
                m_planning_started_ns.store(send_ns);
                m_planning_timeout_logged = false;

                auto request = std::make_shared<interfaces::srv::Planner::Request>();
                request->command = "generateTrajectory";
                request->value[0] = Xd[0];
                request->value[1] = Xd[1];
                request->value[2] = Xd[2];

                auto response_callback = [this](rclcpp::Client<interfaces::srv::Planner>::SharedFuture f) { handle_planner_response(f, false); };
                auto future_result = m_planner_client->async_send_request(request, response_callback);

                m_Xd_prev = Xd;
                m_q_prev = m_q;
                m_flag_planning = true;
                m_plan_needed = false;
                m_flag_planner_updated = true;
                emitPlannerStatus();
                RCLCPP_INFO(this->get_logger(),
                            "[comm] planner/command generateTrajectory request #%d sent at %.3f: target=[%.4f, %.4f, %.4f] "
                            "q(wire) alpha1=%.4f beta1=%.4f alpha2=%.4f beta2=%.4f alpha_diff1=%.4f alpha_diff2=%.4f",
                            seq, send_ns / 1.0e9, Xd[0], Xd[1], Xd[2],
                            q_snapshot[0], q_snapshot[1], q_snapshot[2], q_snapshot[3], tube_1_theta_diff, tube_2_theta_diff);
                m_planner_branch_reason_prev.clear();
            }
            else
            {
                std::string reason;
                if (!m_reached)
                    reason = "waiting for reached";
                else if (m_flag_planning)
                    reason = "waiting for planner response";
                else
                    reason = "target/q unchanged";

                if (reason != m_planner_branch_reason_prev)
                {
                    RCLCPP_INFO(get_logger(), "[procedure] not calling planner: %s", reason.c_str());
                    m_planner_branch_reason_prev = reason;
                }
            }
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Deployment && !m_closed_loop_enabled)
    {
        maybeRequestDeploymentReplan();

        const bool inserting_now = !m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert);
        const bool retracting_now = !m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract);
        int snapshot_index;
        size_t snapshot_adjusted_size;

        // Physics-order measured config for the stale-plan/retract tolerance checks below
        // (same wire->physics mapping publish_position uses).
        const blaze::StaticVector<double, 6> measured({q_snapshot[1], q_snapshot[3], 0.0, q_snapshot[0], q_snapshot[2], 0.0});
        {
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            snapshot_index = m_current_config_index;
            snapshot_adjusted_size = m_q_list_adjusted.size();
        }
        if (inserting_now && !m_insert_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] insertion started at waypoint %d/%zu", snapshot_index + 1, snapshot_adjusted_size);
        else if (!inserting_now && m_insert_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] insertion stopped at waypoint %d/%zu", snapshot_index + 1, snapshot_adjusted_size);
        m_insert_active_prev = inserting_now;
        if (retracting_now && !m_retract_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] retraction started at waypoint %d/%zu", snapshot_index + 1, snapshot_adjusted_size);
        else if (!retracting_now && m_retract_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] retraction stopped at waypoint %d/%zu", snapshot_index + 1, snapshot_adjusted_size);
        if (!inserting_now)
            m_insert_refused = false;
        if (!retracting_now)
        {
            m_retract_checked = false;
            m_retract_refused = false;
        }
        m_retract_active_prev = retracting_now;

        if (!m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert))
        {
            m_retracting = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_q_list_adjusted.empty())
            {
                m_auto_insert = false;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[cmd] no accepted plan: select a reachable target before inserting");
            }
            else if (getReachStatus())
            {
                m_waypoint_wait_index = -1;
                if (m_current_config_index == 0 && !m_q_list_adjusted.empty() &&
                    !manager_deploy::configMatches(m_q_list_adjusted[0], measured, k_plan_match_beta_tol, k_theta_threshold))
                {
                    m_auto_insert = false;
                    m_plan_needed = true;
                    const auto &plan_start = m_q_list_adjusted[0];
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "[cmd] insertion refused: measured q does not match the plan start (beta1 %.4f vs %.4f, beta2 %.4f vs %.4f, alpha1 %.4f vs %.4f, alpha2 %.4f vs %.4f) - re-plan in Select Target mode",
                                         measured[0], plan_start[0], measured[1], plan_start[1], measured[3], plan_start[3], measured[4], plan_start[4]);
                    if (!m_insert_refused)
                    {
                        setPlannerMessage("insertion refused: robot is not at the plan start - re-plan");
                        emitPlannerStatus();
                        m_insert_refused = true;
                    }
                }
                else
                {
                    m_current_config_index += 1;
                    if (m_current_config_index < m_q_list_adjusted.size())
                    {
                        // Capture frame in session A during auto-insertion
                        if (m_test_running && m_auto_insert)
                        {
                            captureRecordingFrame("data");
                        }

                        publish_position(m_q_list_adjusted[m_current_config_index]);

                        const auto wire = ctr_common::physicsToWire(m_q_list_adjusted[m_current_config_index]);
                        RCLCPP_DEBUG(this->get_logger(),
                                     "[cmd] Sent q [%d/%zu]: physics beta1=%.3f beta2=%.3f alpha1=%.3f alpha2=%.3f, "
                                     "wire alpha1=%.3f beta1=%.3f alpha2=%.3f beta2=%.3f",
                                     m_current_config_index + 1, m_q_list_adjusted.size(),
                                     m_q_list_adjusted[m_current_config_index][0],
                                     m_q_list_adjusted[m_current_config_index][1],
                                     m_q_list_adjusted[m_current_config_index][3],
                                     m_q_list_adjusted[m_current_config_index][4],
                                     wire[0], wire[1], wire[2], wire[3]);
                        m_insert_complete_logged = false;
                    }
                    else
                    {
                        m_auto_insert = false;
                        m_current_config_index = m_q_list_adjusted.size() - 1;
                        if (!m_insert_complete_logged)
                        {
                            RCLCPP_INFO(this->get_logger(), "[cmd] insertion complete: max waypoint %d/%zu reached",
                                        m_current_config_index + 1, m_q_list_adjusted.size());
                            m_insert_complete_logged = true;
                        }
                    }
                }
            }
            else
            {
                logWaypointWaitWarn(m_current_config_index, m_q_list_adjusted[m_current_config_index], q_snapshot);
            }
        }
        else if (!m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract))
        {
            m_retracting = true;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_q_list_adjusted.empty())
            {
                RCLCPP_WARN(this->get_logger(), "[cmd] Adjusted deployment list is empty; cannot send positions.");
            }

            if (getReachStatus())
            {
                m_waypoint_wait_index = -1;
                // Checked once per retract episode, at the first settled cycle; a refusal stays
                // latched until the retract command is released, so a held button cannot step.
                if (m_retract_refused ||
                    (!m_retract_checked && m_current_config_index > 0 &&
                     static_cast<size_t>(m_current_config_index) < m_q_list_adjusted.size() &&
                     !manager_deploy::configMatches(m_q_list_adjusted[m_current_config_index], measured, k_plan_match_beta_tol, k_theta_threshold)))
                {
                    m_auto_retract = false;
                    if (!m_retract_refused)
                    {
                        const auto &at_waypoint = m_q_list_adjusted[m_current_config_index];
                        RCLCPP_WARN(this->get_logger(),
                                    "[cmd] retraction refused: robot is not on the executed trajectory at waypoint %d (beta1 %.4f vs %.4f, beta2 %.4f vs %.4f, alpha1 %.4f vs %.4f, alpha2 %.4f vs %.4f) - use Go Home",
                                    m_current_config_index, measured[0], at_waypoint[0], measured[1], at_waypoint[1], measured[3], at_waypoint[3], measured[4], at_waypoint[4]);
                        setPlannerMessage("retraction refused: robot is off the executed trajectory - use Go Home");
                        emitPlannerStatus();
                        m_retract_refused = true;
                    }
                }
                else
                {
                    m_retract_checked = true;
                    m_current_config_index--;
                    if (m_current_config_index >= 0)
                    {
                        publish_position(m_q_list_adjusted[m_current_config_index]);

                        const auto wire = ctr_common::physicsToWire(m_q_list_adjusted[m_current_config_index]);
                        RCLCPP_DEBUG(this->get_logger(),
                                     "[cmd] Sent q [%d/%zu]: physics beta1=%.3f beta2=%.3f alpha1=%.3f alpha2=%.3f, "
                                     "wire alpha1=%.3f beta1=%.3f alpha2=%.3f beta2=%.3f",
                                     m_current_config_index + 1, m_q_list_adjusted.size(),
                                     m_q_list_adjusted[m_current_config_index][0],
                                     m_q_list_adjusted[m_current_config_index][1],
                                     m_q_list_adjusted[m_current_config_index][3],
                                     m_q_list_adjusted[m_current_config_index][4],
                                     wire[0], wire[1], wire[2], wire[3]);
                        m_retract_complete_logged = false;
                    }
                    else
                    {
                        m_auto_retract = false;
                        if (!m_retract_complete_logged)
                        {
                            if (!m_q_list_adjusted.empty())
                            {
                                const auto &plan_start = m_q_list_adjusted[0];
                                RCLCPP_INFO(this->get_logger(),
                                            "[cmd] retraction complete: plan start reached (physics beta1=%.4f beta2=%.4f alpha1=%.4f alpha2=%.4f)",
                                            plan_start[0], plan_start[1], plan_start[3], plan_start[4]);
                            }
                            m_retract_complete_logged = true;
                        }
                        m_current_config_index = 0;
                        m_q_list_adjusted.clear();
                        invalidateForceBaseline();
                        m_plan_needed = true;
                    }
                }
            }
            else if (m_current_config_index >= 0 && static_cast<size_t>(m_current_config_index) < m_q_list_adjusted.size())
            {
                logWaypointWaitWarn(m_current_config_index, m_q_list_adjusted[m_current_config_index], q_snapshot);
            }
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Deployment && m_closed_loop_enabled)
    {
        // No Xd - (X - Xsim) target shift: Xsim = PINN(q, f_est) and the EKF fits f_est
        // so that Xsim tracks X, so the shift is ~0 by construction. The model mismatch is
        // corrected inside the planner instead (generateTrajectoryClosedLoop plans with f_est).
        bool target_changed = (Xd - m_Xd_closed_loop_prev).norm() > k_target_threshold;
        bool q_changed = blaze::norm((m_q - m_q_prev) / k_input_scale) > k_q_threshold;
        double Xe = (X - Xd).norm();

        // Call planner if target changed significantly
        if (m_reached && !m_flag_planning && (target_changed || q_changed) && !m_retracting && (Xe > (k_target_threshold / 2)))
        {
            m_flag_planning = true;
            emitPlannerStatus();

            {
                std::lock_guard<std::mutex> lock(m_force_mutex);
                m_f_pending = m_f_est; // baseline promoted to m_f_at_plan when the plan is accepted
            }

            // Capture the request's identity/send-time BEFORE async_send_request:
            // the response can arrive on another executor thread before this
            // function returns.
            const int seq = m_planner_request_seq.fetch_add(1) + 1;
            const int64_t send_ns = this->now().nanoseconds();
            m_planning_started_ns.store(send_ns);
            m_planning_timeout_logged = false;

            auto request = std::make_shared<interfaces::srv::Planner::Request>();
            request->command = "generateTrajectoryClosedLoop";
            request->value[0] = Xd[0];
            request->value[1] = Xd[1];
            request->value[2] = Xd[2];

            auto response_callback = [this](rclcpp::Client<interfaces::srv::Planner>::SharedFuture f) { handle_planner_response(f, true); };
            auto future_result = m_planner_client->async_send_request(request, response_callback);
            RCLCPP_INFO(this->get_logger(),
                        "[comm] planner/command generateTrajectoryClosedLoop request #%d sent at %.3f: "
                        "target=[%.4f, %.4f, %.4f] q(wire) alpha1=%.4f beta1=%.4f alpha2=%.4f beta2=%.4f",
                        seq, send_ns / 1.0e9, Xd[0], Xd[1], Xd[2],
                        q_snapshot[0], q_snapshot[1], q_snapshot[2], q_snapshot[3]);

            m_Xd_closed_loop_prev = Xd;
            m_q_prev = m_q;

            std::this_thread::sleep_for(10ms);
        }

        const bool inserting_now = !m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert);
        const bool retracting_now = !m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract);
        int snapshot_index;
        size_t snapshot_adjusted_size, snapshot_actuated_size;
        {
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            snapshot_index = m_current_config_index;
            snapshot_adjusted_size = m_q_list_adjusted.size();
            snapshot_actuated_size = m_q_list_actuated.size();
        }
        if (inserting_now && !m_insert_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] insertion started at waypoint %d/%zu", snapshot_index + 1, snapshot_adjusted_size);
        else if (!inserting_now && m_insert_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] insertion stopped at waypoint %d/%zu", snapshot_index + 1, snapshot_adjusted_size);
        m_insert_active_prev = inserting_now;
        if (retracting_now && !m_retract_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] retraction started at waypoint %zu actuated", snapshot_actuated_size);
        else if (!retracting_now && m_retract_active_prev)
            RCLCPP_INFO(get_logger(), "[cmd] retraction stopped at waypoint %zu actuated", snapshot_actuated_size);
        m_retract_active_prev = retracting_now;

        // Handle manual and auto insertion
        if (!m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert))
        {
            m_retracting = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_q_list_adjusted.empty())
            {
                RCLCPP_WARN(this->get_logger(), "[cmd] Adjusted deployment list is empty; cannot send positions.");
            }

            if (getReachStatus())
            {
                m_waypoint_wait_index = -1;
                if (m_q_list_actuated.empty())
                {
                    blaze::StaticVector<double, 6> q = blaze::StaticVector<double, 6>({m_q[1], m_q[3], 0.0, m_q[0], m_q[2], 0.0});
                    m_q_list_actuated.push_back(q);
                }

                m_current_config_index += 1;
                if (m_current_config_index < m_q_list_adjusted.size())
                {
                    // Capture frame in session A during auto-insertion
                    if (m_test_running && m_auto_insert)
                    {
                        captureRecordingFrame("data");
                    }

                    publish_position(m_q_list_adjusted[m_current_config_index]);

                    const auto wire = ctr_common::physicsToWire(m_q_list_adjusted[m_current_config_index]);
                    RCLCPP_DEBUG(this->get_logger(),
                                 "[cmd] Sent q [%d/%zu]: physics beta1=%.3f beta2=%.3f alpha1=%.3f alpha2=%.3f, "
                                 "wire alpha1=%.3f beta1=%.3f alpha2=%.3f beta2=%.3f",
                                 m_current_config_index + 1, m_q_list_adjusted.size(),
                                 m_q_list_adjusted[m_current_config_index][0],
                                 m_q_list_adjusted[m_current_config_index][1],
                                 m_q_list_adjusted[m_current_config_index][3],
                                 m_q_list_adjusted[m_current_config_index][4],
                                 wire[0], wire[1], wire[2], wire[3]);

                    m_q_list_actuated.push_back(m_q_list_adjusted[m_current_config_index]);
                    m_insert_complete_logged = false;
                }
                else
                {
                    m_auto_insert = false;
                    m_q_list_adjusted.clear();
                    invalidateForceBaseline();
                    m_current_config_index = 0;
                    if (!m_insert_complete_logged)
                    {
                        RCLCPP_INFO(this->get_logger(), "[cmd] insertion complete: max deployment index reached");
                        m_insert_complete_logged = true;
                    }
                }
            }
            else if (!m_q_list_adjusted.empty() && m_current_config_index >= 0 &&
                     static_cast<size_t>(m_current_config_index) < m_q_list_adjusted.size())
            {
                logWaypointWaitWarn(m_current_config_index, m_q_list_adjusted[m_current_config_index], q_snapshot);
            }
        }
        // Handle manual and auto retraction
        else if (!m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract))
        {
            m_retracting = true;
            m_auto_insert = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);

            if (getReachStatus())
            {
                m_waypoint_wait_index = -1;
                if (m_q_list_actuated.size() > 1)
                {
                    m_q_list_actuated.pop_back();
                    publish_position(m_q_list_actuated.back());
                    const auto wire = ctr_common::physicsToWire(m_q_list_actuated.back());
                    RCLCPP_DEBUG(this->get_logger(),
                                 "[cmd] Sent q [%zu/%zu]: physics beta1=%.3f beta2=%.3f alpha1=%.3f alpha2=%.3f, "
                                 "wire alpha1=%.3f beta1=%.3f alpha2=%.3f beta2=%.3f",
                                 m_q_list_actuated.size() + 1, m_q_list_actuated.size() + 2,
                                 m_q_list_actuated.back()[0], m_q_list_actuated.back()[1],
                                 m_q_list_actuated.back()[3], m_q_list_actuated.back()[4],
                                 wire[0], wire[1], wire[2], wire[3]);
                    m_retract_complete_logged = false;
                }
                else
                {
                    m_auto_retract = false;
                    m_plan_needed = true;
                    m_q_list_actuated.clear();
                    m_q_list_adjusted.clear();
                    invalidateForceBaseline();

                    // Delete plannedPath.csv
                    std::filesystem::path file_path =
                        ctr_common::resolveDataRoot(*this, "manager") / "Shared_Files" / "plannedPath.csv";
                    
                    if (std::filesystem::exists(file_path))
                    {
                        std::filesystem::remove(file_path);
                        RCLCPP_INFO(this->get_logger(), "[cmd] Deleted plannedPath.csv");
                    }
                    
                    if (!m_retract_complete_logged)
                    {
                        RCLCPP_INFO(this->get_logger(), "[cmd] retraction complete: min deployment index reached (%zu actuated)", m_q_list_actuated.size());
                        m_retract_complete_logged = true;
                    }
                }
            }
            else if (!m_q_list_actuated.empty())
            {
                logWaypointWaitWarn(static_cast<int>(m_q_list_actuated.size()), m_q_list_actuated.back(), q_snapshot);
            }
        }
    }
}

void MasterNode::publish_position(const blaze::StaticVector<double, 6> &q)
{
    interfaces::msg::Jointspace msg;
    msg.position = ctr_common::physicsToWire(q);
    RCLCPP_DEBUG(this->get_logger(), "[cmd] publish_position wire: alpha1=%.4f beta1=%.4f alpha2=%.4f beta2=%.4f",
                 msg.position[0], msg.position[1], msg.position[2], msg.position[3]);
    m_pub_joint_targ->publish(msg);
}

// Diagnostic-only: WARN (throttled) when the deployment loop has been waiting
// on "reached" at the same waypoint for more than 5 s. Tracks the wait start
// time keyed by `index` so it resets whenever the commanded waypoint changes.
void MasterNode::logWaypointWaitWarn(int index, const blaze::StaticVector<double, 6> &target,
                                      const blaze::StaticVector<double, 4UL> &q_feedback)
{
    if (index != m_waypoint_wait_index)
    {
        m_waypoint_wait_index = index;
        m_waypoint_wait_start_time = this->now();
        return;
    }
    const double elapsed = (this->now() - m_waypoint_wait_start_time).seconds();
    if (elapsed > 5.0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[cmd] waiting for 'reached' at waypoint %d for %.1f s: target(physics) beta1=%.4f beta2=%.4f "
                             "alpha1=%.4f alpha2=%.4f, feedback(wire) alpha1=%.4f beta1=%.4f alpha2=%.4f beta2=%.4f",
                             index, elapsed, target[0], target[1], target[3], target[4],
                             q_feedback[0], q_feedback[1], q_feedback[2], q_feedback[3]);
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

// Load the planner's CSV output and swap it in as the active deployment list.
bool MasterNode::loadPlannedPath()
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
            RCLCPP_INFO(this->get_logger(), "[traj] plannedPath.csv loaded (written %.1f s ago)", age / 1000.0);
        }

        m_q_list_adjusted = ctr_common::downsamplePath(m_q_list, ctr_common::kWaypointBetaStep, ctr_common::kWaypointAlphaStep);
        m_current_config_index = 0;
        m_waypoint_wait_index = -1;
        m_insert_complete_logged = false;
        m_retract_complete_logged = false;

        RCLCPP_INFO(get_logger(), "[traj] %zu rows loaded, %zu rows after step-size adjustment (beta step=%.4f m, alpha step=%.4f rad)",
                    m_q_list.size(), m_q_list_adjusted.size(), ctr_common::kWaypointBetaStep, ctr_common::kWaypointAlphaStep);

        if (!m_q_list_adjusted.empty())
        {
            const auto &first = m_q_list_adjusted.front();
            const auto &last = m_q_list_adjusted.back();
            RCLCPP_INFO(get_logger(),
                        "[traj] first waypoint (physics): beta1=%.4f beta2=%.4f beta3=%.4f alpha1=%.4f alpha2=%.4f alpha3=%.4f",
                        first[0], first[1], first[2], first[3], first[4], first[5]);
            RCLCPP_INFO(get_logger(),
                        "[traj] last waypoint (physics): beta1=%.4f beta2=%.4f beta3=%.4f alpha1=%.4f alpha2=%.4f alpha3=%.4f",
                        last[0], last[1], last[2], last[3], last[4], last[5]);

            blaze::StaticVector<double, 6> q_min = m_q_list_adjusted.front();
            blaze::StaticVector<double, 6> q_max = m_q_list_adjusted.front();
            bool any_out_of_limits = false;
            for (const auto &q : m_q_list_adjusted)
            {
                for (size_t i = 0; i < 6; ++i)
                {
                    q_min[i] = std::min(q_min[i], q[i]);
                    q_max[i] = std::max(q_max[i], q[i]);
                }
                const auto wire = ctr_common::physicsToWire(q);
                for (size_t i = 0; i < 4; ++i)
                {
                    if (wire[i] < m_minCurrentPosLimit[i] || wire[i] > m_maxCurrentPosLimit[i])
                    {
                        any_out_of_limits = true;
                    }
                }
            }
            RCLCPP_INFO(get_logger(),
                        "[traj] per-joint range (physics): beta1=[%.4f,%.4f] beta2=[%.4f,%.4f] alpha1=[%.4f,%.4f] alpha2=[%.4f,%.4f]",
                        q_min[0], q_max[0], q_min[1], q_max[1], q_min[3], q_max[3], q_min[4], q_max[4]);
            if (any_out_of_limits)
            {
                RCLCPP_WARN(get_logger(),
                            "[traj] one or more waypoints fall outside the current limits snapshot (wire order "
                            "alpha1,beta1,alpha2,beta2) min=[%.4f,%.4f,%.4f,%.4f] max=[%.4f,%.4f,%.4f,%.4f]; "
                            "translation limits are dynamic/coupled, this is only a snapshot",
                            m_minCurrentPosLimit[0], m_minCurrentPosLimit[1], m_minCurrentPosLimit[2], m_minCurrentPosLimit[3],
                            m_maxCurrentPosLimit[0], m_maxCurrentPosLimit[1], m_maxCurrentPosLimit[2], m_maxCurrentPosLimit[3]);
            }
        }

        return true;
    }
    return false;
}

// Splice an accepted replanDeployment response onto the executed trajectory
// instead of replacing it: the CSV only covers the remaining deployment from
// q_now, so swapping it in wholesale (like loadPlannedPath) would make a
// subsequent retract stop at the replan's start instead of the original plan
// start. Rejects (and keeps the old list) if the new tail does not start at
// the waypoint the robot is currently on.
bool MasterNode::spliceReplannedPath()
{
    std::vector<blaze::StaticVector<double, 6>> local;
    if (!read_path_from_csv(local, "plannedPath.csv") || local.empty())
    {
        return false;
    }
    const auto adjusted = ctr_common::downsamplePath(local, ctr_common::kWaypointBetaStep, ctr_common::kWaypointAlphaStep);

    std::lock_guard<std::mutex> lock(m_deploy_mutex);
    auto spliced = manager_deploy::spliceReplan(m_q_list_adjusted, m_current_config_index, adjusted,
                                                 k_plan_match_beta_tol, k_theta_threshold);
    if (!spliced)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[traj] replanned path rejected: does not start at current waypoint %d of %zu (replan start beta1=%.4f beta2=%.4f alpha1=%.4f alpha2=%.4f)",
                     m_current_config_index, m_q_list_adjusted.size(),
                     adjusted.front()[0], adjusted.front()[1], adjusted.front()[3], adjusted.front()[4]);
        return false;
    }

    const int index = m_current_config_index;
    m_q_list_adjusted = std::move(*spliced);
    m_waypoint_wait_index = -1;
    m_insert_complete_logged = false;
    RCLCPP_INFO(this->get_logger(),
                "[traj] replan spliced at waypoint %d: %d executed + %zu new = %zu waypoints",
                index, index + 1, adjusted.size() - 1, m_q_list_adjusted.size());
    return true;
}

// Homing (goHome / findRotaryHomeAndGoHome / findLinearHome) physically moves
// the robot back to its reference pose; any executed trajectory the deployment
// loop was tracking is now stale. Discard it so a later Auto Retract does not
// try to replay the pre-homing path, and force the next Select-Target request
// so the operator gets a fresh plan from the homed configuration.
void MasterNode::resetDeploymentState(const std::string &reason)
{
    {
        std::lock_guard<std::mutex> lock(m_deploy_mutex);
        m_q_list_adjusted.clear();
        m_q_list_actuated.clear();
        m_current_config_index = 0;
        m_waypoint_wait_index = -1;
    }
    m_auto_insert = false;
    m_auto_retract = false;
    m_plan_needed = true;
    invalidateForceBaseline();
    RCLCPP_INFO(this->get_logger(), "[cmd] deployment state reset after '%s': executed trajectory discarded", reason.c_str());
}

bool MasterNode::read_path_from_csv(std::vector<blaze::StaticVector<double, 6>> &init_q_list, const std::string &fileName)
{
    const std::filesystem::path file_path =
        ctr_common::resolveDataRoot(*this, "manager") / "Shared_Files" / fileName;

    const auto rows = ctr_common::csv::readNumericCsv(file_path);
    if (!rows)
    {
        RCLCPP_ERROR(get_logger(), "[traj] Failed to open file: %s", file_path.c_str());
        return false;
    }

    // Legacy 6-column layout or the planner's 4-column layout; see
    // manager_csv::parsePathRows for the expansion semantics.
    size_t bad_rows = 0;
    init_q_list = manager_csv::parsePathRows(*rows, &bad_rows);
    if (bad_rows > 0)
    {
        RCLCPP_WARN(get_logger(), "[traj] %zu rows did not contain 4 or 6 values and were skipped", bad_rows);
    }

    RCLCPP_INFO(get_logger(), "[traj] Loaded %zu path points from CSV file.", init_q_list.size());
    return true;
}

void MasterNode::read_targets_from_csv(std::vector<Eigen::Vector3d> &target_list, const std::string &fileName)
{
    const std::filesystem::path file_path =
        ctr_common::resolveDataRoot(*this, "manager") / "Input_Files" / fileName;

    const auto rows = ctr_common::csv::readNumericCsv(file_path);
    if (!rows)
    {
        RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    target_list.clear();

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
            m_Xd = m_test_targets[m_current_target_index];
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
            if (m_planner_success && m_flag_planner_updated)
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
        else if (time_in_state > 60.0)
        {
            RCLCPP_WARN(get_logger(), "[Test] Retraction timeout, skipping to next target");
            m_auto_retract = false;
            {
                std::lock_guard<std::mutex> lock(m_deploy_mutex);
                m_q_list_adjusted.clear();
                m_q_list_actuated.clear();
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

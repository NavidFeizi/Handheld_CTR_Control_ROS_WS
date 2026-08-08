#include "manager/master_node.hpp"

#include "ctr_common/csv_io.hpp"
#include "ctr_common/joint_conventions.hpp"
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
    auto request = std::make_shared<interfaces::srv::Config::Request>();
    request->command = command;

    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
    auto client = use_enable_service ? m_robot_enable_client : m_robot_config_client;
    auto response_received_callback = std::bind(&MasterNode::handle_service_response, this, std::placeholders::_1);
    (void)client->async_send_request(request, response_received_callback);
}

void MasterNode::handleFreezeButtonClicked(QPushButton *freeze_button)
{
    m_robot_frozen = !m_robot_frozen;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = m_robot_frozen;

    auto result_callback = [this, freeze_button](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            freeze_button->setText(m_robot_frozen ? "Unfreeze Robot" : "Freeze Robot");
            // Set button color: red when unfrozen (showing "Freeze Robot"), normal when frozen
            if (m_robot_frozen)
            {
                freeze_button->setStyleSheet(""); // Reset to default style when frozen
            }
            else
            {
                freeze_button->setStyleSheet("background-color: rgb(255, 0, 0); color: white;"); // Red background when unfrozen
            }
            RCLCPP_INFO(this->get_logger(), "Robot %s", m_robot_frozen ? "frozen" : "unfrozen");
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
    Eigen::Vector3d error = m_tip_position - m_Xd;
    
    // Update row 5 for CSV target
    QTableWidget* emt_table = m_gui_manager->getEmtStatusTable();
    emt_table->setItem(5, 0, new QTableWidgetItem(QString::number(m_Xd[0], 'f', 3)));
    emt_table->setItem(5, 1, new QTableWidgetItem(QString::number(m_Xd[1], 'f', 3)));
    emt_table->setItem(5, 2, new QTableWidgetItem(QString::number(m_Xd[2], 'f', 3)));
    emt_table->setItem(5, 3, new QTableWidgetItem(QString::number(m_Xd.norm(), 'f', 3)));
    emt_table->setItem(5, 4, new QTableWidgetItem(QString::number(atan2(m_Xd[1], m_Xd[0]), 'f', 3)));
    
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
        // read_path_from_csv(m_q_list, "plannedPath.csv");
        // m_current_config_index = 0;
        // m_q_list_adjusted = adjustConfigurationListStepSize(m_q_list, m_insertion_step);
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
    while (!m_robot_config_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Robot config service not available, waiting ...");
    }

    m_robot_enable_client = create_client<interfaces::srv::Config>("robot_enable");
    while (!m_robot_enable_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Robot enable service not available, waiting ...");
    }

    m_planner_client = create_client<interfaces::srv::Planner>("planner/command");
    while (!m_planner_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(get_logger(), "planner/command service not available, waiting ...");
    }

    m_freeze_robot_client = create_client<std_srvs::srv::SetBool>("freeze_robot");
    while (!m_freeze_robot_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(get_logger(), "freeze_robot service not available, waiting ...");
    }

    m_recording_client = create_client<interfaces::srv::Recording>("recording");
    while (!m_recording_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(get_logger(), "recording service not available, waiting ...");
    }

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
    }

    emit robotStatusUpdated(m_enabled, m_procedure, m_head_attached, m_engaged, m_locked);
    emit update_enable_button_Text(m_enabled ? "Disable" : "Enable");
}

void MasterNode::manualInterface_callback(const interfaces::msg::Interface::SharedPtr msg)
{
    m_interface_key_prev = m_interface_key;

    for (int i = 0; i < 7; i++)
    {
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
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    targetFrame.c_str(), sourceFrame.c_str(), ex.what());
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
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    targetFrame.c_str(), sourceFrame.c_str(), ex.what());
        return;
    }
    m_trans_probe = tf2::transformToEigen(tf2_tran).matrix();

    // Update positions
    if (!m_test_running && !m_use_csv_target)
    {
        m_Xd = m_trans_probe.block<3, 1>(0, 3); // target tip position

        // Publish target position
        auto target_msg = interfaces::msg::Taskspace();
        target_msg.p[0] = m_Xd[0];
        target_msg.p[1] = m_Xd[1];
        target_msg.p[2] = m_Xd[2];
        m_pub_task_target->publish(target_msg);
    }

    m_X = m_trans_tip.block<3, 1>(0, 3); // Current tip position
    m_tip_position = m_X;  // Store for CSV target error calculation
    
    // Update CSV target display if in CSV mode
    if (m_use_csv_target)
    {
        updateCsvTargetDisplay();
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

    m_planner_success = response->success;
    m_planner_ik_error = response->value;

    // In closed-loop mode, bypass error check before first insertion to ensure initial plan exists
    // bool bypass_error_check = m_closed_loop_enabled && m_q_list_actuated.empty();
    bool bypass_error_check = false;

    RCLCPP_INFO(this->get_logger(), "Planner bypass_error_check=%s", bypass_error_check ? "true" : "false");
    
    if ((m_planner_success || bypass_error_check) && m_planner_ik_error < k_ik_error_threshold)
    {
        if (loadPlannedPath())
        {
            std::lock_guard<std::mutex> lock(m_force_mutex);
            m_f_at_plan = m_f_pending;
            m_f_at_plan_valid = true;
            m_replan_attempts = 0;
            m_replan_backoff_s = m_replan_cooldown_s;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load plannedPath.csv - path will be empty");
            m_planner_success = false;
        }
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

    if (response->success && loadPlannedPath())
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
            return;
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

void MasterNode::control_loop()
{
    // Update automated test state machine
    updateTestStateMachine();

    Eigen::Vector3d X = m_X;
    Eigen::Vector3d Xd = m_Xd;

    if (m_procedure && m_high_level_mode == HighLvlCtrMode::Planner)
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
        }
        // If target position changed significantly, call planner to generate new path
        else
        {
            // std::cout << "m_reached: " << m_reached << ", !m_flag_planning: " << (!m_flag_planning) << ", target_changed: " << target_changed << ", q_changed: " << q_changed << std::endl;
            if (m_reached && (!m_flag_planning) && (target_changed || q_changed))
            {
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
                m_flag_planner_updated = true;
                emit plannerStatusUpdated(m_flag_planning, m_planner_success, m_planner_ik_error);
                RCLCPP_INFO(this->get_logger(), "Planner called.");
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
            if (m_q_list_adjusted.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Adjusted deployment list is empty; cannot send positions.");
            }

            if (getReachStatus())
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

                    RCLCPP_DEBUG(this->get_logger(), "Sent q [%d/%zu]: β1=%.3f, β2=%.3f, α1=%.3f, α2=%.3f",
                                 m_current_config_index + 1, m_q_list_adjusted.size(),
                                 m_q_list_adjusted[m_current_config_index][0],
                                 m_q_list_adjusted[m_current_config_index][1],
                                 m_q_list_adjusted[m_current_config_index][3],
                                 m_q_list_adjusted[m_current_config_index][4]);
                }
                else
                {
                    m_auto_insert = false;
                    m_current_config_index = m_q_list_adjusted.size() - 1;
                    RCLCPP_DEBUG(this->get_logger(), "Max deployment index reached: %d", m_current_config_index);
                }
            }
        }
        else if (!m_flag_planning && (m_interface_key[k_backward_button_idx] || m_auto_retract))
        {
            m_retracting = true;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_q_list_adjusted.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Adjusted deployment list is empty; cannot send positions.");
            }

            if (getReachStatus())
            {
                m_current_config_index--;
                if (m_current_config_index >= 0)
                {
                    publish_position(m_q_list_adjusted[m_current_config_index]);

                    RCLCPP_DEBUG(this->get_logger(), "Sent q [%d/%zu]:  β1=%.3f, β2=%.3f, α1=%.3f, α2=%.3f",
                                 m_current_config_index + 1, m_q_list_adjusted.size(),
                                 m_q_list_adjusted[m_current_config_index][0],
                                 m_q_list_adjusted[m_current_config_index][1],
                                 m_q_list_adjusted[m_current_config_index][3],
                                 m_q_list_adjusted[m_current_config_index][4]);
                }
                else
                {
                    m_auto_retract = false;
                    m_current_config_index = 0;
                    m_q_list_adjusted.clear();
                    invalidateForceBaseline();
                    RCLCPP_DEBUG(this->get_logger(), "Min deployment index reached: %d", m_current_config_index);
                }
            }
        }
    }
    else if (m_procedure && m_high_level_mode == HighLvlCtrMode::Deployment && m_closed_loop_enabled)
    {
        Eigen::Vector3d Xsim_error(0.0, 0.0, 0.0);
        Xsim_error = X - m_Xsim;
        Eigen::Vector3d Xd_adj = Xd - Xsim_error;
        bool target_changed = (Xd_adj - m_Xd_adj_prev).norm() > k_target_threshold;
        bool q_changed = blaze::norm((m_q - m_q_prev) / k_input_scale) > k_q_threshold;
        double Xe = (X - Xd).norm();

        // Call planner if target changed significantly
        if (m_reached && !m_flag_planning && (target_changed || q_changed) && !m_retracting && (Xe > (k_target_threshold / 2)))
        {
            m_flag_planning = true;
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
            RCLCPP_INFO(this->get_logger(), "Planner called.");

            m_Xd_adj_prev = Xd_adj;
            m_q_prev = m_q;

            std::this_thread::sleep_for(10ms);
        }

        // Handle manual and auto insertion
        if (!m_flag_planning && (m_interface_key[k_forward_button_idx] || m_auto_insert))
        {
            m_retracting = false;
            std::lock_guard<std::mutex> lock(m_deploy_mutex);
            if (m_q_list_adjusted.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Adjusted deployment list is empty; cannot send positions.");
            }

            if (getReachStatus())
            {
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

                    RCLCPP_INFO(this->get_logger(), "Sent q [%d/%zu]:  β1=%.3f, β2=%.3f, α1=%.3f, α2=%.3f",
                                m_current_config_index + 1, m_q_list_adjusted.size(),
                                m_q_list_adjusted[m_current_config_index][0],
                                m_q_list_adjusted[m_current_config_index][1],
                                m_q_list_adjusted[m_current_config_index][3],
                                m_q_list_adjusted[m_current_config_index][4]);

                    m_q_list_actuated.push_back(m_q_list_adjusted[m_current_config_index]);
                }
                else
                {
                    m_auto_insert = false;
                    m_q_list_adjusted.clear();
                    invalidateForceBaseline();
                    m_current_config_index = 0;
                    RCLCPP_DEBUG(this->get_logger(), "Max deployment index reached: %d", m_current_config_index);
                }
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
                if (m_q_list_actuated.size() > 1)
                {
                    m_q_list_actuated.pop_back();
                    publish_position(m_q_list_actuated.back());
                    RCLCPP_INFO(this->get_logger(), "Sent q [%zu/%zu]:  β1=%.3f, β2=%.3f, α1=%.3f, α2=%.3f",
                                m_q_list_actuated.size() + 1, m_q_list_actuated.size() + 2,
                                m_q_list_actuated.back()[0], m_q_list_actuated.back()[1],
                                m_q_list_actuated.back()[3], m_q_list_actuated.back()[4]);
                }
                else
                {
                    m_auto_retract = false;
                    m_q_list_actuated.clear();
                    m_q_list_adjusted.clear();
                    invalidateForceBaseline();

                    // Delete plannedPath.csv
                    std::filesystem::path file_path =
                        ctr_common::resolveDataRoot(*this, "manager") / "Shared_Files" / "plannedPath.csv";
                    
                    if (std::filesystem::exists(file_path))
                    {
                        std::filesystem::remove(file_path);
                        RCLCPP_INFO(this->get_logger(), "Deleted plannedPath.csv");
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Min deployment index reached: %zu", m_q_list_actuated.size());
                }
            }
        }
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
            RCLCPP_INFO(this->get_logger(), "plannedPath.csv loaded (written %.1f s ago)", age / 1000.0);
        }

        m_q_list_adjusted = adjustConfigurationListStepSize(m_q_list, m_insertion_step);
        m_current_config_index = 0;
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

    init_q_list.clear();

    // The file may be in the legacy 6-column layout [β₁, β₂, β₃, α₁, α₂, α₃]
    // (β₃/α₃ are the unactuated outer tube, always 0) or the new planner
    // 4-column layout [β₁, β₂, α₁, α₂] (no header — non-numeric lines are
    // skipped by the parser). A 4-column row is expanded to the 6-element
    // layout so the rest of the (6-column) pipeline can consume it unchanged.
    for (const auto &row : *rows)
    {
        if (row.size() == 6)
        {
            init_q_list.push_back({row[0], row[1], row[2], row[3], row[4], row[5]});
        }
        else if (row.size() == 4)
        {
            init_q_list.push_back({row[0], row[1], 0.0, row[2], row[3], 0.0});
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Row does not contain 4 or 6 values (got %zu)", row.size());
        }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu path points from CSV file.", init_q_list.size());
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

std::vector<blaze::StaticVector<double, 6>> MasterNode::adjustConfigurationListStepSize(const std::vector<blaze::StaticVector<double, 6>> &q_list_in, double step_size)
{
    std::vector<blaze::StaticVector<double, 6>> q_list_out;

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
            m_gui_manager->getPlannerRadioButton()->setChecked(true);
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
            m_gui_manager->getDeploymentRadioButton()->setChecked(true);
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

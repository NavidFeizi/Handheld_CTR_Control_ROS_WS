#ifndef MASTER_NODE_HPP
#define MASTER_NODE_HPP

#include <QWidget>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/status.hpp"
#include "interfaces/msg/interface.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/force.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/srv/planner.hpp"
#include "interfaces/srv/recording.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <blaze/Blaze.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <algorithm>
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <atomic>

// #include "manager/qt_gui.hpp"

#include <QApplication>
#include <QPushButton>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <thread>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_eigen/tf2_eigen.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "master_qt_gui.hpp"

enum class CtrlMode : int
{
    Config = 0x00,
    Manual = 0x01,
    Position = 0x02,
    Velocity = 0x03,
};

enum class HighLvlCtrMode : int
{
    Planner = 0x00,
    Deployment = 0x01,
    None = 0x02,
};

enum class TestState : int
{
    Idle = 0,
    SelectingTarget = 1,
    WaitingForPlanning = 2,
    SwitchingToDeployment = 3,
    Inserting = 4,
    WaitingInsertionComplete = 5,
    Retracting = 6,
    WaitingRetractionComplete = 7,
    Complete = 8
};

class MasterNode : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit MasterNode(QWidget* parent = nullptr);
    
    // Public methods for GUI callbacks
    void sendConfigCommand(const std::string& command, bool use_enable_service);
    void handleFreezeButtonClicked(QPushButton* freeze_button);
    void handleAutoInsertClicked();
    void handleAutoRetractClicked();
    void handleTestButtonClicked();
    void handleToggleTargetModeClicked();
    void handleNextTargetClicked();
    void handlePrevTargetClicked();
    void updateCsvTargetDisplay();

signals:
    void update_enable_button_Text(QString text);
    void update_wrench_pos_button_Text(QString text);
    void robotStatusUpdated(bool enable, bool procedure, bool head, bool attached, int locked);
    void emtUpdated(double tx, double ty, double tz, double px, double py, double pz, 
                    double pinnx, double pinny, double pinnz);
    void plannerStatusUpdated(bool planning, bool success, double ik_error);

public slots:
    void onClosedLoopToggled(bool checked);
    void onCtrlModeClicked(int id);

private:
    // GUI manager
    std::unique_ptr<QtGuiManager> m_gui_manager;
    
    // ROS interfaces initialization
    void initRosInterfaces();
    
    // ROS callbacks
    void jointsConfig_timerCallback(const interfaces::msg::Jointspace::SharedPtr msg);
    void robotStatus_callback(const interfaces::msg::Status::SharedPtr msg);
    void manualInterface_callback(const interfaces::msg::Interface::SharedPtr msg);
    void updateSimout(const interfaces::msg::Taskspace::SharedPtr msg);
    void updateForceEstimate(const interfaces::msg::Force::SharedPtr msg);
    void tf2_receive_timer_callback();
    void handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future);
    void handle_planner_response(const rclcpp::Client<interfaces::srv::Planner>::SharedFuture future);
    void handle_replan_response(const rclcpp::Client<interfaces::srv::Planner>::SharedFuture future);

    // Control functions
    void control_loop();
    // Planning is triggered implicitly by control_loop, not by any button. When one of
    // its gates is closed nothing happens and nothing is logged, which is impossible to
    // diagnose from the operator's seat; these name the first gate that is blocking.
    // Each reason uses its own throttle macro call site so that a change of reason is
    // reported promptly instead of being swallowed by a shared throttle window.
    void reportPlannerGate(double tube_1_theta_diff, double tube_2_theta_diff,
                           bool target_changed, bool q_changed);
    // True once a rejected plan's cooldown has elapsed, so the same target is retried.
    bool planRetryDue() const;
    // Deployment's counterpart to reportPlannerGate. Without it the Deployment branch
    // falls through every condition and logs nothing, so "waiting for the operator" and
    // "the control loop is dead" look identical from the terminal.
    void reportDeploymentGate();
    std::string missingServicesDescription() const;
    // Arms the cooldown retry and says why the plan was dropped. `reason` is logged verbatim.
    void armPlanRetry(const std::string &reason);
    void maybeRequestDeploymentReplan();
    void invalidateForceBaseline();
    void publish_position(const blaze::StaticVector<double, 6>& q);

    // Utility functions
    bool loadPlannedPath();
    bool read_path_from_csv(std::vector<blaze::StaticVector<double, 6>>& init_q_list,
                               const std::string& fileName);
    void read_targets_from_csv(std::vector<Eigen::Vector3d>& target_list, const std::string& fileName);
    void updateTestStateMachine();
    void captureRecordingFrame(const std::string& session_name);
    bool getReachStatus();
    std::vector<blaze::StaticVector<double, 6>> adjustConfigurationListStepSize(
        const std::vector<blaze::StaticVector<double, 6>>& q_list_in, double step_size);
    void log_position(Eigen::Vector3d position, std::string prefix = "");
    
    // Member constants
    static constexpr size_t k_forward_button_idx = 1;
    static constexpr size_t k_backward_button_idx = 0;
    static constexpr double k_theta_threshold = 10.0 * M_PI / 180.0;
    static constexpr double k_target_threshold = 0.002;
    static constexpr double k_q_threshold = 0.002;
    static constexpr blaze::StaticVector<double, 4UL> k_input_scale = {1.00, 1.00, 20.0, 20.0};
    // Replanning tunables — declared as ROS parameters in the constructor
    double m_force_replan_threshold = 0.12; // N; ‖f_now − f_at_plan‖ that triggers a deployment replan
    double m_replan_cooldown_s = 2.0;       // s between replan requests (EKF ramps at f_dot ≈ 0.1 N/s)
    size_t m_min_remaining_waypoints = 5;   // ≈10 mm at m_insertion_step; below this a replan is not worth the pause

    // Rejected-replan backoff: after each rejection the wait doubles (capped);
    // after k_max_replan_attempts consecutive rejections replanning is
    // suppressed until the force drift recovers below threshold/2 (hysteresis)
    // or a plan/replan is accepted.
    static constexpr int k_max_replan_attempts = 3;
    static constexpr double k_replan_backoff_cap_s = 60.0;
    int m_replan_attempts = 0;
    double m_replan_backoff_s = 2.0;
    std::string m_targets_csv = "random_interior_points.csv"; // target list in Input_Files/
    // async_send_request has no timeout: if the planner dies mid-request m_flag_planning
    // never clears and every deployment branch is blocked for the rest of the session.
    double m_planner_timeout_s = 15.0;
    // A rejected plan latches m_Xd_prev just like an accepted one, so without a retry the
    // loop reports "target unchanged" for the rest of the session unless the operator moves
    // the probe. Re-arm the same target after this cooldown instead of going quiet.
    double m_plan_retry_cooldown_s = 5.0;

    // Member variables
    CtrlMode m_ctrl_mode = CtrlMode::Config;
    CtrlMode m_ctrl_mode_prev = CtrlMode::Config;
    HighLvlCtrMode m_high_level_mode = HighLvlCtrMode::Planner;

    bool m_closed_loop_enabled = false;
    std::atomic<bool> m_flag_planning{false}; // written by service callbacks, read by control_loop and GUI
    std::atomic<double> m_planner_request_time_s{0.0}; // clock seconds when m_flag_planning was raised
    std::atomic<bool> m_services_ready{false};
    rclcpp::TimerBase::SharedPtr m_readiness_timer;
    bool m_flag_planner_updated = false;
    std::atomic<bool> m_planner_success{false};
    std::atomic<double> m_planner_ik_error{0.0};
    // Set by the response handlers when a plan is rejected; consumed by control_loop as an
    // extra request trigger once the cooldown has elapsed.
    std::atomic<bool> m_plan_retry_armed{false};
    std::atomic<double> m_plan_retry_after_s{0.0};
    
    blaze::StaticVector<double, 4> m_com_vel;
    blaze::StaticVector<double, 4UL> m_q, m_q_des, m_q_error, m_q_abs, m_q_prev;
    blaze::StaticVector<double, 4UL> m_qdot, m_qdot_manual, m_qdot_des, m_qdot_error, m_qdot_forward;
    blaze::StaticVector<double, 4UL> m_current;
    blaze::StaticVector<double, 4UL> m_minCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
    blaze::StaticVector<double, 4UL> m_maxCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
    
    std::array<std::atomic<bool>, 7> m_interface_key = {}; // written by the interface subscription, read by control_loop
    std::array<bool, 7> m_interface_key_prev = {0, 0, 0, 0, 0, 0, 0};
    
    std::vector<blaze::StaticVector<double, 6>> m_q_list;
    std::vector<blaze::StaticVector<double, 6>> m_q_list_adjusted;
    std::vector<blaze::StaticVector<double, 6>> m_q_list_actuated;
    std::mutex m_deploy_mutex; // guards m_q_list, m_q_list_adjusted, m_q_list_actuated, m_current_config_index
                               // (written by planner-response callbacks, read/written by the control timer)
    int m_current_config_index = 0;
    double m_insertion_step = 2e-3;
    bool m_deploy_button_held = false;
    std::atomic<bool> m_auto_insert{false};   // GUI thread <-> control_loop
    std::atomic<bool> m_auto_retract{false};  // GUI thread <-> control_loop
    bool m_retracting = false;

    double k_ik_error_threshold = 0.003;
    
    // Automated test variables
    std::atomic<bool> m_test_running{false};
    TestState m_test_state = TestState::Idle;
    std::vector<Eigen::Vector3d> m_test_targets;
    size_t m_current_target_index = 0;
    rclcpp::Time m_test_state_entry_time;
    bool m_target_set = false;
    
    // Manual CSV target selection variables
    std::atomic<bool> m_use_csv_target{false};
    std::vector<Eigen::Vector3d> m_csv_targets;
    size_t m_csv_target_index = 0;
    
    // One initializer per declarator: `bool a, b, c = false;` initializes only `c`, which
    // left m_procedure/m_reached indeterminate until the first robot_status message.
    bool m_enabled = false;
    bool m_procedure = false;
    bool m_reached = false;
    bool m_encoder = false;
    bool m_engaged = false;
    bool m_ready_to_engage = false;
    bool m_head_attached = false;
    int m_locked = 0;
    std::atomic<bool> m_robot_frozen{false};
    bool m_flag_manual = false;
    bool m_flag_use_target_action = false;
    bool m_flag_enabled = false;
    bool m_trans_limit = false;
    bool m_trans_limit_prev = false;
    
    blaze::StaticVector<bool, 4UL> m_enabledJoints, m_encoderJoints, m_reachedJoints;
    // Previous enable-fault latches, for edge-triggered logging (the Status
    // topic republishes on a timer, so level-triggered would flood the log).
    std::array<bool, 4> m_enableFaultPrev = {0, 0, 0, 0};
    
    // Eigen's default ctor leaves these uninitialized; m_Xd_prev is read by the
    // target_changed comparison in control_loop before anything assigns it.
    Eigen::Matrix4d m_trans_tip = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d m_trans_probe = Eigen::Matrix4d::Identity();
    Eigen::Vector3d m_Xd = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_Xd_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_Xd_adj_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_X = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_Xsim = Eigen::Vector3d::Zero();
    std::mutex m_feedback_mutex; // guards m_X, m_Xsim, m_Xd, m_tip_position, m_q, m_qdot, m_current
    Eigen::Vector3d m_tip_position = Eigen::Vector3d::Zero();  // For CSV target error calculation

    // Force-triggered deployment replanning
    std::mutex m_force_mutex; // guards m_f_est (force subscription) AND the plan
                              // force baseline m_f_at_plan / m_f_pending /
                              // m_f_at_plan_valid (written from service-response
                              // callbacks, read from the Qt control loop)
    Eigen::Vector3d m_f_est = Eigen::Vector3d::Zero();     // latest EKF tip-force estimate [N]
    Eigen::Vector3d m_f_at_plan = Eigen::Vector3d::Zero(); // force the ACTIVE waypoint list was planned with
    Eigen::Vector3d m_f_pending = Eigen::Vector3d::Zero(); // snapshot taken when a planner request is sent
    bool m_f_at_plan_valid = false;
    rclcpp::Time m_last_replan_request_time{0, 0, RCL_ROS_TIME};
    int m_replan_count = 0;
    
    // ROS2 interfaces
    rclcpp::CallbackGroup::SharedPtr m_cbGroup1;
    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subscription_status;
    rclcpp::Subscription<interfaces::msg::Interface>::SharedPtr m_subscription_interface;
    rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_sim_out;
    rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_sub_force;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_config_client;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_enable_client;
    rclcpp::Client<interfaces::srv::Planner>::SharedPtr m_planner_client;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_freeze_robot_client;
    rclcpp::Client<interfaces::srv::Recording>::SharedPtr m_recording_client;
    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_manual_vel;
    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_pub_joint_targ;
    rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_pub_task_target;
    
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf2_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_broadcast;
    rclcpp::TimerBase::SharedPtr m_tf2_timer;
    rclcpp::CallbackGroup::SharedPtr m_callback_group_tf2;
    rclcpp::CallbackGroup::SharedPtr m_callback_group_pub1;
    rclcpp::TimerBase::SharedPtr m_control_timer;
};

#endif // MASTER_NODE_HPP

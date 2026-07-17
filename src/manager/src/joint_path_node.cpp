#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <optional>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/status.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/srv/recording.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string package_name = "manager"; // Replace with any package in your workspace
std::string PACKAGE_SHARE_DIR = ament_index_cpp::get_package_share_directory(package_name);

enum class CtrlMode : int
{
    Config = 0x00,
    Manual = 0x01,
    Position = 0x02,
    Velocity = 0x03,
};

struct VelocitySample
{
    double timestamp;
    std::vector<double> velocities; // alpha1, alpha2, alpha3, beta1, beta2, beta3
};

class JointPathNode : public rclcpp::Node
{
public:
    JointPathNode(rclcpp::executors::MultiThreadedExecutor *exec) : rclcpp::Node("ctr_jointpath_node"), m_executor(exec)
    {
        JointPathNode::declare_parameters();
        JointPathNode::initRosInterfaces();
        JointPathNode::read_samples_from_csv(m_q_list, "plannedPath.csv");

        // std::thread([this]()
        //             { this->run_sequence(); })
        //     .detach();

        std::thread([this]()
                    { this->run_velocity_tracking(); })
            .detach();
    }

private:
    void declare_parameters()
    {
        declare_parameter<double>("velocity_magnitude", 0.5); // [rad/s] or [m/s] per joint units
        m_vel_mag = get_parameter("velocity_magnitude").as_double();

        declare_parameter<double>("sample_time_ms", 10.0); // Velocity command sample time in ms
        m_sample_time_ms = get_parameter("sample_time_ms").as_double();

        declare_parameter<std::string>("velocity_csv", "targetVelocity.csv");
        m_velocity_csv = get_parameter("velocity_csv").as_string();

        // Initial position before velocity tracking
        declare_parameter<std::vector<double>>("initial_position", {-0.070, -0.035, 0.0, M_PI/2, M_PI/2, 0.0});
        auto init_pos_vec = get_parameter("initial_position").as_double_array();
        if (init_pos_vec.size() == 6)
        {
            for (size_t i = 0; i < 6; ++i)
                m_initial_position[i] = init_pos_vec[i];
        }
    }

    // Initialize ROS interfaces, including publishers, subscribers, and service clients.
    void initRosInterfaces()
    {
        cb_state_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_comm_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions subs_opts;
        subs_opts.callback_group = cb_comm_;

        m_subs_joints = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&JointPathNode::on_feedback, this, std::placeholders::_1), subs_opts);
        m_subs_status = create_subscription<interfaces::msg::Status>("robot_status", 10, std::bind(&JointPathNode::on_status, this, std::placeholders::_1), subs_opts);

        m_pub_joint_targ = create_publisher<interfaces::msg::Jointspace>("joint_space/target", 10);

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
            RCLCPP_INFO(get_logger(), "Robot config service not available, waiting ...");
        }

        // Recorder service
        m_record_client = this->create_client<interfaces::srv::Recording>("recording");
        while (!m_record_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Record service not available, waiting again...");
        }

        // Initialize velocity timer (disabled initially)
        m_velocity_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(m_sample_time_ms)), std::bind(&JointPathNode::velocity_timer_callback, this),cb_comm_);
        m_velocity_timer->cancel(); // Start disabled
    }

    // ---------- Helpers ----------
    static std::string vec_to_str(const std::vector<double> &v)
    {
        std::ostringstream os;
        os << "[";
        for (size_t i = 0; i < v.size(); ++i)
        {
            os << v[i];
            if (i + 1 < v.size())
                os << ",";
        }
        os << "]";
        return os.str();
    }

    void wait_for_service(const rclcpp::Client<interfaces::srv::Config>::SharedPtr &c, const char *name)
    {
        while (!c->wait_for_service(1s))
        {
            if (!rclcpp::ok())
                return;
            RCLCPP_WARN(get_logger(), "Waiting for service: %s ...", name);
        }
    }

    void ctrlMode(int mode)
    {
        // Send request
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        const std::string &command = "setCtrlMode";
        request->command = command;
        request->value = mode;
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&JointPathNode::handle_service_response, this, std::placeholders::_1);
        auto fut = m_robot_config_client->async_send_request(request, response_received_callback);
        // RCLCPP_INFO(this->get_logger(), "Sent CtrlMode change request: %d", mode);
    }

    void call_enable_toggle()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "toggleEnable";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&JointPathNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_enable_client->async_send_request(request, response_received_callback);
    }

    void disable()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "disable";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&JointPathNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_enable_client->async_send_request(request, response_received_callback);
    }

    void publish_position(const blaze::StaticVector<double, 6> &q)
    {
        interfaces::msg::Jointspace msg;
        msg.position[0] = q[3]; // alpha1
        msg.position[1] = q[0]; // beta1
        msg.position[2] = q[4]; // alpha2
        msg.position[3] = q[1]; // beta2
        m_pub_joint_targ->publish(msg);
    }

    void publish_velocity(const blaze::StaticVector<double, 6> &q_dot)
    {
        interfaces::msg::Jointspace msg;
        msg.velocity[0] = q_dot[3];
        msg.velocity[1] = q_dot[0];
        msg.velocity[2] = q_dot[4];
        msg.velocity[3] = q_dot[1];
        m_pub_joint_targ->publish(msg);
    }

    void velocity_timer_callback()
    {
        if (m_velocity_idx >= m_velocity_trajectory.size())
        {
            // Trajectory complete - send zero velocity and disable
            blaze::StaticVector<double, 6> zero_vel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            publish_velocity(zero_vel);
            
            m_velocity_timer->cancel();
            RCLCPP_INFO(get_logger(), "Velocity tracking complete.");
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            disable();
            RCLCPP_INFO(get_logger(), "Robot disabled after velocity tracking.");
            return;
        }

        const auto &q_dot_target = m_velocity_trajectory[m_velocity_idx];
        publish_velocity(q_dot_target);

        if (m_velocity_idx % 50 == 0) // Log every 50 samples to avoid spam
        {
            RCLCPP_INFO(get_logger(), "Vel cmd %zu/%zu: [α1=%+.3f, β1=%+.3f, α2=%+.3f, β2=%+.3f]",
                        m_velocity_idx, m_velocity_trajectory.size(),
                        q_dot_target[3], q_dot_target[0], q_dot_target[4], q_dot_target[1]);
        }

        m_velocity_idx++;
    }

    void handle_record_response(rclcpp::Client<interfaces::srv::Recording>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Recording started successfully: %s", response->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", response->message.c_str());
        }
    }

    // ---------- Callbacks ----------
    void on_status(const interfaces::msg::Status::SharedPtr msg)
    {
        m_enabled = msg->enable[0] * msg->enable[1] * msg->enable[2] * msg->enable[3];
        m_reached = msg->reached[0] * msg->reached[1] * msg->reached[2] * msg->reached[3];
    }

    void on_feedback(const interfaces::msg::Jointspace::SharedPtr msg)
    {
        for (int i = 0; i < 4; ++i)
        {
            m_q[i] = msg->position[i];
            m_dq[i] = msg->velocity[i];
            m_curr[i] = msg->current[i];
        }
    }

    void handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
    {
        // Get the result of the future object
        auto response = future.get();

        if (response->success)
        {
            // RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
        }
        else
        {
            // RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
        }
    }

    // ---------- Main ----------
    /**
     * @brief Executes the main control sequence for the robotic arm trajectory following.
     *
     * This function orchestrates the complete workflow:
     * 1. Disables the robot and switches to Position control mode
     * 2. Initializes the recording system
     * 3. Enables the robot
     * 4. Iterates through planned joint-space waypoints from m_q_list with a configurable step size
     * 5. For each waypoint:
     *    - Publishes the target joint position
     *    - Waits until the robot reaches the target position
     *    - Captures a recording frame at the reached position
     * 6. Disables the robot and closes the recording session
     *
     * The sequence processes waypoints at intervals determined by the 'step' variable,
     * ensuring the last waypoint is always executed. It blocks on reaching each target
     * before proceeding to the next one.
     *
     * @note This function runs in a detached thread spawned during object construction.
     * @see do_one() helper lambda that handles individual waypoint execution
     */
    void run_sequence()
    {
        // Disable and set to Position mode
        disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ctrlMode(static_cast<int>(CtrlMode::Position));
        RCLCPP_INFO(get_logger(), "Mode -> Position");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Send recording command
        {
            auto request = std::make_shared<interfaces::srv::Recording::Request>();
            request->command = "init";
            request->duration = 0.0; // Not used for capture
            auto future_result = m_record_client->async_send_request(request);
            // block until the service response arrives
            future_result.wait();
            auto response = future_result.get();
            if (response->success)
                RCLCPP_INFO(this->get_logger(), "Init success: %s", response->message.c_str());
            else
                RCLCPP_ERROR(this->get_logger(), "Init failed: %s", response->message.c_str());
        }

        call_enable_toggle();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto do_one = [&](size_t i)
        {
            m_q_target = m_q_list[i];
            publish_position(m_q_target);
            RCLCPP_INFO(get_logger(), "Target %zu sent: [%f, %f, %f, %f]",
                        i, m_q_target[3], m_q_target[0], m_q_target[4], m_q_target[1]);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            while (rclcpp::ok() && !m_reached)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

            RCLCPP_INFO(get_logger(), "Reached target %zu.", i);

            auto req = std::make_shared<interfaces::srv::Recording::Request>();
            req->command = "capture";
            req->duration = 0.0;
            auto fut = m_record_client->async_send_request(req);
            fut.wait();
            auto resp = fut.get();
            if (resp->success)
                RCLCPP_INFO(this->get_logger(), "Capture success: %s", resp->message.c_str());
            else
                RCLCPP_ERROR(this->get_logger(), "Capture failed: %s", resp->message.c_str());
        };

        // for (size_t i = 0; i < m_q_list.size(); i += step)
        // {
        //     m_q_target = m_q_list[i];
        //     publish_position(m_q_target);
        //     RCLCPP_INFO(get_logger(), "Target %zu sent: [%f, %f, %f, %f]",
        //                 i, m_q_target[3], m_q_target[0], m_q_target[4], m_q_target[1]);

        //     // Wait until reached
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     while (rclcpp::ok() && !m_reached)
        //     {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //     }

        //     RCLCPP_INFO(get_logger(), "Reached target %zu.", i);

        //     // Send recording command
        //     auto request = std::make_shared<interfaces::srv::Recording::Request>();
        //     request->command = "capture";
        //     request->duration = 0.0; // Not used for capture
        //     auto future_result = m_record_client->async_send_request(request);
        //     // block until the service response arrives
        //     future_result.wait();
        //     auto response = future_result.get();
        //     if (response->success)
        //         RCLCPP_INFO(this->get_logger(), "Capture success: %s", response->message.c_str());
        //     else
        //         RCLCPP_ERROR(this->get_logger(), "Capture failed: %s", response->message.c_str());
        // }

        size_t step = 10; // change this value to adjust the step size through m_q_list

        const size_t N = m_q_list.size();
        if (N == 0)
            return;

        if (step == 0)
        { // safety
            do_one(N - 1);
            return;
        }

        for (size_t i = 0; /*no cond*/;)
        {
            do_one(i);
            if (i == N - 1)
                break; // we did the last one
            size_t next = i + step;
            if (next >= N)
                next = N - 1; // clamp to last index
            if (next == i)
                break; // paranoia guard
            i = next;
        }

        disable();

        // Send close command
        {
            auto request = std::make_shared<interfaces::srv::Recording::Request>();
            request->command = "close";
            request->duration = 0.0; // Not used for capture
            auto future_result = m_record_client->async_send_request(request);
            // block until the service response arrives
            future_result.wait();
            auto response = future_result.get();
            if (response->success)
                RCLCPP_INFO(this->get_logger(), "Close success: %s", response->message.c_str());
            else
                RCLCPP_ERROR(this->get_logger(), "Close failed: %s", response->message.c_str());
        }
    }

    /**
     * @brief Executes velocity tracking sequence from CSV file.
     *
     * Workflow:
     * 1. Disables robot and switches to Position mode
     * 2. Moves to initial position specified in parameters
     * 3. Switches to Velocity mode
     * 4. Reads target velocities from CSV file
     * 5. Commands velocities at configured sample rate
     * 6. Disables robot after completion
     *
     * CSV Format: Each row contains 6 velocity values (beta1, beta2, beta3, alpha1, alpha2, alpha3)
     * Sample time is configured via "sample_time_ms" parameter
     */
    void run_velocity_tracking()
    {
        // Step 1: Disable and set to Position mode
        disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ctrlMode(static_cast<int>(CtrlMode::Position));
        RCLCPP_INFO(get_logger(), "Mode -> Position");
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));

        // Step 2: Enable and move to initial position
        call_enable_toggle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        publish_position(m_initial_position);
        RCLCPP_INFO(get_logger(), "Moving to: [β1=%f, β2=%f, α1=%f, α2=%f]",
                    m_initial_position[0], m_initial_position[1],
                    m_initial_position[3], m_initial_position[4]);

        // Wait until initial position is reached
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while (!m_reached)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

        RCLCPP_INFO(get_logger(), "Initial position reached.");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Step 3: Switch to Velocity mode
        disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        ctrlMode(static_cast<int>(CtrlMode::Velocity));
        RCLCPP_INFO(get_logger(), "Mode -> Velocity");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        call_enable_toggle();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Step 4: Load velocity trajectory from CSV
        std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
        ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
        std::filesystem::path file_path = ws_dir / "Shared" / m_velocity_csv;

        std::vector<VelocitySample> velocity_samples;
        try
        {
            velocity_samples = read_velocity_samples_from_csv(file_path.string());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load velocity CSV: %s", e.what());
            disable();
            return;
        }

        if (velocity_samples.empty())
        {
            RCLCPP_ERROR(get_logger(), "No velocity data loaded from CSV. Aborting.");
            disable();
            return;
        }

        // Convert to blaze vectors
        m_velocity_trajectory.clear();
        for (const auto &sample : velocity_samples)
        {
            if (sample.velocities.size() == 6)
            {
                blaze::StaticVector<double, 6> q_dot = {
                    sample.velocities[0], sample.velocities[1], sample.velocities[2],
                    sample.velocities[3], sample.velocities[4], sample.velocities[5]};
                m_velocity_trajectory.push_back(q_dot);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Velocity sample has %zu values, expected 6. Skipping.",
                            sample.velocities.size());
            }
        }

        if (m_velocity_trajectory.empty())
        {
            RCLCPP_ERROR(get_logger(), "No valid velocity waypoints after conversion. Aborting.");
            disable();
            return;
        }

        RCLCPP_INFO(get_logger(), "Starting velocity tracking with %zu waypoints at %.1f ms intervals",
                    m_velocity_trajectory.size(), m_sample_time_ms);

        // Step 5: Send zero velocity initially and start timer
        blaze::StaticVector<double, 6> zero_vel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        publish_velocity(zero_vel);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Step 5: Trigger recording for the computed duration
        double total_duration = 0.0;
        if (!velocity_samples.empty())
        {
            total_duration = velocity_samples.back().timestamp;
        }

        auto record_request = std::make_shared<interfaces::srv::Recording::Request>();
        record_request->command = "start";
        record_request->duration = total_duration + 5.0;
        // auto record_future = m_record_client->async_send_request(record_request, std::bind(&JointPathNode::handle_record_response, this, std::placeholders::_1));
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Recording>::SharedFuture;
        auto response_received_callback = std::bind(&JointPathNode::handle_record_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(record_request, response_received_callback);


        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Reset index and start timer
        m_velocity_idx = 0;
        m_velocity_timer->reset();
        RCLCPP_INFO(get_logger(), "Velocity timer started.");
    }

    // Read initialization points from CSV file
    void read_samples_from_csv(std::vector<blaze::StaticVector<double, 6>> &init_q_list, const std::string &fileName)
    {
        std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
        ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
        std::filesystem::path file_path = ws_dir / "Shared" / fileName;

        // Open the file
        std::ifstream file;
        file.open(file_path, std::ifstream::in);
        if (!file.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        // Clear existing data
        init_q_list.clear();

        std::string line;

        // discard first line (header)
        if (std::getline(file, line))
        { /* optionally log */
        }

        // Read file line by line
        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            std::string value;
            std::vector<double> row;

            // Parse comma-separated values
            while (std::getline(ss, value, ','))
            {
                try
                {
                    row.push_back(std::stod(value));
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to parse value: %s", value.c_str());
                }
            }

            // Check if we have exactly 6 values for our StaticVector
            if (row.size() == 6)
            {
                blaze::StaticVector<double, 6> q_point = {row[0], row[1], row[2], row[3], row[4], row[5]};
                init_q_list.push_back(q_point);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Line does not contain exactly 6 values: %s", line.c_str());
            }
        }

        file.close();
        RCLCPP_INFO(get_logger(), "Loaded %zu points from CSV file: %s", init_q_list.size(), file_path.c_str());
    }

    std::vector<VelocitySample> read_velocity_samples_from_csv(const std::string &filepath)
    {
        std::vector<VelocitySample> samples;
        std::ifstream file(filepath);

        if (!file.is_open())
        {
            throw std::runtime_error("Could not open file: " + filepath);
        }

        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string token;

            VelocitySample sample;
            int col = 0;

            while (std::getline(iss, token, ','))
            {
                if (col == 0)
                {
                    sample.timestamp = std::stod(token);
                }
                else
                {
                    sample.velocities.push_back(std::stod(token));
                }
                col++;
            }

            if (col == 7)
            { // 1 timestamp + 6 velocities
                samples.push_back(sample);
            }
        }

        file.close();
        return samples;
    }

    // ---------- Members ----------
    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subs_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subs_status;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_config_client;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_enable_client;
    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_pub_joint_vel;
    rclcpp::TimerBase::SharedPtr m_read_key_timer;

    rclcpp::Client<interfaces::srv::Recording>::SharedPtr m_record_client;

    rclcpp::CallbackGroup::SharedPtr cb_state_, cb_comm_;

    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_pub_joint_targ;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr cli_cfg_, cli_enable_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::executors::MultiThreadedExecutor *m_executor;

    double m_sample_time_ms{10.0};
    std::string m_velocity_csv{"targetVelocity.csv"};
    blaze::StaticVector<double, 6> m_initial_position{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Velocity tracking members
    rclcpp::TimerBase::SharedPtr m_velocity_timer;
    std::vector<blaze::StaticVector<double, 6>> m_velocity_trajectory;
    size_t m_velocity_idx{0};

    // Params
    double m_vel_mag{0.5};

    std::vector<blaze::StaticVector<double, 6>> m_q_list = {
        {1.0, -0.144, 0.0, 0.0, -0.060, 0.0},
        {2.0, -0.134, 0.5, 0.0, -0.060, 0.0}};

    size_t m_target_idx{0};

    blaze::StaticVector<double, 6> m_q_target;
    std::vector<double> thresholds_ = {M_PI, -1 * M_PI, 0.0}; // for joint 0

    // Feedback/state
    bool m_reached{false};
    bool m_enabled{false};
    blaze::StaticVector<double, 4> m_q{0, 0, 0, 0}, m_dq{0, 0, 0, 0}, m_curr{0, 0, 0, 0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    auto node = std::make_shared<JointPathNode>(&exec);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

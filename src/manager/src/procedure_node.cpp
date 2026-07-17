#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <optional>
#include <atomic>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/status.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/srv/recording.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class CtrlMode : int
{
    Config = 0x00,
    Manual = 0x01,
    Position = 0x02,
    Velocity = 0x03,
};

class ProcedureNode : public rclcpp::Node
{
public:
    ProcedureNode() : rclcpp::Node("ctr_procedure_node")
    {
        ProcedureNode::declare_parameters();

        ProcedureNode::initRosInterfaces();

        m_q_target = m_init_q_list[m_target_idx];

        // Precompute the ping-pong order: 0→1→2→...→(n-1)→(n-2)→...→1, repeat
        // build_pingpong_sequence();

        // Kick off
        m_state = State::INIT;
    }

private:
    void declare_parameters()
    {
        declare_parameter<double>("velocity_magnitude", 0.5); // [rad/s] or [m/s] per joint units
        declare_parameter<double>("reach_tol", 1e-3);         // position reach tolerance for initial target
        declare_parameter<bool>("auto_enable", true);

        m_vel_mag = get_parameter("velocity_magnitude").as_double();
        reach_tol_ = get_parameter("reach_tol").as_double();
        auto_enable_ = get_parameter("auto_enable").as_bool();
    }

    //
    void initRosInterfaces()
    {
        cb_state_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_comm_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions subs_opts;
        subs_opts.callback_group = cb_comm_;

        m_subs_joints = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&ProcedureNode::on_feedback, this, std::placeholders::_1), subs_opts);
        m_subs_status = create_subscription<interfaces::msg::Status>("robot_status", 10, std::bind(&ProcedureNode::on_status, this, std::placeholders::_1), subs_opts);

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

        // ---------------- Timers (state machine) ----------------
        timer_ = create_wall_timer(
            100ms, std::bind(&ProcedureNode::tick, this), cb_state_);

        // RCLCPP_INFO(get_logger(), "Ready. thresholds=%s, loops=%d, joint=%d, |v|=%.5f",
        //             vec_to_str(thresholds_).c_str(), loops_, joint_index_, vel_mag_);
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
        auto response_received_callback = std::bind(&ProcedureNode::handle_service_response, this, std::placeholders::_1);
        auto fut = m_robot_config_client->async_send_request(request, response_received_callback);
        // RCLCPP_INFO(this->get_logger(), "Sent CtrlMode change request: %d", mode);
    }

    void call_enable_toggle()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "toggleEnable";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&ProcedureNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_enable_client->async_send_request(request, response_received_callback);
    }

    void disable()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "disable";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&ProcedureNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_enable_client->async_send_request(request, response_received_callback);
    }

    void publish_position(const blaze::StaticVector<double, 4> &q)
    {
        interfaces::msg::Jointspace msg;
        msg.position[0] = q[0];
        msg.position[1] = q[1];
        msg.position[2] = q[2];
        msg.position[3] = q[3];
        // velocities left at zero
        m_pub_joint_targ->publish(msg);
    }

    void publish_velocity(const blaze::StaticVector<double, 4> &q_dot)
    {
        interfaces::msg::Jointspace msg;
        msg.velocity[0] = q_dot[0];
        msg.velocity[1] = q_dot[1];
        msg.velocity[2] = q_dot[2];
        msg.velocity[3] = q_dot[3];
        msg.position[0] = m_q[0];
        msg.position[1] = m_q[1];
        msg.position[2] = m_q[2];
        msg.position[3] = m_q[3];
        m_pub_joint_targ->publish(msg);
    }

    bool all_enabled_() const
    {
        return m_en[0] && m_en[1] && m_en[2] && m_en[3];
    }

    bool reached_pos() const
    {
        double err = blaze::norm(m_q - m_q_target);
        return err <= reach_tol_;
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
        for (int i = 0; i < 4; ++i)
            m_en[i] = msg->enable[i];
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

    // ---------- State machine ----------
    enum class State
    {
        INIT,
        ENSURE_ENABLED,
        SET_POS_MODE,
        SEND_POS0,
        WAIT_POS0,
        SET_VEL_MODE,
        ROTATION_PING_PONG,
        RUN_VEL_TO_THRESH, // drive until threshold idx = seq_indices_[seq_ptr_]
        DONE
    };

    void tick()
    {
        switch (m_state)
        {
        case State::INIT:
        {
            disable();
            m_state = State::SET_POS_MODE;
            break;
        }

        case State::SET_POS_MODE:
        {
            publish_position(m_q);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            ctrlMode(static_cast<int>(CtrlMode::Position));
            RCLCPP_INFO(get_logger(), "Mode -> Position");
            m_state = State::SEND_POS0;
            break;
        }

        case State::SEND_POS0:
        {
            if (!all_enabled_())
            {
                if (++enable_wait_ticks_ < 10)
                {
                    m_state = State::SEND_POS0;
                    break;
                }
                call_enable_toggle();
                enable_wait_ticks_ = 0; // give it a beat to propagate
            }

            // refresh current target from the list
            if (m_target_idx < m_init_q_list.size())
            {
                m_q_target = m_init_q_list[m_target_idx];
            }

            publish_position(m_q_target);
            std::vector<double> q_target_vec = {m_q_target[0], m_q_target[1], m_q_target[2], m_q_target[3]};
            RCLCPP_INFO(get_logger(), "Position target sent: q_target[%s]", vec_to_str(q_target_vec).c_str());
            pos_wait_ticks_ = 0;
            m_state = State::WAIT_POS0;
            break;
        }

        case State::WAIT_POS0:
        {
            if (reached_pos())
            {
                std::vector<double> q_vec = {m_q[0], m_q[1], m_q[2], m_q[3]};
                RCLCPP_INFO(get_logger(), "Reached position target q[%s]:", vec_to_str(q_vec).c_str());

                m_state = State::SET_VEL_MODE;
                break;
            }
            if (++pos_wait_ticks_ % 50 == 0)
            {                                 // every ~1s
                publish_position(m_q_target); // re-publish target (robustness)
            }
            break;
        }

        case State::SET_VEL_MODE:
        {
            ctrlMode(static_cast<int>(CtrlMode::Velocity));
            RCLCPP_INFO(get_logger(), "Mode -> Velocity");

            // Start Recording
            auto request = std::make_shared<interfaces::srv::Recording::Request>();
            request->command = "start"; // Set the desired command
            request->duration = 0.0; // Set the desired duration
            using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Recording>::SharedFuture;
            auto response_received_callback = std::bind(&ProcedureNode::handle_record_response, this, std::placeholders::_1);
            auto future_result = m_record_client->async_send_request(request, response_received_callback);

            m_state = State::ROTATION_PING_PONG;
            break;
        }

        case State::ROTATION_PING_PONG:
        {
            // (assumes thresholds_.size() >= 3)
            if (thresholds_.size() < 3)
            {
                RCLCPP_ERROR(get_logger(), "Need at least 3 thresholds for ping-pong.");
                m_state = State::DONE;
                break;
            }

            // Initialize the 3-step ping-pong toward thresholds_[0]
            m_thresh_stage = 0;

            // Choose sign to move toward the first threshold
            const double q0 = m_q[0];
            const double th0 = thresholds_[0];
            m_vel_sign0 = (th0 >= q0) ? +1 : -1;

            // Start moving joint 0 at |v| = 0.2
            blaze::StaticVector<double, 4> q_dot_targ{0.0, 0.0, 0.0, 0.0};
            q_dot_targ[0] = m_vel_mag * m_vel_sign0;
            publish_velocity(q_dot_targ);

            m_vel_repub_ticks = 0;
            m_state = State::RUN_VEL_TO_THRESH;
            break;
        }

        case State::RUN_VEL_TO_THRESH:
        {
            // Drive joint 0 at constant speed; flip sign at each threshold crossing.
            if (m_thresh_stage >= 3)
            {
                // finished all three thresholds for THIS initial target
                publish_velocity({0.0, 0.0, 0.0, 0.0}); // stop before switching modes
                // publish_position(m_q);

                if (m_target_idx + 1 < m_init_q_list.size())
                {
                    // move to the next initial target
                    ++m_target_idx;

                    // reset ping-pong counters for the next cycle
                    m_thresh_stage = 0;
                    m_vel_repub_ticks = 0;

                    // go back to position mode -> send next target -> wait -> velocity -> ping-pong
                    m_state = State::SET_POS_MODE;
                }
                else
                {
                    // no more targets
                    m_state = State::DONE;
                }
                break;
            }

            const double q0 = m_q[0];
            const double targ = thresholds_[m_thresh_stage];

            // Check crossing depending on current sign
            const bool crossed = (m_vel_sign0 > 0) ? (q0 >= targ) : (q0 <= targ);

            if (crossed)
            {
                RCLCPP_INFO(get_logger(),
                            "Crossed thresholds_[%d]=%.6f at q0=%.6f",
                            m_thresh_stage, targ, q0);

                ++m_thresh_stage;
                if (m_thresh_stage >= 3)
                {
                    // finished all three thresholds for THIS initial target
                    publish_velocity({0.0, 0.0, 0.0, 0.0}); // stop before switching modes

                    if (m_target_idx + 1 < m_init_q_list.size())
                    {
                        // move to the next initial target
                        ++m_target_idx;

                        // reset ping-pong counters for the next cycle
                        m_thresh_stage = 0;
                        m_vel_repub_ticks = 0;

                        // go back to position mode -> send next target -> wait -> velocity -> ping-pong
                        m_state = State::SET_POS_MODE;

                        // Stop Recording
                        auto request = std::make_shared<interfaces::srv::Recording::Request>();
                        request->command = "stop"; // Set the desired command
                        request->duration = 0.0; // Set the desired duration
                        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Recording>::SharedFuture;
                        auto response_received_callback = std::bind(&ProcedureNode::handle_record_response, this, std::placeholders::_1);
                        auto future_result = m_record_client->async_send_request(request, response_received_callback);


                    }
                    else
                    {
                        // no more targets
                        m_state = State::DONE;
                    }
                    break;
                }
                // Flip sign and continue toward the next threshold
                m_vel_sign0 *= -1;

                blaze::StaticVector<double, 4> q_dot_targ{0.0, 0.0, 0.0, 0.0};
                q_dot_targ[0] = m_vel_mag * m_vel_sign0;
                publish_velocity(q_dot_targ);

                m_vel_repub_ticks = 0;
            }
            else
            {
                // Keep streaming the same velocity for robustness
                if ((++m_vel_repub_ticks % 20) == 0)
                { // ~400ms if tick is 20ms
                    blaze::StaticVector<double, 4> q_dot_targ{0.0, 0.0, 0.0, 0.0};
                    q_dot_targ[0] = m_vel_mag * m_vel_sign0;
                    publish_velocity(q_dot_targ);
                }
            }
            break;
        }

        case State::DONE:
        {
            if (!flag_once_)
            {
                // stop motion; end procedure flag in robot; (optionally) switch to Config or leave as Velocity with zero
                publish_velocity({0.0, 0.0, 0.0, 0.0});
                disable();
                RCLCPP_INFO(get_logger(), "Completed. ");
                flag_once_ = true;
            }
            break;
        }
        } // switch
    }

    // ---------- Members ----------
    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subs_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subs_status;
    // rclcpp::Subscription<interfaces::msg::Interface>::SharedPtr m_subs_interface;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_config_client;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_enable_client;
    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_pub_joint_vel;
    rclcpp::TimerBase::SharedPtr m_read_key_timer;

    rclcpp::Client<interfaces::srv::Recording>::SharedPtr m_record_client;

    rclcpp::CallbackGroup::SharedPtr cb_state_, cb_comm_;

    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_pub_joint_targ;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr cli_cfg_, cli_enable_;
    rclcpp::TimerBase::SharedPtr timer_;

    int m_thresh_stage{0};    // 0 -> thresholds_[0], 1 -> thresholds_[1], 2 -> thresholds_[2]
    int m_vel_sign0{+1};      // velocity sign for joint 0
    int m_vel_repub_ticks{0}; // periodic republish (robustness)
    bool flag_once_{false};
    // Params
    double m_vel_mag{0.5};
    double reach_tol_{1e-3};
    bool auto_enable_{true};

    std::vector<blaze::StaticVector<double, 4>> m_init_q_list = {
        {0.0, -0.156, 0.0, -0.072},
        {0.0, -0.146, 0.0, -0.072},
        {0.0, -0.136, 0.0, -0.072},
        {0.0, -0.126, 0.0, -0.072},
        {0.0, -0.116, 0.0, -0.072},
        {0.0, -0.102, 0.0, -0.072},
        {0.0, -0.144, 0.0, -0.060},
        {0.0, -0.134, 0.0, -0.060},
        {0.0, -0.124, 0.0, -0.060},
        {0.0, -0.114, 0.0, -0.060},
        {0.0, -0.104, 0.0, -0.060},
        {0.0, -0.090, 0.0, -0.060},
        {0.0, -0.134, 0.0, -0.050},
        {0.0, -0.124, 0.0, -0.050},
        {0.0, -0.114, 0.0, -0.050},
        {0.0, -0.104, 0.0, -0.050},
        {0.0, -0.094, 0.0, -0.050},
        {0.0, -0.080, 0.0, -0.050},
        {0.0, -0.124, 0.0, -0.040},
        {0.0, -0.114, 0.0, -0.040},
        {0.0, -0.104, 0.0, -0.040},
        {0.0, -0.094, 0.0, -0.040},
        {0.0, -0.084, 0.0, -0.040},
        {0.0, -0.070, 0.0, -0.040},
        {0.0, -0.118, 0.0, -0.034},
        {0.0, -0.108, 0.0, -0.034},
        {0.0, -0.098, 0.0, -0.034},
        {0.0, -0.088, 0.0, -0.034},
        {0.0, -0.078, 0.0, -0.034},
        {0.0, -0.064, 0.0, -0.034}};

    size_t m_target_idx{0};

    blaze::StaticVector<double, 4> m_q_target;
    std::vector<double> thresholds_ = {M_PI, -1 * M_PI, 0.0}; // for joint 0

    // Feedback/state
    std::array<bool, 4> m_en{false, false, false, false};
    blaze::StaticVector<double, 4> m_q{0, 0, 0, 0}, m_dq{0, 0, 0, 0}, m_curr{0, 0, 0, 0};

    // Ping-pong sequencing over indices into thresholds_
    std::vector<int> m_seq_indices;
    size_t seq_ptr_{0};
    int vel_sign_{+1};

    // State machine
    State m_state{State::INIT};
    int enable_wait_ticks_{10};
    int pos_wait_ticks_{0};
    int vel_repub_ticks_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcedureNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

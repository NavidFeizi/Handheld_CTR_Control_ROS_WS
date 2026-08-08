#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <filesystem>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/force.hpp"
#include "interfaces/srv/recording.hpp"
#include "ctr_common/output_session.hpp"
#include "ctr_common/runtime_paths.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_eigen/tf2_eigen.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// Struct to hold data for each recording session
struct RecordingSession
{
  std::string name;
  std::ofstream robot_file;
  std::ofstream taskspace_file;
  std::ofstream computed_file;  // New: for reference/simulation/force data
  bool is_recording = false;
  double rec_duration = 0.0;
  rclcpp::Time t0;
};

class RecorderNode : public rclcpp::Node
{
public:
  RecorderNode() : Node("recorder"), m_count(0)
  {
    m_clock = this->get_clock();
    m_t0 = m_clock->now();

    RecorderNode::declare_parameters();
    RecorderNode::setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "Recorder node initialized");
  }

private:
  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    declare_parameter<double>("sample_time", 25E-3);
    m_sample_time = get_parameter("sample_time").as_double();
  }

  // Function to set up ROS interfaces including subscriptions, services, and timers
  void setup_ros_interfaces()
  {
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_subs_x = create_subscription<interfaces::msg::Taskspace>("task_space/feedback/base_tool", 20, std::bind(&RecorderNode::update_x_callback, this, _1));
    m_subs_x_r = create_subscription<interfaces::msg::Taskspace>("task_space/target", 20, std::bind(&RecorderNode::update_x_r_callback, this, _1));
    m_subs_x_sim = create_subscription<interfaces::msg::Taskspace>("task_space/sim_out", 20, std::bind(&RecorderNode::update_x_sim_callback, this, _1));
    m_subs_q = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&RecorderNode::update_q_callback, this, _1));
    m_subs_q_r = create_subscription<interfaces::msg::Jointspace>("joint_space/target", 10, std::bind(&RecorderNode::update_q_r_callback, this, _1));
    m_subs_ctrl_time = create_subscription<std_msgs::msg::Float64>("mpc/computation_time", 10, std::bind(&RecorderNode::update_ctrl_time_callback, this, _1));
    
    // Subscribe to force estimation from EKF
    m_subs_force = create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&RecorderNode::update_force_callback, this, _1));

    /// listener to tf2 transormation messages
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf2_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    m_callback_group_srv = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    m_service = this->create_service<interfaces::srv::Recording>("recording", std::bind(&RecorderNode::handle_recording, this, _1, _2));

    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_record_timer = this->create_wall_timer(sample_time, std::bind(&RecorderNode::dump_callback, this), m_callback_group_read);
  }

  // Subscriber callback function to update current robot joints space
  void update_q_callback(const interfaces::msg::Jointspace::SharedPtr msg)
  {
    for (int i = 0; i < 4; i++)
    {
      m_q[i] = msg->position[i];
      m_q_dot[i] = msg->velocity[i];
      m_current[i] = msg->current[i];
    }
  }

  void update_q_r_callback(const interfaces::msg::Jointspace::SharedPtr msg)
  {
    for (int i = 0; i < 4; i++)
    {
      m_q_r[i] = msg->position[i];
      m_q_r_dot[i] = msg->velocity[i];
    }
  }

  void update_x_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x[0] = msg->p[0];
    m_x[1] = msg->p[1];
    m_x[2] = msg->p[2];
    m_x_dot[0] = msg->q[0];
    m_x_dot[1] = msg->q[1];
    m_x_dot[2] = msg->q[2];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  void update_x_r_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_r[0] = msg->p[0];
    m_x_r[1] = msg->p[1];
    m_x_r[2] = msg->p[2];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  void update_x_sim_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_sim[0] = msg->p[0];
    m_x_sim[1] = msg->p[1];
    m_x_sim[2] = msg->p[2];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  void update_ctrl_time_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
  {
    m_ctrl_time = msg->data;
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  void update_force_callback(const interfaces::msg::Force::ConstSharedPtr msg)
  {
    m_force_est[0] = msg->x;
    m_force_est[1] = msg->y;
    m_force_est[2] = msg->z;
    m_force_magnitude = msg->magnitude;
  }

  // Function to capture a single frame of data and write to dump files for a specific session
  void capture_frame(RecordingSession& session)
  {
    const auto now = m_clock->now();
    const double t = (now - session.t0).seconds();

    capture_robot_data(session, t);
    capture_taskspace_data(session, t);
    capture_computed_data(session, t);
  }

  // Function to capture and write robot joint-space data to file
  void capture_robot_data(RecordingSession& session, double timestamp)
  {
    session.robot_file << std::fixed << std::setprecision(3)
                      << timestamp << ","
                      << std::fixed << std::setprecision(6)
                      << m_q[0] << ',' << m_q[1] << ',' << m_q[2] << ',' << m_q[3] << ','
                      << m_q_dot[0] << ',' << m_q_dot[1] << ',' << m_q_dot[2] << ',' << m_q_dot[3] << ','
                      << m_current[0] << ',' << m_current[1] << ',' << m_current[2] << ',' << m_current[3]
                      << ',' << m_q_r[0] << ',' << m_q_r[1] << ',' << m_q_r[2] << ',' << m_q_r[3] << ','
                      << m_q_r_dot[0] << ',' << m_q_r_dot[1] << ',' << m_q_r_dot[2] << ',' << m_q_r_dot[3]
                      << '\n';
    session.robot_file.flush();
  }

  // Function to capture and write computed data (reference, simulation, force estimate) to file
  void capture_computed_data(RecordingSession& session, double timestamp)
  {
    session.computed_file << std::fixed << std::setprecision(3) << timestamp << ","
                          << std::fixed << std::setprecision(6)
                          << m_x_r[0] << ',' << m_x_r[1] << ',' << m_x_r[2] << ','
                          << m_x_sim[0] << ',' << m_x_sim[1] << ',' << m_x_sim[2] << ','
                          << m_force_est[0] << ',' << m_force_est[1] << ',' << m_force_est[2] << ','
                          << m_force_magnitude << ','
                          << m_ctrl_time << '\n';
    session.computed_file.flush();
  }

  // Function to capture and write task-space data (EM tracker sensors) to file
  void capture_taskspace_data(RecordingSession& session, double timestamp)
  {
    session.taskspace_file << std::fixed << std::setprecision(3) << timestamp << ",";

    // Lambda to capture and write transform data (position + quaternion)
    auto write_transform = [&](const std::string& frame_name) {
      geometry_msgs::msg::TransformStamped tf_stamped;
      if (lookup_transform("robot_base", frame_name, tf_stamped))
      {
        Eigen::Matrix4d eigen_trans = tf2::transformToEigen(tf_stamped).matrix();
        Eigen::Matrix3d rotation_matrix = eigen_trans.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation_matrix);

        session.taskspace_file << std::setprecision(5)
                               << eigen_trans(0, 3) << ',' << eigen_trans(1, 3) << ',' << eigen_trans(2, 3) << ','
                               << quaternion.w() << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z() << ',';
      }
      else
      {
        // Write NaN if transform not available
        session.taskspace_file << "NaN,NaN,NaN,NaN,NaN,NaN,NaN,";
      }
    };

    // Capture ctr_tip transform
    write_transform("ctr_tip");

    // Capture sensor transforms
    write_transform("sensor_1");
    write_transform("sensor_2");
    write_transform("sensor_3");

    session.taskspace_file << '\n';
    session.taskspace_file.flush();
  }

  // Helper function to lookup TF2 transforms with error handling
  bool lookup_transform(const std::string& target_frame, const std::string& source_frame, 
                       geometry_msgs::msg::TransformStamped& transform)
  {
    try
    {
      transform = m_tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      // Only log occasionally to avoid spam
      static std::map<std::string, rclcpp::Time> last_warning_time;
      auto now = m_clock->now();
      std::string key = target_frame + "->" + source_frame;
      
      if (last_warning_time.find(key) == last_warning_time.end() || 
          (now - last_warning_time[key]).seconds() > 5.0)
      {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                   target_frame.c_str(), source_frame.c_str(), ex.what());
        last_warning_time[key] = now;
      }
      return false;
    }
  }

  

  // Function to create and open dump files for a specific recording session
  bool create_dump_files(const std::string& session_name)
  {
    const std::filesystem::path output_base =
        ctr_common::resolveDataRoot(*this, "manager") / "Output_Files";
    const std::string folder_address = ctr_common::makeSessionDir(output_base, session_name).string();
    RCLCPP_INFO(this->get_logger(), "[%s] Recording session directory: %s", session_name.c_str(), folder_address.c_str());

    // Create new session
    RecordingSession& session = m_sessions[session_name];
    session.name = session_name;
    session.t0 = m_clock->now();
    
    // Open Robot.csv
    session.robot_file.open(folder_address + "/" + "Robot.csv");
    if (!session.robot_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to open Robot.csv", session_name.c_str());
      return false;
    }
    session.robot_file << "t,q1,q2,q3,q4,q1_dot,q2_dot,q3_dot,q4_dot,c1,c2,c3,c4";
    session.robot_file << ",q_r1,q_r2,q_r3,q_r4,q_r1_dot,q_r2_dot,q_r3_dot,q_r4_dot";
    session.robot_file << "\n";

    // Open EMtracker.csv
    session.taskspace_file.open(folder_address + "/" + "EMtracker.csv");
    if (!session.taskspace_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to open EMtracker.csv", session_name.c_str());
      return false;
    }
    session.taskspace_file << "t,x,y,z,qw,qx,qy,qz";
    session.taskspace_file << ",s1_x,s1_y,s1_z,s1_qw,s1_qx,s1_qy,s1_qz";
    session.taskspace_file << ",s2_x,s2_y,s2_z,s2_qw,s2_qx,s2_qy,s2_qz";
    session.taskspace_file << ",s3_x,s3_y,s3_z,s3_qw,s3_qx,s3_qy,s3_qz";
    session.taskspace_file << "\n";

    // Open Computed.csv
    session.computed_file.open(folder_address + "/" + "Computed.csv");
    if (!session.computed_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to open Computed.csv", session_name.c_str());
      return false;
    }
    session.computed_file << "t,x_r,y_r,z_r,x_sim,y_sim,z_sim";
    session.computed_file << ",f_x,f_y,f_z,f_mag,ctrl_time";
    session.computed_file << "\n";

    RCLCPP_INFO(this->get_logger(), "[%s] Dump files opened - folder: %s", session_name.c_str(), folder_address.c_str());
    return true;
  }

  // Timer callback function to periodically save data to dump files for all active sessions
  void dump_callback()
  {
    m_t = m_clock->now();
    
    // Iterate through all sessions
    for (auto& [name, session] : m_sessions)
    {
      if (!session.robot_file.is_open() || !session.taskspace_file.is_open() || !session.computed_file.is_open())
        continue;
        
      rclcpp::Duration duration = m_t - session.t0;
      double time = static_cast<double>(duration.nanoseconds()) / 1E9;

      if (time < session.rec_duration)
      {
        if (!session.is_recording)
        {
          session.is_recording = true;
          RCLCPP_INFO(this->get_logger(), "[%s] Recording started - Sampling time: %0.1f [ms]", 
                     name.c_str(), m_sample_time * 1e3);
        }
        capture_frame(session);
      }
      else
      {
        if (session.is_recording)
        {
          session.is_recording = false;
          RCLCPP_INFO(this->get_logger(), "[%s] Recording stopped - Time: %.2f [s]", name.c_str(), time);

          session.robot_file.close();
          session.taskspace_file.close();
          session.computed_file.close();
          RCLCPP_INFO(this->get_logger(), "[%s] Dump files closed", name.c_str());
        }
      }
    }
  }

  // Service callback to handle recording requests for named sessions
  void handle_recording(const std::shared_ptr<interfaces::srv::Recording::Request> request, 
                       std::shared_ptr<interfaces::srv::Recording::Response> response)
  {
    std::string session_name = request->name.empty() ? "default" : request->name;
    
    if (request->command == "start")
    {
      if (!create_dump_files(session_name))
      {
        response->success = false;
        response->message = "[" + session_name + "] Failed to create dump files";
        return;
      }
      
      auto& session = m_sessions[session_name];
      response->success = true;
      if (request->duration > 0)
      {
        session.rec_duration = request->duration;
        session.t0 = m_clock->now();
        response->message = "[" + session_name + "] Recording started for " + std::to_string(session.rec_duration) + " seconds";
      }
      else
      {
        session.rec_duration = std::numeric_limits<double>::infinity();
        session.t0 = m_clock->now();
        response->message = "[" + session_name + "] Recording started until stopped by user";
      }
    }
    else if (request->command == "stop")
    {
      auto it = m_sessions.find(session_name);
      if (it != m_sessions.end())
      {
        it->second.rec_duration = 0.0;
        response->success = true;
        response->message = "[" + session_name + "] Recording stopped by user";
      }
      else
      {
        response->success = false;
        response->message = "[" + session_name + "] Session not found";
      }
    }
    else if (request->command == "init")
    {
      if (create_dump_files(session_name))
      {
        response->success = true;
        response->message = "[" + session_name + "] Dump files created";
      }
      else
      {
        response->success = false;
        response->message = "[" + session_name + "] Failed to create dump files";
      }
    }
    else if (request->command == "capture")
    {
      auto it = m_sessions.find(session_name);
      if (it != m_sessions.end() && it->second.robot_file.is_open())
      {
        for (int i = 0; i < 4; i++)
        {
          capture_frame(it->second);
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        response->success = true;
        response->message = "[" + session_name + "] 4 frames captured";
      }
      else
      {
        response->success = false;
        response->message = "[" + session_name + "] Session not found or files not open. Use 'init' first.";
      }
    }
    else if (request->command == "close")
    {
      auto it = m_sessions.find(session_name);
      if (it != m_sessions.end())
      {
        it->second.robot_file.close();
        it->second.taskspace_file.close();
        RCLCPP_INFO(this->get_logger(), "[%s] Dump files closed", session_name.c_str());
        m_sessions.erase(it);  // Remove session from map
        response->success = true;
        response->message = "[" + session_name + "] Dump files closed";
      }
      else
      {
        response->success = false;
        response->message = "[" + session_name + "] Session not found";
      }
    }
    else
    {
      response->success = false;
      response->message = "Unknown command: " + request->command + ". Use 'start', 'stop', 'init', 'capture', or 'close'.";
    }

    return;
  }

  size_t m_count;
  double m_sample_time;
  double m_time = 0.0;

  // Map to store multiple recording sessions by name
  std::map<std::string, RecordingSession> m_sessions;

  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Time m_t0{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_t{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr m_record_timer;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_1;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_2;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subs_x, m_subs_x_r, m_subs_x_sim;
  
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_4;
  rclcpp::Service<interfaces::srv::Recording>::SharedPtr m_service;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subs_q, m_subs_q_r;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_subs_ctrl_time;
  rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_subs_force;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_srv;
  rclcpp::TimerBase::SharedPtr m_tf2_timer;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf2_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_broadcast;

  blaze::StaticVector<double, 4UL> m_current = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 4UL> m_q, m_q_dot = blaze::StaticVector<double, 4UL>(0.0);  
  blaze::StaticVector<double, 4UL> m_q_r, m_q_r_dot = blaze::StaticVector<double, 4UL>(0.0);   // reference trajectory 
  blaze::StaticVector<double, 3UL> m_x, m_x_dot = blaze::StaticVector<double, 3UL>(0.0);
  blaze::StaticVector<double, 3UL> m_x_r = blaze::StaticVector<double, 3UL>(0.0);
  blaze::StaticVector<double, 3UL> m_x_sim = blaze::StaticVector<double, 3UL>(0.0);
  blaze::StaticVector<double, 3UL> m_force_est = blaze::StaticVector<double, 3UL>(0.0);

  double m_ctrl_time = 0.0;
  double m_force_magnitude = 0.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecorderNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
};
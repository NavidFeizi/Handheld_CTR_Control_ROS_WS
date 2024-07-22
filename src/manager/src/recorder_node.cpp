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
#include "interfaces/srv/startrecording.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

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
    declare_parameter<double>("sample_time", 4E-3);
    m_sample_time = get_parameter("sample_time").as_double();
  }

  // Function to set up ROS interfaces including subscriptions, services, and timers
  void setup_ros_interfaces()
  {
    m_callback_group_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_4 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_5 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto options_1 = rclcpp::SubscriptionOptions();
    auto options_2 = rclcpp::SubscriptionOptions();
    auto options_3 = rclcpp::SubscriptionOptions();
    auto options_4 = rclcpp::SubscriptionOptions();
    auto options_5 = rclcpp::SubscriptionOptions();

    options_1.callback_group = m_callback_group_1;
    options_2.callback_group = m_callback_group_2;
    options_3.callback_group = m_callback_group_3;
    options_3.callback_group = m_callback_group_4;
    options_3.callback_group = m_callback_group_5;

    m_subscription_1 = this->create_subscription<interfaces::msg::Jointspace>(
        "JointsActuation", rclcpp::QoS(10), std::bind(&RecorderNode::desired_joints_config_callback, this, _1), options_1);
    m_subscription_2 = this->create_subscription<interfaces::msg::Jointspace>(
        "JointSpaceStatus", rclcpp::QoS(10), std::bind(&RecorderNode::joints_status_callback, this, _1), options_2);
    m_subscription_3 = this->create_subscription<interfaces::msg::Taskspace>(
        "EMTracker", rclcpp::QoS(10), std::bind(&RecorderNode::catheter_tip_callback, this, _1), options_3);
    m_subscription_4 = this->create_subscription<interfaces::msg::Taskspace>(
        "EMTracker_org", rclcpp::QoS(10), std::bind(&RecorderNode::catheter_tip_org_callback, this, _1), options_3);

    m_service = this->create_service<interfaces::srv::Startrecording>(
        "start_recording", std::bind(&RecorderNode::handle_start_recording, this, _1, std::placeholders::_2));

    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_save_timer = this->create_wall_timer(sample_time, std::bind(&RecorderNode::dump_callback, this), m_callback_group_read);
  }

  // Subscriber callback function to update desired joint positions and velocities
  void desired_joints_config_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    m_q[0UL] = msg->position[0UL];
    m_q[1UL] = msg->position[1UL];
    m_q[2UL] = msg->position[2UL];
    m_q[3UL] = msg->position[3UL];

    m_q_dot[0UL] = msg->velocity[0UL];
    m_q_dot[1UL] = msg->velocity[1UL];
    m_q_dot[2UL] = msg->velocity[2UL];
    m_q_dot[3UL] = msg->velocity[3UL];
    // RCLCPP_INFO(this->get_logger(), "New target");
  }

  // Subscriber callback function to update current robot joints status
  void joints_status_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    m_joint_space_pos_abs[0UL] = msg->position_abs[0UL];
    m_joint_space_pos_abs[1UL] = msg->position_abs[1UL];
    m_joint_space_pos_abs[2UL] = msg->position_abs[2UL];
    m_joint_space_pos_abs[3UL] = msg->position_abs[3UL];

    m_joint_space_pos[0UL] = msg->position[0UL];
    m_joint_space_pos[1UL] = msg->position[1UL];
    m_joint_space_pos[2UL] = msg->position[2UL];
    m_joint_space_pos[3UL] = msg->position[3UL];

    m_joint_space_vel[0UL] = msg->velocity[0UL];
    m_joint_space_vel[1UL] = msg->velocity[1UL];
    m_joint_space_vel[2UL] = msg->velocity[2UL];
    m_joint_space_vel[3UL] = msg->velocity[3UL];

    // current[0UL] = msg->current[0UL];
    // current[1UL] = msg->current[1UL];
    // current[2UL] = msg->current[2UL];
    // current[3UL] = msg->current[3UL];
    // current[4UL] = msg->current[4UL];
    // current[5UL] = msg->current[5UL];

    // RCLCPP_INFO(this->get_logger(), "New joint space status");
  }

  // Subscriber callback function to update catheter tip position from the em tracker
  void catheter_tip_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_task_space_pos[0UL] = msg->p[0UL];
    m_task_space_pos[1UL] = msg->p[1UL];
    m_task_space_pos[2UL] = msg->p[2UL];
    m_task_space_vel[0UL] = msg->q[0UL];
    m_task_space_vel[1UL] = msg->q[1UL];
    m_task_space_vel[2UL] = msg->q[2UL];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  // Subscriber callback function to update catheter tip origianl (non filtered) position and velocity from the em tracker
  void catheter_tip_org_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_task_space_pos_nf[0UL] = msg->p[0UL];
    m_task_space_pos_nf[1UL] = msg->p[1UL];
    m_task_space_pos_nf[2UL] = msg->p[2UL];
    m_task_space_vel_nf[0UL] = msg->q[0UL];
    m_task_space_vel_nf[1UL] = msg->q[1UL];
    m_task_space_vel_nf[2UL] = msg->q[2UL];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  // Function to create and open dump files for recording
  void create_dump_files()
  {
    std::string package_name = "publisher";
    std::string workspace_directory = ament_index_cpp::get_package_share_directory(package_name);
    auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    std::string folder_address = workspace_directory + "/../../../../Output_Files/" + ss.str();
    std::filesystem::create_directories(folder_address);

    m_robot_dump_file.open(folder_address + "/" + "Robot.csv");
    if (!m_robot_dump_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Robot.csv");
      return;
    }
    m_robot_dump_file << "t,q0,q1,q2,q3,q4,q5,"
                      << "q0_dot,q1_dot,q2_dot,q3_dot,q4_dot,q5_dot,"
                      << "p0,p1,p2,p3,p4,p5,"
                      << "p0_abs,p1_abs,p2_abs,p3_abs,p4_abs,p5_abs,"
                      << "v0,v1,v2,v3,v4,v5,"
                      << "c0,c1,c2,c3,c4,c5\n";

    m_ft_sensor_dump_file.open(folder_address + "/" + "FTsensor.csv");
    if (!m_ft_sensor_dump_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open FTsensor.csv");
      return;
    }
    m_ft_sensor_dump_file << "t,Fx,Fy,Fz,Tx,Ty,Tz\n";

    m_em_tracker_dump_file.open(folder_address + "/" + "EMtracker.csv");
    if (!m_em_tracker_dump_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open EMtracker.csv");
      return;
    }
    m_em_tracker_dump_file << "t,p_x,p_y,p_z,v_x,v_y,v_z,p_x_nf,p_y_nf,p_z_nf,v_x_nf,v_y_nf,v_z_nf\n";

    RCLCPP_INFO(this->get_logger(), "Dump files opened - folder name: %s", ss.str().c_str());
  }

  // Timer callback function to periodically save data to dump files
  void dump_callback()
  {
    m_t = m_clock->now();
    rclcpp::Duration duration = m_t - m_t0;
    double time = static_cast<double>(duration.nanoseconds()) / 1.00E9;

    if (time < m_rec_duration)
    {
      if (!m_flag_recording)
      {
        m_flag_recording = true;
        RCLCPP_INFO(this->get_logger(), "Recording started - Sampling time: %0.1f [ms]", m_sample_time * 1.00E3);
      };
      m_robot_dump_file << std::fixed << std::setprecision(3)
                        << time << ","
                        << std::fixed << std::setprecision(6)
                        << m_q[0UL] << ',' << m_q[1UL] << ',' << m_q[2UL] << ',' << m_q[3UL] << ',' << m_q[4UL] << ',' << m_q[5UL] << ','
                        << m_q_dot[0UL] << ',' << m_q_dot[1UL] << ',' << m_q_dot[2UL] << ',' << m_q_dot[3UL] << ',' << m_q_dot[4UL] << ',' << m_q_dot[5UL] << ','
                        << m_joint_space_pos[0UL] << ',' << m_joint_space_pos[1UL] << ',' << m_joint_space_pos[2UL] << ',' << m_joint_space_pos[3UL] << ',' << m_joint_space_pos[4UL] << ',' << m_joint_space_pos[5UL] << ','
                        << m_joint_space_pos_abs[0UL] << ',' << m_joint_space_pos_abs[1UL] << ',' << m_joint_space_pos_abs[2UL] << ',' << m_joint_space_pos_abs[3UL] << ',' << m_joint_space_pos_abs[4UL] << ',' << m_joint_space_pos_abs[5UL] << ','
                        << std::fixed << std::setprecision(11)
                        << m_joint_space_vel[0UL] << ',' << m_joint_space_vel[1UL] << ',' << m_joint_space_vel[2UL] << ',' << m_joint_space_vel[3UL] << ',' << m_joint_space_vel[4UL] << ',' << m_joint_space_vel[5UL] << ','
                        << std::fixed << std::setprecision(4)
                        << m_current[0UL] << ',' << m_current[1UL] << ',' << m_current[2UL] << ',' << m_current[3UL] << ',' << m_current[4UL] << ',' << m_current[5UL] << '\n';

      m_ft_sensor_dump_file << std::fixed << std::setprecision(3)
                            << time << ","
                            << std::setprecision(2)
                            << m_force_torque[0UL] << ',' << m_force_torque[1UL] << ',' << m_force_torque[2UL] << ','
                            << std::setprecision(4)
                            << m_force_torque[3UL] << ',' << m_force_torque[4UL] << ',' << m_force_torque[5UL] << '\n';

      m_em_tracker_dump_file << std::fixed << std::setprecision(3)
                             << time << ","
                             << std::setprecision(5)
                             << m_task_space_pos[0UL] << ',' << m_task_space_pos[1UL] << ',' << m_task_space_pos[2UL] << ',' 
                             << m_task_space_vel[0UL] << ',' << m_task_space_vel[1UL] << ',' << m_task_space_vel[2UL] << ',' 
                             << m_task_space_pos_nf[0UL] << ',' << m_task_space_pos_nf[1UL] << ',' << m_task_space_pos_nf[2UL] << ',' 
                             << m_task_space_vel_nf[0UL] << ',' << m_task_space_vel_nf[1UL] << ',' << m_task_space_vel_nf[2UL] <<'\n';
    }
    else
    {
      if (m_flag_recording)
      {
        m_flag_recording = false;
        RCLCPP_INFO(this->get_logger(), "Recording stopped - Time: %.2f [s]", time);

        m_robot_dump_file.close();
        m_ft_sensor_dump_file.close();
        m_em_tracker_dump_file.close();
        RCLCPP_INFO(this->get_logger(), "Dump files closed");
      };
    }
  }

  // Service callback to handle start recording requests
  void handle_start_recording(const std::shared_ptr<interfaces::srv::Startrecording::Request> &request,
                              std::shared_ptr<interfaces::srv::Startrecording::Response> &response)
  {
    RecorderNode::create_dump_files();
    response->success = true;
    m_t0 = m_clock->now();
    m_rec_duration = request->duration;
    response->message = "Recording started";
  }

  size_t m_count;
  double m_sample_time;
  double m_rec_duration, m_time = 0.00;
  bool m_flag_recording = false;

  std::ofstream m_robot_dump_file;
  std::ofstream m_ft_sensor_dump_file;
  std::ofstream m_em_tracker_dump_file;

  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Time m_t0{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_t{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr m_save_timer;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_1;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_2;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_3;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_4;
  rclcpp::Service<interfaces::srv::Startrecording>::SharedPtr m_service;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_2;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_3;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_4;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_5;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;

  blaze::StaticVector<double, 6UL> m_joint_space_pos_abs;
  blaze::StaticVector<double, 6UL> m_joint_space_pos;
  blaze::StaticVector<double, 6UL> m_joint_space_vel;
  blaze::StaticVector<double, 6UL> m_current;
  blaze::StaticVector<double, 6UL> m_q;
  blaze::StaticVector<double, 6UL> m_q_dot;
  blaze::StaticVector<double, 3UL> m_task_space_pos, m_task_space_vel;
  blaze::StaticVector<double, 3UL> m_task_space_pos_nf, m_task_space_vel_nf; // No filter
  blaze::StaticVector<double, 6UL> m_force_torque;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecorderNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
};
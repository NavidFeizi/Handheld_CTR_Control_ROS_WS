#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/startrecording.hpp"
#include "interfaces/action/target.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &filePath);

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode() : Node("manager"), count_(0)
  {
    ManagerNode::declare_parameters();
    ManagerNode::setup_ros_interfaces();

    std::string fileName("Trajectory.csv");
    ManagerNode::load_input_files(m_trajectory, fileName);

    ManagerNode::send_recod_request();

    RCLCPP_INFO(this->get_logger(), "Manager node initialized");
  }

  ~ManagerNode()
  {
    if (m_is_experiment_running)
    {
      m_is_experiment_running = false;
      if (m_experiment_thread.joinable())
      {
        m_experiment_thread.join();
      }
    }
  }

private:
  // member variables
  size_t count_;
  double m_sample_time = 1e-3; //[s]
  double t0_, m_t, t0 = 0;

  double m_expt_time = 30.0;
  double m_x, m_x_prev = 0.0;

  std::vector<double> m_traj;
  long unsigned int m_traj_row = 0;

  blaze::HybridMatrix<double, 60000UL, 13UL> m_trajectory;
  // std::unique_ptr<LinearDecreaseGenerator> generator;

  std::unique_ptr<ButterworthFilter<1>> m_filter;

  interfaces::msg::Jointspace m_msg;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub;
  rclcpp::Client<interfaces::srv::Startrecording>::SharedPtr m_record_client;
  rclcpp_action::Client<interfaces::action::Target>::SharedPtr m_client;

  std::atomic<bool> m_is_experiment_running;
  std::thread m_experiment_thread;

  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    declare_parameter<double>("sample_time", 4E-3);
    m_sample_time = get_parameter("sample_time").as_double();
  }

  // Function to set up ROS interfaces including subscriptions, services, and timers
  void setup_ros_interfaces()
  {
    m_callback_group_pub = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_publisher = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation", 10); // to directly actuate the robot

    // Recorder service
    m_record_client = this->create_client<interfaces::srv::Startrecording>("start_recording");
    while (!m_record_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Record service not available, waiting again...");
    }

    auto dt = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_timer = this->create_wall_timer(dt, std::bind(&ManagerNode::target_joints_callback, this), m_callback_group_pub);

    // Target action
    m_client = rclcpp_action::create_client<interfaces::action::Target>(this, "navigate_to_target");
    if (!m_client->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }
    // Setup callbacks for action client
    auto m_send_goal_options = rclcpp_action::Client<interfaces::action::Target>::SendGoalOptions();
    m_send_goal_options.feedback_callback = std::bind(&ManagerNode::action_feedback_callback, this, _1);
    m_send_goal_options.result_callback = std::bind(&ManagerNode::action_result_callback, this, _1);
  }

  // Function to load input CSV files
  void load_input_files(blaze::HybridMatrix<double, 60000UL, 13UL> &trajectory, const std::string &fileName)
  {
    // /** read actuation trajectory */
    std::string package_name = "publisher"; // Replace with any package in your workspace
    std::string workspace_directory = ament_index_cpp::get_package_share_directory(package_name);
    // std::string fileName("Trajectory");
    std::string filePath = workspace_directory + "/../../../../Trajectories/" + fileName;
    std::cout << "ROS workspace directory: " << filePath << std::endl;
    readFromCSV(trajectory, filePath);
    std::cout << "--- Trajectory loaded ---" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  //
  void send_recod_request()
  {
    auto request = std::make_shared<interfaces::srv::Startrecording::Request>();
    request->duration = 30.0; // Set the desired duration

    using ServiceResponseFuture =
        rclcpp::Client<interfaces::srv::Startrecording>::SharedFuture;
    auto response_received_callback = std::bind(&ManagerNode::handle_recod_response, this, std::placeholders::_1);

    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

  //
  void handle_recod_response(rclcpp::Client<interfaces::srv::Startrecording>::SharedFuture future)
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

  void send_goal()
  {
    using namespace std::placeholders;

    auto goal_msg = my_robot_interfaces::action::Target::Goal();
    goal_msg.p[0] = target[0];
    goal_msg.p[1] = target[1];
    goal_msg.p[2] = target[2];

    m_client->async_send_goal(goal_msg, m_send_goal_options);
  }

  //
  void action_feedback_callback(
      rclcpp_action::ClientGoalHandle<interfaces::action::Target>::SharedPtr,
      const std::shared_ptr<const interfaces::action::Target::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance to target: %.2f, Status: %s", feedback->distance_to_target, feedback->current_status.c_str());
  }

  void action_result_callback(const rclcpp_action::ClientGoalHandle<interfaces::action::Target>::WrappedResult &result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Target reached successfully");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach target");
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// function that reads relevant clinical data from CSV files for each case
template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &filePath)
{
  // std::string filePath, file;

  // // relative path to the folder containing clinical data
  // filePath = "../../Trajectories/";
  // file = trajectoryName + ".csv";

  // file from which the information will be read from
  std::ifstream CSV_file;
  std::cout << filePath << std::endl;
  CSV_file.open((filePath).c_str(), std::ifstream::in);
  if (!CSV_file.is_open())
  {
    std::cerr << "Error opening the CSV file within: " << __PRETTY_FUNCTION__ << "\n";
    return Mat = -1.00;
  }

  typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

  std::string line;

  size_t row = 0UL, col = 0UL;
  double value;

  while (std::getline(CSV_file, line))
  {
    Tokenizer tokenizer(line);
    col = 0UL;

    for (Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
    {
      value = std::stod(*it);
      Mat(row, col) = value;
      ++col;
    }
    ++row;
  }

  CSV_file.close();
  Mat.resize(row, col, true);

  return Mat;
}
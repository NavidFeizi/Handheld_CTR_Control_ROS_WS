#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <boost/tokenizer.hpp>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/startrecording.hpp"
#include "interfaces/action/target.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

std::string package_name = "manager"; // Replace with any package in your workspace
std::string PACKAGE_SHARE_DIR = ament_index_cpp::get_package_share_directory(package_name);

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::filesystem::path &filePath);

class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode() : Node("manager"), count_(0)
  {
    ManagerNode::declare_parameters();
    ManagerNode::setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "Manager node initialized");

    // initialize output file
    std::string output_fileName = "output.csv";
    std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
    ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
    std::filesystem::path filePath = ws_dir / "Output_Files" / output_fileName;
    m_output_file.open(filePath, std::ios::out);
    m_output_file << "Targ_X,Targ_Y,Targ_Z, Phan_Tool_X, Phan_Tool_Y, Phan_Tool_Z, Base_Tool_X, Base_Tool_Y, Base_Tool_Z\n";
    RCLCPP_INFO(this->get_logger(), "--- Output file initialized ---");

    // load input file
    std::string input_fileName("Targets.csv");
    ManagerNode::load_input_files(m_targets, input_fileName);
    RCLCPP_INFO(this->get_logger(), "Target files loaded");

    // ManagerNode::send_recod_request();

    ManagerNode::send_next_target();
  }

  ~ManagerNode()
  {
    if (m_output_file.is_open())
    {
      m_output_file.close();
    }
  }

private:
  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declare_parameters()
  {
    declare_parameter<double>("sample_time", 4.00E-3);
    m_sample_time = get_parameter("sample_time").as_double();
  }

  // Function to set up ROS interfaces including subscriptions, services, and timers
  void setup_ros_interfaces()
  {
    // m_publisher = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation", 10); // to directly actuate the robot

    // m_callback_group_pub = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    // m_timer = this->create_wall_timer(sample_time, std::bind(&ManagerNode::target_joints_callback, this), m_callback_group_pub);

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

    // Target action
    m_client = rclcpp_action::create_client<interfaces::action::Target>(this, "navigate_to_target");
    if (!m_client->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    }

    // Tool position subscriptions
    m_callback_group_1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto options_1 = rclcpp::SubscriptionOptions();
    auto options_2 = rclcpp::SubscriptionOptions();
    options_1.callback_group = m_callback_group_1;
    options_2.callback_group = m_callback_group_2;
    m_subscription_tool_1 = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_base_tool", rclcpp::QoS(10), std::bind(&ManagerNode::base_tool_callback, this, _1), options_1);
    m_subscription_tool_2 = this->create_subscription<interfaces::msg::Taskspace>(
        "emt_phantom_tool", rclcpp::QoS(10), std::bind(&ManagerNode::phantom_tool_callback, this, _1), options_2);
  }

  // Function to load input CSV files
  void load_input_files(blaze::HybridMatrix<double, 200UL, 6UL> &targets, const std::string &fileName)
  {
    std::filesystem::path ws_dir(PACKAGE_SHARE_DIR);
    ws_dir = ws_dir.parent_path().parent_path().parent_path().parent_path();
    std::filesystem::path filePath = ws_dir / "Input_Files" / fileName;

    readFromCSV(targets, filePath);
    RCLCPP_INFO(this->get_logger(), "--- Targets loaded ---");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  // Record
  void send_recod_request()
  {
    auto request = std::make_shared<interfaces::srv::Startrecording::Request>();
    request->duration = 30.00; // Set the desired duration

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

  //
  void send_next_target()
  {
    if (m_current_target_index >= m_targets.rows())
    {
      RCLCPP_INFO(this->get_logger(), "All targets have been processed.");
      return;
    }

    auto goal_msg = interfaces::action::Target::Goal();
    goal_msg.target_pose[0UL] = m_targets(m_current_target_index, 3) * -1;
    goal_msg.target_pose[1UL] = m_targets(m_current_target_index, 4) * -1;
    goal_msg.target_pose[2UL] = m_targets(m_current_target_index, 5);

    goal_msg.calyx_pose[0UL] = m_targets(m_current_target_index, 0);
    goal_msg.calyx_pose[1UL] = m_targets(m_current_target_index, 1);
    goal_msg.calyx_pose[2UL] = m_targets(m_current_target_index, 2);

    auto send_goal_options = rclcpp_action::Client<interfaces::action::Target>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&ManagerNode::action_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ManagerNode::action_result_callback, this, _1);

    m_client->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Task target #%ld sent", m_current_target_index);
  }

  //
  void action_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<interfaces::action::Target>> goal_handle, const std::shared_ptr<const interfaces::action::Target::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance to target: %.2f, Status: %s", feedback->distance_to_target, feedback->current_status.c_str());
  }

  //
  void action_result_callback(const rclcpp_action::ClientGoalHandle<interfaces::action::Target>::WrappedResult &result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Target reached successfully");

      RCLCPP_INFO(this->get_logger(), "Collecting tool position");
      blaze::StaticVector<double, 3UL> sum_base_tool_pos = blaze::StaticVector<double, 3UL>(0.0);
      blaze::StaticVector<double, 3UL> sum_phantom_tool_pos = blaze::StaticVector<double, 3UL>(0.0);

      int num_samples = 100;
      for (int i = 0; i < num_samples; ++i)
      {
        // Assuming you update these vectors elsewhere in your program
        sum_base_tool_pos += m_base_tool_pos;
        sum_phantom_tool_pos += m_phantom_tool_pos;

        // Wait for 26 milliseconds before the next reading
        std::this_thread::sleep_for(std::chrono::milliseconds(26));
      }

      // Calculate the average position for each tool
      blaze::StaticVector<double, 3UL> avg_base_tool_pos = sum_base_tool_pos / num_samples;
      blaze::StaticVector<double, 3UL> avg_phantom_tool_pos = sum_phantom_tool_pos / num_samples;

      m_output_file << m_targets(m_current_target_index, 0) << ","
                    << m_targets(m_current_target_index, 1) << ","
                    << m_targets(m_current_target_index, 2) << ","
                    << avg_phantom_tool_pos[0] << ","
                    << avg_phantom_tool_pos[1] << ","
                    << avg_phantom_tool_pos[2] << ","
                    << avg_base_tool_pos[0] << ","
                    << avg_base_tool_pos[1] << ","
                    << avg_base_tool_pos[2] << std::endl;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach target");
    }
    ++m_current_target_index; // Move to the next target
    send_next_target();       // Send the next target
  }

  // Subscriber callback function to update catheter tip position from the em tracker
  void base_tool_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_base_tool_pos[0UL] = msg->p[0UL];
    m_base_tool_pos[1UL] = msg->p[1UL];
    m_base_tool_pos[2UL] = msg->p[2UL];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  // Subscriber callback function to update catheter tip position from the em tracker
  void phantom_tool_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_phantom_tool_pos[0UL] = msg->p[0UL];
    m_phantom_tool_pos[1UL] = msg->p[1UL];
    m_phantom_tool_pos[2UL] = msg->p[2UL];
    // RCLCPP_INFO(this->get_logger(), "New tip");
  }

  // member variables
  size_t count_;
  double m_sample_time = 1.00E-3; //[s]
  double m_t, m_t0 = 0.00;

  std::vector<double> m_traj;
  long unsigned int m_traj_row = 0;

  std::ofstream m_output_file;

  size_t m_current_target_index = 0;
  blaze::HybridMatrix<double, 200UL, 6UL> m_targets;
  blaze::StaticVector<double, 3UL> m_base_tool_pos, m_phantom_tool_pos;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher;
  rclcpp::Client<interfaces::srv::Startrecording>::SharedPtr m_record_client;
  rclcpp_action::Client<interfaces::action::Target>::SharedPtr m_client;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_tool_1;
  rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_tool_2;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_2;
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
MatrixType readFromCSV(MatrixType &Mat, const std::filesystem::path &filePath)
{
  std::ifstream CSV_file;
  CSV_file.open(filePath, std::ifstream::in);
  if (!CSV_file.is_open())
  {
    throw std::runtime_error("Error opening the CSV file: " + filePath.string());
  }

  typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

  std::string line;

  // Skip the first line (header)
  if (!std::getline(CSV_file, line))
  {
    std::cerr << "Error reading the header line\n";
    return Mat = -1.00;
  }

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
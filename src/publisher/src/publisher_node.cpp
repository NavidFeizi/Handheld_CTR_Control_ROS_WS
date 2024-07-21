#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <signal.h>
#include <random>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <boost/tokenizer.hpp>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/startrecording.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "signal_generator.cpp"
#include "Butterworth.hpp"

using namespace std::chrono_literals;

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &filePath);

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher"), count_(0)
  {
    PublisherNode::declare_parameters();
    PublisherNode::setup_ros_interfaces();

    // std::string fileName("Trajectory.csv");
    // PublisherNode::load_input_files(m_trajectory, fileName);

    double fc = 10.0;
    m_filter = std::make_unique<ButterworthFilter<1>>(m_sample_time);
    m_filter->update_coeffs(fc);

    // PublisherNode::send_recod_request();

    m_expt_time = 30.0;
    int num_expt = 50;
    std::thread(&PublisherNode::dataset_expt, this, num_expt).detach();

    rclcpp::Time now = this->get_clock()->now();
    t0_ = static_cast<double>(now.nanoseconds()) / 1E9;
    m_t = 0.0;

    RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
  }

  ~PublisherNode()
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

    // m_publisher = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation_target", 10);     // to actuate the task space jarget generator
    m_publisher = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation", 10); // to directly actuate the robot

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
    m_timer = this->create_wall_timer(dt, std::bind(&PublisherNode::target_joints_callback, this), m_callback_group_pub);
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
    auto response_received_callback = std::bind(&PublisherNode::handle_recod_response, this, std::placeholders::_1);

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

  // Run the experiment with random inputs signals multiple times for dataset generation
  void dataset_expt(int num_expt)
  {
    m_is_experiment_running = true;
    TrajectoryType traj_type = TrajectoryType::MultiSine; // or TrajectoryType::LinearDecrease   - CyclycNearPulse, MultiSine
    // sample_time, num_waves , min_frequency[Hz], max_frequency[Hz] , min_amplitude, max_amplitude[m], total_samples
    MultiSineParams multiSineParams{m_sample_time, 7, 0.0, 2.0, 0.0, 0.006, m_expt_time};
    LinearDecreaseParams linearDecreaseParams{10.0, 50, 100, 50, 1000};
    // sample_time[s]; max_amplitude[m]; rise_duration[s]; unforced_time[s]; fall_vel[m/s]; cycle_period[s]; total_time[s];
    MultiCycleParams multiCycleParams{m_sample_time, 0.005, 0.350, 1.0, 0.25, 2.0, m_expt_time};

    std::unique_ptr<TrajectoryGenerator> m_signal_generator = createTrajectoryGenerator(traj_type, multiSineParams, linearDecreaseParams, multiCycleParams);

    RCLCPP_INFO(this->get_logger(), "Dataset collecting experiment started");

    for (int i = 0; i < num_expt; ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Running experiment %d/%d", i + 1, num_expt);
      m_signal_generator->gen_trajectory();
      m_signal_generator->flip_trajectory_sign();
      m_traj = m_signal_generator->get_trajectory();
      PublisherNode::send_recod_request();
      m_traj_row = 0;
      while (m_traj_row < m_traj.size())
      {
        wait(4);
      }
      wait(1000);
    }

    RCLCPP_INFO(this->get_logger(), "Dataset collecting experiment finished");
  }

  // Timer callback function to publish the target joints config
  void target_joints_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    m_t = m_t + m_sample_time;

    // PublisherNode::multi_sine(m_t, m_x);

    // constexpr double period = 5.0;     // Period of the pulse signal
    // constexpr double amplitude = 5.0; // [mm] Amplitude of the pulse signal
    // constexpr double velocity = 60.0;  // [mm/s] Velocity of the ramp rise and fall
    // PublisherNode::pulse_signal(period, amplitude, velocity, m_t, x);

    if (m_traj_row < m_traj.size())
    {
      blaze::StaticVector<double, 1> x_temp = {m_traj[m_traj_row] * 1e3};
      x_temp = m_filter->add_data_point(x_temp);
      m_x = x_temp[0];
      ++m_traj_row;
    }

    // calcualte velocity signal
    double x_dot = (m_x - m_x_prev) / m_sample_time;
    m_x_prev = m_x;

    m_msg.position[0UL] = m_x * 1e-3;
    m_msg.position[1UL] = 0.0;
    m_msg.position[2UL] = -1 * m_x * 1e-3;
    m_msg.position[3UL] = 0.0;
    m_msg.velocity[0UL] = x_dot * 1e-3; // Scaling the velocity similarly to position
    m_msg.velocity[1UL] = 0.0;
    m_msg.velocity[2UL] = -1 * x_dot * 1e-3;
    m_msg.velocity[3UL] = 0.0;

    m_publisher->publish(m_msg);

    std::cout << "new target: ";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << m_msg.position[0UL] << std::endl;
  }

  //
  void multi_sine(const double t, double &x)
  {
    double frequency = 0.1;
    double amplitude = 5.0;

    x = 1.0 * amplitude * (std::sin(2 * M_PI * frequency * 1.0 * t - M_PI / 2)); // Sinusoidal value at time t
    // target += 0.3 * amplitude * (std::sin(2 * M_PI * frequency * 1.3 * t - M_PI / 2));        // Additional sinusoidal value at time t
    // target += 0.2 * amplitude * (std::sin(2 * M_PI * frequency * 2.3 * t - M_PI / 2));        // Additional sinusoidal value at time t
  }

  // Generates a pulse-like signal with smooth transitions
  void pulse_signal(const double period, const double amplitude, const double velocity, const double t, double &x)
  {

    double half_period = period / 2.0;
    double ramp_time = amplitude / velocity; // Time taken for the ramp to rise or fall

    // Calculate the time within the current period
    double mod_time = fmod(t, period);

    // Generate the signal
    if (mod_time < ramp_time)
    {
      x = (velocity * mod_time); // Rising ramp
    }
    else if (mod_time >= ramp_time && mod_time < half_period)
    {
      x = amplitude; // High state
    }
    else if (mod_time >= half_period && mod_time < (half_period + ramp_time))
    {
      x = amplitude - velocity * (mod_time - half_period); // Falling ramp
    }
    else if (mod_time >= (half_period + ramp_time) && mod_time < period)
    {
      x = 0; // Low state
    }
    else
    {
      // Rising ramp again (since we are in the next period's ramp-up phase)
      x = (velocity * (mod_time - (period - ramp_time)));
    }
  }

  // // step and publish random actuation
  // void stepRandomActuation()
  // {
  //   auto message = interfaces::msg::Jointspace();
  //   // get current node time
  //   rclcpp::Time now = this->get_clock()->now();
  //   double t = static_cast<double>(now.nanoseconds()) / 1E9;
  //   double sim_time = t - t0_;

  //   double q = generator_->get_next_signal();

  //   message.tension[0UL] = q;
  //   message.tension[1UL] = 0.0;
  //   message.tension[2UL] = 0.0;
  //   message.tension[3UL] = 0.0;

  //   m_publisher->publish(message);

  // }

  // Wait in milliseconds
  void wait(int milliseconds)
  {
    rclcpp::Rate rate(1000.0 / milliseconds);
    rate.sleep();
  }

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

  std::atomic<bool> m_is_experiment_running;
  std::thread m_experiment_thread;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherNode>();
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
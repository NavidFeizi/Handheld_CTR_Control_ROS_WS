#include <chrono>
#include <blaze/Blaze.h>
#include <iostream>
#include <string>
#include <iomanip> 
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/taskspace.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

void print_tip(double t, blaze::StaticVector<double, 3UL> position, blaze::StaticVector<double, 2UL> q);

class TrajectoryNode : public rclcpp::Node
{
protected:
  static constexpr size_t m = 3UL;
  static constexpr size_t N = 6UL;
  static constexpr size_t h = 10UL;

public:
  TrajectoryNode() : Node("mpc")
  {
    this->declareParameters();
    this->setupdynamicparameterupdates();
    this->setupRosInterfaces();

    // node initialization time
    rclcpp::Time now = this->get_clock()->now();
    m_t_init = static_cast<double>(now.nanoseconds()) / 1.00E9;
  }

private:
  // Function to declare and initialize parameters - parameters values should be set from the launch file
  void declareParameters()
  {
    declare_parameter<double>("sample_time", 50E-3);
    m_sample_time = get_parameter("sample_time").as_double();

    declare_parameter<double>("omega_theta", 0.25);
    m_omega_th = get_parameter("omega_theta").as_double();

    declare_parameter<double>("omega_radius", 2.0);
    m_omega_r = get_parameter("omega_radius").as_double();

    declare_parameter<double>("omega_z", 1.5);
    m_omega_z = get_parameter("omega_z").as_double();

    declare_parameter<double>("radius_base", 0.040);
    m_radius_base = get_parameter("radius_base").as_double();

    declare_parameter<double>("radius_amp", 0.015);
    m_radius_amp = get_parameter("radius_amp").as_double();

    declare_parameter<double>("z_amp", 0.005);
    m_z_amp = get_parameter("z_amp").as_double();

    declare_parameter<std::vector<double>>("p_centre", {0.0, 0.0, 0.125});
    std::vector<double> m_p_centre = get_parameter("p_centre").as_double_array();
  }

  // Function to setup dynamic parameter callback
  void setupdynamicparameterupdates()
  {  
    auto param_callback =
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "omega_theta")
        {
          m_omega_th = parameter.as_double();
        }
        else if (parameter.get_name() == "omega_radius")
        {
          m_omega_r = parameter.as_double();
        }
        else if (parameter.get_name() == "omega_z")
        {
          m_omega_z = parameter.as_double();
        }
        else if (parameter.get_name() == "radius_base")
        {
          m_radius_base = parameter.as_double();
        }
        else if (parameter.get_name() == "radius_amp")
        {
          m_radius_amp = parameter.as_double();
        }
        else if (parameter.get_name() == "z_amp")
        {
          m_z_amp = parameter.as_double();
        }
        else if (parameter.get_name() == "p_centre")
        {
          auto values = parameter.as_double_array();
          if (values.size() != 3)
          {
            result.successful = false;
            result.reason = parameter.get_name() + " must contain " + std::to_string(3) + " elements.";
            continue;
          }
          for (size_t i = 0; i < 3; ++i)
          {
            m_p_centre[i] = values[i];
          }
        }
        else if (parameter.get_name() == "sample_time")
        {
          // Note: sample_time changes require recreating the timer
          result.successful = false;
          result.reason = "sample_time cannot be changed dynamically. Restart node to apply changes.";
        }
        else
        {
          result.successful = false;
          result.reason = "Parameter " + parameter.get_name() + " not recognized.";
        }
      }
      return result;
    };
    m_param_callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  // Publisher, Subscribers, and Timers
  void setupRosInterfaces()
  {
    // publisher to publish reference signal
    m_publisher_reference = this->create_publisher<interfaces::msg::Taskspace>("/task_space/target", 10);

    // Create wall timers with different callback groups
    m_callback_group_ref = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto dt = std::chrono::microseconds(static_cast<int>(m_sample_time * 1.00E6));
    m_ref_timer = this->create_wall_timer(dt, std::bind(&TrajectoryNode::trajectory_gen, this), m_callback_group_ref);
  }

  // radially and vertically modulated circular trajectory generator loop
  void trajectory_gen()
  {
    if (m_first_call)
    {
      m_t_init = static_cast<double>(this->get_clock()->now().nanoseconds()) / 1.00E9;
      m_first_call = false;
    }

    double t = (static_cast<double>(this->get_clock()->now().nanoseconds()) / 1.00E9) - m_t_init;

    // Center position
    const double x0 = m_p_centre[0], y0 = m_p_centre[1], z0 = m_p_centre[2];

    const double theta0 = m_omega_th * t;
    const double theta0_r = m_omega_r * t;
    const double theta0_z = m_omega_z * t;

    // Hold the same target over the horizon (use k = 0 values)
    const double th = theta0;
    const double th_r = theta0_r;
    const double th_z = theta0_z;

    const double theta =  -1.0 * M_PI +  0.5 * (1.0 + std::sin(-M_PI / 2.0 + th)) * M_PI * 1.0;

    const double rad = m_radius_base + m_radius_amp * std::sin(th_r);

    double x = x0 + rad * std::cos(theta);
    double y = y0 + rad * std::sin(theta);
    double z = z0 + m_z_amp * std::sin(th_z);
  
    auto msg = interfaces::msg::Taskspace();
    msg.p[0] = x;
    msg.p[1] = y;
    msg.p[2] = z;
    m_publisher_reference->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "| Ref: %0.3f, %0.3f, %0.3f m", x, y, z);
  }

  double m_radius_base = 0.040;     // [m]
  double m_radius_amp = 0.015;   // [m]
  double m_z_amp = 0.015;   // [m]
  double m_omega_th = 0.25;  // [rad/s]
  double m_omega_r = 2.0; // [rad/s]
  double m_omega_z = 1.5; // [rad/s]
  std::vector<double> m_p_centre = {0.0, 0.0, 0.130}; // [m]

  double m_t_init = 0.00;
  double m_sample_time;
  
  std::string m_model_name;
  
  blaze::StaticVector<double, N> m_q; 

  bool m_first_call = true;
  blaze::StaticMatrix<double, h, m, blaze::columnMajor> m_xRef;

  blaze::StaticVector<double, m> m_x;
  blaze::StaticVector<double, m> m_ref;

  rclcpp::TimerBase::SharedPtr m_sim_timer;               // timer object to step simulation
  rclcpp::TimerBase::SharedPtr m_ref_timer;           // timer object to states publisher
  rclcpp::TimerBase::SharedPtr m_feedback_timer;          // timer object to update feedback
  rclcpp::CallbackGroup::SharedPtr m_callback_group_ref; // callback grounp for running simulation callback function on separate thread                         
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_reference;       // publisher object

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}


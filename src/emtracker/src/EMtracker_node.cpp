#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <blaze/Blaze.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "EMTracker.hpp"
// #include "butterworth.hpp"

#include "igtlOSUtil.h"
#include "igtlPointMessage.h"
#include "igtlTransformMessage.h"
#include "igtlClientSocket.h"

#include "interfaces/msg/taskspace.hpp"

using namespace std::chrono_literals;

class EMTrackerNode : public rclcpp::Node
{
protected:
public:
  EMTrackerNode()
      : Node("emtracker"), count_(0)
  {
    // Set default parameteres and allow it to be overridden by a launch file or command line parameter
    this->declare_parameter<double>("sample_time", 25E-3);
    m_sample_time = this->get_parameter("sample_time").as_double();
    this->declare_parameter<double>("cutoff_freq", 6.6);
    m_cutoff_freq = this->get_parameter("cutoff_freq").as_double();
    this->declare_parameter<bool>("send_on_igtl", false);
    m_flag_igtl = this->get_parameter("send_on_igtl").as_bool();

    m_filter = std::make_unique<ButterworthFilter<3UL>>(m_sample_time);
    m_filter->update_coeffs(m_cutoff_freq);

    std::string hostname = "/dev/ttyUSB0";
    EMTrackerNode::setup_emtracker(hostname);
    if (m_flag_igtl)
    {
      EMTrackerNode::setup_igtl();
    }
    EMTrackerNode::setup_ros_interfaces();
    EMTrackerNode::setup_parameters_callback();
  }

  ~EMTrackerNode()
  {
    // Destructor will automatically close the hardware connection
  }

private:
  void setup_ros_interfaces()
  {
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_publisher_base = this->create_publisher<interfaces::msg::Taskspace>("emt_base_tool", 10);
    m_publisher_phantom = this->create_publisher<interfaces::msg::Taskspace>("emt_phantom_tool", 10);
    m_publisher_phantom_base = this->create_publisher<interfaces::msg::Taskspace>("emt_phantom_base", 10);

    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_timer = this->create_wall_timer(
        sample_time, std::bind(&EMTrackerNode::read_callback, this), m_callback_group_read);

    // Service server for freeze_phantom using std_srvs/SetBool
    m_freeze_phantom_service = this->create_service<std_srvs::srv::SetBool>(
        "freeze_phantom",
        std::bind(&EMTrackerNode::handle_freeze_phantom, this, std::placeholders::_1, std::placeholders::_2));
  }

  void setup_parameters_callback()
  {
    // Set up parameter callback
    auto param_callback =
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "cutoff_freq")
        {
          m_cutoff_freq = parameter.as_double();
          // m_emt->set_filter_params(0.5, m_cutoff_freq);
          m_filter->update_coeffs(m_cutoff_freq);
        }
      }
      // Update control logic if parameters are successfully updated
      if (result.successful)
      {
        RCLCPP_INFO(this->get_logger(), "Updated filter cutoff: %.2f Hz", m_cutoff_freq);
      }
      return result;
    };
    param_callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  void setup_emtracker(std::string hostname)
  {
    /** initialize emtracker **/
    double cutoff_freq = 30.0; //[Hz]
    bool debug_mode = false;
    m_emt = std::make_unique<EMTracker>(hostname, m_sample_time, cutoff_freq, debug_mode); // Allocate the object dynamically

    // // landmark registration process - uncomment only if you want to redo landmark registration
    // std::string landmarks = "landmarks_truth_ctr_robot.csv";
    // std::string ref_sensor_name = "robot"; // "robot", "phantom", "tool"
    // m_emt->landmark_registration(landmarks, ref_sensor_name);
    // EMTrackerNode::~EMTrackerNode();

    //
    m_emt->start_read_thread();
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  }

  void setup_igtl()
  {
    char *hostname = "localhost";
    int port = 18944;

    std::cout << hostname << std::endl;

    this->m_socket = igtl::ClientSocket::New();
    int r = this->m_socket->ConnectToServer(hostname, port);

    if (r != 0)
    {
      std::cerr << "Cannot connect to the server." << std::endl;
      exit(0);
    }

    m_pm_emtracker = igtl::PointMessage::New();
    m_pm_emtracker->SetDeviceName("emtracker");

    m_trans_probe = igtl::TransformMessage::New();
    m_trans_probe->SetDeviceName("emt_probe");

    m_trans_robot = igtl::TransformMessage::New();
    m_trans_robot->SetDeviceName("emt_robot");

    m_trans_tool = igtl::TransformMessage::New();
    m_trans_tool->SetDeviceName("emt_tool");

    // setting up the points
    m_pe_tool = igtl::PointElement::New();
    m_pe_probe = igtl::PointElement::New();
    m_pe_base = igtl::PointElement::New();

    std::cerr << "IGTLink is ready" << std::endl;
  }

  void read_callback()
  {
    auto msg_base = interfaces::msg::Taskspace();
    auto msg_phantom = interfaces::msg::Taskspace();
    auto msg_phantom_base = interfaces::msg::Taskspace();

    // get current time
    rclcpp::Time now = this->get_clock()->now();

    double sample_time;
    quatTransformation tool_transform_in_robot, tool_transform_dot_in_robot;

    blaze::StaticVector<double, 3UL> tool_pos_flt, tool_vel_flt = blaze::StaticVector<double, 3UL>(0.0);

    m_emt->get_tool_transform_in_robot(tool_transform_in_robot);
    m_emt->get_tool_transform_in_robot_dot(tool_transform_dot_in_robot);
    m_emt->get_tool_transform_in_phantom(m_tool_transform);
    m_emt->get_robot_transform_in_phantom(m_robot_transform);
    m_emt->get_probe_transform_in_phantom(m_probe_transform);

    m_emt->get_sample_time(sample_time);

    tool_pos_flt = m_filter->add_data_point(m_tool_transform.translation);
    tool_vel_flt = (tool_pos_flt - m_tool_pos_flt_prev) / m_sample_time;
    m_tool_pos_flt_prev = tool_pos_flt;

    msg_phantom.p[0] = m_tool_transform.translation[0]; // to align with cathter robot system
    msg_phantom.p[1] = m_tool_transform.translation[1];
    msg_phantom.p[2] = m_tool_transform.translation[2];

    msg_base.p[0] = tool_transform_in_robot.translation[0]; // to align with cathter robot system
    msg_base.p[1] = tool_transform_in_robot.translation[1];
    msg_base.p[2] = tool_transform_in_robot.translation[2];

    msg_phantom_base.p[0] = m_robot_transform.translation[0];
    msg_phantom_base.p[1] = m_robot_transform.translation[1];
    msg_phantom_base.p[2] = m_robot_transform.translation[2];
    msg_phantom_base.h[0] = m_robot_transform.rotation[0];
    msg_phantom_base.h[1] = m_robot_transform.rotation[1];
    msg_phantom_base.h[2] = m_robot_transform.rotation[2];
    msg_phantom_base.h[3] = m_robot_transform.rotation[3];

    // msg.p_phantom_probe[0] = m_probe_transform.translation[0]; // to align with cathter robot system
    // msg.p_phantom_probe[1] = m_probe_transform.translation[1];
    // msg.p_phantom_probe[2] = m_probe_transform.translation[2];

    m_publisher_phantom->publish(msg_phantom);
    m_publisher_base->publish(msg_base);
    m_publisher_phantom_base->publish(msg_phantom_base);

    // send on IGTLink
    if (m_flag_igtl)
    {
      // EMTrackerNode::igtl_points_callback();
      EMTrackerNode::igtl_probe_tran_callback();
      EMTrackerNode::igtl_robot_tran_callback();
      EMTrackerNode::igtl_tool_tran_callback();
    }

    // double time = static_cast<double>(now.nanoseconds()) / 1E9;
    // log_position(time, tool_transform.translation, sample_time);
    // log_position(time, Pos_tip_F, SampleTime);
  }

  void igtl_points_callback()
  {
    m_pm_emtracker->ClearPointElement();
    // tool in phantom frame
    m_pe_tool->SetName("tool");
    m_pe_tool->SetRadius(5.0);
    m_pe_tool->SetRGBA(100, 100, 0, 255);
    m_pe_tool->SetPosition(m_tool_transform.translation[0] * 1e3,
                           m_tool_transform.translation[1] * 1e3,
                           m_tool_transform.translation[2] * 1e3);

    // probe in phantom frame
    m_pe_probe->SetName("probe");
    m_pe_probe->SetRadius(5.0);
    m_pe_probe->SetRGBA(100, 100, 0, 255);
    m_pe_probe->SetPosition(m_probe_transform.translation[0] * 1e3,
                            m_probe_transform.translation[1] * 1e3,
                            m_probe_transform.translation[2] * 1e3);

    // ctr base in phantom frame
    m_pe_base->SetName("base");
    m_pe_base->SetRadius(5.0);
    m_pe_base->SetRGBA(100, 100, 0, 255);
    m_pe_base->SetPosition(m_robot_transform.translation[0] * 1e3,
                           m_robot_transform.translation[1] * 1e3,
                           m_robot_transform.translation[2] * 1e3);

    // pack point into the point message
    m_pm_emtracker->AddPointElement(m_pe_tool);
    m_pm_emtracker->AddPointElement(m_pe_probe);
    m_pm_emtracker->AddPointElement(m_pe_base);
    m_pm_emtracker->Pack();

    // Send the message
    m_socket->Send(m_pm_emtracker->GetPackPointer(), m_pm_emtracker->GetPackSize());
    // RCLCPP_INFO(this->get_logger(), "IGTL Sent");
  }

  void igtl_probe_tran_callback()
  {
    igtl::Matrix4x4 trans;
    // igtl::IdentityMatrix(trans);
    blaze::StaticMatrix<double, 4, 4> tran_matrix_probe = m_probe_transform.toMatrix();
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        trans[i][j] = tran_matrix_probe(i, j); // Access elements assuming Blaze uses zero-based indexing
      }
    }
    // convert m to mm
    trans[0][3] = trans[0][3] * 1e3;
    trans[1][3] = trans[1][3] * 1e3;
    trans[2][3] = trans[2][3] * 1e3;

    m_trans_probe->SetMatrix(trans);
    m_trans_probe->Pack();

    // Send the message
    m_socket->Send(m_trans_probe->GetPackPointer(), m_trans_probe->GetPackSize());
    // RCLCPP_INFO(this->get_logger(), "IGTL Sent");
  }

  void igtl_robot_tran_callback()
  {
    igtl::Matrix4x4 trans;
    blaze::StaticMatrix<double, 4, 4> tran_matrix_robot = m_robot_transform.toMatrix();
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        trans[i][j] = tran_matrix_robot(i, j); // Access elements assuming Blaze uses zero-based indexing
      }
    }
    // convert m to mm
    trans[0][3] = trans[0][3] * 1e3;
    trans[1][3] = trans[1][3] * 1e3;
    trans[2][3] = trans[2][3] * 1e3;

    m_trans_robot->SetMatrix(trans);
    m_trans_robot->Pack();

    // Send the message
    m_socket->Send(m_trans_robot->GetPackPointer(), m_trans_robot->GetPackSize());
    // RCLCPP_INFO(this->get_logger(), "IGTL Sent");
  }

  void igtl_tool_tran_callback()
  {
    igtl::Matrix4x4 trans;
    blaze::StaticMatrix<double, 4, 4> tran_matrix_robot = m_tool_transform.toMatrix();
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        trans[i][j] = tran_matrix_robot(i, j); // Access elements assuming Blaze uses zero-based indexing
      }
    }
    // convert m to mm
    trans[0][3] = trans[0][3] * 1e3;
    trans[1][3] = trans[1][3] * 1e3;
    trans[2][3] = trans[2][3] * 1e3;

    m_trans_tool->SetMatrix(trans);
    m_trans_tool->Pack();

    // Send the message
    m_socket->Send(m_trans_tool->GetPackPointer(), m_trans_tool->GetPackSize());
    // RCLCPP_INFO(this->get_logger(), "IGTL Sent");
  }

  void handle_freeze_phantom(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      m_emt->freeze_phantom(true);
      response->success = true;
      response->message = "Phantom frozen";
      RCLCPP_INFO(this->get_logger(), "Phantom is frozen");
    }
    else
    {
      m_emt->freeze_phantom(false);
      response->success = true;
      response->message = "Phantom defrozen";
      RCLCPP_INFO(this->get_logger(), "Phantom is active");
    }
  }

  void log_position(blaze::StaticVector<double, 3> position, double sample_time)
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

    oss << "X:" << print_with_space_if_positive(position[0]) << "  "
        << "Y:" << print_with_space_if_positive(position[1]) << "  "
        << "Z:" << print_with_space_if_positive(position[2]) << " [m]"
        << "  |  dT:" << std::fixed << std::setprecision(1) << sample_time * 1e3 << " [ms]";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  size_t count_;
  double m_sample_time;
  double m_cutoff_freq;
  bool m_flag_igtl;
  std::unique_ptr<EMTracker> m_emt;
  std::unique_ptr<ButterworthFilter<3UL>> m_filter;
  blaze::StaticVector<double, 3UL> m_tool_pos_flt_prev = blaze::StaticVector<double, 3UL>(0.0);
  quatTransformation m_tool_transform, m_robot_transform, m_probe_transform;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_base, m_publisher_phantom, m_publisher_phantom_base;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_heartbeat_publisher;
  rclcpp::TimerBase::SharedPtr m_timer_heartbeat;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_heartbeat;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_freeze_phantom_service;

  igtl::ClientSocket::Pointer m_socket;
  igtl::PointMessage::Pointer m_pm_emtracker;
  igtl::TransformMessage::Pointer m_trans_probe, m_trans_robot, m_trans_tool;
  igtl::PointElement::Pointer m_pe_tool, m_pe_probe, m_pe_base;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EMTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

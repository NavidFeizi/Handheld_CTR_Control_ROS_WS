#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <blaze/Blaze.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/transformation.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "igtlOSUtil.h"
#include "igtlPointMessage.h"
#include "igtlTransformMessage.h"
#include "igtlClientSocket.h"
#include "EMTracker.hpp"

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

    this->declare_parameter<bool>("enable_position_logging", true);
    m_flag_log_position = this->get_parameter("enable_position_logging").as_bool();

    this->declare_parameter<std::string>("host_name", "/dev/ttyUSB0");
    std::string host_name = this->get_parameter("host_name").as_string();

    m_filter = std::make_unique<ButterworthFilter<3UL>>(m_sample_time);
    m_filter->update_coeffs(m_cutoff_freq);
    
    EMTrackerNode::setup_emtracker(host_name);
    if (m_flag_igtl)
    {
      std::string host_name = "localhost";
      int port = 18944;
      EMTrackerNode::setup_igtl(host_name, port);
    }
    EMTrackerNode::setup_ros_interfaces();
    EMTrackerNode::setup_parameters_callback();
  }

  ~EMTrackerNode()
  {
    // Destructor will automatically close the hardware connection
  }

private:

  /// @brief Resolve the writable EM-tracker config/data directory. Parameter
  /// "emtracker_data_dir" wins; default is $HOME/Documents/handheld_CTR/
  /// emtracker_config/. On first run the installed read-only seeds
  /// (share/emtracker/config) are copied in; existing files are never
  /// overwritten, so registration results and edited configs survive.
  std::string resolveConfigDir()
  {
    std::string dir = this->declare_parameter<std::string>("emtracker_data_dir", "");
    if (dir.empty())
    {
      const char *home = std::getenv("HOME");
      dir = std::string(home != nullptr ? home : ".") + "/Documents/handheld_CTR/emtracker_config/";
    }

    std::filesystem::create_directories(dir);
    const std::filesystem::path seeds =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("emtracker")) / "config";
    if (std::filesystem::exists(seeds))
    {
      std::filesystem::copy(seeds, dir,
                            std::filesystem::copy_options::recursive |
                                std::filesystem::copy_options::skip_existing);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No installed config seeds found at %s", seeds.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "EM tracker config directory: %s", dir.c_str());
    return dir;
  }

  /// @brief Function to set up ROS interfaces including subscriptions, services, and timers
  void setup_ros_interfaces()
  {
    // Publishers for broadcasting the sensor readings
    m_publisher_base = this->create_publisher<interfaces::msg::Taskspace>("task_space/feedback/base_tool", 10);
    m_publisher_phantom = this->create_publisher<interfaces::msg::Taskspace>("task_space/feedback/phantom_tool", 10);
    m_publisher_phantom_base = this->create_publisher<interfaces::msg::Taskspace>("task_space/feedback/phantom_base", 10);

    // Broadcast all sensors readings usign tf2
    m_tf2_broadcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer to read emtracker data periodically
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_timer = this->create_wall_timer(
        sample_time, std::bind(&EMTrackerNode::read_callback, this), m_callback_group_read);

    // Service server for freeze_phantom using std_srvs/SetBool
    m_freeze_phantom_service = this->create_service<std_srvs::srv::SetBool>(
        "freeze_phantom",
        std::bind(&EMTrackerNode::handle_freeze_phantom, this, std::placeholders::_1, std::placeholders::_2));

    // Service server for freeze_robot using std_srvs/SetBool
    m_freeze_robot_service = this->create_service<std_srvs::srv::SetBool>(
        "freeze_robot",
        std::bind(&EMTrackerNode::handle_freeze_robot, this, std::placeholders::_1, std::placeholders::_2));

    // Service server for geting the CT-SCAN -> CTR tranformation using tranformation.hpp
    m_ctr_tranform_service = this->create_service<interfaces::srv::Transformation>(
        "get_transformation",
        std::bind(&EMTrackerNode::handle_tranformation_service_request, this, std::placeholders::_1, std::placeholders::_2));
  }

  // Function to set up parameter callback for parameters dynamic configurations
  void setup_parameters_callback()
  {
    // may need to be updated
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

  // setup emtracker and start the reading thread
  void setup_emtracker(std::string host_name)
  {
    /** initialize emtracker **/
    double cutoff_freq = 30.0; //[Hz]
    bool debug_mode = false;
    const std::string config_dir = resolveConfigDir();
    m_emt = std::make_unique<EMTracker>(host_name, m_sample_time, cutoff_freq, debug_mode, config_dir); // Allocate the object dynamically

    // // landmark registration process - uncomment only if you want to redo landmark registration
    // std::string landmarks = "landmarks_truth_sensor_1-3.csv";
    // std::string ref_sensor_name = "sensor_3"; // "robot", "phantom", "tool", "sensor_1", "sensor_2"
    // m_emt->landmark_registration(landmarks, ref_sensor_name);
    // EMTrackerNode::~EMTrackerNode();

    //
    m_emt->start_read_thread();
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  }

  void setup_igtl(std::string host_name, int port)
  {
    std::cout << host_name << std::endl;

    this->m_socket = igtl::ClientSocket::New();
    int r = this->m_socket->ConnectToServer(host_name.c_str(), port);

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
    auto msg_em_base = interfaces::msg::Taskspace();

    // get current time
    rclcpp::Time now = this->get_clock()->now();

    double sample_time;
    quatTransformation tool_in_robot, tool_dot_in_robot, robot_in_em, tool_in_em, probe_in_robot;
    quatTransformation sensor_1_in_robot, sensor_2_in_robot, sensor_3_in_robot;

    blaze::StaticVector<double, 3UL> tool_pos_flt, tool_vel_flt = blaze::StaticVector<double, 3UL>(0.0);

    m_emt->get_tool_transform_in_robot(tool_in_robot);
    m_emt->get_tool_transform_in_robot_dot(tool_dot_in_robot);
    m_emt->get_tool_transform_in_phantom(m_tool_in_phantom);
    m_emt->get_robot_transform_in_phantom(m_robot_in_phantom);
    m_emt->get_probe_transform_in_phantom(m_probe_in_phantom);
    m_emt->get_probe_transform_in_robot(probe_in_robot);

    m_emt->get_sample_time(sample_time);

    tool_pos_flt = m_filter->add_data_point(tool_in_robot.translation);
    tool_vel_flt = (tool_pos_flt - m_tool_pos_flt_prev) / m_sample_time;
    m_tool_pos_flt_prev = tool_pos_flt;

    msg_phantom.p[0] = m_tool_in_phantom.translation[0]; // to align with cathter robot system
    msg_phantom.p[1] = m_tool_in_phantom.translation[1];
    msg_phantom.p[2] = m_tool_in_phantom.translation[2];

    // msg_base.p[0] = tool_in_robot.translation[0]; // to align with cathter robot system
    // msg_base.p[1] = tool_in_robot.translation[1];
    // msg_base.p[2] = tool_in_robot.translation[2];

    msg_base.p[0] = tool_pos_flt[0]; // to align with cathter robot system
    msg_base.p[1] = tool_pos_flt[1];
    msg_base.p[2] = tool_pos_flt[2];

    msg_phantom_base.p[0] = m_robot_in_phantom.translation[0];
    msg_phantom_base.p[1] = m_robot_in_phantom.translation[1];
    msg_phantom_base.p[2] = m_robot_in_phantom.translation[2];
    msg_phantom_base.h[0] = m_robot_in_phantom.rotation[0];
    msg_phantom_base.h[1] = m_robot_in_phantom.rotation[1];
    msg_phantom_base.h[2] = m_robot_in_phantom.rotation[2];
    msg_phantom_base.h[3] = m_robot_in_phantom.rotation[3];

    // msg.p_phantom_probe[0] = m_probe_transform.translation[0]; // to align with cathter robot system
    // msg.p_phantom_probe[1] = m_probe_transform.translation[1];
    // msg.p_phantom_probe[2] = m_probe_transform.translation[2];

    m_publisher_phantom->publish(msg_phantom);
    m_publisher_base->publish(msg_base);
    // m_publisher_phantom_base->publish(msg_phantom_base);

    /// using tf2
    std::vector<geometry_msgs::msg::TransformStamped> tf2_transforms;
    quatTransformation tran_in_em;
    geometry_msgs::msg::TransformStamped tf2_tran;
    tf2_tran.header.stamp = this->get_clock()->now();
    if (m_emt->get_robot_transform_in_em(tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "robot_base";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_tool_transform_in_em(tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "ctr_tip";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_phantom_transform_in_em(tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "phantom";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_usprobe_transform_in_em(tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "us_probe";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_probe_transform_in_em(tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "probe";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_sensor_transform_in_em("sensor_1", tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "sensor_1";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_sensor_transform_in_em("sensor_2", tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "sensor_2";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    if (m_emt->get_sensor_transform_in_em("sensor_3", tran_in_em) == 0 && std::isfinite(tran_in_em.translation[0]))
    {
      tf2_tran.header.frame_id = "em_tracker";
      tf2_tran.child_frame_id = "sensor_3";
      tf2_tran.transform.translation.x = tran_in_em.translation[0];
      tf2_tran.transform.translation.y = tran_in_em.translation[1];
      tf2_tran.transform.translation.z = tran_in_em.translation[2];
      tf2_tran.transform.rotation.w = tran_in_em.rotation[0];
      tf2_tran.transform.rotation.x = tran_in_em.rotation[1];
      tf2_tran.transform.rotation.y = tran_in_em.rotation[2];
      tf2_tran.transform.rotation.z = tran_in_em.rotation[3];
      tf2_transforms.push_back(tf2_tran);
    }
    m_tf2_broadcast->sendTransform(tf2_transforms);

    // send on IGTLink
    if (m_flag_igtl)
    {
      // EMTrackerNode::igtl_points_callback();
      EMTrackerNode::igtl_probe_tran_callback();
      EMTrackerNode::igtl_robot_tran_callback();
      EMTrackerNode::igtl_tool_tran_callback();
    }

    double time = static_cast<double>(now.nanoseconds()) / 1E9;
    // log_position(time, tool_transform.translation, sample_time);
    // log_position(tool_in_robot.translation, sample_time, "tip");
    if (m_flag_log_position)
    {
      log_position(tool_in_robot, sample_time, "tip");
    }
    // log_position(tool_pos_flt, sample_time, "tip_flt");
    // log_position(probe_in_robot.translation, sample_time, "prb");
  }

  void handle_tranformation_service_request(const std::shared_ptr<interfaces::srv::Transformation::Request> request, std::shared_ptr<interfaces::srv::Transformation::Response> response)
  {
    // Populate your transformation matrix here
    // Example transformation matrix (identity matrix)

    blaze::StaticMatrix<double, 3UL, 3UL> R;
    quat2Rotmat(m_robot_in_phantom.rotation, R);

    blaze::StaticMatrix<double, 4, 4> tran_matrix_robot = m_robot_in_phantom.toMatrix();

    response->transformation_matrix = {
        tran_matrix_robot(0, 0), tran_matrix_robot(0, 1), tran_matrix_robot(0, 2), tran_matrix_robot(0, 3),
        tran_matrix_robot(1, 0), tran_matrix_robot(1, 1), tran_matrix_robot(1, 2), tran_matrix_robot(1, 3),
        tran_matrix_robot(2, 0), tran_matrix_robot(2, 1), tran_matrix_robot(2, 2), tran_matrix_robot(2, 3),
        0.0, 0.0, 0.0, 1.0};

    std::cout << "CT-SCAN -> CTR transformation: \n"
              << tran_matrix_robot << std::endl;

    RCLCPP_INFO(this->get_logger(), "Sending transformation matrix");
  }

  void igtl_points_callback()
  {
    m_pm_emtracker->ClearPointElement();
    // tool in phantom frame
    m_pe_tool->SetName("tool");
    m_pe_tool->SetRadius(5.0);
    m_pe_tool->SetRGBA(100, 100, 0, 255);
    m_pe_tool->SetPosition(m_tool_in_phantom.translation[0] * 1e3,
                           m_tool_in_phantom.translation[1] * 1e3,
                           m_tool_in_phantom.translation[2] * 1e3);

    // probe in phantom frame
    m_pe_probe->SetName("probe");
    m_pe_probe->SetRadius(5.0);
    m_pe_probe->SetRGBA(100, 100, 0, 255);
    m_pe_probe->SetPosition(m_probe_in_phantom.translation[0] * 1e3,
                            m_probe_in_phantom.translation[1] * 1e3,
                            m_probe_in_phantom.translation[2] * 1e3);

    // ctr base in phantom frame
    m_pe_base->SetName("base");
    m_pe_base->SetRadius(5.0);
    m_pe_base->SetRGBA(100, 100, 0, 255);
    m_pe_base->SetPosition(m_robot_in_phantom.translation[0] * 1e3,
                           m_robot_in_phantom.translation[1] * 1e3,
                           m_robot_in_phantom.translation[2] * 1e3);

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
    blaze::StaticMatrix<double, 4, 4> tran_matrix_probe = m_probe_in_phantom.toMatrix();
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
    blaze::StaticMatrix<double, 4, 4> tran_matrix_robot = m_robot_in_phantom.toMatrix();
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
    blaze::StaticMatrix<double, 4, 4> tran_matrix_robot = m_tool_in_phantom.toMatrix();
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

  void handle_freeze_robot(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      m_emt->freeze_robot(true);
      response->success = true;
      response->message = "Robot frozen";
      RCLCPP_INFO(this->get_logger(), "Robot is frozen");
    }
    else
    {
      m_emt->freeze_robot(false);
      response->success = true;
      response->message = "Robot defrozen";
      RCLCPP_INFO(this->get_logger(), "Robot is active");
    }
  }

  void log_position(const quatTransformation& tool_in_robot, double sample_time, std::string prefix = "")
  {
    std::ostringstream oss;
    auto print_with_space_if_positive = [](double value)
    {
      std::ostringstream tmp;
      tmp << std::fixed << std::setprecision(4);
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

    oss << '[' << prefix << "] " 
        << "x:" << print_with_space_if_positive(tool_in_robot.translation[0]) << "  "
        << "y:" << print_with_space_if_positive(tool_in_robot.translation[1]) << "  "
        << "z:" << print_with_space_if_positive(tool_in_robot.translation[2]) << " [m] | "
        << "qo:" << print_with_space_if_positive(tool_in_robot.rotation[0]) << "  "
        << "qx:" << print_with_space_if_positive(tool_in_robot.rotation[1]) << "  "
        << "qy:" << print_with_space_if_positive(tool_in_robot.rotation[2]) << "  "
        << "qz:" << print_with_space_if_positive(tool_in_robot.rotation[3]) << "  "
        << "dt:" << std::fixed << std::setprecision(1) << sample_time * 1e3 << " [ms]";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void log_position(blaze::StaticVector<double, 3> position, double sample_time, std::string prefix = "")
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

    oss << '[' << prefix << "] " 
        << "x:" << print_with_space_if_positive(position[0]) << "  "
        << "y:" << print_with_space_if_positive(position[1]) << "  "
        << "z:" << print_with_space_if_positive(position[2]) << " [m]"
        << " | dt:" << std::fixed << std::setprecision(1) << sample_time * 1e3 << " [ms]";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  size_t count_;
  double m_sample_time;
  double m_cutoff_freq;
  bool m_flag_igtl;
  bool m_flag_log_position;
  std::unique_ptr<EMTracker> m_emt;
  std::unique_ptr<ButterworthFilter<3UL>> m_filter;
  blaze::StaticVector<double, 3UL> m_tool_pos_flt_prev = blaze::StaticVector<double, 3UL>(0.0);
  quatTransformation m_tool_in_phantom, m_robot_in_phantom, m_probe_in_phantom;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher_base, m_publisher_phantom, m_publisher_phantom_base;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_heartbeat_publisher;
  rclcpp::TimerBase::SharedPtr m_timer_heartbeat;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_heartbeat;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_freeze_phantom_service;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_freeze_robot_service;
  rclcpp::Service<interfaces::srv::Transformation>::SharedPtr m_ctr_tranform_service;

  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_broadcast;

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

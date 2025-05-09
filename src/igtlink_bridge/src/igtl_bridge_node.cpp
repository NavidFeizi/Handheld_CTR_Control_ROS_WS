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
#include "std_srvs/srv/trigger.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/transformation.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/msg/status.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_eigen/tf2_eigen.hpp"

#include "igtlOSUtil.h"
#include "igtlClientSocket.h"
#include "igtlMessageHeader.h"
#include "igtlPointMessage.h"
#include "igtlTransformMessage.h"
#include "igtlStatusMessage.h"
#include "igtlCommandMessage.h"
#include "igtlStringMessage.h"

#include "interfaces/msg/taskspace.hpp"

using namespace std::chrono_literals;

class IGTLinkBridgeNode : public rclcpp::Node
{
protected:
public:
  IGTLinkBridgeNode()
      : Node("igtlink_bridge")
  {
    declare_parameters();
    setup_igtl(m_hostname, m_port);
    RCLCPP_INFO(get_logger(), "Checkpoint_1");
    setup_ros_interfaces();
    RCLCPP_INFO(get_logger(), "Checkpoint_2");
  }

  ~IGTLinkBridgeNode()
  {
    stop_igtl_receiver_ = true;
    if (m_igtl_receiver_thread.joinable())
      m_igtl_receiver_thread.join();
    // Destructor will automatically close the hardware connection
  }

private:
  void declare_parameters()
  {
    /// Set default parameteres and allow it to be overridden by a launch file or command line parameter
    declare_parameter<double>("sample_time", 5E-3);
    m_sample_time = get_parameter("sample_time").as_double();
    declare_parameter<int>("port", 18944);
    m_port = get_parameter("port").as_int();
    declare_parameter<std::string>("hostname", "localhost");
    m_hostname = get_parameter("hostname").as_string();
  }

  void setup_ros_interfaces()
  {
    m_callback_group_read = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_callback_group_tf2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    /// listener to tf2 transormation messages
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf2_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    // m_tf2_timer = this->create_wall_timer(50ms, std::bind(&IGTLinkBridgeNode::tf2_receive_timer_callback, this), m_callback_group_tf2);

    // broadcast sensors readings
    m_tf2_broadcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // /// timer to triger OpenIGTLink command message receive
    // auto sample_time = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    // m_timer = this->create_wall_timer(sample_time, std::bind(&IGTLinkBridgeNode::igtl_receive_command_timer_callback, this), m_callback_group_read);

    /// robot setup service
    m_robot_setup_client = create_client<interfaces::srv::Config>("robot_config");

    /// robot setup service
    m_planner_client = create_client<std_srvs::srv::Trigger>("manual_target");

    m_cbGroup1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup4 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    auto subs_options_2 = rclcpp::SubscriptionOptions();
    auto subs_options_3 = rclcpp::SubscriptionOptions();
    auto subs_options_4 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_cbGroup1;
    subs_options_2.callback_group = m_cbGroup2;
    subs_options_3.callback_group = m_cbGroup3;
    subs_options_4.callback_group = m_cbGroup4;

    m_subsPath = create_subscription<std_msgs::msg::Float64MultiArray>("task_space/path", 10, std::bind(&IGTLinkBridgeNode::publishPath_callback, this, std::placeholders::_1), subs_options_1);
    m_subsShapeT1 = create_subscription<std_msgs::msg::Float64MultiArray>("shape/tube_1", 10, std::bind(&IGTLinkBridgeNode::publishShapeT1_callback, this, std::placeholders::_1), subs_options_2);
    m_subsShapeT2 = create_subscription<std_msgs::msg::Float64MultiArray>("shape/tube_2", 10, std::bind(&IGTLinkBridgeNode::publishShapeT2_callback, this, std::placeholders::_1), subs_options_3);
    m_subsShapeT3 = create_subscription<std_msgs::msg::Float64MultiArray>("shape/tube_3", 10, std::bind(&IGTLinkBridgeNode::publishShapeT3_callback, this, std::placeholders::_1), subs_options_4);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    m_igtl_receiver_thread = std::thread(&IGTLinkBridgeNode::igtl_receive_loop, this);
  }

  /// setup OpenIGTLink client
  void setup_igtl(std::string hostname, int port)
  {
    RCLCPP_INFO(get_logger(), "Connecting to server at %s:%d", hostname.c_str(), port);

    m_socket = igtl::ClientSocket::New();

    int r = m_socket->ConnectToServer(hostname.c_str(), port);
    if (r == 0)
    {
      RCLCPP_INFO(get_logger(), "Connected to the OpenIGTLink server.");
    }
    else

    {
      RCLCPP_INFO(get_logger(), "Fail to connect to %s:%d. Retrying...", hostname.c_str(), port);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Allocate message class for planned path
    m_pointMsg_Path = igtl::PointMessage::New();
    m_pointMsg_Path->SetDeviceName("IP5");
    // stringStream object for naming points in OpenIGTLink
    std::stringstream ss;
    // setting up the points
    for (size_t idx = 0UL; idx < 1000UL; ++idx)
    {
      m_pointersOfPoints_Path[idx] = igtl::PointElement::New();
      ss << "Point_" << std::to_string(idx);
      // clears the stringStream object
      ss.str(std::string());
      m_pointersOfPoints_Path[idx]->SetName(ss.str().c_str());
      m_pointersOfPoints_Path[idx]->SetGroupName("GROUP_0");
      m_pointersOfPoints_Path[idx]->SetRadius(0.50);
      m_pointersOfPoints_Path[idx]->SetOwner("IMAGE_0");
      m_pointersOfPoints_Path[idx]->SetRGBA(100, 0, 5, 15);
    }

    // Allocate message class
    m_pointMsg_Tb1 = igtl::PointMessage::New();
    m_pointMsg_Tb1->SetDeviceName("IP0");
    m_pointMsg_Tb2 = igtl::PointMessage::New();
    m_pointMsg_Tb2->SetDeviceName("IP1");
    m_pointMsg_Tb3 = igtl::PointMessage::New();
    m_pointMsg_Tb3->SetDeviceName("IP2");
    // setting up the points
    for (size_t idx = 0UL; idx < 1000UL; ++idx)
    {
      m_pointersOfPoints_Tb1[idx] = igtl::PointElement::New();
      m_pointersOfPoints_Tb2[idx] = igtl::PointElement::New();
      m_pointersOfPoints_Tb3[idx] = igtl::PointElement::New();
      ss << "Point_" << std::to_string(idx);
      // clears the stringStream object
      ss.str(std::string());

      m_pointersOfPoints_Tb1[idx]->SetName(ss.str().c_str());
      m_pointersOfPoints_Tb1[idx]->SetGroupName("GROUP_0");
      m_pointersOfPoints_Tb1[idx]->SetRadius(0.50);
      m_pointersOfPoints_Tb1[idx]->SetOwner("IMAGE_0");
      m_pointersOfPoints_Tb1[idx]->SetRGBA(100, 0, 5, 15);

      m_pointersOfPoints_Tb2[idx]->SetName(ss.str().c_str());
      m_pointersOfPoints_Tb2[idx]->SetGroupName("GROUP_0");
      m_pointersOfPoints_Tb2[idx]->SetRadius(1.00);
      m_pointersOfPoints_Tb2[idx]->SetOwner("IMAGE_0");
      m_pointersOfPoints_Tb2[idx]->SetRGBA(0, 5, 100, 15);

      m_pointersOfPoints_Tb3[idx]->SetName(ss.str().c_str());
      m_pointersOfPoints_Tb3[idx]->SetGroupName("GROUP_0");
      m_pointersOfPoints_Tb3[idx]->SetRadius(1.50);
      m_pointersOfPoints_Tb3[idx]->SetOwner("IMAGE_0");
      m_pointersOfPoints_Tb3[idx]->SetRGBA(5, 100, 0, 15);
    }

    // m_pm_emtracker = igtl::PointMessage::New();
    // m_pm_emtracker->SetDeviceName("emtracker");

    // m_trans_probe = igtl::TransformMessage::New();
    // m_trans_probe->SetDeviceName("emt_probe");

    // m_trans_robot = igtl::TransformMessage::New();
    // m_trans_robot->SetDeviceName("emt_robot");

    // m_trans_tool = igtl::TransformMessage::New();
    // m_trans_tool->SetDeviceName("emt_tool");

    // // setting up the points
    // m_pe_tool = igtl::PointElement::New();
    // m_pe_probe = igtl::PointElement::New();
    // m_pe_base = igtl::PointElement::New();
  }

  /// listen to ROS2 tf2 messages and send available frames is OpenIGTLink transfrom message
  void tf2_receive_timer_callback()
  {
    geometry_msgs::msg::TransformStamped tf2_tran;
    auto frameNames = m_tf_buffer->getAllFrameNames();

    for (auto sourceFrame : frameNames)
    {
      if (sourceFrame == "em_tracker" || sourceFrame == "manual_target")
        continue;
      // std::string sourceFrame = "ctr_tip";
      std::string targetFrame = "em_tracker";
      try
      {
        tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
        // std::cout << "X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            targetFrame.c_str(), sourceFrame.c_str(), ex.what());
        return;
      }
      Eigen::Matrix4d eigen_trans = tf2::transformToEigen(tf2_tran).matrix();

      igtl::Matrix4x4 trans, trans_asc;
      for (int i = 0; i < 4; ++i)
      {
        for (int j = 0; j < 4; ++j)
        {
          trans[i][j] = eigen_trans(i, j);
        }
      }
      // convert m to mm
      trans[0][3] = trans[0][3] * 1e3;
      trans[1][3] = trans[1][3] * 1e3;
      trans[2][3] = trans[2][3] * 1e3;

      ModifyOrientationFromAuroraToAscension(trans, trans_asc);
      ModifyOrientationFromAuroraToAscensionInplace(trans);

      auto m_trans_robot = igtl::TransformMessage::New();
      m_trans_robot->SetDeviceName(sourceFrame);
      // m_trans_robot->SetMessageID(5);
      m_trans_robot->SetMatrix(trans_asc);
      m_trans_robot->Pack();

      // Send the message
      m_socket->Send(m_trans_robot->GetPackPointer(), m_trans_robot->GetPackSize());
      // RCLCPP_INFO(this->get_logger(), "IGTL EM tf2 sent");
    }

    std::string sourceFrame = "robot_base";
    std::string targetFrame = "em_tracker";
    try
    {
      tf2_tran = m_tf_buffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
      // std::cout << "X: " << tf2_tran.transform.translation.x << "  Y: " << tf2_tran.transform.translation.y << "  Z: " << tf2_tran.transform.translation.z << std::endl;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          targetFrame.c_str(), sourceFrame.c_str(), ex.what());
      return;
    }
    Eigen::Matrix4d m_robot_trans = tf2::transformToEigen(tf2_tran).matrix();
  }

  /// broadcast tranfromation from Slicer to tf2
  void tf2_broadcast_transformation(std::string frameName, const igtl::Matrix4x4 &trans)
  {
    // TODO: fix the transfromation from Ascension to Aurora

    /// using tf2
    std::vector<geometry_msgs::msg::TransformStamped> tf2_transforms;
    // igtl::Matrix4x4 trans;
    geometry_msgs::msg::TransformStamped tf2_tran;
    tf2_tran.header.stamp = this->get_clock()->now();

    tf2::Matrix3x3 rot_matrix;
    for (int i; i < 3; i++)
    {
      for (int j; j < 3; j++)
      {
        rot_matrix[i][j] = trans[i][j];
      }
    }

    tf2::Quaternion q;
    rot_matrix.getRotation(q);

    tf2_tran.header.frame_id = "em_tracker";
    tf2_tran.child_frame_id = frameName;
    tf2_tran.transform.translation.x = trans[0][3];
    tf2_tran.transform.translation.y = trans[1][3];
    tf2_tran.transform.translation.z = trans[2][3];
    tf2_tran.transform.rotation.w = q.getW();
    tf2_tran.transform.rotation.x = q.getX();
    tf2_tran.transform.rotation.y = q.getY();
    tf2_tran.transform.rotation.z = q.getZ();
    tf2_transforms.push_back(tf2_tran);

    m_tf2_broadcast->sendTransform(tf2_transforms);
  }

  /// reads the COMMAND Messages form OpenIGTLink and send them over ROS2 robot setup service
  void igtl_receive_loop()
  {
    while (rclcpp::ok() && !stop_igtl_receiver_)
    {
      /// Create a message buffer to receive header
      igtl::MessageHeader::Pointer header = igtl::MessageHeader::New();
      igtl::TimeStamp::Pointer ts = igtl::TimeStamp::New();
      header->InitPack();

      bool timeout(false);
      int r = m_socket->Receive(header->GetPackPointer(), header->GetPackSize(), timeout);
      if (r == 0)
        continue;

      header->Unpack();
      std::string type = header->GetDeviceType();

      if (type == "COMMAND")
      {
        /// Create a command message
        igtl::CommandMessage::Pointer msg = igtl::CommandMessage::New();
        msg->SetMessageHeader(header);
        msg->AllocatePack();
        m_socket->Receive(msg->GetPackBodyPointer(), msg->GetPackBodySize(), timeout);
        msg->Unpack();

        std::string deviceName = msg->GetDeviceName();
        std::string commandName = msg->GetCommandName();
        std::string commandContent = msg->GetCommandContent();

        RCLCPP_INFO(this->get_logger(), "Received IGTL COMMAND: \n \t\t deviceName: %s \n \t\t commandName: %s \n \t\t commandContent %s", deviceName.c_str(), commandName.c_str(), commandContent.c_str());

        if (commandName == "RobotSetup")
        {
          auto request = std::make_shared<interfaces::srv::Config::Request>();
          request->command = commandContent;
          // using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
          auto future_result = m_robot_setup_client->async_send_request(request, std::bind(&IGTLinkBridgeNode::handle_service_response, this, std::placeholders::_1));
        }

        else if (commandName == "RobotManualTrajectory")
        {
          auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
          // using ServiceResponseFuture_2 = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
          auto future_result_2 = m_planner_client->async_send_request(request, std::bind(&IGTLinkBridgeNode::handle_service_response_2, this, std::placeholders::_1));
        }
      }

      else if (type == "TRANSFORM")
      {
        igtl::TransformMessage::Pointer trans_msg = igtl::TransformMessage::New();
        trans_msg->SetMessageHeader(header);
        trans_msg->AllocatePack();

        // Receive body
        r = m_socket->Receive(trans_msg->GetPackBodyPointer(), trans_msg->GetPackBodySize(), timeout);
        if (r == 0)
        {
          std::cerr << "Failed to receive transform body." << std::endl;
          return;
        }

        // Unpack the message
        int c = trans_msg->Unpack(1); // 1 = CRC check
        if (!(c & igtl::MessageHeader::UNPACK_BODY))
        {
          std::cerr << "Failed to unpack transform message." << std::endl;
          return;
        }

        igtl::Matrix4x4 matrix;
        trans_msg->GetMatrix(matrix);

        if (trans_msg->GetDeviceName() == "BaseLinearTransform")
        {
          tf2_broadcast_transformation("manual_target", matrix);
          RCLCPP_INFO(get_logger(), "BaseLinearTransform sent");
        }
        if (trans_msg->GetDeviceName() == "TragetLinearTransform")
        {
          tf2_broadcast_transformation("manual_target", matrix);
          RCLCPP_INFO(get_logger(), "TragetLinearTransform sent");
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  /// reads the COMMAND Messages form OpenIGTLink and send them over ROS2 robot setup service
  void igtl_receive_command_timer_callback()
  {
    while (rclcpp::ok())
    {
      /// Create a message buffer to receive header
      igtl::MessageHeader::Pointer header = igtl::MessageHeader::New();
      igtl::TimeStamp::Pointer ts = igtl::TimeStamp::New();

      header->InitPack();

      bool timeout(false);
      if (m_socket->Receive(header->GetPackPointer(), header->GetPackSize(), timeout) == 0)
        continue;

      header->Unpack();
      std::string type = header->GetDeviceType();

      if (strcmp(header->GetDeviceType(), "COMMAND") == 0)
      {
        /// Create a command message
        m_cmd_msg = igtl::CommandMessage::New();
        m_cmd_msg->SetMessageHeader(header);
        m_cmd_msg->AllocatePack();

        m_socket->Receive(m_cmd_msg->GetPackBodyPointer(), m_cmd_msg->GetPackBodySize(), timeout);
        m_cmd_msg->Unpack();

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        std::string deviceName = m_cmd_msg->GetDeviceName();
        std::string commandName = m_cmd_msg->GetCommandName();
        std::string commandContent = m_cmd_msg->GetCommandContent();

        RCLCPP_INFO(this->get_logger(), "Received IGTL COMMAND: \n \t\t deviceName: %s \n \t\t commandName: %s \n \t\t commandContent %s", deviceName.c_str(), commandName.c_str(), commandContent.c_str());

        if (commandName == "RobotSetup")
        {
          request->command = commandContent;
          using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
          auto response_received_callback = std::bind(&IGTLinkBridgeNode::handle_service_response, this, std::placeholders::_1);
          auto future_result = m_robot_setup_client->async_send_request(request, response_received_callback);
        }

        else if (commandName == "RobotManualTrajectory")
        {
          auto request_2 = std::make_shared<std_srvs::srv::Trigger::Request>();
          using ServiceResponseFuture_2 = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
          auto response_received_callback_planner = std::bind(&IGTLinkBridgeNode::handle_service_response_2, this, std::placeholders::_1);
          auto future_result_2 = m_planner_client->async_send_request(request_2, response_received_callback_planner);

          // auto m_manual_tran = igtl::TransformMessage::New();
          // if (m_manual_tran->GetDeviceName() == "BaseLinearTransform")
          // {
          //   igtl::Matrix4x4 trans;
          //   m_manual_tran->GetMatrix(trans);
          //   RCLCPP_INFO(get_logger(), "To be developed");
          // }
          // if (m_manual_tran->GetDeviceName() == "To be developed")
          // {
          //   igtl::Matrix4x4 trans;
          //   m_manual_tran->GetMatrix(trans);
          // }
        }
      }
    }
  }

  /// publish the planned path
  void publishPath_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> Path = multiArrayMsgToBlaze(msg);
    // transformPathInpalce(Path, m_robot_trans); // tranform from robot_base from to emframe
    broadcastVectorOfPoints(m_pointMsg_Path, m_pointersOfPoints_Path, Path);
  }

  /// publish the backbone shape of tube 1 (inner tube)
  void publishShapeT1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> Shape = multiArrayMsgToBlaze(msg);
    // transformPathInpalce(Shape, m_robot_trans); // tranform from robot_base from to emframe
    // std::cout << "Tb1: \n" << Shape * 1E3 << std::endl;
    broadcastVectorOfPoints(m_pointMsg_Tb1, m_pointersOfPoints_Tb1, Shape);
  }

  /// publish the backbone shape of tube 2 (middle tube)
  void publishShapeT2_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> Shape = multiArrayMsgToBlaze(msg);
    // transformPathInpalce(Shape, m_robot_trans); // tranform from robot_base from to emframe
    broadcastVectorOfPoints(m_pointMsg_Tb2, m_pointersOfPoints_Tb2, Shape);
  }

  /// publish the backbone shape of tube 3 (outer tube)
  void publishShapeT3_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> Shape = multiArrayMsgToBlaze(msg);
    // transformPathInpalce(Shape, m_robot_trans); // tranform from robot_base from to emframe
    broadcastVectorOfPoints(m_pointMsg_Tb3, m_pointersOfPoints_Tb3, Shape);
  }

  /// Conver Float64MultiArray message to Blaze hybrid matrix
  blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> multiArrayMsgToBlaze(const std_msgs::msg::Float64MultiArray::SharedPtr &msg)
  {
    blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> mat;
    size_t rows = msg->layout.dim[0].size;
    size_t cols = msg->layout.dim[1].size;
    mat.resize(rows, cols);

    // RCLCPP_INFO(this->get_logger(), "Received matrix: %zu x %zu", rows, cols);

    for (size_t i = 0; i < rows; ++i)
    {
      for (size_t j = 0; j < cols; ++j)
      {
        mat(i, j) = msg->data[i * cols + j];
      }
    }
    return mat;
  }

  /// Apply rigid trandformation to a matrix of 3D points
  void transformPathInpalce(blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> &path, const Eigen::Matrix4d &trans)
  {
    // Extract rotation (top-left 3x3) and translation (top-right 3x1)
    Eigen::Matrix3d R = trans.block<3, 3>(0, 0);
    Eigen::Vector3d t = trans.block<3, 1>(0, 3);

    for (size_t i = 0; i < path.rows(); ++i)
    {
      // Convert Blaze row to Eigen vector
      Eigen::Vector3d p(path(i, 0), path(i, 1), path(i, 2));

      // Apply transformation: p' = R * p + t
      p = R * p + t;

      // Store back to Blaze matrix
      path(i, 0) = p(0);
      path(i, 1) = p(1);
      path(i, 2) = p(2);
    }
  }

  // Broadcasts a vector of 3D Points (path) to 3D-Slicer
  void broadcastVectorOfPoints(const igtl::PointMessage::Pointer &pointMsg,
                               std::array<igtl::PointElement::Pointer, 1000UL> pointersOfPoints,
                               const blaze::HybridMatrix<double, 1000UL, 3UL, blaze::rowMajor> &points)
  {
    pointMsg->ClearPointElement();

    for (size_t idx = 0UL; idx < points.rows(); ++idx)
    {
      auto point = igtl::PointElement::New();
      pointersOfPoints[idx]->SetPosition(points(idx, 0UL) * 1e3, points(idx, 1UL) * 1e3, points(idx, 2UL) * 1e3);
      pointMsg->AddPointElement(pointersOfPoints[idx]);
    }
    pointMsg->Pack();
    m_socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
  }

  ///
  void handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
  {
    // Get the result of the future object
    auto response = future.get();
    auto rts_str = igtl::StringMessage::New();
    rts_str->SetDeviceName("Robot");

    if (response->success)
    {
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
    }
    rts_str->SetString(response->message.c_str());
    rts_str->Pack();

    m_socket->Send(rts_str->GetPackPointer(), rts_str->GetPackSize());
    std::cout << "[Robot] Send RTS_COMMAND response." << std::endl;
  }

  ///
  void handle_service_response_2(const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    // Get the result of the future object
    auto response = future.get();
    auto rts_str = igtl::StringMessage::New();
    rts_str->SetDeviceName("Robot");

    if (response->success)
    {
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
    }
    rts_str->SetString(response->message.c_str());
    rts_str->Pack();

    m_socket->Send(rts_str->GetPackPointer(), rts_str->GetPackSize());
    std::cout << "[Robot] Send RTS_COMMAND response." << std::endl;
  }

  ///
  void ModifyOrientationFromAuroraToAscension(const igtl::Matrix4x4 &input, igtl::Matrix4x4 &output)
  {
    // interchanging the axis
    // x = -z
    // z = -x
    // y = -y

    for (int i = 0; i < 3; i++)
    {
      output[i][0] = -input[i][2];
      output[i][1] = -input[i][1];
      output[i][2] = -input[i][0];
      output[i][3] = input[i][3];
    }
  }

  ///
  void ModifyOrientationFromAuroraToAscensionInplace(igtl::Matrix4x4 &mat)
  {
    for (int i = 0; i < 3; i++)
    {
      double buffer = mat[i][0];
      mat[i][0] = -mat[i][2];
      mat[i][1] = -mat[i][1];
      mat[i][2] = -buffer;
      mat[i][3] = mat[i][3];
    }
  }

  ///
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

  double m_sample_time;
  std::string m_hostname = "localhost";
  int m_port = 18944; // Slicer

  std::thread m_igtl_receiver_thread;
  std::atomic<bool> stop_igtl_receiver_ = false;

  // quatTransformation m_tool_transform, m_robot_transform, m_probe_transform;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::TimerBase::SharedPtr m_tf2_timer;
  rclcpp::Publisher<interfaces::msg::Taskspace>::SharedPtr m_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_heartbeat_publisher;
  rclcpp::TimerBase::SharedPtr m_timer_heartbeat;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_read, m_callback_group_tf2;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_heartbeat;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_freeze_phantom_service;
  rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_setup_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_planner_client;
  rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_enable_client;
  rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_robot_status_subs;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_subsPath;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_subsShapeT1, m_subsShapeT2, m_subsShapeT3;

  rclcpp::CallbackGroup::SharedPtr m_cbGroup1, m_cbGroup2, m_cbGroup3, m_cbGroup4;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf2_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_broadcast;

  igtl::ClientSocket::Pointer m_socket;
  igtl::PointMessage::Pointer m_pm_emtracker;
  igtl::PointMessage::Pointer m_pointMsg_Path;

  igtl::PointMessage::Pointer m_pointMsg_Tb1, m_pointMsg_Tb2, m_pointMsg_Tb3;
  igtl::TransformMessage::Pointer m_trans_probe, m_trans_robot, m_trans_tool;
  igtl::PointElement::Pointer m_pe_tool, m_pe_probe, m_pe_base;

  std::array<igtl::PointElement::Pointer, 1000UL> m_pointersOfPoints_Path;
  std::array<igtl::PointElement::Pointer, 1000UL> m_pointersOfPoints_Tb1, m_pointersOfPoints_Tb2, m_pointersOfPoints_Tb3;

  igtl::CommandMessage::Pointer m_cmd_msg;

  Eigen::Matrix4d m_robot_trans;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IGTLinkBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

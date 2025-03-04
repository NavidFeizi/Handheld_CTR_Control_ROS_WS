#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <QObject>
#include <QString>
#include <QPushButton>
#include <QKeyEvent>
#include <QSet>
#include <QDebug>
#include <QRadioButton>
#include <QGroupBox>
#include <QtConcurrent>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "interfaces/msg/status.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/srv/config.hpp"
#include "interfaces/action/jointstarget.hpp"
#include "interfaces/srv/jointstarget.hpp"

#include "Robot.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// Declare your enum class
enum class CtrlMode : int
{
  Config = 0x00,
  Manual = 0x01,
  Position = 0x02,
  Velocity = 0x03,
};

class RobotNode : public QWidget, public rclcpp::Node, public CTRobot
{
  Q_OBJECT

public:
  explicit RobotNode(QWidget *parent = nullptr) : QWidget(parent), rclcpp::Node("ctr_robot")
  {
    m_flag_use_target_action = true;
    // declare_parameters();
    initGui();
    initRosInterfaces();
    startRobotCommunication(m_sample_time);
  }

  void keyPressEvent(QKeyEvent *event) override
  {
    m_keysPressed.insert(event->key()); // Store pressed key
  }

  void keyReleaseEvent(QKeyEvent *event) override
  {
    m_keysPressed.remove(event->key()); // Remove key when released
  }

signals: // Functions declared (Qt specific)
  void dataReceived_1(QString m1_v1, QString m1_v2, QString m1_v3, QString m2_v1, QString m2_v2, QString m2_v3, QString m3_v1, QString m3_v2, QString m3_v3, QString m4_v1, QString m4_v2, QString m4_v3);
  void dataReceived_2(QString m1_l1, QString m1_l2, QString m2_l1, QString m2_l2, QString m3_l1, QString m3_l2, QString m4_l1, QString m4_l2);
  void update_enable_button_Text(QString text);     // Signal to update button text
  void update_wrench_pos_button_Text(QString text); // Signal to update button text

private slots: // Functions that receive and handle signals - can be connected to signals (Qt specific)
  void updateLabel_1(QString m1_v1, QString m1_v2, QString m1_v3, QString m2_v1, QString m2_v2, QString m2_v3, QString m3_v1, QString m3_v2, QString m3_v3, QString m4_v1, QString m4_v2, QString m4_v3)
  {
    QString text = QString("          Inr Rot  |  Inr Trn  |  Mdl Rot  |  Mdl Trn\n"
                           "Position: %1  |  %2   |   %3   |   %4\n"
                           "Velocity: %5  |  %6   |   %7   |   %8\n"
                           "Current:  %9  |  %10   |   %11   |   %12")
                       .arg(m1_v1)
                       .arg(m2_v1)
                       .arg(m3_v1)
                       .arg(m4_v1)
                       .arg(m1_v2)
                       .arg(m2_v2)
                       .arg(m3_v2)
                       .arg(m4_v2)
                       .arg(m1_v3)
                       .arg(m2_v3)
                       .arg(m3_v3)
                       .arg(m4_v3);

    mp_label_1->setText(text);
  }

  void updateLabel_2(QString m1_l1, QString m1_l2, QString m2_l1, QString m2_l2, QString m3_l1, QString m3_l2, QString m4_l1, QString m4_l2)
  {
    QString text = QString("Inr rot |  Inr tran |  Mdl rot |  Mdl tran \n"
                           "min:  %1  |  %3   |   %5   |   %7\n"
                           "max:  %2  |  %4   |   %6   |   %8")
                       .arg(m1_l1)
                       .arg(m1_l2)
                       .arg(m2_l1)
                       .arg(m2_l2)
                       .arg(m3_l1)
                       .arg(m3_l2)
                       .arg(m4_l1)
                       .arg(m4_l2);

    mp_label_2->setText(text);
  }

  // enable or distable the robot
  void toggleEnable()
  {
    // QtConcurrent::run([this]()
    //                   {
    blaze::StaticVector<bool, 4UL> status;
    getEnableStatus(status);

    if (status[0] || status[1] || status[2] || status[3])
      enableOperation(false);
    else
      enableOperation(true);
    // });
  }

  // engage the couplings and move the the linear statges to the back of the robot to home the encoders
  void findEncoders()
  {
    QtConcurrent::run([this]()
                      {
                        mp_optionCtrlmMode0->setChecked(true);
                        bool flag_middle_encoder_set = false;
                        bool flag_inner_encoder_set = false;
                        m_maxTorqueNegative = {400.0, 300.0, 400.0, 300.0};
                        m_maxTorquePositive = {400.0, 300.0, 400.0, 300.0};
                        m_maxDcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
                        m_maxVel = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
                        m_maxAcc = 0.15 * m_maxDcc;
                        setOperationMode(OpMode::VelocityProfile);
                        setMaxTorque(m_maxTorqueNegative, m_maxTorquePositive);
                        setProfileParams(m_maxVel, m_maxAcc, m_maxDcc);
                        // move linear joints with constant velocity untill motor current reache a threshold
                        blaze::StaticVector<double, 4UL> target_vel = {-0.0, -0.0055, -0.0, -0.005};
                        setTargetVel(target_vel);
                        getCurrent(m_current);
                        enableOperation(true);
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        while (!flag_inner_encoder_set || !flag_middle_encoder_set)
                        {
                          if (m_current[1] <= -1 * (m_maxTorqueNegative[1] - 10.0) && !flag_inner_encoder_set)
                          {
                            m_logger->info("[Master] inner carriage hit back");
                            target_vel[1] = 0.0;
                            setTargetVel(target_vel);
                            flag_inner_encoder_set = true;
                          }
                          if (m_current[3] <= -1 * (m_maxTorqueNegative[3] - 10.0) && !flag_middle_encoder_set)
                          {
                            m_logger->info("[Master] middle carriage hit back");
                            target_vel[3] = 0.0;
                            setTargetVel(target_vel);
                            flag_middle_encoder_set = true;
                          }
                          std::this_thread::sleep_for(std::chrono::milliseconds(10));
                          getCurrent(m_current);
                        }
                        // rotate rotary joints to make sure couplings are engaged
                        setTargetVel({2.0, 0.0, 2.0, 0.0});
                        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                        setTargetVel({0.0, 0.0, 0.0, 0.0});
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        // set encoders
                        this->m_inrTubeTrn->setEncoder(inner_back_pos);
                        this->m_mdlTubeTrn->setEncoder(middle_back_pos);
                        this->m_inrTubeRot->setEncoder(0.0);
                        this->m_mdlTubeRot->setEncoder(0.0);

                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        m_logger->info("[Master] encoders found");
                        enableOperation(false);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // prepare linear states close to preengages positon then engage the collets
  void engageColletsToWrench()
  {
    QtConcurrent::run([this]()
                      {
      mp_optionCtrlmMode0->setChecked(true);
      
      blaze::StaticVector<bool, 4UL> reach;
      blaze::StaticVector<bool, 4UL> encoderStatus;
      blaze::StaticVector<double, 4UL> targetPosTemp;

      if (!m_flag_readyToEngage)
      {
        blaze::StaticVector<bool, 4UL> status;
        getEncoderStatus(status); // updates encoders set flag
        if (true)                 // (status[1] * status[3])                                           // check if linear encoders are set
        {
          m_logger->info("Preparing to engage collets");

          m_maxDcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
          m_maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00};   // [deg/s] and [mm/s]
          m_maxTorqueNegative = {200.0, 400.0, 200.0, 400.0};
          m_maxTorquePositive = {200.0, 400.0, 200.0, 400.0};
          setOperationMode(OpMode::PositionProfile);
          setProfileParams(m_maxVel, m_maxDcc, m_maxDcc);
          setMaxTorque(m_maxTorqueNegative, m_maxTorquePositive);
          enableOperation(true);

          targetPosTemp = {m_x[0], m_pos_preEngage[1], m_x[2], m_pos_preEngage[3]};
          setTargetPos(targetPosTemp);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          getReachedStatus(reach);
          while (!(reach[1] && reach[3]))
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            getReachedStatus(reach);
          }
          m_logger->info("Ready to engage collets");
        }
      }
      else if (m_flag_readyToEngage)
      {
        m_maxDcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
        m_maxVel = {60.00 * M_PI / 180.00, 10.00 / 1000.00, 60.00 * M_PI / 180.00, 10.00 / 1000.00};   // [deg/s] and [mm/s]
        m_maxTorqueNegative = {350.0, 250.0, 350.0, 250.0};
        m_maxTorquePositive = {350.0, 250.0, 350.0, 250.0};
        
        getEncoderStatus(encoderStatus); // updates encoders set flag
        if (true)                 // (status[1] * status[3])                                           // check if linear encoders are set
        {
          setOperationMode(OpMode::PositionProfile);
          setProfileParams(m_maxVel, m_maxDcc, m_maxDcc);
          setMaxTorque(m_maxTorqueNegative, m_maxTorquePositive);
          enableOperation(true);

          // first position the middle collet
          m_logger->info("Engaging middle collet");
          setTargetPos({m_x[0], m_pos_preEngage[1], m_x[2], m_pos_engage[3] - 0.004});
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          getReachedStatus(reach);
          while (!(reach[3]))
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            getReachedStatus(reach);
          }

          targetPosTemp = {m_x[0], m_pos_preEngage[1], m_x[2], m_pos_engage[3]};
          setTargetPos(targetPosTemp);
          std::this_thread::sleep_for(std::chrono::milliseconds(1500));
          getReachedStatus(reach);
          // switch rotation direction of the collet until the collet is set in place
          while (!(reach[3]))
          {
            targetPosTemp[3] = m_pos_engage[3] - 0.004;
            setTargetPos(targetPosTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            targetPosTemp[2] = m_x[2] + 0.07 * M_PI;
            setTargetPos(targetPosTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            targetPosTemp[2] = m_x[2];
            targetPosTemp[3] = m_pos_engage[3];
            setTargetPos(targetPosTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            getReachedStatus(reach);
          }
          setTargetPos(m_x);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          m_logger->info("Step one done");
          targetPosTemp = m_x;
          targetPosTemp[2] = targetPosTemp[2] + 20.0 * M_PI;
          setTargetPos(targetPosTemp);
          while (!(m_current[2] > (m_maxTorquePositive[2] - 10.0)))
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
          setTargetPos(m_x);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          m_logger->info("Middle collet engaged");

          m_logger->info("Engaging the inner collet");
          // then position the inner collet
          setTargetPos({m_x[0], m_pos_engage[1] - 0.004, m_x[2], m_pos_engage[3]});
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          getReachedStatus(reach);
          while (!(reach[3]))
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            getReachedStatus(reach);
          }

          targetPosTemp = {m_x[0], m_pos_engage[1], m_x[2], m_pos_engage[3]};
          setTargetPos(targetPosTemp);
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          getReachedStatus(reach);
          // switch rotation direction of the collet until the collet is set in place
          while (!(reach[1]))
          {
            targetPosTemp[1] = m_pos_engage[1] - 0.004;
            setTargetPos(targetPosTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            targetPosTemp[0] = m_x[0] + 0.07 * M_PI;
            setTargetPos(targetPosTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            targetPosTemp[0] = m_x[0];
            targetPosTemp[1] = m_pos_engage[1];
            setTargetPos(targetPosTemp);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            getReachedStatus(reach);
          }
          setTargetPos(m_x);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          m_logger->info("Step one done");

          targetPosTemp = m_x;
          targetPosTemp[0] = targetPosTemp[0] + 20.0 * M_PI;
          setTargetPos(targetPosTemp);
          while (!(m_current[0] > (m_maxTorquePositive[0] - 10.0)))
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
          setTargetPos(m_x);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));

          m_logger->info("Inner collet engaged");
          m_logger->info("Both collets engaged");
        }
      } });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // lock collets
  void lockCollets()
  {
    QtConcurrent::run([this]()
                      {
    mp_optionCtrlmMode0->setChecked(true);
    blaze::StaticVector<bool, 4UL> reach;
    if (m_flagEngaged)                                     
    {
      m_logger->info("Locking collets");
      m_maxDcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
      m_maxVel = {100.00 * M_PI / 20.00, 0.50 / 1000.00, 100.00 * M_PI / 20.00, 0.50 / 1000.00};     // [deg/s] and [mm/s]
      m_maxTorqueNegative = {400.0, 300.0, 400.0, 300.0};
      m_maxTorquePositive = {400.0, 300.0, 400.0, 300.0};

      setOperationMode(OpMode::PositionProfile);
      setProfileParams(m_maxVel, m_maxDcc, m_maxDcc);
      setMaxTorque(m_maxTorqueNegative, m_maxTorquePositive);
      enableOperation(true);

      // set_target_position(m_pos_prewrench);
      setTargetPos({m_x[0] + 15 * M_PI, m_x[1] + 0.0010, m_x[2] + 15 * M_PI, m_x[3] + 0.0005}); // *********** the parameters must be adjusted based on screw lead *********
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      getReachedStatus(reach);
      while (!(m_current[0] > (m_maxTorquePositive[0] - 10.0) && reach[1] && m_current[2] > (m_maxTorquePositive[2] - 10.0) && reach[3]))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        getReachedStatus(reach);
      }
      setTargetPos(m_x);
      m_logger->info("Collets locked");
    }
    else
    {
      m_logger->warn("Collets must be engaged before locking");
    } });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // unlock collets
  void unlockCollets()
  {
    QtConcurrent::run([this]()
                      {
    mp_optionCtrlmMode0->setChecked(true);
    
    blaze::StaticVector<bool, 4UL> reach;
    if (m_flagEngaged) 
    {
      m_logger->info("Unlocking collets");
      m_maxDcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
      m_maxVel = {100.00 * M_PI / 20.00, 0.50 / 1000.00, 100.00 * M_PI / 20.00, 0.50 / 1000.00};     // [deg/s] and [mm/s]
      m_maxTorqueNegative = {1000.0, 220.0, 1000.0, 220.0};
      m_maxTorquePositive = {1000.0, 220.0, 1000.0, 220.0};

      setOperationMode(OpMode::PositionProfile);
      setProfileParams(m_maxVel, m_maxDcc, m_maxDcc);
      setMaxTorque(m_maxTorqueNegative, m_maxTorquePositive);
      enableOperation(true);

      // set_target_position(m_pos_prewrench);
      setTargetPos({m_x[0] - 4 * M_PI, m_x[1] - 0.0010, m_x[2] - 4 * M_PI, m_x[3] - 0.0005}); // *********** the parameters must be adjusted based on screw lead *********
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      getReachedStatus(reach);
      while (!(reach[0] && reach[1] && reach[2] && reach[3]))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        getReachedStatus(reach);
      }
      m_logger->info("Collets unlocked");
    }
    else
    {
      m_logger->warn("Collets must be engaged before unlocking");
    } });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // to be developed
  void setRotaryHome()
  {
    QtConcurrent::run([this]()
                      { m_logger->info("Setting Rotary Home"); });
  }

  // to be developed
  void gotoHome()
  {
    QtConcurrent::run([this]()
                      { m_logger->info("Going to Home"); });
  }

  //
  void onCtrlModeSelected()
  {
    QRadioButton *selectedButton = qobject_cast<QRadioButton *>(sender());
    if (selectedButton && selectedButton->isChecked())
    {
      QString selectedText = selectedButton->text();
      if (selectedText == "Config")
      {
        // unlock translation limits
        mp_optionTranLim1->setChecked(true);
        mp_bulletTranLimGroup->setEnabled(false);

        m_mode = CtrlMode::Config;
        RCLCPP_INFO(this->get_logger(), "Selected Mode: Config");
      }
      else if (selectedText == "Manual")
      {
        // unlock translation limits
        mp_bulletTranLimGroup->setEnabled(true);
        mp_optionTranLim0->setChecked(true);
        //

        blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
        blaze::StaticVector<double, 4UL> max_vel = {100.00 * M_PI / 180.00, 10.00 / 1000.00, 100.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
        blaze::StaticVector<double, 4UL> negative = {500.0, 500.0, 500.0, 500.0};
        blaze::StaticVector<double, 4UL> positive = {500.0, 500.0, 500.0, 500.0};
        setOperationMode(OpMode::VelocityProfile);
        setProfileParams(max_vel, max_dcc, max_dcc);
        setMaxTorque(negative, positive);
        m_mode = CtrlMode::Manual;
        RCLCPP_INFO(this->get_logger(), "Selected Mode: Manual");
      }
      else if (selectedText == "Position")
      {
        // enable translation limits and lock them
        mp_optionTranLim0->setChecked(true);
        mp_bulletTranLimGroup->setEnabled(false);
        //
        setOperationMode(OpMode::PositionProfile);
        m_mode = CtrlMode::Position;
        RCLCPP_INFO(this->get_logger(), "Selected Mode: Position");
      }
      else if (selectedText == "Velocity")
      {
        // enable translation limits and lock them
        mp_optionTranLim0->setChecked(true);
        mp_bulletTranLimGroup->setEnabled(false);
        setOperationMode(OpMode::VelocityProfile);
        m_mode = CtrlMode::Velocity;
        RCLCPP_INFO(this->get_logger(), "Selected Mode: Velocity");
      }
    }
  }

  //
  void onTranLimSelected()
  {
    QRadioButton *selectedButton = qobject_cast<QRadioButton *>(sender());
    if (selectedButton && selectedButton->isChecked())
    {
      QString selectedText = selectedButton->text();
      if (selectedText == "ON")
      {
        m_trans_limit = true;
        RCLCPP_INFO(this->get_logger(), "Translation Limit ON");
      }
      else if (selectedText == "OFF")
      {
        m_trans_limit = false;
        RCLCPP_INFO(this->get_logger(), "Translation Limit OFF");
      }
    }
  }

private:
  // Declare ROS parameters
  void declare_parameters()
  {
    declare_parameter<double>("Kp", 4.60);
    declare_parameter<double>("Ki", 2.60);
    m_kp = get_parameter("Kp").as_double();
    m_ki = get_parameter("Ki").as_double();
  }

  // Setup ROS interfaces including publishers, subscribers, services, and timers
  void initRosInterfaces()
  {
    // Create callback groups to ensure mutually exclusive callbacks
    m_cbGroup1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // m_cbGroup4 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup5 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cbGroup6 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber to receive target joint configurations
    auto subs_options_1 = rclcpp::SubscriptionOptions();
    subs_options_1.callback_group = m_cbGroup2;
    m_subscription_target = create_subscription<interfaces::msg::Jointspace>("joints_space/target", 10, std::bind(&RobotNode::jointSpaceTarget_callback, this, _1), subs_options_1);

    // Publisher to broadcast the robot status
    m_publisher_status = create_publisher<interfaces::msg::Status>("robot_status", 10);
    // Publisher to broadcast the current joint configurations
    m_publisher_joints = create_publisher<interfaces::msg::Jointspace>("/joint_space/feedback", 10);
    // configing service
    m_homing_service = this->create_service<interfaces::srv::Config>("config", std::bind(&RobotNode::configService_callback, this, _1, _2));

    // Low-level control loop timer
    auto control_sample_time = std::chrono::milliseconds(static_cast<int>(50));
    m_control_loop_timer = create_wall_timer(control_sample_time, std::bind(&RobotNode::targetCommand_timerCallback, this), m_cbGroup1);
    // Timer to read joint configurations periodically
    m_joints_config_timer = create_wall_timer(10ms, std::bind(&RobotNode::jointsConfig_timerCallback, this), m_cbGroup3);
    // Timer to read robot status periodically
    m_read_robot_timer = create_wall_timer(10ms, std::bind(&RobotNode::robotStatus_timerCallback, this), m_cbGroup3);
    // Timer to read keyboard inputs
    m_read_key_timer = this->create_wall_timer(10ms, std::bind(&RobotNode::keyboardRead_timerCallback, this), m_cbGroup5);

    // Initialize the watchdog timer
    // m_watchdog_timer_target = this->create_wall_timer(2000ms, std::bind(&RobotNode::check_target_publisher_alive, this), m_cbGroup6);

    // // Action Server Setup
    // m_action_server = rclcpp_action::create_server<interfaces::action::Jointstarget>(
    //     this, "joints_target",
    //     std::bind(&RobotNode::handle_goal, this, _1, _2),
    //     std::bind(&RobotNode::handle_cancel, this, _1),
    //     std::bind(&RobotNode::handle_accepted, this, _1));
  }

  // Setup ROS parameter callback function to handle dynamic parameter (Kp, Ki) updates - ** TEMP ** - needs to be further developed
  void rosParameterUpdate_callback()
  {
    auto param_callback = [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "Kp")
        {
          m_kp = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Parameter 'Kp' set to: %f", m_kp);
        }
        else if (parameter.get_name() == "Ki")
        {
          m_ki = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Parameter 'Ki' set to: %f", m_ki);
        }
        else
        {
          result.successful = false;
          result.reason = "Unknown parameter";
          RCLCPP_WARN(get_logger(), "Unknown parameter '%s'", parameter.get_name().c_str());
        }
      }

      return result;
    };

    m_param_callback_handle = add_on_set_parameters_callback(param_callback);
  }

  // initialize graphic user interface
  void initGui()
  {
    setWindowTitle("Handheld CTR");
    setGeometry(200, 200, 400, 300);
    setFocusPolicy(Qt::StrongFocus); // Ensure QT captures arrow keys

    // Create Layout
    QVBoxLayout *layout = new QVBoxLayout(this);

    // Create Label
    mp_label_1 = new QLabel("Waiting for data...", this);
    mp_label_1->setAlignment(Qt::AlignLeft);
    mp_label_2 = new QLabel("Waiting for data...", this);
    mp_label_2->setAlignment(Qt::AlignLeft);
    layout->addWidget(mp_label_1);
    layout->addWidget(mp_label_2);

    // radio button group for controller mode
    mp_bulletCtrlmModeGroup = new QGroupBox("Control Mode", this);
    mp_bulletLayoutCtrlMode = new QVBoxLayout();
    mp_optionCtrlmMode0 = new QRadioButton("Config", this);
    mp_optionCtrlmMode1 = new QRadioButton("Manual", this);
    mp_optionCtrlmMode2 = new QRadioButton("Velocity", this);
    mp_optionCtrlmMode3 = new QRadioButton("Position", this);
    mp_optionCtrlmMode1->setChecked(true);

    mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode0);
    mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode1);
    mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode2);
    mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode3);
    mp_bulletCtrlmModeGroup->setLayout(mp_bulletLayoutCtrlMode);
    layout->addWidget(mp_bulletCtrlmModeGroup);

    // radio button group for transmistion limits
    mp_bulletTranLimGroup = new QGroupBox("Trans Limit", this);
    mp_bulletLayoutTranLim = new QVBoxLayout();
    mp_optionTranLim0 = new QRadioButton("ON", this);
    mp_optionTranLim1 = new QRadioButton("OFF", this);
    mp_optionTranLim1->setChecked(true);

    mp_bulletLayoutTranLim->addWidget(mp_optionTranLim0);
    mp_bulletLayoutTranLim->addWidget(mp_optionTranLim1);
    mp_bulletTranLimGroup->setLayout(mp_bulletLayoutTranLim);
    layout->addWidget(mp_bulletTranLimGroup);

    // Create Buttons
    QPushButton *enable_button = new QPushButton("Disable", this);
    QPushButton *find_button = new QPushButton("Find", this);
    QPushButton *wrench_pos_button = new QPushButton("Prepare to Position Wrench", this);
    mp_unlock_button = new QPushButton("Unlock", this);
    mp_lock_button = new QPushButton("Lock", this);
    QPushButton *setRotaryHome_button = new QPushButton("Set Rotary Home", this);
    QPushButton *home_button = new QPushButton("Go to Home", this);

    mp_unlock_button->setEnabled(false); // Disabling the button
    mp_lock_button->setEnabled(false);   // Disabling the button

    // Arrange Unlock & Lock buttons side by side
    QHBoxLayout *buttonRow = new QHBoxLayout();
    buttonRow->addWidget(mp_unlock_button);
    buttonRow->addWidget(mp_lock_button);

    // Add buttons to layout
    layout->addWidget(enable_button);
    layout->addWidget(find_button);
    layout->addWidget(wrench_pos_button);
    layout->addLayout(buttonRow);
    layout->addWidget(setRotaryHome_button);
    layout->addWidget(home_button);

    setLayout(layout); // Apply layout to the window

    // Connect buttons to ROS2 services
    connect(enable_button, &QPushButton::clicked, this, &RobotNode::toggleEnable);
    connect(find_button, &QPushButton::clicked, this, &RobotNode::findEncoders);
    connect(wrench_pos_button, &QPushButton::clicked, this, &RobotNode::engageColletsToWrench);
    connect(mp_lock_button, &QPushButton::clicked, this, &RobotNode::lockCollets);
    connect(mp_unlock_button, &QPushButton::clicked, this, &RobotNode::unlockCollets);
    connect(setRotaryHome_button, &QPushButton::clicked, this, &RobotNode::setRotaryHome);
    connect(home_button, &QPushButton::clicked, this, &RobotNode::gotoHome);

    // Signal connections for updating GUI elements based on ROS2 data
    connect(this, &RobotNode::update_enable_button_Text, enable_button, &QPushButton::setText);
    connect(this, &RobotNode::update_wrench_pos_button_Text, wrench_pos_button, &QPushButton::setText);
    connect(this, &RobotNode::dataReceived_1, this, &RobotNode::updateLabel_1);
    connect(this, &RobotNode::dataReceived_2, this, &RobotNode::updateLabel_2);

    connect(mp_optionCtrlmMode0, &QRadioButton::toggled, this, &RobotNode::onCtrlModeSelected);
    connect(mp_optionCtrlmMode1, &QRadioButton::toggled, this, &RobotNode::onCtrlModeSelected);
    connect(mp_optionCtrlmMode2, &QRadioButton::toggled, this, &RobotNode::onCtrlModeSelected);
    connect(mp_optionCtrlmMode3, &QRadioButton::toggled, this, &RobotNode::onCtrlModeSelected);

    connect(mp_optionTranLim0, &QRadioButton::toggled, this, &RobotNode::onTranLimSelected);
    connect(mp_optionTranLim1, &QRadioButton::toggled, this, &RobotNode::onTranLimSelected);
  }

  //
  void keyboardRead_timerCallback()
  {
    // Reset velocity vector
    m_xdot_manual = {0.0, 0.0, 0.0, 0.0};

    // Check for active keys and update movement values
    if (m_keysPressed.contains(Qt::Key_Right))
      m_xdot_manual[0] = 1.0;
    if (m_keysPressed.contains(Qt::Key_Left))
      m_xdot_manual[0] = -1.0;
    if (m_keysPressed.contains(Qt::Key_Up))
      m_xdot_manual[1] = 0.01;
    if (m_keysPressed.contains(Qt::Key_Down))
      m_xdot_manual[1] = -0.01;
    if (m_keysPressed.contains(Qt::Key_D))
      m_xdot_manual[2] = 1.0;
    if (m_keysPressed.contains(Qt::Key_A))
      m_xdot_manual[2] = -1.0;
    if (m_keysPressed.contains(Qt::Key_W))
      m_xdot_manual[3] = 0.01;
    if (m_keysPressed.contains(Qt::Key_S))
      m_xdot_manual[3] = -0.01;

    // Send updated movement command
    // std::cout << "vel: " << m_xdot_manual[0] << " ," << m_xdot_manual[1] << " ," << m_xdot_manual[2] << " ," << m_xdot_manual[3] << std::endl;
  }

  // Timer callback function to read the current joint configurations and publish them
  void robotStatus_timerCallback()
  {
    auto msg = interfaces::msg::Status();
    blaze::StaticVector<bool, 4UL> status;
    blaze::StaticVector<bool, 4UL> en_status;
    getEnableStatus(en_status);
    msg.enable[0] = en_status[0];
    msg.enable[1] = en_status[1];
    msg.enable[2] = en_status[2];
    msg.enable[3] = en_status[3];
    getEncoderStatus(status);
    msg.encoder[0] = status[0];
    msg.encoder[1] = status[1];
    msg.encoder[2] = status[2];
    msg.encoder[3] = status[3];
    getReachedStatus(status);
    msg.reached[0] = status[0];
    msg.reached[1] = status[1];
    msg.reached[2] = status[2];
    msg.reached[3] = status[3];
    if ((abs(m_x[1] - m_pos_preEngage[1]) < 0.001) && (abs(m_x[3] - m_pos_preEngage[3]) < 0.001))
      m_flag_readyToEngage = true;
    else
      m_flag_readyToEngage = false;
    if ((abs(m_x[1] - m_pos_engage[1]) < 0.003) && (abs(m_x[3] - m_pos_engage[3]) < 0.002))
      m_flagEngaged = true;
    else
      m_flagEngaged = false;
    msg.ready_to_engage = m_flag_readyToEngage;
    msg.engaged = m_flagEngaged;
    msg.unlocked = m_unlocked;
    msg.locked = m_locked;

    mp_unlock_button->setEnabled(m_flagEngaged); // Disabling the button
    mp_lock_button->setEnabled(m_flagEngaged);   // Disabling the button

    m_publisher_status->publish(msg);

    // update GUI button texts
    emit update_enable_button_Text((en_status[0] || en_status[1] || en_status[2] || en_status[3]) ? "Disable" : "Enable");
    emit update_wrench_pos_button_Text(m_flag_readyToEngage ? "Engage Collets" : "Prepare to Engage Collets");
  }

  // Timer callback function to read the current joint configurations and publish them
  void jointsConfig_timerCallback()
  {
    auto msg = interfaces::msg::Jointspace();
    getPos(m_x);
    getVel(m_xdot);
    getCurrent(m_current);
    getPosLimit(m_minCurrentPosLimit, m_maxCurrentPosLimit);

    minDynamicPosLimit[1] = std::max(innerStaticLimit[0], m_x[3] - linear_stage_max_gap);
    maxDynamicPosLimit[1] = std::min(innerStaticLimit[1], m_x[3] - linear_stage_min_gap);

    minDynamicPosLimit[3] = std::max(middleStaticLimit[0], m_x[1] + linear_stage_min_gap);
    maxDynamicPosLimit[3] = std::min(middleStaticLimit[1], m_x[1] + linear_stage_max_gap);

    msg.position[0UL] = m_x[0UL];
    msg.position[1UL] = m_x[1UL];
    msg.position[2UL] = m_x[2UL];
    msg.position[3UL] = m_x[3UL];
    msg.velocity[0UL] = m_xdot[0UL];
    msg.velocity[1UL] = m_xdot[1UL];
    msg.velocity[2UL] = m_xdot[2UL];
    msg.velocity[3UL] = m_xdot[3UL];
    msg.current[0UL] = m_current[0UL];
    msg.current[1UL] = m_current[1UL];
    msg.current[2UL] = m_current[2UL];
    msg.current[3UL] = m_current[3UL];
    m_publisher_joints->publish(msg);

    emit dataReceived_1(QString::number(m_x[0], 'f', 4), QString::number(m_xdot[0], 'f', 4), QString::number(m_current[0], 'f', 1),
                        QString::number(m_x[1], 'f', 4), QString::number(m_xdot[1], 'f', 4), QString::number(m_current[1], 'f', 1),
                        QString::number(m_x[2], 'f', 4), QString::number(m_xdot[2], 'f', 4), QString::number(m_current[2], 'f', 1),
                        QString::number(m_x[3], 'f', 4), QString::number(m_xdot[3], 'f', 4), QString::number(m_current[3], 'f', 1));

    emit dataReceived_2(QString::number(m_minCurrentPosLimit[0], 'f', 4), QString::number(m_maxCurrentPosLimit[0], 'f', 4),
                        QString::number(m_minCurrentPosLimit[1], 'f', 4), QString::number(m_maxCurrentPosLimit[1], 'f', 4),
                        QString::number(m_minCurrentPosLimit[2], 'f', 4), QString::number(m_maxCurrentPosLimit[2], 'f', 4),
                        QString::number(m_minCurrentPosLimit[3], 'f', 4), QString::number(m_maxCurrentPosLimit[3], 'f', 4));
  }

  // Timer callback function for joint space control loop
  void jointSpaceControl_callback()
  {
    blaze::StaticVector<double, 4L> q_dot_command, x_des, x_dot_des;

    if (!m_flag_manual)
    {
      x_des = m_x_des;
      x_dot_des = m_xdot_des;

      // joint_space_control_step
      m_x_error = x_des - m_x;
      m_x_error_int = m_x_error_int + m_x_error * m_sample_time * 1e-3;
      q_dot_command = m_x_error * m_kp + m_x_error_int * m_ki + x_dot_des;

      // joint_space_control_step(x_des, x_dot_des, q_dot_command);
      // Set_Target_Velocity(q_dot_command);
    }
  }

  // Set the target position/velocity in the robot - Depreciated
  void targetCommand_timerCallback()
  {
    switch (m_mode)
    {
    case CtrlMode::Manual:
      setTargetVel(m_xdot_manual);
      break;
    case CtrlMode::Velocity:
      setTargetVel(m_xdot_des);
      break;
    case CtrlMode::Position:
      setTargetPos(m_x_des);
      break;
    }
    if (m_trans_limit)
    {
      setPosLimit(minDynamicPosLimit, maxDynamicPosLimit);
    }
    else
    {
      setPosLimit(minDynamicPosLimitInf, maxDynamicPosLimitInf);
    }
  }

  // Subscription callback function to updates the target joint positions and velocities
  void jointSpaceTarget_callback(const interfaces::msg::Jointspace::ConstSharedPtr msg)
  {
    if (!m_flag_manual && !m_flag_use_target_action)
    {
      m_targpublisher_alive_tmep = true;
      m_x_des = blaze::StaticVector<double, 4UL>(0.00);

      switch (m_mode)
      {
      case CtrlMode::Velocity:
        m_xdot_des[0UL] = msg->velocity[0UL];
        m_xdot_des[1UL] = msg->velocity[1UL];
        m_xdot_des[2UL] = msg->velocity[2UL];
        m_xdot_des[3UL] = msg->velocity[3UL];
        break;
      case CtrlMode::Position:
        m_x_des[0UL] = msg->position[0UL];
        m_x_des[1UL] = msg->position[1UL];
        m_x_des[2UL] = msg->position[2UL];
        m_x_des[3UL] = msg->position[3UL];
        break;
      }
    }
  }

  // Subscription callback function updates the current catheter tip status using EMTracker topic
  void current_tool_callback(const interfaces::msg::Taskspace::ConstSharedPtr msg)
  {
    m_x_tip = {msg->p[1UL] * 1.00E3, -msg->p[0UL] * 1.00E3, msg->p[2UL] * 1.00E3};
    m_emtracker_alive_tmep = true;
  }

  // Service callback to perform the homing procedure
  void configService_callback(const std::shared_ptr<interfaces::srv::Config::Request> request, std::shared_ptr<interfaces::srv::Config::Response> response)
  {
    if (request->command == "ON")
    {
      m_flag_manual = true;
      response->position[0UL] = m_x[0UL];
      response->position[1UL] = m_x[1UL];
      response->position[2UL] = m_x[2UL];
      response->position[3UL] = m_x[3UL];
      response->success = true;
      response->message = "Manual mode activated";
      RCLCPP_INFO(this->get_logger(), "Manual mode activated");
    }
    else if (request->command == "OFF")
    {
      m_flag_manual = false;
      response->success = true;
      response->message = "Manual mode OFF";
      RCLCPP_INFO(this->get_logger(), "Manual mode deactivated");
    }

    // else if (request->command == "enable")
    // {
    //   enableOperation(true);
    //   // Set_Zero_Position(blaze::StaticVector<double, 4UL>(0.00));
    //   response->success = true;
    //   response->message = "enabled";
    //   RCLCPP_INFO(this->get_logger(), "enabled");
    // }

    // else if (request->command == "disable")
    // {
    //   enableOperation(false);
    //   m_flag_manual = false;
    //   response->success = true;
    //   response->message = "disabled";
    //   RCLCPP_INFO(this->get_logger(), "disabled");
    // }

    else if (request->command == "move")
    {
      if (m_flag_manual)
      {
        m_xdot_des[0UL] = request->target[0UL];
        m_xdot_des[1UL] = request->target[1UL];
        m_xdot_des[2UL] = request->target[2UL];
        m_xdot_des[3UL] = request->target[3UL];
        response->success = true;
        if (m_xdot_des[0] == 0 && m_xdot_des[1] == 0 && m_xdot_des[2] == 0 && m_xdot_des[3] == 0)
        {
          response->message = "Stopped";
        }
        else
        {
          response->message = "Moving";
        }
      }
      else
      {
        response->success = false;
        response->message = "Manual mode is not deactive";
      }
    }

    else if (request->command == "set_home")
    {
      if (m_flag_manual)
      {
        // Set_Zero_Position(blaze::StaticVector<double, 4UL>(0.00));
        m_x_des = blaze::StaticVector<double, 4UL>(0.00);
        response->success = true;
        response->message = "Home is set";
        response->position[0UL] = m_x[0UL];
        response->position[1UL] = m_x[1UL];
        response->position[2UL] = m_x[2UL];
        response->position[3UL] = m_x[3UL];
      }
      else
      {
        response->success = false;
        response->message = "Manual mode is not deactive";
      }
    }

    else
    {
      // RCLCPP_WARN(this->get_logger(), "Invalid command: ", request->command);
      response->success = false;
      response->message = "Invalid command";
    }
  }

  //
  void handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
  {
    // Get the result of the future object
    auto response = future.get();

    if (response->success)
    {
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
    }
  }

  // Timer callback function to check if the EMTracker is still alive
  void check_target_publisher_alive()
  {
    // Set the flag to false; it will be set to true if current_catheter_tip_callback is called
    if (m_targpublisher_alive_tmep)
    {
      if (!m_targpublisher_alive)
      {
        m_targpublisher_alive = true;
        RCLCPP_WARN(get_logger(), "publisher_node is alive");
      }
    }
    else
    {
      if (m_targpublisher_alive)
      {
        m_targpublisher_alive = false;
        // m_x_des = blaze::StaticVector<double, 4UL>(0.00);
        // m_x_dot_des = blaze::StaticVector<double, 4UL>(0.00);
        RCLCPP_WARN(get_logger(), "publisher_node is dead");
      }
    }
    m_targpublisher_alive_tmep = false;
  }

  // Wait in milliseconds
  void wait(int milliseconds)
  {
    rclcpp::Rate rate(1000.00 / milliseconds);
    rate.sleep();
  }

  static constexpr int m_sample_time = 20; //[ms]
  static constexpr blaze::StaticVector<double, 4> m_pos_preEngage = {0.0, -0.0550, 0.0, -0.0240};
  static constexpr blaze::StaticVector<double, 4> m_pos_engage = {0.0, -0.0465, 0.0, -0.0195};
  static constexpr double inner_back_pos = -0.1294;
  static constexpr double middle_back_pos = -0.0925;
  static constexpr double linear_stage_min_gap = 0.030;
  static constexpr double linear_stage_max_gap = 0.070;
  static constexpr blaze::StaticVector<double, 2> innerStaticLimit = {inner_back_pos + 0.001, -0.0240};
  static constexpr blaze::StaticVector<double, 2> middleStaticLimit = {middle_back_pos + 0.001, m_pos_preEngage[3]};

  static constexpr blaze::StaticVector<double, 4> minDynamicPosLimitInf = {-40 * M_PI, -0.50, -40 * M_PI, -0.50};
  static constexpr blaze::StaticVector<double, 4> maxDynamicPosLimitInf = {40 * M_PI, 0.50, 40 * M_PI, 0.50};

  blaze::StaticVector<double, 4> minDynamicPosLimit = {-20 * M_PI, 0.001, -20 * M_PI, 0.040};
  blaze::StaticVector<double, 4> maxDynamicPosLimit = {20 * M_PI, 0.088, 20 * M_PI, 0.129};

  // blaze::StaticVector<double, 2> middleDynamicLimit = {0.040, 0.129};
  // blaze::StaticVector<double, 2> innerDynamicLimit = {0.001, 0.088};

  bool m_flag_manual, m_flag_use_target_action, m_flag_enabled, m_trans_limit = false;
  bool m_emtracker_alive, m_emtracker_alive_tmep, m_targpublisher_alive, m_targpublisher_alive_tmep = false;
  bool m_flag_readyToEngage, m_flagEngaged, m_unlocked, m_locked = false;
  double m_kp, m_ki = 0.00;

  // Qt related variables
  QSet<int> m_keysPressed;
  QLabel *mp_label_1, *mp_label_2;
  QGroupBox *mp_bulletCtrlmModeGroup;
  QVBoxLayout *mp_bulletLayoutCtrlMode;
  QRadioButton *mp_optionCtrlmMode0, *mp_optionCtrlmMode1, *mp_optionCtrlmMode2, *mp_optionCtrlmMode3;
  QGroupBox *mp_bulletTranLimGroup;
  QVBoxLayout *mp_bulletLayoutTranLim;
  QRadioButton *mp_optionTranLim0, *mp_optionTranLim1;
  QPushButton *mp_unlock_button, *mp_lock_button;

  CtrlMode m_mode; // controller mode (manual, velocity, position)

  rclcpp::TimerBase::SharedPtr m_watchdog_timer_target;
  rclcpp::TimerBase::SharedPtr m_read_robot_timer;
  rclcpp::TimerBase::SharedPtr m_joints_config_timer;
  rclcpp::TimerBase::SharedPtr m_control_loop_timer;
  rclcpp::TimerBase::SharedPtr m_position_control_timer;
  rclcpp::TimerBase::SharedPtr m_system_identification_timer;
  rclcpp::TimerBase::SharedPtr m_read_key_timer;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_joints;
  rclcpp::Publisher<interfaces::msg::Status>::SharedPtr m_publisher_status;
  rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_target;
  rclcpp::Service<interfaces::srv::Config>::SharedPtr m_homing_service;
  // rclcpp_action::Server<interfaces::action::Jointstarget>::SharedPtr m_action_server;

  rclcpp::CallbackGroup::SharedPtr m_cbGroup1, m_cbGroup2, m_cbGroup3, m_cbGroup4, m_cbGroup5, m_cbGroup6;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  blaze::StaticVector<double, 4UL> m_x, m_x_des, m_x_error, m_x_abs;                                // in SI units
  blaze::StaticVector<double, 4UL> m_xdot, m_xdot_manual, m_xdot_des, m_xdot_error, m_xdot_forward; // in SI units
  blaze::StaticVector<double, 4UL> m_x_error_int;                                                   // in SI units
  blaze::StaticVector<double, 4UL> m_current;
  blaze::StaticVector<double, 4UL> m_minCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 4UL> m_maxCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
  blaze::StaticVector<double, 3UL> m_x_tip;

  blaze::StaticVector<double, 4UL> m_maxTorqueNegative;
  blaze::StaticVector<double, 4UL> m_maxTorquePositive;
  blaze::StaticVector<double, 4UL> m_maxDcc; // [deg/s^2] and [mm/s^2]
  blaze::StaticVector<double, 4UL> m_maxVel; // [deg/s] and [mm/s]
  blaze::StaticVector<double, 4UL> m_maxAcc;
};

#include "robot_node.moc"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<RobotNode>();
  node->show();     // Display the GUI
  node->setFocus(); // Force focus on startup to capture arrow keys

  // Use MultiThreadedExecutor with multiple threads
  std::thread ros_spin_thread([&]()
                              {
       rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 7);
       executor.add_node(node);
       executor.spin(); 
       rclcpp::shutdown(); });

  int result = app.exec();
  rclcpp::shutdown();
  ros_spin_thread.join();
  return result;
}

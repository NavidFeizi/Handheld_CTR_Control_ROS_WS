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
#include <QButtonGroup>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <iostream> // Required for std::cout
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/status.hpp"
#include "interfaces/msg/interface.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/config.hpp"
#include <thread>
#include <vector>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

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

class GuiNode : public QWidget, public rclcpp::Node
{        // Fix: QObject first
Q_OBJECT // Needed for Qt signals/slots

    public : explicit GuiNode(QWidget *parent = nullptr) : QWidget(parent), rclcpp::Node("qt_gui_node")
    {
        initGui();
        initRosInterfaces();
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        m_keysPressed.insert(event->key()); // Store pressed key
    }

    void keyReleaseEvent(QKeyEvent *event) override
    {
        m_keysPressed.remove(event->key()); // Remove key when released
    }

signals:                                              // Functions declared (Qt specific)
    void update_enable_button_Text(QString text);     // Signal to update button text
    void update_wrench_pos_button_Text(QString text); // Signal to update button text

private slots: // Functions that receive and handle signals - can be connected to signals (Qt specific)
    //
    void toggleEnable()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "toggleEnable";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_enable_client->async_send_request(request, response_received_callback);
    }

    //
    void findLinearHome()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "findLinearHome";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void engageCollets()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "engageCollets";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void disengageCollets()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "disengageCollets";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void lockCollets()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "lockCollets";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void unlockCollets()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "unlockCollets";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void findRotaryHome()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "findRotaryHome";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void goHome()
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "goHome";
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
    }

    //
    void onCtrlModeClicked(int mode)
    {
        // Prevent immediate visual check
        mp_ctrl_mode_group->setExclusive(false);
        mp_optionCtrlmMode0->setChecked(false);
        mp_optionCtrlmMode1->setChecked(false);
        mp_optionCtrlmMode2->setChecked(false);
        mp_optionCtrlmMode3->setChecked(false);
        mp_ctrl_mode_group->setExclusive(true);

        // Send request
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "setCtrlMode";
        request->value = mode;
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
        // RCLCPP_INFO(this->get_logger(), "Sent CtrlMode change request: %d", mode);
    }

    //
    void onTranLimClicked(int enable)
    {
        // Prevent immediate visual check
        mp_trans_lim_group->setExclusive(false);
        mp_optionTranLimOff->setChecked(false);
        mp_optionTranLimOn->setChecked(false);
        mp_trans_lim_group->setExclusive(true);

        // Send request
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = "setTransLimMode";
        request->value = enable;
        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_robot_config_client->async_send_request(request, response_received_callback);
        // RCLCPP_INFO(this->get_logger(), "Sent TransLim change request: %d", enable);
    }

private:
    // initialize graphic user interface
    void initGui()
    {
        setWindowTitle("Handheld CTR");
        setGeometry(200, 200, 550, 800);
        setFocusPolicy(Qt::StrongFocus); // Ensure QT captures arrow keys

        // Create Layout
        QVBoxLayout *layout = new QVBoxLayout(this);

        QLabel *tableTitle = new QLabel("Robot info", this);
        tableTitle->setAlignment(Qt::AlignCenter); // optional: center-align the title
        layout->addWidget(tableTitle);
        mp_table_robot = new QTableWidget(1, 5, this);
        mp_table_robot->setHorizontalHeaderLabels({"Head", "Enable", "Engaged", "Lock", "EStop"});
        mp_table_robot->setVerticalHeaderLabels({" Status  "});
        mp_table_robot->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_robot->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_robot->setFixedHeight(50);
        layout->addWidget(mp_table_robot);
        setLayout(layout);

        tableTitle = new QLabel("Joints info", this);
        tableTitle->setAlignment(Qt::AlignCenter); // optional: center-align the title
        layout->addWidget(tableTitle);
        mp_table_joints = new QTableWidget(8, 4, this);
        mp_table_joints->setHorizontalHeaderLabels({"Inr Rot", "Inr Trn", "Mdl Rot", "Mdl Trn"});
        mp_table_joints->setVerticalHeaderLabels({" Enable", " Position", " Velocity", " Current", " Pos limit min ", " Pos limit max ", " Encoder set", " Reached"});
        mp_table_joints->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_joints->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_joints->setFixedHeight(300);
        layout->addWidget(mp_table_joints);
        setLayout(layout);

        tableTitle = new QLabel("Interface", this);
        tableTitle->setAlignment(Qt::AlignCenter); // optional: center-align the title
        layout->addWidget(tableTitle);
        mp_table_interface = new QTableWidget(1, 7, this);
        mp_table_interface->setHorizontalHeaderLabels({"1", "2", "3", "4", "5", "6", "7"});
        mp_table_interface->setVerticalHeaderLabels({" Pressed  "});
        mp_table_interface->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_interface->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_interface->setFixedHeight(50);
        layout->addWidget(mp_table_interface);
        setLayout(layout);

        tableTitle = new QLabel("Controls", this);
        tableTitle->setAlignment(Qt::AlignCenter); // optional: center-align the title
        layout->addWidget(tableTitle);

        QHBoxLayout *bulletLayoutGroups = new QHBoxLayout();
        // radio button group for controller mode
        mp_bulletCtrlmModeGroup = new QGroupBox("Control Mode", this); // for visual grouping
        mp_bulletLayoutCtrlMode = new QVBoxLayout();
        mp_optionCtrlmMode0 = new QRadioButton("Config", this);
        mp_optionCtrlmMode1 = new QRadioButton("Manual", this);
        mp_optionCtrlmMode2 = new QRadioButton("Velocity", this);
        mp_optionCtrlmMode3 = new QRadioButton("Position", this);
        // for managing click events and mapping to enum
        mp_ctrl_mode_group = new QButtonGroup(this);
        mp_ctrl_mode_group->addButton(mp_optionCtrlmMode0, static_cast<int>(CtrlMode::Config));
        mp_ctrl_mode_group->addButton(mp_optionCtrlmMode1, static_cast<int>(CtrlMode::Manual));
        mp_ctrl_mode_group->addButton(mp_optionCtrlmMode2, static_cast<int>(CtrlMode::Velocity));
        mp_ctrl_mode_group->addButton(mp_optionCtrlmMode3, static_cast<int>(CtrlMode::Position));
        mp_ctrl_mode_group->setExclusive(true);
        // for UI display
        mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode0);
        mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode1);
        mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode2);
        mp_bulletLayoutCtrlMode->addWidget(mp_optionCtrlmMode3);
        mp_bulletCtrlmModeGroup->setLayout(mp_bulletLayoutCtrlMode);

        // radio button group for transmistion limits
        mp_bulletTranLimGroup = new QGroupBox("Trans Limit", this); // for visual grouping
        mp_bulletLayoutTranLim = new QVBoxLayout();
        mp_optionTranLimOn = new QRadioButton("ON", this);
        mp_optionTranLimOff = new QRadioButton("OFF", this);
        // for managing click events and mapping
        mp_trans_lim_group = new QButtonGroup(this);
        mp_trans_lim_group->addButton(mp_optionTranLimOn, 1);
        mp_trans_lim_group->addButton(mp_optionTranLimOff, 0);
        mp_trans_lim_group->setExclusive(true);
        // for UI display
        mp_bulletLayoutTranLim->addWidget(mp_optionTranLimOff);
        mp_bulletLayoutTranLim->addWidget(mp_optionTranLimOn);
        mp_bulletTranLimGroup->setLayout(mp_bulletLayoutTranLim);

        bulletLayoutGroups->addWidget(mp_bulletCtrlmModeGroup);
        bulletLayoutGroups->addWidget(mp_bulletTranLimGroup);
        layout->addLayout(bulletLayoutGroups);

        // Create Buttons
        QPushButton *enable_button = new QPushButton("Disable", this);
        QPushButton *find_button = new QPushButton("Find", this);
        QPushButton *engage_button = new QPushButton("Engage", this);
        QPushButton *disengage_button = new QPushButton("Disengage", this);
        mp_unlock_button = new QPushButton("Unlock", this);
        mp_lock_button = new QPushButton("Lock", this);
        QPushButton *setRotaryHome_button = new QPushButton("Find Rotary Home", this);
        QPushButton *home_button = new QPushButton("Go to Home", this);

        mp_unlock_button->setEnabled(false); // Disabling the button
        mp_lock_button->setEnabled(false);   // Disabling the button

        // Arrange Unlock & Lock buttons side by side
        QHBoxLayout *buttonRow_0 = new QHBoxLayout();
        buttonRow_0->addWidget(engage_button);
        buttonRow_0->addWidget(disengage_button);

        // Arrange Unlock & Lock buttons side by side
        QHBoxLayout *buttonRow_1 = new QHBoxLayout();
        buttonRow_1->addWidget(mp_unlock_button);
        buttonRow_1->addWidget(mp_lock_button);

        // Add buttons to layout
        layout->addWidget(enable_button);
        layout->addWidget(find_button);
        layout->addLayout(buttonRow_0);
        layout->addLayout(buttonRow_1);
        layout->addWidget(setRotaryHome_button);
        layout->addWidget(home_button);

        setLayout(layout); // Apply layout to the window

        // Connect buttons to ROS2 services
        connect(enable_button, &QPushButton::clicked, this, &GuiNode::toggleEnable);
        connect(find_button, &QPushButton::clicked, this, &GuiNode::findLinearHome);
        connect(engage_button, &QPushButton::clicked, this, &GuiNode::engageCollets);
        connect(disengage_button, &QPushButton::clicked, this, &GuiNode::disengageCollets);
        connect(mp_lock_button, &QPushButton::clicked, this, &GuiNode::lockCollets);
        connect(mp_unlock_button, &QPushButton::clicked, this, &GuiNode::unlockCollets);
        connect(setRotaryHome_button, &QPushButton::clicked, this, &GuiNode::findRotaryHome);
        connect(home_button, &QPushButton::clicked, this, &GuiNode::goHome);

        // Signal connections for updating GUI elements based on ROS2 data
        connect(this, &GuiNode::update_enable_button_Text, enable_button, &QPushButton::setText);
        connect(this, &GuiNode::update_wrench_pos_button_Text, engage_button, &QPushButton::setText);

        connect(mp_ctrl_mode_group, QOverload<int>::of(&QButtonGroup::idClicked),
                this, &GuiNode::onCtrlModeClicked);

        connect(mp_trans_lim_group, QOverload<int>::of(&QButtonGroup::idClicked),
                this, &GuiNode::onTranLimClicked);
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

        if (m_mode == CtrlMode::Manual)
        {
            auto msg = interfaces::msg::Jointspace();
            msg.velocity[0UL] = m_xdot_manual[0UL];
            msg.velocity[1UL] = m_xdot_manual[1UL];
            msg.velocity[2UL] = m_xdot_manual[2UL];
            msg.velocity[3UL] = m_xdot_manual[3UL];
            m_publisher_manual_vel->publish(msg);
        }
    }

    void updateTable_Joints(blaze::StaticVector<double, 4> m_x,
                            blaze::StaticVector<double, 4> m_xdot,
                            blaze::StaticVector<double, 4> m_current)
    {
        mp_table_joints->setItem(1, 0, new QTableWidgetItem(QString::number(m_x[0], 'f', 2)));
        mp_table_joints->setItem(1, 1, new QTableWidgetItem(QString::number(m_x[1], 'f', 4)));
        mp_table_joints->setItem(1, 2, new QTableWidgetItem(QString::number(m_x[2], 'f', 2)));
        mp_table_joints->setItem(1, 3, new QTableWidgetItem(QString::number(m_x[3], 'f', 4)));

        mp_table_joints->setItem(2, 0, new QTableWidgetItem(QString::number(m_xdot[0], 'f', 2)));
        mp_table_joints->setItem(2, 1, new QTableWidgetItem(QString::number(m_xdot[1], 'f', 4)));
        mp_table_joints->setItem(2, 2, new QTableWidgetItem(QString::number(m_xdot[2], 'f', 2)));
        mp_table_joints->setItem(2, 3, new QTableWidgetItem(QString::number(m_xdot[3], 'f', 4)));

        mp_table_joints->setItem(3, 0, new QTableWidgetItem(QString::number(m_current[0], 'f', 1)));
        mp_table_joints->setItem(3, 1, new QTableWidgetItem(QString::number(m_current[1], 'f', 1)));
        mp_table_joints->setItem(3, 2, new QTableWidgetItem(QString::number(m_current[2], 'f', 1)));
        mp_table_joints->setItem(3, 3, new QTableWidgetItem(QString::number(m_current[3], 'f', 1)));
    }

    void updateTable_JointsStatus(blaze::StaticVector<bool, 4> enable,
                                  blaze::StaticVector<double, 4> minPosLimit,
                                  blaze::StaticVector<double, 4> maxPosLimit,
                                  blaze::StaticVector<bool, 4> encoder,
                                  blaze::StaticVector<bool, 4> reached)
    {
        mp_table_joints->setItem(0, 0, new QTableWidgetItem(enable[0] ? "ON" : "OFF"));
        mp_table_joints->setItem(0, 1, new QTableWidgetItem(enable[1] ? "ON" : "OFF"));
        mp_table_joints->setItem(0, 2, new QTableWidgetItem(enable[2] ? "ON" : "OFF"));
        mp_table_joints->setItem(0, 3, new QTableWidgetItem(enable[3] ? "ON" : "OFF"));

        mp_table_joints->setItem(4, 0, new QTableWidgetItem(QString::number(minPosLimit[0], 'f', 2)));
        mp_table_joints->setItem(4, 1, new QTableWidgetItem(QString::number(minPosLimit[1], 'f', 4)));
        mp_table_joints->setItem(4, 2, new QTableWidgetItem(QString::number(minPosLimit[2], 'f', 2)));
        mp_table_joints->setItem(4, 3, new QTableWidgetItem(QString::number(minPosLimit[3], 'f', 4)));

        mp_table_joints->setItem(5, 0, new QTableWidgetItem(QString::number(maxPosLimit[0], 'f', 2)));
        mp_table_joints->setItem(5, 1, new QTableWidgetItem(QString::number(maxPosLimit[1], 'f', 4)));
        mp_table_joints->setItem(5, 2, new QTableWidgetItem(QString::number(maxPosLimit[2], 'f', 2)));
        mp_table_joints->setItem(5, 3, new QTableWidgetItem(QString::number(maxPosLimit[3], 'f', 4)));

        mp_table_joints->setItem(6, 0, new QTableWidgetItem(QString::number(static_cast<int>(encoder[0]))));
        mp_table_joints->setItem(6, 1, new QTableWidgetItem(QString::number(static_cast<int>(encoder[1]))));
        mp_table_joints->setItem(6, 2, new QTableWidgetItem(QString::number(static_cast<int>(encoder[2]))));
        mp_table_joints->setItem(6, 3, new QTableWidgetItem(QString::number(static_cast<int>(encoder[3]))));

        mp_table_joints->setItem(7, 0, new QTableWidgetItem(QString::number(static_cast<int>(reached[0]))));
        mp_table_joints->setItem(7, 1, new QTableWidgetItem(QString::number(static_cast<int>(reached[1]))));
        mp_table_joints->setItem(7, 2, new QTableWidgetItem(QString::number(static_cast<int>(reached[2]))));
        mp_table_joints->setItem(7, 3, new QTableWidgetItem(QString::number(static_cast<int>(reached[3]))));
    }

    void updateTable_RobotStatus(bool enable, bool head, bool attached, int locked)
    {
        mp_table_robot->setItem(0, 0, new QTableWidgetItem(head ? "Attached" : "Detached"));
        mp_table_robot->setItem(0, 1, new QTableWidgetItem(enable ? "ON" : "OFF"));
        mp_table_robot->setItem(0, 2, new QTableWidgetItem(attached ? "Yes" : "No"));
        if (locked == 1)
        {
            mp_table_robot->setItem(0, 3, new QTableWidgetItem("Locked"));
        }
        else if (locked == -1)
        {
            mp_table_robot->setItem(0, 3, new QTableWidgetItem("Unlocked"));
        }
        else
        {
            mp_table_robot->setItem(0, 3, new QTableWidgetItem("Unknown"));
        }
    }

    void updateTable_RobotStatus(std::array<bool, 7> pressed)
    {
        mp_table_interface->setItem(0, 0, new QTableWidgetItem(QString::number(static_cast<int>(pressed[0]))));
        mp_table_interface->setItem(0, 1, new QTableWidgetItem(QString::number(static_cast<int>(pressed[1]))));
        mp_table_interface->setItem(0, 2, new QTableWidgetItem(QString::number(static_cast<int>(pressed[2]))));
        mp_table_interface->setItem(0, 3, new QTableWidgetItem(QString::number(static_cast<int>(pressed[3]))));
        mp_table_interface->setItem(0, 4, new QTableWidgetItem(QString::number(static_cast<int>(pressed[4]))));
        mp_table_interface->setItem(0, 5, new QTableWidgetItem(QString::number(static_cast<int>(pressed[5]))));
        mp_table_interface->setItem(0, 6, new QTableWidgetItem(QString::number(static_cast<int>(pressed[6]))));
    }

    //
    void initRosInterfaces()
    {
        m_subscription_joints = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&GuiNode::jointsConfig_timerCallback, this, std::placeholders::_1));
        m_subscription_status = create_subscription<interfaces::msg::Status>("robot_status", 10, std::bind(&GuiNode::robot_status_callback, this, std::placeholders::_1));
        m_subscription_interface = create_subscription<interfaces::msg::Interface>("manual_interface", 10, std::bind(&GuiNode::manual_interface_callback, this, std::placeholders::_1));

        m_publisher_manual_vel = create_publisher<interfaces::msg::Jointspace>("joint_space/manual_vel", 10);

        m_cbGroup1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_read_key_timer = create_wall_timer(20ms, std::bind(&GuiNode::keyboardRead_timerCallback, this), m_cbGroup1);

        m_robot_config_client = create_client<interfaces::srv::Config>("robot_config");
        while (!m_robot_config_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Robot config service not available, waiting ...");
        }

        m_robot_enable_client = create_client<interfaces::srv::Config>("robot_enable");
        while (!m_robot_enable_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Robot config service not available, waiting ...");
        }
    }

    //
    void jointsConfig_timerCallback(const interfaces::msg::Jointspace::SharedPtr msg)
    {
        for (int i = 0; i < 4; i++)
        {
            m_x[i] = msg->position[i];
            m_xdot[i] = msg->velocity[i];
            m_current[i] = msg->current[i];
        }
        updateTable_Joints(m_x, m_xdot, m_current);
    }

    //
    void robot_status_callback(const interfaces::msg::Status::SharedPtr msg)
    {
        m_enabled = msg->enable[0] * msg->enable[1] * msg->enable[2] * msg->enable[3];
        m_reached = msg->reached[0] * msg->reached[1] * msg->reached[2] * msg->reached[3];
        m_encoder = msg->encoder[0] * msg->encoder[1] * msg->encoder[2] * msg->encoder[3];
        m_ready_to_engage = msg->ready_to_engage;
        m_engaged = msg->engaged;
        m_locked = msg->locked;
        m_head_attached = msg->head_attached;

        m_trans_limit = msg->trans_limit_en;
        m_mode = static_cast<CtrlMode>(msg->control_mode);

        for (int i = 0; i < 4; i++)
        {
            m_minCurrentPosLimit[i] = msg->min_pos_limit[i];
            m_maxCurrentPosLimit[i] = msg->max_pos_limit[i];
            m_enabledJoints[i] = msg->enable[i];
            m_encoderJoints[i] = msg->encoder[i];
            m_reachedJoints[i] = msg->reached[i];
        }

        mp_unlock_button->setEnabled(m_engaged); // Enable/Disabling the lock button
        mp_lock_button->setEnabled(m_engaged);   // Enable/Disabling the unlock button

        if (m_mode_prev != m_mode)
        {
            QMetaObject::invokeMethod(this, [this]()
                                      {
                mp_ctrl_mode_group->setExclusive(false);
                mp_optionCtrlmMode0->setChecked(false);
                mp_optionCtrlmMode1->setChecked(false);
                mp_optionCtrlmMode2->setChecked(false);
                mp_optionCtrlmMode3->setChecked(false);
                switch (m_mode) {
                  case CtrlMode::Config:   mp_optionCtrlmMode0->setChecked(true); mp_bulletTranLimGroup->setEnabled(false); break;
                  case CtrlMode::Manual:   mp_optionCtrlmMode1->setChecked(true); mp_bulletTranLimGroup->setEnabled(true); break;
                  case CtrlMode::Velocity: mp_optionCtrlmMode2->setChecked(true); mp_bulletTranLimGroup->setEnabled(false); break;
                  case CtrlMode::Position: mp_optionCtrlmMode3->setChecked(true); mp_bulletTranLimGroup->setEnabled(false); break;
                }
                mp_ctrl_mode_group->setExclusive(true); }, Qt::QueuedConnection);
            m_mode_prev = m_mode;
        }
        if (m_trans_limit != m_trans_limit_prev)
        {
            QMetaObject::invokeMethod(this, [this]()
                                      {
                mp_trans_lim_group->setExclusive(false);
                mp_optionTranLimOn->setChecked(false);
                mp_optionTranLimOff->setChecked(false);
                if (m_trans_limit)
                    mp_optionTranLimOn->setChecked(true); // ON
                else
                    mp_optionTranLimOff->setChecked(true); // OFF
                mp_trans_lim_group->setExclusive(true); }, Qt::QueuedConnection); // Ensure it's run on the Qt GUI thread
            m_trans_limit_prev = m_trans_limit;
        }

        updateTable_JointsStatus(m_enabledJoints, m_minCurrentPosLimit, m_maxCurrentPosLimit, m_encoderJoints, m_reachedJoints);
        updateTable_RobotStatus(m_enabled, m_head_attached, m_engaged, m_locked);

        emit update_enable_button_Text(m_enabled ? "Disable" : "Enable");
        // emit update_wrench_pos_button_Text(m_ready_to_engage ? "Engage Collets" : "Prepare to Engage Collets");
    }

    //
    void manual_interface_callback(const interfaces::msg::Interface::SharedPtr msg)
    {
        for (int i = 0; i < 7; i++)
        {
            m_interface_key[i] = msg->interface_key[i];
        }
        updateTable_RobotStatus(m_interface_key);
    }

    // Set the target position/velocity in the robot - Depreciated
    void targetCommand_timerCallback()
    {
        // switch (m_mode)
        // {
        // case CtrlMode::Manual:
        //     setTargetVel(m_xdot_manual);
        //     break;
        // case CtrlMode::Velocity:
        //     setTargetVel(m_xdot_des);
        //     break;
        // case CtrlMode::Position:
        //     setTargetPos(m_x_des);
        //     break;
        // }
        // if (m_trans_limit)
        // {
        //     setPosLimit(minDynamicPosLimit, maxDynamicPosLimit);
        // }
        // else
        // {
        //     setPosLimit(minDynamicPosLimitInf, maxDynamicPosLimitInf);
        // }
    }

    //
    void handle_service_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
    {
        // Get the result of the future object
        auto response = future.get();

        if (response->success)
        {
            // RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
        }
        else
        {
            // RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
        }
    }

    bool m_flag_manual, m_flag_use_target_action, m_flag_enabled, m_trans_limit, m_trans_limit_prev = false;

    // Qt related variables
    QSet<int> m_keysPressed;
    // QLabel *mp_label_1, *mp_label_2, *mp_label_3, *mp_label_4;
    QGroupBox *mp_bulletCtrlmModeGroup;
    QButtonGroup *mp_ctrl_mode_group, *mp_trans_lim_group;
    QVBoxLayout *mp_bulletLayoutCtrlMode;
    QRadioButton *mp_optionCtrlmMode0, *mp_optionCtrlmMode1, *mp_optionCtrlmMode2, *mp_optionCtrlmMode3;
    QGroupBox *mp_bulletTranLimGroup;
    QVBoxLayout *mp_bulletLayoutTranLim;
    QRadioButton *mp_optionTranLimOff, *mp_optionTranLimOn;
    QPushButton *mp_unlock_button, *mp_lock_button;
    QTableWidget *mp_table_joints, *mp_table_robot, *mp_table_interface;

    CtrlMode m_mode, m_mode_prev; // controller mode (manual, velocity, position)

    blaze::StaticVector<double, 4> m_com_vel; // Velocity vector

    blaze::StaticVector<double, 4UL> m_x, m_x_des, m_x_error, m_x_abs;                                // in SI units
    blaze::StaticVector<double, 4UL> m_xdot, m_xdot_manual, m_xdot_des, m_xdot_error, m_xdot_forward; // in SI units
    blaze::StaticVector<double, 4UL> m_x_error_int;                                                   // in SI units
    blaze::StaticVector<double, 4UL> m_current;

    blaze::StaticVector<double, 4UL> m_minCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
    blaze::StaticVector<double, 4UL> m_maxCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);

    std::array<bool, 7> m_interface_key = {0, 0, 0, 0, 0, 0, 0};

    bool m_enabled, m_reached, m_encoder = false;                                     // Tracks button state
    bool m_engaged, m_ready_to_engage, m_head_attached = false; // Tracks button state
    int m_locked;

    blaze::StaticVector<bool, 4UL> m_enabledJoints, m_encoderJoints, m_reachedJoints;

    rclcpp::CallbackGroup::SharedPtr m_cbGroup1;

    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subscription_status;
    rclcpp::Subscription<interfaces::msg::Interface>::SharedPtr m_subscription_interface;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_config_client;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_enable_client;
    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_manual_vel;
    rclcpp::TimerBase::SharedPtr m_read_key_timer;
};

#include "qt_node.moc"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<GuiNode>();
    node->show();     // Display the GUI
    node->setFocus(); // Force focus on startup to capture arrow keys

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

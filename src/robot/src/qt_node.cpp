#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
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
#include <QBrush>
#include <QColor>
#include <QGridLayout>

#include <atomic>
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
#include "interfaces/msg/taskspace.hpp"
#include "interfaces/msg/force.hpp"
#include "interfaces/msg/ekf_residual.hpp"
#include "interfaces/srv/config.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
        m_xdot_command = {2.0, 0.012, 2.0, 0.012};
        initGui();
        initRosInterfaces();
    }

    // Key state is written here (Qt thread) and read by the 20 ms ROS wall
    // timer (executor thread) — a QSet is not thread-safe, so the 8 tracked
    // keys live in an atomic bitmask instead.
    static int keyBit(int qt_key)
    {
        switch (qt_key)
        {
        case Qt::Key_Right: return 0;
        case Qt::Key_Left: return 1;
        case Qt::Key_Up: return 2;
        case Qt::Key_Down: return 3;
        case Qt::Key_D: return 4;
        case Qt::Key_A: return 5;
        case Qt::Key_W: return 6;
        case Qt::Key_S: return 7;
        default: return -1;
        }
    }

    static bool keyDown(uint32_t keys, int qt_key)
    {
        const int bit = keyBit(qt_key);
        return bit >= 0 && (keys & (1u << bit)) != 0u;
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        const int bit = keyBit(event->key());
        if (bit >= 0)
            m_keysPressed.fetch_or(1u << bit);
    }

    void keyReleaseEvent(QKeyEvent *event) override
    {
        const int bit = keyBit(event->key());
        if (bit >= 0)
            m_keysPressed.fetch_and(~(1u << bit));
    }

signals:                                              // Functions declared (Qt specific)
    void update_enable_button_Text(QString text);     // Signal to update button text
    void update_wrench_pos_button_Text(QString text); // Signal to update button text

private slots: // Functions that receive and handle signals - can be connected to signals (Qt specific)
    void sendConfigCommand(const std::string &command, bool use_enable_service)
    {
        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = command;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto client = use_enable_service ? m_robot_enable_client : m_robot_config_client;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        (void)client->async_send_request(request, response_received_callback);
    }

    void handleFreezeButtonClicked()
    {
        m_robot_frozen = !m_robot_frozen;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = m_robot_frozen;

        auto result_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                QMetaObject::invokeMethod(
                    this,
                    [this]()
                    {
                        mp_freeze_button->setText(m_robot_frozen ? "Unfreeze Robot" : "Freeze Robot");
                        if (m_robot_frozen)
                        {
                            mp_freeze_button->setStyleSheet("");
                        }
                        else
                        {
                            mp_freeze_button->setStyleSheet("background-color: rgb(255, 0, 0); color: white;");
                        }
                    },
                    Qt::QueuedConnection);
                RCLCPP_INFO(this->get_logger(), "Robot %s", m_robot_frozen ? "frozen" : "unfrozen");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to %s robot: %s",
                             m_robot_frozen ? "freeze" : "unfreeze", response->message.c_str());
            }
        };

        m_freeze_robot_client->async_send_request(request, result_callback);
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

    void handleIgtlConnectClicked()
    {
        if (!m_igtl_connect_client || !m_igtl_connect_client->service_is_ready())
        {
            RCLCPP_WARN(this->get_logger(), "igtl_bridge/connect service is not ready.");
            return;
        }

        const bool currently_connected = m_igtl_connected.load(std::memory_order_relaxed);
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = !currently_connected; // connect if disconnected, disconnect if connected

        auto result_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "IGTLink connect request succeeded: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "IGTLink connect request failed: %s", response->message.c_str());
            }
        };

        m_igtl_connect_client->async_send_request(request, result_callback);
    }

private:
    // initialize graphic user interface
    void initGui()
    {
        setWindowTitle("Handheld CTR");
        setGeometry(200, 200, 900, 850);
        setFocusPolicy(Qt::StrongFocus); // Ensure QT captures arrow keys

        // Outer vertical layout: Robot info spans full window width at top;
        // equal-width left/right columns below.
        QVBoxLayout *outerLayout = new QVBoxLayout(this);
        QHBoxLayout *columnsLayout = new QHBoxLayout();
        QVBoxLayout *layout = new QVBoxLayout();
        QVBoxLayout *rightLayout = new QVBoxLayout();

        // --- Robot info (full window width, 8 columns) ---
        QLabel *tableTitle = new QLabel("Robot info", this);
        tableTitle->setAlignment(Qt::AlignCenter);
        outerLayout->addWidget(tableTitle);
        mp_table_robot = new QTableWidget(1, 8, this);
        mp_table_robot->setHorizontalHeaderLabels({"Head", "Enable", "Procedure", "Engaged", "Lock", "EStop", "EMTracker", "IGTLink"});
        mp_table_robot->setVerticalHeaderLabels({" Status  "});
        mp_table_robot->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_robot->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_robot->setFixedHeight(50);
        outerLayout->addWidget(mp_table_robot);
        outerLayout->addLayout(columnsLayout);

        tableTitle = new QLabel("Joints info", this);
        tableTitle->setAlignment(Qt::AlignCenter); // optional: center-align the title
        layout->addWidget(tableTitle);
        mp_table_joints = new QTableWidget(10, 4, this);
        mp_table_joints->setHorizontalHeaderLabels({"Inr Rot", "Inr Trn", "Mdl Rot", "Mdl Trn"});
        mp_table_joints->setVerticalHeaderLabels({" Enable", " Position", " Velocity", " Current", " Pos limit min ", " Pos limit max ", " Encoder set", " Reached", " CPU Temp ", " Winding Temp "});
        mp_table_joints->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_joints->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_joints->setFixedHeight(330);
        layout->addWidget(mp_table_joints);

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

        tableTitle = new QLabel("Controls", this);
        tableTitle->setAlignment(Qt::AlignCenter); // optional: center-align the title
        layout->addWidget(tableTitle);

        QHBoxLayout *bulletLayoutGroups = new QHBoxLayout();
        // radio button group for controller mode
        mp_bulletCtrlmModeGroup = new QGroupBox("Control Mode", this); // for visual grouping
        QGridLayout *ctrlModeGridLayout = new QGridLayout();
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
        // 2-column grid: Config/Manual in left column, Velocity/Position in right
        ctrlModeGridLayout->addWidget(mp_optionCtrlmMode0, 0, 0);
        ctrlModeGridLayout->addWidget(mp_optionCtrlmMode1, 1, 0);
        ctrlModeGridLayout->addWidget(mp_optionCtrlmMode2, 0, 1);
        ctrlModeGridLayout->addWidget(mp_optionCtrlmMode3, 1, 1);
        mp_bulletCtrlmModeGroup->setLayout(ctrlModeGridLayout);

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

        // Create Buttons (Freeze Robot is placed in the right column)
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

        // Arrange Engage & Disengage buttons side by side
        QHBoxLayout *buttonRow_0 = new QHBoxLayout();
        buttonRow_0->addWidget(engage_button);
        buttonRow_0->addWidget(disengage_button);

        // Arrange Unlock & Lock buttons side by side
        QHBoxLayout *buttonRow_1 = new QHBoxLayout();
        buttonRow_1->addWidget(mp_unlock_button);
        buttonRow_1->addWidget(mp_lock_button);

        // Add buttons to left layout
        layout->addWidget(enable_button);
        layout->addWidget(find_button);
        layout->addLayout(buttonRow_0);
        layout->addLayout(buttonRow_1);
        layout->addWidget(setRotaryHome_button);
        layout->addWidget(home_button);

        // --- Right column: Cartesian Space ---
        QLabel *cartesianTitle = new QLabel("Cartesian Space", this);
        cartesianTitle->setAlignment(Qt::AlignCenter);
        rightLayout->addWidget(cartesianTitle);

        mp_table_cartesian = new QTableWidget(7, 5, this);
        mp_table_cartesian->setHorizontalHeaderLabels({"x", "y", "z", "norm", "theta"});
        mp_table_cartesian->setVerticalHeaderLabels({" Tip ", " PINNs ", "PINNs-Tip Error ", " Probe ", " Probe-Tip Error ", " CSV Target ", " CSV-Tip Error "});
        mp_table_cartesian->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_cartesian->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_cartesian->setFixedHeight(250);
        rightLayout->addWidget(mp_table_cartesian);

        // --- EKF / Force ---
        QLabel *ekfTitle = new QLabel("EKF / Force", this);
        ekfTitle->setAlignment(Qt::AlignCenter);
        rightLayout->addWidget(ekfTitle);

        mp_table_ekf = new QTableWidget(3, 4, this);
        mp_table_ekf->setHorizontalHeaderLabels({"x", "y", "z", "mag"});
        mp_table_ekf->setVerticalHeaderLabels({" Force Est ", " Pos Residual ", " Rot Residual "});
        mp_table_ekf->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_ekf->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_ekf->setFixedHeight(140);
        rightLayout->addWidget(mp_table_ekf);

        // --- Planner Status ---
        QLabel *plannerTitle = new QLabel("Planner Status", this);
        plannerTitle->setAlignment(Qt::AlignCenter);
        rightLayout->addWidget(plannerTitle);

        mp_table_planner = new QTableWidget(1, 3, this);
        mp_table_planner->setHorizontalHeaderLabels({"Status", "Success", "IK Error"});
        mp_table_planner->setVerticalHeaderLabels({" Planner  "});
        mp_table_planner->setEditTriggers(QAbstractItemView::NoEditTriggers);
        mp_table_planner->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        mp_table_planner->setFixedHeight(50);
        rightLayout->addWidget(mp_table_planner);

        // --- Freeze Robot and IGTLink reconnect controls ---
        mp_freeze_button = new QPushButton("Freeze Robot", this);
        mp_freeze_button->setStyleSheet("background-color: rgb(255, 0, 0); color: white;");
        rightLayout->addWidget(mp_freeze_button);

        mp_connect_igtl_button = new QPushButton("Connect IGTLink", this);
        rightLayout->addWidget(mp_connect_igtl_button);
        rightLayout->addStretch();

        columnsLayout->addLayout(layout, 1);
        columnsLayout->addLayout(rightLayout, 1);
        setLayout(outerLayout);

        // Connect buttons to ROS2 services

        connect(enable_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("toggleEnable", /*use_enable_service=*/true); });

        connect(find_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("findLinearHome", /*use_enable_service=*/false); });

        connect(engage_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("engageCollets", /*use_enable_service=*/false); });

        connect(disengage_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("disengageCollets", /*use_enable_service=*/false); });

        connect(mp_lock_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("lockCollets", /*use_enable_service=*/false); });

        connect(mp_unlock_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("unlockCollets", /*use_enable_service=*/false); });

        connect(mp_freeze_button, &QPushButton::clicked, this, &GuiNode::handleFreezeButtonClicked);
        connect(mp_connect_igtl_button, &QPushButton::clicked, this, &GuiNode::handleIgtlConnectClicked);

        connect(setRotaryHome_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("findRotaryHome", /*use_enable_service=*/false); });

        connect(home_button, &QPushButton::clicked, this, [this]()
                { sendConfigCommand("goHome", /*use_enable_service=*/false); });


        // connect(enable_button, &QPushButton::clicked, this, &GuiNode::toggleEnable);
        // connect(find_button, &QPushButton::clicked, this, &GuiNode::findLinearHome);
        // connect(engage_button, &QPushButton::clicked, this, &GuiNode::engageCollets);
        // connect(disengage_button, &QPushButton::clicked, this, &GuiNode::disengageCollets);
        // connect(mp_lock_button, &QPushButton::clicked, this, &GuiNode::lockCollets);
        // connect(mp_unlock_button, &QPushButton::clicked, this, &GuiNode::unlockCollets);
        // connect(setRotaryHome_button, &QPushButton::clicked, this, &GuiNode::findRotaryHome);
        // connect(home_button, &QPushButton::clicked, this, &GuiNode::goHome);

        // Signal connections for updating GUI elements based on ROS2 data
        connect(this, &GuiNode::update_enable_button_Text, enable_button, &QPushButton::setText);
        connect(this, &GuiNode::update_wrench_pos_button_Text, engage_button, &QPushButton::setText);

        connect(mp_ctrl_mode_group, QOverload<int>::of(&QButtonGroup::idClicked),
                this, &GuiNode::onCtrlModeClicked);

        connect(mp_trans_lim_group, QOverload<int>::of(&QButtonGroup::idClicked),
                this, &GuiNode::onTranLimClicked);

        for (int row = 0; row < mp_table_cartesian->rowCount(); ++row)
        {
            for (int col = 0; col < mp_table_cartesian->columnCount(); ++col)
            {
                mp_table_cartesian->setItem(row, col, new QTableWidgetItem("--"));
            }
        }

        mp_table_robot->setItem(0, 5, new QTableWidgetItem("--"));
        mp_table_robot->setItem(0, 6, new QTableWidgetItem("--"));
        mp_table_robot->setItem(0, 7, new QTableWidgetItem("--"));

        mp_table_planner->setItem(0, 0, new QTableWidgetItem("Idle"));
        mp_table_planner->setItem(0, 1, new QTableWidgetItem("--"));
        mp_table_planner->setItem(0, 2, new QTableWidgetItem("--"));

        for (int row = 0; row < mp_table_ekf->rowCount(); ++row)
        {
            for (int col = 0; col < mp_table_ekf->columnCount(); ++col)
            {
                mp_table_ekf->setItem(row, col, new QTableWidgetItem("--"));
            }
        }
    }

    //
    void keyboardRead_timerCallback()
    {
        // Reset velocity vector
        m_xdot_manual = {0.0, 0.0, 0.0, 0.0};
        

        // Check for active keys and update movement values (single atomic read)
        const uint32_t keys = m_keysPressed.load();
        if (keyDown(keys, Qt::Key_Right))
            m_xdot_manual[0] = m_xdot_command[0];
        if (keyDown(keys, Qt::Key_Left))
            m_xdot_manual[0] = -1 * m_xdot_command[0];
        if (keyDown(keys, Qt::Key_Up))
            m_xdot_manual[1] = m_xdot_command[1];
        if (keyDown(keys, Qt::Key_Down))
            m_xdot_manual[1] = -1 * m_xdot_command[1];
        if (keyDown(keys, Qt::Key_D))
            m_xdot_manual[2] = m_xdot_command[2];
        if (keyDown(keys, Qt::Key_A))
            m_xdot_manual[2] = -1 * m_xdot_command[2];
        if (keyDown(keys, Qt::Key_W))
            m_xdot_manual[3] = m_xdot_command[3];
        if (keyDown(keys, Qt::Key_S))
            m_xdot_manual[3] = -1 * m_xdot_command[3];
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

    
    //
    void updateTable_Joints(blaze::StaticVector<double, 4> m_x, blaze::StaticVector<double, 4> m_xdot, blaze::StaticVector<double, 4> m_current)
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
                                  blaze::StaticVector<bool, 4> reached,
                                  blaze::StaticVector<double, 4> cpuTemp,
                                  blaze::StaticVector<double, 4> driverTemp)
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

        mp_table_joints->setItem(8, 0, new QTableWidgetItem(QString::number(cpuTemp[0], 'f', 1)));
        mp_table_joints->setItem(8, 1, new QTableWidgetItem(QString::number(cpuTemp[1], 'f', 1)));
        mp_table_joints->setItem(8, 2, new QTableWidgetItem(QString::number(cpuTemp[2], 'f', 1)));
        mp_table_joints->setItem(8, 3, new QTableWidgetItem(QString::number(cpuTemp[3], 'f', 1)));

        mp_table_joints->setItem(9, 0, new QTableWidgetItem(QString::number(driverTemp[0], 'f', 1)));
        mp_table_joints->setItem(9, 1, new QTableWidgetItem(QString::number(driverTemp[1], 'f', 1)));
        mp_table_joints->setItem(9, 2, new QTableWidgetItem(QString::number(driverTemp[2], 'f', 1)));
        mp_table_joints->setItem(9, 3, new QTableWidgetItem(QString::number(driverTemp[3], 'f', 1)));
    }

    void updateTable_RobotStatus(bool enable, bool procedure, bool head, bool attached, int locked)
    {
        mp_table_robot->setItem(0, 0, new QTableWidgetItem(head ? "Attached" : "Detached"));
        mp_table_robot->setItem(0, 1, new QTableWidgetItem(enable ? "ON" : "OFF"));
        mp_table_robot->setItem(0, 2, new QTableWidgetItem(procedure ? "ON" : "OFF"));
        mp_table_robot->setItem(0, 3, new QTableWidgetItem(attached ? "Yes" : "No"));
        if (locked == 1)
        {
            mp_table_robot->setItem(0, 4, new QTableWidgetItem("Locked"));
        }
        else if (locked == -1)
        {
            mp_table_robot->setItem(0, 4, new QTableWidgetItem("Unlocked"));
        }
        else
        {
            mp_table_robot->setItem(0, 4, new QTableWidgetItem("Unknown"));
        }
    }

    //
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

    void updateTable_Cartesian(const std::array<double, 3> &tip)
    {
        const double norm = std::sqrt(tip[0] * tip[0] + tip[1] * tip[1] + tip[2] * tip[2]);
        const double theta = std::atan2(tip[1], tip[0]);

        mp_table_cartesian->setItem(0, 0, new QTableWidgetItem(QString::number(tip[0], 'f', 4)));
        mp_table_cartesian->setItem(0, 1, new QTableWidgetItem(QString::number(tip[1], 'f', 4)));
        mp_table_cartesian->setItem(0, 2, new QTableWidgetItem(QString::number(tip[2], 'f', 4)));
        mp_table_cartesian->setItem(0, 3, new QTableWidgetItem(QString::number(norm, 'f', 4)));
        mp_table_cartesian->setItem(0, 4, new QTableWidgetItem(QString::number(theta, 'f', 3)));
    }

    void updateTable_PlannerStatus(bool planning, bool success, double ik_error)
    {
        QTableWidgetItem *status_item = new QTableWidgetItem(planning ? "Planning" : "Idle");
        if (planning)
        {
            status_item->setForeground(QBrush(QColor(255, 140, 0)));
        }
        mp_table_planner->setItem(0, 0, status_item);

        QTableWidgetItem *success_item = new QTableWidgetItem(success ? "Yes" : "No");
        if (!success)
        {
            success_item->setForeground(QBrush(QColor(255, 0, 0)));
        }
        mp_table_planner->setItem(0, 1, success_item);

        QTableWidgetItem *ik_error_item = new QTableWidgetItem(QString::number(ik_error, 'f', 4));
        if (ik_error > 0.003)
        {
            ik_error_item->setForeground(QBrush(QColor(255, 0, 0)));
        }
        mp_table_planner->setItem(0, 2, ik_error_item);
    }

    void updateTable_EkfForce(const std::array<double, 4> &force_estimate,
                              const std::array<double, 4> &position_residual,
                              const std::array<double, 4> &orientation_residual)
    {
        for (int col = 0; col < 4; ++col)
        {
            mp_table_ekf->setItem(0, col, new QTableWidgetItem(QString::number(force_estimate[col], 'f', 4)));
            mp_table_ekf->setItem(1, col, new QTableWidgetItem(QString::number(position_residual[col], 'f', 4)));
            mp_table_ekf->setItem(2, col, new QTableWidgetItem(QString::number(orientation_residual[col], 'f', 4)));
        }
    }

    //
    void initRosInterfaces()
    {
        m_subscription_joints = create_subscription<interfaces::msg::Jointspace>("joint_space/feedback", 10, std::bind(&GuiNode::jointsConfig_timerCallback, this, std::placeholders::_1));
        m_subscription_status = create_subscription<interfaces::msg::Status>("robot_status", 10, std::bind(&GuiNode::robot_status_callback, this, std::placeholders::_1));
        m_subscription_interface = create_subscription<interfaces::msg::Interface>("manual_interface", 10, std::bind(&GuiNode::manual_interface_callback, this, std::placeholders::_1));
        m_subscription_taskspace = create_subscription<interfaces::msg::Taskspace>("/task_space/feedback/base_tool", 10, std::bind(&GuiNode::taskspace_callback, this, std::placeholders::_1));
        m_subscription_force_estimate = create_subscription<interfaces::msg::Force>("task_space/force_estimate", 10, std::bind(&GuiNode::force_estimate_callback, this, std::placeholders::_1));
        m_subscription_ekf_residual = create_subscription<interfaces::msg::EKFResidual>("EKF/residual_error", 10, std::bind(&GuiNode::ekf_residual_callback, this, std::placeholders::_1));
        m_subscription_igtl_connected = create_subscription<std_msgs::msg::String>("igtl_bridge/connected", 10, std::bind(&GuiNode::igtl_connected_callback, this, std::placeholders::_1));

        m_publisher_manual_vel = create_publisher<interfaces::msg::Jointspace>("joint_space/manual_vel", 10);

        m_cbGroup1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_read_key_timer = create_wall_timer(20ms, std::bind(&GuiNode::keyboardRead_timerCallback, this), m_cbGroup1);
        m_connection_check_timer = create_wall_timer(500ms, std::bind(&GuiNode::connectionStatus_timerCallback, this));

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

        m_freeze_robot_client = create_client<std_srvs::srv::SetBool>("freeze_robot");
        while (!m_freeze_robot_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for freeze_robot service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "freeze_robot service not available, waiting ...");
        }

        m_igtl_connect_client = create_client<std_srvs::srv::SetBool>("igtl_bridge/connect");
    }

    //
    void jointsConfig_timerCallback(const interfaces::msg::Jointspace::SharedPtr msg)
    {
        // Copy data out of the message
        blaze::StaticVector<double, 4> x, xdot, current;
        for (int i = 0; i < 4; i++)
        {
            x[i]      = msg->position[i];
            xdot[i]   = msg->velocity[i];
            current[i]= msg->current[i];
        }

        // Schedule GUI update on Qt thread
        QMetaObject::invokeMethod(
            this,
            [this, x, xdot, current]() {
                updateTable_Joints(x, xdot, current);
            },
            Qt::QueuedConnection);
    }

    void robot_status_callback(const interfaces::msg::Status::SharedPtr msg)
{
    // Update internal state (these are just POD writes; ok-ish)
    m_enabled        = msg->enable[0] * msg->enable[1] * msg->enable[2] * msg->enable[3];
    m_reached        = msg->reached[0] * msg->reached[1] * msg->reached[2] * msg->reached[3];
    m_encoder        = msg->encoder[0] * msg->encoder[1] * msg->encoder[2] * msg->encoder[3];
    m_procedure      = msg->procedure;
    m_ready_to_engage= msg->ready_to_engage;
    m_engaged        = msg->engaged;
    m_locked         = msg->locked;
    m_head_attached  = msg->head_attached;

    m_trans_limit    = msg->trans_limit_en;
    m_mode           = static_cast<CtrlMode>(msg->control_mode);

    blaze::StaticVector<double, 4> minPosLimit, maxPosLimit;
    blaze::StaticVector<bool,   4> enabledJoints, encoderJoints, reachedJoints;
    blaze::StaticVector<double, 4> cpuTemp, windingTemp;

    for (int i = 0; i < 4; i++)
    {
        minPosLimit[i]   = msg->min_pos_limit[i];
        maxPosLimit[i]   = msg->max_pos_limit[i];
        enabledJoints[i] = msg->enable[i];
        encoderJoints[i] = msg->encoder[i];
        reachedJoints[i] = msg->reached[i];
        cpuTemp[i] = msg->cpu_temp[i];
        windingTemp[i] = msg->winding_temp[i];
    }

    // Capture everything needed for GUI in local copies
    bool   enabled       = m_enabled;
    bool   procedure     = m_procedure;
    bool   head_attached = m_head_attached;
    bool   engaged       = m_engaged;
    int    locked        = m_locked;
    bool   trans_limit   = m_trans_limit;
    CtrlMode mode        = m_mode;

    QMetaObject::invokeMethod(
        this,
        [this,
         enabled, procedure, head_attached, engaged, locked,
         mode, trans_limit,
         minPosLimit, maxPosLimit,
         enabledJoints, encoderJoints, reachedJoints, cpuTemp, windingTemp]() mutable
        {
            // --- Buttons ---
            mp_unlock_button->setEnabled(engaged);
            mp_lock_button->setEnabled(engaged);

            // --- Joints table ---
            updateTable_JointsStatus(enabledJoints,
                                     minPosLimit,
                                     maxPosLimit,
                                     encoderJoints,
                                     reachedJoints,
                                     cpuTemp,
                                     windingTemp);

            // --- Robot status table ---
            updateTable_RobotStatus(enabled, procedure, head_attached, engaged, locked);

            // --- Enable button text ---
            emit update_enable_button_Text(enabled ? "Disable" : "Enable");

            // --- Mode radio buttons / Trans limit radio buttons ---
            if (mode != m_mode_prev)
            {
                mp_ctrl_mode_group->setExclusive(false);
                mp_optionCtrlmMode0->setChecked(false);
                mp_optionCtrlmMode1->setChecked(false);
                mp_optionCtrlmMode2->setChecked(false);
                mp_optionCtrlmMode3->setChecked(false);

                switch (mode)
                {
                case CtrlMode::Config:
                    mp_optionCtrlmMode0->setChecked(true);
                    mp_bulletTranLimGroup->setEnabled(false);
                    break;
                case CtrlMode::Manual:
                    mp_optionCtrlmMode1->setChecked(true);
                    mp_bulletTranLimGroup->setEnabled(true);
                    break;
                case CtrlMode::Velocity:
                    mp_optionCtrlmMode2->setChecked(true);
                    mp_bulletTranLimGroup->setEnabled(false);
                    break;
                case CtrlMode::Position:
                    mp_optionCtrlmMode3->setChecked(true);
                    mp_bulletTranLimGroup->setEnabled(false);
                    break;
                }
                mp_ctrl_mode_group->setExclusive(true);
                m_mode_prev = mode;
            }

            if (trans_limit != m_trans_limit_prev)
            {
                mp_trans_lim_group->setExclusive(false);
                mp_optionTranLimOn->setChecked(false);
                mp_optionTranLimOff->setChecked(false);

                if (trans_limit)
                    mp_optionTranLimOn->setChecked(true);
                else
                    mp_optionTranLimOff->setChecked(true);

                mp_trans_lim_group->setExclusive(true);
                m_trans_limit_prev = trans_limit;
            }
        },
        Qt::QueuedConnection);
}


    // // update robot status information
    // void robot_status_callback(const interfaces::msg::Status::SharedPtr msg)
    // {
    //     m_enabled = msg->enable[0] * msg->enable[1] * msg->enable[2] * msg->enable[3];
    //     m_reached = msg->reached[0] * msg->reached[1] * msg->reached[2] * msg->reached[3];
    //     m_encoder = msg->encoder[0] * msg->encoder[1] * msg->encoder[2] * msg->encoder[3];
    //     m_procedure = msg->procedure;
    //     m_ready_to_engage = msg->ready_to_engage;
    //     m_engaged = msg->engaged;
    //     m_locked = msg->locked;
    //     m_head_attached = msg->head_attached;

    //     m_trans_limit = msg->trans_limit_en;
    //     m_mode = static_cast<CtrlMode>(msg->control_mode);

    //     for (int i = 0; i < 4; i++)
    //     {
    //         m_minCurrentPosLimit[i] = msg->min_pos_limit[i];
    //         m_maxCurrentPosLimit[i] = msg->max_pos_limit[i];
    //         m_enabledJoints[i] = msg->enable[i];
    //         m_encoderJoints[i] = msg->encoder[i];
    //         m_reachedJoints[i] = msg->reached[i];
    //     }

    //     mp_unlock_button->setEnabled(m_engaged); // Enable/Disabling the lock button
    //     mp_lock_button->setEnabled(m_engaged);   // Enable/Disabling the unlock button

    //     if (m_mode_prev != m_mode)
    //     {
    //         QMetaObject::invokeMethod(this, [this]()
    //                                   {
    //             mp_ctrl_mode_group->setExclusive(false);
    //             mp_optionCtrlmMode0->setChecked(false);
    //             mp_optionCtrlmMode1->setChecked(false);
    //             mp_optionCtrlmMode2->setChecked(false);
    //             mp_optionCtrlmMode3->setChecked(false);
    //             switch (m_mode) {
    //               case CtrlMode::Config:   mp_optionCtrlmMode0->setChecked(true); mp_bulletTranLimGroup->setEnabled(false); break;
    //               case CtrlMode::Manual:   mp_optionCtrlmMode1->setChecked(true); mp_bulletTranLimGroup->setEnabled(true); break;
    //               case CtrlMode::Velocity: mp_optionCtrlmMode2->setChecked(true); mp_bulletTranLimGroup->setEnabled(false); break;
    //               case CtrlMode::Position: mp_optionCtrlmMode3->setChecked(true); mp_bulletTranLimGroup->setEnabled(false); break;
    //             }
    //             mp_ctrl_mode_group->setExclusive(true); }, Qt::QueuedConnection);
    //         m_mode_prev = m_mode;
    //     }
    //     if (m_trans_limit != m_trans_limit_prev)
    //     {
    //         QMetaObject::invokeMethod(this, [this]()
    //                                   {
    //             mp_trans_lim_group->setExclusive(false);
    //             mp_optionTranLimOn->setChecked(false);
    //             mp_optionTranLimOff->setChecked(false);
    //             if (m_trans_limit)
    //                 mp_optionTranLimOn->setChecked(true); // ON
    //             else
    //                 mp_optionTranLimOff->setChecked(true); // OFF
    //             mp_trans_lim_group->setExclusive(true); }, Qt::QueuedConnection); // Ensure it's run on the Qt GUI thread
    //         m_trans_limit_prev = m_trans_limit;
    //     }

    //     updateTable_JointsStatus(m_enabledJoints, m_minCurrentPosLimit, m_maxCurrentPosLimit, m_encoderJoints, m_reachedJoints);
    //     updateTable_RobotStatus(m_enabled, m_procedure, m_head_attached, m_engaged, m_locked);

    //     emit update_enable_button_Text(m_enabled ? "Disable" : "Enable");
    //     // emit update_wrench_pos_button_Text(m_ready_to_engage ? "Engage Collets" : "Prepare to Engage Collets");
    // }

    //
    void manual_interface_callback(const interfaces::msg::Interface::SharedPtr msg)
    {
        std::array<bool, 7> keys;
        for (int i = 0; i < 7; i++)
        {
            keys[i] = msg->interface_key[i];
            m_interface_key[i] = keys[i];  // keep internal copy if you want
        }

        QMetaObject::invokeMethod(
            this,
            [this, keys]() {
                updateTable_RobotStatus(keys);
            },
            Qt::QueuedConnection);
    }

    void taskspace_callback(const interfaces::msg::Taskspace::SharedPtr msg)
    {
        m_emtracker_last_ns.store(this->now().nanoseconds(), std::memory_order_relaxed);
        std::array<double, 3> tip = {msg->p[0], msg->p[1], msg->p[2]};

        QMetaObject::invokeMethod(
            this,
            [this, tip]() {
                updateTable_Cartesian(tip);
            },
            Qt::QueuedConnection);
    }

    void force_estimate_callback(const interfaces::msg::Force::SharedPtr msg)
    {
        m_force_estimate = {msg->x, msg->y, msg->z, msg->magnitude};

        QMetaObject::invokeMethod(
            this,
            [this]() {
                updateTable_EkfForce(m_force_estimate, m_position_residual, m_orientation_residual);
            },
            Qt::QueuedConnection);
    }

    void ekf_residual_callback(const interfaces::msg::EKFResidual::SharedPtr msg)
    {
        m_position_residual = {msg->x, msg->y, msg->z, msg->pos_mag};
        m_orientation_residual = {msg->theta_x, msg->theta_y, msg->theta_z, msg->orientation_mag};

        QMetaObject::invokeMethod(
            this,
            [this]() {
                updateTable_EkfForce(m_force_estimate, m_position_residual, m_orientation_residual);
            },
            Qt::QueuedConnection);
    }

    void igtl_connected_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        m_igtl_connected.store(msg->data == "connected", std::memory_order_relaxed);
    }

    void connectionStatus_timerCallback()
    {
        const int64_t last_ns = m_emtracker_last_ns.load(std::memory_order_relaxed);
        const bool emtracker_alive = (last_ns > 0) &&
            ((this->now().nanoseconds() - last_ns) < 1'000'000'000LL); // 1 s threshold
        const bool igtl_connected = m_igtl_connected.load(std::memory_order_relaxed);

        QMetaObject::invokeMethod(
            this,
            [this, emtracker_alive, igtl_connected]()
            {
                auto *emt_item = new QTableWidgetItem(emtracker_alive ? "Active" : "Dead");
                emt_item->setForeground(QBrush(emtracker_alive ? QColor(0, 180, 0) : QColor(200, 0, 0)));
                mp_table_robot->setItem(0, 6, emt_item);

                auto *igtl_item = new QTableWidgetItem(igtl_connected ? "Connected" : "Disconnected");
                igtl_item->setForeground(QBrush(igtl_connected ? QColor(0, 180, 0) : QColor(200, 0, 0)));
                mp_table_robot->setItem(0, 7, igtl_item);

                mp_connect_igtl_button->setText(igtl_connected ? "Disconnect IGTLink" : "Connect IGTLink");
            },
            Qt::QueuedConnection);
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
    std::atomic<uint32_t> m_keysPressed{0};
    // QLabel *mp_label_1, *mp_label_2, *mp_label_3, *mp_label_4;
    QGroupBox *mp_bulletCtrlmModeGroup;
    QButtonGroup *mp_ctrl_mode_group, *mp_trans_lim_group;
    QRadioButton *mp_optionCtrlmMode0, *mp_optionCtrlmMode1, *mp_optionCtrlmMode2, *mp_optionCtrlmMode3;
    QGroupBox *mp_bulletTranLimGroup;
    QVBoxLayout *mp_bulletLayoutTranLim;
    QRadioButton *mp_optionTranLimOff, *mp_optionTranLimOn;
    QPushButton *mp_unlock_button, *mp_lock_button, *mp_freeze_button, *mp_connect_igtl_button;
    QTableWidget *mp_table_joints, *mp_table_robot, *mp_table_interface, *mp_table_cartesian, *mp_table_planner, *mp_table_ekf;

    std::atomic<CtrlMode> m_mode;
    CtrlMode m_mode_prev; // controller mode (manual, velocity, position); prev only touched by the key timer

    blaze::StaticVector<double, 4> m_com_vel; // Velocity vector

    blaze::StaticVector<double, 4UL> m_x, m_x_des, m_x_error, m_x_abs;                                // in SI units
    blaze::StaticVector<double, 4UL> m_xdot, m_xdot_manual, m_xdot_des, m_xdot_error, m_xdot_forward; // in SI units
    blaze::StaticVector<double, 4UL> m_x_error_int;                                                   // in SI units
    blaze::StaticVector<double, 4UL> m_current;

    blaze::StaticVector<double, 4UL> m_xdot_command;
    
    blaze::StaticVector<double, 4UL> m_minCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);
    blaze::StaticVector<double, 4UL> m_maxCurrentPosLimit = blaze::StaticVector<double, 4UL>(0.0);

    std::array<bool, 7> m_interface_key = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 4> m_force_estimate = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> m_position_residual = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> m_orientation_residual = {0.0, 0.0, 0.0, 0.0};

    bool m_enabled, m_procedure, m_reached, m_encoder = false;                                     // Tracks button state
    bool m_engaged, m_ready_to_engage, m_head_attached = false; // Tracks button state
    bool m_robot_frozen = false;
    int m_locked;

    blaze::StaticVector<bool, 4UL> m_enabledJoints, m_encoderJoints, m_reachedJoints;

    rclcpp::CallbackGroup::SharedPtr m_cbGroup1;

    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subscription_status;
    rclcpp::Subscription<interfaces::msg::Interface>::SharedPtr m_subscription_interface;
    rclcpp::Subscription<interfaces::msg::Taskspace>::SharedPtr m_subscription_taskspace;
    rclcpp::Subscription<interfaces::msg::Force>::SharedPtr m_subscription_force_estimate;
    rclcpp::Subscription<interfaces::msg::EKFResidual>::SharedPtr m_subscription_ekf_residual;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription_igtl_connected;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_config_client;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_robot_enable_client;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_freeze_robot_client;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_igtl_connect_client;
    rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher_manual_vel;
    rclcpp::TimerBase::SharedPtr m_read_key_timer;
    rclcpp::TimerBase::SharedPtr m_connection_check_timer;

    std::atomic<int64_t> m_emtracker_last_ns{0};
    std::atomic<bool> m_igtl_connected{false};
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

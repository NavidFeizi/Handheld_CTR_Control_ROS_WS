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

#include <iostream> // Required for std::cout
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/status.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/config.hpp"
#include <thread>
#include <vector>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

// constexpr char Key_Right 16777236;
// constexpr char Key_Left 16777234;
// constexpr char Key_Right 16777235;
// constexpr char Key_Right 16777237;

class GuiNode : public QWidget, public rclcpp::Node
{        // Fix: QObject first
Q_OBJECT // Needed for Qt signals/slots
    public : explicit GuiNode(QWidget *parent = nullptr)
    : QWidget(parent), rclcpp::Node("qt_gui_node")
    {
        setWindowTitle("Qt ROS2 Multi-Key Handler");
        setGeometry(100, 100, 400, 300);

        setFocusPolicy(Qt::StrongFocus);  // Ensure QT captures arrow keys

        // Create Layout
        QVBoxLayout *layout = new QVBoxLayout(this);

        // Create Label
        label_ = new QLabel("Waiting for data...", this);
        label_->setAlignment(Qt::AlignCenter);
        layout->addWidget(label_);

        // Create Buttons
        QPushButton *enable_button = new QPushButton("Disable", this);
        QPushButton *find_button = new QPushButton("Find", this);
        QPushButton *wrench_pos_button = new QPushButton("Prepare to Position Wrench", this);
        QPushButton *unlock_button = new QPushButton("Unlock", this);
        QPushButton *lock_button = new QPushButton("Lock", this);

        // Arrange Unlock & Lock buttons side by side
        QHBoxLayout *buttonRow = new QHBoxLayout();
        buttonRow->addWidget(unlock_button);
        buttonRow->addWidget(lock_button);

        // Add buttons to layout
        layout->addWidget(enable_button);
        layout->addWidget(find_button);
        layout->addWidget(wrench_pos_button);
        layout->addLayout(buttonRow);

        setLayout(layout); // Apply layout to the window

        // Connect buttons to ROS2 services
        connect(enable_button, &QPushButton::clicked, this, &GuiNode::send_enable_request);
        connect(find_button, &QPushButton::clicked, this, &GuiNode::send_find_request);
        connect(wrench_pos_button, &QPushButton::clicked, this, &GuiNode::send_pos_wrench_request);
        connect(lock_button, &QPushButton::clicked, this, &GuiNode::send_lock_request);
        connect(unlock_button, &QPushButton::clicked, this, &GuiNode::send_unlock_request);

        // Signal connections for updating GUI elements based on ROS2 data
        connect(this, &GuiNode::update_enable_button_Text, enable_button, &QPushButton::setText);
        connect(this, &GuiNode::update_wrench_pos_button_Text, wrench_pos_button, &QPushButton::setText);
        connect(this, &GuiNode::dataReceived, this, &GuiNode::updateLabel);

        // ROS2 Setup
        this->rosInterfaces();
    }

    void rosInterfaces()
    {
        m_subscription_joints = this->create_subscription<interfaces::msg::Jointspace>("/joint_space/feedback", 10, std::bind(&GuiNode::joints_feedback_callback, this, std::placeholders::_1));
        m_subscription_status = this->create_subscription<interfaces::msg::Status>("robot_status", 10, std::bind(&GuiNode::robot_status_callback, this, std::placeholders::_1));

        m_record_client = this->create_client<interfaces::srv::Config>("config");
        while (!m_record_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Record service not available, waiting again...");
        }

        m_callback_group1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sample_time = std::chrono::milliseconds(50);  // Read keys every 50ms
        m_read_key_timer = this->create_wall_timer(sample_time, std::bind(&GuiNode::key_read_callback, this), m_callback_group1);
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        // qDebug() << "Key Pressed:" << event->key();
        keysPressed.insert(event->key()); // Store pressed key
    }

    void keyReleaseEvent(QKeyEvent *event) override
    {
        keysPressed.remove(event->key()); // Remove key when released
        // qDebug() << "Key Released:" << event->key();
    }

signals:
    void dataReceived(QString motor1, QString motor2, QString motor3, QString motor4);

    void update_enable_button_Text(QString text); // Signal to update button text

    void update_wrench_pos_button_Text(QString text); // Signal to update button text

    void update_lock_button_Text(QString text); // Signal to update button text

public slots:
    void updateLabel(QString motor1, QString motor2, QString motor3, QString motor4)
    {
        QString text = QString("inr rot: %1\n inr trn: %2\n mdl rot: %3\n mdl trn: %4")
                           .arg(motor1)
                           .arg(motor2)
                           .arg(motor3)
                           .arg(motor4);
        label_->setText(text);
    }

    //
    void send_enable_request()
    {
        std::string message = is_enabled_ ? "disable" : "enable"; // Toggle command

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    //
    void send_find_request()
    {
        std::string message = "find"; // Toggle command

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    //
    void send_pos_wrench_request()
    {
        std::string message;

        if (!m_ready_to_proceed)
        {
            message = "prepare_position_wrench";
        }
        else if (m_ready_to_proceed)
        {
            message = "position_wrench";
        }

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    void send_lock_request()
    {
        std::string message;
        message = "lock";

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    void send_unlock_request()
    {
        std::string message;
        message = "unlock";

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&GuiNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

private:
    void key_read_callback()
    {
        // Reset velocity vector
        m_com_vel = {0.0, 0.0, 0.0, 0.0};

        // Check for active keys and update movement values
        if (keysPressed.contains(Qt::Key_Right))
            m_com_vel[0] = 2.0;
        if (keysPressed.contains(Qt::Key_Left))
            m_com_vel[0] = -2.0;
        if (keysPressed.contains(Qt::Key_Up))
            m_com_vel[1] = 1.0;
        if (keysPressed.contains(Qt::Key_Down))
            m_com_vel[1] = -1.0;
        if (keysPressed.contains(Qt::Key_D))
            m_com_vel[2] = 2.0;
        if (keysPressed.contains(Qt::Key_A))
            m_com_vel[2] = -2.0;
        if (keysPressed.contains(Qt::Key_W))
            m_com_vel[3] = 1.0;
        if (keysPressed.contains(Qt::Key_S))
            m_com_vel[3] = -1.0;

        // Send updated movement command
        std::cout << "vel: " << m_com_vel[0] << " ," << m_com_vel[1] << " ," << m_com_vel[2] << " ," << m_com_vel[3] << std::endl;
    }

    void joints_feedback_callback(const interfaces::msg::Jointspace::SharedPtr msg)
    {
        emit dataReceived(QString::number(msg->current[0], 'f', 1),
                          QString::number(msg->current[1], 'f', 1),
                          QString::number(msg->current[2], 'f', 1),
                          QString::number(msg->current[3], 'f', 1));
    }

    void robot_status_callback(const interfaces::msg::Status::SharedPtr msg)
    {
        is_enabled_ = msg->enable[0] * msg->enable[1] * msg->enable[2] * msg->enable[3];
        m_ready_to_proceed = msg->proceed_ready;
        m_ready_to_unlock = msg->unlock_ready;
        m_unlocked = msg->unlocked;
        m_locked = msg->locked;

        emit update_enable_button_Text(is_enabled_ ? "Disable" : "Enable");

        emit update_wrench_pos_button_Text(m_ready_to_proceed ? "Position Wrench" : "Prepare to Position Wrench");

        emit update_lock_button_Text(m_unlocked ? "lock" : "unlock");
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

    QSet<int> keysPressed;                    // Tracks currently pressed keys
    blaze::StaticVector<double, 4> m_com_vel; // Velocity vector
    QLabel *label_;
    bool m_flag_manual = false;

    bool is_enabled_ = false;                                                 // Tracks button state
    bool m_ready_to_proceed, m_ready_to_unlock, m_unlocked, m_locked = false; // Tracks button state

    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subscription_status;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_record_client;
    rclcpp::CallbackGroup::SharedPtr m_callback_group1;
    rclcpp::TimerBase::SharedPtr m_read_key_timer;
};

#include "qt_node.moc"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<GuiNode>();
    node->show();  // Display the GUI
    node->setFocus();  // Force focus on startup

    std::thread ros_spin_thread([&]()
                                {
        rclcpp::spin(node);
        rclcpp::shutdown(); });

    int result = app.exec();
    ros_spin_thread.join();
    return result;
}

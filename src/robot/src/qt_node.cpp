#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <QObject>
#include <QString>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/status.hpp"
#include "interfaces/msg/jointspace.hpp"
#include "interfaces/srv/config.hpp"
#include <thread>

class MyNode : public QObject, public rclcpp::Node
{        // Fix: QObject first
Q_OBJECT // Needed for Qt signals/slots

    public : MyNode(QLabel *label) : rclcpp::Node("qt_gui_node"), label_(label)
    {
        RCLCPP_INFO(this->get_logger(), "Qt GUI Node Started");

        m_subscription_joints = this->create_subscription<interfaces::msg::Jointspace>(
            "/joint_space/feedback", 10,
            std::bind(&MyNode::joints_feedback_callback, this, std::placeholders::_1));

        m_subscription_status = this->create_subscription<interfaces::msg::Status>(
            "robot_status", 10,
            std::bind(&MyNode::robot_status_callback, this, std::placeholders::_1));

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
        auto response_received_callback = std::bind(&MyNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    //
    void send_find_request()
    {
        std::string message = "find"; // Toggle command

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&MyNode::handle_service_response, this, std::placeholders::_1);
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
        auto response_received_callback = std::bind(&MyNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    void send_lock_request()
    {
        std::string message;
        message = "lock"; 

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&MyNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

    void send_unlock_request()
    {
        std::string message;
        message = "unlock"; 

        auto request = std::make_shared<interfaces::srv::Config::Request>();
        request->command = message;

        using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
        auto response_received_callback = std::bind(&MyNode::handle_service_response, this, std::placeholders::_1);
        auto future_result = m_record_client->async_send_request(request, response_received_callback);
    }

private:
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

    bool is_enabled_ = false;       // Tracks button state
    bool m_ready_to_proceed, m_ready_to_unlock, m_unlocked, m_locked = false; // Tracks button state

    rclcpp::Subscription<interfaces::msg::Jointspace>::SharedPtr m_subscription_joints;
    rclcpp::Subscription<interfaces::msg::Status>::SharedPtr m_subscription_status;
    rclcpp::Client<interfaces::srv::Config>::SharedPtr m_record_client;

    QLabel *label_;
};

#include "qt_node.moc"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    QWidget window;
    QVBoxLayout layout;

    window.setWindowTitle("ROS2 Qt GUI");
    window.setGeometry(100, 100, 400, 300);

    QLabel label("Waiting for data...", &window);
    label.setAlignment(Qt::AlignCenter);

    QPushButton enable_button("Disable", &window);
    QPushButton find_button("Find", &window);
    QPushButton wrench_pos_button("Prepare to Position Wrench", &window);
    QPushButton unlock_button("Unlock", &window);
    QPushButton lock_button("Lock", &window);

    // unlock_button.setFixedSize(50, 25); // Adjust width & height
    // lock_button.setFixedSize(50, 25);

    QHBoxLayout buttonRow;
    buttonRow.addWidget(&unlock_button);
    buttonRow.addWidget(&lock_button);

    layout.addWidget(&label);
    layout.addWidget(&enable_button);
    layout.addWidget(&find_button);
    layout.addWidget(&wrench_pos_button);
    layout.addLayout(&buttonRow);  // Add the horizontal layout


    // QHBoxLayout buttonRow;
    // buttonRow.addWidget(&unlock_button);
    // buttonRow.addWidget(&lock_button);

    // layout.addWidget(&unlock_button);
    // layout.addWidget(&lock_button);

    window.setLayout(&layout);
    window.show();

    auto node = std::make_shared<MyNode>(&label);
    QObject::connect(&enable_button, &QPushButton::clicked, node.get(), &MyNode::send_enable_request);
    QObject::connect(node.get(), &MyNode::update_enable_button_Text, &enable_button, &QPushButton::setText);

    QObject::connect(&find_button, &QPushButton::clicked, node.get(), &MyNode::send_find_request);

    QObject::connect(&wrench_pos_button, &QPushButton::clicked, node.get(), &MyNode::send_pos_wrench_request);
    QObject::connect(node.get(), &MyNode::update_wrench_pos_button_Text, &wrench_pos_button, &QPushButton::setText);

    QObject::connect(&lock_button, &QPushButton::clicked, node.get(), &MyNode::send_lock_request);

    QObject::connect(&unlock_button, &QPushButton::clicked, node.get(), &MyNode::send_unlock_request);


    QObject::connect(node.get(), &MyNode::dataReceived, node.get(), &MyNode::updateLabel);

    std::thread ros_spin_thread([&]()
                                {
        rclcpp::spin(node);
        rclcpp::shutdown(); });

    int result = app.exec();
    ros_spin_thread.join();
    return result;
}

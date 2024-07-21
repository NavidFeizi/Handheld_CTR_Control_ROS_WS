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
#include "interfaces/srv/homing.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// #include "keyboardInput.hpp"
#include <SDL2/SDL.h>

using namespace std::chrono_literals;

class KeyboardNode : public rclcpp::Node
{
public:
  KeyboardNode() : Node("keyboard"), count_(0)
  {
    steps = {1.0 * M_PI / 180, 0.5E-3, 1.0 * M_PI / 180, 0.5E-3};
    target = {0.0, 0.0, 0.0, 0.0};
    counter = 0;
    m_sample_time = 1e-3;
    t0_ = 0.0;
    m_t = 0.0;

    running = true;
    shiftQPressed = false;

    // KeyboardNode::declare_parameters();
    KeyboardNode::setup_ros_interfaces();
    KeyboardNode::initialize_keyboard();

    rclcpp::Time now = this->get_clock()->now();
    t0_ = static_cast<double>(now.nanoseconds()) / 1E9;
    m_t = 0.0;

    RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
  }

  ~KeyboardNode()
  {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    // delete counter;
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
    m_callback_group_event = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // m_publisher = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation_target", 10);     // to actuate the task space jarget generator
    m_publisher = this->create_publisher<interfaces::msg::Jointspace>("JointsActuation", 10); // to directly actuate the robot

    m_record_client = this->create_client<interfaces::srv::Homing>("homing");
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
    m_timer = this->create_wall_timer(dt, std::bind(&KeyboardNode::target_joints_callback, this), m_callback_group_pub);

    m_timer_event = this->create_wall_timer(1ms, std::bind(&KeyboardNode::event_callback, this), m_callback_group_event);
  }

  //
  void send_homing_request(std::string message)
  {
    auto request = std::make_shared<interfaces::srv::Homing::Request>();
    request->command = message; // Set the desired duration

    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Homing>::SharedFuture;
    auto response_received_callback = std::bind(&KeyboardNode::handle_homing_response, this, std::placeholders::_1);
    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

  //
  void send_target(const blaze::StaticVector<double, 4UL> targ)
  {
    auto request = std::make_shared<interfaces::srv::Homing::Request>();
    request->command = "move";    // Set the desired duration
    request->target[0] = targ[0]; // Set the desired duration
    request->target[1] = targ[1]; // Set the desired duration
    request->target[2] = targ[2]; // Set the desired duration
    request->target[3] = targ[3]; // Set the desired duration

    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Homing>::SharedFuture;
    auto response_received_callback = std::bind(&KeyboardNode::handle_homing_response, this, std::placeholders::_1);
    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

  //
  void handle_homing_response(const rclcpp::Client<interfaces::srv::Homing>::SharedFuture future)
  {
    // Get the result of the future object
    auto response = future.get();

    if (response->success)
    {
      if (response->message == "Manual mode activated" or response->message == "Home is set")
      {
        target[0] = response->position[0];
        target[1] = response->position[1];
        target[2] = response->position[2];
        target[3] = response->position[3];
        RCLCPP_INFO(this->get_logger(), "Current positino is: %0.4f  %0.4f  %0.4f  %0.4f",
                    response->position[0], response->position[1], response->position[2], response->position[3]);
      }

      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", response->message.c_str());
    }
  }

  // Run the experiment with random inputs signals multiple times for dataset generation
  void initialize_keyboard()
  {
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "SDL_Init Error: %s", SDL_GetError());
      throw std::runtime_error("Failed to initialize SDL");
    }

    window = SDL_CreateWindow("Keyboard Input", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN);
    if (!window)
    {
      SDL_Quit();
      throw std::runtime_error("Failed to create SDL window");
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer)
    {
      SDL_DestroyWindow(window);
      SDL_Quit();
      throw std::runtime_error("Failed to create SDL renderer");
    }

    lastPressTime = std::chrono::high_resolution_clock::now();
    std::signal(SIGINT, [](int sig)
                { std::cout << "Received SIGINT, exiting..." << std::endl; std::exit(0); });
  }

  //
  void handleSignal(int signal)
  {
    if (signal == SIGINT)
    {
      std::cout << "Received SIGINT, exiting..." << std::endl;
      SDL_Quit();
      std::exit(0);
    }
  }

  //
  void event_callback()
  {
    // Clear the screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE); // Set background color to black
    SDL_RenderClear(renderer);

    // Coordinates for the triangle
    int centerX = 320; // Half of window width
    int centerY = 240; // Half of window height
    int size = 100;    // Size of the triangle

    int x1 = centerX, y1 = centerY - size;
    int x2 = centerX + size, y2 = centerY + size;
    int x3 = centerX - size, y3 = centerY + size;

    drawTriangle(renderer, x1, y1, x2, y2, x3, y3);

    SDL_RenderPresent(renderer);

    while (SDL_PollEvent(&event))
    {
      if (event.type == SDL_WINDOWEVENT)
      {
        if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED)
        {
          isFocused = true;
          // std::cout << "Window focused" << std::endl;
        }
        else if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)
        {
          isFocused = false;
          // std::cout << "Window unfocused" << std::endl;
        }
        else if (event.window.event == SDL_WINDOWEVENT_CLOSE)
        {
          running = false;
        }
      }
      else if (event.type == SDL_KEYDOWN && isFocused)
      {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPressTime);

        if (timeDiff.count() >= 50)
        {
          SDL_Keycode key = event.key.keysym.sym;
          // std::cout << "event: " << key << std::endl;
          switch (key)
          {
          case SDLK_RIGHT:
            target[0] += steps[0];
            KeyboardNode::send_target(target);
            break;
          case SDLK_LEFT:
            target[0] -= steps[0];
            KeyboardNode::send_target(target);
            break;
          case SDLK_UP:
            target[1] += steps[1];
            KeyboardNode::send_target(target);
            break;
          case SDLK_DOWN:
            target[1] -= steps[1];
            KeyboardNode::send_target(target);
            break;
          case SDLK_d:
            target[2] += steps[2];
            KeyboardNode::send_target(target);
            break;
          case SDLK_a:
            target[2] -= steps[2];
            KeyboardNode::send_target(target);
            break;
          case SDLK_w:
            target[3] += steps[3];
            KeyboardNode::send_target(target);
            break;
          case SDLK_s:
            target[3] -= steps[3];
            KeyboardNode::send_target(target);
            break;
          case SDLK_0:
            target = {0.0, 0.0, 0.0, 0.0};
            KeyboardNode::send_target(target);
            break;
          case SDLK_SPACE:
            if (m_flag_manual)
            {
              m_flag_manual = false;
              KeyboardNode::send_homing_request("OFF");
            }
            else
            {
              m_flag_manual = true;
              KeyboardNode::send_homing_request("ON");
            }
            break;
          case SDLK_h:
            if (m_flag_manual)
            {
              KeyboardNode::send_homing_request("set_home");
            }
            break;
          default:
            break;
          }

          lastPressTime = std::chrono::high_resolution_clock::now();
        }
      }
      else if (event.type == SDL_QUIT)
      {
        running = false;
      }
    }
  }

  // Timer callback function to publish the target joints config
  void target_joints_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    m_msg.position[0UL] = target[0];
    m_msg.position[1UL] = target[1];
    m_msg.position[2UL] = target[2];
    m_msg.position[3UL] = target[3];

    // std::cout << "new target: ";
    // std::cout << std::fixed << std::setprecision(4);
    // std::cout << m_msg.position[0UL] << std::endl;
  }

  void drawTriangle(SDL_Renderer *renderer, int x1, int y1, int x2, int y2, int x3, int y3)
  {
    // Set the color for drawing the triangle
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE); // White

    // Draw lines between the triangle's points
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    SDL_RenderDrawLine(renderer, x2, y2, x3, y3);
    SDL_RenderDrawLine(renderer, x3, y3, x1, y1);
  }

  // Wait in milliseconds
  void wait(int milliseconds)
  {
    rclcpp::Rate rate(1000.0 / milliseconds);
    rate.sleep();
  }

  const std::vector<double> m_steps = {1.0 * M_PI / 180, 0.5E-3, 1.0 * M_PI / 180, 0.5E-3};
  // std::vector<double> target(4, 0);

  // member variables
  size_t count_;
  double m_sample_time = 1e-3; //[s]
  double t0_, m_t, t0 = 0;

  double m_expt_time = 30.0;

  interfaces::msg::Jointspace m_msg;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::TimerBase::SharedPtr m_timer_event;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_event;
  rclcpp::Client<interfaces::srv::Homing>::SharedPtr m_record_client;

  std::atomic<bool> m_is_experiment_running;
  std::thread m_experiment_thread;

  bool m_flag_manual = false;

  SDL_Event event;
  bool isFocused = true;

  // std::vector<double> &target;
  std::vector<double> steps;
  blaze::StaticVector<double, 4UL> target = blaze::StaticVector<double, 4UL>(0.0);
  std::unique_ptr<int> counter;

  bool shiftQPressed;
  std::chrono::high_resolution_clock::time_point lastPressTime;
  SDL_Window *window;
  SDL_Renderer *renderer;
  bool running;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

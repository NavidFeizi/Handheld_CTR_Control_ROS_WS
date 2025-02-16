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
#include "interfaces/srv/config.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// #include "keyboardInput.hpp"
#include <SDL2/SDL.h>

using namespace std::chrono_literals;

class KeyboardNode : public rclcpp::Node
{
public:
  KeyboardNode() : Node("keyboard"), count_(0)
  {
    steps = {1.00 * M_PI / 180.00, 0.50E-3, 1.00 * M_PI / 180.00, 0.50E-3};
    m_com_vel = 0.00;
    counter = 0;
    m_sample_time = 1.00E-3;
    t0_ = 0.00;
    m_t = 0.00;

    running = true;
    shiftQPressed = false;

    // KeyboardNode::declare_parameters();
    KeyboardNode::setup_ros_interfaces();
    KeyboardNode::initialize_keyboard();

    rclcpp::Time now = this->get_clock()->now();
    t0_ = static_cast<double>(now.nanoseconds()) / 1E9;
    m_t = 0.00;

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

    auto dt = std::chrono::microseconds(static_cast<int>(m_sample_time * 1e6));
    m_timer = this->create_wall_timer(dt, std::bind(&KeyboardNode::target_joints_callback, this), m_callback_group_pub);

    m_timer_event = this->create_wall_timer(1ms, std::bind(&KeyboardNode::event_callback, this), m_callback_group_event);
  }

  //
  void send_homing_request(std::string message)
  {
    auto request = std::make_shared<interfaces::srv::Config::Request>();
    request->command = message; // Set the desired duration

    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
    auto response_received_callback = std::bind(&KeyboardNode::handle_homing_response, this, std::placeholders::_1);
    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

  //
  void send_target(const blaze::StaticVector<double, 4UL> targ)
  {
    auto request = std::make_shared<interfaces::srv::Config::Request>();
    request->command = "move";        // Set the desired duration
    request->target[0UL] = targ[0UL]; // Set the desired duration
    request->target[1UL] = targ[1UL]; // Set the desired duration
    request->target[2UL] = targ[2UL]; // Set the desired duration
    request->target[3UL] = targ[3UL]; // Set the desired duration

    using ServiceResponseFuture = rclcpp::Client<interfaces::srv::Config>::SharedFuture;
    auto response_received_callback = std::bind(&KeyboardNode::handle_homing_response, this, std::placeholders::_1);
    auto future_result = m_record_client->async_send_request(request, response_received_callback);
  }

  //
  void handle_homing_response(const rclcpp::Client<interfaces::srv::Config>::SharedFuture future)
  {
    // Get the result of the future object
    auto response = future.get();

    if (response->success)
    {
      if (response->message == "Manual mode activated" or response->message == "Home is set")
      {
        m_com_vel[0UL] = response->position[0UL];
        m_com_vel[1UL] = response->position[1UL];
        m_com_vel[2UL] = response->position[2UL];
        m_com_vel[3UL] = response->position[3UL];
        RCLCPP_INFO(this->get_logger(), "Current positino is: %0.4f  %0.4f  %0.4f  %0.4f",
                    response->position[0UL], response->position[1UL], response->position[2UL], response->position[3UL]);
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

    m_com_vel = {0.0, 0.0, 0.0, 0.0,};

    // Coordinates for the triangle
    constexpr int centerX = 320; // Half of window width
    constexpr int centerY = 240; // Half of window height
    constexpr int size = 100;    // Size of the triangle

    constexpr int x1 = centerX, y1 = centerY - size;
    constexpr int x2 = centerX + size, y2 = centerY + size;
    constexpr int x3 = centerX - size, y3 = centerY + size;

    drawTriangle(renderer, x1, y1, x2, y2, x3, y3);

    SDL_RenderPresent(renderer);

    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_WINDOWEVENT)
        {
            if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED)
                isFocused = true;
            else if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)
                isFocused = false;
            else if (event.window.event == SDL_WINDOWEVENT_CLOSE)
                running = false;
        }
        else if (event.type == SDL_QUIT)
        {
            running = false;
        }
    }

    const Uint8 *state = SDL_GetKeyboardState(NULL);
    if (state[SDL_SCANCODE_RIGHT] && !state[SDL_SCANCODE_LEFT])  m_com_vel[0UL] = 2.0;
    if (state[SDL_SCANCODE_LEFT] && !state[SDL_SCANCODE_RIGHT])   m_com_vel[0UL] = -2.0;
    if (state[SDL_SCANCODE_UP] && !state[SDL_SCANCODE_DOWN])     m_com_vel[1UL] = 1.0;
    if (state[SDL_SCANCODE_DOWN] && !state[SDL_SCANCODE_UP])   m_com_vel[1UL] = -1.0;
    if (state[SDL_SCANCODE_D] && !state[SDL_SCANCODE_A])      m_com_vel[2UL] = 2.0;
    if (state[SDL_SCANCODE_A] && !state[SDL_SCANCODE_D])      m_com_vel[2UL] = -2.0;
    if (state[SDL_SCANCODE_W] && !state[SDL_SCANCODE_S])      m_com_vel[3UL] = 1.0;
    if (state[SDL_SCANCODE_S] && !state[SDL_SCANCODE_W])      m_com_vel[3UL] = -1.0;

    KeyboardNode::send_target(m_com_vel);

    if (state[SDL_SCANCODE_SPACE] && isFocused) {
      if (m_flag_manual) {
          m_flag_manual = false;
          KeyboardNode::send_homing_request("OFF");
      } else {
          m_flag_manual = true;
          KeyboardNode::send_homing_request("ON");
      }
    }
    if (state[SDL_SCANCODE_H] && m_flag_manual) {
        KeyboardNode::send_homing_request("set_home");
    }

    lastPressTime = std::chrono::high_resolution_clock::now();

      // else if (event.type == SDL_KEYDOWN && isFocused)
      // {
      //   auto currentTime = std::chrono::high_resolution_clock::now();
      //   auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPressTime);

      //   if (timeDiff.count() >= 50)
      //   {
      //     SDL_Keycode key = event.key.keysym.sym;
      //     // std::cout << "event: " << key << std::endl;
      //     switch (key)
      //     {
      //     case SDLK_RIGHT:
      //       m_com_vel[0UL] = 1.0;
      //       break;
      //     case SDLK_LEFT:
      //       m_com_vel[0UL] = -1.0;
      //       break;
      //     case SDLK_UP:
      //       m_com_vel[1UL] = 1.0;
      //       break;
      //     case SDLK_DOWN:
      //       m_com_vel[1UL] = -1.0;
      //       break;
      //     case SDLK_d:
      //       m_com_vel[2UL] = 1.0;
      //       break;
      //     case SDLK_a:
      //       m_com_vel[2UL] = -1.0;
      //       break;
      //     case SDLK_w:
      //       m_com_vel[3UL] = 1.0;
      //       break;
      //     case SDLK_s:
      //       m_com_vel[3UL] = -1.0;
      //       KeyboardNode::send_target(m_com_vel);
      //       break;
      //     case SDLK_0:
      //       m_com_vel = 0.00;
      //       KeyboardNode::send_target(m_com_vel);
      //       break;
      //     case SDLK_SPACE:
      //       if (m_flag_manual)
      //       {
      //         m_flag_manual = false;
      //         KeyboardNode::send_homing_request("OFF");
      //       }
      //       else
      //       {
      //         m_flag_manual = true;
      //         KeyboardNode::send_homing_request("ON");
      //       }
      //       break;
      //     case SDLK_h:
      //       if (m_flag_manual)
      //       {
      //         KeyboardNode::send_homing_request("set_home");
      //       }
      //       break;
      //     default:
      //       break;
      //     }

      //     lastPressTime = std::chrono::high_resolution_clock::now();
      //   }
      // }
      // else if (event.type == SDL_QUIT)
      // {
      //   running = false;
      // }
    // }
  }

  // Timer callback function to publish the target joints config
  void target_joints_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    m_msg.position[0UL] = m_com_vel[0UL];
    m_msg.position[1UL] = m_com_vel[1UL];
    m_msg.position[2UL] = m_com_vel[2UL];
    m_msg.position[3UL] = m_com_vel[3UL];

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
    rclcpp::Rate rate(1000.00 / milliseconds);
    rate.sleep();
  }

  const std::vector<double> m_steps = {1.00 * M_PI / 180.00, 0.50E-3, 1.00 * M_PI / 180.00, 0.50E-3};
  // std::vector<double> target(4, 0);

  // member variables
  size_t count_;
  double m_sample_time = 1.00E-3; //[s]
  double t0_, m_t, t0 = 0.00;

  const double m_expt_time = 30.0;

  interfaces::msg::Jointspace m_msg;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::TimerBase::SharedPtr m_timer_event;
  rclcpp::Publisher<interfaces::msg::Jointspace>::SharedPtr m_publisher;

  rclcpp::CallbackGroup::SharedPtr m_callback_group_pub;
  rclcpp::CallbackGroup::SharedPtr m_callback_group_event;
  rclcpp::Client<interfaces::srv::Config>::SharedPtr m_record_client;

  std::atomic<bool> m_is_experiment_running;
  std::thread m_experiment_thread;

  bool m_flag_manual = false;

  SDL_Event event;
  bool isFocused = true;

  // std::vector<double> &target;
  std::vector<double> steps;
  blaze::StaticVector<double, 4UL> m_com_vel = blaze::StaticVector<double, 4UL>(0.0);
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

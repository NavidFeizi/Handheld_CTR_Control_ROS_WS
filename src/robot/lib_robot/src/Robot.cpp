#include "Robot.hpp"

using namespace std::chrono_literals;
using namespace lely;

std::vector<double> Position_Target_Generator(double t);

CTRobot::CTRobot(int sample_time,
                 int operation_mode,
                 blaze::StaticVector<double, 4UL> max_acc,
                 blaze::StaticVector<double, 4UL> max_vel,
                 bool position_limit)

{ // pulse per mm/deg - conversion to SI unit is done internally in the node class
  this->encoder_res = {100 * 180 * M_1_PI,
                       1000 * 1000.0,
                       100 * 180 * M_1_PI,
                       1000 * 1000.0};  // this is not real encoder count,
  this->velocity_factor = {10.0,
                           10.0,
                           10.0,
                           10.0}; 

  // the real encoder count and gear ratio are integrated into
  // motion controller factors. these are just a factor
  // to convert 0.01 mm to mm and 0.1 deg to deg
  this->gear_ratio = {1, 1, 1, 1}; // deg->rev or mm->rev
  m_max_acc = max_acc;             // deg->rev or mm->rev
  m_max_vel = max_vel;             // deg->rev or mm->rev
  this->sample_time = sample_time; // commandPeriod [ms], minimum
  this->operation_mode = operation_mode;

  this->lowerBounds = {-2 * M_PI, 0.0, -2 * M_PI, 0.0};
  this->upperBounds = {2 * M_PI, 97.0E-3, 2 * M_PI, 52.0E-3};

  this->posOffsets = {0.0, -147.0E-3, 0.0, -77.0E-3};
  this->clearance_min = 29.0E-3;
  this->clearance_max = 70.0E-3;
  this->flag_position_limit = position_limit;

  // /* Create log file and name it with time*/
  // std::filesystem::create_directories(Log_directory);
  // // Get the current date and time to create a unique filename
  // auto now = std::chrono::system_clock::now();
  // auto time = std::chrono::system_clock::to_time_t(now);
  // struct tm tm;
  // localtime_r(&time, &tm);
  // // Format the filename using the date and time
  // char buffer[80];
  // strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &tm);
  // std::string filename = std::string(buffer) + ".txt"; // Use the formatted time as the filename
  // logFile.open(Log_directory + filename, std::ios::app);
  // if (!logFile.is_open())
  // {
  //   std::cerr << "Failed to open the log file for writing." << std::endl;
  // }
}

/* Copy constructor */
CTRobot::CTRobot(const CTRobot &rhs) : m_inner_rot(rhs.m_inner_rot),
                                       m_inner_tran(rhs.m_inner_tran),
                                       m_middle_rot(rhs.m_middle_rot),
                                       m_middle_tran(rhs.m_middle_tran) {};

/* class destructor: move the joints to zero position, disable, and close cotrollers. */
CTRobot::~CTRobot()
{
  Log_message("------- Closing -------", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // if (logFile.is_open())
  // {
  //   logFile.close(); // Close the file in the destructor
  // }
}

/* initilizes the fiber driver for each node and what the status on all nodes */
void CTRobot::Fiber_loop()
{
  // Initialize the I/O library. This is required on Windows, but a no-op on
  // Linux (for now).
  io::IoGuard io_guard;

  // Create an I/O context to synchronize I/O services during shutdown.
  io::Context ctx;
  // Create a platform-specific I/O polling instance to monitor the CAN bus, as
  // well as timers and signals.
  io::Poll poll(ctx);
  // Create a polling event loop and pass it the platform-independent polling
  // interface. If no tasks are pending, the event loop will poll for I/O
  // events.
  ev::Loop loop(poll.get_poll());
  // I/O devices only need access to the executor interface of the event loop.
  auto exec = loop.get_executor();
  // Create a timer using a monotonic clock, i.e., a clock that is not affected
  // by discontinuous jumps in the system time.
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);

  // Create a virtual SocketCAN CAN controller and channel, and do not modify
  // the current CAN bus state or bitrate.
  io::CanController ctrl("can0");
  io::CanChannel chan(poll, exec);

  chan.open(ctrl);

  // Create a CANopen master with node-ID 1. The master is asynchronous, which
  // means every user-defined callback for a CANopen event will be posted as a
  // task on the event loop, instead of being invoked during the event
  // processing by the stack.
  std::string master_dcf = "/master.dcf";
  std::string master_bin = "/master.bin";
  canopen::AsyncMaster master(timer,
                              chan,
                              CANopenFiles_directory + master_dcf,
                              CANopenFiles_directory + master_bin,
                              7);

  // Create a signal handler.
  io::SignalSet sigset(poll, exec);
  // Watch for Ctrl+C or process termination.
  sigset.insert(SIGHUP);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);

  // Submit a task to be executed when a signal is raised. We don't care which.
  sigset.submit_wait([&](int /*signo*/)
                     {
    // If the signal is raised again, terminate immediately.
    sigset.clear();
    // Tell the master to start the deconfiguration process for all nodes, and
    // submit a task to be executed once that process completes.
    master.AsyncDeconfig().submit(exec, [&]() {
      // Perform a clean shutdown.
      ctx.shutdown();
    }); });

  // Start the NMT service of the master by pretending to receive a 'reset
  // node' command.
  master.Reset();

  // Run the event loop until no tasks remain (or the I/O context is shut down).

  // Create a driver for the slave with node-ID 2.
  // MyDriver driver_5(exec, master, 5);

  exec.post([&]()
            { m_inner_rot = std::make_shared<Node>(this, exec, master, 3, encoder_res[0], gear_ratio[0], velocity_factor[0],
                                                   sample_time, operation_mode, m_max_acc[0], m_max_vel[0]); });
  exec.post([&]()
            { m_inner_tran = std::make_shared<Node>(this, exec, master, 4, encoder_res[1], gear_ratio[1], velocity_factor[1],
                                                    sample_time, operation_mode, m_max_acc[1], m_max_vel[1]); });
  exec.post([&]()
            { m_middle_rot = std::make_shared<Node>(this, exec, master, 1, encoder_res[2], gear_ratio[2], velocity_factor[2],
                                                    sample_time, operation_mode, m_max_acc[2], m_max_vel[2]); });
  exec.post([&]()
            { m_middle_tran = std::make_shared<Node>(this, exec, master, 2, encoder_res[3], gear_ratio[3], velocity_factor[3],
                                                     sample_time, operation_mode, m_max_acc[3], m_max_vel[3]); });
  {
    std::thread t1([&]()
                   { loop.run(); });
    t1.detach();
  }
  // this wait is mandatory to star the loop before commanding targets
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::chrono::seconds timeout(5);
  auto startTime = std::chrono::high_resolution_clock::now();
  while (!m_boot_success)
  {

    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
    if (elapsedTime >= timeout)
    {
      Log_message("Bootup Timed Out!  --->  Reseting All Nodes -------", true);
      std::raise(SIGINT);
      master.Reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      // break; // Exit the loop on timeout
    }
    if (m_inner_rot->flag_bootSuccess &&
        m_inner_tran->flag_bootSuccess &&
        m_middle_rot->flag_bootSuccess &&
        m_middle_tran->flag_bootSuccess)
    {
      m_boot_success = true;
      Log_message("------- Nodes Booted Successfully -------", true);
    }
  }

  while (true)
  {
    if (CTRobot::check_all_nodes_switched_on())
    {
      if (!m_flag_robot_switched_on)
      {
        m_flag_robot_switched_on = true;
        Log_message("------- Nodes Switched ON -------", true);
      }
    }
    else
    {
      if (m_flag_robot_switched_on)
      {
        m_flag_robot_switched_on = false;
        Log_message("------- At Least One Node Switched OFF -------", true);
      }
    }

    if (CTRobot::check_all_nodes_enabled())
    {
      if (!m_flag_operation_enabled)
      {
        m_flag_operation_enabled = true;
        Log_message("------- Nodes Enabled -------", true);
      }
    }
    else
    {
      if (m_flag_operation_enabled)
      {
        m_flag_operation_enabled = false;
        Log_message("------- At Least One Node Disabled -------", true);
      }
    }

    if (CTRobot::check_all_encoders_set())
    {
      if (!m_encoders_set)
      {
        m_encoders_set = true;
        Log_message("------- Encoders Set -------", true);
      }
    }
    else
    {
      if (m_encoders_set)
      {
        m_encoders_set = false;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

bool CTRobot::check_all_nodes_switched_on()
{
  return m_inner_rot->m_status_word.switched_ON &&
         m_inner_tran->m_status_word.switched_ON &&
         m_middle_rot->m_status_word.switched_ON &&
         m_middle_tran->m_status_word.switched_ON;
}

bool CTRobot::check_all_nodes_enabled()
{
  return m_inner_rot->m_status_word.operation_enabled &&
         m_inner_tran->m_status_word.operation_enabled &&
         m_middle_rot->m_status_word.operation_enabled &&
         m_middle_tran->m_status_word.operation_enabled;
}

bool CTRobot::check_all_encoders_set()
{
  return m_inner_rot->encoder_set &&
         m_inner_tran->encoder_set &&
         m_middle_rot->encoder_set &&
         m_middle_tran->encoder_set;
}

/* starts the Fiber_Loop in a separate thread */
void CTRobot::Start_Thread()
{
  EnableThread = std::thread(&CTRobot::Fiber_loop, this);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (!m_inner_rot->m_tasks_posted ||
         !m_inner_tran->m_tasks_posted ||
         !m_middle_rot->m_tasks_posted ||
         !m_middle_tran->m_tasks_posted)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Log_message("------- Tasks Posted -------", true);
}

/* enable operation of all joints
  @param enable: true = enable, false = disable */
void CTRobot::Enable_Operation(bool enable)
{
  m_inner_rot->Enable_operation(enable);
  m_inner_tran->Enable_operation(enable);
  m_middle_rot->Enable_operation(enable);
  m_middle_tran->Enable_operation(enable);
}

/* set the current positoin of the robot as zero position on the motion controller
  - this function halts the target task loop while running */
void CTRobot::Set_Zero_Position(blaze::StaticVector<double, 4> offset)
{
  Log_message("---- Setting zero position ----", true);
  m_inner_rot->m_configing = true;
  m_inner_tran->m_configing = true;
  m_middle_rot->m_configing = true;
  m_middle_tran->m_configing = true;

  while (m_inner_rot->m_configing || m_inner_tran->m_configing || m_middle_rot->m_configing || m_middle_tran->m_configing)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Log_message("---- Setting zero position done ----", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/* returns the status of motion controller switches
  - true, when all motion controllers are switched on and ready to enable operation */
bool CTRobot::Get_Controller_Switch_Status()
{
  return m_flag_robot_switched_on;
}

/* Actuatre robot to the targert absolute joint positions in SI unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void CTRobot::set_target_position(blaze::StaticVector<double, 4> posTarget)
{
  if (this->flag_position_limit)
  {
    if (!(CTRobot::Position_limits_check(posTarget) == 0))
    {
      return;
    }
  }

  m_inner_rot->Set_target_pos(posTarget[0]);   // in [rad]
  m_inner_tran->Set_target_pos(posTarget[1]);  // in [m]
  m_middle_rot->Set_target_pos(posTarget[2]);  // in [rad]
  m_middle_tran->Set_target_pos(posTarget[3]); // in [m]
}

/* Actuatre robot to the targert absolute joint positions in SI unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void CTRobot::Set_Target_Velocity(blaze::StaticVector<double, 4> velTarget)
{
  m_inner_rot->Set_target_vel(velTarget[0]);   // in [rad]
  m_inner_tran->Set_target_vel(velTarget[1]);  // in [m]
  m_middle_rot->Set_target_vel(velTarget[2]);  // in [rad]
  m_middle_tran->Set_target_vel(velTarget[3]); // in [m]
}

/* Gets the current in [mA] unit */
void CTRobot::Get_Current(blaze::StaticVector<int, 4> *p_current)
{
  if (p_current)
  {
    // Get the motor torque for each member and store it in p_current
    this->m_inner_rot->Get_current(&((*p_current)[0]));
    this->m_inner_tran->Get_current(&((*p_current)[1]));
    this->m_middle_rot->Get_current(&((*p_current)[2]));
    this->m_middle_tran->Get_current(&((*p_current)[3]));
  }
}

/* Gets the current velocity (with respect to zero position - distal limit) of all actuators in SI unit */
void CTRobot::Get_Velocity(blaze::StaticVector<double, 4> *p_velCurrent)
{
  if (p_velCurrent)
  {
    // Get the motor torque for each member and store it in p_current
    this->m_inner_rot->Get_actual_vel(&((*p_velCurrent)[0]));
    this->m_inner_tran->Get_actual_vel(&((*p_velCurrent)[1]));
    this->m_middle_rot->Get_actual_vel(&((*p_velCurrent)[2]));
    this->m_middle_tran->Get_actual_vel(&((*p_velCurrent)[3]));
  }
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in SI unit */
void CTRobot::Get_Position(blaze::StaticVector<double, 4> *p_posCurrent)
{
  if (p_posCurrent)
  {
    // Get the motor position for each member and store it in p_current
    this->m_inner_rot->Get_actual_pos(&((*p_posCurrent)[0]));
    this->m_inner_tran->Get_actual_pos(&((*p_posCurrent)[1]));
    this->m_middle_rot->Get_actual_pos(&((*p_posCurrent)[2]));
    this->m_middle_tran->Get_actual_pos(&((*p_posCurrent)[3]));
  }
}

/**/
void CTRobot::Get_PosVelCur(blaze::StaticVector<double, 4> *p_posCurrent,
                            blaze::StaticVector<double, 4> *p_velCurrent,
                            blaze::StaticVector<int, 4> *p_current)
{
  CTRobot::Get_Position(p_posCurrent);
  CTRobot::Get_Velocity(p_velCurrent);
  CTRobot::Get_Current(p_current);
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in [mm] or [deg] unit */
void CTRobot::Convert_pos_to_CTR_frame(blaze::StaticVector<double, 4> &posCurrent,
                                       blaze::StaticVector<double, 4> *posInCTRFrame)
{
  for (size_t i = 0; i < posCurrent.size(); ++i)
  {
    (*posInCTRFrame)[i] = posCurrent[i] + this->posOffsets[i];
  }
}

/**/
int CTRobot::Position_limits_check(blaze::StaticVector<double, 4> posTarget)
{
  for (size_t i = 0; i < posTarget.size(); ++i)
  {
    if (posTarget[i] < lowerBounds[i])
    {
      std::cout << "static lower bound reached" << std::endl;
      return -1;
    }
    if (posTarget[i] > upperBounds[i])
    {
      std::cout << "static upper bound reached" << std::endl;
      return -1;
    }
  }
  blaze::StaticVector<double, 4> posInCTRFrame = blaze::StaticVector<double, 4>(0.0);

  CTRobot::Convert_pos_to_CTR_frame(posTarget, &posInCTRFrame);
  if ((posInCTRFrame[3] - posInCTRFrame[1]) < clearance_min)
  {
    std::cout << "dynamic position lower limit reached - preventing collision => position target ignored" << std::endl;
    return -1;
  }
  if ((posInCTRFrame[3] - posInCTRFrame[1]) > clearance_max)
  {
    std::cout << "dynamic position upper limit reached - preventing illegan tube configuration => position target ignored" << std::endl;
    return -1;
  }
  return 0;
}

/**/
bool CTRobot::Get_reachStatus()
{
  if (m_inner_rot->Is_reached() &&
      m_inner_tran->Is_reached() &&
      m_middle_rot->Is_reached() &&
      m_middle_tran->Is_reached())
  {
    return 1;
  }
  else
  {
    return 0;
  }
  // std::vector<double> thresholds = {0.1, 0.05, 0.1, 0.05};
  // std::vector<double> velocity;
  // CTRobot::Get_Velocity(&velocity);

  // for (size_t i = 0; i < velocity.size(); i++)
  // {
  //   if (std::abs(velocity[i]) >= thresholds[i])
  //   {
  //     return false; // At least one velocity exceeds the threshold
  //   }
  // }
  // return true;
}

/**/
void CTRobot::Wait_until_reach()
{
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (!CTRobot::Get_reachStatus())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

/**/
void CTRobot::Log_message(const std::string &message, bool print)
{
  // Get the current date and time to include in the log entry
  time_t now = time(0);
  tm *localTime = localtime(&now);
  char timestamp[80];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localTime);
  if (logFile.is_open())
  {
    // Write the message to the log file along with a timestamp
    logFile << "[Robot : " << timestamp << "] " << message << std::endl;
  }
  else
  {
    // std::cerr << "Error: Unable to open the log file for writing." << std::endl;
  }
  // Print the message to the console using std::cout
  if (print)
  {
    std::cout << "[Robot : " << timestamp << "] " << message << std::endl;
  }
}

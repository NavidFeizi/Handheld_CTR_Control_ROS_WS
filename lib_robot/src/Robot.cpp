#include "Robot.hpp"

using namespace std::chrono_literals;
using namespace lely;

std::vector<double> Position_Target_Generator(double t);

CTRobot::CTRobot(int sample_time,
                 int operation_mode,
                 std::vector<double> max_acc,
                 std::vector<double> max_vel,
                 bool position_limit)

{ // pulse per mm/deg - conversion to SI unit is done internally in the node class
  this->encoder_res = {100 * 180 * M_1_PI,
                       1000 * 1000.0,
                       100 * 180 * M_1_PI,
                       1000 * 1000.0};
  this->velocity_factor = {10.0,
                           10.0,
                           10.0,
                           10.0}; // this is not real encoder count,

  // the real encoder count and gear ratio are integrated into
  // motion controller factors. these are just a factor
  // to convert 0.01 mm to mm and 0.1 deg to deg
  this->gear_ratio = {1, 1, 1, 1}; // deg->rev or mm->rev
  this->max_acc = max_acc;         // deg->rev or mm->rev
  this->max_vel = max_vel;         // deg->rev or mm->rev
  this->sample_time = sample_time; // commandPeriod [ms], minimum
  this->operation_mode = operation_mode;

  this->lowerBounds = {-6 * M_PI, 0.0, -6 * M_PI, 0.0};
  this->upperBounds = {6 * M_PI, 97.0E-3, 6 * M_PI, 52.0E-3};

  this->posOffsets = {0.0, -147.0E-3, 0.0, -77.0E-3};
  this->clearance_min = 29.0E-3;
  this->clearance_max = 71.0E-3;
  this->flag_position_limit = position_limit;

  /* Create log file and name it with time*/
  std::filesystem::create_directories(Log_directory);
  // Get the current date and time to create a unique filename
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  struct tm tm;
  localtime_r(&time, &tm);
  // Format the filename using the date and time
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &tm);
  std::string filename = std::string(buffer) + ".txt"; // Use the formatted time as the filename
  logFile.open(Log_directory + filename, std::ios::app);
  if (!logFile.is_open())
  {
    std::cerr << "Failed to open the log file for writing." << std::endl;
  }
}

/* Copy constructor */
CTRobot::CTRobot(const CTRobot &rhs) : inner_rot(rhs.inner_rot),
                                       inner_tran(rhs.inner_tran),
                                       middle_rot(rhs.middle_rot),
                                       middle_tran(rhs.middle_tran){};

/* class destructor: move the joints to zero position, disable, and close cotrollers. */
CTRobot::~CTRobot()
{
  Log_message("Closing", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (logFile.is_open())
  {
    logFile.close(); // Close the file in the destructor
  }
  
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
  canopen::AsyncMaster master(timer, chan, CANopenFiles_directory + master_dcf, CANopenFiles_directory + master_bin, 7);

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
            { inner_rot = std::make_shared<Node>(this, exec, master, 1, encoder_res[0], gear_ratio[0], velocity_factor[0],
                                                 sample_time, operation_mode, max_acc[0], max_vel[0]); });
  exec.post([&]()
            { inner_tran = std::make_shared<Node>(this, exec, master, 2, encoder_res[1], gear_ratio[1], velocity_factor[1],
                                                  sample_time, operation_mode, max_acc[1], max_vel[1]); });
  exec.post([&]()
            { middle_rot = std::make_shared<Node>(this, exec, master, 3, encoder_res[2], gear_ratio[2], velocity_factor[2],
                                                  sample_time, operation_mode, max_acc[2], max_vel[2]); });
  exec.post([&]()
            { middle_tran = std::make_shared<Node>(this, exec, master, 4, encoder_res[3], gear_ratio[3], velocity_factor[3],
                                                   sample_time, operation_mode, max_acc[3], max_vel[3]); });
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
      Log_message("Bootup Timed Out!  --->  Reseting All Nodes ----------------------", true);
      std::raise(SIGINT);
      master.Reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      // break; // Exit the loop on timeout
    }
    if (inner_rot->flag_bootSuccess &&
        inner_tran->flag_bootSuccess &&
        middle_rot->flag_bootSuccess &&
        middle_tran->flag_bootSuccess)
    {
      m_boot_success = true;
      Log_message("---------------------- Nodes Booted Successfully ----------------------", true);
    }
  }

  while (true)
  {
    if (inner_rot->m_status_word.switched_ON & inner_tran->m_status_word.switched_ON &
        middle_rot->m_status_word.switched_ON & middle_tran->m_status_word.switched_ON)
    {
      if (!this->flag_robot_switched_ON)
      {
        flag_robot_switched_ON = true;
        Log_message("---------------------- Nodes Switched ON ----------------------", true);
      }
    }
    else
    {
      if (this->flag_robot_switched_ON)
      {
        this->flag_robot_switched_ON = false;
        Log_message("---------------------- At Least One Node Switched OFF ----------------------", true);
      }
    }

    if (inner_rot->m_status_word.operation_enabled & inner_tran->m_status_word.operation_enabled &
        middle_rot->m_status_word.operation_enabled & middle_tran->m_status_word.operation_enabled)
    {
      if (!this->operation_enabled)
      {
        operation_enabled = true;
        Log_message("---------------------- Nodes Enabled ----------------------", true);
      }
    }
    else
    {
      if (this->operation_enabled)
      {
        this->operation_enabled = false;
        Log_message("---------------------- At Least One Node Disabled ----------------------", true);
      }
    }

    if (inner_rot->encoder_set & inner_tran->encoder_set &
        middle_rot->encoder_set & middle_tran->encoder_set)
    {
      if (!this->encoders_set)
      {
        encoders_set = true;
        Log_message("---------------------- Encoders Set ----------------------", true);
      }
    }
    else
    {
      if (this->encoders_set)
      {
        this->encoders_set = false;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

/* starts the Fiber_Loop in a separate thread */
void CTRobot::Start_Thread()
{
  EnableThread = std::thread(&CTRobot::Fiber_loop, this);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (!inner_rot->m_tasks_posted ||
         !inner_tran->m_tasks_posted ||
         !middle_rot->m_tasks_posted ||
         !middle_tran->m_tasks_posted)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Log_message("---------------------- Tasks Posted ----------------------", true);
}

/* enable operation of all joints
  @param enable: true = enable, false = disable */
void CTRobot::Enable_Operation(bool enable)
{
  inner_rot->Enable_operation(enable);
  inner_tran->Enable_operation(enable);
  middle_rot->Enable_operation(enable);
  middle_tran->Enable_operation(enable);
}

/* set the current positoin of the robot as zero position on the motion controller
  - this function halts the target task loop while running */
void CTRobot::Set_Zero_Position(std::vector<double> offset)
{
  Log_message("---- Setting zero position ----", true);
  inner_rot->m_configing = true;
  inner_tran->m_configing = true;
  middle_rot->m_configing = true;
  middle_tran->m_configing = true;

  while (inner_rot->m_configing || inner_tran->m_configing || middle_rot->m_configing || middle_tran->m_configing)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Log_message("---- Setting zero position done ----", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/* find the positive mechanical limit of the linear joints based on current threshold
  - this function currently works when the mode of operation is PV*/
void CTRobot::Find_Fowrard_Limits()
{
  std::vector<double> vel_findlimit(4, 4.0);                    //[mm] or [deg] / [sec]
  std::vector<double> current_threshold = {300, 300, 300, 300}; // [mA]

  std::vector<int> p_current(4, 0);
  std::vector<bool> limit_reached(4, false);
  const int maxBufferSize = 100;
  std::vector<std::deque<int>> currentBuffers(4); // Circular buffers for each joint
  std::vector<double> current_avg(4, 0.0);

  inner_rot->Set_operation_mode(3);

  this->Set_Target_Velocity(vel_findlimit);

  inner_rot->Enable_operation(true);
  inner_tran->Enable_operation(true);
  middle_rot->Enable_operation(true);
  middle_tran->Enable_operation(true);

  while (!std::all_of(limit_reached.begin(), limit_reached.end(), [](bool l)
                      { return l; }))
  {
    // update average current
    this->Get_Current(&p_current);
    for (int node = 0; node < 6; ++node)
    {
      currentBuffers[node].push_back(p_current[node]);

      if (currentBuffers[node].size() > maxBufferSize)
      {
        currentBuffers[node].pop_front(); // Keep the buffer size at 100
      }

      int sum = 0;
      for (int current : currentBuffers[node])
      {
        sum += current;
      }

      current_avg[node] = static_cast<double>(sum) / currentBuffers[node].size();
    }

    if (abs(current_avg[0]) >= current_threshold[0] && !limit_reached[0])
    {
      limit_reached[0] = true;
      vel_findlimit[0] = 0;
      inner_rot->Enable_operation(false);
      inner_rot->m_configing = true;
    }

    if (abs(current_avg[1]) >= current_threshold[1] && !limit_reached[1])
    {
      limit_reached[1] = true;
      vel_findlimit[1] = 0;
      inner_tran->Enable_operation(false);
      inner_tran->m_configing = true;
    }

    if (abs(current_avg[2]) >= current_threshold[2] && !limit_reached[2])
    {
      limit_reached[2] = true;
      vel_findlimit[2] = 0;
      middle_rot->Enable_operation(false);
      middle_rot->m_configing = true;
    }

    if (abs(current_avg[3]) >= current_threshold[3] && !limit_reached[3])
    {
      limit_reached[3] = true;
      vel_findlimit[3] = 0;
      middle_tran->Enable_operation(false);
      middle_tran->m_configing = true;
    }

    std::cout << "Finding limit... \n"
              << "Node 1:"
              << "averaged current :" << current_avg[0] << " | reached: " << limit_reached[0] << "\n"
              << "Node 2:"
              << "averaged current :" << current_avg[1] << " | reached: " << limit_reached[1] << "\n"
              << "Node 3:"
              << "averaged current :" << current_avg[2] << " | reached: " << limit_reached[2] << "\n"
              << "Node 4:"
              << "averaged current :" << current_avg[3] << " | reached: " << limit_reached[3] << "\n"
              << "-----------------------------------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::vector<double> v = {0.0, 0.0, 0.0, 0.0};
  CTRobot::Set_Target_Velocity(v);
  CTRobot::Enable_Operation(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "---- Finding limit done ----" << std::endl;
}

/* returns the status of motion controller switches
  - true, when all motion controllers are switched on and ready to enable operation */
bool CTRobot::Get_Controller_Switch_Status()
{
  return flag_robot_switched_ON;
}

/* Actuatre robot to the targert absolute joint positions in mm or deg unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void CTRobot::Set_Target_AbsPosition(std::vector<double> posTarget)
{
  if (this->flag_position_limit)
  {
    if (!(CTRobot::Position_limits_check(posTarget) == 0))
    {
      return;
    }
  }

  inner_rot->Set_target_pos(posTarget[0]);   // in [mm]
  inner_tran->Set_target_pos(posTarget[1]);  // in [deg]
  middle_rot->Set_target_pos(posTarget[2]);  // in [mm]
  middle_tran->Set_target_pos(posTarget[3]); // in [mm]
}
/* Actuatre robot to the targert absolute joint positions in [mm/s] or [deg/s] unit
        (with respect to zero position (distal limit) - positive value is towards proximal end)     */
void CTRobot::Set_Target_Velocity(std::vector<double> velTarget)
{
  inner_rot->Set_target_vel(velTarget[0]);   // in [mm]
  inner_tran->Set_target_vel(velTarget[1]);  // in [deg]
  middle_rot->Set_target_vel(velTarget[2]);  // in [mm]
  middle_tran->Set_target_vel(velTarget[3]); // in [mm]
}

/* Gets the current in [mA] unit */
void CTRobot::Get_Current(std::vector<int> *p_current)
{
  if (p_current)
  {
    p_current->clear(); // Clear the vector if it's not empty

    int Current;

    // Get the motor torque for each member and store it in p_current
    this->inner_rot->Get_current(&Current);
    p_current->push_back(Current);
    this->inner_tran->Get_current(&Current);
    p_current->push_back(Current);
    this->middle_rot->Get_current(&Current);
    p_current->push_back(Current);
    this->middle_tran->Get_current(&Current);
    p_current->push_back(Current);
  }
}

/* Gets the current velocity (with respect to zero position - distal limit) of all actuators in [mm/s] or [deg/s] unit */
void CTRobot::Get_Velocity(std::vector<double> *p_velCurrent)
{
  if (p_velCurrent)
  {
    p_velCurrent->clear(); // Clear the vector if it's not empty

    double Vel;

    // Get the motor torque for each member and store it in p_current
    this->inner_rot->Get_actual_vel(&Vel);
    p_velCurrent->push_back(Vel);
    this->inner_tran->Get_actual_vel(&Vel);
    p_velCurrent->push_back(Vel);
    this->middle_rot->Get_actual_vel(&Vel);
    p_velCurrent->push_back(Vel);
    this->middle_tran->Get_actual_vel(&Vel);
    p_velCurrent->push_back(Vel);
  }
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in [mm] or [deg] unit */
void CTRobot::Get_Position(std::vector<double> *p_posCurrent)
{
  if (p_posCurrent)
  {
    p_posCurrent->clear(); // Clear the vector if it's not empty

    double Pos;

    // Get the motor position for each member and store it in p_current
    this->inner_rot->Get_actual_pos(&Pos);
    p_posCurrent->push_back(Pos);
    this->inner_tran->Get_actual_pos(&Pos);
    p_posCurrent->push_back(Pos);
    this->middle_rot->Get_actual_pos(&Pos);
    p_posCurrent->push_back(Pos);
    this->middle_tran->Get_actual_pos(&Pos);
    p_posCurrent->push_back(Pos);
  }
}

void CTRobot::Get_PosVelCur(std::vector<double> *p_posCurrent,
                            std::vector<double> *p_velCurrent,
                            std::vector<int> *p_current)
{
  CTRobot::Get_Position(p_posCurrent);
  CTRobot::Get_Velocity(p_velCurrent);
  CTRobot::Get_Current(p_current);
}

/* Gets the current absolute position (with respect to zero position - distal limit) of all actuators in [mm] or [deg] unit */
void CTRobot::Convert_pos_to_CTR_frame(std::vector<double> &posCurrent, std::vector<double> *posInCTRFrame)
{
  for (size_t i = 0; i < posCurrent.size(); ++i)
  {
    (*posInCTRFrame)[i] = posCurrent[i] + this->posOffsets[i];
  }
}

int CTRobot::Position_limits_check(std::vector<double> posTarget)
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
  std::vector<double> posInCTRFrame(4, 0.0);

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

bool CTRobot::Get_reachStatus()
{
  if (inner_rot->Is_reached() && inner_tran->Is_reached() && middle_rot->Is_reached() && middle_tran->Is_reached())
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

void CTRobot::Wait_until_reach()
{
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (!CTRobot::Get_reachStatus())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

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
    std::cerr << "Error: Unable to open the log file for writing." << std::endl;
  }
  // Print the message to the console using std::cout
  if (print)
  {
    std::cout << "[Robot : " << timestamp << "] " << message << std::endl;
  }
}

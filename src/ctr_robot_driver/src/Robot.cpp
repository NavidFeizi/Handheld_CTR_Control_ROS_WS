#include "ctr_robot_driver/Robot.hpp"

using namespace std::chrono_literals;
using namespace lely;

std::vector<double> Position_Target_Generator(double t);

CTRobot::CTRobot(bool position_limit, blaze::StaticVector<double, 4UL> max_vel, blaze::StaticVector<double, 4UL> max_acc)
{
  m_maxVel = max_vel; // [deg/s] and [mm/s]
  m_maxAcc = max_acc; // [deg/s^2] and [mm/s^2]
  // m_max_acc = max_acc;             // deg->rev or mm->rev
  // m_max_vel = max_vel;             // deg->rev or mm->rev
  // this->m_sampleTime = sample_time; // commandPeriod [ms], minimum
  this->operation_mode = OpMode::VelocityProfile;
  this->m_lowerBounds = {-2 * M_PI, 0.0, -2 * M_PI, 0.0};
  this->m_upperBounds = {2 * M_PI, 97.0E-3, 2 * M_PI, 52.0E-3};
  this->m_posOffsets = {0.0, -147.0E-3, 0.0, -77.0E-3};
  this->m_minClearance = 30.0E-3;
  this->m_maxClearance = 70.0E-3;
  this->m_flagPositionLimit = position_limit;

  m_shared_state = std::make_shared<SharedState>(); // states are shared with the Cia301 nodes
  CTRobot::initLogger();
}

/* default constructor */
CTRobot::CTRobot()
    : CTRobot(false,
              {200.00 * M_PI / 180.00, 10.00 * 1e-3, 200.00 * M_PI / 180.00, 10.00 * 1e-3},
              {200.00 * M_PI / 180.00, 10.00 * 1e-3, 200.00 * M_PI / 180.00, 10.00 * 1e-3}) {} // Calls parameterized constructor


/* class destructor: disable, and close cotrollers. */
CTRobot::~CTRobot()
{
  if (m_logger)
    m_logger->info("[CAN Master]  Closing");
  shutdown();
}

/* Orderly teardown: stop the monitor loop, deconfigure the CANopen nodes and
   stop the event loop, then join both threads (they used to be leaked/detached,
   which put the joinable m_thread destructor one CTRobot destruction away from
   std::terminate). Safe to call more than once. */
void CTRobot::shutdown()
{
  m_monitor_stop = true;
  if (m_monitorThread.joinable())
    m_monitorThread.join();
  if (m_can_running && m_request_can_shutdown)
    m_request_can_shutdown();
  if (m_canThread.joinable())
    m_canThread.join();
}

/* initilizes the fiber driver for each node and what the status on all nodes */
void CTRobot::startCANopenNodes()
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
  io::CanController ctrl(m_paths.can_interface.c_str());
  io::CanChannel chan(poll, exec);

  chan.open(ctrl);

  // Create a CANopen master with node-ID 1. The master is asynchronous, which
  // means every user-defined callback for a CANopen event will be posted as a
  // task on the event loop, instead of being invoked during the event
  // processing by the stack.
  canopen::AsyncMaster master(timer,
                              chan,
                              m_paths.canopen_dir + "/master.dcf",
                              m_paths.canopen_dir + "/master.bin",
                              7);

  // (The old io::SignalSet handler is gone: rclcpp owns process signals, and
  // shutdown() now drives the same AsyncDeconfig -> ctx.shutdown() sequence.)

  // Start the NMT service of the master by pretending to receive a 'reset
  // node' command.
  master.Reset();

  // Run the event loop until no tasks remain (or the I/O context is shut down).

  // Create a driver for the slave with node-ID 2.
  // MyDriver driver_5(exec, master, 5);

  exec.post([&]()
            { m_inrTubeRot = std::make_shared<Cia301Node>(exec, master, 1, "Faulhaber", m_encodersResolution[0], m_gearRatios[0], m_velocityFactors[0],
                                                          m_sampleTime, operation_mode, m_maxAcc[0], m_maxVel[0],
                                                          m_paths.resolvedEncoderMemoryDir(), m_shared_state, m_logger); });
  exec.post([&]()
            { m_inrTubeTrn = std::make_shared<Cia301Node>(exec, master, 2, "Faulhaber", m_encodersResolution[1], m_gearRatios[1], m_velocityFactors[1],
                                                          m_sampleTime, operation_mode, m_maxAcc[1], m_maxVel[1],
                                                          m_paths.resolvedEncoderMemoryDir(), m_shared_state, m_logger); });
  exec.post([&]()
            { m_mdlTubeRot = std::make_shared<Cia301Node>(exec, master, 3, "Faulhaber", m_encodersResolution[2], m_gearRatios[2], m_velocityFactors[2],
                                                          m_sampleTime, operation_mode, m_maxAcc[2], m_maxVel[2],
                                                          m_paths.resolvedEncoderMemoryDir(), m_shared_state, m_logger); });
  exec.post([&]()
            { m_mdlTubeTrn = std::make_shared<Cia301Node>(exec, master, 4, "Faulhaber", m_encodersResolution[3], m_gearRatios[3], m_velocityFactors[3],
                                                          m_sampleTime, operation_mode, m_maxAcc[3], m_maxVel[3],
                                                          m_paths.resolvedEncoderMemoryDir(), m_shared_state, m_logger); });
  // Hand shutdown()/the monitor controlled access to the stack-owned CANopen
  // objects. Valid while this thread is inside loop.run().
  m_request_can_shutdown = [&master, exec, &ctx]()
  {
    master.AsyncDeconfig().submit(exec, [&ctx]()
                                  { ctx.shutdown(); });
  };
  m_request_master_reset = [&master]()
  { master.Reset(); };

  m_monitor_stop = false;
  m_monitorThread = std::thread(&CTRobot::monitorLoop, this);

  // Run the event loop in THIS thread (it used to run in a detached thread
  // referencing this stack frame). Returns when ctx.shutdown() is invoked.
  m_can_running = true;
  loop.run();
  m_can_running = false;
}

/* Boot / switch-on / enable watch loop (used to run inline in the CAN thread
   with no way to stop it). Exits when m_monitor_stop is set or boot fails. */
void CTRobot::monitorLoop()
{
  // give the event loop time to create the node drivers
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(m_paths.boot_timeout_s));
  const int max_boot_attempts = m_paths.boot_max_attempts;
  int boot_attempts = 0;
  auto startTime = std::chrono::steady_clock::now();
  while (!m_monitor_stop && !m_shared_state->m_boot_success)
  {
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedTime = currentTime - startTime;
    if (elapsedTime >= timeout)
    {
      // The old code raised SIGINT at the whole process here. Retry the NMT
      // reset a bounded number of times, then report failure and stand down.
      boot_attempts++;
      if (boot_attempts > max_boot_attempts)
      {
        m_logger->error("[CAN Master] Bootup failed after {} reset attempts - giving up (hardware unavailable)",
                        max_boot_attempts);
        m_boot_failed = true;
        return;
      }
      m_logger->warn("[CAN Master] Bootup timed out - resetting all nodes (attempt {}/{})",
                     boot_attempts, max_boot_attempts);
      if (m_request_master_reset)
        m_request_master_reset();
      startTime = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    if (m_inrTubeRot && m_inrTubeTrn && m_mdlTubeRot && m_mdlTubeTrn &&
        m_inrTubeRot->getFlags(Flags::FlagIndex::BOOT_SUCCESS) &&
        m_inrTubeTrn->getFlags(Flags::FlagIndex::BOOT_SUCCESS) &&
        m_mdlTubeRot->getFlags(Flags::FlagIndex::BOOT_SUCCESS) &&
        m_mdlTubeTrn->getFlags(Flags::FlagIndex::BOOT_SUCCESS))
    {
      m_shared_state->m_boot_success = true;
      m_logger->info("[CAN Master] Nodes Booted Successfully");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  while (!m_monitor_stop && !m_shared_state->m_flag_robot_switched_on)
  {
    if (CTRobot::getSwitchStatus())
    {
      if (!m_shared_state->m_flag_robot_switched_on)
      {
        m_shared_state->m_flag_robot_switched_on = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_logger->info("[CAN Master] All Nodes Switched ON");
      }
    }
    else
    {
      if (m_shared_state->m_flag_robot_switched_on)
      {
        m_shared_state->m_flag_robot_switched_on = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        m_logger->info("[CAN Master] At Least One Node Switched OFF");
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  while (!m_monitor_stop)
  {
    auto en_status = CTRobot::getEnableStatus();
    if (en_status[0] && en_status[1] && en_status[2] && en_status[3])
    {
      if (!m_shared_state->m_flag_operation_enabled)
      {
        m_shared_state->m_flag_operation_enabled = true;
        m_shared_state->m_flag_operation_enabled_2 = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        m_logger->info("[CAN Master] All Nodes Enabled");
      }
    }
    else
    {
      if (m_shared_state->m_flag_operation_enabled)
      {
        m_shared_state->m_flag_operation_enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        m_logger->info("[CAN Master] At Least One Node Disabled");
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}

/* starts the CANopen event loop and monitor threads; returns false if the
   nodes never boot (the caller keeps the node alive, hardware unavailable) */
bool CTRobot::startRobotCommunication(int sample_time)
{
  m_sampleTime = sample_time;
  m_canThread = std::thread(&CTRobot::startCANopenNodes, this);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // wait until all node tasks are posted (or boot definitively fails)
  while (!m_inrTubeRot || !m_inrTubeTrn || !m_mdlTubeRot || !m_mdlTubeTrn ||
         !m_inrTubeRot->getFlags(Flags::FlagIndex::TASKS_POSTED) ||
         !m_inrTubeTrn->getFlags(Flags::FlagIndex::TASKS_POSTED) ||
         !m_mdlTubeRot->getFlags(Flags::FlagIndex::TASKS_POSTED) ||
         !m_mdlTubeTrn->getFlags(Flags::FlagIndex::TASKS_POSTED))
  {
    if (m_boot_failed)
    {
      m_logger->error("[CAN Master] Robot communication failed to start");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  m_logger->info("[CAN Master] Tasks Posted");
  return true;
}

/* enable operation of all joints
  @param enable: true = enable, false = disable */
void CTRobot::enableOperation(const bool enable)
{
  // blaze::StaticVector<double, 4> temp = blaze::StaticVector<double, 4>(0.0);
  // setTargetVel(temp);
  // getPos(temp);
  // setTargetPos(temp);
  m_inrTubeRot->enableOperation(enable);
  m_inrTubeTrn->enableOperation(enable);
  m_mdlTubeRot->enableOperation(enable);
  m_mdlTubeTrn->enableOperation(enable);
  std::this_thread::sleep_for(std::chrono::milliseconds(300)); // wait for the command to be executed
}

/* template */
bool CTRobot::getSwitchStatus(blaze::StaticVector<bool, 4> &status) const
{
  status[0] = m_inrTubeRot->getStatusword().switched_ON;
  status[1] = m_inrTubeTrn->getStatusword().switched_ON;
  status[2] = m_mdlTubeRot->getStatusword().switched_ON;
  status[3] = m_mdlTubeTrn->getStatusword().switched_ON;
  return status[0] && status[1] && status[2] && status[3];
}

/* overloaded */
bool CTRobot::getSwitchStatus() const
{
  blaze::StaticVector<bool, 4> status;
  return this->getSwitchStatus(status);
}

/* template */
blaze::StaticVector<bool, 4> CTRobot::getEnableStatus() const
{
  blaze::StaticVector<bool, 4> status;
  status[0] = m_inrTubeRot->getStatusword().operation_enabled;
  status[1] = m_inrTubeTrn->getStatusword().operation_enabled;
  status[2] = m_mdlTubeRot->getStatusword().operation_enabled;
  status[3] = m_mdlTubeTrn->getStatusword().operation_enabled;
  return status;
}

/* template */
bool CTRobot::getDisabledStatus(blaze::StaticVector<bool, 4> &status) const
{
  status[0] = !m_inrTubeRot->getStatusword().operation_enabled;
  status[1] = !m_inrTubeTrn->getStatusword().operation_enabled;
  status[2] = !m_mdlTubeRot->getStatusword().operation_enabled;
  status[3] = !m_mdlTubeTrn->getStatusword().operation_enabled;
  return status[0] && status[1] && status[2] && status[3];
}

/* overloaded */
bool CTRobot::getDisabledStatus() const
{
  blaze::StaticVector<bool, 4> status;
  return this->getDisabledStatus(status);
}

/* template */
blaze::StaticVector<bool, 4> CTRobot::getEncoderStatus() const
{
  blaze::StaticVector<bool, 4> status;
  status[0] = m_inrTubeRot->getFlags(Flags::FlagIndex::ENCODER_SET);
  status[1] = m_inrTubeTrn->getFlags(Flags::FlagIndex::ENCODER_SET);
  status[2] = m_mdlTubeRot->getFlags(Flags::FlagIndex::ENCODER_SET);
  status[3] = m_mdlTubeTrn->getFlags(Flags::FlagIndex::ENCODER_SET);
  return status;
}

/* template */
blaze::StaticVector<bool, 4> CTRobot::getReachedStatus() const
{
  blaze::StaticVector<bool, 4> status;
  status[0] = m_inrTubeRot->isReached();
  status[1] = m_inrTubeTrn->isReached();
  status[2] = m_mdlTubeRot->isReached();
  status[3] = m_mdlTubeTrn->isReached();
  return status;
}

/* */
void CTRobot::setEncoders(const blaze::StaticVector<double, 4> val)
{
  m_inrTubeRot->setEncoder(val[0]);
  m_inrTubeTrn->setEncoder(val[1]);
  m_mdlTubeRot->setEncoder(val[2]);
  m_mdlTubeTrn->setEncoder(val[3]);
}

/* set the maximum velocity */
void CTRobot::setMaxVel(const blaze::StaticVector<double, 4UL> &maxVel)
{
  m_maxVel = maxVel;
}

/* set the maximum acceleration */
void CTRobot::setMaxAcc(const blaze::StaticVector<double, 4UL> &maxAcc)
{
  m_maxAcc = maxAcc;
}

/*  */
void CTRobot::setPosLimit(const blaze::StaticVector<double, 4> &min, const blaze::StaticVector<double, 4> &max) const
{
  m_inrTubeRot->setPosLimit(min[0], max[0]);
  m_inrTubeTrn->setPosLimit(min[1], max[1]);
  m_mdlTubeRot->setPosLimit(min[2], max[2]);
  m_mdlTubeTrn->setPosLimit(min[3], max[3]);
}

/* Actuatre robot to the targert absolute joint positions in SI unit with respect to zero position
  operation mode must be set to PositionProfile in advance     */
void CTRobot::setTargetPos(const blaze::StaticVector<double, 4> &target)
{
  // if (this->flag_position_limit)
  // {
  //   if (!(CTRobot::Position_limits_check(target) == 0))
  //   {
  //     return;
  //   }
  // }
  m_inrTubeRot->setPos(target[0]); // in [rad]
  m_inrTubeTrn->setPos(target[1]); // in [m]
  m_mdlTubeRot->setPos(target[2]); // in [rad]
  m_mdlTubeTrn->setPos(target[3]); // in [m]
}

/* Actuatre robot with the targert velocity in SI unit
  operation mode must be set to VelocityProfile in advance*/
void CTRobot::setTargetVel(const blaze::StaticVector<double, 4> &target)
{
  blaze::StaticVector<double, 4> velTarget = target;

  // // Clamp the velTarget values to stay within the bounds
  // for (size_t i = 0; i < velTarget.size(); ++i)
  // {
  //   if (velTarget[i] > velocity_upper_bounds[i])
  //   {
  //     velTarget[i] = velocity_upper_bounds[i];
  //   }
  //   else if (velTarget[i] < velocity_lower_bounds[i])
  //   {
  //     velTarget[i] = velocity_lower_bounds[i];
  //   }
  // }

  // Now set the clamped values

  m_inrTubeRot->setVel(velTarget[0]); // in [rad]
  m_inrTubeTrn->setVel(velTarget[1]); // in [m]
  m_mdlTubeRot->setVel(velTarget[2]); // in [rad]
  m_mdlTubeTrn->setVel(velTarget[3]); // in [m]
}

/* set the limit of the maximum current (or torque) */
void CTRobot::setMaxTorque(const blaze::StaticVector<double, 4UL> negative,
                           const blaze::StaticVector<double, 4UL> positive)
{
  m_inrTubeRot->setMaxTorque(negative[0], positive[0]); // in [rad]
  m_inrTubeTrn->setMaxTorque(negative[1], positive[1]); // in [m]
  m_mdlTubeRot->setMaxTorque(negative[2], positive[2]); // in [rad]
  m_mdlTubeTrn->setMaxTorque(negative[3], positive[3]); // in [m]
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

/* set the parameters of the postion profile (acc, dcc, vel) for PositionProfile mode
  set the parameters of the velocty profile (acc, dcc) for VelocityProfile mode*/
void CTRobot::setProfileParams(const blaze::StaticVector<double, 4UL> max_vel,
                               const blaze::StaticVector<double, 4UL> max_acc,
                               const blaze::StaticVector<double, 4UL> max_dcc)
{
  m_inrTubeRot->setProfileParams(max_acc[0], max_dcc[0], max_vel[0]); // in [rad]
  m_inrTubeTrn->setProfileParams(max_acc[1], max_dcc[1], max_vel[1]); // in [m]
  m_mdlTubeRot->setProfileParams(max_acc[2], max_dcc[2], max_vel[2]); // in [rad]
  m_mdlTubeTrn->setProfileParams(max_acc[3], max_dcc[3], max_vel[3]); // in [m]
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

/*set operation mode for all joints*/
void CTRobot::setOperationMode(const OpMode mode)
{
  if (!(static_cast<int8_t>(mode) == static_cast<int8_t>(m_inrTubeRot->getOperationMode()) &&
        static_cast<int8_t>(mode) == static_cast<int8_t>(m_inrTubeTrn->getOperationMode()) &&
        static_cast<int8_t>(mode) == static_cast<int8_t>(m_mdlTubeRot->getOperationMode()) &&
        static_cast<int8_t>(mode) == static_cast<int8_t>(m_mdlTubeTrn->getOperationMode())))
  {
    m_inrTubeRot->setOperationMode(mode); // in [rad]
    m_inrTubeTrn->setOperationMode(mode); // in [m]
    m_mdlTubeRot->setOperationMode(mode); // in [rad]
    m_mdlTubeTrn->setOperationMode(mode); // in [m]
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  else
    m_logger->debug("[CAN Master] Requeset operation mode is already set");
}

/* Gets the current in [mA] unit */
blaze::StaticVector<double, 4> CTRobot::getCurrent() const
{
  blaze::StaticVector<double, 4> val;
  this->m_inrTubeRot->getCurrentAvg(val[0]);
  this->m_inrTubeTrn->getCurrentAvg(val[1]);
  this->m_mdlTubeRot->getCurrentAvg(val[2]);
  this->m_mdlTubeTrn->getCurrentAvg(val[3]);
  return val;
}

/* Gets the current velocity of all actuators in SI unit */
void CTRobot::getVel(blaze::StaticVector<double, 4> &val) const
{
  this->m_inrTubeRot->getVel(val[0]);
  this->m_inrTubeTrn->getVel(val[1]);
  this->m_mdlTubeRot->getVel(val[2]);
  this->m_mdlTubeTrn->getVel(val[3]);
}

/* Gets the current absolute position of all actuators in SI unit */
void CTRobot::getPos(blaze::StaticVector<double, 4> &val) const
{
  this->m_inrTubeRot->getPos(val[0]);
  this->m_inrTubeTrn->getPos(val[1]);
  this->m_mdlTubeRot->getPos(val[2]);
  this->m_mdlTubeTrn->getPos(val[3]);
}

/* Gets the current absolute position of all actuators in SI unit */
void CTRobot::getPosLimit(blaze::StaticVector<double, 4> &min, blaze::StaticVector<double, 4> &max) const
{
  this->m_inrTubeRot->getPosLimit(min[0], max[0]);
  this->m_inrTubeTrn->getPosLimit(min[1], max[1]);
  this->m_mdlTubeRot->getPosLimit(min[2], max[2]);
  this->m_mdlTubeTrn->getPosLimit(min[3], max[3]);
}

/* Gets the current absolute position of all actuators in SI unit */
void CTRobot::getTemperature(blaze::StaticVector<int32_t, 4> &cpu, blaze::StaticVector<int32_t, 4> &winding) const
{
  cpu[0] = this->m_inrTubeRot->getCpuTemp();
  cpu[1] = this->m_inrTubeTrn->getCpuTemp();
  cpu[2] = this->m_mdlTubeRot->getCpuTemp();
  cpu[3] = this->m_mdlTubeTrn->getCpuTemp();

  winding[0] = this->m_inrTubeRot->getWindingTemp();
  winding[1] = this->m_inrTubeTrn->getWindingTemp();
  winding[2] = this->m_mdlTubeRot->getWindingTemp();
  winding[3] = this->m_mdlTubeTrn->getWindingTemp();
}

// get digital inputs
void CTRobot::getDigitalIn(blaze::StaticVector<std::bitset<32>, 4> &in) const
{
  in[0] = this->m_inrTubeRot->getDigitalIn();
  in[1] = this->m_inrTubeTrn->getDigitalIn();
  in[2] = this->m_mdlTubeRot->getDigitalIn();
  in[3] = this->m_mdlTubeTrn->getDigitalIn();
}

// to be delveloped
void CTRobot::getInterface() const
{
  std::bitset<32> d_in_0 = this->m_inrTubeRot->getDigitalIn();
  std::bitset<32> d_in_1 = this->m_inrTubeTrn->getDigitalIn();
  std::bitset<32> d_in_2 = this->m_mdlTubeRot->getDigitalIn();
  std::bitset<32> d_in_3 = this->m_mdlTubeTrn->getDigitalIn();

  m_input->setKeyUp(d_in_0[17]);
  m_input->setKeyDown(d_in_0[18]);
  m_input->setKeyRight(d_in_1[17]);
  m_input->setKeyLeft(d_in_1[18]);
  m_input->setKeyForward(d_in_2[17]);
  m_input->setKeyBackward(d_in_2[18]);
  m_input->setKeyCenter(d_in_3[17]);

  // m_input->setLED1(true);
}

// /* Actuatre robot to the targert absolute joint positions in SI unit
//   (with respect to zero position (distal limit) - positive value is towards proximal end)     */
// void CTRobot::findTransEncoders()
// {
//   blaze::StaticVector<double, 4UL> negative = {400.0, 220.0, 400.0, 220.0};
//   blaze::StaticVector<double, 4UL> positive = {400.0, 220.0, 400.0, 220.0};

//   blaze::StaticVector<double, 4UL> max_dcc = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s^2] and [mm/s^2]
//   blaze::StaticVector<double, 4UL> max_vel = {200.00 * M_PI / 180.00, 10.00 / 1000.00, 200.00 * M_PI / 180.00, 10.00 / 1000.00}; // [deg/s] and [mm/s]
//   blaze::StaticVector<double, 4UL> max_acc = 0.1 * max_dcc;
//   double current = 0.0;
//   double pos1, pos0, pos2 = 0.0;
//   double collet_minimum_clearance = 0.040; // *********** needs to be adjusted ***********

//   CTRobot::setOperationMode(OpMode::VelocityProfile);
//   CTRobot::setMaxTorque(negative, positive);
//   CTRobot::setProfileParams(max_vel, max_acc, max_dcc);

//   this->m_inrTubeTrn->getPos(pos1);

//   CTRobot::setTargetVel({-0.0, -0.003, -0.0, -0.003});

//   this->m_inrTubeTrn->getCurrentAvg(current);
//   CTRobot::enableOperation(true);
//   std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   while (current > -210.0)
//   {
//     std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     this->m_inrTubeTrn->getCurrentAvg(current);
//   }
//   m_logger->info("[CAN Master] inner carriage hit back");
//   CTRobot::setTargetVel({0.0, 0.0, 0.0, 0.0});
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   this->m_inrTubeTrn->getPos(pos0);

//   // this->m_inner_tran->set_encoder(0.0);
//   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//   // CTRobot::Enable_Operation(true);

//   // std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   CTRobot::setTargetVel({0.0, 0.003, 0.0, 0.0});
//   std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   this->m_inrTubeTrn->getCurrentAvg(current);

//   m_logger->info("[CAN Master] checkpoint_3");
//   while (current < 210.0)
//   {
//     std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     this->m_inrTubeTrn->getCurrentAvg(current);
//   }
//   m_logger->info("[CAN Master] inner carriage hit middle carriage");
//   CTRobot::setTargetVel({1.0, 0.0, 1.0, 0.0});
//   std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//   CTRobot::setTargetVel({0.0, 0.0, 0.0, 0.0});
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   this->m_inrTubeTrn->getPos(pos2);
//   this->m_inrTubeTrn->setEncoder(pos2 - pos0);
//   this->m_mdlTubeTrn->setEncoder(pos2 - pos0 + collet_minimum_clearance);
//   std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//   m_logger->info("[CAN Master] encoders found");
//   CTRobot::enableOperation(false);
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));
// }

/* Gets the current absolute position (with respect to zero position) of all actuators in [mm] or [deg] unit */
void CTRobot::convPosToRobotFrame(const blaze::StaticVector<double, 4> &posCurrent,
                                  blaze::StaticVector<double, 4> &posInCTRFrame) const
{
  for (size_t i = 0; i < posCurrent.size(); ++i)
  {
    posInCTRFrame[i] = posCurrent[i] + this->m_posOffsets[i];
  }
}

/**/
int CTRobot::checkPosLimits(const blaze::StaticVector<double, 4> &posTarget) const
{
  for (size_t i = 0; i < posTarget.size(); ++i)
  {
    if (posTarget[i] < m_lowerBounds[i])
    {
      std::cout << "static lower bound reached" << std::endl;
      return -1;
    }
    if (posTarget[i] > m_upperBounds[i])
    {
      std::cout << "static upper bound reached" << std::endl;
      return -1;
    }
  }
  blaze::StaticVector<double, 4> posInCTRFrame = blaze::StaticVector<double, 4>(0.0);

  this->convPosToRobotFrame(posTarget, posInCTRFrame);
  if ((posInCTRFrame[3] - posInCTRFrame[1]) < m_minClearance)
  {
    std::cout << "dynamic position lower limit reached - preventing collision => position target ignored" << std::endl;
    return -1;
  }
  if ((posInCTRFrame[3] - posInCTRFrame[1]) > m_maxClearance)
  {
    std::cout << "dynamic position upper limit reached - preventing illegan tube configuration => position target ignored" << std::endl;
    return -1;
  }
  return 0;
}

/* variable wait untill current position is reached to target position */
void CTRobot::waitUntilReach(const std::atomic<bool> &cancel_flag) const
{
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  blaze::StaticVector<bool, 4> status = getReachedStatus();
  while (!status[0] || !status[1] || !status[2] || !status[3])
  {
    if (cancel_flag.load())
    {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    status = getReachedStatus();
  }
}

/* variable wait untill current position is reached to target position */
void CTRobot::waitUntilReach() const
{
  std::atomic<bool> cancel_flag = false;
  waitUntilReach(cancel_flag);
}

/* variable wait untill current position is reached to target position */
void CTRobot::waitUntilTransReach(const std::atomic<bool> &cancel_flag) const
{
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (!m_inrTubeTrn->isReached() || !m_mdlTubeTrn->isReached())
  {
    if (cancel_flag.load())
    {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

/* variable wait untill current position is reached to target position */
void CTRobot::waitUntilTransReach() const
{
  std::atomic<bool> cancel_flag = false;
  waitUntilTransReach(cancel_flag);
}

/* Get the current date and time and format the filename using the date and time  */
void CTRobot::initLogger()
{
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  struct tm tm;
  localtime_r(&time, &tm);
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &tm);
  std::string filename = std::string(buffer) + ".txt"; // Use the formatted time as the filename
  std::filesystem::create_directories(m_paths.log_dir);
  std::string logDir = m_paths.log_dir + filename;

  /* instansiate the logger object */
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logDir, true);
  console_sink->set_level(spdlog::level::info);                                                               // Only print warnings and above to console
  file_sink->set_level(spdlog::level::debug);                                                                 // Log all messages to the file
  this->m_logger = std::make_shared<spdlog::logger>("CTR", spdlog::sinks_init_list{console_sink, file_sink}); // instansiate the logger
  this->m_logger->set_level(spdlog::level::debug);                                                            // This enables debug messages to be processed
  this->m_logger->flush_on(spdlog::level::debug);                                                             // Flush immediately on debug level
}

//==========================================================================================================
// 					 This code is a part of the robot actuated catheter project
//  		   		Canadian Surgical Technologies and Advanced Robotics (CSTAR).
// 							Copyright (C) 2022 Navid Feizi <nfeizi@uwo.ca>
//					     Copyright (C) 2022 Filipe C. Pedrosa <fpedrosa@uwo.ca>
//
//       Project developed under the supervision of Dr Jayender Jagadeesan (❖) Dr Rajni Patel (◈)
//  ========================================================================================================
//   (◈) CSTAR (Canadian Surgical Technologies & Advanced Robotics) @ Western University, London, ON  Canada
//             (❖) Surgical Planning Lab @ Brigham and Women's Hospital / Harvard Medical School
//==========================================================================================================

/*
  This class is responsible for creating nodes that facilitate communication with motion controllers (Faulhaber
  and Maxon) using the CANopen protocol. It supports the CiA301 operation layer, and CiA402 device profile .
*/

#include "CiA301node.hpp"

using namespace std::chrono_literals;
using namespace lely;

extern std::unordered_map<std::string, uint8_t> faulhaberComCodeDictionary;

/** Overloaded Constructor
 * @brief FiberDriver base class with master and id parameters
 *
 * @param parent Pointer to the parent robot class. Used for log record and some flag check.
 * @param exec Pointer to an event executor (ev_exec_t) for asynchronous operations.
 * @param master A reference to a CANopen AsyncMaster, which is responsible for communication.
 * @param NodeID The unique node ID (1-256) of the motion controller.
 * @param EncoderResolution Pulse per SI position unit.
 * @param GearRatio Gear ratio if not set in the motion controller factors.
 * @param VelocityFactor The velocity factor multiplier set in the motion controller factors / 60.
 * @param SampleTime An unsigned integer specifying the command execution period in [ms] for this Node.
 * @param OperationMode An int that defines the mode of operation (position profile, velocity profile, etc.).
 * @param ProfileAcc Profile position or profile velocity acceleration in SI units.
 * @param ProfileVel Profile position velocity in SI units.
 */
Cia301Node::Cia301Node(ev_exec_t * /*exec*/,
                       canopen::AsyncMaster &master,
                       unsigned int NodeID,
                       std::string ControllerBrand,
                       double EncoderResolution,
                       double GearRatio,
                       double VelocityFactor,
                       unsigned int SampleTime,
                       OpMode OperationMode,
                       double ProfileAccSI,
                       double ProfileVelSI,
                       //    double CurrentThreshold,
                       //    double VelFindLimit,
                       std::shared_ptr<SharedState> sharedState,
                       std::shared_ptr<spdlog::logger> shared_logger)
    : FiberDriver(master, NodeID), robot_states(sharedState), logger(shared_logger)
{
    this->m_controller_brand = ControllerBrand;
    this->m_sampleTime = SampleTime;
    this->m_operation_mode = OperationMode;
    this->m_gearRatio = GearRatio;
    this->m_ppu = (EncoderResolution / GearRatio);
    // this->m_current_threshold = CurrentThreshold;
    // this->m_vel_findlimit = VelFindLimit;
    this->m_nodeId = std::to_string(NodeID);

    m_flags.set(Flags::FlagIndex::BOOT_SUCCESS, false);
    m_flags.set(Flags::FlagIndex::TASKS_POSTED, false);
    m_flags.set(Flags::FlagIndex::ENCODER_SET, false);
    m_flags.set(Flags::FlagIndex::NEW_TARG_READY, false);
    m_flags.set(Flags::FlagIndex::ENCODER_MEM_READY, false);

    // check motion controller brand and convert profile position parameters
    if (m_controller_brand == "Maxon" || m_controller_brand == "Faulhaber")
    {
        m_conversionFactor = (m_controller_brand == "Maxon") ? (60.0 / m_gearRatio) : (m_ppu / VelocityFactor);
        m_profile_vel = static_cast<unsigned int>(abs(ProfileVelSI * m_conversionFactor));
        m_profile_acc = static_cast<unsigned int>(abs(ProfileAccSI * m_conversionFactor));
    }
    else
    {
        logger->critical("[Node " + m_nodeId + "] Unknown motion controller brand | \"Maxon\", and \"Faulhaber\" are supported.");
    }

    logger->debug("[Node " + m_nodeId + "] " +
                  "Controller brand: \"" + m_controller_brand + "\" - " +
                  "Operation layer: " + " CiA301 - " +
                  "PPU: " + std::to_string(m_ppu) + " - " +
                  "Gear ratio: " + std::to_string(GearRatio) + " - " +
                  "Encoder resoluton: " + std::to_string(EncoderResolution));
}

Cia301Node::~Cia301Node()
{
    // if (m_encoder_memory_file.is_open())
    //     m_encoder_memory_file.close();
}

// ============================== CANopen Communication Methods ==============================
/* This function gets called during the boot-up process for the node. The
    'res' parameter is the function that MUST be invoked when the configuration
    is complete. Because this function runs as a task inside a coroutine, it
    can suspend itself and wait for an asynchronous function, such as an SDO
    request, to complete. */
void Cia301Node::OnConfig(std::function<void(std::error_code ec)> res) noexcept
{
    logger->debug("[Node " + m_nodeId + "] Configing");
    Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
    try
    {
        res({}); // Indicate successful completion of the configuration by invoking the 'res' function.
    }
    catch (canopen::SdoError &e)
    {
        // If one of the SDO requests resulted in an error, abort the configuration
        // and report the error code using the 'res' function.
        logger->debug("[Node " + m_nodeId + "] Error Configing");
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        res(e.code());
    }
}

/* This function gets called when the boot-up process of the node completes.
    The 'st' parameter contains the last known NMT state of the slave
    (typically pre-operational), 'es' the error code (0 on success), and 'what'
    a description of the error, if any.*/
void Cia301Node::OnBoot(canopen::NmtState /*st*/, char es, const std::string &what) noexcept
{
    if (!es || es == 'L')
    {
        // Successful boot-up or boot-up is in progress ("L" state).
        m_flags.set(Flags::FlagIndex::BOOT_SUCCESS, true);
        logger->debug("[Node " + m_nodeId + "] Booted successfully.");

        while (!robot_states->m_boot_success)                         // wait for all other nodes to boot
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for all other nodes to boot

        Cia301Node::ResetFault_();                                    // reset faults if there is any
        Cia301Node::SwitchOn();                                       // switch ON
        while (!robot_states->m_flag_robot_switched_on)               // wait for other nodes to switch ON
            Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for other nodes to switch ON
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));    // wait for better log

        Cia301Node::SetOperationMode_(m_operation_mode);                            // set mode of operation
        Cia301Node::SetProfileParams_(m_profile_acc, m_profile_acc, m_profile_vel); // set profile parameters
        Wait(AsyncWait(duration(std::chrono::milliseconds(200))));                  // wait for for a clean log file

        Cia301Node::SetTpdoTranstype_(1, 1);                      // set tPDO1 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file
        Cia301Node::SetTpdoTranstype_(2, 1);                      // set tPDO2 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file
        Cia301Node::SetTpdoTranstype_(4, 1);                      // set tPDO4 transmission type to trigger with SYNC
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for for a clean log file

        if (m_useEncoderMemory)
            this->ReadEncoderMemFile(EnoderStoreFiles_directory, m_encoderMem, m_encoderMemFile); // read or create the memory file
        Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                                // this wait is necessary to give time opening memory files
        m_flags.set(Flags::FlagIndex::ENCODER_MEM_READY, true);

        // Node::Find_Fowrard_Limits();
        // CiA301Node::SetEncoder(m_encoder_memory_value);               // set encoder value
        // CiA301Node::EnableOperation(true);                            // enable operation before posting the tasks
        // while (!robot_states->m_flag_operation_enabled)               // wait for other nodes to enabled
        //     Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // wait for other nodes to enabled
        // Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));   // wait for a clean log
        // m_flags.set(Flags::FlagIndex::NEW_TARG_READY, true);

        Post(&Cia301Node::TaskTarget, this);
        // Post(&Cia301Node::TaskConfig, this);
        // Post(&Cia301Node::TaskOperation, this);
        m_flags.set(Flags::FlagIndex::TASKS_POSTED, true);
        logger->debug("[Node " + m_nodeId + "] Task posted");
    }
    else
    {
        logger->critical("[Node " + m_nodeId + "] Failed to boot: " + what);
    }
}

/* This function is similar to OnConfg(), but it gets called by the
    AsyncDeconfig() method of the master. */
void Cia301Node::OnDeconfig(std::function<void(std::error_code ec)> res) noexcept
{
    logger->debug("[Node " + m_nodeId + "] Deconfiging");
    try
    {
        m_controlWord.enable_voltage = 1;
        m_controlWord.quick_stop = 0;                                            // set quick stop
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get()));    // write on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));                // wait for safety
        m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word
        // if (m_encoder_memory_file.is_open())                                      // close encoder memory file
        //     m_encoder_memory_file.close();
        logger->debug("[Node " + m_nodeId + "] " + m_statusWord.getCiA402StatusMessage());
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        res({});
    }
    catch (canopen::SdoError &e)
    {
        logger->error("[Node " + m_nodeId + "] Error Deconfiging");
        res(e.code());
    }
}

/* This function gets called every time a value is written to the local object
    dictionary of the master by an RPDO (or SDO, but that is unlikely for a
    master), *and* the object has a known mapping to an object on the slave for
    which this class is the driver. The 'idx' and 'subidx' parameters are the
    object index and sub-index of the object on the slave, not the local object
    dictionary of the master. */
void Cia301Node::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
    // if RxPDO 1 received ---- info in RxPDO1 -> [1] Status Word
    if (idx == STATUS_WORD_IDX && subidx == 0)
    {
        m_statusWord.update(rpdo_mapped[STATUS_WORD_IDX][0]); // update status word

        // check the target position acknowledge bit (12-stw) => setpoint is acknowlesged by the motioncontroller
        // switch off the new target bit (4-ctrlw) if acknowledged
        if (m_current_operation_mode == OpMode::PositionProfile)
        {
            if (m_statusWord.bit12 && !m_bit12Prev) // rising edge of bit12 (set point acknowledge)
            {
                m_controlWord.bit4 = 0;                                 // set new target bit (bit 4)
                tpdo_mapped[CONTROL_WORD_IDX][0] = m_controlWord.get(); // unset new target bit
                tpdo_mapped[CONTROL_WORD_IDX][0].WriteEvent();          // Trigger write events for PDO1.
                m_bit12Prev = m_statusWord.bit12;
                if (m_printPdos)
                {
                    std::cout << "=> tPDO1 [" << m_nodeId << "]: Control: 0b" << ToBinaryString(m_controlWord.get()) << std::endl;
                    std::cout << "         [" << m_nodeId << "]:[" << std::hex << CONTROL_WORD_IDX << "][" << static_cast<int>(0) << "]: 0x" << std::hex << m_controlWord.get() << std::endl;
                }
            }
            else if (!m_statusWord.bit12 && m_bit12Prev) // falling edge of bit12 (set point acknowledge)
            {
                m_flags.set(Flags::FlagIndex::NEW_TARG_READY, true);
                m_bit12Prev = m_statusWord.bit12;
            }
        }
    }
    // if RxPDO 1 received ---- info in RxPDO1 -> [2] Actual Positon
    if (idx == ACTUAL_POSITION_IDX && subidx == 0)
    {
        m_currentPos = rpdo_mapped[ACTUAL_POSITION_IDX][0];    // update actual position
        m_currentPosSi = double(m_currentPos) / double(m_ppu); // convert to SI unit

        // store the actual posotion in encoder memory
        if (m_useEncoderMemory && m_statusWord.switched_ON && m_flags.get(Flags::FlagIndex::ENCODER_MEM_READY))
        {
            if (m_encoderMemFile.is_open())
            {
                m_encoderMemFile.seekp(0); // Move the write pointer to the beginning
                double pval;
                Cia301Node::getPos(pval);
                m_encoderMemFile << std::fixed << std::setprecision(12) << pval;
            }
            else
            {
                logger->critical("[Node " + m_nodeId + "] Unable to open encoder memory file to write......");
            }
        }
    }

    if (idx == ACTUAL_VELOCITY_FAULHABER_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Actual Velocity
    {
        m_currentVel = rpdo_mapped[ACTUAL_VELOCITY_FAULHABER_IDX][0]; // Update actual velocity
        m_currentVelSi = double(m_currentVel) / m_conversionFactor;   // Update actual velocity in SI units. It differes based on Brand
    }
    if (idx == ACTUAL_TORQUE_FAULHABER_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Torque
    {
        m_current = rpdo_mapped[ACTUAL_TORQUE_FAULHABER_IDX][0];          // Update torque
        m_currentSi = double(m_current) * m_statusWord.operation_enabled; // no cunvertion

        m_currentSiHist.push_back(m_currentSi);
        if (m_currentSiHist.size() > 10)
        {
            m_currentSiHist.pop_front();
        }
    }
    if (idx == ACTUAL_VELOCITY_MAXON_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Actual Velocity
    {
        m_currentVel = rpdo_mapped[ACTUAL_VELOCITY_MAXON_IDX][0];   // Update actual velocity
        m_currentVelSi = double(m_currentVel) / m_conversionFactor; // Update actual velocity in SI units. It differes based on Brand
    }
    if (idx == ACTUAL_CURRENT_MAXON_IDX && subidx == 0) // if RxPDO 2 received ---- info in RxPDO2 -> [1] Current
    {
        m_current = rpdo_mapped[ACTUAL_CURRENT_MAXON_IDX][0];                    // Update current or torque
        m_currentSi = double(m_current) / 1000 * m_statusWord.operation_enabled; // Convert to SI unit

        m_currentSiHist.push_back(m_currentSi);
        if (m_currentSiHist.size() > 10)
        {
            m_currentSiHist.pop_front();
        }
    }

    // if RxPDO 4 received
    if (idx == POSITION_LIMIT && subidx == 0x01) // if RxPDO 2 received ---- info in RxPDO4 -> [1] min positon limit
    {
        m_currentPosLimit[0] = rpdo_mapped[POSITION_LIMIT][0x01];              // Update current min position limit
        m_currentPosLimitSi[0] = double(m_currentPosLimit[0]) / double(m_ppu); // convert to SI unit
    }
    if (idx == POSITION_LIMIT && subidx == 0x02) // if RxPDO 2 received ---- info in RxPDO4 -> [1] max position
    {
        m_currentPosLimit[1] = rpdo_mapped[POSITION_LIMIT][0x02];              // Update current max position limit
        m_currentPosLimitSi[1] = double(m_currentPosLimit[1]) / double(m_ppu); // convert to SI unit
    }

    // you can add more PDOs here to be recognized
    // if Received unknown PDO
    if (!((idx == ACTUAL_TORQUE_FAULHABER_IDX) || (idx == ACTUAL_CURRENT_MAXON_IDX) || (idx == ACTUAL_VELOCITY_FAULHABER_IDX) || (idx == ACTUAL_VELOCITY_MAXON_IDX) || (idx == STATUS_WORD_IDX) || (idx == ACTUAL_POSITION_IDX) || (idx == POSITION_LIMIT)))
    {
        std::stringstream ss;
        ss << "[Node " + m_nodeId << ": unknown PDO - idx: 0x" << std::hex << idx << " subidx: 0x" << std::hex << subidx; // Convert to hex, uppercase letters
        logger->warn(ss.str());
    }

    if (m_printPdos)
    {
        std::cout << "<= rPDO1 [" << m_nodeId << "]: pos: " << std::dec << m_currentPos << "  \tStatus: 0b" << ToBinaryString(m_statusWord.statusword) << std::endl;
        std::cout << "         [" << m_nodeId << "]:[" << std::hex << 0x6064 << "][" << static_cast<int>(0) << "]: " << std::dec << m_currentPos << std::endl;
        std::cout << "         [" << m_nodeId << "]:[" << std::hex << 0x6041 << "][" << static_cast<int>(0) << "]: " << std::hex << "0x" << m_statusWord.statusword << std::endl;
    }
    if (m_printPdos)
    {
        std::cout << "<= rPDO2 [" << m_nodeId << "]: Vel: " << std::dec << m_currentVel << "  \tToruqe: " << m_current << std::endl;
        std::cout << "         [" << m_nodeId << "]:[" << std::hex << 0x6077 << "][" << static_cast<int>(0) << "]: " << std::dec << m_current << std::endl;
        std::cout << "         [" << m_nodeId << "]:[" << std::hex << 0x606c << "][" << static_cast<int>(0) << "]: " << std::dec << m_currentVel << std::endl;
    }
}

/* */
void Cia301Node::OnSync(uint8_t cnt, const time_point &t) noexcept
{
    (void)cnt;
    (void)t;
    // Send the scaled_position_command to the slave's object dictionary entry 0x6064.
    try
    {
    }
    catch (const canopen::SdoError &e)
    {
        // Handle SDO write error
        std::cerr << "SDO write error: " << e.what() << std::endl;
    }
}

// ============================== CiA402 State Machine Methods ==============================
/* A member function to switch on the driver. Supports CiA402 State Machine Application Layer.
    The Switch_ON function should be called in the OnBoot callback function.*/
void Cia301Node::SwitchOn()
{
    std::string message;
    m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get statusword on SDO
    message = m_statusWord.getCiA402StatusMessage();
    if (m_statusWord.Switch_On_Disabled)
    {
        m_controlWord.switch_ON = 0;
        m_controlWord.enable_voltage = 1;
        m_controlWord.quick_stop = 1;
        m_controlWord.enable_operation = 0;
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get())); // shut down (switch on disable)
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
        m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
        message = message + "  =>  " + m_statusWord.getCiA402StatusMessage();
    }
    if ((m_statusWord.ready_to_switch_ON && !m_statusWord.switched_ON) ||
        m_statusWord.operation_enabled)
    {
        m_controlWord.switch_ON = 1;
        m_controlWord.enable_voltage = 1;
        m_controlWord.quick_stop = 1;
        m_controlWord.enable_operation = 0;
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get())); // switch on or disable operation
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
        m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000)));
        message = message + "  =>  " + m_statusWord.getCiA402StatusMessage();
    }

    if (m_statusWord.getCiA402StatusMessage() == "Switched On")
    {
        logger->debug("[Node " + m_nodeId + "] " + message);
    }
    else
    {
        logger->error("[Node " + m_nodeId + "] " + message);
    }
}

/** A member function to enabled / disable the driver operation
    supports CiA402 Application Layer over SDO
    @param enable: true = enable, false = disable*/
void Cia301Node::EnableOp_(const bool enable)
{
    logger->debug("[Node " + m_nodeId + "] " + "requested to switch operation to : " + std::to_string(enable));

    int max_attempts = 10;
    int attempt_count = 0;

    m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO)

    // enable operation
    if (enable && !m_statusWord.operation_enabled)
    {
        while (!m_statusWord.operation_enabled)
        {
            if (attempt_count >= max_attempts) // If the loop has run 10 times without enabling the operation, return an error
            {
                logger->error("[Node " + m_nodeId + "] Operation not enabled after " + std::to_string(max_attempts) + " attempts - current status: " + m_statusWord.getCiA402StatusMessage());
                throw std::runtime_error("Error: Operation not enabled after 10 attempts");
            }
            Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x000F));                 // set the state macine to enabled operation
            m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
            attempt_count++;
        }
        logger->debug("[Node " + m_nodeId + "] " + m_statusWord.getCiA402StatusMessage());
    }
    // switch on or disable operation
    else if (!enable && m_statusWord.operation_enabled)
    {
        while (m_statusWord.operation_enabled)
        {
            if (attempt_count >= max_attempts) // If the loop has run 10 times without enabling the operation, return an error
            {
                logger->critical("[Node " + m_nodeId + "] Operation not disabled after " + std::to_string(max_attempts) + " attempts - current status: " + m_statusWord.getCiA402StatusMessage());
            }
            Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, 0x0007));                 // set the state macine to switched on
            m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
            attempt_count++;
        }
        logger->debug("[Node " + m_nodeId + "] " + m_statusWord.getCiA402StatusMessage());
        double temp = 0.0;
        setVel(temp);
        getPos(temp);
        setPos(temp);
    }
    else if (enable && m_statusWord.operation_enabled)
    {
        logger->debug("[Node " + m_nodeId + "] request to enable - alraedy enbaled");
    }
    else
    {
        logger->debug("[Node " + m_nodeId + "] request to disable - alraedy disabled");
    }
}

/** A member function to enabled / disable the driver operation
    supports CiA402 Application Layer over SDO
    @param enable: true = enable, false = disable*/
// ********************** this function is under development **********************
void Cia301Node::EnableOperationWithPdo_(const bool enable)
{
    // (void) enable;
    // using TPDO1
    m_controlWord.enable_operation = enable;
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_controlWord.get(); // enable
    tpdo_mapped[CONTROL_WORD_IDX][0].WriteEvent();
    // flag_operation_enabled = true;

    // m_control_word.enable_operation = enable;
    // tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // disbale
    // tpdo_mapped[CONTROL_WORD_IDX][0].WriteEvent();
    // flag_operation_enabled = false;
}

/* A member funtion to reset faults using SDO
    should be called in oOnConfig*/
void Cia301Node::ResetFault_()
{
    m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // update status word
    if (m_statusWord.fault)
    {
        // logger->warn("[Node " + node_id + "] fault code" + std::to_string(Wait(AsyncRead<uint16_t>(0x2321, 0x0000))), true); // update status word

        m_controlWord.fault_reset = 0;                                           // make the bit zero to rise it later for reserting fault
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get()));    // set the controlword on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                // wait for safety
        m_controlWord.fault_reset = 1;                                           // fault resets at rising edge
        Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get()));    // set the controlword on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                // wait for safety
        m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
        logger->warn("[Node " + m_nodeId + "] Reset fault => Fault status: " + std::to_string(m_statusWord.fault));
    }
    else
    {
        logger->debug("[Node " + m_nodeId + "] No fault to reset");
    }
}

/*  This task is reponsible for cyclic sending target positions or target velocities through PDO2 / PDO3
    This task is posted in in the OnBoot function */
void Cia301Node::TaskTarget() noexcept
{
    while (true)
    {
        // send position/velocity target
        if (robot_states->m_flag_operation_enabled & !m_isConfiguring)
        {
            m_flag_target_task_processing = true;
            if (m_current_operation_mode == OpMode::PositionProfile) // Position Profile mode
            {
                m_controlWord.bit4 = 1; // set new target bit (bit 4)
                m_controlWord.bit5 = 1; // set immediately bit (bit 5)
                if (m_flags.get(Flags::FlagIndex::NEW_TARG_READY))
                {
                    if (m_targetPos != m_targetPosPrev)
                    {
                        tpdo_mapped[TARGET_POSITION_IDX][0] = m_targetPos; // target position
                        tpdo_mapped[CONTROL_WORD_IDX][0] = m_controlWord.get();
                        tpdo_mapped[TARGET_POSITION_IDX][0].WriteEvent(); // Trigger write events for PDO1.
                        m_targetPosPrev = m_targetPos;
                        m_flags.set(Flags::FlagIndex::NEW_TARG_READY, false);
                        m_statusWord.bit10 = 0;

                        if (m_printPdos)
                        {
                            std::cout << "=> tPDO2 [" << m_nodeId << "]: target: " << std::dec << m_targetPos << std::endl;
                            std::cout << "         [" << m_nodeId << "]:[" << std::hex << CONTROL_WORD_IDX << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(m_controlWord.get()) << std::hex << "   (0x" << m_controlWord.get() << ")" << std::endl;
                        }
                    }
                }
                else
                {
                    // logger->error("[Node " + m_nodeId + "] Motion controller is not ready to receive a new position target! (PDO is sending too fast - please wait!)");
                }
            }
            // velocity profile mode is not verified - may need some improvement
            else if (m_current_operation_mode == OpMode::VelocityProfile) // Velocity Profile mode
            {
                m_controlWord.enable_operation = 1;                     // set immediately bit (bit 5)
                tpdo_mapped[TARGET_VELOCITY_IDX][0] = m_targetVel;      // target velocity
                tpdo_mapped[CONTROL_WORD_IDX][0] = m_controlWord.get(); // enable
                tpdo_mapped[TARGET_VELOCITY_IDX][0].WriteEvent();       // Trigger write events for PDO1.
                                                                        // std::cout << "target vel: " << vel << std::endl;
                                                                        // if (flag_printlog)
                                                                        // {
                                                                        //     std::cout << "=> tPDO2 [" << node_id << "]: target: " << std::dec << vel << std::endl;
                                                                        //     std::cout << "         [" << node_id << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(ctrl) << std::hex << "   (0x" << ctrl << ")" << std::endl;
                                                                        // }
            }
            // else if (m_current_operation_mode == OpMode::Velocity) // Velocity mode
            // {
            //     // shared_state->Log_message("[Node " + node_id +
            //     //                           ": sending velocity mode setting = " + std::to_string(m_target_vel),
            //     //                       true);
            //     m_control_word.enable_operation = 1;                     // set immediately bit (bit 5)
            //     tpdo_mapped[0x206B][0] = m_target_vel;                   // target velocity
            //     tpdo_mapped[CONTROL_WORD_IDX][0] = m_control_word.get(); // enable
            //     tpdo_mapped[0x206B][0].WriteEvent();                     // Trigger write events for PDO1.
            //                                                              // std::cout << "target vel: " << vel << std::endl;
            //                                                              // if (flag_printlog)
            //                                                              // {
            //                                                              //     std::cout << "=> tPDO2 [" << node_id << "]: target: " << std::dec << vel << std::endl;
            //                                                              //     std::cout << "         [" << node_id << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(ctrl) << std::hex << "   (0x" << ctrl << ")" << std::endl;
            //                                                              // }
            // }
            m_flag_target_task_processing = false;
        }

        else if (m_isConfiguring)
        {
            while (m_flag_target_task_processing)
                Wait(AsyncWait(duration(std::chrono::microseconds(5))));
            if (m_commandMsg == "enable")
            {
                Cia301Node::EnableOp_(true);
            }
            else if (m_commandMsg == "disable")
            {
                Cia301Node::EnableOp_(false);
            }
            else if (m_commandMsg == "set_max_torque")
            {
                Cia301Node::SetMaxTorque_(m_set_max_torque[0], m_set_max_torque[1]);
            }
            else if (m_commandMsg == "set_profile_params")
            {
                Cia301Node::SetProfileParams_(m_set_profile_acc, m_set_profile_dcc, m_set_profile_vel); // set profile parameters
            }
            else if (m_commandMsg == "set_encoder")
            {
                Cia301Node::SetEncoder_(m_set_encoder);
            }
            else if (m_commandMsg == "set_operation_mode")
            {
                Cia301Node::SetOperationMode_(m_set_operation_mode);
            }
            m_commandMsg = "";

            // Wait(AsyncWait(duration(std::chrono::microseconds(m_sampleTime))));
            m_isConfiguring = false;
        }

        else
        {
            // logger->debug("[Node " + m_node_id + "] Robot is not operational");
        }

        // PDO4
        tpdo_mapped[POSITION_LIMIT][0x01] = m_PosLimit[0]; // min position limit
        tpdo_mapped[POSITION_LIMIT][0x02] = m_PosLimit[1]; // max position limit
        tpdo_mapped[POSITION_LIMIT][0x01].WriteEvent();    // Trigger write events for PDO4.

        Wait(AsyncWait(duration(std::chrono::milliseconds(m_sampleTime - 2))));
    }
}

/*  This task is reponsible for changing controller configuration.
    "Task Target" should be halted using flag_config when this task is actioning*/
void Cia301Node::TaskConfig() noexcept
{
    while (true)
    {
        if (m_isConfiguring)
        {
            if (m_commandMsg == "set_max_torque")
            {
                Cia301Node::SetMaxTorque_(m_set_max_torque[0], m_set_max_torque[1]);
                m_commandMsg = "";
            }
            else if (m_commandMsg == "set_profile_params")
            {
                Cia301Node::SetProfileParams_(m_set_profile_acc, m_set_profile_dcc, m_set_profile_vel); // set profile parameters
                m_commandMsg = "";
            }
            else if (m_commandMsg == "set_encoder")
            {
                Cia301Node::SetEncoder_(m_set_encoder);
                m_commandMsg = "";
            }
            else if (m_commandMsg == "set_operation_mode")
            {
                Cia301Node::SetOperationMode_(m_set_operation_mode);
                m_commandMsg = "";
            }

            Wait(AsyncWait(duration(std::chrono::microseconds(m_sampleTime))));
            m_isConfiguring = false;
        }
        Wait(AsyncWait(duration(std::chrono::microseconds(m_sampleTime))));
    }
}

/*  This task is reponsible for enable/disable operation*/
void Cia301Node::TaskOperation() noexcept
{
    while (true)
    {
        if (m_isConfiguring)
        {
            while (m_flag_target_task_processing)
                Wait(AsyncWait(duration(std::chrono::microseconds(1))));
            if (m_commandMsg == "enable")
            {
                Cia301Node::EnableOp_(true);
                m_commandMsg = "";
            }
            else if (m_commandMsg == "disable")
            {
                Cia301Node::EnableOp_(false);
                m_commandMsg = "";
            }

            Wait(AsyncWait(duration(std::chrono::microseconds(m_sampleTime))));
            m_isConfiguring = false;
        }
        Wait(AsyncWait(duration(std::chrono::microseconds(m_sampleTime))));
    }
}

// ============================== Configuration Methods ==============================
/** A member function to set the mode of operation. The state machine needs fo be in switch on (disabled operation) before using
    @param operationMode:   An integer defining the mode of operation based on the motion controller datasheet
                            0=disable, 1=PP, 3=VP, 6=Home, -1=FaulhaberCommand*/
void Cia301Node::SetOperationMode_(OpMode operationMode)
{
    logger->debug("[Node " + m_nodeId + "] Setting mode of operation to " + std::to_string(int8_t(operationMode)));
    m_current_operation_mode = operationMode;

    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    Wait(AsyncWrite<int8_t>(MODE_OF_OPERATION_IDX, 0, static_cast<int8_t>(m_current_operation_mode))); // Set the mode of operation
    int8_t op_mod_disp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));                  // read the actual model of operation
    while (static_cast<int8_t>(op_mod_disp) != static_cast<int8_t>(m_current_operation_mode))          // wait until the mode change is confirmed
    {
        op_mod_disp = Wait(AsyncRead<int8_t>(MODE_OF_OPERATION_DISP_IDX, 0x0000));
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
    }
    logger->debug("[Node " + m_nodeId + "] Mode of operation = " + std::to_string(int(op_mod_disp)));
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order
}

/**/
void Cia301Node::SetMaxTorque_(const double negative, const double positive)
{
    logger->debug("[Node " + m_nodeId + "] Setting max negative torque to " + std::to_string(int8_t(negative)) + " and positive to " + std::to_string(int8_t(positive)));
    Wait(AsyncWrite<uint16_t>(0x60E0, 0x00, static_cast<uint16_t>(positive)));
    Wait(AsyncWrite<uint16_t>(0x60E1, 0x00, static_cast<uint16_t>(negative)));
}

/** This function sets the transmission type of a specific Transmit Process Data Object (TPDO).
    @param numPDO: An integer specifying the PDO number (1-4) to configure.
    @param transmisionType: An integer representing the desired transmission type
        (see CANopen SYNC object documentation) for the PDO.
        0 => synchronous, acyclic   (after SYNC if it has been changed)
        1 => synchronous, cyclical  (after SYNC)
        255 => asynchronous (event controlled)
        ... */
void Cia301Node::SetTpdoTranstype_(const int NumPDO, const int TransmisionType)
{
    if (TransmisionType >= 0 && TransmisionType <= 255 && TransmisionType != 254)
    {
        switch (NumPDO)
        {
        case 1:
            Wait(AsyncWrite<int8_t>(0x1800, 2, TransmisionType)); // Configure TPDO1 transmission type.
            logger->debug("[Node " + m_nodeId + "] Setting TPDO1 transmission type to " + std::to_string(TransmisionType));
            break;
        case 2:
            Wait(AsyncWrite<int8_t>(0x1801, 2, TransmisionType)); // Configure TPDO2 transmission type.
            logger->debug("[Node " + m_nodeId + "] Setting TPDO2 transmission type to " + std::to_string(TransmisionType));
            break;
        case 3:
            Wait(AsyncWrite<int8_t>(0x1802, 2, TransmisionType)); // Configure TPDO3 transmission type.
            logger->debug("[Node " + m_nodeId + "] Setting TPDO3 transmission type to " + std::to_string(TransmisionType));
            break;
        case 4:
            Wait(AsyncWrite<int8_t>(0x1803, 2, TransmisionType)); // Configure TPDO4 transmission type.
            logger->debug("[Node " + m_nodeId + "] Setting TPDO4 transmission type to " + std::to_string(TransmisionType));
            break;
        default:
            logger->warn("[Node " + m_nodeId + "] Invalid TPDO number. TPDO 1-4 are supported");
            break;
        }
    }
    else
    {
        logger->warn("[Node " + m_nodeId + "] Invalid transmission type. Type must be in the range [0, 255] excluding 254.");
    }
}

/** Member function to set the position profile parameters for the node.
   @param MaxAcc: Maximum acceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxDcc: Maximum deceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxVel: Maximum velocity     for CiA301 => [pulse/s]      for FC => [mm/s]     */
void Cia301Node::SetProfileParams_(const int MaxAcc, const int MaxDcc, const int MaxVel)
{
    if (m_controller_brand == "Maxon")
    {
        logger->debug("[Node " + m_nodeId +
                      "]: Setting profile position parameters to: " +
                      "ACC: " + std::to_string(MaxAcc) + " [rpm/s]  |  " +
                      "DCC: " + std::to_string(MaxDcc) + " [rpm/s]  |  " +
                      "Vel: " + std::to_string(MaxVel) + " [rpm]  ");
        Wait(AsyncWrite<uint32_t>(0x6065, 00, static_cast<uint32_t>(1000000)));   // Max. Following Error
        Wait(AsyncWrite<int32_t>(0x607D, 01, static_cast<int32_t>(-2147483648))); // Min. Position Limit    (PP)
        Wait(AsyncWrite<int32_t>(0x607D, 02, static_cast<int32_t>(2147483647)));  // Max. Position Limit    (PP)
        Wait(AsyncWrite<uint32_t>(0x6081, 00, static_cast<uint32_t>(MaxVel)));    // set profile velocity   (PP)
        Wait(AsyncWrite<uint32_t>(0x6083, 00, static_cast<uint32_t>(MaxAcc)));    // set profile acc    (PP & PV)
        Wait(AsyncWrite<uint32_t>(0x6084, 00, static_cast<uint32_t>(MaxDcc)));    // set profile dcc    (PP & PV)
        Wait(AsyncWrite<uint32_t>(0x60C5, 00, static_cast<uint32_t>(10000000)));  // set max ACC        (Velocity mode)

        // Wait(AsyncWrite<int32_t>(0x6086, 00, static_cast<uint16_t>(0))); // Max. Position Limit
    }
    else
    {
        logger->debug("[Node " + m_nodeId +
                      "]: Setting profile position parameters to: " +
                      "ACC: " + std::to_string(MaxAcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                      "DCC: " + std::to_string(MaxDcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                      "Vel: " + std::to_string(MaxVel) + " [0.01mm/s] or [0.1deg/s]  ");
        // Wait(AsyncWrite<uint32_t>(0x6065, 00, static_cast<uint32_t>(1000000)));   // Max. Following Error
        // Wait(AsyncWrite<int32_t>(0x607D, 01, static_cast<int32_t>(-2147483648))); // Min. Position Limit    (PP)
        // Wait(AsyncWrite<int32_t>(0x607D, 02, static_cast<int32_t>(2147483647)));  // Max. Position Limit    (PP)
        Wait(AsyncWrite<uint32_t>(0x6081, 00, static_cast<uint32_t>(MaxVel))); // set profile velocity   (PP)
        Wait(AsyncWrite<uint32_t>(0x6083, 00, static_cast<uint32_t>(MaxAcc))); // set profile acc    (PP & PV)
        Wait(AsyncWrite<uint32_t>(0x6084, 00, static_cast<uint32_t>(MaxDcc))); // set profile dcc    (PP & PV)

        // Wait(AsyncWrite<uint32_t>(0x60C5, 00, static_cast<uint32_t>(10000000)));  // set max ACC        (Velocity mode)
        // Wait(AsyncWrite<int32_t>(0x6086, 00, static_cast<uint16_t>(0))); // Max. Position Limit
    }
}

/** This function sets the current position of the encoder. It is verified only for CiA301
    Other tasks should be on hold when "Set_encoder" is running
    @param offset set value for the current positon in userdefined unit (see Faulhaber documentation)*/
void Cia301Node::SetEncoder_(const double value)
{
    bool flag_home_done = false;
    logger->debug("[Node " + m_nodeId + "] Setting encoder value...");

    Cia301Node::EnableOp_(false);                                                    // disable and wait to make sure motion is stopped - disabling is necessary to change the mode of operation
    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                       // wait for better log order
    Cia301Node::SetOperationMode_(OpMode::Homing);                                   // switch to Homing mode
    int8_t homing_method = (m_controller_brand == "Maxon") ? 35 : 37;                // Homing method 35 for Maxon and 37 for Faulhaber
    Wait(AsyncWrite<int8_t>(0x6098, 00, static_cast<int8_t>(homing_method)));        // set Homing method
    Cia301Node::EnableOp_(true);                                                     // must be enabled before start homing
    Wait(AsyncWait(duration(std::chrono::milliseconds(500))));                       // wait for better log order
    Wait(AsyncWrite<int32_t>(0x607C, 00, static_cast<int32_t>(-1 * value * m_ppu))); // set Homing offset
    m_controlWord.enable_operation = 1;                                              // must be enabled before start homing
    m_controlWord.bit4 = 0;                                                          // set Bit 4 to 0 to tuggle to 1 later for start of homing
    Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get()));            // send controlword on SDO
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                        // wait for safety
    m_controlWord.bit4 = 1;                                                          // set Bit 4 0->1 (rising edge means start homing)
    Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get()));            // send controlword on SDO
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                        // wait for safety
    logger->debug("[Node " + m_nodeId + "] encoder set value: " + std::to_string(value) + " | pulse count: " + std::to_string(static_cast<int32_t>(-1 * value * m_ppu)));
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    while (!flag_home_done) // wait until homing is confirmed
    {
        m_statusWord.update(Wait(AsyncRead<uint16_t>(STATUS_WORD_IDX, 0x0000))); // get status word on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));                // wait for safety
        logger->debug("[Node " + m_nodeId + "] enable: " + std::to_string(m_statusWord.operation_enabled) + ": bit10: " + std::to_string(m_statusWord.bit10) + ": bit12: " + std::to_string(m_statusWord.bit12) + ": bit13: " + std::to_string(m_statusWord.bit13));
        if (m_statusWord.bit12 && m_statusWord.bit10) // check the bit (12-homing attained) and bit (10-target reached)
        {
            m_controlWord.bit4 = 0;                                               // Bit 4 to 0
            Wait(AsyncWrite<uint16_t>(CONTROL_WORD_IDX, 0, m_controlWord.get())); // send controlword on SDO
            flag_home_done = true;                                                // set homing finished flag
            logger->debug("[Node " + m_nodeId + "] Current position set to " + std::to_string(value));
            m_flags.set(Flags::FlagIndex::ENCODER_SET, true);
        }
        if (m_statusWord.bit13)
        {
            // m_encoder_memory_file.close();
            logger->critical("[Node " + m_nodeId + "] A homing error has occurred");
        }
    }
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    Cia301Node::EnableOp_(false);                              // disable before changing operation mode
    Cia301Node::SetOperationMode_(m_operation_mode);           // bring back to original operation mode
    Wait(AsyncWait(duration(std::chrono::milliseconds(500)))); // wait for better log order

    // Node::Set_operation_mode(0);
    // Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
}

/*needs to be verified and tested*/
void Cia301Node::FindFowrardLimits_()
{
    // double vel_findlimit = -0.00004; //[m/s] or [red/sec]
    bool limit_reached = false;
    const int maxBufferSize = 10;
    std::deque<double> currentBuffers; // Circular buffers for each joint
    double pos_0, pos_1, current_avg, current = 0.0;

    Cia301Node::getPos(pos_0);

    Cia301Node::EnableOp_(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    Cia301Node::SetOperationMode_(OpMode::VelocityProfile);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    Cia301Node::EnableOp_(true);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));

    m_controlWord.enable_operation = 1;                                                  // set immediately bit (bit 5)
    tpdo_mapped[TARGET_VELOCITY_IDX][0] = static_cast<int32_t>(m_vel_findlimit * m_ppu); // target velocity
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_controlWord.get();                              // enable
    tpdo_mapped[TARGET_VELOCITY_IDX][0].WriteEvent();                                    // Trigger write events for PDO1.

    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));

    while (!limit_reached)
    {
        // update average current
        Cia301Node::getCurrent(current);

        currentBuffers.push_back(current);
        if (currentBuffers.size() > maxBufferSize)
        {
            currentBuffers.pop_front(); // Keep the buffer size at 100
        }
        double sum = 0.0;
        for (auto it = currentBuffers.begin(); it != currentBuffers.end(); ++it)
        {
            sum += *it;
        }
        current_avg = sum / static_cast<double>(currentBuffers.size());

        if (abs(current) >= m_current_threshold && !limit_reached && currentBuffers.size() == maxBufferSize)
        {
            limit_reached = true;
            Cia301Node::EnableOp_(false);
            Cia301Node::getPos(pos_1);
            m_pos_offset_SI = pos_0 - pos_1;
        }

        std::cout << "Node :" << m_nodeId << "  "
                  << "averaged current: " << current_avg * 1e3 << " [mA]  | reached: " << limit_reached << std::endl;
        Wait(AsyncWait(duration(std::chrono::milliseconds(1))));
    }
    blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
    Cia301Node::setVel(0.0);
    Cia301Node::EnableOp_(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    std::cout << "---- Node X -> Finding limit done ----" << std::endl;
}

/* Sets in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void Cia301Node::setHomeOffsetValue(const double val)
{
    m_pos_offset_SI = val;
}

// The member functions below trigger an associated function inside "taskOperation" or "taskConfig"
// to comply with asynchronous commanding through SDO while "Task Target" is posted.
// Since the actual function is executed on another thread (fiber), the task execution may take some time.
// Therefore, a wait should be used after calling the functions below.

/**/
void Cia301Node::enableOperation(const bool enable)
{
    m_isConfiguring = true;
    if (enable)
        m_commandMsg = "enable";
    else
        m_commandMsg = "disable";
}

/**/
void Cia301Node::setMaxTorque(const double negative, const double positive)
{
    m_set_max_torque[0] = negative;
    m_set_max_torque[1] = positive;
    m_isConfiguring = true;
    m_commandMsg = "set_max_torque";
}

/**/
void Cia301Node::setProfileParams(const double acc, const double dcc, const double vel)
{
    m_set_profile_acc = static_cast<unsigned int>(abs(acc * m_conversionFactor));
    m_set_profile_dcc = static_cast<unsigned int>(abs(dcc * m_conversionFactor));
    m_set_profile_vel = static_cast<unsigned int>(abs(vel * m_conversionFactor));

    m_isConfiguring = true;
    m_commandMsg = "set_profile_params";
}

/**/
void Cia301Node::setEncoder(const double offset)
{
    m_set_encoder = offset;
    m_isConfiguring = true;
    m_commandMsg = "set_encoder";
}

/**/
void Cia301Node::setOperationMode(const OpMode mode)
{
    m_set_operation_mode = mode;
    m_isConfiguring = true;
    m_commandMsg = "set_operation_mode";
}

/* Sets position limit SI unit and converts to motion controller unit
    and updates the associates variable to be send to motion controller*/
void Cia301Node::setPosLimit(const double min, const double max)
{
    m_PosLimitSi[0] = min;
    m_PosLimitSi[1] = max;
    m_PosLimit[0] = static_cast<int32_t>(min * m_ppu);  // min
    m_PosLimit[1] = static_cast<int32_t>(max * m_ppu);  // max
}

// ============================== Command Methods ==============================
/* Sets target position in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void Cia301Node::setPosAbs(const double val)
{
    m_targetPosSi = val;
    m_targetPos = static_cast<int32_t>(m_targetPosSi * m_ppu);
}

/* Sets target position in [m] or [rad] and converts to motion controller unit
    and updates the associates variable to be executes in the next sample*/
void Cia301Node::setPos(const double val)
{
    m_targetPosSi = val + m_pos_offset_SI;
    m_targetPos = static_cast<int32_t>(m_targetPosSi * m_ppu);
}

/* Sets target valocit in [m/s] or [rad/s] and converts motion controller unit
    and updates the associates variable to be executes in the next sample*/
void Cia301Node::setVel(const double val)
{
    m_targetVelSi = val;
    m_targetVel = static_cast<int32_t>(m_targetVelSi * m_conversionFactor);
}

/*needs to be verified and tested*/
void Cia301Node::toZeroPos()
{
    // double vel_findlimit = -0.00004; //[m/s] or [red/sec]
    bool limit_reached = false;
    const int maxBufferSize = 10;
    std::deque<double> currentBuffers; // Circular buffers for each joint
    double pos_0, pos_1, current_avg, current = 0.0;

    Cia301Node::getPos(pos_0);

    Cia301Node::EnableOp_(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    Cia301Node::SetOperationMode_(OpMode::PositionProfile);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    Cia301Node::EnableOp_(true);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));

    m_controlWord.enable_operation = 1;                                                  // set immediately bit (bit 5)
    tpdo_mapped[TARGET_VELOCITY_IDX][0] = static_cast<int32_t>(m_vel_findlimit * m_ppu); // target velocity
    tpdo_mapped[CONTROL_WORD_IDX][0] = m_controlWord.get();                              // enable
    tpdo_mapped[TARGET_VELOCITY_IDX][0].WriteEvent();                                    // Trigger write events for PDO1.

    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));

    while (!limit_reached)
    {
        // update average current
        Cia301Node::getCurrent(current);

        currentBuffers.push_back(current);
        if (currentBuffers.size() > maxBufferSize)
        {
            currentBuffers.pop_front(); // Keep the buffer size at 100
        }
        double sum = 0.0;
        for (auto it = currentBuffers.begin(); it != currentBuffers.end(); ++it)
        {
            sum += *it;
        }
        current_avg = sum / static_cast<double>(currentBuffers.size());

        if (abs(current) >= m_current_threshold && !limit_reached && currentBuffers.size() == maxBufferSize)
        {
            limit_reached = true;
            Cia301Node::EnableOp_(false);
            Cia301Node::getPos(pos_1);
            m_pos_offset_SI = pos_0 - pos_1;
        }

        std::cout << "Node :" << m_nodeId << "  "
                  << "averaged current: " << current_avg * 1e3 << " [mA]  | reached: " << limit_reached << std::endl;
        Wait(AsyncWait(duration(std::chrono::milliseconds(1))));
    }
    blaze::StaticVector<double, 6> v = blaze::StaticVector<double, 6>(0);
    Cia301Node::setVel(0.0);
    Cia301Node::EnableOp_(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    std::cout << "---- Node X -> Finding limit done ----" << std::endl;
}

// ============================== Feedback Methods ==============================
/* accessor to motor current [A]*/
void Cia301Node::getCurrent(double &current) const
{
    current = m_currentSi;
}

/* accessor to moving average of motor current [A]*/
void Cia301Node::getCurrentAvg(double &current) const
{
    current = std::accumulate(m_currentSiHist.begin(), m_currentSiHist.end(), 0.0) / m_currentSiHist.size();
}

/* accessor toactual motor positon [m] or [rad]*/
void Cia301Node::getPosAbs(double &actualPosPtr) const
{
    actualPosPtr = m_currentPosSi;
}

/* accessor toactual motor positon [m] or [rad]*/
void Cia301Node::getPos(double &actualPosPtr) const
{
    actualPosPtr = m_currentPosSi - m_pos_offset_SI;
}

/* accessor toactual motor positon [m] or [rad]*/
void Cia301Node::getPosLimit(double &min, double &max) const
{
    min = m_currentPosLimitSi[0];
    max = m_currentPosLimitSi[1];
}

/* accessor to actual motor velocity [m/s] or [rad/s]*/
void Cia301Node::getVel(double &actualVelPtr) const
{
    // *actualVelPtr = double(vel_actual / (60.0 / gear_ratio));
    actualVelPtr = m_currentVelSi;
}

/* if target is reached */
bool Cia301Node::isReached() const
{
    if (m_current_operation_mode == OpMode::PositionProfile || m_current_operation_mode == OpMode::VelocityProfile)
    {
        return m_statusWord.bit10;
    }
    else
    {
        logger->debug("[Node " + m_nodeId + "]Target reached is only defined for PP and PV modes ");
        return 0;
    }
}

// ============================== Utility Methods ==============================
/* need a cleanup*/
void Cia301Node::ReadEncoderMemFile(const std::string &directoryPath, double &encoder_memory, std::ofstream &encoder_memory_file)
{
    // check for encoder memory file to set
    std::stringstream file_name;
    file_name << directoryPath << "encoder_memory_node_" << m_nodeId << ".dat";

    if (std::filesystem::exists(file_name.str())) // Check if the file exists
    {
        // std::cout << "[Node " << id << ": Encoder memory file axists" << std::endl;
        std::ifstream temp(file_name.str());
        if (temp.is_open())
        {
            try
            {
                std::string content;
                temp >> content;                     // Read the content as a string
                encoder_memory = std::stod(content); // Convert the string to double
                logger->debug("[Node " + m_nodeId + "] Encoder memory = " + std::to_string(encoder_memory));
                temp.close();
            }
            catch (const std::exception &e)
            {
                // Handle the exception here, e.g., log an error message
                temp.close();
                logger->critical("[Node " + m_nodeId + "] Error reading encoder memory. Check memory files: " + directoryPath + "|" + e.what());
            }
        }
        else
        {
            logger->critical("[Node " + m_nodeId + "] Unable to open encoder memory file to read: " + directoryPath);
        }

        (encoder_memory_file).open(file_name.str(), std::ios::out);
        if (!(encoder_memory_file).is_open())
        {
            logger->critical("[Node " + m_nodeId + "] Unable to open encoder memory file to write: " + directoryPath);
        }
    }
    else
    {
        if (!std::filesystem::exists(directoryPath))
        {
            if (std::filesystem::create_directories(directoryPath))
            {
                logger->debug("[Node " + m_nodeId + "] Enoder memory folder directory created: " + directoryPath);
            }
            else
            {
                logger->critical("[Node " + m_nodeId + "] Failed to create memory file directory: " + directoryPath);
            }
        }

        encoder_memory = 0;

        (encoder_memory_file).open(file_name.str(), std::ios::out);
        if ((encoder_memory_file).is_open())
        {
            encoder_memory_file << 0;
            logger->debug("[Node " + m_nodeId + "] Memory file built and opened");
        }
        else
        {
            logger->critical("[Node " + m_nodeId + "] Unable to open built encoder memory file to write");
        }
    }
}

// Getter function to retrieve the StatusWord
StatusWord Cia301Node::getStatusword() const
{
    return m_statusWord;
}

// Getters
bool Cia301Node::getFlags(const Flags::FlagIndex index) const
{
    return m_flags.get(index);
}

// Getters
OpMode Cia301Node::getOperationMode() const
{
    return m_current_operation_mode;
}
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
  This class is responsible for creating virtual nodes that facilitate communication with
  motion controllers (Faulhaber or Maxon) using the CANopen protocol.
  It supports the CiA301 operation layer, Faulhaber channel commands (which is now considered obsolete), and CiA402 device profile .
*/

#include "CiA301node.hpp"

using namespace std::chrono_literals;
using namespace lely;

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
Node::Node(CTRobot *parent, ev_exec_t * /*exec*/, canopen::AsyncMaster &master, unsigned int NodeID,
           double EncoderResolution, double GearRatio, double VelocityFactor,
           unsigned int SampleTime, int OperationMode, double ProfileAcc, double ProfileVel)
    : FiberDriver(master, NodeID), parent_robot(parent)
{
    this->m_controller_brand = "Faulhaber"; // manually set the brand: "Faulhaber" or "Maxon"
    this->m_sample_time = SampleTime;
    this->m_operation_mode = OperationMode;
    this->m_ppu = (EncoderResolution / GearRatio);

    // convert profile position parameters
    if (this->m_controller_brand == "Maxon")
    {
        this->m_profile_acc = static_cast<unsigned int>(ProfileAcc * 60 / GearRatio); // mm/s^2 to rpm/s
        this->m_profile_vel = static_cast<unsigned int>(ProfileVel * 60 / GearRatio); // mm/s to  rpm
    }
    else if (this->m_controller_brand == "Faulhaber")
    {
        this->m_profile_acc = static_cast<unsigned int>(ProfileAcc * this->m_ppu / VelocityFactor); // mm/s^2 to rpm/s
        this->m_profile_vel = static_cast<unsigned int>(ProfileVel * this->m_ppu / VelocityFactor); // mm/s to  rpm
    }
    else
    {
        parent_robot->Log_message("Node " + std::to_string(NodeID) +
                                      ": Unknown motion controler brand",
                                  true);
        exit(1);
    }
    parent_robot->Log_message("Node " + std::to_string(NodeID) + ": " +
                                  "Controller brand: " + this->m_controller_brand +
                                  "PPU: " + std::to_string(this->m_ppu) + ":   " +
                                  "Gear ratio: " + std::to_string(GearRatio) + ":   " +
                                  "Encoder resoluton: " + std::to_string(EncoderResolution),
                              false);
}

Node::~Node()
{
    m_encoder_memory_file.close();
    delete parent_robot;
}

/* A member function to switch on the driver
    supports CiA402 State Machine Application Layer (not good for Faulhaber Command)
    the switch on function should be called in the OnConfig callback function.*/
void Node::Swtitch_ON()
{
    std::string message;
    m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get statusword on SDO
    message = m_status_word.getCiA402StatusMessage();
    if (m_status_word.Switch_On_Disabled)
    {
        m_control_word.switch_ON = 0;
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 1;
        m_control_word.enable_operation = 0;
        Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get())); // shut down (switch on disable)
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000)));
        message = message + "  =>  " + m_status_word.getCiA402StatusMessage();
    }
    if ((m_status_word.ready_to_switch_ON && !m_status_word.switched_ON) ||
        m_status_word.operation_enabled)
    {
        m_control_word.switch_ON = 1;
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 1;
        m_control_word.enable_operation = 0;
        Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get())); // switch on or disable operation
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000)));
        auto startTime = std::chrono::high_resolution_clock::now(); // timer for timeout check
        auto timeoutDuration = std::chrono::seconds(2);
        while (!m_status_word.switched_ON)
        {
            m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000)));
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
            if (elapsedTime >= timeoutDuration)
            {
                parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                              "Timeout reached (2 seconds)",
                                          true);
                message = message + "  =>  " + "Switch on timed out!";
                std::exit(1);
            }
        }
        message = message + "  =>  " + m_status_word.getCiA402StatusMessage();
    }
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) + ": " + message, false);
}

/** A member function to enabled / disable the driver operation
    upports CiA301 Application Layer (not good for Faulhaber Command)
    the Enable member function should be called in the OnConfig callback function.
    @param enable: true =  enable, false = disable*/
void Node::Enable_operation(bool enable)
{
    // using TPDO1
    if (enable)
    {
        m_control_word.enable_operation = 1;
        tpdo_mapped[0x6040][0] = m_control_word.get(); // enable
        tpdo_mapped[0x6040][0].WriteEvent();
        flag_operation_enabled = true;
        // flag_new_positon_targ_ready = true;
    }
    else
    {
        m_control_word.enable_operation = 0;
        tpdo_mapped[0x6040][0] = m_control_word.get(); // disbale
        tpdo_mapped[0x6040][0].WriteEvent();
        flag_operation_enabled = false;
    }
    // using SDO (SDO method cannot be called out of the Task loop. It causes segmentation fault error)
    // Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x000F));
    // uint16_t statusword = Wait(AsyncRead<uint16_t>(0x6041, 0x0000));
    // while (getCiA402StatusMessage(statusword) != "Operation enabled")
    // {
    //     statusword = Wait(AsyncRead<uint16_t>(0x6041, 0x0000));
    // }
    // statusword = Wait(AsyncRead<uint16_t>(0x6041, 0x0000));
    // std::cout << "Node " << static_cast<int>(id()) << ": " << getCiA402StatusMessage(statusword) << std::endl;
    // if (getCiA402StatusMessage(statusword) == "Operation enabled")
    // {
    //     flag_operation_enabled = true; = true;
    // }
}

/* A member funtion to reset faults
    should be called in oOnConfig*/
void Node::Reset_fault()
{
    if (m_status_word.fault)
    {
        m_control_word.fault_reset = 0;
        Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get()));
        m_control_word.fault_reset = 1;
        Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get()));
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Reset fault => Fault status: " + std::to_string(m_status_word.fault),
                                  true);
    }
    else
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": No fault to reset",
                                  false);
    }
}

/** A member function to set the mode of operation -- it automatically disables the operation
    @param operationMode:   An integer defining the mode of operation
                            based on the motion controller datasheet
                            0=disable, 1=PP, 3=VP, 6=Home */
void Node::Set_operation_mode(int operationMode)
{
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Setting mode of operation to " + std::to_string(int(operationMode)),
                              false);
    m_current_operation_mode = operationMode;
    // operation must be disabled before changing operation mode
    Node::Enable_operation(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
    m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
    while (m_status_word.operation_enabled)
    {
        Node::Enable_operation(false);
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
    }
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Disabling ",
                              false);

    // Set the mode of operation
    Wait(AsyncWrite<int8_t>(0x6060, 0, m_current_operation_mode));
    int8_t op_mod_disp = Wait(AsyncRead<int8_t>(0x6061, 0x0000));
    // wait until receive mode change confirmation
    while (op_mod_disp != m_current_operation_mode)
    {
        Wait(AsyncWait(duration(std::chrono::milliseconds(5))));
        op_mod_disp = Wait(AsyncRead<int8_t>(0x6061, 0x0000));
    }
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Mode of operation = " + std::to_string(int(op_mod_disp)),
                              false);
}

/** This function sets the transmission type of a specific Transmit Process Data Object (TPDO).
    @param numPDO: An integer specifying the PDO number (1-4) to configure.
    @param transmisionType: An integer representing the desired transmission type
        (see CANopen SYNC object documentation) for the PDO.
        0 => synchronous, acyclic   (after SYNC if it has been changed)
        1 => synchronous, cyclical  (after SYNC)
        255 => asynchronous (event controlled)
        ... */
void Node::Set_tpdo_transtype(int NumPDO, int TransmisionType)
{
    if (TransmisionType >= 0 && TransmisionType <= 255 && TransmisionType != 254)
    {
        switch (NumPDO)
        {
        case 1:
            // Configure TPDO1 transmission type.
            Wait(AsyncWrite<int8_t>(0x1800, 2, TransmisionType));
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Setting TPDO1 transmission type to " + std::to_string(TransmisionType),
                                      false);
            break;
        case 2:
            // Configure TPDO2 transmission type.
            Wait(AsyncWrite<int8_t>(0x1801, 2, TransmisionType));
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Setting TPDO2 transmission type to " + std::to_string(TransmisionType),
                                      false);
            break;
        case 3:
            // Configure TPDO3 transmission type.
            Wait(AsyncWrite<int8_t>(0x1802, 2, TransmisionType));
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Setting TPDO3 transmission type to " + std::to_string(TransmisionType),
                                      false);
            break;
        case 4:
            // Configure TPDO3 transmission type.
            Wait(AsyncWrite<int8_t>(0x1803, 2, TransmisionType));
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Setting TPDO4 transmission type to " + std::to_string(TransmisionType),
                                      false);
            break;
        default:
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ":Invalid TPDO number. TPDO 1-4 are supported",
                                      true);
            break;
        }
    }
    else
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ":Invalid transmission type. Type must be in the range [0, 255] excluding 254.",
                                  true);
    }
}

/** Member function to set the position profile parameters for the node.
   @param MaxAcc: Maximum acceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxDcc: Maximum deceleration   for CiA301 => [pulse/s^2]    for FC => [mm/s^2]
   @param MaxVel: Maximum velocity     for CiA301 => [pulse/s]      for FC => [mm/s]     */
void Node::Set_profile_params(int MaxAcc, int MaxDcc, int MaxVel)
{
    if (this->m_controller_brand == "Maxon")
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Setting profile position parameters to: " +
                                      "ACC: " + std::to_string(MaxAcc) + " [rpm/s]  |  " +
                                      "DCC: " + std::to_string(MaxDcc) + " [rpm/s]  |  " +
                                      "Vel: " + std::to_string(MaxVel) + " [rpm]  ",
                                  false);
    }
    else if (this->m_controller_brand == "Faulhaber")
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Setting profile position parameters to: " +
                                      "ACC: " + std::to_string(MaxAcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                                      "DCC: " + std::to_string(MaxDcc) + " [0.01mm/s^2] or [0.1deg/s^2]  |  " +
                                      "Vel: " + std::to_string(MaxVel) + " [0.01mm/s] or [0.1deg/s]  ",
                                  false);
    }
    Wait(AsyncWrite<int32_t>(0x6065, 00, static_cast<uint32_t>(50000)));      // Max. Following Error
    Wait(AsyncWrite<int32_t>(0x607D, 01, static_cast<int32_t>(-2147483648))); // Min. Position Limit    (PP)
    Wait(AsyncWrite<int32_t>(0x607D, 02, static_cast<int32_t>(2147483647)));  // Max. Position Limit    (PP)
    Wait(AsyncWrite<int32_t>(0x6081, 00, static_cast<uint32_t>(MaxVel)));     // set profile velocity   (PP)
    Wait(AsyncWrite<int32_t>(0x6083, 00, static_cast<uint32_t>(MaxAcc)));     // set profile acc    (PP & PV)
    Wait(AsyncWrite<int32_t>(0x6084, 00, static_cast<uint32_t>(MaxDcc)));     // set profile dcc    (PP & PV)
    // Wait(AsyncWrite<int32_t>(0x6086, 00, static_cast<uint16_t>(0))); // Max. Position Limit
}

/** This function sets the current position of the encoder.
    it should be called inside the OnConfig tast, and other tasks should
    be held when this one is running
    @param offset set value for the current positon in userdefined unit (see Faulhaber documentation)*/
void Node::Set_encoder(double value)
{
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Setting encoder value ",
                              false);
    bool flag_home_done = false;
    Node::Enable_operation(false); // disable and wait to make sure motion is stopped
    Wait(AsyncWait(duration(std::chrono::milliseconds(1000))));
    Node::Set_operation_mode(6); // switch to Homing mode
    if (this->m_controller_brand == "Maxon")
    {
        Wait(AsyncWrite<int8_t>(0x6098, 00, 35)); // Homing method 35 for Maxon and 37 for Faulhaber
    }
    else if (this->m_controller_brand == "Faulhaber")
    {
        Wait(AsyncWrite<int8_t>(0x6098, 00, 37));
    }

    Node::Enable_operation(true); // must be enabled before start homing
    Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
    m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
    while (!m_status_word.operation_enabled)
    {
        Node::Enable_operation(true);
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
    }

    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": encoder set value: " + std::to_string(value),
                              false);

    Wait(AsyncWrite<int32_t>(0x607C, 00, static_cast<int32_t>(-1 * value * this->m_ppu))); // Homing offset
    m_control_word.bit4 = 0;
    Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get())); // Bit 4 to 0
    m_control_word.bit4 = 1;
    Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get())); // Bit 4 0->1 (start homing)
    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Checkpoint 5",
                              false);

    // wait until the homing confirms
    while (!flag_home_done)
    {
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
        Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
        // check the bit (12-homing attained) and bit (10-target reached)
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": enable: " + std::to_string(m_status_word.operation_enabled) +
                                      ": bit10: " + std::to_string(m_status_word.bit10) +
                                      ": bit12: " + std::to_string(m_status_word.bit12) +
                                      ": bit13: " + std::to_string(m_status_word.bit13),
                                  false);
        if (m_status_word.bit12 && m_status_word.bit10)
        {
            m_control_word.bit4 = 0;
            Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get())); // Bit 4 to 0
            flag_home_done = true;
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Current position set to " + std::to_string(value),
                                      false);
        }
        if (m_status_word.bit13)
        {
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": A homing error has occurred",
                                      true);
            m_encoder_memory_file.close();
            std::exit(1);
        }
    }

    Node::Enable_operation(false);
    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Checkpoint 6",
                              false);
    Node::Set_operation_mode(0);
    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Checkpoint 7",
                              false);
    Node::Set_operation_mode(this->m_operation_mode); // bring back to original operation mode
    Wait(AsyncWait(duration(std::chrono::milliseconds(100))));
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Checkpoint 8",
                              false);
}

/* This function gets called during the boot-up process for the node. The
    'res' parameter is the function that MUST be invoked when the configuration
    is complete. Because this function runs as a task inside a coroutine, it
    can suspend itself and wait for an asynchronous function, such as an SDO
    request, to complete. */
void Node::OnConfig(std::function<void(std::error_code ec)> res) noexcept
{
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) + ": Booting", false);
    try
    {
        Node::Reset_fault();
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
        while (m_status_word.fault)
        {
            Node::Reset_fault();
            Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
            m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
        }
        // Node::Swtitch_ON();
        // Node::Set_operation_mode(this->m_operation_mode);
        // Node::Set_profile_params(this->m_profile_acc, this->m_profile_acc, this->m_profile_vel);
        res({}); // Indicate successful completion of the configuration by invoking the 'res' function.
    }
    catch (canopen::SdoError &e)
    {
        // If one of the SDO requests resulted in an error, abort the configuration
        // and report the error code using the 'res' function.
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) + ": Error Configing", true);
        res(e.code());
    }
}

/* This function gets called when the boot-up process of the node completes.
    The 'st' parameter contains the last known NMT state of the slave
    (typically pre-operational), 'es' the error code (0 on success), and 'what'
    a description of the error, if any.*/
void Node::OnBoot(canopen::NmtState /*st*/, char es,
                  const std::string &what) noexcept
{
    if (!es || es == 'L')
    {
        // Successful boot-up or boot-up is in progress ("L" state).
        flag_bootSuccess = true;
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Booted successfully.",
                                  false);
        // wait for all other nodes to boot
        while (!parent_robot->m_boot_success)
        {
            Wait(AsyncWait(duration(std::chrono::milliseconds(5))));
        }

        Node::Swtitch_ON();
        Node::Enable_operation(true);
        Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Enabling test",
                                  false);
        while (!m_status_word.operation_enabled)
        {
            Node::Enable_operation(true);
            Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
            m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word on SDO
        }
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Enabled",
                                  false);
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        while (!parent_robot->operation_enabled)
        {
            Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        }

        Node::Set_operation_mode(this->m_operation_mode);
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));

        Node::Set_profile_params(this->m_profile_acc, this->m_profile_acc, this->m_profile_vel);
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));

        Node::Set_tpdo_transtype(1, 1);
        Node::Set_tpdo_transtype(2, 1);
        Node::Read_from_encoder_memory_file(EnoderStoreFiles_directory,
                                            &this->m_encoder_memory_value,
                                            &this->m_encoder_memory_file);
        Wait(AsyncWait(duration(std::chrono::milliseconds(50)))); // this wait is necessary to give time opening memory files
        m_encoder_memory_ready = true;

        Node::Set_encoder(m_encoder_memory_value);
        Node::Enable_operation(true);
        encoder_set =  true;

        // wait for all nodes to get operational
        while (!parent_robot->operation_enabled || !parent_robot->encoders_set)
        {
            Wait(AsyncWait(duration(std::chrono::milliseconds(10))));
        }
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": " + m_status_word.getCiA402StatusMessage(),
                                  false);

        m_new_pos_targ_ready = true;
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));

        Post(&Node::Task_target, this);
        Post(&Node::Task_config, this);
        m_tasks_posted = true;
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Task posted",
                                  false);
    }
    else
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Failed to boot: " + what,
                                  true);
    }
}

/* This function is similar to OnConfg(), but it gets called by the
    AsyncDeconfig() method of the master. */
void Node::OnDeconfig(std::function<void(std::error_code ec)> res) noexcept
{
    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                  ": Deconfiging",
                              false);
    try
    {
        m_control_word.enable_voltage = 1;
        m_control_word.quick_stop = 0;
        Wait(AsyncWrite<uint16_t>(0x6040, 0, m_control_word.get())); // Quick Stop
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        m_status_word.update(Wait(AsyncRead<uint16_t>(0x6041, 0x0000))); // get status word
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) + ": " +
                                      m_status_word.getCiA402StatusMessage(),
                                  true);
        m_encoder_memory_file.close();
        delete parent_robot;
        Wait(AsyncWait(duration(std::chrono::milliseconds(50))));
        res({});
    }
    catch (canopen::SdoError &e)
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      ": Error Deconfiging",
                                  true);
        res(e.code());
    }
}

/* This function gets called every time a value is written to the local object
    dictionary of the master by an RPDO (or SDO, but that is unlikely for a
    master), *and* the object has a known mapping to an object on the slave for
    which this class is the driver. The 'idx' and 'subidx' parameters are the
    object index and sub-index of the object on the slave, not the local object
    dictionary of the master. */
void Node::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
    // if RxPDO 1 received ---- info in RxPDO1 -> [1] Status Word, [2] Actual Positon
    if (idx == 0x6041 && subidx == 0)
    {
        // update status word
        m_status_word.update(rpdo_mapped[0x6041][0]);
        // update actual position
        m_actual_pos = rpdo_mapped[0x6064][0];
        m_actual_pos_SI = double(m_actual_pos) / double(m_ppu);
        // store the actual posotion in encoder memory
        if (m_status_word.switched_ON && m_encoder_memory_ready)
        {
            if (m_encoder_memory_file.is_open())
            {
                m_encoder_memory_file.seekp(0); // Move the write pointer to the beginning
                double pval;
                Node::Get_actual_pos(&pval);
                m_encoder_memory_file << std::fixed << std::setprecision(12) << pval;
            }
            else
            {
                parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                              ": Unable to open encoder memory file to write",
                                          true);
                exit(1);
            }
        }

        // check the target position acknowledge bit (12-stw) => setpoint is acknowlesged by the motioncontroller
        // switch off the new target bit (4-ctrlw) if acknowledged
        if (m_current_operation_mode == 1)
        {
            if (m_status_word.bit12 && !bit12_prev) // rising edge of bit12 (set point acknowledge)
            {
                m_control_word.bit4 = 0;                       // set new target bit (bit 4)
                tpdo_mapped[0x6040][0] = m_control_word.get(); // unset new target bit
                tpdo_mapped[0x6040][0].WriteEvent();           // Trigger write events for PDO1.
                bit12_prev = m_status_word.bit12;
                if (print_pdos)
                {
                    std::cout << "=> tPDO1 [" << static_cast<int>(id())
                              << "]: Control: 0b" << ToBinaryString(m_control_word.get()) << std::endl;
                    std::cout << "         [" << static_cast<int>(id())
                              << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0x"
                              << std::hex << m_control_word.get() << std::endl;
                }
            }
            else if (!m_status_word.bit12 && bit12_prev) // falling edge of bit12 (set point acknowledge)
            {
                m_new_pos_targ_ready = true;
                bit12_prev = m_status_word.bit12;
            }
        }

        if (print_pdos)
        {
            std::cout << "<= rPDO1 [" << static_cast<int>(id())
                      << "]: pos: " << std::dec << this->m_actual_pos
                      << "  \tStatus: 0b" << ToBinaryString(m_status_word.statusword) << std::endl;
            std::cout << "         [" << static_cast<int>(id())
                      << "]:[" << std::hex << 0x6064 << "][" << static_cast<int>(0) << "]: "
                      << std::dec << this->m_actual_pos << std::endl;
            std::cout << "         [" << static_cast<int>(id())
                      << "]:[" << std::hex << 0x6041 << "][" << static_cast<int>(0) << "]: "
                      << std::hex << "0x" << m_status_word.statusword << std::endl;
        }
    }

    // if RxPDO 2 received ---- info in RxPDO2 -> [1] Actual Velocity, [2] Current(Troque)
    else if (idx == 0x606c && subidx == 0)
    {
        // update current
        if (m_controller_brand == "Maxon")
        {
            m_current = rpdo_mapped[0x6078][0];      // update current
            m_current_SI = double(m_current) / 1000; // convert to SI unit
        }
        else // (this->controller_brand == "Faulhaber")
        {
            m_current = rpdo_mapped[0x6077][0];      // update torque
            m_current_SI = double(m_current) / 1000; // convert to SI unit
        }

        // update actual velocity
        m_actual_vel = rpdo_mapped[0x606c][0];
        m_actual_vel_SI = double(m_actual_vel) / double(m_ppu); // convert velocity to user defined unit

        if (print_pdos)
        {
            std::cout << "<= rPDO2 [" << static_cast<int>(id()) << "]: Vel: " << std::dec << this->m_actual_vel << "  \tToruqe: " << this->m_current << std::endl;
            std::cout << "         [" << static_cast<int>(id()) << "]:[" << std::hex << 0x6077 << "][" << static_cast<int>(0) << "]: " << std::dec << this->m_current << std::endl;
            std::cout << "         [" << static_cast<int>(id()) << "]:[" << std::hex << 0x606c << "][" << static_cast<int>(0) << "]: " << std::dec << this->m_actual_vel << std::endl;
        }
    }
    else
    {
        // std::cout << "<= rPDO [" <<static_cast<int>(id())<< "]: idx:" << std::hex << idx << "   subidx: " <<  subidx << std::endl;
    }
}

/* */
void Node::OnSync(uint8_t cnt, const time_point &t) noexcept
{
}

/*  This task is reponsible for cyclic sending target positions or target velocities through PDO2 / PDO3
    This task is posted in in the OnBoot function */
void Node::Task_target() noexcept
{
    while (true)
    {
        if (parent_robot->operation_enabled & !m_configing)
        {
            if (m_current_operation_mode == 1) // Position Profile mode
            {
                m_control_word.bit4 = 1; // set new target bit (bit 4)
                m_control_word.bit5 = 1; // set immediately bit (bit 5)
                if (m_new_pos_targ_ready)
                {
                    if (m_target_pos != m_target_pos_prev)
                    {
                        tpdo_mapped[0x607A][0] = m_target_pos; // target position
                        tpdo_mapped[0x6040][0] = m_control_word.get();
                        tpdo_mapped[0x607A][0].WriteEvent(); // Trigger write events for PDO1.
                        m_target_pos_prev = m_target_pos;
                        m_new_pos_targ_ready = false;
                        m_status_word.bit10 = 0;

                        if (print_pdos)
                        {
                            std::cout << "=> tPDO2 [" << static_cast<int>(id()) << "]: target: " << std::dec << this->m_target_pos << std::endl;
                            std::cout << "         [" << static_cast<int>(id()) << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(m_control_word.get()) << std::hex << "   (0x" << m_control_word.get() << ")" << std::endl;
                        }
                    }
                }
                else
                {
                    parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                                  ": Motion controller is not ready to receive a new position target! (please wait!)",
                                              true);
                }
            }
            // velocity profile mode is not verified - may need some eimprovement
            else if (m_current_operation_mode == 3) // Velocity Profile mode
            {
                m_control_word.enable_operation = 1;           // set immediately bit (bit 5)
                tpdo_mapped[0x60FF][0] = m_target_vel;         // target velocity
                tpdo_mapped[0x6040][0] = m_control_word.get(); // enable
                tpdo_mapped[0x60FF][0].WriteEvent();           // Trigger write events for PDO1.
                                                               // std::cout << "target vel: " << vel << std::endl;
                                                               // if (flag_printlog)
                                                               // {
                                                               //     std::cout << "=> tPDO2 [" << static_cast<int>(id()) << "]: target: " << std::dec << vel << std::endl;
                                                               //     std::cout << "         [" << static_cast<int>(id()) << "]:[" << std::hex << 0x6040 << "][" << static_cast<int>(0) << "]: 0b " << ToBinaryString(ctrl) << std::hex << "   (0x" << ctrl << ")" << std::endl;
                                                               // }
            }
        }
        else
        {
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Robot is not operational",
                                      false);
        }

        Wait(AsyncWait(duration(std::chrono::milliseconds(m_sample_time))));
    }
}

/*  This task is reponsible for changing ccontroller configuration (currently setting zero positon).
    Other tasks should be halted using flag_config when this task is actioning*/
void Node::Task_config() noexcept
{
    while (true)
    {
        if (m_configing)
        {
            Node::Set_encoder(0);
            Node::Enable_operation(true);
            m_configing = false;
        }
        Wait(AsyncWait(duration(std::chrono::milliseconds(m_sample_time))));
    }
}

/* Sets target position in [m] or [rad] and coverts to motion controller unit
    and updates the associates variable to be axecutes in the next sample*/
void Node::Set_target_pos(double val)
{
    m_target_pos_SI = val;
    this->m_target_pos = static_cast<int32_t>(val * this->m_ppu);
}

/* Sets target valocit in [m/s] or [rad/s] and coverts motion controller unit
    and updates the associates variable to be axecutes in the next sample*/
void Node::Set_target_vel(double val)
{
    // this->vel_target = static_cast<int32_t>(val * 60 / gear_ratio);
    m_target_vel_SI = val;
    this->m_target_vel = static_cast<int32_t>(val * this->m_ppu);
}

/* accessor to motor current [A]*/
void Node::Get_current(int *currentPtr)
{
    *currentPtr = m_current_SI;
}

/* accessor toactual motor positon [m] or [rad]*/
void Node::Get_actual_pos(double *actualPosPtr)
{
    *actualPosPtr = m_actual_pos_SI;
}

/* accessor to actual motor velocity [m/s] or [rad/s]*/
void Node::Get_actual_vel(double *actualVelPtr)
{
    // *actualVelPtr = double(vel_actual / (60.0 / gear_ratio));
    *actualVelPtr = m_actual_vel_SI;
}

/* if target is reached */
bool Node::Is_reached()
{
    if (m_current_operation_mode == 1 || m_current_operation_mode == 3)
    {
        return m_status_word.bit10;
    }
    else
    {
        parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                      "Target reached is only defined for PP and PV modes ",
                                  true);
        return 0;
    }
}

/**/
void Node::Read_from_encoder_memory_file(const std::string &directory,
                                         double *encoder_memory,
                                         std::ofstream *encoder_memory_file)
{
    // check for encoder memory file to set
    std::stringstream file_name;
    file_name << directory << "encoder_memory_node_" << static_cast<int>(id()) << ".dat";

    if (std::filesystem::exists(file_name.str())) // Check if the file exists
    {
        // std::cout << "Node " << id << ": Encoder memory file axists" << std::endl;
        std::ifstream temp(file_name.str());
        if (temp.is_open())
        {
            try
            {
                std::string content;
                temp >> content;                      // Read the content as a string
                *encoder_memory = std::stod(content); // Convert the string to double
                parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                              ": Encoder memory = " + std::to_string(*encoder_memory),
                                          false);
                temp.close();
            }
            catch (const std::exception &e)
            {
                // Handle the exception here, e.g., log an error message
                parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                              ": Error reading encoder memory. Check memory files: " + e.what(),
                                          true);
                temp.close();
                std::exit(1); // Exit with error code 1
            }
        }
        else
        {
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          " :Unable to open encoder memory file to read",
                                      true);
            std::exit(1); // Exit with error code 1
        }

        (*encoder_memory_file).open(file_name.str(), std::ios::out);
        if (!(*encoder_memory_file).is_open())
        {
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          " :Unable to open encoder memory file to write",
                                      true);
            std::exit(1); // Exit with error code 1
        }
    }
    else
    {
        *encoder_memory = 0;
        (*encoder_memory_file).open(file_name.str(), std::ios::out);
        if ((*encoder_memory_file).is_open())
        {
            *encoder_memory_file << 0;
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          ": Memory file built and opened",
                                      false);
        }
        else
        {
            parent_robot->Log_message("Node " + std::to_string(static_cast<int>(id())) +
                                          " :Unable to open built encoder memory file to write",
                                      true);
            std::exit(1); // Exit with error code 1
        }
    }
}

/* convert binary to decimal*/
uint16_t bin2Dec(const std::string &binaryString)
{
    return std::bitset<16>(binaryString).to_ulong();
}

/* convert binary to string*/
template <typename T>
std::string ToBinaryString(T value)
{
    size_t numBits = sizeof(T) * 8;
    std::string binaryString = std::bitset<32>(value).to_string().substr(32 - numBits);
    // Insert spaces every 4 bits
    for (size_t i = binaryString.size() - 4; i > 0; i -= 4)
    {
        binaryString.insert(i, " ");
    }
    return binaryString;
}

/**/
ControlWord::ControlWord()
{
    this->controlword = 0x0000;
}

/* converts bits to int16_t*/
int16_t ControlWord::get()
{
    controlword = (switch_ON << 0) |
                  (enable_voltage << 1) |
                  (quick_stop << 2) |
                  (enable_operation << 3) |
                  (bit4 << 4) |
                  (bit5 << 5) |
                  (bit6 << 6) |
                  (fault_reset << 7) |
                  (halt << 8) |
                  (bit9 << 9) |
                  (bit10 << 10) |
                  (bit11 << 11) |
                  (bit12 << 12) |
                  (bit13 << 13) |
                  (bit14 << 14) |
                  (bit15 << 15);
    return controlword;
}

void ControlWord::set(int16_t input)
{
    switch_ON = (input & 0x0001) != 0;
    enable_voltage = (input & 0x0002) != 0;
    quick_stop = (input & 0x0004) != 0;
    enable_operation = (input & 0x0008) != 0;
    bit4 = (input & 0x0010) != 0;
    bit5 = (input & 0x0020) != 0;
    bit6 = (input & 0x0040) != 0;
    fault_reset = (input & 0x0080) != 0;
    halt = (input & 0x0100) != 0;
    bit9 = (input & 0x0200) != 0;
    bit10 = (input & 0x0400) != 0;
    bit11 = (input & 0x0800) != 0;
    bit12 = (input & 0x1000) != 0;
    bit13 = (input & 0x2000) != 0;
    bit14 = (input & 0x4000) != 0;
    bit15 = (input & 0x8000) != 0;
}

/**/
StatusWord::StatusWord()
{
    this->statusword = 0x0000;
    StatusWord::update(0x0000);
}

/* updates all bits */
void StatusWord::update(uint16_t newStatusWord)
{
    this->statusword = newStatusWord;
    ready_to_switch_ON = (statusword & 0x0001) != 0;
    switched_ON = (statusword & 0x0002) != 0;
    operation_enabled = (statusword & 0x0004) != 0;
    fault = (statusword & 0x0008) != 0;
    Voltage_Enabled = (statusword & 0x0010) != 0;
    Quick_Stop = (statusword & 0x0020) != 0;
    Switch_On_Disabled = (statusword & 0x0040) != 0;
    Warning = (statusword & 0x0080) != 0;
    // bit8 = (statusword & 0x0100) != 0;
    // bit9 = (statusword & 0x0200) != 0;
    bit10 = (statusword & 0x0400) != 0;
    Internal_Limit_Active = (statusword & 0x0800) != 0;
    bit12 = (statusword & 0x1000) != 0;
    bit13 = (statusword & 0x2000) != 0;
    // bit14 = (statusword & 0x4000) != 0;
    // bit15 = (statusword & 0x8000) != 0;
}

/* Member function that converts CiA402 code number to string message*/
std::string StatusWord::getCiA402StatusMessage()
{
    std::string message;
    if (operation_enabled)
    {
        message = "Operation enabled ";
    }
    else if (switched_ON)
    {
        message = "Switched On ";
    }
    else if (ready_to_switch_ON)
    {
        message = "Ready to Switch On ";
    }
    else if (Switch_On_Disabled)
    {
        message = "Switch on disabled ";
    }
    else
    {
        message = "Unkonwn CiA 402 status ";
    }

    if (fault)
    {
        message = "Fault | " + message;
    }

    if (Warning)
    {
        message = "Warning | " + message;
    }

    return message;
}
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

#pragma once

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/driver.hpp>
#include <lely/coapp/device.hpp>
#include <lely/coapp/node.hpp>
#include <lely/coapp/master.hpp>
#include <lely/util/result.hpp>
#include <lely/ev/future.hpp>
#include <iostream>
#include <thread>
#include <string>
#include <bitset>
#include <fstream>
#include <cmath>
#include <future>
#include <iomanip> // Include this header for std::hex
#include <unordered_map>

#include <filesystem>
#include <sstream>

#include "CiA301node.hpp"
#include "Robot.hpp"

using namespace std::chrono_literals;
using namespace lely;

uint16_t bin2Dec(const std::string &binaryString);
std::string dec2Bin(uint16_t decimalValue);
// std::string getCiA402StatusMessage(uint16_t value);
template <typename T>
std::string ToBinaryString(T value);
// bool fileExists(const std::string &filename);

class CTRobot;

struct ControlWord
{
public:
  ControlWord();
  int16_t get();
  void set(int16_t ctrlword);

  int16_t controlword;
  bool switch_ON : 1;        // bit 0
  bool enable_voltage : 1;   // bit 1
  bool quick_stop : 1;       // bit 2
  bool enable_operation : 1; // bit 3
  bool bit4 : 1;             // bit 4     Operation Mode Specific
  bool bit5 : 1;             // bit 5     Operation Mode Specific
  bool bit6 : 1;             // bit 6     Operation Mode Specific
  bool fault_reset : 1;      // bit 7
  bool halt : 1;             // bit 8
  bool bit9 : 1;             // bit 9     Change on set-point (only in Profile position mode)
  bool bit10 : 1;            // bit 10
  bool bit11 : 1;            // bit 11
  bool bit12 : 1;            // bit 12
  bool bit13 : 1;            // bit 13
  bool bit14 : 1;            // bit 14
  bool bit15 : 1;            // bit 15
};

struct StatusWord
{
public:
  StatusWord();
  // get status word on SDO and translate bits
  void update(uint16_t newStatusWord);
  std::string getCiA402StatusMessage();

  uint16_t statusword;
  bool ready_to_switch_ON : 1; // bit 0
  bool switched_ON : 1;        // bit 1
  bool operation_enabled : 1;  // bit 2
  bool fault : 1;              // bit 3
  bool Voltage_Enabled : 1;    // bit 4
  bool Quick_Stop : 1;         // bit 5
  bool Switch_On_Disabled : 1; // bit 6
  bool Warning : 1;            // bit 7
  // bool bit8 : 1;                  // bit 8
  // bool bit9 : 1;                  // bit 9
  bool bit10 : 1;                 // bit 10   Target Reached(PP)
  bool Internal_Limit_Active : 1; // bit 11
  bool bit12 : 1;                 // bit 12   Set-Point Acknowledge(PP)
  bool bit13 : 1;                 // bit 13
  // bool bit14 : 1;                 // bit 14
  // bool bit15 : 1;                 // bit 15
};

class Node : public canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;
  friend class CTRobot;

  Node(CTRobot *parent, ev_exec_t *exec, canopen::AsyncMaster &master, unsigned int id,
       double encoderRes, double gearRatio, double velocityFactor,
       unsigned int commandPeriod, int operationMode, double maxAcc, double maxVel);
  ~Node();
  void Set_target_pos(double val);
  void Set_target_vel(double val);
  void Set_operationMode(int val);
  void Get_current(int *motorCurrentPtr);
  void Get_actual_pos(double *actualPosPtr);
  void Get_actual_vel(double *actualVelPtr);
  bool Is_reached();

private:
  void OnBoot(canopen::NmtState st, char es,
              const std::string &what) noexcept override;
  void OnConfig(std::function<void(std::error_code ec)> res) noexcept override;
  void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override;
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  void OnSync(uint8_t cnt, const time_point &t) noexcept override;
  void Task_target() noexcept;
  void Task_config() noexcept;
  void Set_tpdo_transtype(int numPDO, int type);
  void Reset_fault();
  void Set_operation_mode(int operationMode);
  void Set_profile_params(int maxACC, int maxDCC, int maxSpeed);
  void Swtitch_ON();
  void Enable_operation(bool enable);
  void Set_encoder(double offset);
  void Read_from_encoder_memory_file(const std::string &directory,
                                     double *encoder_memory,
                                     std::ofstream *encoder_memory_file);

  CTRobot *parent_robot;
  bool print_pdos = false;

  bool m_new_pos_targ_ready = false;
  bool flag_operation_enabled = false;
  bool flag_robot_switched_ON = false;
  int m_configing = 0; // 1=set zero, 2=find limit
  bool flag_bootSuccess = false;
  bool m_tasks_posted = false;
  bool encoder_set = false;

  ControlWord m_control_word;          // Status words updates automatically on RPDO1 write
  StatusWord m_status_word;            // Status words updates automatically on RPDO1 write
  std::string m_controller_brand;      // MotionController brand -- "Maxon" or "Faulhaber"
  std::ofstream m_encoder_memory_file; // File to read from / write on encoder memory to keep track of positoin for future start ups
  double m_encoder_memory_value;       // Loaded previous encoder value from memory file
  double m_ppu;                        // pulse per unit (user defined unit for Faulhaber)
  unsigned int m_sample_time;
  int m_operation_mode;
  int m_current_operation_mode;

  bool m_encoder_memory_ready = false;
  bool m_new_target = false;

  int m_profile_acc;
  int m_profile_vel;

  double m_target_pos_SI;    // in user defined unit
  double m_target_vel_SI;    // in user defined unit
  double m_actual_pos_SI;    // in user defined unit
  double m_actual_vel_SI;    // in user defined unit
  double m_current_SI;       // in user defined unit
  int32_t m_target_pos;      // not in user defined unit
  int32_t m_target_pos_prev; // not in user defined unit
  int32_t m_target_vel;      // not in user defined unit
  int32_t m_actual_pos;      // not in user defined unit
  int32_t m_actual_vel;      // not in user defined unit
  int16_t m_current;         // current(Maxon) ot torque(Faulhaber)
  int16_t ctrlword;

  bool bit12_prev = 0;
};
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
#include <fstream>
#include <filesystem>
#include <sstream>
#include <memory>
#include <bitset>
#include "spdlog/spdlog.h"

#include "CiA301node.hpp"
#include "Robot.hpp"
#include "CanEssentials.hpp"
#include "SharedStates.hpp"

using namespace std::chrono_literals;
using namespace lely;

class Flags;

class Cia301Node : public canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;
  // Constructor and Destructor
  Cia301Node(ev_exec_t *exec,
             canopen::AsyncMaster &master,
             unsigned int id,
             std::string ControllerBrand,
             double encoderRes,
             double gearRatio,
             double VelocityFactor,
             unsigned int commandPeriod,
             OpMode operationMode,
             double maxAcc,
             double maxVel,
             //  double current_threshold,
             //  double vel_findlimit,
             std::shared_ptr<SharedState> sharedState,
             std::shared_ptr<spdlog::logger> shared_logger);
  ~Cia301Node();

  // ============================== Configuration Methods ==============================
  void setMaxTorque(const double negative, const double positive);
  void setProfileParams(const double acc, const double dcc, const double vel);
  void setEncoder(const double offset);
  void setOperationMode(const OpMode mode);
  void setHomeOffsetValue(const double val);
  StatusWord getStatusword() const;
  bool getFlags(const Flags::FlagIndex index) const;
  OpMode getOperationMode() const;
  void setPosLimit(const double min, const double max);

  // ============================== Command Methods ==============================
  void setPosAbs(const double val);
  void setPos(const double val);
  void setVel(const double val);
  void toZeroPos();
  void enableOperation(const bool enable);

  // ============================== Feedback Methods ==============================
  void getCurrent(double &motorCurrentPtr) const;
  void getCurrentAvg(double &motorCurrentPtr) const;
  void getPosAbs(double &actualPosPtr) const;
  void getPos(double &actualPosPtr) const;
  void getVel(double &actualVelPtr) const;
  bool isReached() const;
  void getPosLimit(double &min, double &max) const;

  // ============================== Status Methods ==============================
  int32_t getCpuTemp() const;
  int32_t getDriverTemp() const;
  std::bitset<32> getDigitalIn() const;

private:
  // ============================== CANopen Communication Methods ==============================
  void OnBoot(canopen::NmtState st, char es, const std::string &what) noexcept override;
  void OnConfig(std::function<void(std::error_code ec)> res) noexcept override;
  void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override;
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  void OnSync(uint8_t cnt, const time_point &t) noexcept override;
  void TaskTarget() noexcept;
  void TaskConfig() noexcept;
  void TaskOperation() noexcept;

  // ============================== CiA402 State Machine Methods ==============================
  void SwitchOn();
  void EnableOp_(const bool enable);
  void EnableOperationWithPdo_(const bool enable);
  void ResetFault_();
  void SetMaxTorque_(const double negative, const double positive);
  void SetOperationMode_(const OpMode operationMode);

  // ============================== Configuration Methods ==============================
  void SetTpdoTranstype_(const int numPDO, const int type);
  void SetProfileParams_(const int maxACC, const int maxDCC, const int maxSpeed);
  void SetEncoder_(const double offset);
  void FindFowrardLimits_();

  // ============================== Utility Methods ==============================
  void ReadEncoderMemFile(const std::string &directory, double &encoder_memory, std::ofstream &encoder_memory_file);

public:
private:
  static constexpr bool m_useEncoderMemory = false; // read and store encoder memory in a file?

  // ============================== Physical constants ==============================
  std::string m_nodeId;           // node ID string to facilitate logging
  std::string m_controller_brand; // MotionController brand -- "Maxon" or "Faulhaber"
  double m_ppu;                   // pulse per unit (user defined unit for Faulhaber)
  double m_gearRatio;             // rad->rev or m->rev
  double m_conversionFactor;
  int m_axisDir;

  // ============================ Configurable variables ============================
  unsigned int m_sampleTime;       // unit: [ms]
  int m_profile_acc;               // in SI unit
  int m_profile_vel;               // in SI unit  (only for position profile (PP) mode)
  double m_vel_findlimit;          // in SI unit
  double m_current_threshold;      // in mA or ??
  double m_pos_offset_SI = 0.0;    // in SI unit
  OpMode m_operation_mode;         // mode of operation
  OpMode m_current_operation_mode; // mode of operation
  int m_isConfiguring = 0;         // 1=set zero, 2=find limit

  // ======================== Command and feedback variables ========================
  double m_targetPosSi;                                                   // in SI unit
  double m_targetVelSi;                                                   // in SI unit
  double m_currentPosSi;                                                  // in SI unit
  double m_currentVelSi;                                                  // in SI unit
  double m_currentSi;                                                     // in SI unit
  std::deque<double> m_currentSiHist;                                     // history of current for averating
  int32_t m_targetPos;                                                    // in motion controller unit (pulse/) not in user defined unit
  int32_t m_targetPosPrev;                                                // in motion controller unit (pulse/) not in user defined unit
  int32_t m_targetVel;                                                    // in motion controller unit (pulse/) not in user defined unit
  int32_t m_currentPos;                                                   // in motion controller unit (pulse/) not in user defined unit
  int32_t m_currentVel;                                                   // in motion controller unit (pulse/) not in user defined unit
  int16_t m_current;                                                      // current(Maxon) ot torque(Faulhaber)
  blaze::StaticVector<int32_t, 2> m_PosLimit = {-2147483648, 2147483647}; // position limit
  blaze::StaticVector<double, 2> m_PosLimitSi;                            // position limit in SI unit
  blaze::StaticVector<int32_t, 2> m_currentPosLimit;                      // actual position limit
  blaze::StaticVector<double, 2> m_currentPosLimitSi;                     // actual position limit in SI unit
  int32_t m_temp_cpu;
  int32_t m_temp_power;
  std::bitset<32> m_digital_in;
  // int32_t m_digital_in;

  // ======================== Other variables ========================
  bool m_flag_target_task_processing;
  Flags m_flags;
  std::shared_ptr<SharedState> robot_states; // Shared all node states set by the master (Robot)
  std::shared_ptr<spdlog::logger> logger;    // Shared logger instance
  ControlWord m_controlWord;                 // Control words updates automatically on RPDO1 write
  StatusWord m_statusWord;                   // Status words updates automatically on RPDO1 write
  std::ofstream m_encoderMemFile;            // File to read from / write on encoder memory to keep track of positoin for future start ups
  double m_encoderMem;                       // Loaded previous encoder value from memory file
  bool m_printPdos = false;                  // for tracing debug
  bool m_bit12Prev = false;
  std::string m_commandMsg = "";
  blaze::StaticVector<double, 2> m_set_max_torque = blaze::StaticVector<double, 2>(0.0);
  int m_set_profile_vel, m_set_profile_acc, m_set_profile_dcc;
  double m_set_encoder;
  OpMode m_set_operation_mode;
};

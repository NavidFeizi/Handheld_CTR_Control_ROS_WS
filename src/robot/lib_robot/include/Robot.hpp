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

#pragma once

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/util/result.hpp>
#include <iostream>
#include <lely/ev/future.hpp>
#include <thread>
#include <string>
#include <bitset>
#include <fstream>
#include <cmath>
#include <future>
#include <deque>
#include <filesystem>

#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

#include "spdlog/spdlog.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>

#include <deque>
#include <numeric>
#include <optional>

#include "CiA301node.hpp"
#include "CanEssentials.hpp"
#include "Robot.hpp"
#include "SharedStates.hpp"

#include <cstring>
#include <csignal>

#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#define Sleep(x) usleep((x) * 1000)
#endif

#include <cmath>
#include <iostream>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>

class Cia301Node;

class Interface {
  public:
  Interface() : value(0) {}

      // Individual key setters
  void setKeyUp(bool pressed)       { bits.key_up = pressed; }
  void setKeyDown(bool pressed)     { bits.key_down = pressed; }
  void setKeyRight(bool pressed)    { bits.key_right = pressed; }
  void setKeyLeft(bool pressed)     { bits.key_left = pressed; }
  void setKeyForward(bool pressed)  { bits.key_forward = pressed; }
  void setKeyBackward(bool pressed) { bits.key_backward = pressed; }
  void setKeyCenter(bool pressed)   { bits.key_center = pressed; }

  // LED control
  void setLED1(bool on) { bits.led_1 = on; }
  void setLED2(bool on) { bits.led_2 = on; }

  // Individual key getters
  bool isKeyUp() const       { return bits.key_up; }
  bool isKeyDown() const     { return bits.key_down; }
  bool isKeyRight() const    { return bits.key_right; }
  bool isKeyLeft() const     { return bits.key_left; }
  bool isKeyForward() const  { return bits.key_forward; }
  bool isKeyBackward() const { return bits.key_backward; }
  bool isKeyCenter() const   { return bits.key_center; }

  // LED state
  bool isLED1On() const { return bits.led_1; }
  bool isLED2On() const { return bits.led_2; }

  private:
  union {
    struct {
      uint16_t key_up       : 1; // bit 0
      uint16_t key_down     : 1; // bit 1
      uint16_t key_right    : 1; // bit 2
      uint16_t key_left     : 1; // bit 3
      uint16_t key_forward  : 1; // bit 4
      uint16_t key_backward : 1; // bit 5
      uint16_t key_center   : 1; // bit 6
      uint16_t led_1        : 1; // bit 7
      uint16_t led_2        : 1; // bit 8
      uint16_t reserved     : 7; // bits 9–15 (reserved for future use)
    } bits;
    uint16_t value; // packed representation
  };

};

class CTRobot
{
public:
    CTRobot(bool position_limit);
    CTRobot();

    CTRobot(const CTRobot &rhs);
    ~CTRobot();

    void startRobotCommunication(int sample_time);
    void enableOperation(const bool enable);

    // ========================== Command and Feedback Methods ===========================
    void setTargetPos(const blaze::StaticVector<double, 4UL> &target);
    void setTargetVel(const blaze::StaticVector<double, 4UL> &target);
    blaze::StaticVector<double, 4> getCurrent() const;
    void getVel(blaze::StaticVector<double, 4UL> &val) const;
    void getPos(blaze::StaticVector<double, 4UL> &val) const;
    void getPosLimit(blaze::StaticVector<double, 4> &min, blaze::StaticVector<double, 4> &max) const;


    // ============================== Configuration Methods ==============================
    void setMaxTorque(const blaze::StaticVector<double, 4UL> negative,
                      const blaze::StaticVector<double, 4UL> positive);
    void setProfileParams(const blaze::StaticVector<double, 4UL> max_vel,
                          const blaze::StaticVector<double, 4UL> max_acc,
                          const blaze::StaticVector<double, 4UL> max_dcc);
    void setOperationMode(const OpMode mode);
    void setEncoders(const blaze::StaticVector<double, 4> val);
    bool getSwitchStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getSwitchStatus() const;
    blaze::StaticVector<bool, 4> getEnableStatus() const;
    blaze::StaticVector<bool, 4> getEncoderStatus() const;
    bool getDisabledStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getDisabledStatus() const;
    blaze::StaticVector<bool, 4> getReachedStatus() const;
    void setPosLimit(const blaze::StaticVector<double, 4> &min, const blaze::StaticVector<double, 4> &max) const;

    void getTemperature(blaze::StaticVector<int32_t, 4> &cpu, blaze::StaticVector<int32_t, 4> &driver) const;
    void getDigitalIn(blaze::StaticVector<std::bitset<32>, 4> &in) const;

    void getInterface() const;

    // ============================== Utility Methods ==============================
    void waitUntilReach(const std::atomic<bool>& cancel_flag) const;
    void waitUntilReach() const;
    void waitUntilTransReach(const std::atomic<bool>& cancel_flag) const;
    void waitUntilTransReach() const;
    void findTransEncoders();

    // bool m_boot_success = false;
    // bool m_flag_robot_switched_on = false;
    // bool m_flag_operation_enabled = false;
    // bool m_encoders_set;

protected:
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<SharedState> m_shared_state;

    std::shared_ptr<Cia301Node> m_inrTubeRot;
    std::shared_ptr<Cia301Node> m_inrTubeTrn;
    std::shared_ptr<Cia301Node> m_mdlTubeRot;
    std::shared_ptr<Cia301Node> m_mdlTubeTrn;

private:
    void startCANopenNodes();
    void convPosToRobotFrame(const blaze::StaticVector<double, 4UL> &posCurrent,
                             blaze::StaticVector<double, 4UL> &posInCTRFrame) const;
    int checkPosLimits(const blaze::StaticVector<double, 4UL> &posTarget) const;
    void initLogger();

    std::shared_ptr<Interface> m_input = std::make_shared<Interface>();

    bool m_flagHeadAttached;

    // the real encoder count and gear ratio are integrated into
    // motion controller factors. these are just a factor
    // to convert 0.01 mm to mm and 0.1 deg to deg
    // pulse per mm/deg - conversion to SI unit is done internally in the node class
    static constexpr blaze::StaticVector<double, 4UL> m_encodersResolution = {100 * 180 * M_1_PI, 1000 * 1000.0, 100 * 180 * M_1_PI, 1000 * 1000.0};
    static constexpr blaze::StaticVector<double, 4UL> m_velocityFactors = {10.0, 10.0, 10.0, 10.0};
    static constexpr blaze::StaticVector<double, 4UL> m_gearRatios = {1, 1, 1, 1}; 

    std::thread m_thread;
    
    
    unsigned int m_sampleTime; // commandPeriod [ms]
    OpMode operation_mode;

    blaze::StaticVector<double, 4UL> m_maxAcc; 
    blaze::StaticVector<double, 4UL> m_maxVel;
    blaze::StaticVector<double, 4UL> m_lowerBounds;
    blaze::StaticVector<double, 4UL> m_upperBounds;
    blaze::StaticVector<double, 4UL> m_posOffsets;
    double m_minClearance;
    double m_maxClearance;
    bool m_flagPositionLimit;

    // std::ofstream logFile;

};

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
    void getCurrent(blaze::StaticVector<double, 4UL> &val) const;
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
    void setLinearEncoders(const double inner, const double middle);
    bool getSwitchStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getSwitchStatus() const;
    bool getEnableStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getEnableStatus() const;
    bool getEncoderStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getEncoderStatus() const;
    bool getDisabledStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getDisabledStatus() const;
    bool getReachedStatus(blaze::StaticVector<bool, 4> &status) const;
    bool getReachedStatus() const;
    void setPosLimit(const blaze::StaticVector<double, 4> &min, const blaze::StaticVector<double, 4> &max);


    // ============================== Utility Methods ==============================
    void waitUntilReach();
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

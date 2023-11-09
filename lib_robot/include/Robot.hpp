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

#include "CiA301node.hpp"
// #include "MotionContollerNode.hpp"
#include "Robot.hpp"

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

class Node;

class CTRobot
{

public:
    CTRobot(int sample_time,
            int operation_mode,
            std::vector<double> max_acc,
            std::vector<double> max_vel,
            bool position_limit);
    CTRobot(const CTRobot &rhs);
    ~CTRobot();
    bool Get_Controller_Switch_Status();
    void Start_Thread();
    void Enable_Operation(bool enable);
    void Set_Zero_Position(std::vector<double> offset);
    void Find_Fowrard_Limits();
    void Set_Target_AbsPosition(std::vector<double> posTarget);
    void Set_Target_Velocity(std::vector<double> velTarget);
    void Get_Current(std::vector<int> *p_current);
    void Get_Velocity(std::vector<double> *p_velCurrent);
    void Get_Position(std::vector<double> *p_posCurrent);
    void Get_PosVelCur(std::vector<double> *p_posCurrent,
                       std::vector<double> *p_velCurrent,
                       std::vector<int> *p_current);
    bool Get_reachStatus();
    void Wait_until_reach();
    void Log_message(const std::string &message, bool print);

    bool m_boot_success = false;
    bool flag_robot_switched_ON = false;
    bool operation_enabled = false;
    bool encoders_set;

private:
    void Fiber_loop();
    void Convert_pos_to_CTR_frame(std::vector<double> &posCurrent, std::vector<double> *posInCTRFrame);
    int Position_limits_check(std::vector<double> posTarget);

    std::shared_ptr<Node> inner_rot;
    std::shared_ptr<Node> inner_tran;
    std::shared_ptr<Node> middle_rot;
    std::shared_ptr<Node> middle_tran;
    std::thread EnableThread;

    std::ofstream logFile;

    unsigned int sample_time; // commandPeriod [ms], minimum
    int operation_mode;
    std::vector<double> encoder_res;
    std::vector<double> velocity_factor;
    std::vector<double> gear_ratio;
    std::vector<double> max_acc; // deg->rev or mm->rev
    std::vector<double> max_vel;

    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;
    std::vector<double> posOffsets;
    double clearance_min;
    double clearance_max;
    bool flag_position_limit;

protected:
};

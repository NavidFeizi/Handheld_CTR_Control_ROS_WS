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
            blaze::StaticVector<double, 4UL> max_acc,
            blaze::StaticVector<double, 4UL> max_vel,
            bool position_limit);
    CTRobot(const CTRobot &rhs);
    ~CTRobot();
    bool Get_Controller_Switch_Status();
    void Start_Thread();
    void Enable_Operation(bool enable);
    void Set_Zero_Position(blaze::StaticVector<double, 4UL> offset);

    void set_target_position(blaze::StaticVector<double, 4UL> posTarget);
    void Set_Target_Velocity(blaze::StaticVector<double, 4UL> velTarget);
    void Get_Current(blaze::StaticVector<int, 4UL> *p_current);
    void Get_Velocity(blaze::StaticVector<double, 4UL> *p_velCurrent);
    void Get_Position(blaze::StaticVector<double, 4UL> *p_posCurrent);
    void Get_PosVelCur(blaze::StaticVector<double, 4UL> *p_posCurrent,
                       blaze::StaticVector<double, 4UL> *p_velCurrent,
                       blaze::StaticVector<int, 4UL> *p_current);
    bool Get_reachStatus();
    void Wait_until_reach();
    void Log_message(const std::string &message, bool print);

    bool m_boot_success = false;
    bool m_flag_robot_switched_on = false;
    bool m_flag_operation_enabled = false;
    bool m_encoders_set;

private:
    void Fiber_loop();
    void Convert_pos_to_CTR_frame(blaze::StaticVector<double, 4UL> &posCurrent, blaze::StaticVector<double, 4UL> *posInCTRFrame);
    int Position_limits_check(blaze::StaticVector<double, 4UL> posTarget);

    bool check_all_nodes_switched_on();
    bool check_all_nodes_enabled();
    bool check_all_encoders_set();

    std::shared_ptr<Node> m_inner_rot;
    std::shared_ptr<Node> m_inner_tran;
    std::shared_ptr<Node> m_middle_rot;
    std::shared_ptr<Node> m_middle_tran;
    std::thread EnableThread;

    std::ofstream logFile;

    unsigned int sample_time; // commandPeriod [ms], minimum
    int operation_mode;
    blaze::StaticVector<double, 4UL> encoder_res;
    blaze::StaticVector<double, 4UL> velocity_factor;
    blaze::StaticVector<double, 4UL> gear_ratio;
    blaze::StaticVector<double, 4UL> m_max_acc; // deg->rev or mm->rev
    blaze::StaticVector<double, 4UL> m_max_vel;

    blaze::StaticVector<double, 4UL> lowerBounds;
    blaze::StaticVector<double, 4UL> upperBounds;
    blaze::StaticVector<double, 4UL> posOffsets;
    double clearance_min;
    double clearance_max;
    bool flag_position_limit;

protected:
};

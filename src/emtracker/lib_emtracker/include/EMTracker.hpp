//==========================================================================================================
// 			This code is a part of the concentric tube robot & robot actuated catheter project
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

#include "EMTracker.hpp"
#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"
#include "RigidTransformation.hpp"
#include "Butterworth.hpp"

#include <thread>
#include <atomic>
#include <filesystem>
#include <chrono>
#include <memory>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <yaml-cpp/yaml.h>

/* This class establishes a connection with the NDI Aurora EM tracker, performs landmark registration,
   and reads from previously saved registration information. The class provides the ability to read
   data from the EM tracker on a separate thread and perform the transformation of the tip sensor
   into the robot frame without blocking the program. */

struct SensorConfig
{
  std::string serial_number;
  std::string srom_filename;
  std::string tran_filename;
  bool active = false;
  bool load_srom;
  bool load_tran;
  int probe_handle_num;
};

class Recorder
{
public:
  Recorder(const std::string &filename);

  void Record(const quatTransformation &Reference,
              const quatTransformation &Tip);

  void Close();

private:
  std::string filename_;
  std::ofstream file;
  std::chrono::high_resolution_clock::time_point start_time_;
};

class EMTracker
{
public:
  // EMTracker default constructor
  EMTracker(const std::string &hostname, double sample_time, double cutoff_freq, bool flag_print);
  // // EMTracker overloaded constructor
  // EMTracker(std::string hostname);
  // EMTracker desctructor
  ~EMTracker();
  // copy constructor
  EMTracker(const EMTracker &rhs);
  // move constructor
  EMTracker(EMTracker &&rhs) noexcept;

  void landmark_registration(const std::string &landmarks_file_name, std::string ref_sensor_name);
  void start_read_thread();
  void stop_read_thread();
  void get_tool_transform_in_robot(quatTransformation &transform);
  void get_tool_transform_in_phantom(quatTransformation &transform);
  void get_tool_transform_in_robot_dot(quatTransformation &transform_dot);
  void get_tool_transform_in_em(quatTransformation &transform);
  void get_robot_transform_in_em(quatTransformation &transform);
  void get_robot_transform_in_phantom(quatTransformation &transform);
  void get_probe_transform_in_robot(quatTransformation &transform);
  void get_probe_transform_in_phantom(quatTransformation &transform);
  void get_sample_time(double &SampleTime);
  void set_filter_params(double fc_ref, double fc_tools);

private:
  std::shared_ptr<Recorder> m_recorder;
  std::shared_ptr<CombinedApi> m_combinedAPI;
  std::vector<ToolData> enabledTools;
  std::map<std::string, SensorConfig> sensorConfigMap;
  std::vector<std::string> srom_paths;
  bool m_flag_debug, m_flag_record, m_flag_filter = false;
  std::string m_reference_trans_csv_path, m_tool_trans_csv_path;
  std::thread m_emThread;
  std::atomic<bool> stopFlag; // Flag to control the thread

  quatTransformation m_transform_0_1;      // Transformation from EM frame to CTR Robot frame (ctr base)
  quatTransformation m_transform_0_2;      // Transformation from EM frame to tool frame
  quatTransformation m_transform_0_3;      // Transformation from EM frame to phantom CT frame
  quatTransformation m_transform_0_6;      // Transformation from EM frame to probe
  quatTransformation m_transform_1_2;      // Transformation from CTR Robot frame to tool tip frame
  quatTransformation m_transform_3_2;      // Transformation from phantom CT frame to tool tip frame
  quatTransformation m_transform_3_1;      // Transformation from phantom CT frame to CTR Robot frame
  quatTransformation m_transform_3_6;      // Transformation from phantom CT frame to Probe
  quatTransformation m_transform_1_6;      // Transformation CTR Robot frame to Probe
  quatTransformation m_transform_1_2_prev; // transformation from the system frame to the tool tip (not tool sensor)
  quatTransformation m_transform_dot_1_2;  // derivative of Transformation from CTR Robot frame to tool tip frame

  blaze::StaticVector<double, 3UL> m_tran_vel_2_4;

  std::string m_config_Dir;
  double m_measured_sample_time;
  unsigned int num_landmark;
  int m_num_active_sensors = 0;
  std::vector<quatTransformation> m_sec_transforms;

  std::unique_ptr<ButterworthFilter<3UL>> m_filter_tran_robot, m_filter_tran_tool, m_filter_tran_probe, m_filter_tran_phantom;
  std::unique_ptr<ButterworthFilter<4UL>> m_filter_rot_robot, m_filter_rot_tool, m_filter_rot_probe, m_filter_rot_phantom;

  std::string getToolInfo(std::string toolHandle);
  void onErrorPrintDebugmessage(const std::string &methodName, int errorCode);
  void initializeAndEnableSensors();
  void matchSensors();
  void LoadToolDefinitions2Ports(bool load_all);
  void Read_Loop();
  void ToolData2Vector(const ToolData &toolData, std::vector<double> &toolCoord);
  void ToolData2QuatTransform(const ToolData &input, quatTransformation &output);
  void log_tansformation(const int number, const blaze::StaticVector<double, 3UL> &translation, const blaze::StaticVector<double, 4UL>& rotation, const double sample_time) const;

  /** @brief Checks if all distances are less than a threshold.
    The latest position is assumed to be the center of the sphere.
    @return true if all points are within the sphere with the given radius. */
  bool points_in_sphere(const std::vector<blaze::StaticVector<double, 3UL>> &points_list, const double &radius);
  /* Save collected lanmark to a CSV file with X,Y,Z header */
  void save_landmarks_to_csv(const std::vector<blaze::StaticVector<double, 3>> &data, const std::string &filepath);
  /* Load lanmark from a CSV file with X,Y,Z header */
  void load_landmarks_from_csv(const std::string &filename, std::vector<blaze::StaticVector<double, 3UL>> *output);
  /* Save QuatTransformation data to a CSV file with headers */
  void save_transformation_to_csv(const quatTransformation &data, const std::string &filename);
  /* Load QuatTransformation data from a CSV file */
  bool load_transformation_from_csv(const std::string &filename, quatTransformation &data);
  /* Calcualtes average of each colums */
  void column_average(const std::vector<blaze::StaticVector<double, 3UL>> &data, blaze::StaticVector<double, 3UL> &columnAverages);

  int read_sensor_config_from_yaml(const std::string &file_path, std::map<std::string, SensorConfig> &sensorConfig_map);
};

/* Sleep! */
void SleepSeconds(const unsigned numSeconds);

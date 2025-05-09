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
#include <cstdlib> // For system()

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
  std::string probe_handle;
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

  /**
   * @brief Monitors and captures the probe tip position to determine measured landmarks and calculates a rigid transformation
   *        from the reference sensor frame to the system frame.
   *
   * @details
   * 1. Save the true values of the landmarks (in system frame) in "conf/Landmarks_Truth.csv" with the header X, Y, Z.
   * 2. The reference sensor should be connected to channel 1, and the probe should be connected to channel 2.
   * 3. Ensure the motor section of the robot is not in the EM field to avoid non-uniform DC offset in the EM readings,
   *    which significantly reduces registration error.
   * 4. The code compensates for the motion of the robot frame, but it's better to keep the robot stationary due to potential
   *    interference of the motors with the EM frame during movement.
   * 5. Based on the number of landmarks saved in "conf/Landmarks_Truth.csv," touch each landmark in order with the probe tip
   *    and remain stationary. When 200 consecutive samples are captured within a 1 mm radius sphere, the function saves the
   *    measured landmark value in "conf/Landmark_X.csv" and guides you to move to the next landmark. There is a 5-second wait
   *    between each landmark. Only when 200 stationary consecutive samples are captured does the function proceed to the next
   *    landmark.
   * 6. After all landmarks are saved, the average of each landmark is calculated and saved in "conf/Landmarks_Measured.csv."
   * 7. The rigid transformation from the reference sensor frame to the system frame is saved in "conf/Reference2CTR_Transformation.csv."
   * 8. You can manually copy the transformation to NDI Cygna6D software to save the equivalent .rom format for future applications.
   *
   * @param landmarks_file_name The name of the CSV file that includes the true landmarks' location. This file must be in the config folder.
   */
  void landmark_registration(const std::string &landmarks_file_name, std::string ref_sensor_name);

  /**
   * @brief Starts the read loop in a separate thread to avoid blocking the program.
   *
   * @details This method creates a new thread that runs the EMTracker::Read_Loop() function,
   *          allowing the main program to continue running without being blocked by the read operations.
   */
  void start_read_thread();

  /**
   * @brief Stops the read loop thread safely.
   *
   * @details This method sets a stop flag to true, signaling the read loop to terminate.
   *          If the read loop thread is joinable, it waits for the thread to finish before proceeding.
   */
  void stop_read_thread();

  /**
   * @brief Freeze the phantom.
   */
  void freeze_phantom(bool status);


  /**
   * @brief Retrieves the tool sensor transformation in the EM tracker frame.
   * @param transform Reference to a quatTransformation object to store the tool sensor transformation.
   */
  int get_tool_transform_in_em(quatTransformation &transform);

  /**
   * @brief Retrieves the transformation from the system frame to the EM frame (EM frame in systems frame).
   * @param transform Reference to a quatTransformation object to store the system to EM frame transformation.
   */
  int get_robot_transform_in_em(quatTransformation &transform);
  

  int get_phantom_transform_in_em(quatTransformation &transform);


  int get_usprobe_transform_in_em(quatTransformation &transform);


  int get_probe_transform_in_em(quatTransformation &transform);


  /**
   * @brief Retrieves the tool transformation in the system (CTR or catheter robot) frame.
   * @param transform Reference to a quatTransformation object to store the tool transformation.
   */
  void get_tool_transform_in_robot(quatTransformation &transform);

  /**
   * @brief Retrieves the tool transformation in the phantom frame.
   * @param transform Reference to a quatTransformation object to store the tool transformation.
   */
  void get_tool_transform_in_phantom(quatTransformation &transform);

  /**
   * @brief Retrieves the derivative of the tool transformation in the system (CTR or catheter robot) frame.
   * @param transform_dot Reference to a quatTransformation object to store the tool transformation derivative.
   */
  void get_tool_transform_in_robot_dot(quatTransformation &transform_dot);

  /**
   * @brief Retrieves the transformation from the system base frame to the phantom frame.
   * @param transform Reference to a quatTransformation object to store the system to EM frame transformation.
   */
  void get_robot_transform_in_phantom(quatTransformation &transform);

  /**
   * @brief Retrieves the probe position in the robot frame.
   * @param transform Reference to a quatTransformation object to store the probe transformation.
   */
  void get_probe_transform_in_robot(quatTransformation &transform);

  /**
   * @brief Retrieves the probe position in the phantom frame.
   * @param transform Reference to a quatTransformation object to store the probe transformation.
   */
  void get_probe_transform_in_phantom(quatTransformation &transform);

  /**
   * @brief Retrieves sample time.
   */
  void get_sample_time(double &SampleTime);

  /**
   * @brief Sets the filter parameters for the reference and tools filters.
   *
   * @details This function updates the filter coefficients for both translation and rotation filters
   *          based on the provided cutoff frequencies for the reference and tools.
   *
   * @param fc_ref The cutoff frequency for the reference filter in Hz.
   * @param fc_tools The cutoff frequency for the tools filter in Hz.
   */
  void set_filter_params(double fc_ref, double fc_tools);

private:
  std::shared_ptr<Recorder> m_recorder;
  std::shared_ptr<CombinedApi> m_combinedAPI;
  std::vector<ToolData> enabledTools;
  std::map<std::string, SensorConfig> m_sensorConfigMap;
  std::vector<std::string> srom_paths;
  bool m_flag_debug, m_flag_record, m_flag_filter, m_flag_freeze_phantom = false;
  std::string m_reference_trans_csv_path, m_tool_trans_csv_path;
  std::thread m_emThread;
  std::atomic<bool> stopFlag; // Flag to control the thread

  quatTransformation m_transform_0_1;      // Transformation from EM frame to CTR Robot frame (ctr base)
  quatTransformation m_transform_0_2;      // Transformation from EM frame to tool frame
  quatTransformation m_transform_0_3;      // Transformation from EM frame to phantom CT frame
  quatTransformation m_transform_0_4;      // Transformation from EM frame to us probe frame
  quatTransformation m_transform_0_5;      // Transformation from EM frame to us calibration probe frame
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
  std::vector<std::string> port_handle_unknown_sensors;

  std::unique_ptr<ButterworthFilter<3UL>> m_filter_tran_robot, m_filter_tran_tool, m_filter_tran_probe, m_filter_tran_phantom;
  std::unique_ptr<ButterworthFilter<4UL>> m_filter_rot_robot, m_filter_rot_tool, m_filter_rot_probe, m_filter_rot_phantom;

  std::string getToolInfo(std::string toolHandle);

  /**
   * @brief Prints a debug message if a method call fails.
   *
   * @details To use, pass the method name and the error code returned by the method.
   *          Example: onErrorPrintDebugmessage("capi.initialize()", capi.initialize());
   *          If the call succeeds, this method does nothing.
   *          If the call fails, this method prints an error message to stdout.
   *
   * @param methodName The name of the method that was called.
   * @param errorCode The error code returned by the method.
   */
  void onErrorPrintDebugmessage(const std::string &methodName, int errorCode);

  /**
   * @brief Initialize and enable loaded sensors.
   * @details This is the same regardless of sensor type.
   */
  void initializeAndEnableSensors();

  /**
   * @brief Matches the serial number of the sensors (from soldered SROM) with the serial numbers
   *        in "config.yaml" to name the sensors based on the "config.yaml" file.
   */
  void matchSensors();

  /**
   * @brief Loads the SROM to the sensors with serial numbers mentioned in the "config.yaml" file.
   *
   * @details EM tracker must be reinitialized and tracking must be started again after calling this function.
   */
  void LoadToolDefinitions2Ports(bool load_all);

  /**
   * @brief Loads tool definition files and reads data from the tip sensor and the reference sensor to calculate the transformation
   *        of the tip in the robot's frame.
   *
   * @details This function converts the rotations to Euler angles and updates the values in tipRelTran_ptr.
   *          It is recommended to call this function from a separate thread to avoid blocking the main code.
   */
  void Read_Loop();

  /**
   * @brief Converts sensor position and orientation from ToolData type to a std::vector of translation
   *        and rotation quaternion types.
   *
   * @details This function checks if the tool data transform is missing and sets the corresponding
   *          values in the vector to 0.0 if it is missing. Otherwise, it extracts the translation and
   *          rotation values from the tool data and stores them in the vector.
   *
   * @param toolData The ToolData object containing the sensor position and orientation data.
   * @param toolCoord Reference to a std::vector<double> to store the converted translation and rotation values.
   */
  void ToolData2Vector(const ToolData &toolData, std::vector<double> &toolCoord);

  /**
   * @brief Converts sensor position and orientation from ToolData type to QuatTransformation type.
   *
   * @details This function checks if the tool data transform is not missing and if its status is enabled.
   *          If so, it extracts the rotation (quaternion) and translation values from the tool data and
   *          stores them in the output QuatTransformation object. If the transform is missing or not enabled,
   *          the rotation and translation are set to NaN.
   *
   * @param input The ToolData object containing the sensor position and orientation data.
   * @param output Reference to a quatTransformation object to store the converted transformation values.
   */
  void ToolData2QuatTransform(const ToolData &input, quatTransformation &output);

  // log
  void log_tansformation(const int number, const blaze::StaticVector<double, 3UL> &translation, const blaze::StaticVector<double, 4UL> &rotation, const double sample_time) const;

  /** @brief Checks if all distances are less than a threshold.
    The latest position is assumed to be the center of the sphere.
    @return true if all points are within the sphere with the given radius. */
  bool points_in_sphere(const std::vector<blaze::StaticVector<double, 3UL>> &points_list, const double &radius);

  /**
   * @brief Saves collected landmarks to a CSV file with X, Y, Z headers.
   *
   * @param data A vector of 3D points representing the collected landmarks.
   * @param filepath The path to the CSV file where the data will be saved.
   */
  void save_landmarks_to_csv(const std::vector<blaze::StaticVector<double, 3>> &data, const std::string &filepath);

  /**
   * @brief Loads landmarks from a CSV file with X, Y, Z headers.
   *
   * @param filepath The path to the CSV file to be read.
   * @param data A pointer to a vector of 3D points to store the loaded landmarks.
   */
  void load_landmarks_from_csv(const std::string &filename, std::vector<blaze::StaticVector<double, 3UL>> *output);

  /**
   * @brief Saves QuatTransformation data to a CSV file with headers.
   *
   * @param data The QuatTransformation object containing the transformation data.
   * @param filename The path to the CSV file where the data will be saved.
   */
  void save_transformation_to_csv(const quatTransformation &data, const std::string &filename);

  /* Load QuatTransformation data from a CSV file */
  bool load_transformation_from_csv(const std::string &filename, quatTransformation &data);

  /**
   * @brief Calculates the average of each column in a vector of 3D points.
   *
   * @param data A vector of 3D points from which to calculate the column averages.
   * @param columnAverages Reference to a blaze::StaticVector<double, 3> to store the calculated averages.
   */
  void column_average(const std::vector<blaze::StaticVector<double, 3UL>> &data, blaze::StaticVector<double, 3UL> &columnAverages);

  /**
   * @brief Reads sensor configuration from a YAML file and populates a map with the configuration data.
   *
   * @param file_path The path to the YAML file containing the sensor configuration.
   * @param sensorConfig_map Pointer to a map to store the sensor configuration data.
   *
   * @return 0 on success, -1 on failure.
   */
  int read_sensor_config_from_yaml(const std::string &file_path, std::map<std::string, SensorConfig> &sensorConfig_map);
};

/* Sleep! */
void SleepSeconds(const unsigned numSeconds);

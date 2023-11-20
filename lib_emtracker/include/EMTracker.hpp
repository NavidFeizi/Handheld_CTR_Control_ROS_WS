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
#include "KalmanFilter.hpp"
// #include "KalmanFilter.h"
#include <thread>
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
#include <boost/qvm/all.hpp>
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
  bool active = false;
  bool load_srom;
  int probleHandle_num;
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
  EMTracker();
  // // EMTracker overloaded constructor
  // EMTracker(std::string hostname);
  // EMTracker desctructor
  ~EMTracker();
  // copy constructor
  EMTracker(const EMTracker &rhs);
  // move constructor
  EMTracker(EMTracker &&rhs) noexcept;

  /** @brief This function monitors and captures the probe tip position to determine measured
   *        landmarks and calculates a rigid transformation from the reference sensor frame to the CTR frame.
   * @details
   * 1. Save the true values of the landmarks (in CTR frame) in "conf/Landmarks_Truth.csv" with the header X, Y, Z.
   * 2. The reference sensor should be connected to channel 1, and the probe should be connected to channel 2.
   * 3. Make sure the motor section of the robot is not in the EM field; otherwise, there will be a non-uniform DC offset
   *    in the EM readings, which significantly reduces the registration error.
   * 4. The code compensates for the motion of the robot frame, but it is better to keep the robot stationary
   *    because of the potential interference of the motors with the EM frame in case of movement.
   * 5. Based on the number of landmarks saved in "conf/Landmarks_Truth.csv," you should touch the first to the
   *    last landmark accodring to the order of the truth list with the tip of the probe and stay stationary.
   *    When 200 consecutive samples are captured in a sphere with
   *    a radius of 1 mm, the function saves the measured value of the landmark in "conf/Landmark_X.csv" and
   *    guides you to move to the next landmark. There is a 5-second wait between each landmark. Keep in mind
   *    that landmark positions are saved only when 200 stationary consecutive samples are captured. So, if you move
   *    the probe in space, or if there is noise in the EM readings, the function will not proceed to the next
   *    landmark.
   * 6. When all landmarks are saved, the average of each landmark is calculated and saved in "conf/Landmarks_Measured.csv."
   * 7. The rigid transformation from the reference sensor frame to the CTR frame is saved in "conf/Reference2CTR_Transformation.csv."
   * 8. You can manually copy the transformation to NDI Cygna6D software to save the equivalent .rom format for future applications.*/
  void Landmark_Registration();
  /** @brief Call EMTracker::Read_Loop() from a separate thread to avoid blocking the program. */
  void Start_Read_Thread();

  /**/
  void Get_TipPosition(blaze::StaticVector<double, 3UL> *Translation, blaze::StaticVector<double, 3UL> *Translation_flt);

  void Get_Probe_Position(blaze::StaticVector<double, 3UL> *Translation, blaze::StaticVector<double, 3UL> *Translation_flt);

private:
  std::shared_ptr<Recorder> recorder;
  std::shared_ptr<KalmanFilter> KLF_tip;
  std::shared_ptr<KalmanFilter> KLF_probe;
  std::shared_ptr<CombinedApi> combinedAPI;
  std::vector<ToolData> enabledTools;
  std::map<std::string, SensorConfig> sensorConfigMap;
  std::vector<std::string> srom_paths;
  bool flag_load_sroms;
  bool flag_print = false;
  std::thread emThread;
  std::vector<double> Tran_Tip_Rel;
  std::vector<double> Tran_Tip_Rel_flt;
  std::vector<double> Tran_Tip_Abs;
  std::vector<double> Tran_Probe;
  std::vector<double> Tran_Probe_flt;
  std::string config_Dir;
  unsigned int num_landmark;

  std::string getToolInfo(std::string toolHandle);
  void onErrorPrintDebugmessage_1(std::string methodName, int errorCode);
  void InitializeAndEnableSensors();
  void MatchSensors();
  void LoadToolDefinitions2Ports();
  void Read_Loop();
  void ToolData2Vector(const ToolData &toolData, std::vector<double> *toolCoord);
  void ToolData2QuatTransform(const ToolData &input, quatTransformation *output);
};



/** @brief Checks if all distances are less than a threshold.
  The latest position is assumed to be the center of the sphere.
  @return true if all points are within the sphere with the given radius. */
bool Points_in_Sphere(const std::vector<boost::qvm::vec<double, 3>> &history, const double &threshold);
/* Save collected lanmark to a CSV file with X,Y,Z header */
void Save_Landmarks_To_CSV(const std::vector<std::vector<double>> &data, const std::string &filepath);
/* Load lanmark from a CSV file with X,Y,Z header */
void Load_Landmarks_From_CSV(const std::string &filename, std::vector<std::vector<double>> *output);
/* Save QuatTransformation data to a CSV file with headers */
void Save_QuatTransformation_To_CSV(const quatTransformation &data, const std::string &filename);
/* Load QuatTransformation data from a CSV file */
bool Load_QuatTransformation_From_CSV(const std::string &filename, quatTransformation *data);
/* Calcualtes average of each colums */
void Column_Average(const std::vector<std::vector<double>> &data, std::vector<double> *columnAverages);
/* Converts boost::qvm:;vec position vector to stc::vector */
std::vector<double> qvmVec2StdVec(const boost::qvm::vec<double, 3> &quatVec);
/* Converts stc::vector of boost::qvm:;vec position vector to std::vector of stc::vector */
std::vector<std::vector<double>> boostVec2StdVec(const std::vector<boost::qvm::vec<double, 3>> &boostVecVector);
/* Sleep! */
void SleepSeconds(unsigned numSeconds);

int Read_SensorConfig_from_YAML(const std::string &yamlFilePath, std::map<std::string, SensorConfig> *sensorConfigMap);

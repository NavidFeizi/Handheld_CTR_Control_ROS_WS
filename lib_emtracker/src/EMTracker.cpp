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

#include "EMTracker.hpp"

Recorder::Recorder(const std::string &filename) : filename_(filename)
{
  // Open the CSV file and write headers
  file.open(filename);
  if (file.is_open())
  {
    file << "Time,X_[ref],Y_[ref],Z_[ref],Q0_[ref],Qx_[ref],Qy_[ref],QZ_[ref]"
         << ","
         << "X_[tip],Y_[tip],Z_[tip],Q0_[tip],Qx_[tip],Qy_[tip],Qz_[tip]" << std::endl;
    start_time_ = std::chrono::high_resolution_clock::now(); // Record start time
  }
  else
  {
    std::cerr << "Error opening file to record: " << filename << std::endl;
  }
}

void Recorder::Record(const quatTransformation &Reference,
                      const quatTransformation &Tip)
{
  if (file.is_open())
  {
    auto current_time = std::chrono::high_resolution_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();

    file << elapsed_seconds << ","
         << boost::qvm::X(Reference.translation) << "," << boost::qvm::Y(Reference.translation) << "," << boost::qvm::Z(Reference.translation) << ","
         << boost::qvm::S(Reference.rotation) << "," << boost::qvm::X(Reference.rotation) << "," << boost::qvm::Y(Reference.rotation) << "," << boost::qvm::Z(Reference.rotation) << ","
         << boost::qvm::X(Tip.translation) << "," << boost::qvm::Y(Tip.translation) << "," << boost::qvm::Z(Tip.translation) << ","
         << boost::qvm::S(Tip.rotation) << "," << boost::qvm::X(Tip.rotation) << "," << boost::qvm::Y(Tip.rotation) << "," << boost::qvm::Z(Tip.rotation) << std::endl;
  }
  else
  {
    // std::cerr << "Error: File not open for writing." << std::endl;
  }
}

void Recorder::Close()
{
  file.close();
}

/* overloaded contructor */
/** @brief class establishes a connection with the NDI Aurora EM tracker, performs landmark registration,
   and reads from previously saved registration information. The class provides the ability to read
   data from the EM tracker with the maximum posible frequency (~40Hz)on a separate thread and
   perform the transformation of the tip sensor into the robot frame without blocking the program. */
EMTracker::EMTracker()
{
  std::string filename = "Recordings";
  if (!std::filesystem::exists(Log_directory))
  {
    std::filesystem::create_directory(Log_directory);
  }
  this->recorder = std::make_shared<Recorder>(Log_directory + filename + ".csv");
  // relative transformation of the tip pos in the CTR frame
  this->Tran_Tip_Rel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->Tran_Tip_Rel_flt = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->Tran_Tip_Abs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->Tran_Probe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->Tran_Probe_flt = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // print the reading in the read loop or not?
  this->flag_print = false;
  this->flag_load_sroms = true;
  // directory to save/load registration files
  this->config_Dir = config_Directory;

  // connection paramteres
  static bool useEncryption = false;
  static std::string cipher = "";

  // initiate the NDI library object
  this->combinedAPI = std::make_shared<CombinedApi>();
  // Attempt to connect to the device
  std::string hostname = "/dev/ttyUSB0";
  if (combinedAPI->connect(hostname, useEncryption ? Protocol::SecureTCP : Protocol::TCP, cipher) != 0)
  {
    // Print the error and exit if we can't connect to a device
    std::cout << "Connection Failed!" << std::endl;
    std::cout << "Press Enter to continue...";
    std::cin.ignore();
  }
  std::cout << "Connected!" << std::endl;
  // Wait a second - needed to support connecting to LEMO Vega
  SleepSeconds(1);
  this->combinedAPI->initialize();
  // Once loaded or detected, tools are initialized and enabled the same way
  EMTracker::InitializeAndEnableSensors();
  // If "config.yaml" exists and loaded, sroms loads according to "config.yaml"
  if (!Read_SensorConfig_from_YAML(this->config_Dir + "config.yaml", &sensorConfigMap))
  {
    // match the sensors' serial number with the serial numbers in sensorConfigMap
    EMTracker::MatchSensors();
  }
  else
  {
    std::cerr << "No sensor matching information" << std::endl;
  }
  std::cout << "EM Tracker initialized" << std::endl;

  // Parameters for Kalman Filterring
  this->KLF_tip = std::make_shared<KalmanFilter>(1.00, 1.00);
  this->KLF_probe = std::make_shared<KalmanFilter>(1.00, 1.00);
}

/* Destructor */
EMTracker::~EMTracker()
{
  std::cout << "EMTracker destructor called" << std::endl;
  this->recorder->Close();
  // this->combinedAPI->stopStreaming(); // USTREAM and free any open streams before closing the _connection
  this->combinedAPI->stopTracking();
  if (emThread.joinable())
  {
    emThread.join();
  }
}

/* Copy constructor */
EMTracker::EMTracker(const EMTracker &rhs) : combinedAPI(rhs.combinedAPI){};

/* move constructor */
EMTracker::EMTracker(EMTracker &&rhs) noexcept
{
  // handling self assignment
  if (this != &rhs)
  {
    this->combinedAPI = std::move(rhs.combinedAPI);
  }
}

/* Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output */
std::string EMTracker::getToolInfo(std::string toolHandle)
{
  // Get the port handle info from PHINF
  PortHandleInfo info = this->combinedAPI->portHandleInfo(toolHandle);
  // Return the ID and SerialNumber the desired string format
  std::string outputString = info.getToolId();
  outputString.append(" s/n:").append(info.getSerialNumber());
  return outputString;
}

/** @brief Prints a debug message_1 if a method call failed.
 * @details To use, pass the method name and the error code returned by the method.
 *          Eg: onErrorPrintDebugmessage_1("capi.initialize()", capi.initialize());
 *          If the call succeeds, this method does nothing.
 *          If the call fails, this method prints an error message_1 to stdout.
 */
void EMTracker::onErrorPrintDebugmessage_1(std::string methodName, int errorCode)
{
  if (errorCode < 0)
  {
    std::cout << methodName << " failed: " << this->combinedAPI->errorToString(errorCode) << std::endl;
  }
}

/** @brief Initialize and enable loaded sensors. This is the same regardless of sensor type. */
void EMTracker::InitializeAndEnableSensors()
{
  // portHandles of non-0initialized ports (sensors)
  std::vector<PortHandleInfo> portHandles = this->combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
  for (int i = 0; i < static_cast<int>(portHandles.size()); i++)
  {
    std::cout << "Initializing portHandle #" << i << std::endl;
    // initialize and enbale active ports (sensors)
    onErrorPrintDebugmessage_1("capi.portHandleInitialize()", this->combinedAPI->portHandleInitialize(portHandles[i].getPortHandle()));
    onErrorPrintDebugmessage_1("capi.portHandleEnable()", this->combinedAPI->portHandleEnable(portHandles[i].getPortHandle()));
  }
  for (int i = 0; i < static_cast<int>(portHandles.size()); i++)
  {
    std::vector<PortHandleInfo> portHandles = this->combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    auto PortHandleInf = combinedAPI->portHandleInfo(portHandles[i].getPortHandle());
    std::cout << "--------------------------------------------------- \n"
              << "PortHandle #" << i << "   -   S/N:" << PortHandleInf.getSerialNumber() << "\n"
              << "--------------------------------------------------- " << std::endl;
  }
}

/** @brief  This function matches the serial number of the sensors (from soldered srom) with the
 * serial numbers in "config.yaml" to name the sensors based on the "config.yaml" file.
 */
void EMTracker::MatchSensors()
{
  // portHandles of the enabled ports (sensors)
  std::vector<PortHandleInfo> portHandles = this->combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
  for (int i = 0; i < static_cast<int>(portHandles.size()); i++)
  {
    auto PortHandleInf = this->combinedAPI->portHandleInfo(portHandles[i].getPortHandle());
    std::string portSerialNumber = PortHandleInf.getSerialNumber();
    // Iterate through the map to find the matching serial number
    for (auto &entry : sensorConfigMap)
    {
      if (entry.second.serial_number == portSerialNumber)
      {
        std::cout << "----------------------------------------------------------------- \n"
                  << "Sensor name: \"" << entry.first << "\" found\n"
                  << "S/N: " << portSerialNumber << "\n"
                  << "PortHandle #" << i << std::endl;
        entry.second.probleHandle_num = i; // Set probleHandle_num to i
        entry.second.active = true;
      }
    }
  }
  std::cout << "Sensors serial number matching done" << std::endl;
}

/** @brief Loads the SROM to the sensors with serial numbers mentioned in "config.yaml" file
 *  @details EM tracker must be reinitialized and tracking must be started again after cal to this function*/
void EMTracker::LoadToolDefinitions2Ports()
{
  // portHandles of the enabled ports (sensors)
  std::vector<PortHandleInfo> portHandles = this->combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
  // Iterate through the map to find the matching serial number
  for (auto &entry : sensorConfigMap)
  {
    if (entry.second.active && entry.second.load_srom && !entry.second.srom_filename.empty())
    {
      std::cout << "----------------------------------------------------------------- \n"
                << "Loading Tool Definition: \n"
                << "Sensor name: \"" << entry.first << "\" found\n"
                << "S/N: " << entry.second.serial_number << "\n"
                << "PortHandle #" << entry.second.probleHandle_num << "\n"
                << "ROM file name: " << entry.second.srom_filename << "\"" << std::endl;
      this->combinedAPI->loadSromToPort(this->config_Dir + entry.second.srom_filename, std::stoi(portHandles[entry.second.probleHandle_num].getPortHandle(), 0, 16));
    }
    else
    {
      std::cout << "No Tool Definition file for this sensor" << std::endl;
    }
  }
}

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
void EMTracker::Landmark_Registration()
{
  std::cout << " ### Ladmarks capturing mode ### " << std::endl;
  // check if probe and reference sensor are connected
  if (!(sensorConfigMap["probe"].active) || !(sensorConfigMap["reference"].active))
  {
    if (!sensorConfigMap["probe"].active)
    {
      std::cout << "ERROR: Probe is not connected" << std::endl;
    }
    if (!sensorConfigMap["reference"].active)
    {
      std::cout << "ERROR: Reference sensor is not connected" << std::endl;
    }
    std::cout << "calibration bypassed" << std::endl;
    return;
  }

  EMTracker::onErrorPrintDebugmessage_1("capi.startTracking()", this->combinedAPI->startTracking());

  // Start tracking
  EMTracker::onErrorPrintDebugmessage_1("capi.startTracking()", this->combinedAPI->startTracking());
  std::cout << "Tracking started" << std::endl;

  std::vector<ToolData> sensors_data;
  // 0->EM frame, 1->Reference EM frame, 2-> CTR frame, 3->Probe or CTR tip
  quatTransformation tran_01; // transformation from EM frame to Refernece EM frame
  quatTransformation tran_03; // transformation from EM frame to Probe or CTR tip
  quatTransformation tran_13; // transformation from Reference EM frame to Probe or CTR tip
  quatTransformation tran_12; // transformation from Reference EM frame to CTR frame
  quatTransformation tran_23; // transformation from CTR frame to ctr tip or probe
  quatTransformation tran_01_inv;
  quatTransformation tran_12_inv;
  std::vector<boost::qvm::vec<double, 3>> pointBuffer(200); // Create a buffer to store the past 200 samples
  std::vector<std::vector<double>> landmarks_truth;
  // load truth landmark postion from csv file
  Load_Landmarks_From_CSV(this->config_Dir + "Landmarks_Truth.csv", &landmarks_truth);
  this->num_landmark = landmarks_truth.size();

  std::vector<std::vector<double>> landmarks_measured(6, std::vector<double>(3, 0.0));

  std::cout << "Waiting..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  double threshold = 1; // threshold value for the radius of the error sphere [mm]
  unsigned int i = 0;   // counter for the number of landmark

  // look at probe tip for stationary instances to record landmark positon
  while (true)
  {
    sensors_data = combinedAPI->getTrackingDataBX();
    EMTracker::ToolData2QuatTransform(sensors_data[sensorConfigMap["reference"].probleHandle_num], &tran_01);
    EMTracker::ToolData2QuatTransform(sensors_data[sensorConfigMap["probe"].probleHandle_num], &tran_03);
    Inverse_Quat_Transformation(tran_01, &tran_01_inv);
    // calculate fransformation of the probe in the reference frame.
    Combine_Quat_Transformation(tran_01_inv, tran_03, &tran_13);
    // Add the current sample to the buffer
    pointBuffer.push_back(tran_13.translation);
    // Remove the oldest sample if the buffer size exceeds 200
    if (pointBuffer.size() > 200)
    {
      pointBuffer.erase(pointBuffer.begin());
    }
    // Check if all distances are less than the threshold
    bool status = Points_in_Sphere(pointBuffer, threshold);
    if (pointBuffer.size() == 200 && status)
    {
      // Save the landsmark cartesian position to a CSV file
      // std::string filename = config_Dir + "Landmark_" + std::to_string(i + 1) + ".csv";
      // Save_Landmarks_To_CSV(boostVec2StdVec(pointBuffer), filename);
      Column_Average(boostVec2StdVec(pointBuffer), &landmarks_measured[i]);
      pointBuffer.clear();
      std::cout << "Landmark #" << i + 1 << " saved. Please go to the next Landmark." << std::endl;
      i++;
      if (i >= this->num_landmark)
      {
        std::cout << "---- All landmarks captured ----" << std::endl;
        std::cout << "---- Closing ----" << std::endl;
        break;
      }
      // wait for the operator to move the porobe to the next landmark
      std::cout << "Waiting..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    std::cout << "Probe in reference frame:    "
              << "X: " << boost::qvm::X(tran_13.translation) << "    "
              << "Y: " << boost::qvm::Y(tran_13.translation) << "    "
              << "Z: " << boost::qvm::Z(tran_13.translation) << "    "
              << "Buffer size: " << pointBuffer.size() << std::endl;
  }
  // saved average of each landmark in a CSV
  Save_Landmarks_To_CSV(landmarks_measured, config_Dir + "Landmarks_Measured.csv");

  // // load measured landmarks saved in the previous step and calculate average -- uncomment if you want to bypass capuring landmark data ans use the already saved ones
  // Load_Landmarks_From_CSV(conf_Dir + "Landmarks_Measured.csv", &landmarks_measured);

  // calculate the transformation from EM reference sensor to CTR frame
  Calculate_Transformation(landmarks_measured, landmarks_truth, &tran_12);
  // save the calculated quaterion transformation in csv format
  Save_QuatTransformation_To_CSV(tran_12, config_Dir + "Reference2CTR_Transformation.csv");
  std::cout << "---- Quaternion transformation saved! ----" << std::endl;
}

/** @brief Call EMTracker::Read_Loop() from a separate thread to avoid blocking the program. */
void EMTracker::Start_Read_Thread()
{
  emThread = std::thread(&EMTracker::Read_Loop, this);
}

/** @brief Load Tool Definitio files and Read data from the tip sensor and the reference sensor to calculate the transformation
 *  of the tip in the robot's frame.
 *  @details This function converts the rotations to Euler angles and updates the values in tipRelTran_ptr.
 *  It is recommended to call this function from a separate thread to avoid blocking the main code.*/
void EMTracker::Read_Loop()
{
  // check if reference and tip sensors are connected
  if (!(sensorConfigMap["tip"].active) || !(sensorConfigMap["reference"].active))
  {
    if (!sensorConfigMap["tip"].active)
    {
      std::cout << "ERROR: Tip sensor is not connected" << std::endl;
    }
    if (!sensorConfigMap["reference"].active)
    {
      std::cout << "ERROR: Reference sensor is not connected" << std::endl;
    }
    std::cout << "Read loop bypassed" << std::endl;
    return;
  }

  // load ToolDefinition files
  EMTracker::LoadToolDefinitions2Ports();
  EMTracker::InitializeAndEnableSensors();
  EMTracker::onErrorPrintDebugmessage_1("capi.startTracking()", this->combinedAPI->startTracking());

  std::cout << "Tracking started" << std::endl;
  SleepSeconds(1);

  std::vector<ToolData> sensors_data;
  // initialize transformation structs
  // 0->EM frame, 1->Reference EM frame, 2-> CTR frame, 3->Probe or CTR tip
  quatTransformation tran_01; // transformation from EM frame to Refernece EM frame
  quatTransformation tran_03; // transformation from EM frame to Probe or CTR tip
  quatTransformation tran_13; // transformation from Reference EM frame to Probe or CTR tip
  quatTransformation tran_12; // transformation from Reference EM frame to CTR frame
  quatTransformation tran_23; // transformation from CTR frame to ctr tip or probe
  quatTransformation tran_01_inv;
  quatTransformation tran_12_inv;
  quatTransformation tran_0probe; // transformation from EM frame to Probe
  quatTransformation tran_1probe; // transformation from Reference EM frame to Probe
  // Create a buffer to store the past 200 samples
  std::vector<boost::qvm::vec<double, 3>> pointBuffer(100);

  double azimuth, elevation, roll;
  std::vector<double> vec;
  std::vector<double> vec_probe;
  std::vector<double> vec_probe_flt;

  // load calculated transformation in case srom is not used for EM reference transformation to
  // the robot's frame
  if (!this->flag_load_sroms)
  {
    Load_QuatTransformation_From_CSV(config_Dir + "Reference2CTR_Transformation.csv", &tran_12);
  }

  std::cout << "Reading EM sensors..." << std::endl;
  while (true)
  {
    sensors_data = combinedAPI->getTrackingDataBX();
    EMTracker::ToolData2QuatTransform(sensors_data[sensorConfigMap["reference"].probleHandle_num], &tran_01);
    EMTracker::ToolData2QuatTransform(sensors_data[sensorConfigMap["tip"].probleHandle_num], &tran_03);

    // EMTracker::ToolData2QuatTransform(sensors_data[0], &tran_01);
    // EMTracker::ToolData2QuatTransform(sensors_data[1], &tran_03);

    Inverse_Quat_Transformation(tran_01, &tran_01_inv);
    Combine_Quat_Transformation(tran_01_inv, tran_03, &tran_13);

    if ((sensorConfigMap["probe"].active))
    {
      EMTracker::ToolData2QuatTransform(sensors_data[sensorConfigMap["probe"].probleHandle_num], &tran_0probe);
      Combine_Quat_Transformation(tran_01_inv, tran_0probe, &tran_1probe);
      vec_probe = qvmVec2StdVec(tran_1probe.translation);

      this->KLF_tip->Loop(vec_probe, &vec_probe_flt);

      this->Tran_Probe[0] = vec_probe[0];
      this->Tran_Probe[1] = vec_probe[1];
      this->Tran_Probe[2] = vec_probe[2];

      this->Tran_Probe_flt[0] = vec_probe_flt[0];
      this->Tran_Probe_flt[1] = vec_probe_flt[1];
      this->Tran_Probe_flt[2] = vec_probe_flt[2];
    }

    // If an SROM for the transformation from the reference sensor to the CTR frame is loaded,
    // there is no need for further transformation. Otherwise, quaternion transformation data
    // from the CSV file will be used to perform the transformation manually.
    if (this->flag_load_sroms)
    {
      tran_23 = tran_13;
    }
    else
    {
      Inverse_Quat_Transformation(tran_12, &tran_12_inv);
      Combine_Quat_Transformation(tran_12_inv, tran_13, &tran_23);
    }

    // update the calculated transformation values
    QuaternionToEuler(tran_23.rotation, &azimuth, &elevation, &roll);
    vec = qvmVec2StdVec(tran_23.translation);
    this->Tran_Tip_Rel[0] = vec[0];
    this->Tran_Tip_Rel[1] = vec[1];
    this->Tran_Tip_Rel[2] = vec[2];
    this->Tran_Tip_Rel[3] = azimuth;
    this->Tran_Tip_Rel[4] = elevation;
    this->Tran_Tip_Rel[5] = roll;

    std::vector<double> vec_ftl(3, 0.0);
    this->KLF_tip->Loop(vec, &vec_ftl);

    this->Tran_Tip_Rel_flt[0] = vec_ftl[0];
    this->Tran_Tip_Rel_flt[1] = vec_ftl[1];
    this->Tran_Tip_Rel_flt[2] = vec_ftl[2];

    // // update the calculated transformation values
    // QuaternionToEuler(tran_03.rotation, &azimuth, &elevation, &roll);
    // vec = qvmVec2StdVec(tran_03.translation);
    // this->Tran_Tip_Abs[0] = vec[0];
    // this->Tran_Tip_Abs[1] = vec[1];
    // this->Tran_Tip_Abs[2] = vec[2];
    // this->Tran_Tip_Abs[3] = azimuth;
    // this->Tran_Tip_Abs[4] = elevation;
    // this->Tran_Tip_Abs[5] = roll;

    this->recorder->Record(tran_01, tran_03);

    // print the transformation
    if (this->flag_print)
    {
      std::cout << "X: " << boost::qvm::X(tran_23.translation) << "   "
                << "Y: " << boost::qvm::Y(tran_23.translation) << "   "
                << "Z: " << boost::qvm::Z(tran_23.translation) << "   "
                << "q0: " << boost::qvm::S(tran_23.rotation) << "   "
                << "qX: " << boost::qvm::X(tran_23.rotation) << "   "
                << "qY: " << boost::qvm::Y(tran_23.rotation) << "   "
                << "qZ: " << boost::qvm::Z(tran_23.rotation) << std::endl;
    }
  }
}

/** @brief reg tip positon in the CTR frame*/
void EMTracker::Get_TipPosition(blaze::StaticVector<double, 3UL> *Translation, blaze::StaticVector<double, 3UL> *Translation_flt)
{
  (*Translation)[0] = this->Tran_Tip_Rel[0];
  (*Translation)[1] = this->Tran_Tip_Rel[1];
  (*Translation)[2] = this->Tran_Tip_Rel[2];

  (*Translation_flt)[0] = this->Tran_Tip_Rel_flt[0];
  (*Translation_flt)[1] = this->Tran_Tip_Rel_flt[1];
  (*Translation_flt)[2] = this->Tran_Tip_Rel_flt[2];
}

/** @brief probe positon in the CTR frame*/
void EMTracker::Get_Probe_Position(blaze::StaticVector<double, 3UL> *Translation, blaze::StaticVector<double, 3UL> *Translation_flt)
{
  (*Translation)[0] = this->Tran_Probe[0];
  (*Translation)[1] = this->Tran_Probe[1];
  (*Translation)[2] = this->Tran_Probe[2];

  (*Translation_flt)[0] = this->Tran_Probe_flt[0];
  (*Translation_flt)[1] = this->Tran_Probe_flt[1];
  (*Translation_flt)[2] = this->Tran_Probe_flt[2];
}

/** @brief Converts sensor position and orientation from ToolData type to std::vector of translation
 * and rotation quaternion types. */
void EMTracker::ToolData2Vector(const ToolData &toolData, std::vector<double> *toolCoord)
{
  *toolCoord = {
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.tx,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.ty,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.tz,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.q0,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.qx,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.qy,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.qz,
      (toolData.transform.isMissing()) ? 0.0 : toolData.transform.error};
}

/** @brief Converts sensor position and orientation from ToolData type to QuatTransformation type */
void EMTracker::ToolData2QuatTransform(const ToolData &input, quatTransformation *output)
{
  if (!input.transform.isMissing())
  {
    output->rotation = boost::qvm::quat<double>({input.transform.q0, input.transform.qx, input.transform.qy, input.transform.qz});
    output->translation = boost::qvm::vec<double, 3>({input.transform.tx, input.transform.ty, input.transform.tz});
  }
}

/** @brief Checks if all distances are less than a threshold.
  The latest position is assumed to be the center of the sphere.
  @return true if all points are within the sphere with the given radius. */
bool Points_in_Sphere(const std::vector<boost::qvm::vec<double, 3>> &points_list, const double &radius)
{

  boost::qvm::vec<double, 3> referenceSample = points_list[0];
  for (size_t i = 0; i < points_list.size(); ++i)
  {
    double distance;
    const boost::qvm::vec<double, 3> translation1 = points_list[i];
    const boost::qvm::vec<double, 3> translation2 = referenceSample;
    Euclidean_Distance(translation1, translation2, &distance);

    // If any distance is greater than or equal to the threshold, return false
    if (distance >= radius)
    {
      return false;
    }
  }
  // If all distances are less than the threshold, return true
  return true;
}

/* Save collected lanmark to a CSV file with X,Y,Z header */
void Save_Landmarks_To_CSV(const std::vector<std::vector<double>> &data, const std::string &filepath)
{
  std::filesystem::path filePath = std::filesystem::path(filepath);

  // Ensure the directory exists; create it if it doesn't.
  if (!std::filesystem::exists(filePath.parent_path()))
  {
    try
    {
      std::filesystem::create_directories(filePath.parent_path());
    }
    catch (const std::exception &e)
    {
      std::cerr << "Failed to create the directory: " << e.what() << std::endl;
      return;
    }
  }

  std::ofstream file(filepath); // Pass the filename as a string
  if (!file.is_open())
  {
    std::cerr << "Failed to open file for writing." << std::endl;
    return;
  }

  file << "X"
       << ","
       << "Y"
       << ","
       << "Z"
       << "\n";
  for (const auto &sample : data)
  {
    file << sample[0] << "," << sample[1] << "," << sample[2] << "\n";
  }

  file.close();
}

/* Load lanmark from a CSV file with X,Y,Z header */
void Load_Landmarks_From_CSV(const std::string &filepath, std::vector<std::vector<double>> *data)
{
  // Clear the output vector to start fresh
  data->clear();

  // Open the CSV file
  std::ifstream file(filepath);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file for reading: " << filepath << std::endl;
    return; // Return without modifying the output vector
  }

  std::string line;
  bool firstLine = true; // Flag to skip the first line

  while (std::getline(file, line))
  {
    if (firstLine)
    {
      // Skip the first line (header)
      firstLine = false;
      continue;
    }

    std::vector<double> row; // This vector stores the X, Y, Z values for each line

    std::istringstream ss(line);
    std::string valueStr;
    while (std::getline(ss, valueStr, ','))
    {
      double value;
      if (std::istringstream(valueStr) >> value)
      {
        row.push_back(value);
      }
      else
      {
        std::cerr << "Failed to parse a value: " << valueStr << std::endl;
      }
    }

    (*data).push_back(row);
  }

  file.close(); // Close the file
}

/* Save QuatTransformation data to a CSV file with headers */
void Save_QuatTransformation_To_CSV(const quatTransformation &data, const std::string &filename)
{
  // Open the CSV file for writing
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }

  // Write the CSV headers
  file << "X,Y,Z,Q0,Qx,Qy,Qz";
  if (data.error != 0.0)
  {
    file << ",Error"; // Include 'Error' header if error member is present
  }
  file << std::endl;

  // Write data to the CSV file
  file << boost::qvm::X(data.translation) << "," << boost::qvm::Y(data.translation) << "," << boost::qvm::Z(data.translation) << ","
       << boost::qvm::S(data.rotation) << "," << boost::qvm::X(data.rotation) << "," << boost::qvm::Y(data.rotation) << "," << boost::qvm::Z(data.rotation);

  if (data.error != 0.0)
  {
    file << "," << data.error; // Include error value if it is present
  }
  file << std::endl;

  // Close the file
  file.close();
}

/* Load QuatTransformation data from a CSV file */
bool Load_QuatTransformation_From_CSV(const std::string &filename, quatTransformation *data)
{
  // Open the CSV file for reading
  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file for reading: " << filename << std::endl;
    return false;
  }

  // Define variables to store CSV data
  std::string line;
  std::vector<std::string> headers;
  std::vector<std::string> values;

  // Read the headers (first line) from the CSV file
  if (std::getline(file, line))
  {
    // Tokenize the headers
    std::istringstream headerStream(line);
    std::string header;
    while (std::getline(headerStream, header, ','))
    {
      headers.push_back(header);
    }
  }
  else
  {
    std::cerr << "Failed to read CSV headers." << std::endl;
    file.close();
    return false;
  }

  // Ensure that the headers include the required fields
  if (headers.size() < 7)
  {
    std::cerr << "CSV file is missing required fields." << std::endl;
    file.close();
    return false;
  }

  // Read the values (data) from the CSV file
  if (std::getline(file, line))
  {
    // Tokenize the values
    std::istringstream valueStream(line);
    std::string value;
    while (std::getline(valueStream, value, ','))
    {
      values.push_back(value);
    }
  }
  else
  {
    std::cerr << "Failed to read CSV data." << std::endl;
    file.close();
    return false;
  }

  // Check if there is an error value
  if (values.size() == 7)
  {
    // If no error value, set it to 0
    data->error = 0.0;
  }
  else if (values.size() == 8)
  {
    // If there is an error value, parse it
    data->error = std::stod(values[7]);
  }
  else
  {
    std::cerr << "Unexpected number of values in the CSV file." << std::endl;
    file.close();
    return false;
  }

  // Parse and assign the translation and rotation components
  try
  {
    data->translation = boost::qvm::vec<double, 3>{
        std::stod(values[0]),
        std::stod(values[1]),
        std::stod(values[2])};
    data->rotation = boost::qvm::quat<double>{
        std::stod(values[3]),
        std::stod(values[4]),
        std::stod(values[5]),
        std::stod(values[6])};
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to parse QuatTransformation data from CSV: " << e.what() << std::endl;
    file.close();
    return false;
  }

  // Close the file
  file.close();

  return true;
}

/* Calcualtes average of each colums */
void Column_Average(const std::vector<std::vector<double>> &data, std::vector<double> *columnAverages)

{
  if (data.empty() || data[0].empty())
  {
    std::cerr << "Input data is empty or has empty columns." << std::endl;
  }

  size_t numRows = data.size();
  size_t numCols = (data[0]).size();
  // std::vector<double> columnAverages(numCols, 0.0);

  for (size_t col = 0; col < numCols; ++col)
  {
    double columnSum = 0.0;
    for (size_t row = 0; row < numRows; ++row)
    {
      columnSum += data[row][col];
    }
    (*columnAverages)[col] = columnSum / numRows;
  }
}

/* Converts boost::qvm:;vec position vector to stc::vector */
std::vector<double> qvmVec2StdVec(const boost::qvm::vec<double, 3> &quatVec)
{
  std::vector<double> stdVec(3, 0.0);
  stdVec[0] = boost::qvm::X(quatVec);
  stdVec[1] = boost::qvm::Y(quatVec);
  stdVec[2] = boost::qvm::Z(quatVec);
  return stdVec;
}

/* Converts stc::vector of boost::qvm:;vec position vector to std::vector of stc::vector */
std::vector<std::vector<double>> boostVec2StdVec(const std::vector<boost::qvm::vec<double, 3>> &boostVecVector)
{
  std::vector<std::vector<double>> stdVecVector;
  for (unsigned int i = 0; i < boostVecVector.size(); i++)
  {
    auto stdVec = qvmVec2StdVec(boostVecVector[i]);
    stdVecVector.push_back(stdVec);
  }
  return stdVecVector;
}

/*  */
int Read_SensorConfig_from_YAML(const std::string &yamlFilePath, std::map<std::string, SensorConfig> *sensorConfigMap)
{
  // Load the YAML file
  try
  {
    YAML::Node config = YAML::LoadFile(yamlFilePath);
    // Iterate through the YAML data
    for (const auto &sensorEntry : config["sensor_config"])
    {
      for (YAML::const_iterator it = sensorEntry.begin(); it != sensorEntry.end(); ++it)
      {
        SensorConfig sensorConfig;
        sensorConfig.serial_number = it->second["serial_number"].as<std::string>();
        if (it->second["srom_filename"])
        {
          sensorConfig.srom_filename = it->second["srom_filename"].as<std::string>();
        }
        else
        {
          sensorConfig.srom_filename = "null"; // Provide a default value
        }
        sensorConfig.load_srom = it->second["load_srom"].as<bool>();
        (*sensorConfigMap)[it->second["name"].as<std::string>()] = sensorConfig;
      }
    }
    return 0;
  }
  catch (const YAML::Exception &e)
  {
    std::cout << "Could not load " << yamlFilePath << std::endl;
    return -1;
  }
}

/* Sleep! */
void SleepSeconds(unsigned numSeconds)
{
#ifdef _WIN32
  Sleep((DWORD)1000 * numSeconds); // Sleep(ms)
#else
  sleep(numSeconds); // sleep(sec)
#endif
}

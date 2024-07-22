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

// Save the original settings of cout
std::ios_base::fmtflags originalFlags = std::cout.flags();
std::streamsize originalPrecision = std::cout.precision();

/* overloaded contructor */
/**
 * @brief Establishes a connection with the NDI Aurora EM tracker, performs landmark registration,
 *        and reads from previously saved registration information. The class provides the ability
 *        to read data from the EM tracker with the maximum possible frequency (~66Hz) on a separate
 *        thread and perform the transformation of the tip sensor into the robot frame without blocking
 *        the program.
 *
 * @param hostname The hostname or address of the NDI Aurora EM tracker. Example: "/dev/ttyUSB0".
 * @param sample_time The sampling time for data acquisition. Must align with the NDI lib mode. Sample_time = 0.015 or 0.025
 * @param cutoff_freq The cutoff frequency for the tools filter.
 * @param flag_debug Flag to enable or disable printing in the read loop.
 */
EMTracker::EMTracker(const std::string &hostname, double filter_sample_time, double cutoff_freq, bool flag_debug)
    : m_flag_debug(flag_debug)
{
  m_config_Dir = CONFIG_DIRECTORY;
  m_combinedAPI = std::make_shared<CombinedApi>();

  m_flag_filter = false;

  // Filter parameters
  double fc_ref = 0.50;
  double fc_tools = cutoff_freq;

  // Static connection parameters
  static const bool useEncryption = false;
  static const std::string cipher = "";

  // Attempt to connect to the device
  if (m_combinedAPI->connect(hostname, useEncryption ? Protocol::SecureTCP : Protocol::TCP, cipher) != 0)
  {
    std::cerr << "Connection Failed!" << std::endl;
    std::cerr << "Press Enter to continue...";
    std::cin.ignore();
  }
  else
  {
    std::cout << "Connected!" << std::endl;
  }

  // Wait for the device to be ready
  SleepSeconds(1);
  m_combinedAPI->initialize();
  SleepSeconds(2);

  // Initialize and enable sensors
  initializeAndEnableSensors();

  // Load sensor configuration if available, otherwise match sensors manually
  if (!EMTracker::read_sensor_config_from_yaml(m_config_Dir + "config.yaml", sensorConfigMap))
  {
    matchSensors();
  }
  else
  {
    std::cerr << "No match sensor information from config.yaml" << std::endl;
  }

  std::cout << "EM Tracker initialized." << std::endl;

  // Initialize filters for transformation and rotation data
  m_filter_tran_robot = std::make_unique<ButterworthFilter<3>>(filter_sample_time);
  m_filter_rot_robot = std::make_unique<ButterworthFilter<4>>(filter_sample_time);
  m_filter_tran_tool = std::make_unique<ButterworthFilter<3>>(filter_sample_time);
  m_filter_rot_tool = std::make_unique<ButterworthFilter<4>>(filter_sample_time);
  m_filter_tran_phantom = std::make_unique<ButterworthFilter<3>>(filter_sample_time);
  m_filter_rot_phantom = std::make_unique<ButterworthFilter<4>>(filter_sample_time);
  m_filter_tran_probe = std::make_unique<ButterworthFilter<3>>(filter_sample_time);
  m_filter_rot_probe = std::make_unique<ButterworthFilter<4>>(filter_sample_time);

  // Set filter parameters
  EMTracker::set_filter_params(fc_ref, fc_tools);
}

/* Destructor */
EMTracker::~EMTracker()
{
  std::cout << "EMTracker destructor called" << std::endl;
  EMTracker::stop_read_thread();
  m_combinedAPI->stopTracking();
  std::cout << "Tracking stopped" << std::endl;
}

/* Copy constructor */
EMTracker::EMTracker(const EMTracker &rhs) : m_combinedAPI(rhs.m_combinedAPI) {};

/* move constructor */
EMTracker::EMTracker(EMTracker &&rhs) noexcept
{
  // handling self assignment
  if (this != &rhs)
  {
    this->m_combinedAPI = std::move(rhs.m_combinedAPI);
  }
}

/* Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output */
std::string EMTracker::getToolInfo(std::string toolHandle)
{
  // Get the port handle info from PHINF
  PortHandleInfo info = this->m_combinedAPI->portHandleInfo(toolHandle);
  // Return the ID and SerialNumber the desired string format
  std::string outputString = info.getToolId();
  outputString.append(" s/n:").append(info.getSerialNumber());
  return outputString;
}

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
void EMTracker::onErrorPrintDebugmessage(const std::string &methodName, int errorCode)
{
  if (errorCode < 0)
  {
    std::cerr << methodName << " failed: " << m_combinedAPI->errorToString(errorCode) << std::endl;
  }
}

/**
 * @brief Initialize and enable loaded sensors.
 * @details This is the same regardless of sensor type.
 */
void EMTracker::initializeAndEnableSensors()
{
  // Retrieve port handles of non-initialized ports (sensors)
  std::vector<PortHandleInfo> portHandles = m_combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);

  // Initialize and enable each port handle
  for (int i = 0; i < static_cast<int>(portHandles.size()); ++i)
  {
    std::cout << "Initializing port handle number " << i << std::endl;
    // Initialize and enable active ports (sensors)
    onErrorPrintDebugmessage("capi.portHandleInitialize()", m_combinedAPI->portHandleInitialize(portHandles[i].getPortHandle()));
    onErrorPrintDebugmessage("capi.portHandleEnable()", m_combinedAPI->portHandleEnable(portHandles[i].getPortHandle()));
  }

  std::cout << "--------------# Enabled ports info #--------------" << std::endl;
  // Retrieve and display information about enabled port handles
  for (int i = 0; i < static_cast<int>(portHandles.size()); ++i)
  {
    std::vector<PortHandleInfo> enabledPortHandles = m_combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    auto portHandleInfo = m_combinedAPI->portHandleInfo(enabledPortHandles[i].getPortHandle());
    std::cout << "Port handle #" << i << "   -   S/N: " << portHandleInfo.getSerialNumber() << std::endl;
  }
}

/**
 * @brief Matches the serial number of the sensors (from soldered SROM) with the serial numbers
 *        in "config.yaml" to name the sensors based on the "config.yaml" file.
 */
void EMTracker::matchSensors()
{
  // Retrieve port handles of the enabled ports (sensors)
  std::vector<PortHandleInfo> portHandles = m_combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);

  std::cout << "------# Matching sensors with config.yaml #-------" << std::endl;
  m_num_active_sensors = 0;

  // Iterate through the port handles and match serial numbers with config.yaml
  for (int i = 0; i < static_cast<int>(portHandles.size()); ++i)
  {
    auto portHandleInfo = m_combinedAPI->portHandleInfo(portHandles[i].getPortHandle());
    std::string portSerialNumber = portHandleInfo.getSerialNumber();

    // Iterate through the sensor configuration map to find the matching serial number
    for (auto &entry : sensorConfigMap)
    {
      if (entry.second.serial_number == portSerialNumber)
      {
        std::cout << "Sensor name: \"" << entry.first << "\" found\n"
                  << "S/N: " << portSerialNumber << "\n"
                  << "PortHandle #" << i << "\n"
                  << "--------------------------------------------------"
                  << std::endl;
        entry.second.probe_handle_num = i; // Set probeHandle_num to i
        entry.second.active = true;

        m_sec_transforms.push_back(QuatTransformationStruct()); // initialize with identity transformation
      }
    }
  }

  std::cout << "Sensors serial number matching process finished." << std::endl;
}

/**
 * @brief Loads the SROM to the sensors with serial numbers mentioned in the "config.yaml" file.
 *
 * @details EM tracker must be reinitialized and tracking must be started again after calling this function.
 */
void EMTracker::LoadToolDefinitions2Ports(bool load_all)
{
  std::cout << "---------# Loading tool definition (.rom) #-------\n"
            << "------# and secondary tranformation (.csv) #------"
            << std::endl;

  if (load_all)
  {
    std::cout << "Loading to all sensors" << std::endl;
  }
  else
  {
    std::cout << "Loading to probes only" << std::endl;
  }

  // Retrieve port handles of the enabled ports (sensors)
  std::vector<PortHandleInfo> portHandles = m_combinedAPI->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);

  // Iterate through the sensor configuration map to find the matching serial number
  m_config_Dir = CONFIG_DIRECTORY;
  for (auto &entry : sensorConfigMap)
  {
    quatTransformation transform;
    if (entry.second.active)
    {
      if (load_all || entry.first == "probe_1" || entry.first == "probe_2" || entry.first == "probe_3")
      {
        std::cout << "Sensor name: \"" << entry.first << "\"\n"
                  << "S/N: " << entry.second.serial_number << "\n"
                  << "PortHandle #" << entry.second.probe_handle_num
                  << std::endl;
        if (entry.second.load_srom && !entry.second.srom_filename.empty())
        {
          std::cout << "ROM file name: " << entry.second.srom_filename << std::endl;
          // Load SROM file to the corresponding port handle
          m_combinedAPI->loadSromToPort(m_config_Dir + entry.second.srom_filename,
                                        std::stoi(portHandles[entry.second.probe_handle_num].getPortHandle(), nullptr, 16));
        }
        if (entry.second.load_tran && !entry.second.tran_filename.empty())
        {
          std::string trans_csv_path = m_config_Dir + entry.second.tran_filename;
          EMTracker::load_transformation_from_csv(trans_csv_path, m_sec_transforms[entry.second.probe_handle_num]);
          std::cout << "Secondary tranformation CSV file name: " << entry.second.tran_filename << std::endl;
        }
        if (!(entry.second.load_tran && !entry.second.tran_filename.empty()) && !(entry.second.load_srom && !entry.second.srom_filename.empty()))
        {
          std::cout << "No Tool Definition or Secondary Tranforamtion is needed for " << entry.first << std::endl;
        }
      }
    }
    else
    {
      std::cout << entry.first << " is not active" << std::endl;
    }
    std::cout << "--------------------------------------------------" << std::endl;
  }
}

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
void EMTracker::landmark_registration(const std::string &landmarks_file_name, std::string ref_sensor_name)
{
  std::cout << "-----------# Landmarks capturing mode #-----------" << std::endl;

  // Check if any probe is connected
  if (!(sensorConfigMap["probe_1"].active || sensorConfigMap["probe_2"].active || sensorConfigMap["probe_3"].active))
  {
    std::cerr << "ERROR: Probe is not connected" << std::endl;
    std::cout << "Calibration bypassed" << std::endl;
    return;
  }

  // Check if reference sensor is connected
  if (!sensorConfigMap[ref_sensor_name].active)
  {
    std::cerr << "ERROR: Reference sensor is not connected" << std::endl;
    std::cout << "Calibration bypassed" << std::endl;
    return;
  }

  EMTracker::LoadToolDefinitions2Ports(false);

  // Start tracking
  onErrorPrintDebugmessage("capi.startTracking()", m_combinedAPI->startTracking());
  std::cout << "---------# Landmarks capturing started #----------" << std::endl;

  std::vector<ToolData> sensors_data;
  quatTransformation transform_0_1;                             // transformation from EM frame to Reference EM frame
  quatTransformation transform_0_6;                             // transformation from EM frame to Probe or tool tip
  quatTransformation transform_1_6;                             // transformation from Reference EM frame to Probe or tool tip
  quatTransformation transform_1_2;                             // transformation from Reference EM frame to system frame
  std::vector<blaze::StaticVector<double, 3>> pointBuffer(200); // Create a buffer to store the past 200 samples
  std::vector<blaze::StaticVector<double, 3>> landmarks_truth;

  // Load truth landmark positions from CSV file
  EMTracker::load_landmarks_from_csv(m_config_Dir + landmarks_file_name, &landmarks_truth);
  num_landmark = landmarks_truth.size();

  std::vector<blaze::StaticVector<double, 3>> landmarks_measured(num_landmark, blaze::StaticVector<double, 3>(0.0));

  std::cout << "Num landmarks: " << num_landmark << std::endl;
  std::cout << "Waiting ..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  constexpr double threshold = 1..0; // threshold value for the radius of the error sphere [mm]
  unsigned int i = 0;                // counter for the number of landmarks

  // Monitor the probe tip for stationary instances to record landmark positions
  quatTransformation temp;
  while (true)
  {
    sensors_data = m_combinedAPI->getTrackingDataBX();
    ToolData2QuatTransform(sensors_data[sensorConfigMap[ref_sensor_name].probe_handle_num], transform_0_1);

    if (sensorConfigMap["probe_1"].active)
    {
      ToolData2QuatTransform(sensors_data[sensorConfigMap["probe_1"].probe_handle_num], temp);
      Combine_Quat_Transformation(temp, m_sec_transforms[sensorConfigMap["probe_1"].probe_handle_num], transform_0_6);
    }
    else if (sensorConfigMap["probe_2"].active)
    {
      ToolData2QuatTransform(sensors_data[sensorConfigMap["probe_2"].probe_handle_num], temp);
      Combine_Quat_Transformation(temp, m_sec_transforms[sensorConfigMap["probe_2"].probe_handle_num], transform_0_6);
    }
    else if (sensorConfigMap["probe_3"].active)
    {
      ToolData2QuatTransform(sensors_data[sensorConfigMap["probe_3"].probe_handle_num], temp);
      Combine_Quat_Transformation(temp, m_sec_transforms[sensorConfigMap["probe_3"].probe_handle_num], transform_0_6);
    }
    else
    {
      std::cerr << "Error selecting probe" << std::endl;
    }

    // Calculate the transformation of the probe in the reference frame
    Combine_Quat_Transformation(transform_0_1.inv(), transform_0_6, transform_1_6);

    // Add the current sample to the buffer
    pointBuffer.push_back(transform_1_6.translation);
    // Remove the oldest sample if the buffer size exceeds 200
    if (pointBuffer.size() > 200)
    {
      pointBuffer.erase(pointBuffer.begin());
    }

    // Check if all distances are less than the threshold
    bool status = EMTracker::points_in_sphere(pointBuffer, threshold);
    if (pointBuffer.size() == 200 && status)
    {
      // Calculate and save the average position of the landmark
      EMTracker::column_average(pointBuffer, landmarks_measured[i]);
      pointBuffer.clear();
      std::cout << "Landmark " << i + 1 << "/" << num_landmark << " saved" << std::endl;
      i++;
      if (i >= num_landmark)
      {
        std::cout << "---------# All landmarks captured #---------" << std::endl;
        m_combinedAPI->stopTracking();
        std::cout << "------------# Capturing stopped #-----------" << std::endl;
        break;
      }
      // Wait for the operator to move the probe to the next landmark
      std::cout << "Please go to the next landmark" << std::endl;
      std::cout << "Waiting..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

    std::cout << "Landmark " << i + 1 << "/" << num_landmark << " | "
              << std::fixed << std::setprecision(2)
              << "X: " << transform_1_6.translation[0] << "    "
              << "Y: " << transform_1_6.translation[1] << "    "
              << "Z: " << transform_1_6.translation[2] << "    "
              << std::fixed << std::setprecision(0)
              << "Buffer: " << pointBuffer.size() << std::endl;
  }
  // Restore the original settings
  std::cout.flags(originalFlags);
  std::cout.precision(originalPrecision);

  // Save the measured landmarks' average positions to a CSV file
  EMTracker::save_landmarks_to_csv(landmarks_measured, m_config_Dir + "measured_landmarks.csv");

  // Uncomment if you want to bypass capturing landmark data and use the already saved ones
  // EMTracker::load_landmarks_from_csv(m_config_Dir + "Landmarks_Measured.csv", &landmarks_measured);

  // Calculate the transformation from the reference sensor to the system frame
  std::cout << "Calculating the transformation" << std::endl;
  Calculate_Transformation(landmarks_measured, landmarks_truth, transform_1_2);
  // Save the calculated quaternion transformation in CSV format
  EMTracker::save_transformation_to_csv(transform_1_2, m_config_Dir + "registration_results.csv");
  std::cout << "Transformation saved to 'registration_results.csv'" << std::endl;
}

/**
 * @brief Starts the read loop in a separate thread to avoid blocking the program.
 *
 * @details This method creates a new thread that runs the EMTracker::Read_Loop() function,
 *          allowing the main program to continue running without being blocked by the read operations.
 */
void EMTracker::start_read_thread()
{
  m_emThread = std::thread(&EMTracker::Read_Loop, this);
}

/**
 * @brief Stops the read loop thread safely.
 *
 * @details This method sets a stop flag to true, signaling the read loop to terminate.
 *          If the read loop thread is joinable, it waits for the thread to finish before proceeding.
 */
void EMTracker::stop_read_thread()
{
  stopFlag.store(true); // Set the flag to true to signal the read loop to stop
  if (m_emThread.joinable())
  {
    m_emThread.join(); // Wait for the thread to finish
  }
  std::cout << "Read loop stopped!" << std::endl;
}

/**
 * @brief Loads tool definition files and reads data from the tip sensor and the reference sensor to calculate the transformation
 *        of the tip in the robot's frame.
 *
 * @details This function converts the rotations to Euler angles and updates the values in tipRelTran_ptr.
 *          It is recommended to call this function from a separate thread to avoid blocking the main code.
 */
void EMTracker::Read_Loop()
{
  // Check if sensors are connected
  if (!sensorConfigMap["robot"].active)
  {
    std::cerr << "ERROR: Robot sensor is not connected" << std::endl;
    std::cout << "Read loop bypassed" << std::endl;
    return;
  }
  if (!sensorConfigMap["tool"].active)
  {
    std::cerr << "ERROR: Tool sensor is not connected" << std::endl;
    std::cout << "Read loop bypassed" << std::endl;
    return;
  }
  if (!sensorConfigMap["phantom"].active)
  {
    std::cerr << "ERROR: Phantom sensor is not connected" << std::endl;
    std::cout << "Read loop bypassed" << std::endl;
    return;
  }

  std::vector<ToolData> sensors_data;
  std::vector<blaze::StaticVector<double, 3UL>> pointBuffer(100); // Buffer to store the past 200 samples

  quatTransformation temp_1, temp_2, temp_3, temp_4;

  // Load tool definition files and initialize sensors
  EMTracker::LoadToolDefinitions2Ports(true);
  std::cout << "Tool definitions loading finished" << std::endl;
  EMTracker::initializeAndEnableSensors();
  std::cout << "Sensors enabled" << std::endl;
  EMTracker::onErrorPrintDebugmessage("capi.startTracking()", m_combinedAPI->startTracking());
  SleepSeconds(1);

  std::cout << "Reading EM sensors ..." << std::endl;
  while (!stopFlag.load())
  {
    auto t0 = std::chrono::high_resolution_clock::now();

    sensors_data = m_combinedAPI->getTrackingDataBX();

    ToolData2QuatTransform(sensors_data[sensorConfigMap["robot"].probe_handle_num], temp_1);
    ToolData2QuatTransform(sensors_data[sensorConfigMap["tool"].probe_handle_num], temp_2);
    ToolData2QuatTransform(sensors_data[sensorConfigMap["phantom"].probe_handle_num], temp_3);

    Combine_Quat_Transformation(temp_1, m_sec_transforms[sensorConfigMap["robot"].probe_handle_num], m_transform_0_1);
    Combine_Quat_Transformation(temp_2, m_sec_transforms[sensorConfigMap["tool"].probe_handle_num], m_transform_0_2);
    Combine_Quat_Transformation(temp_3, m_sec_transforms[sensorConfigMap["phantom"].probe_handle_num], m_transform_0_3);

    if (sensorConfigMap["probe_1"].active)
    {
      ToolData2QuatTransform(sensors_data[sensorConfigMap["probe_1"].probe_handle_num], temp_4);
      Combine_Quat_Transformation(temp_4, m_sec_transforms[sensorConfigMap["probe_1"].probe_handle_num], m_transform_0_6);
    }
    else if (sensorConfigMap["probe_2"].active)
    {
      ToolData2QuatTransform(sensors_data[sensorConfigMap["probe_2"].probe_handle_num], temp_4);
      Combine_Quat_Transformation(temp_4, m_sec_transforms[sensorConfigMap["probe_2"].probe_handle_num], m_transform_0_6);
    }
    else if (sensorConfigMap["probe_3"].active)
    {
      ToolData2QuatTransform(sensors_data[sensorConfigMap["probe_3"].probe_handle_num], temp_4);
      Combine_Quat_Transformation(temp_4, m_sec_transforms[sensorConfigMap["probe_3"].probe_handle_num], m_transform_0_6);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> loop_duration = t1 - t0;
    m_measured_sample_time = loop_duration.count() * 1.00E-6;

    // Filter position and orientations
    if (m_flag_filter)
    {
      m_transform_0_1.translation = m_filter_tran_robot->add_data_point(m_transform_0_1.translation);
      m_transform_0_1.rotation = m_filter_rot_robot->add_data_point(m_transform_0_1.rotation);
      m_transform_0_2.translation = m_filter_tran_tool->add_data_point(m_transform_0_2.translation);
      m_transform_0_2.rotation = m_filter_rot_tool->add_data_point(m_transform_0_2.rotation);
      m_transform_0_3.translation = m_filter_tran_phantom->add_data_point(m_transform_0_3.translation);
      m_transform_0_3.rotation = m_filter_rot_phantom->add_data_point(m_transform_0_3.rotation);
    }

    // Calculate the transformation of the probe in the reference frame
    Combine_Quat_Transformation(m_transform_0_1.inv(), m_transform_0_2, m_transform_1_2);
    Combine_Quat_Transformation(m_transform_0_3.inv(), m_transform_0_2, m_transform_3_2);
    Combine_Quat_Transformation(m_transform_0_3.inv(), m_transform_0_1, m_transform_3_1);

    if (sensorConfigMap["probe_1"].active || sensorConfigMap["probe_2"].active || sensorConfigMap["probe_3"].active)
    {
      if (m_flag_filter)
      {
        m_transform_0_6.translation = m_filter_tran_probe->add_data_point(m_transform_0_6.translation);
        m_transform_0_6.rotation = m_filter_rot_probe->add_data_point(m_transform_0_6.rotation);
      }
      Combine_Quat_Transformation(m_transform_0_3.inv(), m_transform_0_6, m_transform_3_6);
    }

    // Calculate tool relative translational velocity
    m_transform_dot_1_2.translation = (m_transform_1_2.translation - m_transform_1_2_prev.translation) / m_measured_sample_time;
    // Calculate tool relative rotational velocity ************ this part of the code must be verified for correctness **********
    const blaze::StaticVector<double, 4UL> delta_q = quaternionMultiply(m_transform_1_2.rotation, m_transform_1_2_prev.inv().rotation);
    const double theta = 2.00 * acos(delta_q[0UL]);
    const blaze::StaticVector<double, 3UL> u = {delta_q[1UL], delta_q[2UL], delta_q[3UL]};
    const double u_norm = std::sqrt(u[0UL] * u[0UL] + u[1UL] * u[1UL] + u[2UL] * u[2UL]);
    const blaze::StaticVector<double, 4UL> log_delta_q = {0.00, (theta * u[0UL]) / u_norm, (theta * u[1UL]) / u_norm, (theta * u[2UL]) / u_norm};
    const blaze::StaticVector<double, 4UL> omega = 2.00 * log_delta_q / m_measured_sample_time;
    m_transform_dot_1_2.rotation = 0.50 * omega * m_transform_1_2.rotation;
    // update prev
    m_transform_1_2_prev = m_transform_1_2;

    // Print the transformation if the flag is set
    if (m_flag_debug)
    {
      EMTracker::log_tansformation(1, m_transform_3_6.translation * 1.00E-3, m_transform_3_6.rotation, m_measured_sample_time);
    }

    // Check the stop flag periodically
    if (stopFlag.load())
    {
      break;
    }
  }
}

/**
 * @brief Sets the filter parameters for the reference and tools filters.
 *
 * @details This function updates the filter coefficients for both translation and rotation filters
 *          based on the provided cutoff frequencies for the reference and tools.
 *
 * @param fc_ref The cutoff frequency for the reference filter in Hz.
 * @param fc_tools The cutoff frequency for the tools filter in Hz.
 */
void EMTracker::set_filter_params(double fc_ref, double fc_tools)
{
  m_filter_tran_robot->update_coeffs(fc_ref);
  m_filter_rot_robot->update_coeffs(fc_ref);
  m_filter_tran_tool->update_coeffs(fc_tools);
  m_filter_rot_tool->update_coeffs(fc_tools);
  m_filter_tran_phantom->update_coeffs(fc_ref);
  m_filter_rot_phantom->update_coeffs(fc_ref);
  m_filter_tran_probe->update_coeffs(fc_tools);
  m_filter_rot_probe->update_coeffs(fc_tools);

  std::cout << "Reference filter cutoff set to: " << fc_ref << " Hz" << "\n"
            << "Tools filter cutoff set to: " << fc_tools << " Hz"
            << std::endl;
}

/**
 * @brief Retrieves the tool transformation in the system (CTR or catheter robot) frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the tool transformation.
 */
void EMTracker::get_tool_transform_in_robot(quatTransformation &transform)
{
  transform.translation = m_transform_1_2.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_1_2.rotation;
}

/**
 * @brief Retrieves the tool transformation in the phantom frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the tool transformation.
 */
void EMTracker::get_tool_transform_in_phantom(quatTransformation &transform)
{
  transform.translation = m_transform_3_2.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_3_2.rotation;
}

/**
 * @brief Retrieves the derivative of the tool transformation in the system (CTR or catheter robot) frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform_dot Reference to a quatTransformation object to store the tool transformation derivative.
 */
void EMTracker::get_tool_transform_in_robot_dot(quatTransformation &transform_dot)
{
  transform_dot.translation = m_transform_dot_1_2.translation * 1.00E-3; // Convert from mm to meters
  transform_dot.rotation = m_transform_dot_1_2.rotation;
}

/**
 * @brief Retrieves the tool sensor transformation in the EM tracker frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the tool sensor transformation.
 */
void EMTracker::get_tool_transform_in_em(quatTransformation &transform)
{
  transform.translation = m_transform_0_2.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_0_2.rotation;
}

/**
 * @brief Retrieves the transformation from the system frame to the EM frame (EM frame in systems frame).
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the system to EM frame transformation.
 */
void EMTracker::get_robot_transform_in_em(quatTransformation &transform)
{
  transform.translation = m_transform_0_1.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_0_1.rotation;
}

/**
 * @brief Retrieves the transformation from the system base frame to the phantom frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the system to EM frame transformation.
 */
void EMTracker::get_robot_transform_in_phantom(quatTransformation &transform)
{
  transform.translation = m_transform_3_1.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_3_1.rotation;
}

/**
 * @brief Retrieves the probe position in the system base frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the probe transformation.
 */
void EMTracker::get_probe_transform_in_robot(quatTransformation &transform)
{
  transform.translation = m_transform_1_6.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_1_6.rotation;
}

/**
 * @brief Retrieves the probe position in the phantom frame.
 *
 * @details The transformation includes translation and rotation, with the rotation represented in quaternion format.
 *          The translation is converted from millimeters to meters.
 *
 * @param transform Reference to a quatTransformation object to store the probe transformation.
 */
void EMTracker::get_probe_transform_in_phantom(quatTransformation &transform)
{
  transform.translation = m_transform_3_6.translation * 1.00E-3; // Convert from mm to meters
  transform.rotation = m_transform_3_6.rotation;
}

/** @brief get sample time */
void EMTracker::get_sample_time(double &sample_time)
{
  sample_time = m_measured_sample_time;
}

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
void EMTracker::ToolData2Vector(const ToolData &toolData, std::vector<double> &toolCoord)
{
  toolCoord = {
      toolData.transform.isMissing() ? 0.00 : toolData.transform.tx,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.ty,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.tz,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.q0,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.qx,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.qy,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.qz,
      toolData.transform.isMissing() ? 0.00 : toolData.transform.error};
}

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
void EMTracker::ToolData2QuatTransform(const ToolData &input, quatTransformation &output)
{
  if (!input.transform.isMissing() && input.transform.status == TransformStatus::Enabled)
  {
    output.rotation = {input.transform.q0, input.transform.qx, input.transform.qy, input.transform.qz};
    output.translation = {input.transform.tx, input.transform.ty, input.transform.tz};
  }
  else
  {
    // Set rotation to NaN
    output.rotation = {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN()};

    // Set translation to NaN
    output.translation = {std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::quiet_NaN()};
  }
}

//
void EMTracker::log_tansformation(const int number, const blaze::StaticVector<double, 3UL> &translation, const blaze::StaticVector<double, 4UL> &rotation, const double sample_time) const
{
  std::ostringstream oss;
  auto print_with_space_if_positive = [](double value)
  {
    std::ostringstream tmp;
    tmp << std::fixed << std::setprecision(3);
    if (value >= 0)
    {
      tmp << " " << value;
    }
    else
    {
      tmp << value;
    }
    return tmp.str();
  };

  oss << "X:" << print_with_space_if_positive(translation[0UL]) << "  "
      << "Y:" << print_with_space_if_positive(translation[1UL]) << "  "
      << "Z:" << print_with_space_if_positive(translation[2UL]) << " [m]"
      << "  |  "
      << "q0:" << print_with_space_if_positive(rotation[0UL]) << "  "
      << "qX:" << print_with_space_if_positive(rotation[1UL]) << "  "
      << "qY:" << print_with_space_if_positive(rotation[2UL]) << "  "
      << "qZ:" << print_with_space_if_positive(rotation[3UL]) << "  |  "
      << "dT:" << std::fixed << std::setprecision(1) << sample_time * 1.00E3 << " [ms]";

  std::cout << number << "  " << oss.str().c_str() << std::endl;
}

/**
 * @brief Checks if all distances are less than a threshold.
 * *
 * @param points_list A vector of 3D points to check.
 * @param radius The radius of the sphere.
 *
 * @return true if all points are within the sphere with the given radius.
 */
bool EMTracker::points_in_sphere(const std::vector<blaze::StaticVector<double, 3UL>> &points_list, const double &radius)
{
  const blaze::StaticVector<double, 3UL> referenceSample = points_list[0UL];

  for (const auto &point : points_list)
  {
    double distance;
    Euclidean_Distance(point, referenceSample, distance);

    // If any distance is greater than or equal to the threshold, return false
    if (distance >= radius || std::isnan(distance))
    {
      return false;
    }
  }

  // If all distances are less than the threshold, return true
  return true;
}

/**
 * @brief Saves collected landmarks to a CSV file with X, Y, Z headers.
 *
 * @param data A vector of 3D points representing the collected landmarks.
 * @param filepath The path to the CSV file where the data will be saved.
 */
void EMTracker::save_landmarks_to_csv(const std::vector<blaze::StaticVector<double, 3UL>> &data, const std::string &filepath)
{
  std::filesystem::path filePath(filepath);

  // Ensure the directory exists; create it if it doesn't.
  if (!std::filesystem::exists(filePath.parent_path()))
    std::filesystem::create_directories(filePath.parent_path());

  std::ofstream file(filepath, std::ios::out | std::ios::trunc); // Open the file for writing with truncation
  if (!file)
  {
    std::cerr << "Failed to open file for writing: " << filepath << std::endl;
    return;
  }

  // Write the CSV header
  file << "X,Y,Z\n";

  // Reserve a buffer size for the file stream to improve performance
  file.rdbuf()->pubsetbuf(nullptr, 0);

  // Write the data points
  for (const auto &sample : data)
    file << sample[0] << ',' << sample[1] << ',' << sample[2] << '\n';

  if (file.fail())
    std::cerr << "Failed to write to file: " << filepath << std::endl;

  // The file will be automatically closed when the ofstream object goes out of scope
}

/**
 * @brief Loads landmarks from a CSV file with X, Y, Z headers.
 *
 * @param filepath The path to the CSV file to be read.
 * @param data A pointer to a vector of 3D points to store the loaded landmarks.
 */
void EMTracker::load_landmarks_from_csv(const std::string &filepath, std::vector<blaze::StaticVector<double, 3UL>> *data)
{
  // Clear the output vector to start fresh
  data->clear();

  // Open the CSV file
  std::ifstream file(filepath, std::ios::in);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file for reading: " << filepath << std::endl;
    return; // Return without modifying the output vector
  }

  std::string line;
  bool firstLine = true; // Flag to skip the first line (header)

  // Reserve some initial capacity to improve memory efficiency
  data->reserve(500); // Adjust the initial capacity as needed

  while (std::getline(file, line))
  {
    if (firstLine)
    {
      // Skip the first line (header)
      firstLine = false;
      continue;
    }

    blaze::StaticVector<double, 3UL> row; // This vector stores the X, Y, Z values for each line
    std::istringstream ss(line);
    std::string valueStr;
    size_t index = 0;

    while (std::getline(ss, valueStr, ',') && index < 3)
    {
      try
      {
        row[index++] = std::stod(valueStr);
      }
      catch (const std::invalid_argument &e)
      {
        std::cerr << "Failed to parse a value: " << valueStr << " - " << e.what() << std::endl;
        break;
      }
      catch (const std::out_of_range &e)
      {
        std::cerr << "Value out of range: " << valueStr << " - " << e.what() << std::endl;
        break;
      }
    }

    if (index == 3)
    { // Ensure we have read exactly 3 values
      data->push_back(row);
    }
    else
    {
      std::cerr << "Incorrect number of values in line: " << line << std::endl;
    }
  }

  file.close(); // Close the file
}

/**
 * @brief Saves QuatTransformation data to a CSV file with headers.
 *
 * @param data The QuatTransformation object containing the transformation data.
 * @param filename The path to the CSV file where the data will be saved.
 */
void EMTracker::save_transformation_to_csv(const quatTransformation &data, const std::string &filename)
{
  // Open the CSV file for writing
  std::ofstream file(filename, std::ios::out | std::ios::trunc);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }

  // Write the CSV headers
  file << "X,Y,Z,Q0,Qx,Qy,Qz";
  if (data.error != 0.00)
  {
    file << ",Error"; // Include 'Error' header if error member is present
  }
  file << std::endl;

  // Write data to the CSV file
  file << data.translation[0UL] << "," << data.translation[1UL] << "," << data.translation[2UL] << ","
       << data.rotation[0UL] << "," << data.rotation[1UL] << "," << data.rotation[2] << "," << data.rotation[3UL];

  if (data.error != 0.00)
  {
    file << "," << data.error; // Include error value if it is present
  }
  file << std::endl;

  // Close the file
  file.close();
}

/* Load QuatTransformation data from a CSV file */
bool EMTracker::load_transformation_from_csv(const std::string &filename, quatTransformation &data)
{
  // Open the CSV file for reading
  std::ifstream file(filename, std::ios::in);
  if (!file)
  {
    std::cerr << "Failed to open file for reading: " << filename << std::endl;
    return false;
  }

  std::string line;
  std::vector<std::string> headers;
  std::vector<std::string> values;

  // Read the headers (first line) from the CSV file
  if (std::getline(file, line))
  {
    std::istringstream headerStream(line);
    std::string header;
    while (std::getline(headerStream, header, ','))
    {
      headers.emplace_back(header);
    }
  }
  else
  {
    std::cerr << "Failed to read CSV headers." << std::endl;
    return false;
  }

  // Ensure that the headers include the required fields
  if (headers.size() < 7)
  {
    std::cerr << "CSV file is missing required fields." << std::endl;
    return false;
  }

  // Read the values (data) from the CSV file
  if (std::getline(file, line))
  {
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
    return false;
  }

  // Validate the number of values
  if (values.size() != 7 && values.size() != 8)
  {
    std::cerr << "Unexpected number of values in the CSV file." << std::endl;
    return false;
  }

  try
  {
    // Parse and assign the translation and rotation components
    data.translation = {
        std::stod(values[0UL]),
        std::stod(values[1UL]),
        std::stod(values[2UL])};
    data.rotation = {
        std::stod(values[3UL]),
        std::stod(values[4UL]),
        std::stod(values[5UL]),
        std::stod(values[6UL])};

    // Parse the error value if present
    if (values.size() == 8)
    {
      data.error = std::stod(values[7]);
    }
    else
    {
      data.error = 0.00; // Default error value
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to parse QuatTransformation data from CSV: " << e.what() << std::endl;
    return false;
  }

  return true;
}

/**
 * @brief Calculates the average of each column in a vector of 3D points.
 *
 * @param data A vector of 3D points from which to calculate the column averages.
 * @param columnAverages Reference to a blaze::StaticVector<double, 3> to store the calculated averages.
 */
void EMTracker::column_average(const std::vector<blaze::StaticVector<double, 3UL>> &data, blaze::StaticVector<double, 3UL> &columnAverages)
{
  if (data.empty())
  {
    std::cerr << "Input data is empty." << std::endl;
    return;
  }

  const size_t numRows = data.size();
  columnAverages = 0.00;

  // Sum each column
  for (const auto &row : data)
  {
    columnAverages[0UL] += row[0UL];
    columnAverages[1UL] += row[1UL];
    columnAverages[2UL] += row[2UL];
  }

  // Divide by the number of rows to get the average
  columnAverages /= numRows;
}

/**
 * @brief Reads sensor configuration from a YAML file and populates a map with the configuration data.
 *
 * @param file_path The path to the YAML file containing the sensor configuration.
 * @param sensorConfig_map Pointer to a map to store the sensor configuration data.
 *
 * @return 0 on success, -1 on failure.
 */
int EMTracker::read_sensor_config_from_yaml(const std::string &file_path, std::map<std::string, SensorConfig> &sensorConfig_map)
{
  try
  {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(file_path);

    // Iterate through the YAML data
    for (const auto &sensorEntry : config["sensor_config"])
    {
      for (YAML::const_iterator it = sensorEntry.begin(); it != sensorEntry.end(); ++it)
      {
        SensorConfig sensorConfig;
        sensorConfig.serial_number = it->second["serial_number"].as<std::string>();

        // Optional parameters with default values
        sensorConfig.srom_filename = it->second["srom_filename"] ? it->second["srom_filename"].as<std::string>() : "null";
        sensorConfig.tran_filename = it->second["tran_filename"] ? it->second["tran_filename"].as<std::string>() : "null";

        sensorConfig.load_srom = it->second["load_srom"].as<bool>();
        sensorConfig.load_tran = it->second["load_tran"].as<bool>();

        sensorConfig_map[it->second["name"].as<std::string>()] = sensorConfig;
      }
    }
    return 0;
  }
  catch (const YAML::Exception &e)
  {
    std::cerr << "Could not load " << file_path << ": " << e.what() << std::endl;
    return -1;
  }
}

/**
 * @brief Pauses the execution of the program for a specified number of seconds.
 */
void SleepSeconds(const unsigned numSeconds)
{
  sleep(numSeconds); // sleep(sec)
}

Recorder::Recorder(const std::string &filename) : filename_(filename)
{
  // Open the CSV file and write headers
  file.open(filename, std::ios::out | std::ios::trunc);
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

void Recorder::Record(const quatTransformation &Reference, const quatTransformation &Tip)
{
  // Open the CSV file and write headers
  file.open(filename, std::ios::out | std::ios::app);

  if (file.is_open())
  {
    auto current_time = std::chrono::high_resolution_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();

    file << elapsed_seconds << ","
         << Reference.translation[0UL] << "," << Reference.translation[1UL] << "," << Reference.translation[2UL] << ","
         << Reference.rotation[0UL] << "," << Reference.rotation[1UL] << "," << Reference.rotation[2UL] << "," << Reference.rotation[3UL] << ","
         << Tip.translation[0UL] << "," << Tip.translation[1UL] << "," << Tip.translation[2UL] << ","
         << Tip.rotation[0UL] << "," << Tip.rotation[1UL] << "," << Tip.rotation[2UL] << "," << Tip.rotation[3UL] << std::endl;
  }
  else
  {
    std::cerr << "Failed to open file for appending: " << filename << std::endl;
  }
}

void Recorder::Close()
{
  file.close();
}
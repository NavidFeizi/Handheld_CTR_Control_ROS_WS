
<div align="center">

# NDI Electromagnetic Tracker

</div>

## Introduction

This software establishes a connection with the NDI Aurora EM tracker to perform the following functions:
* Reads data from tool sensors and the base sensor, and publishes the position of the tool relative to the base frame.
* Performs landmark registration.
* Operates at the maximum sampling frequency ~66Hz - 'turbo mode'.
* For older device versions that do not support USB, adjust the baud rate in `ndi_api` (`CombinedApi.cpp`) to a feasible value.

## Building Requirements

Ensure that the following libraries are installed on your system:

* [Boost](https://www.boost.org/)
* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)
* [LAPACK](http://www.netlib.org/lapack/)
* [NLopt](https://nlopt.readthedocs.io/en/latest/)

### Setup EMtracker USB Connection

1. **Identify connected USB ports:**
   ```bash
   ls /dev/tty*
   ```

2. **Grant access permission to the port:**
   ```bash
   sudo chmod a+rw /dev/ttyUSB*
   ```
   Replace `*` with the number of the connected USB port, usually `0`.

## Build and Run Instructions

To build the nodes, follow these instructions:

1. **Build the packages:**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select interfaces emtracker 
   ```

2. **Run the node:**
   filtering frequnecy can be set from the launch file
   ```bash
   ros2 launch emtracker launch.py
   ```

3. **Adjust filtering frequency on the fly using ROS parameters:**
   ```bash
   ros2 param set emtracker_node cutoff_freq 1.0
   ```

## Landmark Registration Process

To perform landmark registration:
1. Use a pre-calibrated probe to touch the landmarks of the experimental setup.
2. Save the true values of the landmarks in a `.csv` file in the config path.
3. Uncomment the 'landmark registration process section' under `setup_emtracker()` and copy the true landmarks to `std::string landmarks`.
4. Build and run the node, and follow the instructions in the terminal. Touch each landmark in the order mentioned in the truth `.csv` file and hold still. The code guides you to move to the next landmark once enough samples are collected.
5. Once all landmarks are collected, the transformation is calculated through an optimization process, and the results are saved in `registration_results.csv` in the config path.
6. Rename the transformation file and set the name in `config.yaml` so the node understands which registration file to load next time.
7. Comment out the landmark registration section and rebuild the node.
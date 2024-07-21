<div align="center">

# NDI Electromagnetic Tracker

</div>


## Introduction

## Building Requirements

ensure that you have the following libraries installed in your system:

* [Boost](https://www.boost.org/)
* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)
* [LAPACK](http://www.netlib.org/lapack/)


## Build Packages Instruction

To build the nodes, follow these instructions:

1. **Build the packages:**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select interfaces emtracker 
   ```

## Setup the Hardware

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


## Running the Nodes

1. **Run on specific cores:**
   ```bash
   taskset -c 0,1 ros2 run catheter_sim_koopman_cpp simulator
   ```
   Note: `taskset` can also be included in the launch file.

2. **Set parameters:**
   ```bash
   ros2 param set emtracker_node cutoff_freq 1.0
   ```

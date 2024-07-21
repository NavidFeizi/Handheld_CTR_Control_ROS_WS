<div align="center">

# Catheter Robot Node

</div>

This ROS2 Package establishes the CANopen connection to the robot and manages joint space position control. The position control loop is closed over the internal velocity loop of the Maxon motion controller. Real-time communication is achieved using PDO, synchronized with a SYNC command from the master node (PC). The sampling frequency for commanding and reading joint statuses is set to 400 Hz (2.5 ms).

## Building Requirements

Ensure that you have the following libraries installed in your system:

* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)
* [LAPACK](http://www.netlib.org/lapack/)
* [Lely CANopen](https://opensource.lely.com/canopen/)
* Download and install the SocketCAN driver compatible with your Linux kernel from the [HMS Networks website](https://www.hms-networks.com/support/general-downloads).

## Build Instructions

To build the nodes, follow these steps:

1. **Build the packages:**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select interfaces robot
   ```

## Setup USB-CANopen Connection to the Robot

To set up the CANopen connection using the HMS IXXAT USB-to-CAN V2 compact, follow these steps:

1. **Load the CANopen driver:**
   ```bash
   sudo modprobe ix_usb_can
   ```
   If you encounter an error, it indicates the IXXAT SocketCAN driver is not installed. Install a compatible version with your Linux kernel (e.g., IXXAT_SocketCAN_2_0_378_Modified_2023-03-15).

2. **Configure the CAN interface:**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set can0 down
   sudo ip link set can0 type can loopback on
   sudo ip link set can0 up
   ```

3. **Monitor the CAN connection:**
   Open a terminal and run:
   ```bash
   candump can0
   ```

4. **Send a reset command to check all nodes (EPOS2 motion controllers):**
   Open a separate terminal and run:
   ```bash
   cansend can0 000#8200
   ```
   If you see a line similar to the one below among the return packets, it is the echo of the reset command, indicating the connection is working. If not, ensure the "loopback" setting is on. Sometimes, replugging the USB port resolves this issue.
   ```plaintext
   can0  000   [2]  82 00
   ```

## Usage Example

1. **Run the node:**
   ```bash
   source ./install/setup.bash
   taskset -c 0,1 ros2 run catheter_sim_koopman_cpp simulator
   ```
   or run the launch file 
   ```bash
   ros2 launch robot launch.py
   ```
   `taskset` is set in the launch file.

2. **Command homing service:**
   This service uses the EMtracker node (a separate node from the robot) to get the catheter tip position and set the tendons home position such that the catheter tip stays at zero in the home position. The emtracker_node must be alive before the execution of the homing procedure.

   Execute from another terminal:
   ```bash
   ros2 service call /homing std_srvs/srv/Trigger
   ```
3. **Set control paramters parameters:**
   Execute from another terminal:
   ```bash
   source ./install/setup.bash
   ros2 param set robot_node Kp 5.0
   ros2 param set robot_node Ki 1.0
   ```

## To Do

* Clean the robot library and handle turning motos off in a correct way

## Notes

* When tuning the low-level EPOS velocity controller, ensure that you use the setup wizard to clear all previous parameters. Some control parameters may not update after the first tuning, leading to jagged velocity control.


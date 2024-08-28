<div align="center">

# CTR robot control

</div>

## Introduction

## Building Requirements

ensure that you have the following libraries installed in your system:

* [Boost](https://www.boost.org/)
* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)
* [LAPACK](http://www.netlib.org/lapack/)
* [OpenIGTLink](https://openigtlink.org/)


## Build Packages Instruction

To build the nodes, follow these instructions:

1. **Build the packages:**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select interfaces emtracker robot controller manager
   ```

## Setup CANopen Connection to the Robot

To set up the CANopen connection, follow these steps:

1. **Load the CANopen driver:**
   ```bash
   modprobe ix_usb_can
   ```
   If you encounter an error, the IXXAT socket CAN driver is not installed. Install the compatible version with your Linux kernel (IXXAT_SocketCAN_2_0_378_Modified_2023-03-15).

2. **Configure CAN interface:**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set can0 down
   sudo ip link set can0 type can loopback on
   sudo ip link set can0 up
   ```
3. **Monitor CAN connection:**
   Open a terminal and run:
   ```bash
   candump can0
   ```

4. **Send reset command to check all nodes:**
   Open a separate terminal and run:
   ```bash
   cansend can0 000#8200
   ```
   Among the return packets if you see a line similar to the one below, which is the echo of the reset command, you are good to go. Otherwise, the "loopback" setting might not be on. Sometimes, you need to replug the USB port to fix it.
   ```plaintext
   can0  000   [2]  82 00
   ```

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


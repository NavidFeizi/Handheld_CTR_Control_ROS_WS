# Instruction to initialize the CAN connection:
modprobe ix_usb_can

sudo ip link set can0 up type can bitrate 1000000

sudo ip link set can0 txqueuelen 1000

sudo ip link set can0 down

sudo ip link set can0 type can loopback on

sudo ip link set can0 up

candump can0

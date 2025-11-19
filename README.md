# amr_sweeper_battery
ROS2 Node to get battery values from the Daly-BMS in the AMR-Sweeper battery over CANBus

amr_sweeper_battery is a ROS 2 Python package that interfaces a Daly BMS over classic CAN (250 kbit/s, 29-bit extended IDs) and publishes battery status and health information.
The node polls the Daly BMS once per second and exposes high-level battery data using standard ROS messages.

Features
Communicates with Daly BMS using official 0x90–0x98 CAN protocol
Classic CAN 2.0B extended frames, not CAN-FD

Publishes:
/battery_state (sensor_msgs/msg/BatteryState)
    pack voltage, current, SOC
    per-cell voltages
    per-cell temperatures
/battery_health (diagnostic_msgs/msg/DiagnosticArray)
    faults, MOS states, balance status, cell extremes, remaining capacity, DI/DO states, etc.

Simple, minimalistic Python implementation using rclpy + python-can
1 Hz polling rate (configurable)

Installation
Place the package into your workspace:

cd ~/ros2_ws/src
unzip amr_sweeper_battery.zip
cd ..
colcon build --packages-select amr_sweeper_battery
source install/setup.bash

Bring up CAN
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

Run the Node
ros2 launch amr_sweeper_battery battery.launch.py

Topics
Published
Topic	              Message Type	                      Description
/battery_state	    sensor_msgs/BatteryState	          Voltage, current, SOC, cell data
/battery_health	    diagnostic_msgs/DiagnosticArray	    Faults, cycle count, balance, status

Parameters
Name	          Type	    Default	  Description
can_interface	  string	  can0	    Linux CAN interface name
timer_period	  double	  1.0	      Polling rate (seconds)
priority	      int	      0x18	    Daly CAN priority byte
bms_address	    int	      0x01	    Address of BMS device
pc_address	    int	      0x40	    Address claimed by ROS node

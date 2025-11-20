# amr_sweeper_battery
ROS2 Node to get battery values from the Daly-BMS in the AMR-Sweeper battery over CANBus

amr_sweeper_battery is a ROS 2 Python package that interfaces a Daly BMS over classic CAN (250 kbit/s, 29-bit extended IDs) and publishes battery status and health information.
The node polls the Daly BMS once per second and exposes high-level battery data using standard ROS messages.

Features
Communicates with Daly BMS using official 0x90–0x98 CAN protocol
Classic CAN 2.0B extended frames, not CAN-FD

Publishes:

- `/battery_state` (`sensor_msgs/msg/BatteryState`)
  - pack voltage, current, SOC
  - per-cell voltages
  - per-cell temperatures

- `/battery_health` (`diagnostic_msgs/msg/DiagnosticArray`)
  - faults, MOS states, balance status
  - cell voltage extremes & temperature extremes
  - remaining capacity (mAh)
  - DI/DO states

Simple, minimalistic Python implementation using rclpy + python-can
1 Hz polling rate (configurable)

Installation
Place the package into your workspace:

```bash
cd ~/ros2_ws/src
unzip amr_sweeper_battery.zip
cd ..
colcon build --packages-select amr_sweeper_battery
source install/setup.bash
```

Bring up CAN
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

Run the Node
```bash
ros2 launch amr_sweeper_battery battery.launch.py
```

<h3>Topics</h3>

<table>
  <thead>
    <tr>
      <th>Topic</th>
      <th>Message Type</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><code>/battery_state</code></td>
      <td><code>sensor_msgs/msg/BatteryState</code></td>
      <td>
        Pack voltage, current, SOC,<br>
        per-cell voltages and per-sensor temperatures.
      </td>
    </tr>
    <tr>
      <td><code>/battery_health</code></td>
      <td><code>diagnostic_msgs/msg/DiagnosticArray</code></td>
      <td>
        Fault flags, cycle count, remaining capacity,<br>
        min/max cell voltage &amp; temperature, charger/load status,<br>
        balancing cells, and other Daly BMS diagnostics.
      </td>
    </tr>
  </tbody>
</table>

<h3>Parameters</h3>

<table>
  <thead>
    <tr>
      <th>Name</th>
      <th>Type</th>
      <th>Default</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><code>can_interface</code></td>
      <td><code>string</code></td>
      <td><code>"can0"</code></td>
      <td>Linux SocketCAN interface name.</td>
    </tr>
    <tr>
      <td><code>timer_period</code></td>
      <td><code>double</code></td>
      <td><code>1.0</code></td>
      <td>Polling period in seconds (1.0 = 1&nbsp;Hz).</td>
    </tr>
    <tr>
      <td><code>priority</code></td>
      <td><code>int</code></td>
      <td><code>0x18</code></td>
      <td>Daly CAN priority byte used in the arbitration ID.</td>
    </tr>
    <tr>
      <td><code>bms_address</code></td>
      <td><code>int</code></td>
      <td><code>0x01</code></td>
      <td>Daly BMS address byte.</td>
    </tr>
    <tr>
      <td><code>pc_address</code></td>
      <td><code>int</code></td>
      <td><code>0x40</code></td>
      <td>Address claimed by this ROS 2 node (PC/tool address).</td>
    </tr>
  </tbody>
</table>

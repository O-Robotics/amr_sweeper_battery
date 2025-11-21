#!/usr/bin/env python3
from typing import Optional, List
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import can


class DalyBmsCanNode(Node):
    """
    ROS 2 node for Daly BMS over classic CAN (29-bit extended IDs, 250 kbit/s).

    Polls Daly CAN data IDs 0x90–0x98 at 1 Hz and publishes:
      - /battery_state   (sensor_msgs/BatteryState)
      - /battery_health  (diagnostic_msgs/DiagnosticArray)
    """

    DATA_IDS = [0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98]

    def __init__(self) -> None:
        super().__init__("amr_sweeper_battery")

        # Parameters
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("priority", 0x18)
        self.declare_parameter("bms_address", 0x01)
        self.declare_parameter("pc_address", 0x40)

        can_interface = self.get_parameter("can_interface").get_parameter_value().string_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        self.priority = int(self.get_parameter("priority").get_parameter_value().integer_value)
        self.bms_addr = int(self.get_parameter("bms_address").get_parameter_value().integer_value)
        self.pc_addr = int(self.get_parameter("pc_address").get_parameter_value().integer_value)

        self.get_logger().info(
            f"Using interface {can_interface}, 29-bit IDs, "
            f"prio=0x{self.priority:02X}, BMS=0x{self.bms_addr:02X}, PC=0x{self.pc_addr:02X}"
        )

        # Store CAN interface and lazy-init bus/notifier
        self.can_interface = can_interface
        self.bus: Optional[can.BusABC] = None
        self.notifier: Optional[can.Notifier] = None
        self._missing_can_warned: bool = False

        # Try to set up CAN once at startup. If this fails, node still runs.
        if not self._setup_can_bus():
            self.get_logger().warn(
                f"CAN interface '{self.can_interface}' is not available at startup; "
                "will keep retrying in the background."
            )

        # State storage
        self._lock = threading.Lock()

        # 0x90
        self.pack_voltage: Optional[float] = None
        self.pack_current: Optional[float] = None
        self.soc_percent: Optional[float] = None

        # 0x91
        self.max_cell_voltage: Optional[float] = None
        self.max_cell_index: Optional[int] = None
        self.min_cell_voltage: Optional[float] = None
        self.min_cell_index: Optional[int] = None

        # 0x92
        self.max_temp: Optional[float] = None
        self.max_temp_index: Optional[int] = None
        self.min_temp: Optional[float] = None
        self.min_temp_index: Optional[int] = None

        # 0x93
        self.state: Optional[int] = None
        self.charge_mos: Optional[int] = None
        self.discharge_mos: Optional[int] = None
        self.bms_life_cycles: Optional[int] = None
        self.remaining_capacity_mAh: Optional[int] = None

        # 0x94
        self.series_cells: Optional[int] = None
        self.temp_sensors: Optional[int] = None
        self.charger_connected: Optional[bool] = None
        self.load_connected: Optional[bool] = None
        self.di_states: Optional[List[int]] = None
        self.do_states: Optional[List[int]] = None

        # 0x95 / 0x96 / 0x97
        self.cell_voltages: List[float] = []
        self.cell_temperatures: List[float] = []
        self.balance_state: List[int] = []

        # 0x98
        self.failure_bytes: Optional[bytes] = None

        # Publishers
        self.batt_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.health_pub = self.create_publisher(DiagnosticArray, "battery_health", 10)

        # 1 Hz timer
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info("amr_sweeper_battery node started.")


    def _setup_can_bus(self, log_failure: bool = True) -> bool:
        """
        Try to (re-)create the CAN bus and notifier.

        Returns True if the bus is ready, False otherwise.
        """
        # Already set up
        if self.bus is not None:
            return True

        try:
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                bustype="socketcan",
                receive_own_messages=False,
            )
        except (OSError, can.CanError) as exc:
            if log_failure:
                self.get_logger().warn(
                    f"Failed to open CAN interface '{self.can_interface}': {exc}. "
                    "Will retry periodically."
                )
            return False

        listener = _DalyCanListener(self)
        self.notifier = can.Notifier(self.bus, [listener], 1)

        self.get_logger().info(f"Connected to CAN interface '{self.can_interface}'.")
        self._missing_can_warned = False
        return True

    # --- CAN ID helpers -----------------------------------------------------

    def _make_pc_to_bms_id(self, data_id: int) -> int:
        # PC -> BMS: Priority + Data ID + BMS Addr + PC Addr
        return (
            (self.priority << 24)
            | (data_id << 16)
            | (self.bms_addr << 8)
            | self.pc_addr
        )

    @staticmethod
    def _parse_bms_to_pc_id(arb_id: int):
        # BMS -> PC: Priority + Data ID + PC Addr + BMS Addr
        priority = (arb_id >> 24) & 0xFF
        data_id = (arb_id >> 16) & 0xFF
        dst_addr = (arb_id >> 8) & 0xFF
        src_addr = arb_id & 0xFF
        return data_id, src_addr, dst_addr, priority

    # --- Timer --------------------------------------------------------------

    def _on_timer(self) -> None:
        # If no CAN bus yet, try to bring it up and warn/publish diagnostics
        if self.bus is None:
            if not self._missing_can_warned:
                self.get_logger().warn(
                    f"No CAN interface '{self.can_interface}' detected yet; "
                    "battery data will not be updated until it appears."
                )
                self._missing_can_warned = True

            # Publish a diagnostic warning
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()

            status = DiagnosticStatus()
            status.name = "daly_bms_health"
            status.hardware_id = "daly_bms_can"
            status.level = DiagnosticStatus.WARN
            status.message = (
                f"No CAN interface '{self.can_interface}' detected; "
                "battery data is unavailable."
            )

            diag_array.status.append(status)
            self.health_pub.publish(diag_array)

            # Retry without extra log spam on every timer tick
            self._setup_can_bus(log_failure=False)
            return

        # Normal operation: bus is available
        for data_id in self.DATA_IDS:
            try:
                self._send_request(data_id)
            except can.CanError as exc:
                self.get_logger().warn(f"CAN TX failed for 0x{data_id:02X}: {exc}")

        self._publish_battery_state()
        self._publish_battery_health()

    def _send_request(self, data_id: int) -> None:
        arb_id = self._make_pc_to_bms_id(data_id)
        msg = can.Message(
            arbitration_id=arb_id,
            is_extended_id=True,
            data=bytes(8),
        )
        self.bus.send(msg)

    # --- CAN RX handling ----------------------------------------------------

    def handle_can_message(self, msg: can.Message) -> None:
        if not msg.is_extended_id:
            return

        data_id, src_addr, dst_addr, priority = self._parse_bms_to_pc_id(msg.arbitration_id)

        if src_addr != self.bms_addr or dst_addr != self.pc_addr:
            return
        if priority != self.priority:
            return

        data = msg.data

        if data_id == 0x90:
            self._decode_0x90(data)
        elif data_id == 0x91:
            self._decode_0x91(data)
        elif data_id == 0x92:
            self._decode_0x92(data)
        elif data_id == 0x93:
            self._decode_0x93(data)
        elif data_id == 0x94:
            self._decode_0x94(data)
        elif data_id == 0x95:
            self._decode_0x95(data)
        elif data_id == 0x96:
            self._decode_0x96(data)
        elif data_id == 0x97:
            self._decode_0x97(data)
        elif data_id == 0x98:
            self._decode_0x98(data)

    # --- Decoders -----------------------------------------------------------

    def _decode_0x90(self, data: bytes) -> None:
        if len(data) < 8:
            return

        # On your BMS:
        #   bytes 0–1 = pack voltage (0.1 V)
        #   bytes 2–3 = unused / second voltage (currently 0)
        pack_u16 = (data[0] << 8) | data[1]
        curr_u16 = (data[4] << 8) | data[5]
        soc_u16 = (data[6] << 8) | data[7]

        voltage = pack_u16 / 10.0
        current = (curr_u16 - 30000) / 10.0
        soc_percent = soc_u16 / 10.0

        with self._lock:
            self.pack_voltage = voltage
            self.pack_current = current
            self.soc_percent = soc_percent

    def _decode_0x91(self, data: bytes) -> None:
        if len(data) < 6:
            return

        max_mv = (data[0] << 8) | data[1]
        max_idx = data[2]
        min_mv = (data[3] << 8) | data[4]
        min_idx = data[5]

        with self._lock:
            self.max_cell_voltage = max_mv / 1000.0
            self.max_cell_index = max_idx
            self.min_cell_voltage = min_mv / 1000.0
            self.min_cell_index = min_idx

    def _decode_0x92(self, data: bytes) -> None:
        if len(data) < 4:
            return

        max_temp = data[0] - 40
        max_idx = data[1]
        min_temp = data[2] - 40
        min_idx = data[3]

        with self._lock:
            self.max_temp = float(max_temp)
            self.max_temp_index = max_idx
            self.min_temp = float(min_temp)
            self.min_temp_index = min_idx

    def _decode_0x93(self, data: bytes) -> None:
        if len(data) < 8:
            return

        state = data[0]
        chg_mos = data[1]
        dis_mos = data[2]
        life = data[3]
        rem_mAh = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]

        with self._lock:
            self.state = state
            self.charge_mos = chg_mos
            self.discharge_mos = dis_mos
            self.bms_life_cycles = life
            self.remaining_capacity_mAh = rem_mAh

    def _decode_0x94(self, data: bytes) -> None:
        if len(data) < 5:
            return

        series_cells = data[0]
        temp_sensors = data[1]
        charger_status = data[2]
        load_status = data[3]
        di_do_bits = data[4]

        di_states = [(di_do_bits >> i) & 0x1 for i in range(4)]
        do_states = [(di_do_bits >> (4 + i)) & 0x1 for i in range(4)]

        with self._lock:
            self.series_cells = series_cells
            self.temp_sensors = temp_sensors
            self.charger_connected = bool(charger_status)
            self.load_connected = bool(load_status)
            self.di_states = di_states
            self.do_states = do_states

            if series_cells and len(self.cell_voltages) < series_cells:
                self.cell_voltages = [0.0] * series_cells
            if temp_sensors and len(self.cell_temperatures) < temp_sensors:
                self.cell_temperatures = [0.0] * temp_sensors
            if series_cells and len(self.balance_state) < series_cells:
                self.balance_state = [0] * series_cells

    def _decode_0x95(self, data: bytes) -> None:
        if len(data) < 7:
            return

        frame = data[0]
        if frame == 0xFF:
            return

        with self._lock:
            for i in range(3):
                offset = 1 + i * 2
                if offset + 1 >= len(data):
                    break
                raw_mv = (data[offset] << 8) | data[offset + 1]
                # Daly frames: frame = 1..N, each frame has 3 cells
                cell_index = (frame - 1) * 3 + i  # 0-based

                if cell_index >= len(self.cell_voltages):
                    self.cell_voltages.extend(
                        [0.0] * (cell_index + 1 - len(self.cell_voltages))
                    )
                self.cell_voltages[cell_index] = raw_mv / 1000.0

    def _decode_0x96(self, data: bytes) -> None:
        if len(data) < 2:
            return

        frame = data[0]

        with self._lock:
            # If we know how many sensors we actually have, cap to that.
            max_sensors = self.temp_sensors or 0

            for pos in range(1, len(data)):
                temp_index = frame * 7 + (pos - 1)

                if max_sensors and temp_index >= max_sensors:
                    # Remaining bytes are padding; ignore.
                    break

                temp_raw = data[pos]
                if temp_raw == 0xFF:
                    # 0xFF means "no sensor / invalid" on your pack.
                    continue

                temp_c = temp_raw - 40

                if temp_index >= len(self.cell_temperatures):
                    self.cell_temperatures.extend(
                        [0.0] * (temp_index + 1 - len(self.cell_temperatures))
                    )
                self.cell_temperatures[temp_index] = float(temp_c)


    def _decode_0x97(self, data: bytes) -> None:
        if len(data) < 8:
            return

        with self._lock:
            num_cells = max(len(self.balance_state), 48)
            self.balance_state = [0] * num_cells

            for bit in range(48):
                byte_idx = bit // 8
                bit_idx = bit % 8
                val = (data[byte_idx] >> bit_idx) & 0x1
                if bit < len(self.balance_state):
                    self.balance_state[bit] = val

    def _decode_0x98(self, data: bytes) -> None:
        if len(data) < 8:
            return
        with self._lock:
            self.failure_bytes = bytes(data[:8])

    # --- Publishers ---------------------------------------------------------

    def _publish_battery_state(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        with self._lock:
            voltage = self.pack_voltage
            current = self.pack_current
            soc = self.soc_percent
            cell_voltages = list(self.cell_voltages)
            cell_temps = list(self.cell_temperatures)
            state = self.state

        if voltage is not None:
            msg.voltage = float(voltage)
        if current is not None:
            msg.current = float(current)
        if soc is not None:
            msg.percentage = float(soc) / 100.0

        if cell_voltages:
            msg.cell_voltage = [float(v) for v in cell_voltages]
        if cell_temps:
            msg.cell_temperature = [float(t) for t in cell_temps]
            msg.temperature = max(cell_temps)

        if state is not None:
            if state == 1:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif state == 2:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            else:
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        self.batt_pub.publish(msg)

    def _publish_battery_health(self) -> None:
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "daly_bms_health"
        status.hardware_id = "daly_bms_can"

        with self._lock:
            fb = self.failure_bytes
            cycles = self.bms_life_cycles
            rem_cap = self.remaining_capacity_mAh
            max_v = self.max_cell_voltage
            max_v_idx = self.max_cell_index
            min_v = self.min_cell_voltage
            min_v_idx = self.min_cell_index
            max_t = self.max_temp
            max_t_idx = self.max_temp_index
            min_t = self.min_temp
            min_t_idx = self.min_temp_index
            charger = self.charger_connected
            load = self.load_connected
            series_cells = self.series_cells
            temp_sensors = self.temp_sensors
            balances = list(self.balance_state)

        values: List[KeyValue] = []

        def kv(key: str, value: str) -> KeyValue:
            k = KeyValue()
            k.key = key
            k.value = value
            return k

        if series_cells is not None:
            values.append(kv("series_cells", str(series_cells)))
        if temp_sensors is not None:
            values.append(kv("temp_sensors", str(temp_sensors)))
        if cycles is not None:
            values.append(kv("bms_life_cycles", str(cycles)))
        if rem_cap is not None:
            values.append(kv("remaining_capacity_mAh", str(rem_cap)))

        if max_v is not None:
            values.append(kv("max_cell_voltage_V", f"{max_v:.3f}"))
        if max_v_idx is not None:
            values.append(kv("max_cell_index", str(max_v_idx)))
        if min_v is not None:
            values.append(kv("min_cell_voltage_V", f"{min_v:.3f}"))
        if min_v_idx is not None:
            values.append(kv("min_cell_index", str(min_v_idx)))

        if max_t is not None:
            values.append(kv("max_temperature_C", f"{max_t:.1f}"))
        if max_t_idx is not None:
            values.append(kv("max_temp_index", str(max_t_idx)))
        if min_t is not None:
            values.append(kv("min_temperature_C", f"{min_t:.1f}"))
        if min_t_idx is not None:
            values.append(kv("min_temp_index", str(min_t_idx)))

        if charger is not None:
            values.append(kv("charger_connected", str(charger)))
        if load is not None:
            values.append(kv("load_connected", str(load)))

        if balances:
            balancing_cells = [str(i + 1) for i, b in enumerate(balances) if b]
            if balancing_cells:
                values.append(kv("balancing_cells", ",".join(balancing_cells)))

        fault_messages: List[str] = []
        if fb is not None:
            bit_descriptions = {
                (0, 0): "Cell voltage high level 1",
                (0, 1): "Cell voltage high level 2",
                (0, 2): "Cell voltage low level 1",
                (0, 3): "Cell voltage low level 2",
                (0, 4): "Pack voltage high level 1",
                (0, 5): "Pack voltage high level 2",
                (0, 6): "Pack voltage low level 1",
                (0, 7): "Pack voltage low level 2",
                (1, 0): "Charge temp high level 1",
                (1, 1): "Charge temp high level 2",
                (1, 2): "Charge temp low level 1",
                (1, 3): "Charge temp low level 2",
                (1, 4): "Discharge temp high level 1",
                (1, 5): "Discharge temp high level 2",
                (1, 6): "Discharge temp low level 1",
                (1, 7): "Discharge temp low level 2",
                (2, 0): "Charge overcurrent level 1",
                (2, 1): "Charge overcurrent level 2",
                (2, 2): "Discharge overcurrent level 1",
                (2, 3): "Discharge overcurrent level 2",
                (2, 4): "SOC high level 1",
                (2, 5): "SOC high level 2",
                (2, 6): "SOC low level 1",
                (2, 7): "SOC low level 2",
                (3, 0): "Voltage difference level 1",
                (3, 1): "Voltage difference level 2",
                (3, 2): "Temperature difference level 1",
                (3, 3): "Temperature difference level 2",
                (4, 0): "Charge MOS temp high alarm",
                (4, 1): "Discharge MOS temp high alarm",
                (4, 2): "Charge MOS temp sensor error",
                (4, 3): "Discharge MOS temp sensor error",
                (4, 4): "Charge MOS adhesion error",
                (4, 5): "Discharge MOS adhesion error",
                (4, 6): "Charge MOS open circuit error",
                (4, 7): "Discharge MOS open circuit error",
                (5, 0): "AFE collect chip error",
                (5, 1): "Voltage collect dropped",
                (5, 2): "Cell temp sensor error",
                (5, 3): "EEPROM error",
                (5, 4): "RTC error",
                (5, 5): "Precharge failure",
                (5, 6): "Communication failure",
                (5, 7): "Internal communication failure",
                (6, 0): "Current module fault",
                (6, 1): "Pack voltage detect fault",
                (6, 2): "Short circuit protection fault",
                (6, 3): "Low voltage forbidden charge fault",
            }

            any_fault = False
            for byte_index in range(7):
                b = fb[byte_index]
                if b == 0:
                    continue
                for bit in range(8):
                    if b & (1 << bit):
                        any_fault = True
                        desc = bit_descriptions.get(
                            (byte_index, bit),
                            f"Unknown fault byte{byte_index}_bit{bit}",
                        )
                        fault_messages.append(desc)

            values.append(kv("failure_bytes_hex", fb.hex()))
            if fault_messages:
                values.append(kv("active_faults", "; ".join(fault_messages)))

            status.level = (
                DiagnosticStatus.ERROR if any_fault else DiagnosticStatus.OK
            )
            status.message = "Fault(s) present" if any_fault else "No faults"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "No failure frame (0x98) received yet"

        status.values = values
        diag_array.status.append(status)
        self.health_pub.publish(diag_array)


class _DalyCanListener(can.Listener):
    def __init__(self, node: DalyBmsCanNode) -> None:
        super().__init__()
        self._node = node

    def on_message_received(self, msg: can.Message) -> None:
        self._node.handle_can_message(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DalyBmsCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.notifier is not None:
            node.notifier.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

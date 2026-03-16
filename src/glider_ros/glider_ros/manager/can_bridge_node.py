"""
CAN Bridge Node for UCL Underwater Glider
Handles communication between the Pi (ROS2) and 3 Teensys over CAN bus.
Matches the Notion VBD Teensy and P&R Teensy spec exactly.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
import struct
import can

# CAN message bases — add node_id to get actual CAN ID
# Lower ID = higher priority on the bus
FAULT_BASE = 0x050      # Teensy -> Pi, 2Hz, fault reports
CMD_BASE = 0x100        # Pi -> Teensy, 20Hz, position commands
STATUS_BASE = 0x200     # Teensy -> Pi, 50Hz, position + motor state
BMS_BASE = 0x210        # Teensy -> Pi, 1Hz, battery health

# Teensy node IDs
VBD_L = 1
VBD_R = 2
PR = 3

# VBD piston range: 0-1500 on CAN = 0-150mm physical stroke
VBD_STROKE_MAX = 1500

# All 19 faults from Notion spec
# Key = (byte number, bit number), value = fault name
HARD_FAULTS = {
    # Byte 0
    (0, 0): "LEAK",           (0, 1): "DRIVER_FLT",
    (0, 2): "OVERCURRENT",    (0, 3): "STALL",
    (0, 4): "CMD_TO_L",       (0, 5): "NOT_HOMED",
    (0, 6): "POS_LIM",        (0, 7): "BMS_TEMP",
    # Byte 1
    (1, 0): "BMS_TO_L",       (1, 1): "BMS_OC",
    (1, 2): "BMS_SWITCH",
}

SOFT_FAULTS = {
    # Byte 2
    (2, 0): "CMD_TO",         (2, 1): "TOF_INV",
    (2, 2): "TOF_OOR",        (2, 3): "SLIP",
    (2, 4): "ENCODER_INV",    (2, 5): "BMS_TO",
    (2, 6): "BMS_LOW",        (2, 7): "BMS_HIGH",
}


class CanBridgeNode(Node):
    def __init__(self):
        super().__init__("can_bridge_node")

        # CAN setup
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bitrate", 500000)
        channel = self.get_parameter("can_channel").value
        bitrate = self.get_parameter("can_bitrate").value

        try:
            self.bus = can.interface.Bus(channel=channel, bustype="socketcan", bitrate=bitrate)
            self.get_logger().info(f"CAN connected on {channel} at {bitrate} bps")
        except Exception as e:
            self.get_logger().error(f"Failed to open CAN bus: {e}")
            self.bus = None
            return

        # Data from Teensys (updated by read_can)
        self.vbd_left_pos = 0.0         # 0.0 to 1.0
        self.vbd_right_pos = 0.0
        self.pitch_angle = 0.0          # degrees
        self.roll_angle = 0.0           # degrees

        self.leak_left = False
        self.leak_right = False
        self.leak_pr = False

        self.batt_left_v = 0.0          # volts
        self.batt_right_v = 0.0
        self.batt_left_temp = 0         # tenths of degree C
        self.batt_right_temp = 0

        self.state_left = "UNKNOWN"     # "RUN:OK" or "FAULT:LEAK,STALL"
        self.state_right = "UNKNOWN"
        self.state_pr = "UNKNOWN"

        self.first_fault_left = 0       # byte 4 of STATUS_FAULT
        self.first_fault_right = 0
        self.seq_left = 0               # sequence counters
        self.seq_right = 0

        # Commands to Teensys (updated by callbacks, sent by send_commands)
        self.target_vbd_left = 0        # stroke percentage 0-100
        self.target_vbd_right = 0
        self.target_pitch = 50          # percentage, 50 = level = 0 degrees
        self.motors_enabled = False
        self.homing_requested = False
        self.emergency = False
        self.cmd_seq = 0                # outgoing sequence counter

        # Publishers
        self.pub_vbd_left = self.create_publisher(Float64, "/ug/vbd/left/position", 10)
        self.pub_vbd_right = self.create_publisher(Float64, "/ug/vbd/right/position", 10)
        self.pub_pitch = self.create_publisher(Float64, "/ug/pitch/angle", 10)
        self.pub_roll = self.create_publisher(Float64, "/ug/roll/angle", 10)
        self.pub_leak_left = self.create_publisher(Bool, "/ug/leak/left", 10)
        self.pub_leak_right = self.create_publisher(Bool, "/ug/leak/right", 10)
        self.pub_leak_pr = self.create_publisher(Bool, "/ug/leak/pr", 10)
        self.pub_batt_left = self.create_publisher(Float64, "/ug/battery/left", 10)
        self.pub_batt_right = self.create_publisher(Float64, "/ug/battery/right", 10)
        self.pub_teensy_left = self.create_publisher(String, "/ug/teensy/vbd_left/status", 10)
        self.pub_teensy_right = self.create_publisher(String, "/ug/teensy/vbd_right/status", 10)
        self.pub_teensy_pr = self.create_publisher(String, "/ug/teensy/pr/status", 10)

        # Subscribers
        self.create_subscription(Float64, "/ug/cmd/vbd/left", self.on_cmd_vbd_left, 10)
        self.create_subscription(Float64, "/ug/cmd/vbd/right", self.on_cmd_vbd_right, 10)
        self.create_subscription(Float64, "/ug/cmd/pitch", self.on_cmd_pitch, 10)
        self.create_subscription(Float64, "/ug/cmd/roll", self.on_cmd_roll, 10)
        self.create_subscription(Bool, "/ug/cmd/emergency", self.on_cmd_emergency, 10)

        # Timers
        self.create_timer(0.02, self.read_can)          # 50Hz
        self.create_timer(0.05, self.send_commands)      # 20Hz
        self.create_timer(0.1, self.publish_feedback)    # 10Hz
        self.create_timer(1.0, self.publish_health)      #  1Hz

        self.get_logger().info("Bridge node ready")

    # ===========================================
    # READING CAN MESSAGES
    # ===========================================

    def read_can(self):
        if not self.bus:
            return
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            self.route_message(msg.arbitration_id, msg.data)

    def route_message(self, can_id, data):
        if STATUS_BASE < can_id <= STATUS_BASE + 3:
            self.parse_status(can_id - STATUS_BASE, data)
        elif FAULT_BASE < can_id <= FAULT_BASE + 3:
            self.parse_fault(can_id - FAULT_BASE, data)
        elif BMS_BASE < can_id <= BMS_BASE + 3:
            self.parse_bms(can_id - BMS_BASE, data)

    def parse_status(self, node_id, data):
        """Parse STATUS_CONTROL (0x200+id): position, tof, flags, pwm, current"""
        if len(data) < 7:
            return

        # Bytes 0-1: estimated position in tenths of mm (uint16 LE)
        pos_mm = struct.unpack_from('<H', data, 0)[0]

        # Bytes 2-3: ToF measured position (uint16 LE)
        tof_mm = struct.unpack_from('<H', data, 2)[0]

        # Byte 4: status flags
        leak = bool(data[4] & (1 << 1))
        homed = bool(data[4] & (1 << 3))
        moving = bool(data[4] & (1 << 4))
        motor_on = bool(data[4] & (1 << 5))

        # Byte 5: PWM value (0-255)
        pwm = data[5]

        # Byte 7: current sense
        current = data[7]

        # Store based on which Teensy sent it
        if node_id == VBD_L:
            self.vbd_left_pos = pos_mm / VBD_STROKE_MAX
            self.leak_left = leak
        elif node_id == VBD_R:
            self.vbd_right_pos = pos_mm / VBD_STROKE_MAX
            self.leak_right = leak
        elif node_id == PR:
            # P&R reports position as travel percentage
            self.pitch_angle = (pos_mm / 100.0) * 50.0 - 25.0
            self.leak_pr = leak

    def parse_fault(self, node_id, data):
        """Parse STATUS_FAULT (0x050+id): fault bits, state, first fault, seq"""
        if len(data) < 7:
            return

        # Check all hard fault bits (bytes 0-1)
        hard = []
        for (byte_num, bit_num), name in HARD_FAULTS.items():
            if (data[byte_num] >> bit_num) & 1:
                hard.append(name)

        # Check all soft fault bits (byte 2)
        soft = []
        for (byte_num, bit_num), name in SOFT_FAULTS.items():
            if (data[byte_num] >> bit_num) & 1:
                soft.append(name)

        # Byte 4: which hard fault triggered first
        first_fault = data[4]

        # Byte 5: Teensy state
        state_byte = data[5]
        if state_byte & 1:
            state = "INIT"
        elif state_byte & 2:
            state = "RUN"
        elif state_byte & 4:
            state = "FAULT"
        else:
            state = "UNKNOWN"

        # Byte 6: sequence counter
        seq = data[6]

        # Build status string
        all_faults = hard + soft
        fault_text = ",".join(all_faults) if all_faults else "OK"
        status_text = f"{state}:{fault_text}"

        # Store per Teensy
        if node_id == VBD_L:
            self.state_left = status_text
            self.first_fault_left = first_fault
            self.seq_left = seq
            if "LEAK" in hard:
                self.leak_left = True
        elif node_id == VBD_R:
            self.state_right = status_text
            self.first_fault_right = first_fault
            self.seq_right = seq
            if "LEAK" in hard:
                self.leak_right = True
        elif node_id == PR:
            self.state_pr = status_text
            if "LEAK" in hard:
                self.leak_pr = True

    def parse_bms(self, node_id, data):
        """Parse STATUS_BMS (0x210+id): voltage, temp, BMS faults"""
        if len(data) < 5:
            return

        # Bytes 0-1: pack voltage in millivolts (uint16 LE)
        millivolts = struct.unpack_from('<H', data, 0)[0]
        volts = millivolts / 1000.0

        # Bytes 2-3: pack temperature (int16 LE, signed)
        temp = struct.unpack_from('<h', data, 2)[0]

        # Byte 4: BMS fault flags
        bms_flags = data[4]

        if node_id == VBD_L:
            self.batt_left_v = volts
            self.batt_left_temp = temp
        elif node_id == VBD_R:
            self.batt_right_v = volts
            self.batt_right_temp = temp

    # ===========================================
    # SENDING CAN COMMANDS
    # ===========================================

    def send_commands(self):
        """Send CMD_SETPOINT to all 3 Teensys at 20Hz"""
        if not self.bus:
            return

        self.cmd_seq = (self.cmd_seq + 1) % 256

        self.send_setpoint(CMD_BASE + VBD_L, self.target_vbd_left)
        self.send_setpoint(CMD_BASE + VBD_R, self.target_vbd_right)
        self.send_setpoint(CMD_BASE + PR, self.target_pitch)

    def send_setpoint(self, can_id, stroke_percent):
        """Build and send one CMD_SETPOINT frame matching Notion spec"""
        frame = bytearray(8)

        # Byte 0: stroke percentage (uint8, 0-100)
        frame[0] = max(0, min(100, stroke_percent))

        # Byte 3: control bits
        ctrl = 0
        if self.motors_enabled:
            ctrl |= (1 << 0)       # bit 0 = ENABLE
        if self.homing_requested:
            ctrl |= (1 << 1)       # bit 1 = HOMING
        frame[3] = ctrl

        # Byte 6: sequence counter
        frame[6] = self.cmd_seq

        try:
            self.bus.send(can.Message(
                arbitration_id=can_id,
                data=bytes(frame),
                is_extended_id=False
            ))
        except can.CanError as e:
            self.get_logger().error(f"CAN send error 0x{can_id:03X}: {e}")

    # ===========================================
    # COMMAND CALLBACKS
    # ===========================================

    def on_cmd_vbd_left(self, msg):
        """ROS2 0.0-1.0 -> CAN stroke percentage 0-100"""
        val = max(0.0, min(1.0, msg.data))
        self.target_vbd_left = int(val * 100)
        self.motors_enabled = True

    def on_cmd_vbd_right(self, msg):
        val = max(0.0, min(1.0, msg.data))
        self.target_vbd_right = int(val * 100)
        self.motors_enabled = True

    def on_cmd_pitch(self, msg):
        """ROS2 degrees -> CAN percentage. -25=0%, 0=50%, +25=100%"""
        deg = max(-25.0, min(25.0, msg.data))
        self.target_pitch = int(((deg + 25.0) / 50.0) * 100.0)
        self.motors_enabled = True

    def on_cmd_roll(self, msg):
        pass  # TODO: define roll CAN frame with team

    def on_cmd_emergency(self, msg):
        if msg.data:
            self.emergency = True
            self.target_vbd_left = 100      # 100% = fully extended = buoyant
            self.target_vbd_right = 100
            self.target_pitch = 50          # 50% = level
            self.motors_enabled = True
            self.get_logger().warn("EMERGENCY SURFACE")
        else:
            self.emergency = False

    # ===========================================
    # PUBLISHING TO ROS2
    # ===========================================

    def publish_feedback(self):
        """10Hz: push Teensy positions to ROS2"""
        self.pub_float(self.pub_vbd_left, self.vbd_left_pos)
        self.pub_float(self.pub_vbd_right, self.vbd_right_pos)
        self.pub_float(self.pub_pitch, self.pitch_angle)
        self.pub_float(self.pub_roll, self.roll_angle)

    def publish_health(self):
        """1Hz: push leaks, batteries, Teensy states to ROS2"""
        self.pub_bool(self.pub_leak_left, self.leak_left)
        self.pub_bool(self.pub_leak_right, self.leak_right)
        self.pub_bool(self.pub_leak_pr, self.leak_pr)

        self.pub_float(self.pub_batt_left, self.batt_left_v)
        self.pub_float(self.pub_batt_right, self.batt_right_v)

        self.pub_str(self.pub_teensy_left, self.state_left)
        self.pub_str(self.pub_teensy_right, self.state_right)
        self.pub_str(self.pub_teensy_pr, self.state_pr)

        self.get_logger().info(
            f"VBD L={self.vbd_left_pos:.2f} R={self.vbd_right_pos:.2f} "
            f"Pitch={self.pitch_angle:+.1f}deg "
            f"Batt={self.batt_left_v:.1f}/{self.batt_right_v:.1f}V "
            f"States=[{self.state_left}][{self.state_right}][{self.state_pr}] "
            f"{'EMERG' if self.emergency else 'OK'}")

    def pub_float(self, pub, val):
        msg = Float64()
        msg.data = float(val)
        pub.publish(msg)

    def pub_bool(self, pub, val):
        msg = Bool()
        msg.data = bool(val)
        pub.publish(msg)

    def pub_str(self, pub, val):
        msg = String()
        msg.data = str(val)
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

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
FAULT_BASE  = 0x050     # Teensy -> Pi, 2Hz,  fault reports
CMD_BASE    = 0x100     # Pi -> Teensy, 20Hz, position commands
STATUS_BASE = 0x200     # Teensy -> Pi, 50Hz, position + motor state
BMS_BASE    = 0x210     # Teensy -> Pi, 1Hz,  battery health

# Teensy node IDs
VBD_L = 1
VBD_R = 2
PR    = 3

# Position scaling
VBD_STROKE_MIN_MM = 10.0
VBD_RAW_PER_MM  = 100.0
VBD_STROKE_MAX_MM = 145.0
PR_STROKE_MAX_MM  = 110.0

# All 19 faults from Notion spec
HARD_FAULTS = {
    (0, 0): "LEAK",        (0, 1): "DRIVER_FLT",
    (0, 2): "OVERCURRENT", (0, 3): "STALL",
    (0, 4): "CMD_TO_L",    (0, 5): "NOT_HOMED",
    (0, 6): "POS_LIM",     (0, 7): "BMS_TEMP",
    (1, 0): "BMS_TO_L",    (1, 1): "BMS_OC",
    (1, 2): "BMS_SWITCH",
}

SOFT_FAULTS = {
    (2, 0): "CMD_TO",      (2, 1): "TOF_INV",
    (2, 2): "TOF_OOR",     (2, 3): "SLIP",
    (2, 4): "ENCODER_INV", (2, 5): "BMS_TO",
    (2, 6): "BMS_LOW",     (2, 7): "BMS_HIGH",
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

        # Data from Teensys
        self.vbd_left_pos  = 0.0
        self.vbd_right_pos = 0.0
        self.pitch_angle   = 0.0
        self.roll_angle    = 0.0

        self.leak_left  = False
        self.leak_right = False
        self.leak_pr    = False

        self.batt_left_v     = 0.0
        self.batt_right_v    = 0.0
        self.batt_left_temp  = 0
        self.batt_right_temp = 0

        self.state_left  = "UNKNOWN"
        self.state_right = "UNKNOWN"
        self.state_pr    = "UNKNOWN"

        self.first_fault_left  = 0
        self.first_fault_right = 0
        self.seq_left  = 0
        self.seq_right = 0

        self.tof_left_mm  = 0.0
        self.tof_right_mm = 0.0

        # Iridium data
        self.iridium_incoming_message = ""
        self.iridium_signal_strength = ""
        self.iridium_status = ""

        # Commands to Teensys
        self.target_vbd_left  = 0
        self.target_vbd_right = 0
        self.target_pitch     = 50
        self.motors_enabled   = False
        self.homing_requested = False
        self.emergency        = False
        self.cmd_seq          = 0

        # Publishers
        self.pub_vbd_left     = self.create_publisher(Float64, "/ug/vbd/left/position", 10)
        self.pub_vbd_right    = self.create_publisher(Float64, "/ug/vbd/right/position", 10)
        self.pub_pitch        = self.create_publisher(Float64, "/ug/pitch/angle", 10)
        self.pub_roll         = self.create_publisher(Float64, "/ug/roll/angle", 10)
        self.pub_leak_left    = self.create_publisher(Bool, "/ug/leak/left", 10)
        self.pub_leak_right   = self.create_publisher(Bool, "/ug/leak/right", 10)
        self.pub_leak_pr      = self.create_publisher(Bool, "/ug/leak/pr", 10)
        self.pub_batt_left    = self.create_publisher(Float64, "/ug/battery/left", 10)
        self.pub_batt_right   = self.create_publisher(Float64, "/ug/battery/right", 10)
        self.pub_teensy_left  = self.create_publisher(String, "/ug/teensy/vbd_left/status", 10)
        self.pub_teensy_right = self.create_publisher(String, "/ug/teensy/vbd_right/status", 10)
        self.pub_teensy_pr    = self.create_publisher(String, "/ug/teensy/pr/status", 10)
        self.pub_tof_left     = self.create_publisher(Float64, "/ug/tof/left", 10)
        self.pub_tof_right    = self.create_publisher(Float64, "/ug/tof/right", 10)
        self.pub_all          = self.create_publisher(String, "/ug/raw", 10)

        # New Iridium-related publishers
        self.pub_iridium_incoming = self.create_publisher(String, "/ug/iridium/incoming_message", 10)
        self.pub_iridium_signal   = self.create_publisher(String, "/ug/iridium/signal_strength", 10)
        self.pub_iridium_status   = self.create_publisher(String, "/ug/iridium/status", 10)

        # Subscribers for CAN commands
        self.create_subscription(Float64, "/ug/cmd/vbd/left", self.on_cmd_vbd_left, 10)
        self.create_subscription(Float64, "/ug/cmd/vbd/right", self.on_cmd_vbd_right, 10)
        self.create_subscription(Float64, "/ug/cmd/pitch", self.on_cmd_pitch, 10)
        self.create_subscription(Float64, "/ug/cmd/roll", self.on_cmd_roll, 10)
        self.create_subscription(Bool, "/ug/cmd/emergency", self.on_cmd_emergency, 10)

        # New subscribers from Iridium node
        self.create_subscription(String, "/iridium/incoming_message", self.on_iridium_incoming_message, 10)
        self.create_subscription(String, "/iridium/signal_strength", self.on_iridium_signal_strength, 10)
        self.create_subscription(String, "/iridium/status", self.on_iridium_status, 10)

        # Timers
        self.create_timer(0.02, self.read_can)
        self.create_timer(0.05, self.send_commands)
        self.create_timer(0.1,  self.publish_feedback)
        self.create_timer(1.0,  self.publish_health)

        self.get_logger().info("Bridge node ready")

    # ===========================================
    # IRIDIUM CALLBACKS
    # ===========================================

    def on_iridium_incoming_message(self, msg: String):
        self.iridium_incoming_message = msg.data
        self.pub_str(self.pub_iridium_incoming, self.iridium_incoming_message)
        self.get_logger().info(f"Iridium incoming message: {self.iridium_incoming_message}")

    def on_iridium_signal_strength(self, msg: String):
        self.iridium_signal_strength = msg.data
        self.pub_str(self.pub_iridium_signal, self.iridium_signal_strength)

    def on_iridium_status(self, msg: String):
        self.iridium_status = msg.data
        self.pub_str(self.pub_iridium_status, self.iridium_status)

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
        if STATUS_BASE + 1 <= can_id <= STATUS_BASE + 3:
            self.parse_status(can_id - STATUS_BASE, data)
        elif FAULT_BASE + 1 <= can_id <= FAULT_BASE + 3:
            self.parse_fault(can_id - FAULT_BASE, data)
        elif BMS_BASE + 1 <= can_id <= BMS_BASE + 3:
            self.parse_bms(can_id - BMS_BASE, data)

    def parse_status(self, node_id, data):
        if len(data) < 8:
            return

        raw_pos = struct.unpack_from('<H', data, 0)[0]
        pos_mm  = raw_pos / VBD_RAW_PER_MM

        tof_mm = struct.unpack_from('<H', data, 2)[0]

        flags    = data[4]
        leak     = bool(flags & (1 << 1))
        homed    = bool(flags & (1 << 3))
        moving   = bool(flags & (1 << 4))
        motor_on = bool(flags & (1 << 5))

        pwm     = data[5]
        current = struct.unpack_from('<b', data, 7)[0]

        if node_id == VBD_L:
            self.vbd_left_pos = (pos_mm - VBD_STROKE_MIN_MM) / (VBD_STROKE_MAX_MM - VBD_STROKE_MIN_MM)
            self.leak_left    = leak
            self.tof_left_mm  = float(tof_mm)
        elif node_id == VBD_R:
            self.vbd_right_pos = (pos_mm - VBD_STROKE_MIN_MM) / (VBD_STROKE_MAX_MM - VBD_STROKE_MIN_MM)
            self.leak_right    = leak
            self.tof_right_mm  = float(tof_mm)
        elif node_id == PR:
            self.pitch_angle = (pos_mm / PR_STROKE_MAX_MM) * 50.0 - 25.0
            self.leak_pr     = leak

    def parse_fault(self, node_id, data):
        if len(data) < 7:
            return

        hard = [name for (b, bit), name in HARD_FAULTS.items() if (data[b] >> bit) & 1]
        soft = [name for (b, bit), name in SOFT_FAULTS.items() if (data[b] >> bit) & 1]

        first_fault = data[4]

        state_byte = data[5]
        if   state_byte & 1: state = "INIT"
        elif state_byte & 2: state = "RUN"
        elif state_byte & 4: state = "FAULT"
        else:                state = "UNKNOWN"

        seq         = data[6]
        fault_text  = ",".join(hard + soft) if (hard or soft) else "OK"
        status_text = f"{state}:{fault_text}"

        if node_id == VBD_L:
            self.state_left        = status_text
            self.first_fault_left  = first_fault
            self.seq_left          = seq
            if "LEAK" in hard:
                self.leak_left = True
        elif node_id == VBD_R:
            self.state_right       = status_text
            self.first_fault_right = first_fault
            self.seq_right         = seq
            if "LEAK" in hard:
                self.leak_right = True
        elif node_id == PR:
            self.state_pr = status_text
            if "LEAK" in hard:
                self.leak_pr = True

    def parse_bms(self, node_id, data):
        if len(data) < 5:
            return

        volts = struct.unpack_from('<H', data, 0)[0] / 1000.0
        temp  = struct.unpack_from('<h', data, 2)[0]

        if node_id == VBD_L:
            self.batt_left_v    = volts
            self.batt_left_temp = temp
        elif node_id == VBD_R:
            self.batt_right_v    = volts
            self.batt_right_temp = temp

    # ===========================================
    # SENDING CAN COMMANDS
    # ===========================================

    def send_commands(self):
        if not self.bus:
            return
        self.cmd_seq = (self.cmd_seq + 1) % 256
        self.send_setpoint(CMD_BASE + VBD_L, self.target_vbd_left)
        self.send_setpoint(CMD_BASE + VBD_R, self.target_vbd_right)
        self.send_setpoint(CMD_BASE + PR, self.target_pitch)

    def send_setpoint(self, can_id, stroke_percent):
        frame    = bytearray(8)
        frame[0] = max(0, min(100, int(stroke_percent)))

        ctrl = 0
        if self.motors_enabled:
            ctrl |= (1 << 0)
        if self.homing_requested:
            ctrl |= (1 << 1)
        frame[3] = ctrl
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
        self.target_vbd_left = int(max(0.0, min(1.0, msg.data)) * 100)
        self.motors_enabled  = True

    def on_cmd_vbd_right(self, msg):
        self.target_vbd_right = int(max(0.0, min(1.0, msg.data)) * 100)
        self.motors_enabled   = True

    def on_cmd_pitch(self, msg):
        deg = max(-25.0, min(25.0, msg.data))
        self.target_pitch   = int(((deg + 25.0) / 50.0) * 100.0)
        self.motors_enabled = True

    def on_cmd_roll(self, msg):
        pass

    def on_cmd_emergency(self, msg):
        if msg.data:
            self.emergency        = True
            self.target_vbd_left  = 100
            self.target_vbd_right = 100
            self.target_pitch     = 50
            self.motors_enabled   = True
            self.get_logger().warn("EMERGENCY SURFACE")
        else:
            self.emergency = False

    # ===========================================
    # PUBLISHING TO ROS2
    # ===========================================

    def publish_feedback(self):
        self.pub_float(self.pub_vbd_left, self.vbd_left_pos)
        self.pub_float(self.pub_vbd_right, self.vbd_right_pos)
        self.pub_float(self.pub_pitch, self.pitch_angle)
        self.pub_float(self.pub_roll, self.roll_angle)
        self.pub_float(self.pub_tof_left, self.tof_left_mm)
        self.pub_float(self.pub_tof_right, self.tof_right_mm)

    def publish_health(self):
        self.pub_bool(self.pub_leak_left, self.leak_left)
        self.pub_bool(self.pub_leak_right, self.leak_right)
        self.pub_bool(self.pub_leak_pr, self.leak_pr)

        self.pub_float(self.pub_batt_left, self.batt_left_v)
        self.pub_float(self.pub_batt_right, self.batt_right_v)

        self.pub_str(self.pub_teensy_left, self.state_left)
        self.pub_str(self.pub_teensy_right, self.state_right)
        self.pub_str(self.pub_teensy_pr, self.state_pr)

        self.pub_str(
            self.pub_all,
            f"pos_l={self.vbd_left_pos:.3f} pos_r={self.vbd_right_pos:.3f} "
            f"tof_l={self.tof_left_mm:.1f}mm tof_r={self.tof_right_mm:.1f}mm "
            f"pitch={self.pitch_angle:+.1f}deg "
            f"leak_l={self.leak_left} leak_r={self.leak_right} leak_pr={self.leak_pr} "
            f"batt_l={self.batt_left_v:.2f}V batt_r={self.batt_right_v:.2f}V "
            f"state_l={self.state_left} state_r={self.state_right} state_pr={self.state_pr} "
            f"iridium_csq={self.iridium_signal_strength} "
            f"iridium_status={self.iridium_status} "
            f"iridium_msg={self.iridium_incoming_message}"
        )

        self.get_logger().info(
            f"VBD L={self.vbd_left_pos:.2f} R={self.vbd_right_pos:.2f} "
            f"Pitch={self.pitch_angle:+.1f}deg "
            f"Batt={self.batt_left_v:.1f}/{self.batt_right_v:.1f}V "
            f"States=[{self.state_left}][{self.state_right}][{self.state_pr}] "
            f"Iridium CSQ={self.iridium_signal_strength} "
            f"{'EMERG' if self.emergency else 'OK'}"
        )

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
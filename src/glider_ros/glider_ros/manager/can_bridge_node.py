#!/usr/bin/env python3
"""
can_bridge_node.py

Bidirectional bridge between CAN bus (3x Teensy 4.0) and ROS 2 topics.
Matches Jasper's CAN protocol specs exactly:
  - P_R_TEENSY (Node ID 3)
  - VBD_TEENSY (Node ID 1 = Left, Node ID 2 = Right)

Key differences between P&R and VBD:
  CMD:    P&R sends int16 pitch(0.1mm) + int16 roll(0.1deg)
          VBD sends uint8 stroke percentage (0-100)
  STATUS: P&R byte 4-5 = different flag layout than VBD byte 4
          VBD leak = byte4 bit1, P&R leak = byte5 bit2
          VBD pos/tof = uint16 little-endian, P&R = int16 (endianness TBC)
  FAULT:  Completely different bit names per node type
"""

import struct
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64, Bool, UInt8, String

from glider_msgs.action import HomeActuators


VBD_L  = 1
VBD_R  = 2
PR     = 3

CMD_BASE    = 0x100
STATUS_BASE = 0x200
FAULT_BASE  = 0x050
BMS_BASE    = 0x210


class CanBridgeNode(Node):

    def __init__(self):
        super().__init__('can_bridge_node')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('republish_rate_hz', 20.0)

        channel = self.get_parameter('can_channel').value

        try:
            import can as _can
            self.can = _can
            self.bus = _can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(f'CAN bus opened on {channel}')
        except Exception as e:
            self.get_logger().error(f'Failed to open CAN: {e}')
            raise

        self.seq_pr = 0
        self.seq_vbd_l = 0
        self.seq_vbd_r = 0

        self._setup_pr_interface()
        self._setup_vbd_interface('left', VBD_L)
        self._setup_vbd_interface('right', VBD_R)

        # Internal homed/fault state — written from CAN RX thread, read by action server thread
        self._home_lock = threading.Lock()
        self._vbd_left_homed_state = False
        self._vbd_right_homed_state = False
        self._pr_pitch_homed_state = False
        self._pr_roll_homed_state = False
        self._vbd_left_not_homed_fault = False
        self._vbd_right_not_homed_fault = False
        self._pr_pitch_n_home_fault = False
        self._pr_roll_n_home_fault = False

        self._home_action_server = ActionServer(
            self,
            HomeActuators,
            '/bridge/home_actuators',
            self._home_execute,
        )

        self._running = True
        self._rx_thread = threading.Thread(target=self._can_rx_loop, daemon=True)
        self._rx_thread.start()

        rate = self.get_parameter('republish_rate_hz').value
        self.create_timer(1.0 / rate, self._republish_commands)

        self.get_logger().info('CAN bridge running')

    # ══════════════════════════════════════════════
    # Interface setup
    # ══════════════════════════════════════════════

    def _setup_pr_interface(self):
        self.pr_pitch_mm = 0.0
        self.pr_roll_deg = 0.0
        self.pr_enable = False
        self.pr_home = False

        self.create_subscription(Float64, '/cmd/pitch_mm', self._cb_pr_pitch, 10)
        self.create_subscription(Float64, '/cmd/roll_deg', self._cb_pr_roll, 10)
        self.create_subscription(Bool, '/cmd/pr_enable', self._cb_pr_enable, 10)
        self.create_subscription(Bool, '/cmd/pr_home', self._cb_pr_home, 10)

        self.pub_pr_pitch_pos = self.create_publisher(Float64, '/feedback/pr/pitch_pos_mm', 10)
        self.pub_pr_roll_pos = self.create_publisher(Float64, '/feedback/pr/roll_pos_deg', 10)
        self.pub_pr_tof = self.create_publisher(Float64, '/feedback/pr/tof_mm', 10)
        self.pub_pr_leak = self.create_publisher(Bool, '/feedback/pr/leak', 10)
        self.pub_pr_pitch_homed = self.create_publisher(Bool, '/feedback/pr/pitch_homed', 10)
        self.pub_pr_roll_homed = self.create_publisher(Bool, '/feedback/pr/roll_homed', 10)
        self.pub_pr_status = self.create_publisher(String, '/feedback/pr/status_flags', 10)
        self.pub_pr_seq = self.create_publisher(UInt8, '/feedback/pr/seq', 10)
        self.pub_pr_fault = self.create_publisher(String, '/feedback/pr/fault', 10)
        self.pub_pr_bms_v = self.create_publisher(Float64, '/feedback/pr/bms_voltage_v', 10)
        self.pub_pr_bms_t = self.create_publisher(Float64, '/feedback/pr/bms_temp_c', 10)
        self.pub_pr_bms_flag = self.create_publisher(UInt8, '/feedback/pr/bms_flag', 10)

    def _setup_vbd_interface(self, side, node_id):
        setattr(self, f'vbd_{side}_pct', 0)
        setattr(self, f'vbd_{side}_enable', False)
        setattr(self, f'vbd_{side}_home', False)

        self.create_subscription(
            UInt8, f'/cmd/vbd_{side}_pct',
            lambda msg, s=side: setattr(self, f'vbd_{s}_pct', msg.data), 10)
        self.create_subscription(
            Bool, f'/cmd/vbd_{side}_enable',
            lambda msg, s=side: setattr(self, f'vbd_{s}_enable', msg.data), 10)
        self.create_subscription(
            Bool, f'/cmd/vbd_{side}_home',
            lambda msg, s=side: setattr(self, f'vbd_{s}_home', msg.data), 10)

        for name, typ in [('pos_mm', Float64), ('tof_mm', Float64), ('leak', Bool),
                          ('homed', Bool), ('seq', UInt8), ('fault', String),
                          ('bms_voltage_v', Float64), ('bms_temp_c', Float64),
                          ('bms_flag', UInt8)]:
            setattr(self, f'pub_vbd_{side}_{name}',
                self.create_publisher(typ, f'/feedback/vbd_{side}/{name}', 10))

    # ══════════════════════════════════════════════
    # Command callbacks
    # ══════════════════════════════════════════════

    def _cb_pr_pitch(self, msg):
        self.pr_pitch_mm = max(-56.9, min(56.9, msg.data))

    def _cb_pr_roll(self, msg):
        self.pr_roll_deg = max(-90.0, min(90.0, msg.data))

    def _cb_pr_enable(self, msg):
        self.pr_enable = msg.data

    def _cb_pr_home(self, msg):
        self.pr_home = msg.data

    # ══════════════════════════════════════════════
    # Outgoing: ROS topics -> CAN frames
    # ══════════════════════════════════════════════

    def _republish_commands(self):
        self._send_pr_cmd()
        self._send_vbd_cmd(VBD_L, 'left')
        self._send_vbd_cmd(VBD_R, 'right')

    def _send_pr_cmd(self):
        # P&R CMD_SETPOINT (0x103):
        #   Byte 0-1: pitch_setpoint (int16, 0.1mm)
        #   Byte 2-3: roll_setpoint (int16, 0.1deg)
        #   Byte 4:   command mode (bit0=ENABLE, bit1=HOME)
        #   Byte 5:   reserve
        #   Byte 6:   sequence counter
        #   Byte 7:   reserve
        pitch_raw = int(round(self.pr_pitch_mm * 10))
        roll_raw = int(round(self.pr_roll_deg * 10))
        cmd_bits = (0x01 if self.pr_enable else 0) | (0x02 if self.pr_home else 0)
        self.seq_pr = (self.seq_pr + 1) & 0xFF

        data = struct.pack('>hhBxBx', pitch_raw, roll_raw, cmd_bits, self.seq_pr)
        self._tx(CMD_BASE + PR, data)

    def _send_vbd_cmd(self, node_id, side):
        # VBD CMD_SETPOINT (0x101 or 0x102):
        #   Byte 0:   stroke percentage (uint8, 0-100)
        #   Byte 1:   reserve
        #   Byte 2:   reserve
        #   Byte 3:   command mode (bit0=ENABLE, bit1=HOMING)
        #   Byte 4:   reserve
        #   Byte 5:   reserve
        #   Byte 6:   sequence counter
        #   Byte 7:   reserve
        pct = max(0, min(100, getattr(self, f'vbd_{side}_pct')))
        enable = getattr(self, f'vbd_{side}_enable')
        home = getattr(self, f'vbd_{side}_home')
        cmd_bits = (0x01 if enable else 0) | (0x02 if home else 0)

        attr = f'seq_vbd_{"l" if node_id == VBD_L else "r"}'
        seq = (getattr(self, attr) + 1) & 0xFF
        setattr(self, attr, seq)

        data = bytes([pct, 0x00, 0x00, cmd_bits, 0x00, 0x00, seq, 0x00])
        self._tx(CMD_BASE + node_id, data)

    def _tx(self, can_id, data):
        msg = self.can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            self.get_logger().warn(f'CAN TX 0x{can_id:03X} failed: {e}')

    # ══════════════════════════════════════════════
    # Incoming: CAN frames -> ROS topics
    # ══════════════════════════════════════════════

    def _can_rx_loop(self):
        while self._running:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            cid = msg.arbitration_id
            d = msg.data

            for base, handler in [(STATUS_BASE, self._parse_status),
                                  (FAULT_BASE, self._parse_fault),
                                  (BMS_BASE, self._parse_bms)]:
                node_id = cid - base
                if node_id in (VBD_L, VBD_R, PR):
                    handler(node_id, d)
                    break

    # ── STATUS_CONTROL parsing ──

    def _parse_status(self, node_id, d):
        if len(d) < 8:
            return

        if node_id == PR:
            self._parse_pr_status(d)
        elif node_id in (VBD_L, VBD_R):
            self._parse_vbd_status(node_id, d)

    def _parse_pr_status(self, d):
        # P&R STATUS_CONTROL (0x203):
        #   Byte 0-1: pitch_pos (int16, 0.1mm)
        #   Byte 2-3: roll_pos (int16, 0.1deg)
        #   Byte 4:   flags (pitch_dir, roll_dir, pitch_homed, roll_homed,
        #              pitch_moving, roll_moving, pitch_en, roll_en)
        #   Byte 5:   flags (front_lim, rear_lim, leak, hall, -, -, -, -)
        #   Byte 6:   sequence counter
        #   Byte 7:   tof (int8, 1mm)
        pitch_raw, roll_raw = struct.unpack_from('>hh', d, 0)
        b4, b5, seq = d[4], d[5], d[6]
        tof = struct.unpack_from('>b', d, 7)[0]

        self._pub_f64(self.pub_pr_pitch_pos, pitch_raw / 10.0)
        self._pub_f64(self.pub_pr_roll_pos, roll_raw / 10.0)
        self._pub_f64(self.pub_pr_tof, float(tof))

        # P&R: leak is byte 5, bit 2
        self._pub_bool(self.pub_pr_leak, bool(b5 & 0x04))
        # P&R: pitch_homed is byte 4 bit 2, roll_homed is byte 4 bit 3
        pitch_homed = bool(b4 & 0x04)
        roll_homed = bool(b4 & 0x08)
        self._pub_bool(self.pub_pr_pitch_homed, pitch_homed)
        self._pub_bool(self.pub_pr_roll_homed, roll_homed)
        self._pub_u8(self.pub_pr_seq, seq)

        with self._home_lock:
            self._pr_pitch_homed_state = pitch_homed
            self._pr_roll_homed_state = roll_homed

        flags = []
        pr_flag_names_b4 = ['pitch_dir','roll_dir','pitch_homed','roll_homed',
                            'pitch_moving','roll_moving','pitch_en','roll_en']
        pr_flag_names_b5 = ['front_lim','rear_lim','leak','hall']
        for i, n in enumerate(pr_flag_names_b4):
            if b4 & (1 << i): flags.append(n)
        for i, n in enumerate(pr_flag_names_b5):
            if b5 & (1 << i): flags.append(n)
        s = String(); s.data = ','.join(flags) if flags else 'none'
        self.pub_pr_status.publish(s)

    def _parse_vbd_status(self, node_id, d):
        # VBD STATUS_CONTROL (0x201 or 0x202):
        #   Byte 0-1: pos_mm (uint16, LITTLE-ENDIAN)
        #   Byte 2-3: tof_mm (uint16, LITTLE-ENDIAN)
        #   Byte 4:   flags (prox, leak, driver_flt, homed, moving, motor_enabled, dir, -)
        #   Byte 5:   pwm (uint8)
        #   Byte 6:   sequence counter
        #   Byte 7:   current sense (uint8)
        side = 'left' if node_id == VBD_L else 'right'

        pos_raw = struct.unpack_from('<H', d, 0)[0]
        tof_raw = struct.unpack_from('<H', d, 2)[0]
        b4 = d[4]
        seq = d[6]

        # VBD pos is 0-1500 representing 0-150.0mm
        self._pub_f64(getattr(self, f'pub_vbd_{side}_pos_mm'), pos_raw / 10.0)
        self._pub_f64(getattr(self, f'pub_vbd_{side}_tof_mm'), tof_raw / 10.0)

        # VBD: leak is byte 4, bit 1
        leak = bool(b4 & 0x02)
        self._pub_bool(getattr(self, f'pub_vbd_{side}_leak'), leak)

        # VBD: homed is byte 4, bit 3
        homed = bool(b4 & 0x08)
        self._pub_bool(getattr(self, f'pub_vbd_{side}_homed'), homed)
        self._pub_u8(getattr(self, f'pub_vbd_{side}_seq'), seq)

        with self._home_lock:
            if node_id == VBD_L:
                self._vbd_left_homed_state = homed
            else:
                self._vbd_right_homed_state = homed

    # ── STATUS_FAULT parsing ──

    def _parse_fault(self, node_id, d):
        if len(d) < 8:
            return

        if node_id == PR:
            self._parse_pr_fault(d)
        elif node_id in (VBD_L, VBD_R):
            self._parse_vbd_fault(node_id, d)

    def _parse_pr_fault(self, d):
        # P&R FAULT (0x053):
        #   Byte 0: LEAK, DRIVER_PITCH, DRIVER_ROLL, CMD_TO_L,
        #           PITCH_N_HOME, ROLL_N_HOME, BMS_TEMP, BMS_TO_L
        #   Byte 1: BMS_OC, BMS_SWITCH, STALL_PITCH, STALL_ROLL
        #   Byte 2: CMD_TO, TOF_INV, BMS_TO, BMS_LOW, BMS_HIGH
        pr_hard_b0 = ['LEAK','DRIVER_PITCH','DRIVER_ROLL','CMD_TO_L',
                      'PITCH_N_HOME','ROLL_N_HOME','BMS_TEMP','BMS_TO_L']
        pr_hard_b1 = ['BMS_OC','BMS_SWITCH','STALL_PITCH','STALL_ROLL']
        pr_soft_b2 = ['CMD_TO','TOF_INV','BMS_TO','BMS_LOW','BMS_HIGH']

        hards = [n for i, n in enumerate(pr_hard_b0) if d[0] & (1 << i)]
        hards += [n for i, n in enumerate(pr_hard_b1) if d[1] & (1 << i)]
        softs = [n for i, n in enumerate(pr_soft_b2) if d[2] & (1 << i)]

        state = d[5]
        state_names = ['UNHOMED','HOMING','RUN','HOLD','FAULT']
        state_str = state_names[state] if state < len(state_names) else f'UNK({state})'

        with self._home_lock:
            if 'PITCH_N_HOME' in hards:
                self._pr_pitch_n_home_fault = True
            if 'ROLL_N_HOME' in hards:
                self._pr_roll_n_home_fault = True

        self._publish_fault_msg(self.pub_pr_fault, hards, softs, state_str, PR)

    def _parse_vbd_fault(self, node_id, d):
        # VBD FAULT (0x051 or 0x052):
        #   Byte 0: LEAK, DRIVER_FLT, OVERCURRENT, STALL,
        #           CMD_TO_L, NOT_HOMED, POS_LIM, BMS_TEMP
        #   Byte 1: BMS_TO_L, BMS_OC, BMS_SWITCH
        #   Byte 2: CMD_TO, TOF_INV, TOF_OOR, SLIP,
        #           ENCODER_INV, BMS_TO, BMS_LOW, BMS_HIGH
        vbd_hard_b0 = ['LEAK','DRIVER_FLT','OVERCURRENT','STALL',
                       'CMD_TO_L','NOT_HOMED','POS_LIM','BMS_TEMP']
        vbd_hard_b1 = ['BMS_TO_L','BMS_OC','BMS_SWITCH']
        vbd_soft_b2 = ['CMD_TO','TOF_INV','TOF_OOR','SLIP',
                       'ENCODER_INV','BMS_TO','BMS_LOW','BMS_HIGH']

        hards = [n for i, n in enumerate(vbd_hard_b0) if d[0] & (1 << i)]
        hards += [n for i, n in enumerate(vbd_hard_b1) if d[1] & (1 << i)]
        softs = [n for i, n in enumerate(vbd_soft_b2) if d[2] & (1 << i)]

        state = d[5]
        state_names = ['UNHOMED','HOMING','RUN','HOLD','FAULT']
        state_str = state_names[state] if state < len(state_names) else f'UNK({state})'

        with self._home_lock:
            if 'NOT_HOMED' in hards:
                if node_id == VBD_L:
                    self._vbd_left_not_homed_fault = True
                else:
                    self._vbd_right_not_homed_fault = True

        side = 'left' if node_id == VBD_L else 'right'
        pub = getattr(self, f'pub_vbd_{side}_fault')
        self._publish_fault_msg(pub, hards, softs, state_str, node_id)

    def _publish_fault_msg(self, pub, hards, softs, state_str, node_id):
        if hards:
            s = String(); s.data = f'HARD:{",".join(hards)}|state:{state_str}'
            pub.publish(s)
            self.get_logger().error(f'Node {node_id} HARD: {",".join(hards)}')
        elif softs:
            s = String(); s.data = f'SOFT:{",".join(softs)}|state:{state_str}'
            pub.publish(s)
            self.get_logger().warn(f'Node {node_id} soft: {",".join(softs)}')
        else:
            s = String(); s.data = f'OK|state:{state_str}'
            pub.publish(s)

    # ── STATUS_BMS parsing (same format for both P&R and VBD) ──

    def _parse_bms(self, node_id, d):
        # BMS (0x211, 0x212, 0x213):
        #   Byte 0-1: pack_mV (uint16)
        #   Byte 2-3: pack_temp (int16)
        #   Byte 4:   bms flag (uint8)
        #   Byte 6:   sequence counter
        if len(d) < 7:
            return

        pack_mv, temp_raw = struct.unpack_from('>Hh', d, 0)
        bms_flag = d[4]
        voltage_v = pack_mv / 1000.0
        temp_c = temp_raw / 10.0

        if node_id == PR:
            pv, pt, pf = self.pub_pr_bms_v, self.pub_pr_bms_t, self.pub_pr_bms_flag
        else:
            side = 'left' if node_id == VBD_L else 'right'
            pv = getattr(self, f'pub_vbd_{side}_bms_voltage_v')
            pt = getattr(self, f'pub_vbd_{side}_bms_temp_c')
            pf = getattr(self, f'pub_vbd_{side}_bms_flag')

        self._pub_f64(pv, voltage_v)
        self._pub_f64(pt, temp_c)
        self._pub_u8(pf, bms_flag)

        if bms_flag:
            self.get_logger().warn(f'Node {node_id} BMS flag 0x{bms_flag:02X} {voltage_v:.2f}V {temp_c:.1f}C')

    # ══════════════════════════════════════════════
    # HomeActuators action server
    # ══════════════════════════════════════════════

    def _home_execute(self, goal_handle):
        timeout_s = float(goal_handle.request.timeout_s)
        if timeout_s <= 0.0:
            timeout_s = 65.0

        self.get_logger().info(f'HomeActuators: starting (timeout={timeout_s:.0f}s)')

        # Reset homing fault latches from any prior run
        with self._home_lock:
            self._vbd_left_not_homed_fault = False
            self._vbd_right_not_homed_fault = False
            self._pr_pitch_n_home_fault = False
            self._pr_roll_n_home_fault = False

        # Command all devices to enable and home — picked up by _republish_commands at 20 Hz
        self.vbd_left_enable = True
        self.vbd_left_home = True
        self.vbd_right_enable = True
        self.vbd_right_home = True
        self.pr_enable = True
        self.pr_home = True

        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._stop_homing()
                goal_handle.canceled()
                result = HomeActuators.Result()
                result.success = False
                result.status_message = 'Cancelled'
                return result

            with self._home_lock:
                vl_h = self._vbd_left_homed_state
                vr_h = self._vbd_right_homed_state
                pp_h = self._pr_pitch_homed_state
                pr_h = self._pr_roll_homed_state
                vl_f = self._vbd_left_not_homed_fault
                vr_f = self._vbd_right_not_homed_fault
                pp_f = self._pr_pitch_n_home_fault
                pr_f = self._pr_roll_n_home_fault

            feedback = HomeActuators.Feedback()
            feedback.vbd_left_homed = vl_h
            feedback.vbd_right_homed = vr_h
            feedback.pitch_homed = pp_h
            feedback.roll_homed = pr_h
            goal_handle.publish_feedback(feedback)

            # Teensy fault latches mean homing failed on that device
            failed = []
            if vl_f:
                failed.append('vbd_left (NOT_HOMED fault)')
            if vr_f:
                failed.append('vbd_right (NOT_HOMED fault)')
            if pp_f:
                failed.append('pitch (PITCH_N_HOME fault)')
            if pr_f:
                failed.append('roll (ROLL_N_HOME fault)')

            if failed:
                self._stop_homing()
                goal_handle.abort()
                result = HomeActuators.Result()
                result.success = False
                result.status_message = f'Homing fault on: {", ".join(failed)}'
                self.get_logger().error(f'HomeActuators: {result.status_message}')
                return result

            if vl_h and vr_h and pp_h and pr_h:
                self._stop_homing()
                goal_handle.succeed()
                result = HomeActuators.Result()
                result.success = True
                result.status_message = 'All actuators homed successfully'
                self.get_logger().info('HomeActuators: complete')
                return result

            time.sleep(0.2)

        # Deadline exceeded before Teensy fault — our own timeout
        self._stop_homing()
        goal_handle.abort()
        result = HomeActuators.Result()
        result.success = False
        result.status_message = f'Homing timed out after {timeout_s:.0f}s'
        self.get_logger().error(f'HomeActuators: {result.status_message}')
        return result

    def _stop_homing(self):
        """Clear home bits; enable stays on so actuators hold position."""
        self.vbd_left_home = False
        self.vbd_right_home = False
        self.pr_home = False

    # ══════════════════════════════════════════════
    # Publish helpers
    # ══════════════════════════════════════════════

    def _pub_f64(self, pub, val):
        m = Float64(); m.data = val; pub.publish(m)

    def _pub_bool(self, pub, val):
        m = Bool(); m.data = val; pub.publish(m)

    def _pub_u8(self, pub, val):
        m = UInt8(); m.data = val; pub.publish(m)

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

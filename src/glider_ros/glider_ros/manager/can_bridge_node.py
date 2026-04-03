#!/usr/bin/env python3


import struct
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, UInt8, String


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

        self._running = True
        self._rx_thread = threading.Thread(target=self._can_rx_loop, daemon=True)
        self._rx_thread.start()

        rate = self.get_parameter('republish_rate_hz').value
        self.create_timer(1.0 / rate, self._republish_commands)

        self.get_logger().info('CAN bridge running')

    # ── Interface setup ──

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
        setattr(self, f'vbd_{side}_mm', 0.0)
        setattr(self, f'vbd_{side}_enable', False)
        setattr(self, f'vbd_{side}_home', False)

        self.create_subscription(
            Float64, f'/cmd/vbd_{side}_mm',
            lambda msg, s=side: setattr(self, f'vbd_{s}_mm', msg.data), 10)
        self.create_subscription(
            Bool, f'/cmd/vbd_{side}_enable',
            lambda msg, s=side: setattr(self, f'vbd_{s}_enable', msg.data), 10)
        self.create_subscription(
            Bool, f'/cmd/vbd_{side}_home',
            lambda msg, s=side: setattr(self, f'vbd_{s}_home', msg.data), 10)

        for name, typ in [('pos_mm', Float64), ('leak', Bool), ('seq', UInt8),
                          ('fault', String), ('bms_voltage_v', Float64),
                          ('bms_temp_c', Float64), ('bms_flag', UInt8)]:
            setattr(self, f'pub_vbd_{side}_{name}',
                self.create_publisher(typ, f'/feedback/vbd_{side}/{name}', 10))

    # ── Command callbacks ──

    def _cb_pr_pitch(self, msg):
        self.pr_pitch_mm = max(-56.9, min(56.9, msg.data))

    def _cb_pr_roll(self, msg):
        self.pr_roll_deg = max(-90.0, min(90.0, msg.data))

    def _cb_pr_enable(self, msg):
        self.pr_enable = msg.data

    def _cb_pr_home(self, msg):
        self.pr_home = msg.data

    # ── Outgoing: ROS topics -> CAN frames ──

    def _republish_commands(self):
        self._send_pr_cmd()
        self._send_vbd_cmd(VBD_L, 'left')
        self._send_vbd_cmd(VBD_R, 'right')

    def _send_pr_cmd(self):
        pitch_raw = int(round(self.pr_pitch_mm * 10))
        roll_raw = int(round(self.pr_roll_deg * 10))
        cmd_bits = (0x01 if self.pr_enable else 0) | (0x02 if self.pr_home else 0)
        self.seq_pr = (self.seq_pr + 1) & 0xFF

        data = struct.pack('>hhBxBx', pitch_raw, roll_raw, cmd_bits, self.seq_pr)
        self._tx(CMD_BASE + PR, data)

    def _send_vbd_cmd(self, node_id, side):
        pos_mm = getattr(self, f'vbd_{side}_mm')
        enable = getattr(self, f'vbd_{side}_enable')
        home = getattr(self, f'vbd_{side}_home')
        pos_raw = int(round(pos_mm * 10))
        cmd_bits = (0x01 if enable else 0) | (0x02 if home else 0)

        attr = f'seq_vbd_{"l" if node_id == VBD_L else "r"}'
        seq = (getattr(self, attr) + 1) & 0xFF
        setattr(self, attr, seq)

        data = struct.pack('>hxxBxBx', pos_raw, cmd_bits, seq)
        self._tx(CMD_BASE + node_id, data)

    def _tx(self, can_id, data):
        msg = self.can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            self.get_logger().warn(f'CAN TX 0x{can_id:03X} failed: {e}')

    # ── Incoming: CAN frames -> ROS topics ──

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

    def _parse_status(self, node_id, d):
        if len(d) < 8:
            return

        if node_id == PR:
            pitch_raw, roll_raw = struct.unpack_from('>hh', d, 0)
            b4, b5, seq = d[4], d[5], d[6]
            tof = struct.unpack_from('>b', d, 7)[0]

            self._pub_f64(self.pub_pr_pitch_pos, pitch_raw / 10.0)
            self._pub_f64(self.pub_pr_roll_pos, roll_raw / 10.0)
            self._pub_f64(self.pub_pr_tof, float(tof))
            self._pub_bool(self.pub_pr_leak, bool(b5 & 0x04))
            self._pub_bool(self.pub_pr_pitch_homed, bool(b4 & 0x04))
            self._pub_bool(self.pub_pr_roll_homed, bool(b4 & 0x08))
            self._pub_u8(self.pub_pr_seq, seq)

            flags = self._decode_pr_flags(b4, b5)
            s = String(); s.data = ','.join(flags) if flags else 'none'
            self.pub_pr_status.publish(s)

        elif node_id in (VBD_L, VBD_R):
            side = 'left' if node_id == VBD_L else 'right'
            pos_raw = struct.unpack_from('>h', d, 0)[0]
            b5, seq = d[5], d[6]
            leak = bool(b5 & 0x04)

            self._pub_f64(getattr(self, f'pub_vbd_{side}_pos_mm'), pos_raw / 10.0)
            self._pub_bool(getattr(self, f'pub_vbd_{side}_leak'), leak)
            self._pub_u8(getattr(self, f'pub_vbd_{side}_seq'), seq)

    def _parse_fault(self, node_id, d):
        if len(d) < 8:
            return

        hard_names_b0 = ['LEAK','DRIVER_PITCH','DRIVER_ROLL','CMD_TO_L',
                         'PITCH_N_HOME','ROLL_N_HOME','BMS_TEMP','BMS_TO_L']
        hard_names_b1 = ['BMS_OC','BMS_SWITCH','STALL_PITCH','STALL_ROLL']
        soft_names_b2 = ['CMD_TO','TOF_INV','BMS_TO','BMS_LOW','BMS_HIGH']

        hards = [n for i, n in enumerate(hard_names_b0) if d[0] & (1 << i)]
        hards += [n for i, n in enumerate(hard_names_b1) if d[1] & (1 << i)]
        softs = [n for i, n in enumerate(soft_names_b2) if d[2] & (1 << i)]
        state = d[5]
        state_names = ['UNHOMED','HOMING','RUN','HOLD','FAULT']
        state_str = state_names[state] if state < len(state_names) else f'UNK({state})'

        if node_id == PR:
            pub = self.pub_pr_fault
        else:
            side = 'left' if node_id == VBD_L else 'right'
            pub = getattr(self, f'pub_vbd_{side}_fault')

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

    def _parse_bms(self, node_id, d):
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

    def _decode_pr_flags(self, b4, b5):
        flags = []
        names_b4 = ['pitch_dir','roll_dir','pitch_homed','roll_homed',
                     'pitch_moving','roll_moving','pitch_en','roll_en']
        names_b5 = ['front_lim','rear_lim','leak','hall']
        for i, n in enumerate(names_b4):
            if b4 & (1 << i): flags.append(n)
        for i, n in enumerate(names_b5):
            if b5 & (1 << i): flags.append(n)
        return flags

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

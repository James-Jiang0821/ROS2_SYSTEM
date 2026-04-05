#!/usr/bin/env python3
"""
glider_controller.py

Pure control logic node. Does one thing: run Ibrahim's nested PI
controller when told to operate, stop when told to stop.

Subscribes to /mission/command:
  "OPERATION" → start running PI loops
  Anything else → stop, publish nothing

Inside OPERATION, Ibrahim's mission supervisor handles dive/climb
switching internally based on depth. When max_dives reached,
publishes "COMPLETE" on /controller/phase and stops.

All homing, safety, emergency handling is done by other nodes.
This node only does control maths.

Gains from Ibrahim's glider_params.m:
  Outer PI: Kp=1.5, Ki=0.002 (Ti=750s)
  Inner PI: Kp=-0.02, Ki=-0.004 (Ti=5s)
  Roll P: K_phi=0.15
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String, UInt8
from sensor_msgs.msg import Imu


class PIController:
    def __init__(self, kp, ki, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def compute(self, error, dt):
        p_term = self.kp * error
        proposed = p_term + self.ki * (self.integral + error * dt)
        output = max(self.out_min, min(self.out_max, proposed))
        if (proposed > self.out_max and error > 0) or \
           (proposed < self.out_min and error < 0):
            pass
        else:
            self.integral += error * dt
        return output


class FirstOrderFilter:
    def __init__(self, tau, initial=0.0):
        self.tau = max(tau, 1e-6)
        self.y = initial

    def update(self, u, dt):
        if dt <= 0:
            return self.y
        alpha = dt / (self.tau + dt)
        self.y += alpha * (u - self.y)
        return self.y

    def reset(self, value=0.0):
        self.y = value


class GliderController(Node):

    def __init__(self):
        super().__init__('glider_controller')

        self.declare_parameter('Kp_theta', 1.5)
        self.declare_parameter('Ti_theta', 750.0)
        self.declare_parameter('Kp_q', -0.02)
        self.declare_parameter('Ti_q', 5.0)
        self.declare_parameter('K_phi', 0.15)
        self.declare_parameter('q_cmd_max', math.radians(8))
        self.declare_parameter('shift_max_m', 0.0569)
        self.declare_parameter('shift_min_m', -0.0569)
        self.declare_parameter('roll_max_rad', math.pi / 2)
        self.declare_parameter('depth_upper', 1.0)
        self.declare_parameter('depth_lower', 5.0)
        self.declare_parameter('alpha_dive', math.radians(5))
        self.declare_parameter('alpha_rise', math.radians(-5))
        self.declare_parameter('T_alpha_cmd', 2.0)
        self.declare_parameter('T_shift_trim', 5.0)
        self.declare_parameter('T_vbd_cmd', 5.0)
        self.declare_parameter('shift_cmd_tau', 0.01)
        self.declare_parameter('roll_cmd_tau', 0.01)
        self.declare_parameter('max_dives', 1)
        self.declare_parameter('control_rate_hz', 10.0)

        self._load_params()

        self.pi_theta = PIController(
            self.Kp_theta, self.Ki_theta,
            -self.q_cmd_max, self.q_cmd_max)
        self.pi_q = PIController(
            self.Kp_q, self.Ki_q,
            self.shift_min_m, self.shift_max_m)

        self.filt_alpha = FirstOrderFilter(self.T_alpha_cmd, self.alpha_dive)
        self.filt_shift_trim = FirstOrderFilter(self.T_shift_trim, 0.0)
        self.filt_vbd = FirstOrderFilter(self.T_vbd_cmd, 0.0)
        self.filt_shift_cmd = FirstOrderFilter(self.shift_cmd_tau, 0.0)
        self.filt_roll_cmd = FirstOrderFilter(self.roll_cmd_tau, 0.0)

        self.operating = False
        self.diving = True
        self.dive_count = 0

        self.theta = 0.0
        self.q = 0.0
        self.phi = 0.0
        self.depth = 0.0

        self.alpha_ref_raw = self.alpha_dive
        self.shift_trim_raw = 0.0
        self.vbd_raw_pct = 0

        # ── Subscribers ──
        self.create_subscription(Imu, '/imu/data', self._cb_imu, 10)
        self.create_subscription(Float64, '/pressure/depth', self._cb_depth, 10)
        self.create_subscription(String, '/mission/command', self._cb_command, 10)

        # ── Publishers ──
        self.pub_pitch_mm = self.create_publisher(Float64, '/cmd/pitch_mm', 10)
        self.pub_roll_deg = self.create_publisher(Float64, '/cmd/roll_deg', 10)
        self.pub_vbd_left = self.create_publisher(UInt8, '/cmd/vbd_left_pct', 10)
        self.pub_vbd_right = self.create_publisher(UInt8, '/cmd/vbd_right_pct', 10)
        self.pub_phase = self.create_publisher(String, '/controller/phase', 10)

        # ── Timer ──
        rate = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Controller ready: outer PI Kp={self.Kp_theta} Ki={self.Ki_theta:.4f}, '
            f'inner PI Kp={self.Kp_q} Ki={self.Ki_q:.4f}, K_phi={self.K_phi}')

    def _load_params(self):
        p = self.get_parameter
        self.Kp_theta = p('Kp_theta').value
        self.Ki_theta = self.Kp_theta / p('Ti_theta').value
        self.Kp_q = p('Kp_q').value
        self.Ki_q = self.Kp_q / p('Ti_q').value
        self.K_phi = p('K_phi').value
        self.q_cmd_max = p('q_cmd_max').value
        self.shift_max_m = p('shift_max_m').value
        self.shift_min_m = p('shift_min_m').value
        self.roll_max_rad = p('roll_max_rad').value
        self.depth_upper = p('depth_upper').value
        self.depth_lower = p('depth_lower').value
        self.alpha_dive = p('alpha_dive').value
        self.alpha_rise = p('alpha_rise').value
        self.T_alpha_cmd = p('T_alpha_cmd').value
        self.T_shift_trim = p('T_shift_trim').value
        self.T_vbd_cmd = p('T_vbd_cmd').value
        self.shift_cmd_tau = p('shift_cmd_tau').value
        self.roll_cmd_tau = p('roll_cmd_tau').value
        self.max_dives = p('max_dives').value

    # ── Callbacks ──

    def _cb_imu(self, msg):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.phi = math.atan2(sinr, cosr)
        sinp = 2.0 * (qw * qy - qz * qx)
        self.theta = math.asin(max(-1.0, min(1.0, sinp)))
        self.q = msg.angular_velocity.y

    def _cb_depth(self, msg):
        self.depth = msg.data

    def _cb_command(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == 'OPERATION' and not self.operating:
            self._start_operation()
        elif cmd != 'OPERATION' and self.operating:
            self._stop_operation()

    # ── Start / stop ──

    def _start_operation(self):
        self.operating = True
        self.diving = True
        self.dive_count += 1
        self.pi_theta.reset()
        self.pi_q.reset()
        self.filt_alpha.reset(self.alpha_dive)
        self.filt_shift_trim.reset(0.0)
        self.filt_vbd.reset(0.0)
        self.filt_shift_cmd.reset(0.0)
        self.filt_roll_cmd.reset(0.0)
        self.alpha_ref_raw = self.alpha_dive
        self.vbd_raw_pct = 0
        self.get_logger().info(f'OPERATION started (dive #{self.dive_count})')

    def _stop_operation(self):
        self.operating = False
        self.get_logger().info('OPERATION stopped')

    # ── Mission supervisor (Ibrahim's logic) ──

    def _mission_supervisor(self):
        if self.diving and self.depth >= self.depth_lower:
            self.diving = False
            self.pi_theta.reset()
            self.pi_q.reset()
            self.get_logger().info('Depth reached — switching to climb')
        elif not self.diving and self.depth <= self.depth_upper:
            if self.dive_count >= self.max_dives:
                self.operating = False
                self.get_logger().info('Dive complete')
                return False

        if self.diving:
            self.alpha_ref_raw = self.alpha_dive
            self.shift_trim_raw = 0.0
            self.vbd_raw_pct = 0
        else:
            self.alpha_ref_raw = self.alpha_rise
            self.shift_trim_raw = 0.0
            self.vbd_raw_pct = 100
        return True

    # ── Main 10 Hz loop ──

    def _control_loop(self):
        # Publish current state
        s = String()
        s.data = 'OPERATION' if self.operating else 'COMPLETE' if self.dive_count >= self.max_dives else 'IDLE'
        self.pub_phase.publish(s)

        # Not operating — do nothing
        if not self.operating:
            return

        # Mission supervisor: check depth, flip dive/climb
        if not self._mission_supervisor():
            return

        # Smoothing filters
        alpha_filt = self.filt_alpha.update(self.alpha_ref_raw, self.dt)
        shift_trim_filt = self.filt_shift_trim.update(self.shift_trim_raw, self.dt)
        vbd_filt_pct = self.filt_vbd.update(self.vbd_raw_pct, self.dt)

        # Outer PI: angle error → desired pitch rate
        alpha_err = alpha_filt - self.theta
        q_cmd = self.pi_theta.compute(alpha_err, self.dt)

        # Inner PI: rate error → battery shift
        q_err = q_cmd - self.q
        shift_pi_out = self.pi_q.compute(q_err, self.dt)

        # Shift total + smoothing + saturation
        shift_total = shift_pi_out + shift_trim_filt
        shift_smoothed = self.filt_shift_cmd.update(shift_total, self.dt)
        shift_m = max(self.shift_min_m, min(self.shift_max_m, shift_smoothed))

        # Roll P: keep level
        phi_err = 0.0 - self.phi
        roll_p_out = self.K_phi * phi_err
        roll_smoothed = self.filt_roll_cmd.update(roll_p_out, self.dt)
        roll_rad = max(-self.roll_max_rad, min(self.roll_max_rad, roll_smoothed))

        # Convert to CAN protocol units
        shift_mm = shift_m * 1000.0
        roll_deg = math.degrees(roll_rad)
        vbd_pct = max(0, min(100, int(round(vbd_filt_pct))))

        # Publish
        m = Float64(); m.data = shift_mm; self.pub_pitch_mm.publish(m)
        m = Float64(); m.data = roll_deg; self.pub_roll_deg.publish(m)
        m = UInt8(); m.data = vbd_pct; self.pub_vbd_left.publish(m)
        m = UInt8(); m.data = vbd_pct; self.pub_vbd_right.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GliderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
controller_node.py

Glider nested PI controller as a lifecycle node. Activated only during
OPERATION state by the state manager.

Lifecycle:
  configure  → create subscribers, publishers, PI objects
  activate   → reload params (picks up mission depth), reset state, start timer
  deactivate → stop timer
  cleanup    → destroy subscribers, publishers

On activate the node immediately begins diving. When depth_lower is
reached it climbs back to depth_upper, then publishes COMPLETE on
/controller/phase and stops. The state manager sees COMPLETE and
deactivates the node.

Gains from Ibrahim's glider_params.m:
  Outer PI: Kp=1.5, Ki=0.002 (Ti=750s)
  Inner PI: Kp=0.02, Ki=0.004 (Ti=5s)
  Roll outer PI: Kp_phi=0.3, Ki from Ti_phi=15s
  Roll inner PI: Kp_p=4.0, Ki from Ti_p=20000s
"""

import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Bool, Float64, String, UInt8


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
        if not ((proposed > self.out_max and error > 0) or
                (proposed < self.out_min and error < 0)):
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


class GliderController(LifecycleNode):

    def __init__(self):
        super().__init__('glider_controller')

        # Declare all parameters here so they can be set externally at any time
        self.declare_parameter('Kp_theta', 1.5)
        self.declare_parameter('Ti_theta', 750.0)
        self.declare_parameter('Kp_q', 0.02)
        self.declare_parameter('Ti_q', 5.0)
        self.declare_parameter('Kp_phi', 0.3)
        self.declare_parameter('Ti_phi', 15.0)
        self.declare_parameter('Kp_p', 4.0)
        self.declare_parameter('Ti_p', 20000.0)
        self.declare_parameter('q_cmd_max', math.radians(8))
        self.declare_parameter('p_cmd_max', math.radians(5))
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
        self.declare_parameter('roll_cmd_tau', 5.0)
        self.declare_parameter('max_dives', 1)
        self.declare_parameter('control_rate_hz', 10.0)

        # Sensor state — updated by subscribers once configured
        self.theta = 0.0
        self.q = 0.0
        self.phi = 0.0
        self.p = 0.0
        self.depth = 0.0

        # Control state — reset on activate
        self.operating = False
        self.diving = True
        self.dive_count = 0
        self.alpha_ref_raw = 0.0
        self.shift_trim_raw = 0.0
        self.vbd_raw_pct = 0

        # Created in on_configure
        self._sub_force_surface = None
        self._sub_roll = None
        self._sub_roll_rate = None
        self._sub_pitch = None
        self._sub_pitch_rate = None
        self.pi_theta = None
        self.pi_q = None
        self.pi_phi = None
        self.pi_p = None
        self.filt_alpha = None
        self.filt_shift_trim = None
        self.filt_vbd = None
        self.filt_shift_cmd = None
        self.filt_roll_cmd = None
        self._sub_depth = None
        self.pub_pitch_mm = None
        self.pub_roll_deg = None
        self.pub_vbd_left = None
        self.pub_vbd_right = None
        self.pub_phase = None

        # Created in on_activate
        self._ctrl_timer = None
        self.dt = 0.1

    # ── Lifecycle callbacks ──────────────────────────────────────────────────

    def on_configure(self, state):
        self._load_params()

        self.pi_theta = PIController(
            self.Kp_theta, self.Ki_theta, -self.q_cmd_max, self.q_cmd_max)
        self.pi_q = PIController(
            self.Kp_q, self.Ki_q, self.shift_min_m, self.shift_max_m)
        self.pi_phi = PIController(
            self.Kp_phi, self.Ki_phi, -self.p_cmd_max, self.p_cmd_max)
        self.pi_p = PIController(
            self.Kp_p, self.Ki_p, -self.roll_max_rad, self.roll_max_rad)
        self.filt_alpha = FirstOrderFilter(self.T_alpha_cmd, self.alpha_dive)
        self.filt_shift_trim = FirstOrderFilter(self.T_shift_trim, 0.0)
        self.filt_vbd = FirstOrderFilter(self.T_vbd_cmd, 0.0)
        self.filt_shift_cmd = FirstOrderFilter(self.shift_cmd_tau, 0.0)
        self.filt_roll_cmd = FirstOrderFilter(self.roll_cmd_tau, 0.0)

        self._sub_roll = self.create_subscription(
            Float64, '/glider/roll_rad', self._cb_roll, 10)
        self._sub_pitch = self.create_subscription(
            Float64, '/glider/pitch_rad', self._cb_pitch, 10)
        self._sub_pitch_rate = self.create_subscription(
            Float64, '/glider/pitch_rate_rad_s', self._cb_pitch_rate, 10)
        self._sub_roll_rate = self.create_subscription(
            Float64, '/glider/roll_rate_rad_s', self._cb_roll_rate, 10)
        self._sub_depth = self.create_subscription(
            Float64, '/pressure/depth', self._cb_depth, 10)
        self._sub_force_surface = self.create_subscription(
            Bool, '/controller/force_surface', self._cb_force_surface, 10)

        self.pub_pitch_mm = self.create_publisher(Float64, '/controller/pitch_mm', 10)
        self.pub_roll_deg = self.create_publisher(Float64, '/controller/roll_deg', 10)
        self.pub_vbd_left = self.create_publisher(UInt8, '/controller/vbd_left_pct', 10)
        self.pub_vbd_right = self.create_publisher(UInt8, '/controller/vbd_right_pct', 10)
        self.pub_phase = self.create_publisher(String, '/controller/phase', 10)

        self.get_logger().info('Controller configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # Reload params so depth_lower reflects the mission value set by state manager
        self._load_params()

        rate = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / rate

        # Full state reset for a fresh dive
        self.dive_count = 0
        self.diving = True
        self.pi_theta.reset()
        self.pi_q.reset()
        self.pi_phi.reset()
        self.pi_p.reset()
        self.filt_alpha.reset(self.alpha_dive)
        self.filt_shift_trim.reset(0.0)
        self.filt_vbd.reset(0.0)
        self.filt_shift_cmd.reset(0.0)
        self.filt_roll_cmd.reset(0.0)
        self.alpha_ref_raw = self.alpha_dive
        self.vbd_raw_pct = 0

        self.operating = True
        self.dive_count += 1
        self._ctrl_timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Controller activated — diving to {self.depth_lower}m (dive #{self.dive_count})')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        if self._ctrl_timer is not None:
            self.destroy_timer(self._ctrl_timer)
            self._ctrl_timer = None
        self.operating = False
        self.get_logger().info('Controller deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        for attr in ('_sub_roll', '_sub_roll_rate', '_sub_pitch', '_sub_pitch_rate', '_sub_depth', '_sub_force_surface'):
            sub = getattr(self, attr, None)
            if sub:
                self.destroy_subscription(sub)
                setattr(self, attr, None)
        for attr in ('pub_pitch_mm', 'pub_roll_deg', 'pub_vbd_left',
                     'pub_vbd_right', 'pub_phase'):
            pub = getattr(self, attr, None)
            if pub:
                self.destroy_publisher(pub)
                setattr(self, attr, None)
        self.get_logger().info('Controller cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    # ── Parameters ──────────────────────────────────────────────────────────

    def _load_params(self):
        p = self.get_parameter
        self.Kp_theta = p('Kp_theta').value
        self.Ki_theta = self.Kp_theta / p('Ti_theta').value
        self.Kp_q = p('Kp_q').value
        self.Ki_q = self.Kp_q / p('Ti_q').value
        self.Kp_phi = p('Kp_phi').value
        self.Ki_phi = self.Kp_phi / p('Ti_phi').value
        self.Kp_p = p('Kp_p').value
        self.Ki_p = self.Kp_p / p('Ti_p').value
        self.q_cmd_max = p('q_cmd_max').value
        self.p_cmd_max = p('p_cmd_max').value
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

    # ── Sensor callbacks ────────────────────────────────────────────────────

    def _cb_roll(self, msg: Float64):
        self.phi = msg.data

    def _cb_pitch(self, msg: Float64):
        self.theta = msg.data

    def _cb_pitch_rate(self, msg: Float64):
        self.q = msg.data

    def _cb_roll_rate(self, msg: Float64):
        self.p = msg.data

    def _cb_depth(self, msg):
        self.depth = msg.data

    def _cb_force_surface(self, msg: Bool):
        if msg.data and self.operating:
            self.diving = False
            self.operating = True  # keep running so supervisor can publish COMPLETE
            self.get_logger().warn('EMERGENCY: forced to climb — surfacing')

    # ── Mission supervisor ──────────────────────────────────────────────────

    def _mission_supervisor(self):
        if self.diving and self.depth >= self.depth_lower:
            self.diving = False
            self.pi_theta.reset()
            self.pi_q.reset()
            self.pi_phi.reset()
            self.pi_p.reset()
            self.get_logger().info('Depth reached — switching to climb')
        elif not self.diving and self.depth <= self.depth_upper:
            if self.dive_count >= self.max_dives:
                self.operating = False
                self.get_logger().info('Mission complete — at surface')
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

    # ── 10 Hz control loop ──────────────────────────────────────────────────

    def _control_loop(self):
        phase = String()
        phase.data = 'OPERATION' if self.operating else 'COMPLETE'
        self.pub_phase.publish(phase)

        if not self.operating:
            return

        if not self._mission_supervisor():
            return

        alpha_filt = self.filt_alpha.update(self.alpha_ref_raw, self.dt)
        shift_trim_filt = self.filt_shift_trim.update(self.shift_trim_raw, self.dt)
        vbd_filt_pct = self.filt_vbd.update(self.vbd_raw_pct, self.dt)

        alpha_err = alpha_filt - self.theta
        q_cmd = self.pi_theta.compute(alpha_err, self.dt)

        q_err = self.q - q_cmd
        shift_pi_out = self.pi_q.compute(q_err, self.dt)

        shift_total = shift_pi_out + shift_trim_filt
        shift_smoothed = self.filt_shift_cmd.update(shift_total, self.dt)
        shift_m = max(self.shift_min_m, min(self.shift_max_m, shift_smoothed))

        phi_err = 0.0 - self.phi
        p_cmd = self.pi_phi.compute(phi_err, self.dt)

        p_err = self.p - p_cmd
        roll_pi_out = self.pi_p.compute(p_err, self.dt)

        roll_smoothed = self.filt_roll_cmd.update(roll_pi_out, self.dt)
        roll_rad = max(-self.roll_max_rad, min(self.roll_max_rad, roll_smoothed))

        shift_mm = shift_m * 1000.0
        roll_deg = math.degrees(roll_rad)
        vbd_pct = max(0, min(100, int(round(vbd_filt_pct))))

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

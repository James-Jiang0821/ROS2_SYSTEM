#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


WATCHDOG_STALENESS_THRESHOLD_S = 1200.0  # testing value = 20 min; production target TBD

WATCHDOG_TOPICS = [
    "/glider/roll_rad",
    "/glider/pitch_rad",
    "/glider/pitch_rate_rad_s",
    "/pressure/depth",
]


class SafetyNode(Node):
    """
    Monitors hard faults from the CAN bridge and triggers emergency state.

    Subscribes to:
        /bridge/pr/fault        (String)  — P&R Teensy fault messages
        /bridge/vbd_left/fault  (String)  — VBD left Teensy fault messages
        /bridge/vbd_right/fault (String)  — VBD right Teensy fault messages
        /glider/range           (Float64) — filtered sonar altitude (m)
        /glider/roll_rad        (Float64) — watchdog: stale → emergency
        /glider/pitch_rad       (Float64) — watchdog: stale → emergency
        /glider/pitch_rate_rad_s (Float64) — watchdog: stale → emergency
        /pressure/depth         (Float64) — watchdog: stale → emergency

    Publishes:
        /safety/emergency  (Bool)   — latched True once triggered;
                                      republished at 2 Hz so the state manager
                                      receives it even under timing uncertainty
        /safety/detail     (String) — human-readable fault description for telemetry
    """

    def __init__(self):
        super().__init__("safety_node")

        self.declare_parameter("proximity_threshold_m", 5.0) # this can change the sonar threshold
        self._proximity_threshold_m = float(
            self.get_parameter("proximity_threshold_m").value)

        self.declare_parameter("staleness_threshold_s", WATCHDOG_STALENESS_THRESHOLD_S)
        self._staleness_threshold_s = float(
            self.get_parameter("staleness_threshold_s").value)

        # Latch: once a hard fault fires we stay in emergency
        self._emergency_latched = False
        self._emergency_detail = ""

        # Watchdog: track last received time per topic (None = never received)
        self._last_received: dict = {t: None for t in WATCHDOG_TOPICS}
        self._node_start_ns = self.get_clock().now().nanoseconds

        # Publishers
        self._emergency_pub = self.create_publisher(Bool, "/safety/emergency", 10)
        self._emergency_detail_pub = self.create_publisher(String, "/safety/detail", 10)

        # Fault topic subscriptions from the CAN bridge
        self.create_subscription(
            String, "/bridge/pr/fault", self._cb_pr_fault, 10)
        self.create_subscription(
            String, "/bridge/vbd_left/fault", self._cb_vbd_left_fault, 10)
        self.create_subscription(
            String, "/bridge/vbd_right/fault", self._cb_vbd_right_fault, 10)

        # Sonar proximity check
        self.create_subscription(
            Float64, "/glider/range", self._cb_range, 10)

        # Watchdog subscriptions
        for topic in WATCHDOG_TOPICS:
            self.create_subscription(
                Float64, topic,
                lambda msg, t=topic: self._cb_watchdog(t),
                20)

        # 2 Hz republish timer — mirrors the Teensy fault-latch broadcast rate
        self.create_timer(0.5, self._republish_emergency)

        # Staleness check timer — runs every 10 s (threshold defaults to WATCHDOG_STALENESS_THRESHOLD_S)
        self.create_timer(10.0, self._check_staleness)

        self.get_logger().info(
            f"Safety node started (proximity threshold: {self._proximity_threshold_m}m, "
            f"staleness threshold: {self._staleness_threshold_s}s)")

    # ── Fault callbacks ───────────────────────────────────────────────────────

    def _cb_pr_fault(self, msg: String):
        self._check_fault("P_R", msg.data)

    def _cb_vbd_left_fault(self, msg: String):
        self._check_fault("VBD_LEFT", msg.data)

    def _cb_vbd_right_fault(self, msg: String):
        self._check_fault("VBD_RIGHT", msg.data)

    def _cb_range(self, msg: Float64):
        range_m = msg.data
        if range_m < self._proximity_threshold_m:
            detail = f"Proximity alert: obstacle {range_m:.2f}m away (threshold {self._proximity_threshold_m:.1f}m)"
            self._trigger_emergency(detail)

    def _cb_watchdog(self, topic: str):
        self._last_received[topic] = self.get_clock().now().nanoseconds

    def _check_staleness(self):
        if self._emergency_latched:
            return

        now_ns = self.get_clock().now().nanoseconds

        for topic, last_ns in self._last_received.items():
            reference_ns = last_ns if last_ns is not None else self._node_start_ns
            elapsed_s = (now_ns - reference_ns) / 1e9

            if elapsed_s > self._staleness_threshold_s:
                detail = (
                    f"Watchdog: {topic} has not published for "
                    f"{elapsed_s:.0f}s (threshold {self._staleness_threshold_s:.0f}s)"
                )
                self._trigger_emergency(detail)
                return

    # ── Core logic ────────────────────────────────────────────────────────────

    def _check_fault(self, source: str, fault_str: str):
        """Trigger emergency if the bridge reports a hard fault."""
        if not fault_str.startswith("HARD:"):
            return

        # Extract fault names from "HARD:F1,F2|state:STATE"
        fault_part = fault_str.split("|")[0][len("HARD:"):]
        self._trigger_emergency(f"Hard fault on {source}: {fault_part}")

    def _trigger_emergency(self, detail: str):
        """Latch and publish an emergency with the given detail string."""
        if self._emergency_latched:
            return

        self._emergency_latched = True
        self._emergency_detail = detail
        self.get_logger().error(f"SAFETY: {detail} — triggering emergency")

        emergency_msg = Bool()
        emergency_msg.data = True
        self._emergency_pub.publish(emergency_msg)

        detail_msg = String()
        detail_msg.data = detail
        self._emergency_detail_pub.publish(detail_msg)

    def _republish_emergency(self):
        """Republish the emergency signal at 2 Hz while latched."""
        if not self._emergency_latched:
            return

        emergency_msg = Bool()
        emergency_msg.data = True
        self._emergency_pub.publish(emergency_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

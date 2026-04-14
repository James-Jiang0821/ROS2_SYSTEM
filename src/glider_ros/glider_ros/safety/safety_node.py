#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


class SafetyNode(Node):
    """
    Monitors hard faults from the CAN bridge and triggers emergency state.

    Subscribes to:
        /bridge/pr/fault        (String)  — P&R Teensy fault messages
        /bridge/vbd_left/fault  (String)  — VBD left Teensy fault messages
        /bridge/vbd_right/fault (String)  — VBD right Teensy fault messages
        /glider/range           (Float64) — filtered sonar altitude (m)

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

        # Latch: once a hard fault fires we stay in emergency
        self._emergency_latched = False
        self._emergency_detail = ""

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

        # 2 Hz republish timer — mirrors the Teensy fault-latch broadcast rate
        self.create_timer(0.5, self._republish_emergency)

        self.get_logger().info(
            f"Safety node started (proximity threshold: {self._proximity_threshold_m}m)")

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

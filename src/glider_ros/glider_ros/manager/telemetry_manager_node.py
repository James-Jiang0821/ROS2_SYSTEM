#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix


class TelemetryManagerNode(Node):
    def __init__(self):
        super().__init__("telemetry_manager")

        self.declare_parameter("publish_period", 5.0)
        self.declare_parameter("include_unknown_fields", True)

        publish_period = float(self.get_parameter("publish_period").value)
        self.include_unknown_fields = bool(
            self.get_parameter("include_unknown_fields").value)

        # Cached values
        self.latest_lat = None
        self.latest_lon = None
        self.latest_state = "UNKNOWN"
        self.latest_emergency_detail = None

        # Subscriptions
        self.create_subscription(NavSatFix, "/gps/fix", self._cb_gps, 10)
        self.create_subscription(String, "/manager/state", self._cb_state, 10)
        self.create_subscription(String, "/emergency/detail", self._cb_emergency_detail, 10)

        # Publisher
        self.telemetry_pub = self.create_publisher(String, "/iridium/sbdwt", 10)

        self.timer = self.create_timer(publish_period, self._publish_telemetry)

        self.get_logger().info("Telemetry manager started")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cb_gps(self, msg: NavSatFix):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude

    def _cb_state(self, msg: String):
        self.latest_state = msg.data.strip()

    def _cb_emergency_detail(self, msg: String):
        self.latest_emergency_detail = msg.data.strip()

    # ── Telemetry building ───────────────────────────────────────────────────

    def _build_telemetry(self) -> str:
        parts = []

        parts.append(f"STATE={self.latest_state}")

        if self.latest_lat is not None and self.latest_lon is not None:
            parts.append(f"LAT={self.latest_lat:.6f}")
            parts.append(f"LON={self.latest_lon:.6f}")
        elif self.include_unknown_fields:
            parts.append("LAT=UNKNOWN")
            parts.append("LON=UNKNOWN")

        if self.latest_state == "EMERGENCY":
            if self.latest_emergency_detail is not None:
                parts.append(f"ERROR={self.latest_emergency_detail}")
            elif self.include_unknown_fields:
                parts.append("ERROR=UNKNOWN")

        return ",".join(parts)

    def _publish_telemetry(self):
        msg = String()
        msg.data = self._build_telemetry()
        self.telemetry_pub.publish(msg)
        self.get_logger().info(f"Published telemetry: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

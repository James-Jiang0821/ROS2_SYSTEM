#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix


class TelemetryManagerNode(Node):
    def __init__(self):
        super().__init__("telemetry_manager")

        # Parameters
        self.declare_parameter("publish_period", 5.0)
        self.declare_parameter("include_unknown_fields", True)

        publish_period = float(self.get_parameter("publish_period").value)
        self.include_unknown_fields = bool(
            self.get_parameter("include_unknown_fields").value
        )

        # Latest cached values
        self.latest_lat = None
        self.latest_lon = None
        self.latest_bms_status = None
        self.latest_leak_status = None
        self.latest_tof_status = None
        self.latest_motor_current = None

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10
        )

        self.bms_sub = self.create_subscription(
            String,
            "/bridge/bms_status",
            self.bms_callback,
            10
        )

        self.leak_sub = self.create_subscription(
            String,
            "/bridge/leak_status",
            self.leak_callback,
            10
        )

        self.tof_sub = self.create_subscription(
            String,
            "/bridge/tof_status",
            self.tof_callback,
            10
        )

        self.motor_sub = self.create_subscription(
            Float32,
            "/bridge/motor_current",
            self.motor_callback,
            10
        )

        # Publisher
        self.telemetry_pub = self.create_publisher(
            String,
            "/iridium/sbdwt",
            10
        )

        # Timer
        self.timer = self.create_timer(publish_period, self.publish_telemetry)

        self.get_logger().info("Telemetry manager started")

    def gps_callback(self, msg: NavSatFix):
        # Only accept real-looking values
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude

    def bms_callback(self, msg: String):
        self.latest_bms_status = msg.data.strip()

    def leak_callback(self, msg: String):
        self.latest_leak_status = msg.data.strip()

    def tof_callback(self, msg: String):
        self.latest_tof_status = msg.data.strip()

    def motor_callback(self, msg: Float32):
        if math.isfinite(msg.data):
            self.latest_motor_current = float(msg.data)

    def format_field(self, key: str, value):
        return f"{key}={value}"

    def build_telemetry_string(self) -> str:
        parts = []

        # GPS
        if self.latest_lat is not None and self.latest_lon is not None:
            parts.append(self.format_field("LAT", f"{self.latest_lat:.6f}"))
            parts.append(self.format_field("LON", f"{self.latest_lon:.6f}"))
        elif self.include_unknown_fields:
            parts.append(self.format_field("LAT", "UNKNOWN"))
            parts.append(self.format_field("LON", "UNKNOWN"))

        # Safety statuses
        if self.latest_bms_status is not None:
            parts.append(self.format_field("BMS", self.latest_bms_status))
        elif self.include_unknown_fields:
            parts.append(self.format_field("BMS", "UNKNOWN"))

        if self.latest_leak_status is not None:
            parts.append(self.format_field("LEAK", self.latest_leak_status))
        elif self.include_unknown_fields:
            parts.append(self.format_field("LEAK", "UNKNOWN"))

        if self.latest_tof_status is not None:
            parts.append(self.format_field("TOF", self.latest_tof_status))
        elif self.include_unknown_fields:
            parts.append(self.format_field("TOF", "UNKNOWN"))

        if self.latest_motor_current is not None:
            parts.append(self.format_field("MOTOR", f"{self.latest_motor_current:.2f}"))
        elif self.include_unknown_fields:
            parts.append(self.format_field("MOTOR", "UNKNOWN"))

        return ",".join(parts)

    def publish_telemetry(self):
        telemetry_text = self.build_telemetry_string()

        msg = String()
        msg.data = telemetry_text
        self.telemetry_pub.publish(msg)

        self.get_logger().info(f"Published telemetry: {telemetry_text}")


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
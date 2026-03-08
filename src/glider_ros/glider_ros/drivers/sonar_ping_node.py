#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Bool

from brping import Ping1D


class SonarPingNode(Node):
    """
    ROS2 driver node for the Blue Robotics Ping Sonar Altimeter and Echosounder.

    Publishes:
      /sonar/range       -> sensor_msgs/Range   [m]
      /sonar/confidence  -> std_msgs/Float32    [0-100]
      /sonar/connected   -> std_msgs/Bool
    """

    def __init__(self) -> None:
        super().__init__("sonar_ping_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("frame_id", "sonar_link")
        self.declare_parameter("rate_hz", 10.0)

        # Connection mode
        self.declare_parameter("use_udp", False)

        # Serial settings
        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        # UDP settings
        self.declare_parameter("udp_host", "192.168.2.2")
        self.declare_parameter("udp_port", 9090)

        # Range metadata
        self.declare_parameter("field_of_view_rad", 0.0)
        self.declare_parameter("min_range_m", 0.03)
        self.declare_parameter("max_range_m", 50.0)

        # Reconnect behaviour
        self.declare_parameter("reconnect_attempts_before_reset", 5)

        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.use_udp = bool(self.get_parameter("use_udp").value)

        self.device = str(self.get_parameter("device").value)
        self.baudrate = int(self.get_parameter("baudrate").value)

        self.udp_host = str(self.get_parameter("udp_host").value)
        self.udp_port = int(self.get_parameter("udp_port").value)

        self.field_of_view_rad = float(self.get_parameter("field_of_view_rad").value)
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)

        self.reconnect_attempts_before_reset = int(
            self.get_parameter("reconnect_attempts_before_reset").value
        )

        if self.rate_hz <= 0.0:
            self.get_logger().warn("rate_hz must be > 0. Falling back to 10 Hz.")
            self.rate_hz = 10.0

        # ---------------- Publishers ----------------
        self.range_pub = self.create_publisher(Range, "/sonar/range", 10)
        self.confidence_pub = self.create_publisher(Float32, "/sonar/confidence", 10)
        self.connected_pub = self.create_publisher(Bool, "/sonar/connected", 10)

        # ---------------- Internal state ----------------
        self.ping: Optional[Ping1D] = None
        self.connected = False
        self.failed_reads = 0

        self._connect_ping()

        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self._poll_ping)

        self.get_logger().info("sonar_ping_node started")

    def _connect_ping(self) -> None:
        self.connected = False
        self._publish_connected(False)

        try:
            self.ping = Ping1D()

            if self.use_udp:
                self.get_logger().info(
                    f"Connecting to Ping sonar over UDP: {self.udp_host}:{self.udp_port}"
                )
                self.ping.connect_udp(self.udp_host, self.udp_port)
            else:
                self.get_logger().info(
                    f"Connecting to Ping sonar over serial: {self.device} @ {self.baudrate}"
                )
                self.ping.connect_serial(self.device, self.baudrate)

            if not self.ping.initialize():
                self.get_logger().error("Ping sonar initialize() failed")
                self.ping = None
                return

            self.connected = True
            self.failed_reads = 0
            self._publish_connected(True)
            self.get_logger().info("Ping sonar connection established")

        except Exception as e:
            self.ping = None
            self.connected = False
            self._publish_connected(False)
            self.get_logger().error(f"Failed to connect to Ping sonar: {e}")

    def _poll_ping(self) -> None:
        if self.ping is None or not self.connected:
            self._connect_ping()
            return

        try:
            data = self.ping.get_distance()

            if not data:
                self.failed_reads += 1
                self.get_logger().warn(
                    f"No sonar data returned (failed_reads={self.failed_reads})"
                )
                if self.failed_reads >= self.reconnect_attempts_before_reset:
                    self.get_logger().warn("Too many failed reads, resetting connection")
                    self._reset_connection()
                return

            self.failed_reads = 0

            distance_mm = data.get("distance")
            confidence = data.get("confidence")

            if distance_mm is None:
                self.get_logger().warn("Sonar data missing 'distance'")
                return

            distance_m = float(distance_mm) / 1000.0

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = float(self.field_of_view_rad)
            msg.min_range = float(self.min_range_m)
            msg.max_range = float(self.max_range_m)
            msg.range = float(distance_m)
            self.range_pub.publish(msg)

            if confidence is not None:
                conf_msg = Float32()
                conf_msg.data = float(confidence)
                self.confidence_pub.publish(conf_msg)

            if not self.connected:
                self.connected = True
                self._publish_connected(True)

        except Exception as e:
            self.failed_reads += 1
            self.get_logger().error(f"Sonar read error: {e}")

            if self.failed_reads >= self.reconnect_attempts_before_reset:
                self.get_logger().warn("Read failure threshold exceeded, resetting connection")
                self._reset_connection()

    def _reset_connection(self) -> None:
        self.connected = False
        self._publish_connected(False)
        self.ping = None
        self.failed_reads = 0

    def _publish_connected(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self.connected_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SonarPingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
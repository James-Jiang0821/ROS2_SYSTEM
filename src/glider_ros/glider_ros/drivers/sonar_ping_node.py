#!/usr/bin/env python3

from brping import Ping1D

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import Range


class PingSonarNode(Node):
    def __init__(self):
        super().__init__("sonar_ping1d_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("poll_period", 0.2)
        self.declare_parameter("frame_id", "sonar_link")
        self.declare_parameter("min_range_m", 0.3)
        self.declare_parameter("max_range_m", 50.0)
        self.declare_parameter("field_of_view_rad", 0.17)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.poll_period = float(self.get_parameter("poll_period").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.field_of_view_rad = float(self.get_parameter("field_of_view_rad").value)

        # Publishers
        self.range_pub = self.create_publisher(Range, "/sonar/range", 10)
        self.confidence_pub = self.create_publisher(Float32, "/sonar/confidence", 10)
        self.distance_mm_pub = self.create_publisher(Int32, "/sonar/distance_mm", 10)
        self.status_pub = self.create_publisher(String, "/sonar/status", 10)

        self.ping = Ping1D()
        self.connected = False

        self._connect_device()

        self.timer = self.create_timer(self.poll_period, self.read_and_publish)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _connect_device(self):
        try:
            self.get_logger().info(f"Connecting to Ping sonar on {self.port} at {self.baud} baud")
            self.ping.connect_serial(self.port, self.baud)

            if not self.ping.initialize():
                self.get_logger().error("Failed to initialize Ping sonar")
                self.publish_status("init_failed")
                self.connected = False
                return

            self.connected = True
            self.get_logger().info("Ping sonar initialized successfully")
            self.publish_status("connected")

        except Exception as e:
            self.get_logger().error(f"Sonar connection failed: {e}")
            self.publish_status("connect_failed")
            self.connected = False

    def read_and_publish(self):
        if not self.connected:
            self._connect_device()
            return

        try:
            data = self.ping.get_distance()

            if not data:
                self.get_logger().warning("Failed to get distance data")
                self.publish_status("read_failed")
                return

            distance_mm = int(data["distance"])
            confidence = float(data["confidence"])
            distance_m = distance_mm / 1000.0

            # Publish raw distance in mm
            mm_msg = Int32()
            mm_msg.data = distance_mm
            self.distance_mm_pub.publish(mm_msg)

            # Publish confidence
            conf_msg = Float32()
            conf_msg.data = confidence
            self.confidence_pub.publish(conf_msg)

            # Publish ROS Range message
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = self.frame_id
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = self.field_of_view_rad
            range_msg.min_range = self.min_range_m
            range_msg.max_range = self.max_range_m
            range_msg.range = distance_m
            self.range_pub.publish(range_msg)

            self.publish_status("ok")

        except Exception as e:
            self.get_logger().error(f"Sonar read error: {e}")
            self.publish_status("error")
            self.connected = False


def main(args=None):
    rclpy.init(args=args)
    node = PingSonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
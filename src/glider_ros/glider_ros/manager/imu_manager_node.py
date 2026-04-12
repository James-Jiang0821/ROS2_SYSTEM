#!/usr/bin/env python3
"""
IMU Manager Node

Subscribes to /imu/data (orientation quaternion + angular velocity from the
BNO085's onboard 9-DOF fusion) and publishes clean scalar values for the
controller and telemetry.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class ImuManagerNode(Node):
    def __init__(self):
        super().__init__("imu_manager_node")

        self.create_subscription(Imu, "/imu/data", self._on_imu, 20)

        self._roll_pub = self.create_publisher(Float64, "/glider/roll_rad", 20)
        self._pitch_pub = self.create_publisher(Float64, "/glider/pitch_rad", 20)
        self._pitch_rate_pub = self.create_publisher(Float64, "/glider/pitch_rate_rad_s", 20)
        self._heading_pub = self.create_publisher(Float64, "/glider/heading_deg", 20)

    def _on_imu(self, msg: Imu) -> None:
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Quaternion → roll, pitch, yaw
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        heading_deg = math.degrees(yaw) % 360.0
        pitch_rate = msg.angular_velocity.y

        self._roll_pub.publish(Float64(data=roll))
        self._pitch_pub.publish(Float64(data=pitch))
        self._pitch_rate_pub.publish(Float64(data=pitch_rate))
        self._heading_pub.publish(Float64(data=heading_deg))


def main(args=None):
    rclpy.init(args=args)
    node = ImuManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

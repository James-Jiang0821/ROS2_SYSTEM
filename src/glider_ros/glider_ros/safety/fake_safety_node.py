#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class FakeSafetyNode(Node):
    def __init__(self):
        super().__init__("fake_safety_node")

        # Parameters
        self.declare_parameter("publish_period", 2.0)

        publish_period = float(self.get_parameter("publish_period").value)

        # Publishers
        self.bms_pub = self.create_publisher(String, "/bridge/bms_status", 10)
        self.leak_pub = self.create_publisher(String, "/bridge/leak_status", 10)
        self.tof_pub = self.create_publisher(String, "/bridge/tof_status", 10)
        self.motor_pub = self.create_publisher(Float32, "/bridge/motor_current", 10)

        # Internal counter so we can vary outputs over time
        self.counter = 0

        # Timer
        self.timer = self.create_timer(publish_period, self.publish_fake_data)

        self.get_logger().info("Fake safety node started")

    def publish_fake_data(self):
        self.counter += 1

        # Example fake patterns
        # Most of the time: safe
        # Every few cycles: warning/fault-like values
        if self.counter % 15 == 0:
            bms_status = "FAULT"
        else:
            bms_status = "OK"

        if self.counter % 20 == 0:
            leak_status = "LEAK_DETECTED"
        else:
            leak_status = "DRY"

        if self.counter % 10 == 0:
            tof_status = "OBSTACLE_CLOSE"
        else:
            tof_status = "CLEAR"

        motor_current = 1.2 + (self.counter % 5) * 0.35

        # Build messages
        bms_msg = String()
        bms_msg.data = bms_status

        leak_msg = String()
        leak_msg.data = leak_status

        tof_msg = String()
        tof_msg.data = tof_status

        motor_msg = Float32()
        motor_msg.data = float(motor_current)

        # Publish
        self.bms_pub.publish(bms_msg)
        self.leak_pub.publish(leak_msg)
        self.tof_pub.publish(tof_msg)
        self.motor_pub.publish(motor_msg)

        self.get_logger().info(
            f"Published fake safety data | "
            f"BMS={bms_status}, LEAK={leak_status}, TOF={tof_status}, MOTOR={motor_current:.2f} A"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
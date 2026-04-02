#!/usr/bin/env python3
"""
Minimal CAN Bridge Test Node
Only listens to /iridium/incoming_message and forwards it onto CAN.

Purpose:
- Verify that an Iridium incoming ROS message can be picked up by the bridge node
- Verify that the bridge node sends something visible on candump
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can


# Test CAN ID for iridium incoming message -> CAN
# Pick something easy to spot in candump
IRIDIUM_TEST_CAN_ID = 0x321


class CanBridgeNode(Node):
    def __init__(self):
        super().__init__("can_bridge_node")

        # CAN setup
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bitrate", 500000)

        channel = self.get_parameter("can_channel").value
        bitrate = self.get_parameter("can_bitrate").value

        try:
            self.bus = can.interface.Bus(
                channel=channel,
                bustype="socketcan",
                bitrate=bitrate
            )
            self.get_logger().info(f"CAN connected on {channel} at {bitrate} bps")
        except Exception as e:
            self.get_logger().error(f"Failed to open CAN bus: {e}")
            self.bus = None
            return

        # ONLY subscriber kept for this test
        self.create_subscription(
            String,
            "/iridium/incoming_message",
            self.on_iridium_incoming,
            10
        )

        self.get_logger().info("Minimal bridge node ready: listening only to /iridium/incoming_message")

    def on_iridium_incoming(self, msg: String):
        """
        Take the incoming ROS string and send it as CAN data.

        CAN frame max payload is 8 bytes for standard CAN, so for this test
        we truncate the message to the first 8 bytes.
        """
        if not self.bus:
            self.get_logger().error("CAN bus not available")
            return

        text = msg.data
        data = text.encode("utf-8")[:8]   # standard CAN = max 8 bytes

        try:
            self.bus.send(can.Message(
                arbitration_id=IRIDIUM_TEST_CAN_ID,
                data=data,
                is_extended_id=False
            ))
            self.get_logger().info(
                f"Received /iridium/incoming_message='{text}' -> sent CAN 0x{IRIDIUM_TEST_CAN_ID:03X} data={data.hex()}"
            )
        except can.CanError as e:
            self.get_logger().error(f"CAN send error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Sonar Manager Node

Subscribes to raw sonar driver output and publishes clean altitude data.
Responsibilities:
  - Reject readings below a confidence threshold
  - Moving-average filter to smooth noisy altitude readings
  - Re-publish on /glider/altitude for consumption by controllers and telemetry
"""

from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float64


class SonarManagerNode(Node):
    def __init__(self):
        super().__init__("sonar_manager_node")

        self.declare_parameter("min_confidence", 50.0)  # percent; reject readings below this
        self.declare_parameter("filter_window", 5)

        self._min_confidence = float(self.get_parameter("min_confidence").value)
        window = int(self.get_parameter("filter_window").value)

        self._altitude_buf: deque[float] = deque(maxlen=window)
        self._latest_confidence: float = 0.0

        # Subscriptions (raw driver output)
        self.create_subscription(Range, "/sonar/range", self._on_range, 10)
        self.create_subscription(Float32, "/sonar/confidence", self._on_confidence, 10)

        # Publishers (clean output for the rest of the system)
        self._altitude_pub = self.create_publisher(Float64, "/glider/altitude", 10)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_confidence(self, msg: Float32) -> None:
        self._latest_confidence = msg.data

    def _on_range(self, msg: Range) -> None:
        if self._latest_confidence < self._min_confidence:
            return

        # Clamp to valid sensor range before filtering
        if msg.range < msg.min_range or msg.range > msg.max_range:
            return

        self._altitude_buf.append(msg.range)

        out = Float64()
        out.data = sum(self._altitude_buf) / len(self._altitude_buf)
        self._altitude_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SonarManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

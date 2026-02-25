#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class FakeLeakPublisher(Node):
    """
    Publishes /leak_detected as std_msgs/Bool.

    Default behaviour:
      - publishes False at 2 Hz
      - every N seconds, flips to True for `leak_pulse_sec`, then back to False
    """

    def __init__(self):
        super().__init__('fake_leak_publisher')

        # Parameters
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('leak_every_sec', 15.0)
        self.declare_parameter('leak_pulse_sec', 5.0)

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.leak_every_sec = float(self.get_parameter('leak_every_sec').value)
        self.leak_pulse_sec = float(self.get_parameter('leak_pulse_sec').value)

        self.pub = self.create_publisher(Bool, '/leak_detected', 10)

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info(
            f"Publishing /leak_detected at {self.publish_hz} Hz, "
            f"leak_every_sec={self.leak_every_sec}, leak_pulse_sec={self.leak_pulse_sec}"
        )

    def on_timer(self):
        now = self.get_clock().now()
        elapsed = (now - self.t0).nanoseconds / 1e9

        # Leak pattern: every leak_every_sec, set True for leak_pulse_sec
        phase = elapsed % self.leak_every_sec
        leak = phase < self.leak_pulse_sec

        msg = Bool()
        msg.data = leak
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeLeakPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

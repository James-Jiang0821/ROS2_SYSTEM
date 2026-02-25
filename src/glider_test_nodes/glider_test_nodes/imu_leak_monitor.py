#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

class ImuLeakMonitor(Node):
    """
    Subscribes to:
      - /imu/data_raw (sensor_msgs/Imu)
      - /leak_detected (std_msgs/Bool)

    Prints:
      - current leak state changes
      - when leak becomes True, prints the latest IMU sample (accel+gyro)
    """

    def __init__(self):
        super().__init__('imu_leak_monitor')

        self.latest_imu = None
        self.leak_state = None

        self.sub_imu = self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 10)
        self.sub_leak = self.create_subscription(Bool, '/leak_detected', self.on_leak, 10)

        self.get_logger().info("Listening to /imu/data_raw and /leak_detected ...")

    def on_imu(self, msg: Imu):
        self.latest_imu = msg

    def on_leak(self, msg: Bool):
        new_state = bool(msg.data)
        if self.leak_state is None:
            self.leak_state = new_state
            self.get_logger().info(f"Initial leak state: {self.leak_state}")
            return

        if new_state != self.leak_state:
            self.leak_state = new_state
            self.get_logger().warn(f"LEAK STATE CHANGED -> {self.leak_state}")

            if self.leak_state and self.latest_imu is not None:
                la = self.latest_imu.linear_acceleration
                av = self.latest_imu.angular_velocity
                self.get_logger().warn(
                    f"IMU snapshot on leak: "
                    f"accel=({la.x:.3f},{la.y:.3f},{la.z:.3f}) "
                    f"gyro=({av.x:.3f},{av.y:.3f},{av.z:.3f})"
                )
            elif self.leak_state and self.latest_imu is None:
                self.get_logger().warn("Leak became True but no IMU received yet.")

def main():
    rclpy.init()
    node = ImuLeakMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

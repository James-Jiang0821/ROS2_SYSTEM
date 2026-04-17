#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
)


class Bno085ImuNode(Node):
    def __init__(self):
        super().__init__("bno085_imu_node")

        # Parameters
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 10.0)

        self.declare_parameter("lin_acc_cov", 1e-2)
        self.declare_parameter("ang_vel_cov", 1e-2)
        self.declare_parameter("mag_cov", 1e-2)

        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.lin_acc_cov = float(self.get_parameter("lin_acc_cov").value)
        self.ang_vel_cov = float(self.get_parameter("ang_vel_cov").value)
        self.mag_cov = float(self.get_parameter("mag_cov").value)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 20)

        # Sensor + stats
        self.bno = None
        self.good = 0
        self.bad = 0

        self._init_sensor()

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._tick)

    def _init_sensor(self):
        self.get_logger().info("Initialising I2C and BNO085...")

        # BNO085 needs 400kHz I2C. On Raspberry Pi the bus clock is set by
        # /boot/config.txt (add: dtparam=i2c_arm=on,i2c_arm_baudrate=400000);
        # Blinka's busio.I2C frequency= arg is ignored on this platform.
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)

        self.get_logger().info("Resetting BNO085...")
        self.bno.soft_reset()
        time.sleep(2.0)

        self.get_logger().info("Enabling rotation vector (9-DOF onboard fusion)...")
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.1)

        self.get_logger().info("Enabling gyroscope...")
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.1)

        self.get_logger().info("BNO085 ready.")

    def _tick(self):
        try:
            quat = self.bno.quaternion   # (i, j, k, real) == (x, y, z, w)
            gx, gy, gz = self.bno.gyro  # rad/s

            if quat is None:
                return

            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.x = float(quat[0])
            imu_msg.orientation.y = float(quat[1])
            imu_msg.orientation.z = float(quat[2])
            imu_msg.orientation.w = float(quat[3])

            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)

            for i in range(9):
                imu_msg.orientation_covariance[i] = 0.0
                imu_msg.angular_velocity_covariance[i] = 0.0
                imu_msg.linear_acceleration_covariance[i] = -1.0

            imu_msg.angular_velocity_covariance[0] = self.ang_vel_cov
            imu_msg.angular_velocity_covariance[4] = self.ang_vel_cov
            imu_msg.angular_velocity_covariance[8] = self.ang_vel_cov

            self.imu_pub.publish(imu_msg)
            self.good += 1

            if self.good % 10 == 0:
                self.get_logger().info(
                    f"QUAT [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}] | "
                    f"GYR [{gx:.3f}, {gy:.3f}, {gz:.3f}] rad/s"
                )

        except Exception as e:
            self.bad += 1
            self.get_logger().warn(f"IMU read error (count={self.bad}): {e}")

            try:
                self.get_logger().warn("Reinitialising BNO085...")
                self._init_sensor()
            except Exception as reinit_error:
                self.get_logger().error(f"Reinit failed: {reinit_error}")
                time.sleep(1.0)

def main():
    rclpy.init()
    node = Bno085ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
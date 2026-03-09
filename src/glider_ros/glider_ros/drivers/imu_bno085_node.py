#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER


class Bno085ImuNode(Node):
    def __init__(self):
        super().__init__("bno085_imu_node")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("lin_acc_cov", 1e-2)

        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.lin_acc_cov = float(self.get_parameter("lin_acc_cov").value)

        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 20)

        self.bno = None
        self.good = 0
        self.bad = 0
        self.last_stat_t = time.time()
        self.last_accel = None

        self._init_sensor()

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.2
        self.timer = self.create_timer(period, self._tick)

    def _init_sensor(self):
        self.get_logger().info("Initialising I2C and BNO085...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)

        self.get_logger().info("Resetting BNO085...")
        self.bno.soft_reset()
        time.sleep(2.0)

        self.get_logger().info("Enabling accelerometer...")
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        time.sleep(1.0)

        self.get_logger().info("BNO085 ready.")

    def _tick(self):
        try:
            ax, ay, az = self.bno.acceleration

            current = (round(ax, 3), round(ay, 3), round(az, 3))
            if self.last_accel is not None and current == self.last_accel:
                # only warn occasionally to avoid spam
                if self.good % 20 == 0:
                    self.get_logger().warn(f"Accel unchanged: {current}")
            self.last_accel = current

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # orientation unavailable for now
            msg.orientation_covariance[0] = -1.0

            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            # gyro disabled for now
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0

            for i in range(9):
                msg.linear_acceleration_covariance[i] = 0.0
                msg.angular_velocity_covariance[i] = 0.0

            msg.linear_acceleration_covariance[0] = self.lin_acc_cov
            msg.linear_acceleration_covariance[4] = self.lin_acc_cov
            msg.linear_acceleration_covariance[8] = self.lin_acc_cov

            self.imu_pub.publish(msg)
            self.good += 1

            if self.good % 10 == 0:
                self.get_logger().info(
                    f"RAW ax={ax:.3f} ay={ay:.3f} az={az:.3f}"
                )

            t = time.time()
            if t - self.last_stat_t > 2.0:
                self.get_logger().info(f"IMU stats: good={self.good} bad={self.bad}")
                self.last_stat_t = t

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
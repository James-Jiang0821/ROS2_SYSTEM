#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)


class Bno085ImuNode(Node):
    def __init__(self):
        super().__init__("bno085_imu_node")

        # Parameters
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("i2c_frequency", 400000)

        self.declare_parameter("lin_acc_cov", 1e-2)
        self.declare_parameter("ang_vel_cov", 1e-2)
        self.declare_parameter("mag_cov", 1e-2)

        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.i2c_frequency = int(self.get_parameter("i2c_frequency").value)

        self.lin_acc_cov = float(self.get_parameter("lin_acc_cov").value)
        self.ang_vel_cov = float(self.get_parameter("ang_vel_cov").value)
        self.mag_cov = float(self.get_parameter("mag_cov").value)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 20)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 20)

        # Sensor + stats
        self.bno = None
        self.good = 0
        self.bad = 0
        self.last_stat_t = time.time()

        self._init_sensor()

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._tick)

    def _init_sensor(self):
        self.get_logger().info("Initialising I2C and BNO085...")

        i2c = busio.I2C(board.SCL, board.SDA, frequency=self.i2c_frequency)
        self.bno = BNO08X_I2C(i2c)

        self.get_logger().info("Resetting BNO085...")
        self.bno.soft_reset()
        time.sleep(2.0)

        self.get_logger().info("Enabling accelerometer...")
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        time.sleep(0.1)

        self.get_logger().info("Enabling gyroscope...")
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.1)

        self.get_logger().info("Enabling magnetometer...")
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        time.sleep(0.1)

        self.get_logger().info("BNO085 ready.")

    def _tick(self):
        try:
            # Read all enabled sensors
            ax, ay, az = self.bno.acceleration
            gx, gy, gz = self.bno.gyro
            mx, my, mz = self.bno.magnetic

            stamp = self.get_clock().now().to_msg()

            # ---------------------------
            # IMU message
            # ---------------------------
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id

            # Orientation not published here
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 0.0
            imu_msg.orientation_covariance[0] = -1.0

            # Angular velocity (gyro) in rad/s
            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)

            # Linear acceleration in m/s^2
            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)

            # Zero all covariances first
            for i in range(9):
                imu_msg.angular_velocity_covariance[i] = 0.0
                imu_msg.linear_acceleration_covariance[i] = 0.0

            imu_msg.angular_velocity_covariance[0] = self.ang_vel_cov
            imu_msg.angular_velocity_covariance[4] = self.ang_vel_cov
            imu_msg.angular_velocity_covariance[8] = self.ang_vel_cov

            imu_msg.linear_acceleration_covariance[0] = self.lin_acc_cov
            imu_msg.linear_acceleration_covariance[4] = self.lin_acc_cov
            imu_msg.linear_acceleration_covariance[8] = self.lin_acc_cov

            # ---------------------------
            # Magnetic field message
            # ---------------------------
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id

            # Adafruit BNO08x magnetic output is in microtesla (uT),
            # ROS MagneticField expects tesla (T), so convert.
            mag_msg.magnetic_field.x = float(mx) * 1e-6
            mag_msg.magnetic_field.y = float(my) * 1e-6
            mag_msg.magnetic_field.z = float(mz) * 1e-6

            for i in range(9):
                mag_msg.magnetic_field_covariance[i] = 0.0

            mag_msg.magnetic_field_covariance[0] = self.mag_cov
            mag_msg.magnetic_field_covariance[4] = self.mag_cov
            mag_msg.magnetic_field_covariance[8] = self.mag_cov

            # Publish
            self.imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)
            self.good += 1

            if self.good % 10 == 0:
                self.get_logger().info(
                    f"ACC [{ax:.3f}, {ay:.3f}, {az:.3f}] m/s^2 | "
                    f"GYR [{gx:.3f}, {gy:.3f}, {gz:.3f}] rad/s | "
                    f"MAG [{mx:.3f}, {my:.3f}, {mz:.3f}] uT"
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
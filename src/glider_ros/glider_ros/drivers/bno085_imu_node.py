#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField

# I2C + BNO085
import board
import busio
from adafruit_bno08x import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)

class Bno085ImuNode(Node):
    def __init__(self):
        super().__init__('bno085_imu_node')

        # ---------- Params ----------
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('publish_mag', True)

        # Covariances (tune later; placeholders are fine)
        self.declare_parameter('lin_acc_cov', 1e-2)   # (m/s^2)^2
        self.declare_parameter('ang_vel_cov', 1e-3)   # (rad/s)^2
        self.declare_parameter('mag_cov', 1e-10)      # Tesla^2

        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.publish_mag = bool(self.get_parameter('publish_mag').value)

        self.lin_acc_cov = float(self.get_parameter('lin_acc_cov').value)
        self.ang_vel_cov = float(self.get_parameter('ang_vel_cov').value)
        self.mag_cov = float(self.get_parameter('mag_cov').value)

        # ---------- Publishers ----------
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 20)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 20)

        # ---------- Init I2C + Sensor ----------
        self.get_logger().info("Initialising I2C and BNO085...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)

        # Enable the reports we want
        # Note: Units are typically:
        #  - accelerometer: m/s^2
        #  - gyro: rad/s
        #  - magnetometer: microtesla (uT) -> convert to Tesla by *1e-6
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)

        self.get_logger().info("BNO085 initialised and reports enabled.")

        # ---------- Timer ----------
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.02
        self.timer = self.create_timer(period, self._tick)

        # Stats
        self.good = 0
        self.bad = 0
        self.last_stat_t = time.time()

    def _tick(self):
        try:
            # Read sensor
            ax, ay, az = self.bno.acceleration          # m/s^2
            gx, gy, gz = self.bno.gyro                 # rad/s
            mx, my, mz = self.bno.magnetic             # uT

            stamp = self.get_clock().now().to_msg()

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.frame_id

            # Orientation not provided here (you can later use rotation vector if desired)
            imu.orientation_covariance[0] = -1.0

            imu.linear_acceleration.x = float(ax)
            imu.linear_acceleration.y = float(ay)
            imu.linear_acceleration.z = float(az)

            imu.angular_velocity.x = float(gx)
            imu.angular_velocity.y = float(gy)
            imu.angular_velocity.z = float(gz)

            # Covariances (diagonal)
            for i in range(9):
                imu.linear_acceleration_covariance[i] = 0.0
                imu.angular_velocity_covariance[i] = 0.0
            imu.linear_acceleration_covariance[0] = self.lin_acc_cov
            imu.linear_acceleration_covariance[4] = self.lin_acc_cov
            imu.linear_acceleration_covariance[8] = self.lin_acc_cov
            imu.angular_velocity_covariance[0] = self.ang_vel_cov
            imu.angular_velocity_covariance[4] = self.ang_vel_cov
            imu.angular_velocity_covariance[8] = self.ang_vel_cov

            self.imu_pub.publish(imu)

            if self.publish_mag:
                mag = MagneticField()
                mag.header.stamp = stamp
                mag.header.frame_id = self.frame_id

                # Convert uT -> Tesla
                mag.magnetic_field.x = float(mx) * 1e-6
                mag.magnetic_field.y = float(my) * 1e-6
                mag.magnetic_field.z = float(mz) * 1e-6

                for i in range(9):
                    mag.magnetic_field_covariance[i] = 0.0
                mag.magnetic_field_covariance[0] = self.mag_cov
                mag.magnetic_field_covariance[4] = self.mag_cov
                mag.magnetic_field_covariance[8] = self.mag_cov

                self.mag_pub.publish(mag)

            self.good += 1

            t = time.time()
            if t - self.last_stat_t > 2.0:
                self.get_logger().info(f"Published IMU: good={self.good} bad={self.bad}")
                self.last_stat_t = t

        except Exception as e:
            self.bad += 1
            # Don’t spam every tick; warn occasionally
            if self.bad % 50 == 0:
                self.get_logger().warn(f"IMU read/publish error (count={self.bad}): {e}")

def main():
    rclpy.init()
    node = Bno085ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import math
import time
from typing import Optional, List

import serial
import serial.serialutil

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Imu, MagneticField


def parse_csv9(line: str) -> Optional[List[float]]:
    parts = line.strip().split(',')
    if len(parts) != 9:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def remap3(v, idx_map, sign_map):
    return [
        float(sign_map[0]) * float(v[int(idx_map[0])]),
        float(sign_map[1]) * float(v[int(idx_map[1])]),
        float(sign_map[2]) * float(v[int(idx_map[2])]),
    ]


class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')

        # ---------- Params ----------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        # Expected firmware units are often: accel[g], gyro[deg/s], mag[uT]
        self.declare_parameter('accel_scale', 9.80665)  # g -> m/s^2
        self.declare_parameter('gyro_scale', math.pi / 180.0)  # deg/s -> rad/s
        self.declare_parameter('mag_scale', 1e-6)  # uT -> Tesla

        # Axis remap + sign flips (defaults = identity)
        self.declare_parameter('accel_map', [0, 1, 2])
        self.declare_parameter('gyro_map',  [0, 1, 2])
        self.declare_parameter('mag_map',   [0, 1, 2])
        self.declare_parameter('accel_sign', [1.0, 1.0, 1.0])
        self.declare_parameter('gyro_sign',  [1.0, 1.0, 1.0])
        self.declare_parameter('mag_sign',   [1.0, 1.0, 1.0])

        # Covariances (tune later; placeholders are OK)
        self.declare_parameter('lin_acc_cov', 1e-2)   # (m/s^2)^2
        self.declare_parameter('ang_vel_cov', 1e-3)   # (rad/s)^2
        self.declare_parameter('mag_cov', 1e-10)      # Tesla^2

        # Serial behavior
        self.declare_parameter('read_hz', 200.0)      # timer tick; not your sensor rate
        self.declare_parameter('reconnect_s', 1.0)
        self.declare_parameter('drop_garbage_lines', True)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.accel_scale = float(self.get_parameter('accel_scale').value)
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)
        self.mag_scale = float(self.get_parameter('mag_scale').value)

        self.accel_map = list(self.get_parameter('accel_map').value)
        self.gyro_map = list(self.get_parameter('gyro_map').value)
        self.mag_map = list(self.get_parameter('mag_map').value)
        self.accel_sign = list(self.get_parameter('accel_sign').value)
        self.gyro_sign = list(self.get_parameter('gyro_sign').value)
        self.mag_sign = list(self.get_parameter('mag_sign').value)

        self.lin_acc_cov = float(self.get_parameter('lin_acc_cov').value)
        self.ang_vel_cov = float(self.get_parameter('ang_vel_cov').value)
        self.mag_cov = float(self.get_parameter('mag_cov').value)

        self.read_hz = float(self.get_parameter('read_hz').value)
        self.reconnect_s = float(self.get_parameter('reconnect_s').value)
        self.drop_garbage = bool(self.get_parameter('drop_garbage_lines').value)

        # ---------- Publishers ----------
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 20)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 20)

        # ---------- Serial state ----------
        self.ser: Optional[serial.Serial] = None
        self.next_reconnect_time = 0.0

        # Stats
        self.good = 0
        self.bad = 0
        self.last_stat_t = time.time()

        self.get_logger().info(f"Serial IMU bridge starting. Port={self.port} baud={self.baud}")
        self._try_open()

        # Timer loop
        self.timer = self.create_timer(1.0 / self.read_hz, self._tick)

    def _try_open(self):
        now = time.time()
        if now < self.next_reconnect_time:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            # Optional: flush old buffered bytes
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self.get_logger().info("Serial opened.")
        except serial.serialutil.SerialException as e:
            self.ser = None
            self.next_reconnect_time = now + self.reconnect_s
            self.get_logger().warn(f"Serial open failed: {e}. Retrying in {self.reconnect_s}s")

    def _close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _tick(self):
        if self.ser is None:
            self._try_open()
            return

        try:
            # Read all available lines this tick (keeps latency low)
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                vals = parse_csv9(line)

                # If firmware prints headers, optionally drop them
                if vals is None:
                    self.bad += 1
                    if self.drop_garbage:
                        continue
                    else:
                        # If you want to debug raw, log occasionally
                        if self.bad % 50 == 0:
                            self.get_logger().warn(f"Bad line: {line}")
                        continue

                ax, ay, az = vals[0:3]
                gx, gy, gz = vals[3:6]
                mx, my, mz = vals[6:9]

                # Remap + sign
                a = remap3([ax, ay, az], self.accel_map, self.accel_sign)
                g = remap3([gx, gy, gz], self.gyro_map, self.gyro_sign)
                m = remap3([mx, my, mz], self.mag_map, self.mag_sign)

                # Unit conversion
                a = [x * self.accel_scale for x in a]
                g = [x * self.gyro_scale for x in g]
                m = [x * self.mag_scale for x in m]

                stamp = self.get_clock().now().to_msg()

                imu = Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = self.frame_id

                # No orientation estimate at this stage
                imu.orientation_covariance[0] = -1.0

                imu.linear_acceleration.x = a[0]
                imu.linear_acceleration.y = a[1]
                imu.linear_acceleration.z = a[2]

                imu.angular_velocity.x = g[0]
                imu.angular_velocity.y = g[1]
                imu.angular_velocity.z = g[2]

                # Diagonal covariance placeholders
                for i in range(9):
                    imu.linear_acceleration_covariance[i] = 0.0
                    imu.angular_velocity_covariance[i] = 0.0
                imu.linear_acceleration_covariance[0] = self.lin_acc_cov
                imu.linear_acceleration_covariance[4] = self.lin_acc_cov
                imu.linear_acceleration_covariance[8] = self.lin_acc_cov
                imu.angular_velocity_covariance[0] = self.ang_vel_cov
                imu.angular_velocity_covariance[4] = self.ang_vel_cov
                imu.angular_velocity_covariance[8] = self.ang_vel_cov

                mag = MagneticField()
                mag.header.stamp = stamp
                mag.header.frame_id = self.frame_id
                mag.magnetic_field.x = m[0]
                mag.magnetic_field.y = m[1]
                mag.magnetic_field.z = m[2]
                for i in range(9):
                    mag.magnetic_field_covariance[i] = 0.0
                mag.magnetic_field_covariance[0] = self.mag_cov
                mag.magnetic_field_covariance[4] = self.mag_cov
                mag.magnetic_field_covariance[8] = self.mag_cov

                self.imu_pub.publish(imu)
                self.mag_pub.publish(mag)
                self.good += 1

            # Stats log
            t = time.time()
            if t - self.last_stat_t > 2.0:
                self.get_logger().info(f"Published: good={self.good} bad={self.bad}")
                self.last_stat_t = t

        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Serial error: {e} (closing + retry)")
            self._close()
            self.next_reconnect_time = time.time() + self.reconnect_s
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main():
    rclpy.init()
    node = SerialImuBridge()
    try:
        rclpy.spin(node)
    finally:
        node._close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

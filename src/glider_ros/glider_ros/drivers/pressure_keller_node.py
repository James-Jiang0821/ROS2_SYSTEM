#!/usr/bin/env python3

import math
import struct
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float64, String

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    SMBus = None
    i2c_msg = None


class KellerLD:
    """
    Python port of the Blue Robotics Arduino Keller LD logic.

    Based on the Keller 4LD-series behavior used by Blue Robotics BarXT/Bar100-style sensors:
    - I2C address: 0x40
    - Conversion command: 0xAC
    - Reads calibration from memory map
    """

    LD_REQUEST = 0xAC
    LD_CUST_ID0 = 0x00
    LD_CUST_ID1 = 0x01
    LD_SCALING0 = 0x12
    LD_SCALING1 = 0x13
    LD_SCALING2 = 0x14
    LD_SCALING3 = 0x15
    LD_SCALING4 = 0x16

    def __init__(self, bus: SMBus, address: int = 0x40, fluid_density: float = 1029.0):
        self.bus = bus
        self.address = address
        self.fluid_density = fluid_density

        self.equipment: Optional[int] = None
        self.place: Optional[int] = None
        self.file: Optional[int] = None

        self.mode: Optional[int] = None
        self.year: Optional[int] = None
        self.month: Optional[int] = None
        self.day: Optional[int] = None

        self.code: Optional[int] = None

        self.P_mode: float = 0.0
        self.P_min: Optional[float] = None
        self.P_max: Optional[float] = None

        self.P_raw: Optional[int] = None
        self.P_bar: Optional[float] = None
        self.T_degc: Optional[float] = None

    def set_fluid_density(self, density: float) -> None:
        self.fluid_density = density

    def _write_byte(self, value: int) -> None:
        self.bus.write_byte(self.address, value)

    def _read_block(self, length: int) -> list[int]:
        return self.bus.read_i2c_block_data(self.address, 0x00, length)

    from smbus2 import i2c_msg

    def read_memory_map(self, mtp_address):

        # Write the memory address
        write = i2c_msg.write(self.address, [mtp_address])
        self.bus.i2c_rdwr(write)

        time.sleep(0.001)

        # Read 3 bytes
        read = i2c_msg.read(self.address, 3)
        self.bus.i2c_rdwr(read)

        data = list(read)

        status = data[0]
        value = (data[1] << 8) | data[2]

        return value

    @staticmethod
    def _u32_to_float_be(value: int) -> float:
        return struct.unpack(">f", struct.pack(">I", value))[0]

    def init(self) -> None:
        cust_id0 = self.read_memory_map(self.LD_CUST_ID0)
        cust_id1 = self.read_memory_map(self.LD_CUST_ID1)

        self.code = (cust_id1 << 16) | cust_id0
        self.equipment = cust_id0 >> 10
        self.place = cust_id0 & 0b000000111111111
        self.file = cust_id1

        scaling0 = self.read_memory_map(self.LD_SCALING0)

        self.mode = scaling0 & 0b00000011
        self.year = scaling0 >> 11
        self.month = (scaling0 & 0b0000011110000000) >> 7
        self.day = (scaling0 & 0b0000000001111100) >> 2

        # Same mode handling as Arduino library
        if self.mode == 0:
            # PR mode, vented gauge
            self.P_mode = 1.01325
        elif self.mode == 1:
            # PA mode, sealed gauge
            self.P_mode = 1.0
        else:
            # PAA mode, absolute
            self.P_mode = 0.0

        scaling12 = (self.read_memory_map(self.LD_SCALING1) << 16) | self.read_memory_map(self.LD_SCALING2)
        scaling34 = (self.read_memory_map(self.LD_SCALING3) << 16) | self.read_memory_map(self.LD_SCALING4)

        self.P_min = self._u32_to_float_be(scaling12)
        self.P_max = self._u32_to_float_be(scaling34)

    def is_initialized(self) -> bool:
        return (
            self.equipment is not None
            and self.equipment != 63
            and self.P_min is not None
            and self.P_max is not None
        )

    def read(self) -> Tuple[float, float, float]:
        """
        Returns:
            pressure_pa, temperature_c, depth_m
        """
        if not self.is_initialized():
            raise RuntimeError("Sensor not initialized")

        # Send conversion request
        self._write_byte(self.LD_REQUEST)

        # Arduino code uses 9 ms max conversion delay
        time.sleep(0.009)

        data = self._read_block(5)
        if len(data) != 5:
            raise RuntimeError("Failed to read sensor conversion block")

        _status = data[0]
        self.P_raw = (data[1] << 8) | data[2]
        raw_t = (data[3] << 8) | data[4]

        self.P_bar = ((float(self.P_raw) - 16384.0) * (self.P_max - self.P_min) / 32768.0) + self.P_min + self.P_mode
        self.T_degc = (((raw_t >> 4) - 24) * 0.05) - 50.0

        pressure_mbar = self.P_bar * 1000.0
        pressure_pa = pressure_mbar * 100.0
        depth_m = (pressure_pa - 101325.0) / (self.fluid_density * 9.80665)

        return pressure_pa, self.T_degc, depth_m


class KellerPressureNode(Node):
    def __init__(self):
        super().__init__("pressure_keller_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("fluid_density", 1029.0)   # seawater
        self.declare_parameter("frame_id", "pressure_link")

        self.i2c_bus_num = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.fluid_density = float(self.get_parameter("fluid_density").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.pressure_pub = self.create_publisher(FluidPressure, "/pressure/raw_pressure", 10)
        self.temperature_pub = self.create_publisher(Temperature, "/pressure/temperature", 10)
        self.depth_pub = self.create_publisher(Float64, "/pressure/depth", 10)
        self.status_pub = self.create_publisher(String, "/pressure/status", 10)

        self.bus: Optional[SMBus] = None
        self.sensor: Optional[KellerLD] = None

        self._connect_sensor()

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(period, self.timer_callback)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _connect_sensor(self) -> None:
        if SMBus is None:
            err = "python package 'smbus2' is not installed"
            self.get_logger().error(err)
            self.publish_status(err)
            return

        try:
            self.bus = SMBus(self.i2c_bus_num)
            self.sensor = KellerLD(
                bus=self.bus,
                address=self.i2c_address,
                fluid_density=self.fluid_density,
            )
            self.sensor.init()

            if not self.sensor.is_initialized():
                raise RuntimeError("sensor detected but initialization data invalid")

            info = (
                f"Keller sensor initialized on i2c bus {self.i2c_bus_num}, "
                f"address 0x{self.i2c_address:02X}, "
                f"mode={self.sensor.mode}, "
                f"Pmin={self.sensor.P_min:.3f} bar, "
                f"Pmax={self.sensor.P_max:.3f} bar"
            )
            self.get_logger().info(info)
            self.publish_status(info)

        except Exception as exc:
            self.sensor = None
            self.get_logger().error(f"Failed to initialize pressure sensor: {exc}")
            self.publish_status(f"init_failed: {exc}")

    def timer_callback(self) -> None:
        if self.sensor is None:
            return

        try:
            pressure_pa, temp_c, depth_m = self.sensor.read()

            now = self.get_clock().now().to_msg()

            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = now
            pressure_msg.header.frame_id = self.frame_id
            pressure_msg.fluid_pressure = pressure_pa
            pressure_msg.variance = 0.0

            temp_msg = Temperature()
            temp_msg.header.stamp = now
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = temp_c
            temp_msg.variance = 0.0

            depth_msg = Float64()
            depth_msg.data = depth_m

            self.pressure_pub.publish(pressure_msg)
            self.temperature_pub.publish(temp_msg)
            self.depth_pub.publish(depth_msg)

        except Exception as exc:
            self.get_logger().error(f"Pressure read failed: {exc}")
            self.publish_status(f"read_failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = KellerPressureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.bus is not None:
            try:
                node.bus.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
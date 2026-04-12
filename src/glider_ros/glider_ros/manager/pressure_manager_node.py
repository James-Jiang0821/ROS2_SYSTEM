#!/usr/bin/env python3
"""
Pressure Manager Node

Computes calibrated depth from raw pressure using h = (P - P_atm) / (rho * g).
All other pressure/temperature topics are left on their raw driver topics for
logging only — this node's sole output is /glider/depth.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64


class PressureManagerNode(Node):
    def __init__(self):
        super().__init__("pressure_manager_node")

        self.declare_parameter("fluid_density", 1000.0)   # kg/m³ — fresh water default
        self.declare_parameter("gravity", 9.80665)         # m/s²
        self.declare_parameter("atm_pressure_pa", 101325.0)  # Pa — standard atmosphere

        self._rho = float(self.get_parameter("fluid_density").value)
        self._g = float(self.get_parameter("gravity").value)
        self._p_atm = float(self.get_parameter("atm_pressure_pa").value)

        self.create_subscription(FluidPressure, "/pressure/raw_pressure", self._on_pressure, 10)
        self._depth_pub = self.create_publisher(Float64, "/glider/depth", 10)

        self.get_logger().info(
            f"Pressure manager ready — rho={self._rho} kg/m³, "
            f"g={self._g} m/s², P_atm={self._p_atm} Pa"
        )

    def _on_pressure(self, msg: FluidPressure) -> None:
        depth_m = (msg.fluid_pressure - self._p_atm) / (self._rho * self._g)
        out = Float64()
        out.data = max(0.0, depth_m)   # clamp negatives at surface
        self._depth_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PressureManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

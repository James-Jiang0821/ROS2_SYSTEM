#!/usr/bin/env python3
import re
import random
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IridiumCsqNode(Node):
    def __init__(self):
        super().__init__("iridium_csq_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyAMA5")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("base_period", 60.0)   # seconds
        self.declare_parameter("jitter", 10.0)        # +/- seconds

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.base_period = float(self.get_parameter("base_period").value)
        self.jitter = float(self.get_parameter("jitter").value)

        # Publishers
        self.csq_pub = self.create_publisher(String, "/iridium/signal_strength", 10)
        self.status_pub = self.create_publisher(String, "/iridium/status", 10)

        # Serial connection
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=5,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.get_logger().info(f"Opened {self.port} at {self.baud} baud")
            self.publish_string(self.status_pub, f"Opened {self.port} at {self.baud} baud")
        except Exception as e:
            self.ser = None
            err = f"Failed to open serial port: {e}"
            self.get_logger().error(err)
            self.publish_string(self.status_pub, err)

        # One-shot timer pattern so each interval can have fresh jitter
        self.timer = None
        self.schedule_next_poll(initial_delay=2.0)

    def publish_string(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def send_at(self, cmd: str, read_bytes: int = 256) -> str:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write((cmd + "\r").encode())
        self.ser.flush()
        response = self.ser.read(read_bytes).decode(errors="ignore")
        return response

    def extract_csq(self, response: str):
        match = re.search(r"\+CSQ:\s*(\d+)", response)
        if match:
            return int(match.group(1))
        return None

    def schedule_next_poll(self, initial_delay=None):
        if self.timer is not None:
            self.timer.cancel()

        if initial_delay is not None:
            delay = initial_delay
        else:
            delay = self.base_period + random.uniform(-self.jitter, self.jitter)
            delay = max(1.0, delay)  # avoid zero/negative times

        self.get_logger().info(f"Next CSQ poll in {delay:.1f} s")
        self.timer = self.create_timer(delay, self.timer_callback)

    def timer_callback(self):
        # Make this one-shot
        if self.timer is not None:
            self.timer.cancel()

        self.poll_csq()

        # Schedule next jittered poll
        self.schedule_next_poll()

    def poll_csq(self):
        try:
            # Optional modem alive check
            at_resp = self.send_at("AT")
            self.get_logger().info(f"AT response: {repr(at_resp)}")

            csq_resp = self.send_at("AT+CSQ")
            self.get_logger().info(f"CSQ response: {repr(csq_resp)}")

            csq = self.extract_csq(csq_resp)
            if csq is not None:
                self.publish_string(self.csq_pub, str(csq))
                self.publish_string(self.status_pub, f"CSQ={csq}")
                self.get_logger().info(f"Published CSQ={csq}")
            else:
                self.publish_string(self.status_pub, "Could not parse CSQ")
                self.get_logger().warn("Could not parse CSQ response")

        except Exception as e:
            err = f"Iridium CSQ poll error: {e}"
            self.get_logger().error(err)
            self.publish_string(self.status_pub, err)

    def destroy_node(self):
        try:
            if self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IridiumCsqNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
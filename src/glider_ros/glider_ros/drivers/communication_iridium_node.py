#!/usr/bin/env python3
import re
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IridiumBasicNode(Node):

    def __init__(self):
        super().__init__("communication_iridium_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyAMA2")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("poll_period", 90.0)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        poll_period = float(self.get_parameter("poll_period").value)

        # Publishers
        self.status_pub = self.create_publisher(String, "/iridium/status", 10)
        self.csq_pub = self.create_publisher(String, "/iridium/signal_strength", 10)
        self.mt_pub = self.create_publisher(String, "/iridium/incoming_message", 10)

        # Subscriber for outbound telemetry
        self.sbdwt_sub = self.create_subscription(
            String,
            "/iridium/sbdwt",
            self.sbdwt_callback,
            10
        )

        # Cache latest telemetry payload
        self.latest_sbdwt_message = None

        # Serial connection
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=10,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.get_logger().info(f"Opened {port} at {baud} baud")

        # Timer
        self.timer = self.create_timer(poll_period, self.poll_modem)


    def sbdwt_callback(self, msg: String):
        """Store latest telemetry message"""
        self.latest_sbdwt_message = msg.data
        self.get_logger().info(f"Received new SBDWT payload: {msg.data}")


    def publish_string(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)


    def send_at(self, cmd: str, read_bytes: int = 512) -> str:
        """
        Send AT command ending with CR and read response.
        Modem echoes commands.
        """
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r").encode())
        self.ser.flush()
        response = self.ser.read(read_bytes).decode(errors="ignore")
        return response


    def extract_csq(self, response: str):
        match = re.search(r"\+CSQ:\s*(\d+)", response)
        if match:
            return int(match.group(1))
        return None


    def mt_message_present(self, response: str) -> bool:
        """
        Parse +SBDIX line.
        Format:
        +SBDIX: mo_status, momsn, mt_status, mtmsn, mt_len, mt_queued
        """
        match = re.search(r"\+SBDIX:\s*([0-9,\s-]+)", response)
        if not match:
            return False

        try:
            nums = [int(x.strip()) for x in match.group(1).split(",")]
            if len(nums) >= 6:
                mt_len = nums[4]
                mt_queued = nums[5]
                return (mt_len > 0) or (mt_queued > 0)
        except Exception:
            pass

        return False


    def clean_mt_response(self, response: str) -> str:
        """
        Remove echoed command and trailing OK.
        """
        lines = []
        for line in response.splitlines():
            line = line.strip()
            if not line:
                continue
            if line == "AT+SBDRT":
                continue
            if line == "OK":
                continue
            lines.append(line)

        return "\n".join(lines).strip()


    def poll_modem(self):

        try:

            # 1) Check modem alive
            at_resp = self.send_at("AT")
            self.get_logger().info(f"AT response: {repr(at_resp)}")
            self.publish_string(self.status_pub, "AT check complete")

            # 2) Check signal strength
            csq_resp = self.send_at("AT+CSQ")
            self.get_logger().info(f"CSQ response: {repr(csq_resp)}")

            csq = self.extract_csq(csq_resp)

            if csq is not None:
                self.publish_string(self.csq_pub, str(csq))
                self.publish_string(self.status_pub, f"CSQ={csq}")
            else:
                self.publish_string(self.status_pub, "Could not parse CSQ")

            # 3) Run SBD session
            sbdix_resp = self.send_at("AT+SBDIX", read_bytes=1024)
            self.get_logger().info(f"SBDIX response: {repr(sbdix_resp)}")
            self.publish_string(self.status_pub, "Ran SBDIX session")

            # 4) Check for incoming MT message
            if self.mt_message_present(sbdix_resp):

                sbrt_resp = self.send_at("AT+SBDRT", read_bytes=1024)
                self.get_logger().info(f"SBDRT response: {repr(sbrt_resp)}")

                mt_text = self.clean_mt_response(sbrt_resp)

                if mt_text:

                    self.publish_string(self.mt_pub, mt_text)
                    self.publish_string(self.status_pub, f"Received MT: {mt_text}")
                    self.get_logger().info(f"Received MT message: {mt_text}")

                    # 5) Send telemetry reply
                    if self.latest_sbdwt_message is not None:

                        payload = self.latest_sbdwt_message.strip()

                        # Limit payload length for safety
                        if len(payload) > 200:
                            payload = payload[:200]

                    else:

                        payload = "NO_DATA"
                        self.get_logger().warn("No telemetry available for SBDWT")

                    cmd = f"AT+SBDWT={payload}"

                    self.get_logger().info(f"Sending telemetry via SBDWT: {payload}")

                    sbdwt_resp = self.send_at(cmd)
                    self.get_logger().info(f"SBDWT response: {repr(sbdwt_resp)}")

                    self.publish_string(self.status_pub, f"Sent MO telemetry: {payload}")

                    # Send message
                    reply_resp = self.send_at("AT+SBDIX", read_bytes=1024)
                    self.get_logger().info(f"Reply SBDIX response: {repr(reply_resp)}")

                else:

                    self.publish_string(self.status_pub, "MT indicated but message empty")

            else:

                self.publish_string(self.status_pub, "No MT message waiting")

        except Exception as e:

            error_text = f"Iridium node error: {str(e)}"
            self.get_logger().error(error_text)
            self.publish_string(self.status_pub, error_text)


    def destroy_node(self):

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = IridiumBasicNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
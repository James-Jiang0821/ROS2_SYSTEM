#!/usr/bin/env python3
import random
import re
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


@dataclass
class SbdixResult:
    mo_status: int
    momsn: int
    mt_status: int
    mtmsn: int
    mt_len: int
    mt_queued: int


@dataclass
class SbdsxResult:
    mo_flag: int
    momsn: int
    mt_flag: int
    mtmsn: int
    ra_flag: int
    msg_waiting: int


class IridiumState(Enum):
    IDLE = "IDLE"
    SETTLING = "SETTLING"
    ATTEMPTING = "ATTEMPTING"
    BACKOFF = "BACKOFF"
    DONE = "DONE"


class IridiumSbdNode(Node):
    def __init__(self):
        super().__init__("communication_iridium_node")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter("port", "/dev/ttyAMA2")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("serial_timeout", 0.5)

        # Timer-driven comms window
        self.declare_parameter("cycle_period", 300.0)          # seconds between windows
        self.declare_parameter("start_immediately", True)
        self.declare_parameter("settle_time", 60.0)            # wait before first attempt
        self.declare_parameter("retry_delays", [60.0, 90.0, 120.0])
        self.declare_parameter("retry_jitter", 3.0)

        # Modem/session
        self.declare_parameter("min_csq", 2)
        self.declare_parameter("sbd_session_timeout", 60)
        self.declare_parameter("sbd_text_max_len", 340)

        # Logging/behavior
        self.declare_parameter("debug", False)
        self.declare_parameter("tick_period", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)

        self.cycle_period = float(self.get_parameter("cycle_period").value)
        self.start_immediately = bool(self.get_parameter("start_immediately").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.retry_delays = [float(x) for x in self.get_parameter("retry_delays").value]
        self.retry_jitter = float(self.get_parameter("retry_jitter").value)

        self.min_csq = int(self.get_parameter("min_csq").value)
        self.sbd_session_timeout = int(self.get_parameter("sbd_session_timeout").value)
        self.sbd_text_max_len = int(self.get_parameter("sbd_text_max_len").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.tick_period = float(self.get_parameter("tick_period").value)

        # ----------------------------
        # Publishers
        # ----------------------------
        self.status_pub = self.create_publisher(String, "/iridium/status", 10)
        self.csq_pub = self.create_publisher(String, "/iridium/signal_strength", 10)
        self.mt_pub = self.create_publisher(String, "/iridium/incoming_message", 10)

        self.session_result_pub = self.create_publisher(String, "/iridium/session_result", 10)
        self.mo_status_pub = self.create_publisher(String, "/iridium/mo_status", 10)
        self.mt_waiting_pub = self.create_publisher(String, "/iridium/mt_waiting", 10)

        # ----------------------------
        # Subscriber
        # ----------------------------
        self.sbdwt_sub = self.create_subscription(
            String,
            "/iridium/sbdwt",
            self.sbdwt_callback,
            10
        )

        # ----------------------------
        # Outbound message state
        # ----------------------------
        self.pending_outbound: Optional[str] = None
        self.pending_outbound_dirty: bool = False
        self.last_written_outbound: Optional[str] = None
        self.last_successful_outbound: Optional[str] = None

        # ----------------------------
        # Comms window state
        # ----------------------------
        now = time.monotonic()
        self.state = IridiumState.IDLE
        self.window_active = False
        self.window_settle_deadline = 0.0
        self.backoff_deadline = 0.0
        self.next_cycle_time = now if self.start_immediately else now + self.cycle_period
        self.attempt_index = 0
        self.last_csq: Optional[int] = None

        # ----------------------------
        # Serial setup
        # ----------------------------
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.serial_timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.publish_status(f"Opened {self.port} at {self.baud} baud")
        self.log_debug(f"Serial connection opened on {self.port}")

        # Configure modem once at startup
        self.configure_modem()

        # Main state-machine tick
        self.timer = self.create_timer(self.tick_period, self.tick)

    # ------------------------------------------------------------------
    # ROS helpers
    # ------------------------------------------------------------------
    def publish_string(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def publish_status(self, text: str):
        self.publish_string(self.status_pub, text)
        if self.debug:
            self.get_logger().info(text)

    def log_debug(self, text: str):
        if self.debug:
            self.get_logger().info(text)

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------
    def sbdwt_callback(self, msg: String):
        clean = self.sanitize_payload(msg.data)
        self.pending_outbound = clean
        self.pending_outbound_dirty = True
        self.log_debug(f"Updated pending outbound payload: {clean}")

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()

        if self.state == IridiumState.IDLE:
            if now >= self.next_cycle_time:
                self.start_comms_window()
            return

        if self.state == IridiumState.SETTLING:
            if now >= self.window_settle_deadline:
                self.state = IridiumState.ATTEMPTING
                self.publish_status("Iridium settling complete, starting attempt")
            return

        if self.state == IridiumState.ATTEMPTING:
            self.perform_attempt()
            return

        if self.state == IridiumState.BACKOFF:
            if now >= self.backoff_deadline:
                self.state = IridiumState.ATTEMPTING
                self.publish_status("Iridium backoff complete, retrying")
            return

        if self.state == IridiumState.DONE:
            if now >= self.next_cycle_time:
                self.state = IridiumState.IDLE
            return

    def start_comms_window(self):
        self.window_active = True
        self.attempt_index = 0
        self.window_settle_deadline = time.monotonic() + self.settle_time
        self.state = IridiumState.SETTLING
        self.publish_status(
            f"Starting Iridium comms window; settling for {self.settle_time:.0f}s"
        )

    def finish_comms_window(self, reason: str):
        self.window_active = False
        self.state = IridiumState.DONE
        self.next_cycle_time = time.monotonic() + self.cycle_period
        self.publish_status(f"Ending comms window: {reason}")

    # ------------------------------------------------------------------
    # Attempt logic
    # ------------------------------------------------------------------
    def perform_attempt(self):
        try:
            self.attempt_index += 1
            self.publish_status(f"Iridium attempt {self.attempt_index}")

            # Check modem alive
            at_resp = self.send_command("AT", timeout_s=5.0)
            if "OK" not in at_resp:
                raise RuntimeError(f"AT failed: {repr(at_resp)}")

            # Query signal
            csq = self.get_csq()
            self.last_csq = csq
            if csq is not None:
                self.publish_string(self.csq_pub, str(csq))
                self.publish_status(f"CSQ={csq}")
            else:
                self.publish_status("Could not parse CSQ")

            # Publish modem buffer/mailbox status if available
            sbdsx = self.get_sbdsx()
            if sbdsx is not None:
                self.publish_string(
                    self.mt_waiting_pub,
                    f"mt_flag={sbdsx.mt_flag},msg_waiting={sbdsx.msg_waiting}"
                )

            # If signal is below desired threshold, skip this attempt
            if csq is not None and csq < self.min_csq:
                self.publish_status(
                    f"CSQ below threshold ({csq} < {self.min_csq}), not attempting session"
                )
                self.handle_failed_attempt("low_csq")
                return

            # Always use the latest payload if present
            if self.pending_outbound:
                self.write_outbound_payload(self.pending_outbound)

            # Run SBD session:
            # - sends MO if MO buffer has data
            # - checks mailbox for MT
            sbdix = self.run_sbdix()
            if sbdix is None:
                self.handle_failed_attempt("could_not_parse_sbdix")
                return

            # Publish detailed session result
            session_summary = (
                f"mo_status={sbdix.mo_status},momsn={sbdix.momsn},"
                f"mt_status={sbdix.mt_status},mtmsn={sbdix.mtmsn},"
                f"mt_len={sbdix.mt_len},mt_queued={sbdix.mt_queued}"
            )
            self.publish_string(self.session_result_pub, session_summary)
            self.publish_string(self.mo_status_pub, str(sbdix.mo_status))
            self.publish_string(self.mt_waiting_pub, str(sbdix.mt_queued))
            self.log_debug(f"SBDIX parsed: {session_summary}")

            # If MT message arrived, read and publish it
            if sbdix.mt_len > 0:
                mt_text = self.read_mt_message()
                if mt_text:
                    self.publish_string(self.mt_pub, mt_text)
                    self.publish_status(f"Received MT: {mt_text}")
                else:
                    self.publish_status("MT indicated by SBDIX, but MT buffer read empty")

            # Success policy:
            # Treat mo_status 0..4 as success range for a completed session.
            # This is a practical success gate for mailbox checks and normal MO sessions.
            if self.is_session_success(sbdix.mo_status):
                if self.pending_outbound:
                    self.last_successful_outbound = self.pending_outbound
                    self.pending_outbound_dirty = False

                self.finish_comms_window("session_success")
                return

            # Otherwise failed
            self.handle_failed_attempt(f"mo_status_{sbdix.mo_status}")

        except Exception as e:
            error_text = f"Iridium attempt error: {str(e)}"
            self.get_logger().error(error_text)
            self.publish_status(error_text)
            self.handle_failed_attempt("exception")

    def handle_failed_attempt(self, reason: str):
        if self.attempt_index >= len(self.retry_delays):
            self.finish_comms_window(f"attempts_exhausted ({reason})")
            return

        base_delay = self.retry_delays[self.attempt_index - 1]
        jitter = random.uniform(-self.retry_jitter, self.retry_jitter)
        delay = max(1.0, base_delay + jitter)

        self.backoff_deadline = time.monotonic() + delay
        self.state = IridiumState.BACKOFF
        self.publish_status(
            f"Attempt failed ({reason}), retrying in {delay:.1f}s"
        )

    # ------------------------------------------------------------------
    # Modem configuration
    # ------------------------------------------------------------------
    def configure_modem(self):
        try:
            # Basic liveness
            self.send_command("AT", timeout_s=5.0)

            # Disable echo for cleaner parsing
            self.send_command("ATE0", timeout_s=5.0)

            # Set SBD session timeout
            self.send_command(f"AT+SBDST={self.sbd_session_timeout}", timeout_s=5.0)

            self.publish_status("Iridium modem configured")
        except Exception as e:
            error_text = f"Failed to configure modem: {str(e)}"
            self.get_logger().error(error_text)
            self.publish_status(error_text)

    # ------------------------------------------------------------------
    # Serial / AT helpers
    # ------------------------------------------------------------------
    def send_command(
        self,
        cmd: str,
        timeout_s: float = 10.0,
        end_tokens=("OK", "ERROR", "READY")
    ) -> str:
        """
        Send an AT command and read until an expected terminator appears
        or timeout expires.
        """
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        wire = (cmd + "\r").encode("ascii", errors="ignore")
        self.ser.write(wire)
        self.ser.flush()

        deadline = time.monotonic() + timeout_s
        response = b""

        while time.monotonic() < deadline:
            chunk = self.ser.read(256)
            if chunk:
                response += chunk
                text = response.decode(errors="ignore")
                if any(token in text for token in end_tokens):
                    return text
            else:
                time.sleep(0.05)

        return response.decode(errors="ignore")

    def sanitize_payload(self, payload: str) -> str:
        text = payload.replace("\r", " ").replace("\n", " ").strip()
        if len(text) > self.sbd_text_max_len:
            text = text[:self.sbd_text_max_len]
        return text

    # ------------------------------------------------------------------
    # AT command wrappers
    # ------------------------------------------------------------------
    def get_csq(self) -> Optional[int]:
        resp = self.send_command("AT+CSQ", timeout_s=10.0)
        self.log_debug(f"CSQ response: {repr(resp)}")
        match = re.search(r"\+CSQ:\s*(\d+)", resp)
        if match:
            return int(match.group(1))
        return None

    def get_sbdsx(self) -> Optional[SbdsxResult]:
        resp = self.send_command("AT+SBDSX", timeout_s=10.0)
        self.log_debug(f"SBDSX response: {repr(resp)}")
        match = re.search(r"\+SBDSX:\s*([0-9,\s-]+)", resp)
        if not match:
            return None

        try:
            nums = [int(x.strip()) for x in match.group(1).split(",")]
            if len(nums) >= 6:
                return SbdsxResult(
                    mo_flag=nums[0],
                    momsn=nums[1],
                    mt_flag=nums[2],
                    mtmsn=nums[3],
                    ra_flag=nums[4],
                    msg_waiting=nums[5],
                )
        except Exception:
            return None
        return None

    def write_outbound_payload(self, payload: str):
        clean = self.sanitize_payload(payload)
        if not clean:
            self.publish_status("Outbound payload empty, skipping SBDWT")
            return

        # Only rewrite if it changed or was never written
        if clean == self.last_written_outbound and not self.pending_outbound_dirty:
            self.log_debug("Outbound payload unchanged, not rewriting MO buffer")
            return

        resp = self.send_command(f"AT+SBDWT={clean}", timeout_s=10.0)
        self.log_debug(f"SBDWT response: {repr(resp)}")

        if "OK" not in resp:
            raise RuntimeError(f"SBDWT failed: {repr(resp)}")

        self.last_written_outbound = clean
        self.publish_status(f"Loaded MO payload: {clean}")

    def run_sbdix(self) -> Optional[SbdixResult]:
        # Give it enough time for the session plus some margin
        timeout_s = float(self.sbd_session_timeout + 20)
        resp = self.send_command("AT+SBDIX", timeout_s=timeout_s)
        self.log_debug(f"SBDIX response: {repr(resp)}")
        return self.parse_sbdix(resp)

    def parse_sbdix(self, response: str) -> Optional[SbdixResult]:
        match = re.search(r"\+SBDIX:\s*([0-9,\s-]+)", response)
        if not match:
            return None

        try:
            nums = [int(x.strip()) for x in match.group(1).split(",")]
            if len(nums) >= 6:
                return SbdixResult(
                    mo_status=nums[0],
                    momsn=nums[1],
                    mt_status=nums[2],
                    mtmsn=nums[3],
                    mt_len=nums[4],
                    mt_queued=nums[5],
                )
        except Exception:
            return None
        return None

    def read_mt_message(self) -> str:
        resp = self.send_command("AT+SBDRT", timeout_s=10.0)
        self.log_debug(f"SBDRT response: {repr(resp)}")
        return self.clean_sbd_text_response(resp)

    def clean_sbd_text_response(self, response: str) -> str:
        lines = []
        for line in response.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            if stripped == "OK":
                continue
            if stripped == "ERROR":
                continue
            if stripped == "AT+SBDRT":
                continue
            lines.append(stripped)
        return "\n".join(lines).strip()

    def is_session_success(self, mo_status: int) -> bool:
        """
        Practical success rule for v1:
        treat low MO status codes as completed session outcomes.
        """
        return 0 <= mo_status <= 4

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IridiumSbdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
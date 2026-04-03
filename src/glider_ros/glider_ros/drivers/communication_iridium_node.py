#!/usr/bin/env python3
import random
import re
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
import rclpy.executors
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from std_msgs.msg import String
import serial

from glider_msgs.action import IridiumWindow


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


class IridiumSbdNode(Node):
    def __init__(self):
        super().__init__("communication_iridium_node")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter("port", "/dev/ttyAMA2")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("serial_timeout", 0.5)
        self.declare_parameter("settle_time", 60.0)
        self.declare_parameter("retry_delays", [60.0, 90.0, 120.0])
        self.declare_parameter("retry_jitter", 3.0)
        self.declare_parameter("min_csq", 2)
        self.declare_parameter("sbd_session_timeout", 60)
        self.declare_parameter("sbd_text_max_len", 340)
        self.declare_parameter("debug", False)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.retry_delays = [float(x) for x in self.get_parameter("retry_delays").value]
        self.retry_jitter = float(self.get_parameter("retry_jitter").value)
        self.min_csq = int(self.get_parameter("min_csq").value)
        self.sbd_session_timeout = int(self.get_parameter("sbd_session_timeout").value)
        self.sbd_text_max_len = int(self.get_parameter("sbd_text_max_len").value)
        self.debug = bool(self.get_parameter("debug").value)

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
        # Subscriber — telemetry payload from telemetry_manager_node
        # ----------------------------
        self.sbdwt_sub = self.create_subscription(
            String,
            "/iridium/sbdwt",
            self.sbdwt_callback,
            10
        )

        # ----------------------------
        # Outbound payload state (lock protects access from sbdwt_callback
        # thread vs action execute thread)
        # ----------------------------
        self._lock = threading.Lock()
        self.pending_outbound: Optional[str] = None
        self.pending_outbound_dirty: bool = False
        self.last_written_outbound: Optional[str] = None
        self.last_successful_outbound: Optional[str] = None

        # Prevent concurrent windows
        self._window_in_progress = False

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

        # ----------------------------
        # Action server
        # ----------------------------
        self._action_server = ActionServer(
            self,
            IridiumWindow,
            "/iridium/run_window",
            execute_callback=self.execute_window_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Iridium node ready, waiting for action goals")

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
        with self._lock:
            self.pending_outbound = clean
            self.pending_outbound_dirty = True
        self.log_debug(f"Updated pending outbound payload: {clean}")

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        if self._window_in_progress:
            self.get_logger().warn("Rejecting goal: comms window already in progress")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested for Iridium window")
        return CancelResponse.ACCEPT

    def execute_window_callback(self, goal_handle):
        self._window_in_progress = True
        try:
            return self._run_window(goal_handle)
        finally:
            self._window_in_progress = False

    # ------------------------------------------------------------------
    # Window execution (runs in action server thread)
    # ------------------------------------------------------------------
    def _run_window(self, goal_handle) -> IridiumWindow.Result:
        goal = goal_handle.request
        max_attempts = goal.max_attempts if goal.max_attempts > 0 else 3
        settling_time = goal.settling_time_s if goal.settling_time_s > 0.0 else self.settle_time

        # If the goal provides telemetry, use it as the outbound payload override
        if goal.latest_telemetry:
            with self._lock:
                self.pending_outbound = self.sanitize_payload(goal.latest_telemetry)
                self.pending_outbound_dirty = True

        feedback = IridiumWindow.Feedback()

        # ---- Settling phase ----
        feedback.phase = "SETTLING"
        feedback.attempt_number = 0
        goal_handle.publish_feedback(feedback)
        self.publish_status(f"Settling for {settling_time:.0f}s")

        deadline = time.monotonic() + settling_time
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return IridiumWindow.Result()
            time.sleep(1.0)

        # ---- Attempt loop ----
        window_success = False
        mission_received = False
        mission_text = ""
        attempts_used = 0
        status_message = "No attempts completed"

        for i in range(max_attempts):
            attempts_used = i + 1

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return IridiumWindow.Result()

            feedback.phase = f"ATTEMPT_{attempts_used}"
            feedback.attempt_number = attempts_used
            goal_handle.publish_feedback(feedback)
            self.publish_status(f"Iridium attempt {attempts_used}/{max_attempts}")

            attempt = self._run_one_attempt()
            status_message = attempt["status"]

            if attempt["success"]:
                window_success = True
                if attempt["mt_text"]:
                    mission_received = True
                    mission_text = attempt["mt_text"]
                break

            # Failed — backoff unless this was the last attempt
            if i < max_attempts - 1:
                idx = min(i, len(self.retry_delays) - 1)
                base = self.retry_delays[idx]
                delay = max(1.0, base + random.uniform(-self.retry_jitter, self.retry_jitter))
                self.publish_status(f"Attempt {attempts_used} failed ({status_message}), backoff {delay:.1f}s")

                feedback.phase = "BACKOFF"
                goal_handle.publish_feedback(feedback)

                deadline = time.monotonic() + delay
                while time.monotonic() < deadline:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return IridiumWindow.Result()
                    time.sleep(1.0)

        feedback.phase = "DONE"
        goal_handle.publish_feedback(feedback)
        self.publish_status(
            f"Window done — success={window_success}, mission={mission_received}, "
            f"attempts={attempts_used}"
        )

        result = IridiumWindow.Result()
        result.window_success = window_success
        result.mission_received = mission_received
        result.mission_text = mission_text
        result.attempts_used = attempts_used
        result.status_message = status_message

        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Single SBD attempt
    # ------------------------------------------------------------------
    def _run_one_attempt(self) -> dict:
        """
        Run one full SBD session attempt.
        Returns {'success': bool, 'mt_text': str, 'status': str}
        """
        try:
            # Check modem alive
            at_resp = self.send_command("AT", timeout_s=5.0)
            if "OK" not in at_resp:
                return {"success": False, "mt_text": "", "status": f"AT failed: {repr(at_resp)}"}

            # Signal strength
            csq = self.get_csq()
            if csq is not None:
                self.publish_string(self.csq_pub, str(csq))
                self.publish_status(f"CSQ={csq}")
            else:
                self.publish_status("Could not parse CSQ")

            # Mailbox status
            sbdsx = self.get_sbdsx()
            if sbdsx is not None:
                self.publish_string(
                    self.mt_waiting_pub,
                    f"mt_flag={sbdsx.mt_flag},msg_waiting={sbdsx.msg_waiting}"
                )

            # Signal threshold check
            if csq is not None and csq < self.min_csq:
                return {
                    "success": False,
                    "mt_text": "",
                    "status": f"CSQ below threshold ({csq}<{self.min_csq})",
                }

            # Write outbound payload if available
            with self._lock:
                outbound = self.pending_outbound
                dirty = self.pending_outbound_dirty

            if outbound:
                self.write_outbound_payload(outbound, dirty)

            # SBD session
            sbdix = self.run_sbdix()
            if sbdix is None:
                return {"success": False, "mt_text": "", "status": "Could not parse SBDIX response"}

            session_summary = (
                f"mo_status={sbdix.mo_status},momsn={sbdix.momsn},"
                f"mt_status={sbdix.mt_status},mtmsn={sbdix.mtmsn},"
                f"mt_len={sbdix.mt_len},mt_queued={sbdix.mt_queued}"
            )
            self.publish_string(self.session_result_pub, session_summary)
            self.publish_string(self.mo_status_pub, str(sbdix.mo_status))
            self.publish_string(self.mt_waiting_pub, str(sbdix.mt_queued))
            self.log_debug(f"SBDIX parsed: {session_summary}")

            # Read MT message if one arrived
            mt_text = ""
            if sbdix.mt_len > 0:
                mt_text = self.read_mt_message()
                if mt_text:
                    self.publish_string(self.mt_pub, mt_text)
                    self.publish_status(f"Received MT: {mt_text}")
                else:
                    self.publish_status("MT indicated by SBDIX but buffer read empty")

            if self.is_session_success(sbdix.mo_status):
                with self._lock:
                    if outbound:
                        self.last_successful_outbound = outbound
                        self.pending_outbound_dirty = False
                return {
                    "success": True,
                    "mt_text": mt_text,
                    "status": f"Session success (mo_status={sbdix.mo_status})",
                }

            return {
                "success": False,
                "mt_text": "",
                "status": f"Session failed (mo_status={sbdix.mo_status})",
            }

        except Exception as e:
            error = f"Attempt exception: {e}"
            self.get_logger().error(error)
            self.publish_status(error)
            return {"success": False, "mt_text": "", "status": error}

    # ------------------------------------------------------------------
    # Modem configuration
    # ------------------------------------------------------------------
    def configure_modem(self):
        try:
            self.send_command("AT", timeout_s=5.0)
            self.send_command("ATE0", timeout_s=5.0)
            self.send_command(f"AT+SBDST={self.sbd_session_timeout}", timeout_s=5.0)
            self.publish_status("Iridium modem configured")
        except Exception as e:
            error = f"Failed to configure modem: {e}"
            self.get_logger().error(error)
            self.publish_status(error)

    # ------------------------------------------------------------------
    # Serial / AT helpers
    # ------------------------------------------------------------------
    def send_command(
        self,
        cmd: str,
        timeout_s: float = 10.0,
        end_tokens=("OK", "ERROR", "READY"),
    ) -> str:
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
            text = text[: self.sbd_text_max_len]
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

    def write_outbound_payload(self, payload: str, dirty: bool = True):
        clean = self.sanitize_payload(payload)
        if not clean:
            self.publish_status("Outbound payload empty, skipping SBDWT")
            return
        if clean == self.last_written_outbound and not dirty:
            self.log_debug("Outbound payload unchanged, not rewriting MO buffer")
            return
        resp = self.send_command(f"AT+SBDWT={clean}", timeout_s=10.0)
        self.log_debug(f"SBDWT response: {repr(resp)}")
        if "OK" not in resp:
            raise RuntimeError(f"SBDWT failed: {repr(resp)}")
        self.last_written_outbound = clean
        self.publish_status(f"Loaded MO payload: {clean}")

    def run_sbdix(self) -> Optional[SbdixResult]:
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
            if stripped in ("", "OK", "ERROR", "AT+SBDRT"):
                continue
            lines.append(stripped)
        return "\n".join(lines).strip()

    def is_session_success(self, mo_status: int) -> bool:
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
    # MultiThreadedExecutor lets the action execute_callback run in its own
    # thread while the main executor continues processing other callbacks
    # (e.g. sbdwt_callback, publishers) during the long blocking window.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

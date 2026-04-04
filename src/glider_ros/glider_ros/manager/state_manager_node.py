#!/usr/bin/env python3

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from glider_msgs.action import IridiumWindow


class MissionState(Enum):
    IDLE = auto()
    INITIALISE = auto()
    OPERATION = auto()
    EMERGENCY = auto()


class IdlePhase(Enum):
    WAITING_FOR_TIMER = auto()
    ACTIVATING_IRIDIUM = auto()
    SENDING_WINDOW_GOAL = auto()
    WAITING_FOR_WINDOW_RESULT = auto()
    DEACTIVATING_IRIDIUM = auto()


class StateManagerNode(Node):
    def __init__(self):
        super().__init__("state_manager_node")

        # -----------------------------
        # Top-level mission state
        # -----------------------------
        self.state = MissionState.IDLE

        # -----------------------------
        # Idle sub-phase
        # -----------------------------
        self.idle_phase = IdlePhase.WAITING_FOR_TIMER

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("idle_iridium_period_s", 300.0)   # 5 min
        self.declare_parameter("iridium_settling_time_s", 60.0)
        self.declare_parameter("iridium_max_attempts", 3)

        self.idle_iridium_period_s = float(
            self.get_parameter("idle_iridium_period_s").value
        )
        self.iridium_settling_time_s = float(
            self.get_parameter("iridium_settling_time_s").value
        )
        self.iridium_max_attempts = int(
            self.get_parameter("iridium_max_attempts").value
        )

        # -----------------------------
        # Track timing
        # -----------------------------
        now_ns = self.get_clock().now().nanoseconds
        self.last_idle_cycle_start_ns = now_ns

        # -----------------------------
        # State variables
        # -----------------------------
        self.iridium_goal_in_flight = False
        self.pending_mission_text = ""
        self.emergency_triggered = False

        # Lifecycle transition future (shared between activate/deactivate calls)
        self._lc_future = None

        # -----------------------------
        # Publishers
        # -----------------------------
        self.state_pub = self.create_publisher(String, "/manager/state", 10)

        # -----------------------------
        # Safety subscription
        # -----------------------------
        self.create_subscription(
            Bool,
            "/safety/emergency",
            self.emergency_cb,
            10
        )

        # -----------------------------
        # Iridium action client
        # -----------------------------
        self.iridium_client = ActionClient(
            self,
            IridiumWindow,
            "/iridium/run_window"
        )

        # -----------------------------
        # Iridium lifecycle service client
        # -----------------------------
        self._lc_client = self.create_client(
            ChangeState,
            "/communication_iridium_node/change_state"
        )

        # -----------------------------
        # Main loop
        # -----------------------------
        self.timer = self.create_timer(0.2, self.step)

        self.get_logger().info("State manager started in IDLE")

    # =========================================================
    # Callbacks
    # =========================================================

    def emergency_cb(self, msg: Bool):
        if msg.data:
            self.emergency_triggered = True

    # =========================================================
    # Utility
    # =========================================================

    def transition_to(self, new_state: MissionState):
        if self.state != new_state:
            self.get_logger().info(f"STATE: {self.state.name} -> {new_state.name}")
            self.state = new_state

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def idle_timer_elapsed(self) -> bool:
        elapsed_s = (
            self.get_clock().now().nanoseconds - self.last_idle_cycle_start_ns
        ) / 1e9
        return elapsed_s >= self.idle_iridium_period_s

    def reset_idle_timer(self):
        self.last_idle_cycle_start_ns = self.get_clock().now().nanoseconds

    # =========================================================
    # Lifecycle helpers
    # =========================================================

    def _send_lc_transition(self, transition_id: int):
        req = ChangeState.Request()
        req.transition.id = transition_id
        return self._lc_client.call_async(req)

    def activate_iridium(self) -> Optional[bool]:
        """
        Calls the lifecycle 'configure' transition on communication_iridium_node,
        which opens the serial port and starts the action server.

        Returns:
            True  — transition completed successfully
            False — transition failed (or service unavailable)
            None  — still waiting for the response
        """
        if self._lc_future is None:
            if not self._lc_client.service_is_ready():
                self.get_logger().warn(
                    "Iridium lifecycle service not ready; will retry"
                )
                return False
            self.get_logger().info(
                "Sending lifecycle 'configure' to communication_iridium_node"
            )
            self._lc_future = self._send_lc_transition(Transition.TRANSITION_CONFIGURE)
            return None

        if not self._lc_future.done():
            return None

        result = self._lc_future.result()
        self._lc_future = None

        if result is None or not result.success:
            self.get_logger().warn("Iridium 'configure' transition failed")
            return False

        self.get_logger().info("Iridium node configured and ready")
        return True

    def deactivate_iridium(self) -> Optional[bool]:
        """
        Calls the lifecycle 'cleanup' transition on communication_iridium_node,
        which closes the serial port and stops the action server.

        Returns:
            True  — done (success or non-critical failure, always proceed)
            None  — still waiting for the response
        """
        if self._lc_future is None:
            if not self._lc_client.service_is_ready():
                self.get_logger().warn(
                    "Iridium lifecycle service not ready; skipping cleanup"
                )
                return True
            self.get_logger().info(
                "Sending lifecycle 'cleanup' to communication_iridium_node"
            )
            self._lc_future = self._send_lc_transition(Transition.TRANSITION_CLEANUP)
            return None

        if not self._lc_future.done():
            return None

        result = self._lc_future.result()
        self._lc_future = None

        if result is None or not result.success:
            self.get_logger().warn(
                "Iridium 'cleanup' transition failed; continuing anyway"
            )
        else:
            self.get_logger().info("Iridium node cleaned up")

        return True

    # =========================================================
    # Action helpers
    # =========================================================

    def send_iridium_window_goal(self):
        if self.iridium_goal_in_flight:
            return

        if not self.iridium_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("/iridium/run_window action server not available yet")
            return

        goal_msg = IridiumWindow.Goal()
        goal_msg.latest_telemetry = ""  # iridium node uses /iridium/sbdwt subscription
        goal_msg.max_attempts = self.iridium_max_attempts
        goal_msg.settling_time_s = self.iridium_settling_time_s

        self.get_logger().info("Sending /iridium/run_window goal...")
        future = self.iridium_client.send_goal_async(
            goal_msg,
            feedback_callback=self.iridium_feedback_callback
        )
        future.add_done_callback(self.iridium_goal_response_callback)

        self.iridium_goal_in_flight = True
        self.idle_phase = IdlePhase.WAITING_FOR_WINDOW_RESULT

    def iridium_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Iridium window feedback: phase={fb.phase}, attempt={fb.attempt_number}"
        )

    def iridium_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("/iridium/run_window goal rejected")
            self.iridium_goal_in_flight = False
            self.idle_phase = IdlePhase.DEACTIVATING_IRIDIUM
            return

        self.get_logger().info("/iridium/run_window goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.iridium_result_callback)

    def iridium_result_callback(self, future):
        result_wrap = future.result()
        status = result_wrap.status
        result = result_wrap.result

        self.iridium_goal_in_flight = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"/iridium/run_window completed: window_success={result.window_success}, "
                f"mission_received={result.mission_received}, "
                f"attempts={result.attempts_used}, msg='{result.status_message}'"
            )

            if not result.window_success:
                self.get_logger().warn("Iridium window failed; remaining in IDLE")

            if result.mission_received:
                self.pending_mission_text = result.mission_text
                self.get_logger().info(
                    f"Mission received from Iridium: '{self.pending_mission_text}'"
                )
                self.transition_to(MissionState.INITIALISE)
            else:
                self.get_logger().info("No mission received; remaining in IDLE")
                self.idle_phase = IdlePhase.DEACTIVATING_IRIDIUM
        else:
            self.get_logger().warn(
                f"/iridium/run_window ended with status {status}; remaining in IDLE"
            )
            self.idle_phase = IdlePhase.DEACTIVATING_IRIDIUM

    # =========================================================
    # State handlers
    # =========================================================

    def handle_idle(self):
        if self.idle_phase == IdlePhase.WAITING_FOR_TIMER:
            if self.idle_timer_elapsed():
                self.get_logger().info("IDLE timer elapsed; starting Iridium cycle")
                self.idle_phase = IdlePhase.ACTIVATING_IRIDIUM

        elif self.idle_phase == IdlePhase.ACTIVATING_IRIDIUM:
            result = self.activate_iridium()
            if result is True:
                self.idle_phase = IdlePhase.SENDING_WINDOW_GOAL
            elif result is False:
                self.get_logger().warn("Failed to activate Iridium; staying in IDLE")
                self._lc_future = None
                self.reset_idle_timer()
                self.idle_phase = IdlePhase.WAITING_FOR_TIMER
            # None = still waiting, stay in this phase

        elif self.idle_phase == IdlePhase.SENDING_WINDOW_GOAL:
            self.send_iridium_window_goal()

        elif self.idle_phase == IdlePhase.WAITING_FOR_WINDOW_RESULT:
            # Asynchronous action in progress
            pass

        elif self.idle_phase == IdlePhase.DEACTIVATING_IRIDIUM:
            result = self.deactivate_iridium()
            if result is True:
                self.reset_idle_timer()
                self.idle_phase = IdlePhase.WAITING_FOR_TIMER
            # None = still waiting, stay in this phase

    def handle_initialise(self):
        # We will fill this in next
        pass

    def handle_operation(self):
        # We will fill this in later
        pass

    def handle_emergency(self):
        # We will fill this in later
        pass

    # =========================================================
    # Main loop
    # =========================================================

    def step(self):
        self.publish_state()

        # Global emergency override
        if self.emergency_triggered and self.state != MissionState.EMERGENCY:
            self.get_logger().error("Emergency triggered")
            self.transition_to(MissionState.EMERGENCY)

        if self.state == MissionState.IDLE:
            self.handle_idle()
        elif self.state == MissionState.INITIALISE:
            self.handle_initialise()
        elif self.state == MissionState.OPERATION:
            self.handle_operation()
        elif self.state == MissionState.EMERGENCY:
            self.handle_emergency()


def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

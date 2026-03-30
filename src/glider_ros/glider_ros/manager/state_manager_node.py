#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String

# ------------------------------------------------------------
# TODO:
# Replace this with your real action type.
# Example:
# from glider_msgs.action import InitStep
# ------------------------------------------------------------
from example_interfaces.action import Fibonacci as InitActionPlaceholder


class MissionState(Enum):
    INITIALISE = auto()
    IDLE = auto()
    OPERATE = auto()
    EMERGENCY = auto()


class InitPhase(Enum):
    WAIT_BOOT_DELAY = auto()
    SEND_HOME = auto()
    WAIT_HOME_RESULT = auto()
    SEND_ECHO = auto()
    WAIT_ECHO_RESULT = auto()
    DONE = auto()


class StateManager(Node):
    def __init__(self):
        super().__init__("state_manager")

        # -----------------------------
        # Main mission state
        # -----------------------------
        self.state = MissionState.INITIALISE
        self.prev_state = None

        # -----------------------------
        # Initialise sub-steps
        # -----------------------------
        self.init_phase = InitPhase.WAIT_BOOT_DELAY
        self.boot_time_ns = self.get_clock().now().nanoseconds
        self.boot_delay_sec = 60.0

        # -----------------------------
        # Action clients for init steps
        # -----------------------------
        self.home_client = ActionClient(
            self,
            InitActionPlaceholder,
            "/initialise/home"
        )

        self.echo_client = ActionClient(
            self,
            InitActionPlaceholder,
            "/initialise/echo"
        )

        # Track active goal state
        self.home_goal_sent = False
        self.echo_goal_sent = False
        self.waiting_for_action = False

        # -----------------------------
        # Global flags
        # -----------------------------
        self.emergency_triggered = False
        self.mission_start_requested = False

        # -----------------------------
        # Publishers
        # -----------------------------
        self.state_pub = self.create_publisher(String, "/manager/state", 10)
        self.mode_pub = self.create_publisher(String, "/manager/control_mode", 10)
        self.emergency_pub = self.create_publisher(Bool, "/cmd/resurface", 10)

        # -----------------------------
        # Example subscriptions
        # Replace/add your real topics
        # -----------------------------
        self.create_subscription(Bool, "/safety/emergency", self.emergency_cb, 10)
        self.create_subscription(Bool, "/manager/start_mission", self.start_mission_cb, 10)

        # -----------------------------
        # Main loop
        # -----------------------------
        self.timer = self.create_timer(0.2, self.step)

        self.get_logger().info("State manager started in INITIALISE")

    # ============================================================
    # Subscriptions
    # ============================================================

    def emergency_cb(self, msg: Bool):
        if msg.data:
            self.emergency_triggered = True

    def start_mission_cb(self, msg: Bool):
        self.mission_start_requested = msg.data

    # ============================================================
    # Utility
    # ============================================================

    def transition_to(self, new_state: MissionState):
        if new_state != self.state:
            self.get_logger().info(f"STATE: {self.state.name} -> {new_state.name}")
            self.prev_state = self.state
            self.state = new_state

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def publish_resurface(self, active: bool):
        msg = Bool()
        msg.data = active
        self.emergency_pub.publish(msg)

    def boot_delay_done(self) -> bool:
        elapsed_sec = (self.get_clock().now().nanoseconds - self.boot_time_ns) / 1e9
        return elapsed_sec >= self.boot_delay_sec

    # ============================================================
    # Action helpers
    # ============================================================

    def send_home_goal(self):
        if self.home_goal_sent:
            return

        if not self.home_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("/initialise/home action server not available yet")
            return

        # --------------------------------------------------------
        # TODO:
        # Replace with your real goal message.
        # This placeholder uses Fibonacci just so structure is visible.
        # --------------------------------------------------------
        goal_msg = InitActionPlaceholder.Goal()
        goal_msg.order = 1

        self.get_logger().info("Sending /initialise/home goal...")
        future = self.home_client.send_goal_async(goal_msg)
        future.add_done_callback(self.home_goal_response_callback)

        self.home_goal_sent = True
        self.waiting_for_action = True
        self.init_phase = InitPhase.WAIT_HOME_RESULT

    def home_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("/initialise/home goal was rejected")
            self.waiting_for_action = False
            self.transition_to(MissionState.EMERGENCY)
            return

        self.get_logger().info("/initialise/home goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("/initialise/home completed successfully")
            self.waiting_for_action = False
            self.init_phase = InitPhase.SEND_ECHO
        else:
            self.get_logger().error(f"/initialise/home failed with status {status}")
            self.waiting_for_action = False
            self.transition_to(MissionState.EMERGENCY)

    def send_echo_goal(self):
        if self.echo_goal_sent:
            return

        if not self.echo_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("/initialise/echo action server not available yet")
            return

        # --------------------------------------------------------
        # TODO:
        # Replace with your real goal message.
        # --------------------------------------------------------
        goal_msg = InitActionPlaceholder.Goal()
        goal_msg.order = 1

        self.get_logger().info("Sending /initialise/echo goal...")
        future = self.echo_client.send_goal_async(goal_msg)
        future.add_done_callback(self.echo_goal_response_callback)

        self.echo_goal_sent = True
        self.waiting_for_action = True
        self.init_phase = InitPhase.WAIT_ECHO_RESULT

    def echo_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("/initialise/echo goal was rejected")
            self.waiting_for_action = False
            self.transition_to(MissionState.EMERGENCY)
            return

        self.get_logger().info("/initialise/echo goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.echo_result_callback)

    def echo_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("/initialise/echo completed successfully")
            self.waiting_for_action = False
            self.init_phase = InitPhase.DONE
        else:
            self.get_logger().error(f"/initialise/echo failed with status {status}")
            self.waiting_for_action = False
            self.transition_to(MissionState.EMERGENCY)

    # ============================================================
    # State handlers
    # ============================================================

    def handle_initialise(self):
        self.publish_mode("INITIALISE")

        if self.init_phase == InitPhase.WAIT_BOOT_DELAY:
            if self.boot_delay_done():
                self.get_logger().info("Boot delay complete, starting /initialise/home")
                self.init_phase = InitPhase.SEND_HOME

        elif self.init_phase == InitPhase.SEND_HOME:
            self.send_home_goal()

        elif self.init_phase == InitPhase.WAIT_HOME_RESULT:
            # Waiting asynchronously, nothing to do here
            pass

        elif self.init_phase == InitPhase.SEND_ECHO:
            self.send_echo_goal()

        elif self.init_phase == InitPhase.WAIT_ECHO_RESULT:
            # Waiting asynchronously, nothing to do here
            pass

        elif self.init_phase == InitPhase.DONE:
            self.transition_to(MissionState.IDLE)

    def handle_idle(self):
        self.publish_mode("IDLE")

        # Safe/neutral mode while waiting
        # Example: controller disabled or hold-neutral
        if self.mission_start_requested:
            self.get_logger().info("Mission start received")
            self.transition_to(MissionState.OPERATE)

    def handle_operate(self):
        self.publish_mode("OPERATE")

        # Framework only for now
        # Put normal mission logic / setpoint publishing here
        pass

    def handle_emergency(self):
        self.publish_mode("EMERGENCY")

        # Force emergency resurface
        self.publish_resurface(True)

        # Stay latched here unless you later want a manual reset path
        pass

    # ============================================================
    # Main loop
    # ============================================================

    def step(self):
        # Always publish current state
        self.publish_state()

        # Global emergency override from any state
        if self.emergency_triggered and self.state != MissionState.EMERGENCY:
            self.get_logger().error("Emergency trigger received")
            self.transition_to(MissionState.EMERGENCY)

        # State dispatch
        if self.state == MissionState.INITIALISE:
            self.handle_initialise()

        elif self.state == MissionState.IDLE:
            self.handle_idle()

        elif self.state == MissionState.OPERATE:
            self.handle_operate()

        elif self.state == MissionState.EMERGENCY:
            self.handle_emergency()


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
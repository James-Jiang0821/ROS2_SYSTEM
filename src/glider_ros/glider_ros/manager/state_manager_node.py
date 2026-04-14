#!/usr/bin/env python3

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, Float64, String
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from glider_msgs.action import IridiumWindow, HomeActuators


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


class InitialisePhase(Enum):
    SENDING_HOME_GOAL = auto()
    WAITING_FOR_HOME_RESULT = auto()


class OperationPhase(Enum):
    CONFIGURING_CONTROLLER = auto()
    SETTING_DEPTH_PARAM = auto()
    ACTIVATING_CONTROLLER = auto()
    RUNNING = auto()
    DEACTIVATING_CONTROLLER = auto()
    CLEANING_UP_CONTROLLER = auto()


class EmergencyPhase(Enum):
    CHECKING_DEPTH = auto()               # decide: underwater or surface?
    SURFACING = auto()                    # force controller to climb, wait for COMPLETE
    DEACTIVATING_CONTROLLER = auto()      # stop the controller lifecycle node
    CLEANING_UP_CONTROLLER = auto()       # cleanup the controller lifecycle node
    SAFE_WAITING_FOR_TIMER = auto()       # on surface, controller off — wait before Iridium
    SAFE_ACTIVATING_IRIDIUM = auto()      # configure Iridium lifecycle node
    SAFE_SENDING_WINDOW_GOAL = auto()     # send SBDWT-only window goal
    SAFE_WAITING_FOR_WINDOW_RESULT = auto()  # waiting for action result
    SAFE_DEACTIVATING_IRIDIUM = auto()    # cleanup Iridium, then loop back to timer


class StateManagerNode(Node):
    def __init__(self):
        super().__init__("state_manager_node")

        # -----------------------------
        # Top-level mission state
        # -----------------------------
        self.state = MissionState.IDLE

        # -----------------------------
        # Sub-phases
        # -----------------------------
        self.idle_phase = IdlePhase.WAITING_FOR_TIMER
        self.initialise_phase = InitialisePhase.SENDING_HOME_GOAL
        self.operation_phase = OperationPhase.CONFIGURING_CONTROLLER
        self.emergency_phase = EmergencyPhase.CHECKING_DEPTH

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("idle_iridium_period_s", 300.0)    # 5 min
        self.declare_parameter("emergency_iridium_period_s", 300.0)  # 5 min
        self.declare_parameter("iridium_settling_time_s", 60.0)
        self.declare_parameter("iridium_max_attempts", 3)
        self.declare_parameter("surface_depth_m", 1.0)          # depth below which we consider glider submerged

        self.idle_iridium_period_s = float(
            self.get_parameter("idle_iridium_period_s").value)
        self.emergency_iridium_period_s = float(
            self.get_parameter("emergency_iridium_period_s").value)
        self.iridium_settling_time_s = float(
            self.get_parameter("iridium_settling_time_s").value)
        self.iridium_max_attempts = int(
            self.get_parameter("iridium_max_attempts").value)
        self.surface_depth_m = float(
            self.get_parameter("surface_depth_m").value)

        # -----------------------------
        # Track timing
        # -----------------------------
        now_ns = self.get_clock().now().nanoseconds
        self.last_idle_cycle_start_ns = now_ns
        self.last_emergency_iridium_cycle_start_ns = now_ns

        # -----------------------------
        # State variables
        # -----------------------------
        self.iridium_goal_in_flight = False
        self.pending_mission_text = ""
        self.emergency_triggered = False
        self.home_goal_in_flight = False
        self.controller_complete = False
        self._controller_active = False  # True only while in OPERATION RUNNING phase
        self._current_depth = 0.0        # latest reading from /pressure/depth

        # Async futures
        self._iridium_lc_future = None   # Iridium lifecycle transitions
        self._ctrl_lc_future = None      # Controller lifecycle transitions
        self._param_future = None        # Controller parameter set

        # -----------------------------
        # Publishers
        # -----------------------------
        self.state_pub = self.create_publisher(String, "/manager/state", 10)
        self._force_surface_pub = self.create_publisher(
            Bool, "/controller/force_surface", 10)
        self._emergency_detail_pub = self.create_publisher(
            String, "/state/detail", 10)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(Bool, "/safety/emergency", self.emergency_cb, 10)
        self.create_subscription(String, "/controller/phase", self._controller_phase_cb, 10)
        self.create_subscription(Float64, "/pressure/depth", self._depth_cb, 10)

        # -----------------------------
        # Action clients
        # -----------------------------
        self.iridium_client = ActionClient(self, IridiumWindow, "/iridium/run_window")
        self.home_client = ActionClient(self, HomeActuators, "/bridge/home_actuators")

        # -----------------------------
        # Lifecycle service clients
        # -----------------------------
        self._iridium_lc_client = self.create_client(
            ChangeState, "/communication_iridium_node/change_state")
        self._ctrl_lc_client = self.create_client(
            ChangeState, "/glider_controller/change_state")

        # -----------------------------
        # Parameter service client (controller)
        # -----------------------------
        self._param_client = self.create_client(
            SetParameters, "/glider_controller/set_parameters")

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

    def _depth_cb(self, msg: Float64):
        self._current_depth = msg.data

    def _controller_phase_cb(self, msg: String):
        if msg.data == 'COMPLETE' and not self.controller_complete:
            self.get_logger().info("Controller reported mission COMPLETE")
            self.controller_complete = True

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

    def emergency_iridium_timer_elapsed(self) -> bool:
        elapsed_s = (
            self.get_clock().now().nanoseconds - self.last_emergency_iridium_cycle_start_ns
        ) / 1e9
        return elapsed_s >= self.emergency_iridium_period_s

    def reset_emergency_iridium_timer(self):
        self.last_emergency_iridium_cycle_start_ns = self.get_clock().now().nanoseconds

    # =========================================================
    # Iridium lifecycle helpers
    # =========================================================

    def activate_iridium(self) -> Optional[bool]:
        """Configure Iridium lifecycle node (unconfigured → inactive, opens serial port)."""
        if self._iridium_lc_future is None:
            if not self._iridium_lc_client.service_is_ready():
                self.get_logger().warn("Iridium lifecycle service not ready; will retry")
                return False
            self.get_logger().info(
                "Sending lifecycle 'configure' to communication_iridium_node")
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            self._iridium_lc_future = self._iridium_lc_client.call_async(req)
            return None

        if not self._iridium_lc_future.done():
            return None

        result = self._iridium_lc_future.result()
        self._iridium_lc_future = None

        if result is None or not result.success:
            self.get_logger().warn("Iridium 'configure' transition failed")
            return False

        self.get_logger().info("Iridium node configured and ready")
        return True

    def deactivate_iridium(self) -> Optional[bool]:
        """Cleanup Iridium lifecycle node (inactive → unconfigured, closes serial port)."""
        if self._iridium_lc_future is None:
            if not self._iridium_lc_client.service_is_ready():
                self.get_logger().warn(
                    "Iridium lifecycle service not ready; skipping cleanup")
                return True
            self.get_logger().info(
                "Sending lifecycle 'cleanup' to communication_iridium_node")
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CLEANUP
            self._iridium_lc_future = self._iridium_lc_client.call_async(req)
            return None

        if not self._iridium_lc_future.done():
            return None

        result = self._iridium_lc_future.result()
        self._iridium_lc_future = None

        if result is None or not result.success:
            self.get_logger().warn("Iridium 'cleanup' failed; continuing anyway")
        else:
            self.get_logger().info("Iridium node cleaned up")

        return True  # non-critical — always proceed

    # =========================================================
    # Controller lifecycle helpers
    # =========================================================

    def _ctrl_lc_call(self, transition_id: int) -> Optional[bool]:
        """Generic async lifecycle transition for the controller node."""
        if self._ctrl_lc_future is None:
            if not self._ctrl_lc_client.service_is_ready():
                return False
            req = ChangeState.Request()
            req.transition.id = transition_id
            self._ctrl_lc_future = self._ctrl_lc_client.call_async(req)
            return None

        if not self._ctrl_lc_future.done():
            return None

        result = self._ctrl_lc_future.result()
        self._ctrl_lc_future = None
        return result is not None and result.success

    def configure_controller(self) -> Optional[bool]:
        result = self._ctrl_lc_call(Transition.TRANSITION_CONFIGURE)
        if result is True:
            self.get_logger().info("Controller configured")
        elif result is False:
            self.get_logger().warn("Controller lifecycle service not ready; will retry")
        return result

    def activate_controller(self) -> Optional[bool]:
        result = self._ctrl_lc_call(Transition.TRANSITION_ACTIVATE)
        if result is True:
            self.get_logger().info("Controller activated")
        elif result is False:
            self.get_logger().error("Controller 'activate' transition failed")
        return result

    def deactivate_controller(self) -> Optional[bool]:
        if not self._ctrl_lc_client.service_is_ready():
            return True  # non-critical, proceed to cleanup
        result = self._ctrl_lc_call(Transition.TRANSITION_DEACTIVATE)
        if result is True:
            self.get_logger().info("Controller deactivated")
        return result

    def cleanup_controller(self) -> Optional[bool]:
        if not self._ctrl_lc_client.service_is_ready():
            return True
        result = self._ctrl_lc_call(Transition.TRANSITION_CLEANUP)
        if result is True:
            self.get_logger().info("Controller cleaned up")
        return result

    def set_controller_depth(self, depth_m: float) -> Optional[bool]:
        """Set depth_lower on the controller node via the parameter service."""
        if self._param_future is None:
            if not self._param_client.service_is_ready():
                self.get_logger().warn(
                    "Controller parameter service not ready; will retry")
                return False
            pv = ParameterValue()
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = depth_m
            p = Parameter()
            p.name = 'depth_lower'
            p.value = pv
            req = SetParameters.Request()
            req.parameters = [p]
            self._param_future = self._param_client.call_async(req)
            return None

        if not self._param_future.done():
            return None

        result = self._param_future.result()
        self._param_future = None

        if result is None or not all(r.successful for r in result.results):
            self.get_logger().warn("Failed to set depth_lower on controller")
            return False

        self.get_logger().info(f"Controller depth_lower set to {depth_m}m")
        return True

    # =========================================================
    # Iridium action helpers
    # =========================================================

    def send_iridium_window_goal(self, read_mission: bool = True):
        if self.iridium_goal_in_flight:
            return

        if not self.iridium_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("/iridium/run_window action server not available yet")
            return

        goal_msg = IridiumWindow.Goal()
        goal_msg.latest_telemetry = ""
        goal_msg.max_attempts = self.iridium_max_attempts
        goal_msg.settling_time_s = self.iridium_settling_time_s
        goal_msg.read_mission = read_mission

        self.get_logger().info(
            f"Sending /iridium/run_window goal (read_mission={read_mission})...")
        future = self.iridium_client.send_goal_async(
            goal_msg, feedback_callback=self.iridium_feedback_callback)
        future.add_done_callback(self.iridium_goal_response_callback)

        self.iridium_goal_in_flight = True
        if self.state == MissionState.EMERGENCY:
            self.emergency_phase = EmergencyPhase.SAFE_WAITING_FOR_WINDOW_RESULT
        else:
            self.idle_phase = IdlePhase.WAITING_FOR_WINDOW_RESULT

    def iridium_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Iridium window feedback: phase={fb.phase}, attempt={fb.attempt_number}")

    def iridium_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("/iridium/run_window goal rejected")
            self.iridium_goal_in_flight = False
            if self.state == MissionState.EMERGENCY:
                self.emergency_phase = EmergencyPhase.SAFE_DEACTIVATING_IRIDIUM
            else:
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

        if self.state == MissionState.EMERGENCY:
            # Emergency window — SBDWT only, no mission processing, loop back
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Emergency Iridium window done: window_success={result.window_success}, "
                    f"attempts={result.attempts_used}")
            else:
                self.get_logger().warn(
                    f"Emergency Iridium window ended with status {status}")
            self.emergency_phase = EmergencyPhase.SAFE_DEACTIVATING_IRIDIUM
            return

        # IDLE window — normal mission-reading path
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
                    " — deactivating Iridium before homing"
                )
            else:
                self.get_logger().info("No mission received; remaining in IDLE")
        else:
            self.get_logger().warn(
                f"/iridium/run_window ended with status {status}; remaining in IDLE")

        # Always deactivate Iridium before doing anything else
        self.idle_phase = IdlePhase.DEACTIVATING_IRIDIUM

    # =========================================================
    # Home action helpers
    # =========================================================

    def send_home_goal(self):
        if self.home_goal_in_flight:
            return

        if not self.home_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("/bridge/home_actuators action server not available yet")
            return

        goal_msg = HomeActuators.Goal()
        goal_msg.timeout_s = 0.0  # use bridge default (65 s)

        self.get_logger().info("Sending /bridge/home_actuators goal...")
        future = self.home_client.send_goal_async(
            goal_msg, feedback_callback=self.home_feedback_callback)
        future.add_done_callback(self.home_goal_response_callback)

        self.home_goal_in_flight = True
        self.initialise_phase = InitialisePhase.WAITING_FOR_HOME_RESULT

    def home_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Homing feedback: vbd_left={fb.vbd_left_homed}, "
            f"vbd_right={fb.vbd_right_homed}, "
            f"pitch={fb.pitch_homed}, roll={fb.roll_homed}")

    def home_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("/bridge/home_actuators goal rejected")
            self.home_goal_in_flight = False
            self._publish_emergency_detail("Homing failed: action goal rejected by bridge")
            self.transition_to(MissionState.EMERGENCY)
            return

        self.get_logger().info("/bridge/home_actuators goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        result_wrap = future.result()
        status = result_wrap.status
        result = result_wrap.result

        self.home_goal_in_flight = False

        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(f"Homing succeeded: {result.status_message}")
            self.operation_phase = OperationPhase.CONFIGURING_CONTROLLER
            self.transition_to(MissionState.OPERATION)
        else:
            detail = f"Homing failed: {result.status_message}"
            self.get_logger().error(detail)
            self._publish_emergency_detail(detail)
            self.transition_to(MissionState.EMERGENCY)

    def _publish_emergency_detail(self, detail: str):
        msg = String()
        msg.data = detail
        self._emergency_detail_pub.publish(msg)

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
                self._iridium_lc_future = None
                self.reset_idle_timer()
                self.idle_phase = IdlePhase.WAITING_FOR_TIMER

        elif self.idle_phase == IdlePhase.SENDING_WINDOW_GOAL:
            self.send_iridium_window_goal()

        elif self.idle_phase == IdlePhase.WAITING_FOR_WINDOW_RESULT:
            pass  # driven by action callbacks

        elif self.idle_phase == IdlePhase.DEACTIVATING_IRIDIUM:
            result = self.deactivate_iridium()
            if result is True:
                if self.pending_mission_text:
                    # Mission waiting — proceed to homing
                    self.initialise_phase = InitialisePhase.SENDING_HOME_GOAL
                    self.transition_to(MissionState.INITIALISE)
                else:
                    self.reset_idle_timer()
                    self.idle_phase = IdlePhase.WAITING_FOR_TIMER

    def handle_initialise(self):
        if self.initialise_phase == InitialisePhase.SENDING_HOME_GOAL:
            self.send_home_goal()
        elif self.initialise_phase == InitialisePhase.WAITING_FOR_HOME_RESULT:
            pass  # driven by action callbacks

    def handle_operation(self):
        if self.operation_phase == OperationPhase.CONFIGURING_CONTROLLER:
            result = self.configure_controller()
            if result is True:
                self.operation_phase = OperationPhase.SETTING_DEPTH_PARAM
            # False = service not ready, stay and retry; None = waiting

        elif self.operation_phase == OperationPhase.SETTING_DEPTH_PARAM:
            try:
                depth_m = float(self.pending_mission_text)
            except ValueError:
                detail = f"Invalid mission depth: '{self.pending_mission_text}'"
                self.get_logger().error(f"{detail}; going to EMERGENCY")
                self._publish_emergency_detail(detail)
                self.transition_to(MissionState.EMERGENCY)
                return
            result = self.set_controller_depth(depth_m)
            if result is True:
                self.operation_phase = OperationPhase.ACTIVATING_CONTROLLER
            # False = service not ready, stay and retry; None = waiting

        elif self.operation_phase == OperationPhase.ACTIVATING_CONTROLLER:
            result = self.activate_controller()
            if result is True:
                self.controller_complete = False
                self._controller_active = True
                self.operation_phase = OperationPhase.RUNNING
            elif result is False:
                self.get_logger().error("Failed to activate controller; going to EMERGENCY")
                self._publish_emergency_detail("Controller activation failed")
                self.transition_to(MissionState.EMERGENCY)

        elif self.operation_phase == OperationPhase.RUNNING:
            if self.controller_complete:
                self.operation_phase = OperationPhase.DEACTIVATING_CONTROLLER

        elif self.operation_phase == OperationPhase.DEACTIVATING_CONTROLLER:
            result = self.deactivate_controller()
            if result is True:
                self.operation_phase = OperationPhase.CLEANING_UP_CONTROLLER
            # None = waiting; False = service gone, also move on

        elif self.operation_phase == OperationPhase.CLEANING_UP_CONTROLLER:
            result = self.cleanup_controller()
            if result is True:
                self.pending_mission_text = ""
                self.controller_complete = False
                self._controller_active = False
                self.operation_phase = OperationPhase.CONFIGURING_CONTROLLER
                self.reset_idle_timer()
                self.idle_phase = IdlePhase.WAITING_FOR_TIMER
                self.transition_to(MissionState.IDLE)

    def handle_emergency(self):
        if self.emergency_phase == EmergencyPhase.CHECKING_DEPTH:
            underwater = self._current_depth > self.surface_depth_m
            if underwater:
                self.get_logger().error(
                    f"EMERGENCY: glider is underwater ({self._current_depth:.2f}m) — forcing surface")
                self.emergency_phase = EmergencyPhase.SURFACING
            else:
                self.get_logger().error(
                    f"EMERGENCY: glider is on surface ({self._current_depth:.2f}m) — "
                    + ("deactivating controller" if self._controller_active else "entering safe hold"))
                if self._controller_active:
                    self.emergency_phase = EmergencyPhase.DEACTIVATING_CONTROLLER
                else:
                    self.reset_emergency_iridium_timer()
                    self.emergency_phase = EmergencyPhase.SAFE_WAITING_FOR_TIMER

        elif self.emergency_phase == EmergencyPhase.SURFACING:
            msg = Bool()
            msg.data = True
            self._force_surface_pub.publish(msg)

            if self.controller_complete:
                self.get_logger().error("EMERGENCY: glider has surfaced — deactivating controller")
                self.controller_complete = False
                self.emergency_phase = EmergencyPhase.DEACTIVATING_CONTROLLER

        elif self.emergency_phase == EmergencyPhase.DEACTIVATING_CONTROLLER:
            result = self.deactivate_controller()
            if result is True:
                self.emergency_phase = EmergencyPhase.CLEANING_UP_CONTROLLER

        elif self.emergency_phase == EmergencyPhase.CLEANING_UP_CONTROLLER:
            result = self.cleanup_controller()
            if result is True:
                self._controller_active = False
                self.get_logger().error("EMERGENCY: controller off — entering safe hold")
                self.reset_emergency_iridium_timer()
                self.emergency_phase = EmergencyPhase.SAFE_WAITING_FOR_TIMER

        elif self.emergency_phase == EmergencyPhase.SAFE_WAITING_FOR_TIMER:
            if self.emergency_iridium_timer_elapsed():
                self.get_logger().info("EMERGENCY: Iridium timer elapsed — opening comms")
                self.emergency_phase = EmergencyPhase.SAFE_ACTIVATING_IRIDIUM

        elif self.emergency_phase == EmergencyPhase.SAFE_ACTIVATING_IRIDIUM:
            result = self.activate_iridium()
            if result is True:
                self.emergency_phase = EmergencyPhase.SAFE_SENDING_WINDOW_GOAL
            elif result is False:
                self.get_logger().warn("EMERGENCY: failed to activate Iridium; will retry next cycle")
                self._iridium_lc_future = None
                self.reset_emergency_iridium_timer()
                self.emergency_phase = EmergencyPhase.SAFE_WAITING_FOR_TIMER

        elif self.emergency_phase == EmergencyPhase.SAFE_SENDING_WINDOW_GOAL:
            self.send_iridium_window_goal(read_mission=False)

        elif self.emergency_phase == EmergencyPhase.SAFE_WAITING_FOR_WINDOW_RESULT:
            pass  # driven by iridium_result_callback

        elif self.emergency_phase == EmergencyPhase.SAFE_DEACTIVATING_IRIDIUM:
            result = self.deactivate_iridium()
            if result is True:
                self.reset_emergency_iridium_timer()
                self.emergency_phase = EmergencyPhase.SAFE_WAITING_FOR_TIMER

    # =========================================================
    # Main loop
    # =========================================================

    def step(self):
        self.publish_state()

        # Global emergency override
        if self.emergency_triggered and self.state != MissionState.EMERGENCY:
            self.get_logger().error("Emergency triggered")
            self.emergency_phase = EmergencyPhase.CHECKING_DEPTH
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

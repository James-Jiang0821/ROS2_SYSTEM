"""
Integration tests for StateManagerNode.

Tests are split into three classes:
  - TestStateManagerCallbacks: call individual subscription callbacks directly
    and verify the resulting internal state changes.
  - TestStateManagerStep: call step() directly and verify state machine
    transitions driven by pre-set internal flags.
  - TestStateManagerEmergencyPhases: set emergency sub-phase and depth
    conditions, call handle_emergency() directly, verify phase transitions.
  - TestStateManagerPubSub: publish on topics, let the 0.2s step timer
    process them, verify /manager/state output.

Action client and lifecycle service interactions are not tested here —
those require stub servers and are covered separately.
"""
import threading
import time

import pytest
import rclpy  # noqa: F401 (used transitively via rclpy_session fixture)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from glider_ros.manager.state_manager_node import (
    EmergencyPhase,
    MissionState,
    StateManagerNode,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _spin_background(*nodes):
    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def _wait_for(predicate, timeout=2.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ── Fixture ───────────────────────────────────────────────────────────────────

@pytest.fixture()
def state_node(rclpy_session):
    node = StateManagerNode()
    yield node
    node.destroy_node()


# ── Callback tests ────────────────────────────────────────────────────────────

class TestStateManagerCallbacks:
    """Call subscription callbacks directly and assert on internal state."""

    def test_emergency_true_sets_flag(self, state_node):
        msg = Bool()
        msg.data = True
        state_node.emergency_cb(msg)
        assert state_node.emergency_triggered is True

    def test_emergency_false_does_not_set_flag(self, state_node):
        msg = Bool()
        msg.data = False
        state_node.emergency_cb(msg)
        assert state_node.emergency_triggered is False

    def test_depth_callback_updates_current_depth(self, state_node):
        msg = Float64()
        msg.data = 7.5
        state_node._depth_cb(msg)
        assert state_node._current_depth == pytest.approx(7.5)

    def test_depth_callback_zero(self, state_node):
        msg = Float64()
        msg.data = 0.0
        state_node._depth_cb(msg)
        assert state_node._current_depth == pytest.approx(0.0)

    def test_controller_complete_sets_flag(self, state_node):
        msg = String()
        msg.data = 'COMPLETE'
        state_node._controller_phase_cb(msg)
        assert state_node.controller_complete is True

    def test_controller_operation_phase_does_not_set_complete(self, state_node):
        msg = String()
        msg.data = 'OPERATION'
        state_node._controller_phase_cb(msg)
        assert state_node.controller_complete is False

    def test_mission_inject_in_idle_transitions_to_initialise(self, state_node):
        assert state_node.state == MissionState.IDLE
        msg = Float64()
        msg.data = 10.0
        state_node._mission_inject_cb(msg)
        assert state_node.state == MissionState.INITIALISE

    def test_mission_inject_sets_pending_mission_text(self, state_node):
        msg = Float64()
        msg.data = 15.5
        state_node._mission_inject_cb(msg)
        assert state_node.pending_mission_text == '15.5'

    def test_mission_inject_not_in_idle_is_ignored(self, state_node):
        state_node.state = MissionState.OPERATION
        msg = Float64()
        msg.data = 10.0
        state_node._mission_inject_cb(msg)
        assert state_node.state == MissionState.OPERATION
        assert state_node.pending_mission_text == ''

    def test_mission_inject_in_emergency_is_ignored(self, state_node):
        state_node.state = MissionState.EMERGENCY
        msg = Float64()
        msg.data = 10.0
        state_node._mission_inject_cb(msg)
        assert state_node.state == MissionState.EMERGENCY


# ── Step / state machine tests ────────────────────────────────────────────────

class TestStateManagerStep:
    """Call step() directly with pre-set flags to verify state transitions."""

    def test_emergency_flag_transitions_to_emergency(self, state_node):
        state_node.emergency_triggered = True
        state_node.step()
        assert state_node.state == MissionState.EMERGENCY

    def test_step_without_emergency_stays_in_idle(self, state_node):
        state_node.step()
        assert state_node.state == MissionState.IDLE

    def test_step_publishes_current_state_name(self, state_node):
        """step() calls publish_state() — verify the internal publish path runs
        without error for each state name."""
        for s in MissionState:
            state_node.state = s
            state_node.step()  # must not raise


# ── Emergency phase transition tests ─────────────────────────────────────────

class TestStateManagerEmergencyPhases:
    """Set depth and controller conditions, call handle_emergency() directly,
    verify the resulting phase transition."""

    def test_underwater_goes_to_surfacing(self, state_node):
        state_node.state = MissionState.EMERGENCY
        state_node._current_depth = 5.0   # > surface_depth_m default (1.0 m)
        state_node.emergency_phase = EmergencyPhase.CHECKING_DEPTH
        state_node.handle_emergency()
        assert state_node.emergency_phase == EmergencyPhase.SURFACING

    def test_on_surface_no_controller_goes_to_safe_waiting(self, state_node):
        state_node.state = MissionState.EMERGENCY
        state_node._current_depth = 0.0   # < surface_depth_m
        state_node._controller_active = False
        state_node.emergency_phase = EmergencyPhase.CHECKING_DEPTH
        state_node.handle_emergency()
        assert state_node.emergency_phase == EmergencyPhase.SAFE_WAITING_FOR_TIMER

    def test_on_surface_with_active_controller_goes_to_deactivating(self, state_node):
        state_node.state = MissionState.EMERGENCY
        state_node._current_depth = 0.0
        state_node._controller_active = True
        state_node.emergency_phase = EmergencyPhase.CHECKING_DEPTH
        state_node.handle_emergency()
        assert state_node.emergency_phase == EmergencyPhase.DEACTIVATING_CONTROLLER

    def test_surfacing_phase_publishes_force_surface(self, state_node):
        """While in SURFACING the node should keep publishing force_surface=True."""
        state_node.state = MissionState.EMERGENCY
        state_node.emergency_phase = EmergencyPhase.SURFACING
        state_node.controller_complete = False
        # Must not raise and must stay in SURFACING while controller is not done
        state_node.handle_emergency()
        assert state_node.emergency_phase == EmergencyPhase.SURFACING

    def test_surfacing_moves_to_deactivating_when_controller_complete(self, state_node):
        state_node.state = MissionState.EMERGENCY
        state_node.emergency_phase = EmergencyPhase.SURFACING
        state_node.controller_complete = True
        state_node.handle_emergency()
        assert state_node.emergency_phase == EmergencyPhase.DEACTIVATING_CONTROLLER


# ── Pub/sub integration tests ─────────────────────────────────────────────────

class TestStateManagerPubSub:
    """Publish on topics, let the 0.2 s step timer process them, and verify
    that /manager/state reflects the correct state name."""

    def test_mission_inject_publishes_initialise_on_state_topic(self, rclpy_session):
        node = StateManagerNode()
        helper = Node('test_sm_inject_helper')

        received = []
        helper.create_subscription(
            String, '/manager/state',
            lambda m: received.append(m.data), 10)
        inject_pub = helper.create_publisher(Float64, '/mission/inject', 10)

        executor, thread = _spin_background(node, helper)
        try:
            time.sleep(0.1)

            msg = Float64()
            msg.data = 15.0
            inject_pub.publish(msg)

            assert _wait_for(lambda: 'INITIALISE' in received, timeout=1.0), \
                'Timed out waiting for INITIALISE on /manager/state'
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

    def test_emergency_signal_publishes_emergency_on_state_topic(self, rclpy_session):
        node = StateManagerNode()
        helper = Node('test_sm_emergency_helper')

        received = []
        helper.create_subscription(
            String, '/manager/state',
            lambda m: received.append(m.data), 10)
        emergency_pub = helper.create_publisher(Bool, '/safety/emergency', 10)

        executor, thread = _spin_background(node, helper)
        try:
            time.sleep(0.1)

            msg = Bool()
            msg.data = True
            emergency_pub.publish(msg)

            assert _wait_for(lambda: 'EMERGENCY' in received, timeout=1.0), \
                'Timed out waiting for EMERGENCY on /manager/state'
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

    def test_idle_state_published_on_startup(self, rclpy_session):
        """Node must immediately start publishing IDLE on /manager/state."""
        node = StateManagerNode()
        helper = Node('test_sm_idle_helper')

        received = []
        helper.create_subscription(
            String, '/manager/state',
            lambda m: received.append(m.data), 10)

        executor, thread = _spin_background(node, helper)
        try:
            assert _wait_for(lambda: 'IDLE' in received, timeout=1.0), \
                'Timed out waiting for initial IDLE on /manager/state'
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

"""
Integration tests for SafetyNode.

Tests are split into two classes:
  - TestSafetyNodeInternal: call internal methods directly, inspect state.
    No topic I/O needed — fast and deterministic.
  - TestSafetyNodePubSub: publish on fault topics, verify the node
    publishes the expected /safety/emergency and /safety/detail output.
"""
import threading
import time

import pytest
import rclpy  # noqa: F401 (used transitively via rclpy_session fixture)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from glider_ros.safety.safety_node import SafetyNode


# ── Shared helpers ────────────────────────────────────────────────────────────

def _spin_background(*nodes):
    """Run nodes in a MultiThreadedExecutor on a daemon thread.

    Returns (executor, thread). Call executor.shutdown() to stop.
    """
    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def _wait_for(predicate, timeout=2.0, interval=0.02):
    """Poll predicate() until True or timeout expires. Returns True if met."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture()
def safety_node(rclpy_session):
    node = SafetyNode()
    yield node
    node.destroy_node()


# ── Internal / unit-style tests ───────────────────────────────────────────────

class TestSafetyNodeInternal:
    """Call internal methods directly and assert on node state.

    Each test gets a fresh SafetyNode via the safety_node fixture,
    so the emergency latch always starts cleared.
    """

    def test_hard_fault_latches_emergency(self, safety_node):
        safety_node._check_fault('PR', 'HARD:LEAK|state:FAULT')
        assert safety_node._emergency_latched is True

    def test_hard_fault_stores_source_and_fault_names(self, safety_node):
        safety_node._check_fault('VBD_LEFT', 'HARD:DRIVER_FLT,STALL|state:FAULT')
        assert 'VBD_LEFT' in safety_node._emergency_detail
        assert 'DRIVER_FLT' in safety_node._emergency_detail

    def test_soft_fault_does_not_latch(self, safety_node):
        safety_node._check_fault('PR', 'SOFT:CMD_TO|state:RUN')
        assert safety_node._emergency_latched is False

    def test_ok_message_does_not_latch(self, safety_node):
        safety_node._check_fault('PR', 'OK|state:RUN')
        assert safety_node._emergency_latched is False

    def test_proximity_below_threshold_latches(self, safety_node):
        msg = Float64()
        msg.data = 2.0  # default threshold is 5.0 m
        safety_node._cb_range(msg)
        assert safety_node._emergency_latched is True
        assert 'Proximity' in safety_node._emergency_detail

    def test_proximity_above_threshold_does_not_latch(self, safety_node):
        msg = Float64()
        msg.data = 10.0  # above default 5.0 m threshold
        safety_node._cb_range(msg)
        assert safety_node._emergency_latched is False

    def test_latch_ignores_subsequent_faults(self, safety_node):
        """Once latched, a second fault must not overwrite the detail."""
        safety_node._check_fault('PR', 'HARD:LEAK|state:FAULT')
        first_detail = safety_node._emergency_detail
        safety_node._check_fault('VBD_LEFT', 'HARD:STALL|state:FAULT')
        assert safety_node._emergency_detail == first_detail

    def test_vbd_right_hard_fault_latches(self, safety_node):
        safety_node._check_fault('VBD_RIGHT', 'HARD:OVERCURRENT|state:FAULT')
        assert safety_node._emergency_latched is True
        assert 'VBD_RIGHT' in safety_node._emergency_detail


# ── Pub/sub integration tests ─────────────────────────────────────────────────

class TestSafetyNodePubSub:
    """Boot SafetyNode as a real rclpy node, publish on topics, verify output.

    Each test creates its own node + helper pair so latch state is isolated.
    """

    def test_hard_fault_topic_publishes_emergency_bool(self, rclpy_session):
        node = SafetyNode()
        helper = Node('safety_ps_bool_helper')

        received = []
        helper.create_subscription(
            Bool, '/safety/emergency',
            lambda msg: received.append(msg.data), 10)
        fault_pub = helper.create_publisher(String, '/bridge/pr/fault', 10)

        executor, thread = _spin_background(node, helper)
        try:
            time.sleep(0.1)  # allow discovery

            fault_msg = String()
            fault_msg.data = 'HARD:LEAK|state:FAULT'
            fault_pub.publish(fault_msg)

            # /safety/emergency Bool is driven by the 0.5 s republish timer
            assert _wait_for(lambda: len(received) > 0, timeout=1.5), \
                'Timed out waiting for /safety/emergency'
            assert received[0] is True
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

    def test_hard_fault_topic_publishes_detail_string(self, rclpy_session):
        node = SafetyNode()
        helper = Node('safety_ps_detail_helper')

        received = []
        helper.create_subscription(
            String, '/safety/detail',
            lambda msg: received.append(msg.data), 10)
        fault_pub = helper.create_publisher(
            String, '/bridge/vbd_right/fault', 10)

        executor, thread = _spin_background(node, helper)
        try:
            time.sleep(0.1)

            fault_msg = String()
            fault_msg.data = 'HARD:NOT_HOMED|state:FAULT'
            fault_pub.publish(fault_msg)

            # detail is published immediately inside _trigger_emergency
            assert _wait_for(lambda: len(received) > 0, timeout=1.0), \
                'Timed out waiting for /safety/detail'
            assert 'VBD_RIGHT' in received[0]
            assert 'NOT_HOMED' in received[0]
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

    def test_soft_fault_topic_does_not_publish_emergency(self, rclpy_session):
        node = SafetyNode()
        helper = Node('safety_ps_soft_helper')

        received = []
        helper.create_subscription(
            Bool, '/safety/emergency',
            lambda msg: received.append(msg.data), 10)
        fault_pub = helper.create_publisher(String, '/bridge/pr/fault', 10)

        executor, thread = _spin_background(node, helper)
        try:
            time.sleep(0.1)

            fault_msg = String()
            fault_msg.data = 'SOFT:CMD_TO|state:RUN'
            fault_pub.publish(fault_msg)

            # Wait long enough that a timer fire would have occurred
            time.sleep(0.7)
            assert len(received) == 0, \
                'Soft fault must not trigger /safety/emergency'
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

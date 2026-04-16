"""
Integration tests for TelemetryManagerNode.

The node caches values from:
  /gps/fix        NavSatFix  → lat/lon
  /manager/state  String     → mission state name
  /safety/detail  String     → fault detail from SafetyNode
  /state/detail   String     → detail from StateManagerNode

And publishes a formatted telemetry string on /iridium/sbdwt on a timer.

Tests are split into two classes:
  - TestTelemetryBuildInternal: call _build_telemetry() directly after
    setting internal state. Fast, deterministic, no pub/sub needed.
  - TestTelemetryPubSub: publish on subscribed topics, trigger the publish
    method directly (bypassing the 5s timer), verify /iridium/sbdwt output.
"""
import math
import threading
import time

import pytest
import rclpy  # noqa: F401 (used transitively via rclpy_session fixture)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from glider_ros.manager.telemetry_manager_node import TelemetryManagerNode


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_gps(lat, lon):
    msg = NavSatFix()
    msg.latitude = lat
    msg.longitude = lon
    return msg


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
def telemetry_node(rclpy_session):
    """Fresh TelemetryManagerNode per test. No executor — internal calls only."""
    node = TelemetryManagerNode()
    yield node
    node.destroy_node()


# ── Internal tests ─────────────────────────────────────────────────────────────

class TestTelemetryBuildInternal:
    """Call _build_telemetry() directly after manipulating cached state.

    No spinning required — all logic lives in that one pure method.
    """

    # ── Default / unknown fields ──────────────────────────────────────────────

    def test_default_state_is_unknown(self, telemetry_node):
        result = telemetry_node._build_telemetry()
        assert 'STATE=UNKNOWN' in result

    def test_default_includes_lat_lon_unknown(self, telemetry_node):
        result = telemetry_node._build_telemetry()
        assert 'LAT=UNKNOWN' in result
        assert 'LON=UNKNOWN' in result

    def test_include_unknown_fields_false_hides_gps_unknown(self, telemetry_node):
        telemetry_node.include_unknown_fields = False
        result = telemetry_node._build_telemetry()
        assert 'LAT=UNKNOWN' not in result
        assert 'LON=UNKNOWN' not in result

    def test_include_unknown_fields_false_hides_error_unknown(self, telemetry_node):
        telemetry_node.latest_state = 'EMERGENCY'
        telemetry_node.include_unknown_fields = False
        result = telemetry_node._build_telemetry()
        assert 'ERROR=' not in result

    # ── Mission state names ───────────────────────────────────────────────────

    def test_state_idle_no_error_field(self, telemetry_node):
        telemetry_node.latest_state = 'IDLE'
        result = telemetry_node._build_telemetry()
        assert 'STATE=IDLE' in result
        assert 'ERROR=' not in result

    def test_state_initialise_no_error_field(self, telemetry_node):
        telemetry_node.latest_state = 'INITIALISE'
        result = telemetry_node._build_telemetry()
        assert 'STATE=INITIALISE' in result
        assert 'ERROR=' not in result

    def test_state_operation_no_error_field(self, telemetry_node):
        telemetry_node.latest_state = 'OPERATION'
        result = telemetry_node._build_telemetry()
        assert 'STATE=OPERATION' in result
        assert 'ERROR=' not in result

    def test_state_operation_suppresses_error_even_when_detail_cached(self, telemetry_node):
        """Detail from a prior EMERGENCY must not leak into OPERATION telemetry."""
        telemetry_node.latest_state = 'OPERATION'
        telemetry_node._safety_detail = 'Hard fault on PR: LEAK'
        telemetry_node._state_detail = 'Homing failed'
        result = telemetry_node._build_telemetry()
        assert 'ERROR=' not in result

    def test_state_initialise_suppresses_error_even_when_detail_cached(self, telemetry_node):
        telemetry_node.latest_state = 'INITIALISE'
        telemetry_node._safety_detail = 'Hard fault on VBD_LEFT: STALL'
        result = telemetry_node._build_telemetry()
        assert 'ERROR=' not in result

    # ── EMERGENCY state ───────────────────────────────────────────────────────

    def test_state_emergency_no_detail_gives_error_unknown(self, telemetry_node):
        telemetry_node.latest_state = 'EMERGENCY'
        result = telemetry_node._build_telemetry()
        assert 'STATE=EMERGENCY' in result
        assert 'ERROR=UNKNOWN' in result

    def test_state_emergency_with_safety_detail(self, telemetry_node):
        telemetry_node.latest_state = 'EMERGENCY'
        telemetry_node._safety_detail = 'Hard fault on VBD_LEFT: LEAK'
        result = telemetry_node._build_telemetry()
        assert 'Hard fault on VBD_LEFT: LEAK' in result

    def test_state_emergency_with_state_detail(self, telemetry_node):
        telemetry_node.latest_state = 'EMERGENCY'
        telemetry_node._state_detail = 'Homing failed: timeout'
        result = telemetry_node._build_telemetry()
        assert 'Homing failed: timeout' in result

    def test_state_emergency_both_details_joined_with_pipe(self, telemetry_node):
        telemetry_node.latest_state = 'EMERGENCY'
        telemetry_node._safety_detail = 'LEAK detected'
        telemetry_node._state_detail = 'Homing failed'
        result = telemetry_node._build_telemetry()
        assert 'LEAK detected | Homing failed' in result

    # ── GPS ───────────────────────────────────────────────────────────────────

    def test_valid_gps_replaces_unknown(self, telemetry_node):
        telemetry_node.latest_lat = 51.5
        telemetry_node.latest_lon = -0.126
        result = telemetry_node._build_telemetry()
        assert 'LAT=51.500000' in result
        assert 'LON=-0.126000' in result
        assert 'LAT=UNKNOWN' not in result

    def test_nan_gps_is_rejected(self, telemetry_node):
        telemetry_node._cb_gps(_make_gps(math.nan, math.nan))
        assert telemetry_node.latest_lat is None
        assert telemetry_node.latest_lon is None

    def test_inf_gps_is_rejected(self, telemetry_node):
        telemetry_node._cb_gps(_make_gps(math.inf, -math.inf))
        assert telemetry_node.latest_lat is None
        assert telemetry_node.latest_lon is None


# ── Pub/sub tests ──────────────────────────────────────────────────────────────

class TestTelemetryPubSub:
    """Publish on subscribed topics, call _publish_telemetry() to bypass the
    5s timer, and verify the output on /iridium/sbdwt.
    """

    def _run(self, node, helper, pub_fn, check_fn):
        """Shared helper: spin both nodes, run pub_fn, check output."""
        received = []
        helper.create_subscription(
            String, '/iridium/sbdwt',
            lambda m: received.append(m.data), 10)

        executor, thread = _spin_background(node, helper)
        try:
            time.sleep(0.1)
            pub_fn()
            time.sleep(0.1)  # let subscription callbacks update cached state
            node._publish_telemetry()
            assert _wait_for(lambda: len(received) > 0, timeout=1.0), \
                'Timed out waiting for /iridium/sbdwt'
            check_fn(received[-1])
        finally:
            executor.shutdown()
            thread.join(timeout=2.0)
            node.destroy_node()
            helper.destroy_node()

    def test_operation_state_topic_no_error(self, rclpy_session):
        node = TelemetryManagerNode()
        helper = Node('test_tm_operation_helper')
        pub = helper.create_publisher(String, '/manager/state', 10)

        def publish():
            msg = String()
            msg.data = 'OPERATION'
            pub.publish(msg)

        self._run(node, helper, publish,
                  lambda r: (
                      self._assert_in('STATE=OPERATION', r),
                      self._assert_not_in('ERROR=', r)
                  ))

    def test_initialise_state_topic_no_error(self, rclpy_session):
        node = TelemetryManagerNode()
        helper = Node('test_tm_initialise_helper')
        pub = helper.create_publisher(String, '/manager/state', 10)

        def publish():
            msg = String()
            msg.data = 'INITIALISE'
            pub.publish(msg)

        self._run(node, helper, publish,
                  lambda r: (
                      self._assert_in('STATE=INITIALISE', r),
                      self._assert_not_in('ERROR=', r)
                  ))

    def test_emergency_state_with_detail_topic(self, rclpy_session):
        node = TelemetryManagerNode()
        helper = Node('test_tm_emergency_helper')
        state_pub = helper.create_publisher(String, '/manager/state', 10)
        detail_pub = helper.create_publisher(String, '/safety/detail', 10)

        def publish():
            s = String()
            s.data = 'EMERGENCY'
            state_pub.publish(s)
            d = String()
            d.data = 'Hard fault on PR: LEAK'
            detail_pub.publish(d)

        self._run(node, helper, publish,
                  lambda r: (
                      self._assert_in('STATE=EMERGENCY', r),
                      self._assert_in('LEAK', r)
                  ))

    # ── Assert helpers so lambda bodies stay readable ─────────────────────────

    def _assert_in(self, needle, haystack):
        assert needle in haystack, f'{needle!r} not found in {haystack!r}'

    def _assert_not_in(self, needle, haystack):
        assert needle not in haystack, f'{needle!r} unexpectedly found in {haystack!r}'

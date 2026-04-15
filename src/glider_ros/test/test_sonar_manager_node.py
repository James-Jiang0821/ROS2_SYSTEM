"""
Integration tests for SonarManagerNode.

The node subscribes to:
  /sonar/range       sensor_msgs/Range   — raw sonar distance
  /sonar/confidence  std_msgs/Float32    — reading quality (0-100 %)

And publishes:
  /glider/range      std_msgs/Float64    — moving-average filtered output

Tests verify: confidence gating, staleness rejection, out-of-bounds clamping,
and moving-average smoothing.
"""
import threading
import time

import pytest
import rclpy  # noqa: F401 (used transitively via rclpy_session fixture)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float64

from glider_ros.manager.sonar_manager_node import SonarManagerNode


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_range(range_m, min_range=0.3, max_range=50.0):
    msg = Range()
    msg.range = range_m
    msg.min_range = min_range
    msg.max_range = max_range
    return msg


def _make_conf(value):
    msg = Float32()
    msg.data = float(value)
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
def sonar_fixture(rclpy_session):
    """Provide a live SonarManagerNode, publishers, and a received list."""
    node = SonarManagerNode()
    helper = Node('test_sonar_helper')
    executor, thread = _spin_background(node, helper)

    received = []
    helper.create_subscription(
        Float64, '/glider/range',
        lambda m: received.append(m.data), 10)

    range_pub = helper.create_publisher(Range, '/sonar/range', 10)
    conf_pub = helper.create_publisher(Float32, '/sonar/confidence', 10)

    time.sleep(0.1)  # allow topic discovery

    yield node, range_pub, conf_pub, received

    executor.shutdown()
    thread.join(timeout=2.0)
    node.destroy_node()
    helper.destroy_node()


# ── Tests ─────────────────────────────────────────────────────────────────────

class TestSonarManagerNode:

    def test_range_without_confidence_is_dropped(self, sonar_fixture):
        """Range published before any confidence must be silently discarded."""
        _, range_pub, _, received = sonar_fixture
        range_pub.publish(_make_range(5.0))
        time.sleep(0.4)
        assert len(received) == 0, \
            'Range without prior confidence should not be published'

    def test_low_confidence_range_is_dropped(self, sonar_fixture):
        """Confidence below the 50% threshold must gate the range output."""
        _, range_pub, conf_pub, received = sonar_fixture
        conf_pub.publish(_make_conf(30.0))  # below default 50 %
        time.sleep(0.05)

        range_pub.publish(_make_range(5.0))
        time.sleep(0.4)
        assert len(received) == 0, \
            'Range with low confidence should not be published'

    def test_good_confidence_range_is_published(self, sonar_fixture):
        """Confidence above threshold + valid range must produce output."""
        _, range_pub, conf_pub, received = sonar_fixture
        conf_pub.publish(_make_conf(90.0))
        time.sleep(0.05)

        range_pub.publish(_make_range(8.0))
        assert _wait_for(lambda: len(received) > 0), \
            'Timed out: valid range + good confidence should be published'
        assert abs(received[-1] - 8.0) < 0.01

    def test_out_of_range_reading_is_rejected(self, sonar_fixture):
        """Readings outside [min_range, max_range] must be discarded."""
        _, range_pub, conf_pub, received = sonar_fixture
        conf_pub.publish(_make_conf(90.0))
        time.sleep(0.05)

        range_pub.publish(_make_range(100.0, min_range=0.3, max_range=50.0))
        time.sleep(0.4)
        assert len(received) == 0, \
            'Out-of-bounds range should not be published'

    def test_below_min_range_is_rejected(self, sonar_fixture):
        """Readings below min_range must be discarded."""
        _, range_pub, conf_pub, received = sonar_fixture
        conf_pub.publish(_make_conf(90.0))
        time.sleep(0.05)

        range_pub.publish(_make_range(0.1, min_range=0.3, max_range=50.0))
        time.sleep(0.4)
        assert len(received) == 0, \
            'Below-min range should not be published'

    def test_moving_average_over_window(self, sonar_fixture):
        """Output must be the running average over the 5-reading window."""
        _, range_pub, conf_pub, received = sonar_fixture
        readings = [4.0, 6.0, 8.0, 6.0, 6.0]

        for r in readings:
            conf_pub.publish(_make_conf(90.0))  # keep confidence fresh
            time.sleep(0.02)
            range_pub.publish(_make_range(r))
            time.sleep(0.05)

        assert _wait_for(lambda: len(received) >= len(readings)), \
            'Did not receive enough outputs for moving-average check'

        # After all 5 readings the window is full: average == mean of all 5
        expected = sum(readings) / len(readings)
        assert abs(received[-1] - expected) < 0.01, \
            f'Moving average {received[-1]:.3f} != expected {expected:.3f}'

    def test_stale_confidence_causes_range_to_be_dropped(self, sonar_fixture):
        """Confidence older than confidence_max_age_s (0.5 s) must be ignored."""
        _, range_pub, conf_pub, received = sonar_fixture
        conf_pub.publish(_make_conf(90.0))

        # Wait longer than confidence_max_age_s (default 0.5 s)
        time.sleep(0.6)

        range_pub.publish(_make_range(5.0))
        time.sleep(0.3)
        assert len(received) == 0, \
            'Range with stale confidence should be discarded'

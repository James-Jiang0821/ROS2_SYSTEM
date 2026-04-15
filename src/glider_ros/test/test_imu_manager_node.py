"""
Integration tests for ImuManagerNode.

The node subscribes to /imu/data (sensor_msgs/Imu) and publishes:
  /glider/roll_rad          Float64
  /glider/pitch_rad         Float64
  /glider/pitch_rate_rad_s  Float64
  /glider/heading_deg       Float64

Tests verify the quaternion-to-Euler conversion and pitch-rate passthrough
by publishing known quaternions and asserting the expected scalar outputs.
"""
import math
import threading
import time

import pytest
import rclpy  # noqa: F401 (used transitively via rclpy_session fixture)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from glider_ros.manager.imu_manager_node import ImuManagerNode


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_imu(w=1.0, x=0.0, y=0.0, z=0.0, pitch_rate=0.0):
    """Construct a minimal Imu message with the given quaternion and pitch rate."""
    msg = Imu()
    msg.orientation.w = w
    msg.orientation.x = x
    msg.orientation.y = y
    msg.orientation.z = z
    msg.angular_velocity.y = pitch_rate
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
def imu_fixture(rclpy_session):
    """Provide a live ImuManagerNode, a helper publisher, and a results dict."""
    node = ImuManagerNode()
    helper = Node('test_imu_helper')
    executor, thread = _spin_background(node, helper)

    results = {'roll': None, 'pitch': None, 'pitch_rate': None, 'heading': None}

    helper.create_subscription(
        Float64, '/glider/roll_rad',
        lambda m: results.__setitem__('roll', m.data), 20)
    helper.create_subscription(
        Float64, '/glider/pitch_rad',
        lambda m: results.__setitem__('pitch', m.data), 20)
    helper.create_subscription(
        Float64, '/glider/pitch_rate_rad_s',
        lambda m: results.__setitem__('pitch_rate', m.data), 20)
    helper.create_subscription(
        Float64, '/glider/heading_deg',
        lambda m: results.__setitem__('heading', m.data), 20)

    imu_pub = helper.create_publisher(Imu, '/imu/data', 20)

    time.sleep(0.1)  # allow topic discovery before tests publish

    yield results, imu_pub

    executor.shutdown()
    thread.join(timeout=2.0)
    node.destroy_node()
    helper.destroy_node()


# ── Tests ─────────────────────────────────────────────────────────────────────

class TestImuManagerNode:

    def test_identity_quaternion_gives_zero_angles(self, imu_fixture):
        """w=1, x=y=z=0 is the identity rotation: roll=pitch=yaw=0."""
        results, imu_pub = imu_fixture
        imu_pub.publish(_make_imu(w=1.0, x=0.0, y=0.0, z=0.0))

        assert _wait_for(
            lambda: all(results[k] is not None for k in ('roll', 'pitch', 'heading'))
        ), 'Timed out waiting for roll/pitch/heading'

        assert abs(results['roll']) < 1e-6
        assert abs(results['pitch']) < 1e-6
        assert abs(results['heading']) < 1e-6

    def test_90_degree_pitch_up(self, imu_fixture):
        """Rotate 90 deg around Y axis: pitch should equal pi/2."""
        results, imu_pub = imu_fixture
        # Half-angle rotation around Y: w=cos(45 deg), y=sin(45 deg)
        half = math.pi / 4
        imu_pub.publish(_make_imu(w=math.cos(half), y=math.sin(half)))

        assert _wait_for(
            lambda: results['pitch'] is not None and abs(results['pitch']) > 0.1
        ), 'Timed out waiting for non-zero pitch'
        assert abs(results['pitch'] - math.pi / 2) < 0.001

    def test_90_degree_roll(self, imu_fixture):
        """Rotate 90 deg around X axis: roll should equal pi/2."""
        results, imu_pub = imu_fixture
        half = math.pi / 4
        imu_pub.publish(_make_imu(w=math.cos(half), x=math.sin(half)))

        assert _wait_for(
            lambda: results['roll'] is not None and abs(results['roll']) > 0.1
        ), 'Timed out waiting for non-zero roll'
        assert abs(results['roll'] - math.pi / 2) < 0.001

    def test_180_degree_yaw_gives_180_heading(self, imu_fixture):
        """Rotate 180 deg around Z axis: heading should be 180 degrees."""
        results, imu_pub = imu_fixture
        # w=cos(90 deg)=0, z=sin(90 deg)=1
        imu_pub.publish(_make_imu(w=0.0, z=1.0))

        assert _wait_for(lambda: results['heading'] is not None), \
            'Timed out waiting for /glider/heading_deg'
        assert abs(results['heading'] - 180.0) < 0.1

    def test_heading_always_in_0_to_360_range(self, imu_fixture):
        """Heading must be in [0, 360) regardless of yaw direction."""
        results, imu_pub = imu_fixture
        imu_pub.publish(_make_imu(w=1.0))  # identity — heading = 0

        assert _wait_for(lambda: results['heading'] is not None)
        assert 0.0 <= results['heading'] < 360.0

    def test_pitch_rate_passthrough(self, imu_fixture):
        """angular_velocity.y must be passed through as pitch_rate unchanged."""
        results, imu_pub = imu_fixture
        imu_pub.publish(_make_imu(w=1.0, pitch_rate=0.75))

        assert _wait_for(lambda: results['pitch_rate'] is not None), \
            'Timed out waiting for /glider/pitch_rate_rad_s'
        assert abs(results['pitch_rate'] - 0.75) < 1e-6

    def test_negative_pitch_rate(self, imu_fixture):
        results, imu_pub = imu_fixture
        imu_pub.publish(_make_imu(w=1.0, pitch_rate=-0.3))

        assert _wait_for(lambda: results['pitch_rate'] is not None)
        assert abs(results['pitch_rate'] - (-0.3)) < 1e-6

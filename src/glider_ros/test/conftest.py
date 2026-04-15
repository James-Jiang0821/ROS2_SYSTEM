"""
Session-scoped rclpy init/shutdown shared across all integration test modules.
"""
import pytest
import rclpy


@pytest.fixture(scope='session', autouse=True)
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()

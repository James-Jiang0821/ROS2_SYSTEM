from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glider_ros',
            executable='test_iridium_signal_node',
            name='test_iridium_signal_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='gnss_maxm10s_i2c_node',
            name='gnss_maxm10s_i2c_node',
            output='screen'
        ),
    ])
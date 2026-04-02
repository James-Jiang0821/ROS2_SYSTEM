from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glider_ros',
            executable='iridium_signal_node',
            name='iridium_signal_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='gnss_maxm10s_i2c_node',
            name='gnss_maxm10s_i2c_node',
            output='screen'
        ),
    ])
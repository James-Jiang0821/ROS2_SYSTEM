from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glider_ros',
            executable='communication_iridium_node',
            name='communication_iridium_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='gnss_maxm10s_i2c_node',
            name='gnss_maxm10s_i2c_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='fake_safety_node',
            name='fake_safety_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='telemetry_manager_node',
            name='telemetry_manager_node',
            output='screen'
        ),
    ])
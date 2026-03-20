from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glider_ros',
            executable='sonar_ping_node',
            name='sonar_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='fake_safety_node',
            name='fake_safety_node',
            output='screen'
        ),
    ])
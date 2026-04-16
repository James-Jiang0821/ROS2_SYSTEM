import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    controller_params = os.path.join(
        get_package_share_directory('glider_ros'),
        'config', 'controller_params.yaml'
    )

    return LaunchDescription([
        # --- Managers ---
        Node(
            package='glider_ros',
            executable='can_bridge_node',
            name='can_bridge_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='telemetry_manager_node',
            name='telemetry_manager_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='state_manager_node',
            name='state_manager_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='imu_manager_node',
            name='imu_manager_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='sonar_manager_node',
            name='sonar_manager_node',
            output='screen'
        ),

        # --- Safety ---
        Node(
            package='glider_ros',
            executable='safety_node',
            name='safety_node',
            output='screen'
        ),

        # --- Controllers ---
        LifecycleNode(
            package='glider_ros',
            executable='controller_node',
            name='controller_node',
            namespace='',
            parameters=[controller_params],
            output='screen'
        ),
    ])

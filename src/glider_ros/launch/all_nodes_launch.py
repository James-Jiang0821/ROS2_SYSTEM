from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    return LaunchDescription([
        # --- Drivers ---
        Node(
            package='glider_ros',
            executable='imu_bno085_node',
            name='imu_bno085_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='gnss_maxm10s_i2c_node',
            name='gnss_maxm10s_i2c_node',
            output='screen'
        ),
        LifecycleNode(
            package='glider_ros',
            executable='communication_iridium_node',
            name='communication_iridium_node',
            namespace='',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='sonar_ping_node',
            name='sonar_ping_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='pressure_keller_node',
            name='pressure_keller_node',
            output='screen'
        ),

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
        Node(
            package='glider_ros',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    return LaunchDescription([
        # Iridium starts in 'unconfigured' state; state_manager_node configures
        # it when needed via the /communication_iridium_node/change_state service.
        LifecycleNode(
            package='glider_ros',
            executable='communication_iridium_node',
            name='communication_iridium_node',
            namespace='',
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
        Node(
            package='glider_ros',
            executable='state_manager_node',
            name='state_manager_node',
            output='screen'
        ),
    ])

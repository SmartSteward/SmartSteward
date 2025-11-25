"""Launch file for IoT manager node."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for IoT manager."""
    # Get configuration file path
    config_file = PathJoinSubstitution([
        FindPackageShare('iot_manager'),
        'config',
        'iot_manager.yaml'
    ])

    # Create IoT manager node
    iot_manager_node = Node(
        package='iot_manager',
        executable='iot_manager_node',
        name='iot_manager_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        iot_manager_node,
    ])

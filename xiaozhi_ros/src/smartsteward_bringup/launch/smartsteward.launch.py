"""
Main launch file for SmartSteward robot system.

This launch file starts all the core nodes for the SmartSteward robot:
- Motion controller for robot movement
- IoT manager for smart home device control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the main launch description."""
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 communication'
    )

    # Include motion control launch file
    motion_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('motion_control'),
                'launch',
                'motion_control.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
        }.items()
    )

    # Include IoT manager launch file
    iot_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iot_manager'),
                'launch',
                'iot_manager.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        serial_port_arg,
        motion_control_launch,
        iot_manager_launch,
    ])

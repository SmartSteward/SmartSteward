"""Launch file for motion control node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for motion control."""
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 communication'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )

    # Get configuration file path
    config_file = PathJoinSubstitution([
        FindPackageShare('motion_control'),
        'config',
        'motion_control.yaml'
    ])

    # Create motion controller node
    motion_controller_node = Node(
        package='motion_control',
        executable='motion_controller_node',
        name='motion_controller_node',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
            }
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        motion_controller_node,
    ])

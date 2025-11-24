#!/usr/bin/env python3
"""
Motion Controller Node for SmartSteward Robot.

This node handles communication with the STM32 microcontroller via serial port
to control the robot's movement. It subscribes to velocity commands and publishes
the current motion state.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading


class SerialController:
    """Handles serial communication with STM32."""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """
        Initialize serial connection.

        Args:
            port: Serial port path
            baudrate: Communication baudrate
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        self.command_map = {
            'go': b'\x41',
            'right': b'\x43',
            'back': b'\x45',
            'left': b'\x47',
            'stop': b'\x5a',
            'speedup': b'\x58',
            'speeddown': b'\x59',
        }

    def connect(self, logger=None):
        """Establish serial connection."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.connected = True
            return True
        except serial.SerialException as e:
            if logger:
                logger.error(f"Failed to connect to serial port {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def send_command(self, command: str, logger=None):
        """
        Send motion command to STM32.

        Args:
            command: Command string (go, back, left, right, stop, speedup, speeddown)
            logger: Optional ROS logger for error reporting

        Returns:
            bool: True if command sent successfully
        """
        if not self.connected or not self.ser or not self.ser.is_open:
            return False

        if command not in self.command_map:
            if logger:
                logger.warn(f"Unknown command: {command}")
            return False

        try:
            self.ser.write(self.command_map[command])
            return True
        except serial.SerialException as e:
            if logger:
                logger.error(f"Failed to send command: {e}")
            return False


class MotionControllerNode(Node):
    """ROS2 node for robot motion control."""

    def __init__(self):
        """Initialize the motion controller node."""
        super().__init__('motion_controller_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        publish_rate = self.get_parameter('publish_rate').value

        # Initialize serial controller
        self.serial_controller = SerialController(serial_port, baudrate)
        if not self.serial_controller.connect(logger=self.get_logger()):
            self.get_logger().warn(
                f'Failed to connect to serial port {serial_port}. '
                'Will retry on command reception.'
            )

        # Current state
        self.current_state = 'stop'
        self.state_lock = threading.Lock()

        # Create subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for motion state
        self.state_pub = self.create_publisher(
            String,
            'motion_state',
            10
        )

        # Create timer for publishing state
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_state
        )

        self.get_logger().info('Motion controller node started')

    def cmd_vel_callback(self, msg: Twist):
        """
        Process velocity command.

        Converts Twist message to motion commands for the robot.

        Args:
            msg: Twist message with linear and angular velocities
        """
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Determine motion command based on velocities
        command = self._velocity_to_command(linear_x, angular_z)

        # Send command
        if self.serial_controller.send_command(command, logger=self.get_logger()):
            with self.state_lock:
                self.current_state = command
            self.get_logger().debug(f'Sent command: {command}')
        else:
            self.get_logger().warn(f'Failed to send command: {command}')

    def _velocity_to_command(self, linear_x: float, angular_z: float) -> str:
        """
        Convert velocity to motion command.

        Args:
            linear_x: Forward/backward velocity
            angular_z: Rotational velocity

        Returns:
            str: Motion command
        """
        # Simple threshold-based control
        linear_threshold = 0.1
        angular_threshold = 0.1

        if abs(linear_x) < linear_threshold and abs(angular_z) < angular_threshold:
            return 'stop'
        elif abs(angular_z) > abs(linear_x):
            # Rotation dominant
            return 'right' if angular_z < 0 else 'left'
        else:
            # Translation dominant
            return 'go' if linear_x > 0 else 'back'

    def publish_state(self):
        """Publish current motion state."""
        msg = String()
        with self.state_lock:
            msg.data = self.current_state
        self.state_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources before node shutdown."""
        self.serial_controller.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MotionControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

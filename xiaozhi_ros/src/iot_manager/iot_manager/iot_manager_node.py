#!/usr/bin/env python3
"""
IoT Manager Node for SmartSteward Robot.

This node manages smart home IoT devices like lights, sensors, and other
smart home equipment. It provides services for device control and status queries.
"""

import rclpy
from rclpy.node import Node
from smartsteward_interfaces.msg import DeviceState
from smartsteward_interfaces.srv import DeviceControl, DeviceStatus
import threading
from typing import Dict


class Device:
    """Represents an IoT device."""

    def __init__(self, device_id: str, device_type: str):
        """
        Initialize device.

        Args:
            device_id: Unique device identifier
            device_type: Type of device (lamp, sensor, etc.)
        """
        self.device_id = device_id
        self.device_type = device_type
        self.state = 'unknown'
        self.value = 0.0
        self.properties = {}

    def to_msg(self) -> DeviceState:
        """Convert device to ROS message."""
        msg = DeviceState()
        msg.device_id = self.device_id
        msg.device_type = self.device_type
        msg.state = self.state
        msg.value = self.value
        msg.properties = [f"{k}={v}" for k, v in self.properties.items()]
        return msg


class IoTManagerNode(Node):
    """ROS2 node for IoT device management."""

    def __init__(self):
        """Initialize the IoT manager node."""
        super().__init__('iot_manager_node')

        # Device registry
        self.devices: Dict[str, Device] = {}
        self.devices_lock = threading.Lock()

        # Initialize default devices (lamp, car)
        self._initialize_devices()

        # Create services
        self.control_service = self.create_service(
            DeviceControl,
            'device_control',
            self.handle_device_control
        )

        self.status_service = self.create_service(
            DeviceStatus,
            'device_status',
            self.handle_device_status
        )

        # Create publisher for device state updates
        self.state_pub = self.create_publisher(
            DeviceState,
            'device_state',
            10
        )

        self.get_logger().info('IoT manager node started')

    def _initialize_devices(self):
        """Initialize default devices."""
        # Add lamp device
        lamp = Device('lamp_001', 'lamp')
        lamp.state = 'off'
        lamp.value = 0.0  # brightness
        self.devices['lamp_001'] = lamp

        # Add car device (linked to motion controller)
        car = Device('car_001', 'car')
        car.state = 'stop'
        self.devices['car_001'] = car

        # Add temperature sensor
        temp_sensor = Device('temp_001', 'temperature_sensor')
        temp_sensor.state = 'active'
        temp_sensor.value = 25.0  # temperature in Celsius
        self.devices['temp_001'] = temp_sensor

        self.get_logger().info(f'Initialized {len(self.devices)} devices')

    def handle_device_control(self, request, response):
        """
        Handle device control service request.

        Args:
            request: DeviceControl request
            response: DeviceControl response

        Returns:
            DeviceControl response
        """
        device_id = request.device_id
        command = request.command

        self.get_logger().info(
            f'Control request: device={device_id}, command={command}'
        )

        with self.devices_lock:
            if device_id not in self.devices:
                response.success = False
                response.message = f'Device {device_id} not found'
                response.new_state = ''
                return response

            device = self.devices[device_id]

            # Execute command based on device type and command
            try:
                if device.device_type == 'lamp':
                    self._control_lamp(device, command, request.parameters)
                elif device.device_type == 'car':
                    self._control_car(device, command, request.parameters)
                else:
                    response.success = False
                    response.message = f'Unsupported device type: {device.device_type}'
                    return response

                response.success = True
                response.message = f'Command executed successfully'
                response.new_state = device.state

                # Publish state update
                self.state_pub.publish(device.to_msg())

            except Exception as e:
                response.success = False
                response.message = f'Error executing command: {str(e)}'
                response.new_state = device.state

        return response

    def _control_lamp(self, device: Device, command: str, parameters: list):
        """Control lamp device."""
        if command == 'turn_on':
            device.state = 'on'
            device.value = 100.0
        elif command == 'turn_off':
            device.state = 'off'
            device.value = 0.0
        elif command == 'set_brightness':
            # Parse brightness from parameters
            brightness = 50.0
            for param in parameters:
                if param.startswith('brightness='):
                    brightness = float(param.split('=')[1])
            device.state = 'on' if brightness > 0 else 'off'
            device.value = max(0.0, min(100.0, brightness))
        else:
            raise ValueError(f'Unknown lamp command: {command}')

    def _control_car(self, device: Device, command: str, parameters: list):
        """Control car device."""
        valid_states = ['go', 'back', 'left', 'right', 'stop', 'speedup', 'speeddown']
        if command in valid_states:
            device.state = command
        else:
            raise ValueError(f'Unknown car command: {command}')

    def handle_device_status(self, request, response):
        """
        Handle device status service request.

        Args:
            request: DeviceStatus request
            response: DeviceStatus response

        Returns:
            DeviceStatus response
        """
        device_id = request.device_id

        with self.devices_lock:
            if device_id == '':
                # Return all devices
                response.success = True
                response.message = f'Found {len(self.devices)} devices'
                response.devices = [device.to_msg() for device in self.devices.values()]
            elif device_id in self.devices:
                # Return specific device
                response.success = True
                response.message = 'Device found'
                response.devices = [self.devices[device_id].to_msg()]
            else:
                response.success = False
                response.message = f'Device {device_id} not found'
                response.devices = []

        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = IoTManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

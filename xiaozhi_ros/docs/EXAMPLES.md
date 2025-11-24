# SmartSteward ROS 使用示例

本文档提供了使用SmartSteward ROS系统的实际示例。

## 示例1：控制机器人移动

### Python客户端示例

创建一个Python脚本来控制机器人移动：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def move_forward(self, duration=2.0):
        """让机器人前进指定时间"""
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop()
    
    def turn_left(self, duration=1.0):
        """让机器人左转指定时间"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.stop()
    
    def stop(self):
        """停止机器人"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = RobotController()
    
    # 执行一系列移动
    print("前进2秒...")
    controller.move_forward(2.0)
    
    print("左转1秒...")
    controller.turn_left(1.0)
    
    print("再前进2秒...")
    controller.move_forward(2.0)
    
    print("停止")
    controller.stop()
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 命令行示例

```bash
# 前进
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 后退
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 右转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 示例2：控制智能家居设备

### Python客户端示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smartsteward_interfaces.srv import DeviceControl, DeviceStatus

class SmartHomeController(Node):
    def __init__(self):
        super().__init__('smarthome_controller')
        self.control_client = self.create_client(DeviceControl, 'device_control')
        self.status_client = self.create_client(DeviceStatus, 'device_status')
        
        # 等待服务可用
        while not self.control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待device_control服务...')
        while not self.status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待device_status服务...')
    
    def control_device(self, device_id, command, parameters=[]):
        """控制设备"""
        request = DeviceControl.Request()
        request.device_id = device_id
        request.command = command
        request.parameters = parameters
        
        future = self.control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'设备 {device_id}: {response.message}, 新状态: {response.new_state}'
            )
            return response.success
        else:
            self.get_logger().error('服务调用失败')
            return False
    
    def get_device_status(self, device_id=''):
        """查询设备状态"""
        request = DeviceStatus.Request()
        request.device_id = device_id
        
        future = self.status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            return response.devices
        else:
            self.get_logger().error('服务调用失败')
            return []

def main():
    rclpy.init()
    controller = SmartHomeController()
    
    # 查询所有设备
    print("\n查询所有设备:")
    devices = controller.get_device_status()
    for device in devices:
        print(f"  {device.device_id}: {device.device_type} - {device.state}")
    
    # 控制灯光
    print("\n打开灯...")
    controller.control_device('lamp_001', 'turn_on')
    
    print("\n设置灯光亮度为75%...")
    controller.control_device('lamp_001', 'set_brightness', ['brightness=75'])
    
    print("\n关闭灯...")
    controller.control_device('lamp_001', 'turn_off')
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 命令行示例

```bash
# 查询所有设备状态
ros2 service call /device_status smartsteward_interfaces/srv/DeviceStatus "{device_id: ''}"

# 打开灯
ros2 service call /device_control smartsteward_interfaces/srv/DeviceControl "{device_id: 'lamp_001', command: 'turn_on', parameters: []}"

# 设置灯光亮度
ros2 service call /device_control smartsteward_interfaces/srv/DeviceControl "{device_id: 'lamp_001', command: 'set_brightness', parameters: ['brightness=50']}"

# 关闭灯
ros2 service call /device_control smartsteward_interfaces/srv/DeviceControl "{device_id: 'lamp_001', command: 'turn_off', parameters: []}"

# 查询特定设备状态
ros2 service call /device_status smartsteward_interfaces/srv/DeviceStatus "{device_id: 'lamp_001'}"
```

## 示例3：监控机器人状态

### Python订阅者示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from smartsteward_interfaces.msg import DeviceState

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        
        # 订阅运动状态
        self.motion_sub = self.create_subscription(
            String,
            'motion_state',
            self.motion_callback,
            10
        )
        
        # 订阅设备状态
        self.device_sub = self.create_subscription(
            DeviceState,
            'device_state',
            self.device_callback,
            10
        )
        
        self.get_logger().info('状态监控器已启动')
    
    def motion_callback(self, msg):
        self.get_logger().info(f'机器人运动状态: {msg.data}')
    
    def device_callback(self, msg):
        self.get_logger().info(
            f'设备状态更新: {msg.device_id} ({msg.device_type}) - '
            f'{msg.state}, 值: {msg.value}'
        )

def main():
    rclpy.init()
    monitor = StatusMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 命令行监控

```bash
# 监控运动状态
ros2 topic echo /motion_state

# 监控设备状态
ros2 topic echo /device_state

# 监控速度命令
ros2 topic echo /cmd_vel
```

## 示例4：创建简单的自动巡逻程序

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PatrolRobot(Node):
    def __init__(self):
        super().__init__('patrol_robot')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 巡逻序列
        self.sequence = [
            ('forward', 3.0),   # 前进3秒
            ('turn_left', 1.5), # 左转1.5秒
            ('forward', 3.0),   # 前进3秒
            ('turn_left', 1.5), # 左转1.5秒
        ]
        
        self.current_step = 0
        self.step_start_time = time.time()
        
    def timer_callback(self):
        if self.current_step >= len(self.sequence):
            self.get_logger().info('巡逻完成')
            self.stop()
            raise KeyboardInterrupt
        
        action, duration = self.sequence[self.current_step]
        elapsed = time.time() - self.step_start_time
        
        if elapsed < duration:
            msg = Twist()
            if action == 'forward':
                msg.linear.x = 0.3
            elif action == 'turn_left':
                msg.angular.z = 0.5
            self.publisher.publish(msg)
        else:
            # 进入下一步
            self.current_step += 1
            self.step_start_time = time.time()
            self.get_logger().info(f'步骤 {self.current_step}/{len(self.sequence)}')
    
    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

def main():
    rclpy.init()
    patrol = PatrolRobot()
    
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    finally:
        patrol.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 示例5：使用launch文件启动多个示例

创建自定义launch文件 `examples.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动系统节点
        Node(
            package='motion_control',
            executable='motion_controller_node',
            name='motion_controller_node',
            output='screen'
        ),
        Node(
            package='iot_manager',
            executable='iot_manager_node',
            name='iot_manager_node',
            output='screen'
        ),
        # 启动状态监控
        Node(
            package='examples',
            executable='status_monitor',
            name='status_monitor',
            output='screen'
        ),
    ])
```

## 调试技巧

### 1. 查看话题带宽

```bash
ros2 topic bw /cmd_vel
```

### 2. 查看话题频率

```bash
ros2 topic hz /motion_state
```

### 3. 录制和回放

```bash
# 录制所有话题
ros2 bag record -a

# 回放
ros2 bag play <bag_file>

# 录制特定话题
ros2 bag record /cmd_vel /motion_state
```

### 4. 可视化节点图

```bash
# 安装graphviz
sudo apt install graphviz

# 生成节点图
ros2 run rqt_graph rqt_graph
```

## 常见问题解决

### 问题1：节点无法通信

```bash
# 检查节点是否运行
ros2 node list

# 检查话题
ros2 topic list

# 检查节点连接
ros2 node info /motion_controller_node
```

### 问题2：串口连接失败

```bash
# 检查串口设备
ls -l /dev/ttyACM*

# 检查用户权限
groups $USER

# 添加到dialout组
sudo usermod -a -G dialout $USER
```

## 进阶示例

### 使用参数服务器

```python
# 获取参数
serial_port = self.get_parameter('serial_port').value

# 动态修改参数
ros2 param set /motion_controller_node serial_port /dev/ttyUSB0
```

### 使用tf变换

如果要添加机器人位置追踪，可以发布tf变换：

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 在节点中
self.tf_broadcaster = TransformBroadcaster(self)

# 发布变换
t = TransformStamped()
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'odom'
t.child_frame_id = 'base_link'
# ... 设置变换
self.tf_broadcaster.sendTransform(t)
```

## 总结

这些示例展示了如何使用SmartSteward ROS系统的基本功能。您可以根据这些示例创建自己的应用程序，或者将它们组合起来创建更复杂的行为。

更多信息请参考：
- [ROS 2教程](https://docs.ros.org/en/humble/Tutorials.html)
- [ARCHITECTURE.md](ARCHITECTURE.md) - 系统架构详解
- [QUICKSTART.md](../QUICKSTART.md) - 快速入门指南

# 集成指南：xiaozhi与ROS的协同工作

本文档说明如何将原有的xiaozhi Python代码与新建的ROS系统协同工作。

## 架构概述

SmartSteward系统现在采用混合架构：

```
┌─────────────────────────────────────────────────────────────┐
│                    SmartSteward System                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────┐         ┌──────────────────┐         │
│  │  xiaozhi (原系统) │         │   ROS 2 系统      │         │
│  ├──────────────────┤         ├──────────────────┤         │
│  │ - AI对话         │         │ - 运动控制        │         │
│  │ - 语音识别/合成  │◄────────┤ - 设备管理        │         │
│  │ - MCP工具        │  MQTT   │ - 传感器处理      │         │
│  │ - UI显示         │  或API  │ - 标准化接口      │         │
│  │ - 高级AI功能     │         │ - 模块化设计      │         │
│  └──────────────────┘         └──────────────────┘         │
│           │                            │                    │
│           │                            │                    │
│           └────────────┬───────────────┘                    │
│                        │                                    │
│                        ▼                                    │
│           ┌────────────────────────┐                       │
│           │   硬件层 (Raspberry Pi)  │                       │
│           ├────────────────────────┤                       │
│           │ - STM32 (串口)          │                       │
│           │ - 音频设备              │                       │
│           │ - 摄像头                │                       │
│           │ - 传感器                │                       │
│           └────────────────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

## 两种使用模式

### 模式1：独立使用ROS系统

适合只需要基本机器人控制功能的场景。

**优点：**
- 轻量级，资源占用少
- 标准ROS接口，易于集成其他ROS包
- 适合机器人导航、SLAM等传统机器人应用

**使用方法：**
```bash
cd xiaozhi_ros
source install/setup.bash
ros2 launch smartsteward_bringup smartsteward.launch.py
```

### 模式2：ROS + xiaozhi集成使用

适合需要AI对话、语音交互等高级功能的场景。

**优点：**
- 保留xiaozhi的所有AI和语音功能
- 使用ROS管理底层硬件控制
- 模块化，各部分可独立开发和调试

**使用方法：**
1. 启动ROS系统
   ```bash
   cd xiaozhi_ros
   source install/setup.bash
   ros2 launch smartsteward_bringup smartsteward.launch.py
   ```

2. 启动xiaozhi客户端（另一个终端）
   ```bash
   cd xiaozhi
   python main.py --mode cli
   ```

## 通信桥接

为了让xiaozhi与ROS系统通信，有以下几种方案：

### 方案A：通过MQTT桥接（推荐）

xiaozhi原本支持MQTT协议，可以订阅/发布MQTT话题与ROS通信。

#### 1. 安装ROS MQTT桥接包

```bash
sudo apt install ros-humble-mqtt-client
```

#### 2. 创建MQTT-ROS桥接节点

在`xiaozhi_ros/src`下创建桥接节点，将MQTT消息转换为ROS消息。

### 方案B：通过REST API桥接

为ROS系统添加HTTP服务器，xiaozhi通过REST API调用ROS服务。

#### 示例：在IoT Manager中添加HTTP接口

```python
from fastapi import FastAPI
import uvicorn

app = FastAPI()

@app.post("/device/control")
async def control_device(device_id: str, command: str):
    # 调用ROS服务
    # ...
    return {"success": True}
```

### 方案C：Python直接调用rclpy

xiaozhi代码中可以直接引入rclpy，创建ROS节点进行通信。

#### 示例：在xiaozhi中集成ROS节点

```python
# 在xiaozhi/src/iot/things/car.py中
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CarROSBridge(Node):
    def __init__(self):
        super().__init__('car_ros_bridge')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def send_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
```

## 迁移策略

如果您想逐步将xiaozhi功能迁移到ROS：

### 第一阶段：硬件层迁移
- [x] 串口通信（STM32控制）→ motion_control包
- [x] IoT设备管理 → iot_manager包
- [ ] 传感器数据采集 → sensor_nodes包

### 第二阶段：中间层迁移
- [ ] 音频输入/输出 → audio_processing包
- [ ] 摄像头图像采集 → vision_processing包
- [ ] 导航和定位 → navigation包

### 第三阶段：应用层保留
- AI对话和语音识别保持使用xiaozhi
- MCP工具系统保持使用xiaozhi
- UI显示保持使用xiaozhi

## 推荐实践

### 开发阶段
1. ROS系统负责所有硬件I/O和底层控制
2. xiaozhi负责AI、语音、UI等高层应用
3. 通过MQTT或HTTP进行通信

### 部署阶段
1. 使用systemd管理两个系统的启动
2. ROS系统作为系统服务自启动
3. xiaozhi作为用户应用启动

### 示例systemd服务文件

**ROS系统服务** (`/etc/systemd/system/smartsteward-ros.service`):
```ini
[Unit]
Description=SmartSteward ROS System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/SmartSteward/xiaozhi_ros
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch smartsteward_bringup smartsteward.launch.py'
Restart=always

[Install]
WantedBy=multi-user.target
```

**xiaozhi服务** (`/etc/systemd/system/smartsteward-xiaozhi.service`):
```ini
[Unit]
Description=SmartSteward Xiaozhi Client
After=smartsteward-ros.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/SmartSteward/xiaozhi
ExecStart=/usr/bin/python3 main.py --mode cli
Restart=always

[Install]
WantedBy=multi-user.target
```

## 故障排查

### 问题1：xiaozhi无法连接到ROS服务
- 检查ROS节点是否正常运行：`ros2 node list`
- 检查服务是否可用：`ros2 service list`
- 检查网络连接（如果使用MQTT）

### 问题2：串口被多个程序占用
- 确保只有ROS的motion_control节点访问串口
- xiaozhi的Car类应该通过ROS发送命令，而不是直接访问串口

### 问题3：性能问题
- ROS系统和xiaozhi都比较消耗资源
- 建议Raspberry Pi 4 with 4GB+ RAM
- 考虑禁用不需要的功能

## 未来扩展

ROS系统为以下扩展提供了良好基础：

- **SLAM和导航**：使用Nav2进行室内导航
- **机械臂控制**：使用MoveIt!进行机械臂规划
- **多机器人协同**：ROS天然支持分布式系统
- **仿真环境**：使用Gazebo进行仿真测试
- **可视化工具**：使用RViz查看机器人状态

## 参考资料

- [ROS 2与MQTT集成](https://github.com/iot-salzburg/ros2_mqtt_bridge)
- [xiaozhi项目文档](../xiaozhi/README.md)
- [ROS 2教程](https://docs.ros.org/en/humble/Tutorials.html)

# SmartSteward ROS Workspace

这是SmartSteward智能管家机器人的ROS工作空间，用于组织和管理Raspberry Pi上的机器人控制代码。

## 🤖 ROS架构

本项目使用ROS 2框架来组织Raspberry Pi上的代码，提供以下优势：
- **模块化设计**：通过ROS节点实现功能解耦
- **标准化通信**：使用ROS话题、服务和动作进行节点间通信
- **易于扩展**：可以方便地添加新的功能模块
- **工具链完善**：可以使用RViz、rqt等ROS工具进行调试和可视化

## 📦 功能包说明

### motion_control
机器人运动控制包，负责与STM32通信控制小车运动。

**节点：**
- `motion_controller_node`: 接收运动命令并通过串口发送给STM32

**话题：**
- `/cmd_vel` (geometry_msgs/Twist): 订阅速度命令
- `/motion_state` (std_msgs/String): 发布当前运动状态

### iot_manager
IoT设备管理包，管理智能家居设备。

**节点：**
- `iot_manager_node`: 管理所有IoT设备的状态和控制

**服务：**
- `/device_control` (iot_interfaces/DeviceControl): 控制设备
- `/device_status` (iot_interfaces/DeviceStatus): 查询设备状态

### audio_processing
音频处理包，处理语音识别和合成。

**节点：**
- `audio_input_node`: 采集音频输入
- `audio_output_node`: 播放音频输出

**话题：**
- `/audio_input` (audio_interfaces/AudioData): 发布音频数据
- `/audio_output` (audio_interfaces/AudioData): 订阅音频播放数据

### voice_interaction
语音交互包，处理AI对话和语音控制。

**节点：**
- `voice_interaction_node`: 处理语音识别、AI对话和命令执行

**话题：**
- `/voice_command` (std_msgs/String): 发布识别的语音命令
- `/ai_response` (std_msgs/String): 发布AI回复

### smartsteward_interfaces
自定义消息、服务和动作接口定义包。

## 🚀 快速开始

### 安装ROS 2

首先确保已安装ROS 2（推荐Humble或更新版本）：

```bash
# Ubuntu 22.04安装ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 构建工作空间

```bash
cd xiaozhi_ros
colcon build
source install/setup.bash
```

### 启动系统

启动所有节点：
```bash
ros2 launch smartsteward_bringup smartsteward.launch.py
```

或单独启动各个节点：
```bash
# 启动运动控制节点
ros2 run motion_control motion_controller_node

# 启动IoT管理节点
ros2 run iot_manager iot_manager_node

# 启动语音交互节点
ros2 run voice_interaction voice_interaction_node
```

## 🛠️ 开发指南

### 创建新功能包

```bash
cd src
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy
```

### 运行测试

```bash
colcon test
colcon test-result --verbose
```

### 代码格式化

```bash
# Python代码格式化
ament_flake8 src/
ament_pep257 src/
```

## 📝 配置文件

配置文件位于各个功能包的`config`目录下，使用YAML格式。

## 🔗 相关链接

- [ROS 2 文档](https://docs.ros.org/en/humble/)
- [原始xiaozhi项目](../xiaozhi/)
- [SmartSteward项目主页](../README.md)

## 📄 许可证

MIT License

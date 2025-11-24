# SmartSteward ROS 快速入门指南

本指南将帮助您快速设置和运行SmartSteward机器人的ROS系统。

## 系统要求

- **操作系统**：Ubuntu 22.04 LTS (推荐)
- **ROS版本**：ROS 2 Humble或更新版本
- **Python**：Python 3.10+
- **硬件**：Raspberry Pi 3/4，STM32开发板

## 安装步骤

### 1. 安装ROS 2

如果还没有安装ROS 2，请按照以下步骤安装：

```bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS 2 apt源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 安装ROS 2
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. 安装依赖

```bash
cd xiaozhi_ros
./install_dependencies.sh
```

这个脚本会安装：
- colcon构建工具
- rosdep依赖管理工具
- Python依赖（pyserial等）
- 所有ROS包的依赖

### 3. 构建工作空间

```bash
./build.sh
```

构建过程包括：
- 编译自定义消息接口
- 构建所有功能包
- 创建安装目录

### 4. 配置环境

```bash
source install/setup.bash
```

或使用提供的脚本：
```bash
source setup_environment.sh
```

## 运行系统

### 启动完整系统

启动所有节点（运动控制 + IoT管理）：

```bash
ros2 launch smartsteward_bringup smartsteward.launch.py
```

可以指定串口设备：

```bash
ros2 launch smartsteward_bringup smartsteward.launch.py serial_port:=/dev/ttyUSB0
```

### 单独启动节点

**启动运动控制节点：**
```bash
ros2 run motion_control motion_controller_node
```

**启动IoT管理节点：**
```bash
ros2 run iot_manager iot_manager_node
```

## 基本使用

### 控制机器人运动

发送速度命令：
```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

查看运动状态：
```bash
ros2 topic echo /motion_state
```

### 控制IoT设备

**控制灯光：**
```bash
# 打开灯
ros2 service call /device_control smartsteward_interfaces/srv/DeviceControl "{device_id: 'lamp_001', command: 'turn_on', parameters: []}"

# 关闭灯
ros2 service call /device_control smartsteward_interfaces/srv/DeviceControl "{device_id: 'lamp_001', command: 'turn_off', parameters: []}"

# 设置亮度
ros2 service call /device_control smartsteward_interfaces/srv/DeviceControl "{device_id: 'lamp_001', command: 'set_brightness', parameters: ['brightness=75']}"
```

**查询设备状态：**
```bash
# 查询所有设备
ros2 service call /device_status smartsteward_interfaces/srv/DeviceStatus "{device_id: ''}"

# 查询特定设备
ros2 service call /device_status smartsteward_interfaces/srv/DeviceStatus "{device_id: 'lamp_001'}"
```

## 调试工具

### 查看节点列表
```bash
ros2 node list
```

### 查看话题列表
```bash
ros2 topic list
```

### 查看服务列表
```bash
ros2 service list
```

### 查看节点信息
```bash
ros2 node info /motion_controller_node
ros2 node info /iot_manager_node
```

### 可视化工具

**rqt_graph - 查看节点连接图：**
```bash
rqt_graph
```

**rqt_console - 查看日志：**
```bash
rqt_console
```

## 常见问题

### 1. 串口权限问题

如果无法访问串口设备，将用户添加到dialout组：
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 2. 找不到串口设备

查看可用串口：
```bash
ls -l /dev/tty*
```

常见设备名：
- `/dev/ttyACM0` - Arduino/STM32（USB CDC）
- `/dev/ttyUSB0` - USB转串口适配器
- `/dev/ttyAMA0` - Raspberry Pi GPIO串口

### 3. 构建失败

清理并重新构建：
```bash
rm -rf build install log
./build.sh
```

### 4. 依赖问题

重新安装依赖：
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 下一步

- 查看 [README.md](README.md) 了解架构详情
- 添加新的功能包扩展系统
- 集成摄像头、传感器等硬件
- 编写自定义节点实现新功能

## 参考资源

- [ROS 2 文档](https://docs.ros.org/en/humble/)
- [ROS 2 教程](https://docs.ros.org/en/humble/Tutorials.html)
- [原始xiaozhi项目](../xiaozhi/)

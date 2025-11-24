# SmartSteward ROS 架构详解

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SmartSteward ROS System                       │
└─────────────────────────────────────────────────────────────────────┘
                                │
                ┌───────────────┴───────────────┐
                │                               │
        ┌───────▼────────┐            ┌────────▼────────┐
        │ Motion Control │            │  IoT Manager    │
        │     Node       │            │      Node       │
        └───────┬────────┘            └────────┬────────┘
                │                               │
    ┌───────────┼───────────┐      ┌───────────┼──────────┐
    │           │           │      │           │          │
    ▼           ▼           ▼      ▼           ▼          ▼
/cmd_vel   /motion_   Serial  /device_   /device_   /device_
 (sub)      state    Port     control    status     state
           (pub)              (srv)      (srv)      (pub)
```

## 节点详细说明

### 1. Motion Controller Node

**功能：**机器人运动控制

**节点名称：**`/motion_controller_node`

**订阅的话题：**
- `/cmd_vel` (geometry_msgs/Twist)
  - 接收速度命令
  - 线性速度：linear.x (前进/后退)
  - 角速度：angular.z (左转/右转)

**发布的话题：**
- `/motion_state` (std_msgs/String)
  - 发布当前运动状态
  - 可能的值：go, back, left, right, stop, speedup, speeddown

**硬件接口：**
- 串口通信：/dev/ttyACM0 (默认)
- 波特率：115200
- 协议：单字节命令

**参数：**
- `serial_port`: 串口设备路径
- `baudrate`: 通信波特率
- `publish_rate`: 状态发布频率（Hz）

### 2. IoT Manager Node

**功能：**智能家居设备管理

**节点名称：**`/iot_manager_node`

**发布的话题：**
- `/device_state` (smartsteward_interfaces/DeviceState)
  - 发布设备状态变化
  - 包含设备ID、类型、状态、数值等信息

**提供的服务：**
- `/device_control` (smartsteward_interfaces/DeviceControl)
  - 控制设备
  - 请求：device_id, command, parameters
  - 响应：success, message, new_state

- `/device_status` (smartsteward_interfaces/DeviceStatus)
  - 查询设备状态
  - 请求：device_id (空字符串查询所有设备)
  - 响应：success, message, devices[]

**管理的设备：**
- lamp_001: 智能灯
- car_001: 机器人小车
- temp_001: 温度传感器

## 消息接口定义

### AudioData.msg

```
std_msgs/Header header
int32 sample_rate      # 采样率（Hz）
int32 channels         # 声道数
string encoding        # 编码格式
uint8[] data          # 音频数据
```

### DeviceState.msg

```
string device_id       # 设备ID
string device_type     # 设备类型
string state          # 当前状态
float64 value         # 数值
string[] properties   # 其他属性
std_msgs/Header header
```

## 服务接口定义

### DeviceControl.srv

**请求：**
```
string device_id       # 设备ID
string command         # 命令
string[] parameters    # 参数列表
```

**响应：**
```
bool success          # 是否成功
string message        # 消息
string new_state      # 新状态
```

### DeviceStatus.srv

**请求：**
```
string device_id      # 设备ID（空=全部）
```

**响应：**
```
bool success          # 是否成功
string message        # 消息
DeviceState[] devices # 设备列表
```

## 通信流程

### 场景1：语音控制机器人前进

```
用户 → "小智，向前走"
    ↓
xiaozhi语音识别
    ↓
xiaozhi解析命令
    ↓
发布到/cmd_vel
    ↓
motion_controller_node接收
    ↓
发送0x41到串口
    ↓
STM32执行电机控制
    ↓
机器人前进
```

### 场景2：控制智能灯

```
用户 → "打开灯"
    ↓
xiaozhi语音识别
    ↓
调用/device_control服务
    ↓
iot_manager_node处理
    ↓
更新灯状态为"on"
    ↓
发布/device_state
    ↓
xiaozhi接收状态更新
    ↓
语音反馈"已打开灯"
```

## 数据流图

```
┌──────────────┐
│ 外部命令源    │
│ - xiaozhi   │
│ - Web界面    │
│ - 手机App    │
└──────┬───────┘
       │
       ▼
┌──────────────┐     ┌──────────────┐
│  ROS Topics  │────▶│  ROS Nodes   │
│  ROS Services│     │              │
└──────────────┘     └──────┬───────┘
                            │
                            ▼
                    ┌──────────────┐
                    │  Hardware    │
                    │ - STM32      │
                    │ - Sensors    │
                    │ - Actuators  │
                    └──────────────┘
```

## 扩展性设计

### 添加新设备类型

1. 在`iot_manager_node.py`中添加设备初始化：
```python
sensor = Device('sensor_001', 'ultrasonic')
sensor.state = 'active'
sensor.value = 0.0  # distance in cm
```

2. 添加控制逻辑：
```python
def _control_ultrasonic(self, device, command, parameters):
    if command == 'read':
        # 读取传感器数据
        device.value = read_sensor()
```

### 添加新节点

1. 创建新包：
```bash
cd src
ros2 pkg create --build-type ament_python my_new_node --dependencies rclpy
```

2. 编写节点代码

3. 更新bringup launch文件

### 添加新消息类型

1. 在`smartsteward_interfaces/msg/`创建新的.msg文件

2. 更新`CMakeLists.txt`：
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NewMessage.msg"
  ...
)
```

3. 重新编译

## 性能考虑

### 话题频率建议

- `/cmd_vel`: 10-20 Hz
- `/motion_state`: 10 Hz
- `/device_state`: 1 Hz (按需发布)

### 资源使用

**Raspberry Pi 3B+ (1GB RAM):**
- ROS系统：~200MB RAM
- motion_control: ~20MB
- iot_manager: ~30MB

**Raspberry Pi 4B (4GB RAM):**
- 足够运行完整系统
- 可以同时运行ROS + xiaozhi + 其他服务

## 调试技巧

### 实时监控节点通信

```bash
# 监控所有话题
ros2 topic list -v

# 实时查看话题数据
ros2 topic echo /cmd_vel
ros2 topic echo /motion_state

# 查看服务调用
ros2 service call /device_status smartsteward_interfaces/srv/DeviceStatus "{device_id: ''}"
```

### 使用rqt工具

```bash
# 图形化查看节点连接
rqt_graph

# 图形化发布话题
rqt_publisher

# 查看话题数据
rqt_plot
```

### 日志分析

```bash
# 查看节点日志
ros2 run motion_control motion_controller_node --ros-args --log-level debug

# 查看系统日志
ros2 run rqt_console rqt_console
```

## 安全考虑

1. **串口访问权限**
   - 确保运行节点的用户在dialout组中
   - 使用udev规则固定设备名称

2. **网络安全**
   - 如果暴露ROS服务到网络，使用ROS 2的安全功能
   - 考虑使用VPN或防火墙限制访问

3. **错误处理**
   - 所有节点都有异常处理
   - 串口断开时自动重连
   - 设备控制失败时返回错误信息

## 最佳实践

1. **使用launch文件**：而不是手动启动多个节点
2. **参数化配置**：使用YAML配置文件而不是硬编码
3. **命名空间**：在多机器人系统中使用命名空间
4. **日志记录**：使用ROS日志系统而不是print
5. **错误处理**：优雅地处理硬件故障和通信错误

## 参考资料

- [ROS 2设计理念](https://design.ros2.org/)
- [ROS 2最佳实践](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html)
- [Python编程指南](https://docs.ros.org/en/humble/Guides/Python-Programming.html)

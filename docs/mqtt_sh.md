# 智能家居 MQTT Topic 说明文档

本文件说明智能家居系统中使用到的 MQTT Topic，按房间和设备分类，便于开发与维护。

---

## 对于值的说明

- 开关用0/1表示（1开，0关）
- 灯光颜色用0,0,0~255,255,255表示
- 灯光亮度用0~255表示
- 灯光模式用0/1表示（1睡眠模式，0关）
- 风扇档位控制用0~5表示
- 窗户开关用-90~90表示

## 通用

| Topic | 说明 |
|------|------|
| `smarthome/door` | 家中大门的状态上报或开关控制。 |

---

## 客厅（Living Room）

| Topic | 说明 |
|------|------|
| `smarthome/livingroom/led` | 客厅主灯开关控制。 |
| `smarthome/livingroom/ledcolor` | 设置客厅灯光颜色（RGB 或预设颜色）。 |
| `smarthome/livingroom/ledmode` | 调整灯光模式（常亮、呼吸、渐变等）。 |
| `smarthome/livingroom/ledbrightness` | 调整灯光亮度。 |
| `smarthome/livingroom/humidifier` | 加湿器开关控制。 |
| `smarthome/livingroom/fan` | 风扇档位控制。 |
| `smarthome/livingroom/curtain` | 智能窗帘开关。 |
| `smarthome/livingroom/airconditioner` | 空调开关。 |

---

## 厨房（Kitchen）

| Topic | 说明 |
|------|------|
| `smarthome/kitchen/led` | 厨房照明灯开关控制。 |
| `smarthome/kitchen/ledcolor` | 厨房灯光颜色设置。 |
| `smarthome/kitchen/ledmode` | 灯光效果模式切换。 |
| `smarthome/kitchen/ledbrightness` | 灯光亮度调节。 |
| `smarthome/kitchen/fan` | 排风扇开关控制。 |

---

## 卧室（Bedroom）

| Topic | 说明 |
|------|------|
| `smarthome/bedroom/led` | 卧室主灯开关控制。 |
| `smarthome/bedroom/ledcolor` | 卧室灯光颜色设置。 |
| `smarthome/bedroom/ledmode` | 卧室灯光模式控制。 |
| `smarthome/bedroom/ledbrightness` | 灯光亮度控制。 |
| `smarthome/bedroom/humidifier` | 卧室加湿器控制。 |
| `smarthome/bedroom/windows` | 智能窗户开合角度调节。 |
| `smarthome/bedroom/fan` | 风扇档位调节。 |

---

## 浴室（Bathroom）

| Topic | 说明 |
|------|------|
| `smarthome/bathroom/led` | 浴室照明开关。 |
| `smarthome/bathroom/ledcolor` | 浴室灯光颜色调整。 |
| `smarthome/bathroom/ledmode` | 灯光模式切换。 |
| `smarthome/bathroom/ledbrightness` | 亮度调节。 |
| `smarthome/bathroom/fan` | 排风扇开关。 |

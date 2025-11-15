from typing import List

import paho.mqtt.client as mqtt

from src.iot.thing import Parameter, Thing


class Led:
    def __init__(self, publish, room_name):
        self.state = False
        self.color = "255,255,255"
        self.mode = 0
        self.brightness = 150
        self.publish = publish
        self.base_topic = f"smarthome/{room_name}"
        # self.room_name = room_name

    async def set_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.state = state
        print("set_state called with", state)
        await self.publish(f"{self.base_topic}/led", int(state))
        return {"status": "success", "message": "LED状态已更新"}

    async def set_color(self, color: str):
        self.color = color
        await self.publish(f"{self.base_topic}/ledcolor", color)
        return {"status": "success", "message": "LED颜色已更新"}

    async def set_mode(self, mode: int):
        self.mode = mode
        await self.publish(f"{self.base_topic}/ledmode", mode)
        return {"status": "success", "message": "LED模式已更新"}

    async def set_brightness(self, brightness: int):
        self.brightness = brightness
        await self.publish(f"{self.base_topic}/ledbrightness", brightness)
        return {"status": "success", "message": "LED亮度已更新"}

    async def get_state(self):
        return self.state

    async def get_color(self):
        return self.color

    async def get_mode(self):
        return self.mode

    async def get_brightness(self):
        return self.brightness


class LivingRoom:
    DOOR = "smarthome/door"
    ROOMNAME = "livingroom"

    HUMIDIFIER = "smarthome/livingroom/humidifier"
    DOOR = "smarthome/door"
    FAN = "smarthome/livingroom/fan"
    CURTAIN = "smarthome/livingroom/curtain"
    AIRCONDITIONER = "smarthome/livingroom/airconditioner"

    def __init__(self, publish):
        self.publish = publish
        self.humidifier_state = False
        self.door_state = False
        self.fan_state = 0
        self.curtain_state = False
        self.air_conditioner_state = False
        self.led = Led(publish, LivingRoom.ROOMNAME)

    async def set_humidifier_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.humidifier_state = state
        await self.publish(LivingRoom.HUMIDIFIER, int(state))
        return {"status": "success", "message": "加湿器状态已更新"}

    async def set_door_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.door_state = state
        await self.publish(LivingRoom.DOOR, int(state))
        return {"status": "success", "message": "门状态已更新"}

    async def set_fan_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.fan_state = state
        await self.publish(LivingRoom.FAN, int(state))
        return {"status": "success", "message": "风扇状态已更新"}

    async def set_curtain_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.curtain_state = state
        await self.publish(LivingRoom.CURTAIN, state)
        return {"status": "success", "message": "窗帘状态已更新"}

    async def set_air_conditioner_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.air_conditioner_state = state
        await self.publish(LivingRoom.AIRCONDITIONER, int(state))
        return {"status": "success", "message": "空调状态已更新"}

    async def get_air_conditioner_state(self):
        return self.air_conditioner_state

    async def get_humidifier_state(self):
        return self.humidifier_state

    async def get_door_state(self):
        return self.door_state

    async def get_fan_state(self):
        return self.fan_state

    async def get_curtain_state(self):
        return self.curtain_state


class Kitchen:
    ROOMNAME = "kitchen"
    FAN = "smarthome/kitchen/fan"

    def __init__(self, publish):
        self.publish = publish
        self.fan_state = False
        self.led = Led(publish, Kitchen.ROOMNAME)

    async def set_fan_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.fan_state = state
        await self.publish(Kitchen.FAN, int(state))
        return {"status": "success", "message": "风扇状态已更新"}

    async def get_fan_state(self):
        return self.fan_state


class Bedroom:
    ROOMNAME = "bedroom"
    HUMIDIFIER = "smarthome/bedroom/humidifier"
    WINDOWS = "smarthome/bedroom/windows"
    FAN = "smarthome/bedroom/fan"

    def __init__(self, publish):
        self.publish = publish
        self.humidifier_state = False
        self.fan_state = 0
        self.windows_state = 0
        self.led = Led(publish, Bedroom.ROOMNAME)

    async def set_humidifier_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.humidifier_state = state
        await self.publish(Bedroom.HUMIDIFIER, int(state))
        return {"status": "success", "message": "加湿器状态已更新"}

    async def set_fan_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.fan_state = state
        await self.publish(Bedroom.FAN, int(state))
        return {"status": "success", "message": "风扇状态已更新"}

    async def set_windows_state(self, params: List[Parameter]):
        state = params["state"].get_value()
        self.windows_state = state
        await self.publish(Bedroom.WINDOWS, int(state))
        return {"status": "success", "message": "窗户状态已更新"}

    async def get_humidifier_state(self):
        return self.humidifier_state

    async def get_fan_state(self):
        return self.fan_state

    async def get_windows_state(self):
        return self.windows_state


class Bathroom:
    ROOMNAME = "bathroom"
    FAN = "smarthome/bathroom/fan"

    def __init__(self, publish):
        self.publish = publish
        self.fan_state = False
        self.led = Led(publish, Bathroom.ROOMNAME)

    async def set_fan_state(self, state: bool):
        self.fan_state = state
        await self.publish(Bathroom.FAN, state)
        return {"status": "success", "message": "风扇状态已更新"}

    async def get_fan_state(self):
        return self.fan_state


class SmartHome(Thing):
    def __init__(self):
        super().__init__("SmartHome", "智能家居系统")
        self.mqtt = mqtt.Client()
        self.connect_mqtt()
        self.livingroom = LivingRoom(self.publish)
        self.kitchen = Kitchen(self.publish)
        self.bedroom = Bedroom(self.publish)
        self.bathroom = Bathroom(self.publish)

        properties = [
            ("livingroom_led_state", "客厅LED状态", self.livingroom.led.get_state),
            ("livingroom_led_color", "客厅LED颜色", self.livingroom.led.get_color),
            ("livingroom_led_mode", "客厅LED模式", self.livingroom.led.get_mode),
            ("livingroom_led_brightness", "客厅LED亮度", self.livingroom.led.get_brightness),
            ("livingroom_humidifier_state", "客厅加湿器状态", self.livingroom.get_humidifier_state),
            ("livingroom_door_state", "客厅门状态", self.livingroom.get_door_state),
            ("livingroom_fan_state", "客厅风扇状态", self.livingroom.get_fan_state),
            ("livingroom_curtain_state", "客厅窗帘状态", self.livingroom.get_curtain_state),
            ("livingroom_airconditioner_state", "客厅空调状态", self.livingroom.get_air_conditioner_state),
            ("kitchen_led_state", "厨房LED状态", self.kitchen.led.get_state),
            ("kitchen_led_color", "厨房LED颜色", self.kitchen.led.get_color),
            ("kitchen_led_mode", "厨房LED模式", self.kitchen.led.get_mode),
            ("kitchen_led_brightness", "厨房LED亮度", self.kitchen.led.get_brightness),
            ("kitchen_fan_state", "厨房风扇状态", self.kitchen.get_fan_state),
            ("bedroom_led_state", "卧室LED状态", self.bedroom.led.get_state),
            ("bedroom_led_color", "卧室LED颜色", self.bedroom.led.get_color),
            ("bedroom_led_mode", "卧室LED模式", self.bedroom.led.get_mode),
            ("bedroom_led_brightness", "卧室LED亮度", self.bedroom.led.get_brightness),
            ("bedroom_humidifier_state", "卧室加湿器状态", self.bedroom.get_humidifier_state),
            ("bedroom_fan_state", "卧室风扇状态", self.bedroom.get_fan_state),
            ("bedroom_windows_state", "卧室窗户状态", self.bedroom.get_windows_state),
            ("bathroom_led_state", "浴室LED状态", self.bathroom.led.get_state),
            ("bathroom_led_color", "浴室LED颜色", self.bathroom.led.get_color),
            ("bathroom_led_mode", "浴室LED模式", self.bathroom.led.get_mode),
            ("bathroom_led_brightness", "浴室LED亮度", self.bathroom.led.get_brightness),
            ("bathroom_fan_state", "浴室风扇状态", self.bathroom.get_fan_state),
        ]

        methods = [
            (
                "set_livingroom_led_state",
                "设置客厅LED状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.livingroom.led.set_state,
            ),
            (
                "set_livingroom_led_color",
                "设置客厅LED颜色",
                [Parameter("color", "颜色值0,0,0-255,255,255", "string")],
                self.livingroom.led.set_color,
            ),
            (
                "set_livingroom_led_mode",
                "设置客厅LED模式",
                [Parameter("mode", "模式编号1表示睡眠模式,0表示正常模式", "number")],
                self.livingroom.led.set_mode,
            ),
            (
                "set_livingroom_led_brightness",
                "设置客厅LED亮度",
                [Parameter("brightness", "亮度值0-255", "number")],
                self.livingroom.led.set_brightness,
            ),
            (
                "set_livingroom_humidifier_state",
                "设置客厅加湿器状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.livingroom.set_humidifier_state,
            ),
            (
                "set_livingroom_door_state",
                "设置客厅门状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.livingroom.set_door_state,
            ),
            (
                "set_livingroom_fan_state",
                "设置客厅风扇状态",
                [Parameter("state", "风扇速度0-3", "number")],
                self.livingroom.set_fan_state,
            ),
            (
                "set_livingroom_curtain_state",
                "设置客厅窗帘状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.livingroom.set_curtain_state,
            ),
            (
                "set_livingroom_airconditioner_state",
                "设置客厅空调状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.livingroom.set_air_conditioner_state,
            ),
            (
                "set_kitchen_led_state",
                "设置厨房LED状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.kitchen.led.set_state,
            ),
            (
                "set_kitchen_led_color",
                "设置厨房LED颜色",
                [Parameter("color", "颜色值0,0,0-255,255,255", "string")],
                self.kitchen.led.set_color,
            ),
            (
                "set_kitchen_led_mode",
                "设置厨房LED模式",
                [Parameter("mode", "模式编号1表示睡眠模式,0表示正常模式", "number")],
                self.kitchen.led.set_mode,
            ),
            (
                "set_kitchen_led_brightness",
                "设置厨房LED亮度",
                [Parameter("brightness", "亮度值0-255", "number")],
                self.kitchen.led.set_brightness,
            ),
            (
                "set_kitchen_fan_state",
                "设置厨房风扇状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.kitchen.set_fan_state,
            ),
            (
                "set_bedroom_led_state",
                "设置卧室LED状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.bedroom.led.set_state,
            ),
            (
                "set_bedroom_led_color",
                "设置卧室LED颜色",
                [Parameter("color", "颜色值0,0,0-255,255,255", "string")],
                self.bedroom.led.set_color,
            ),
            (
                "set_bedroom_led_mode",
                "设置卧室LED模式",
                [Parameter("mode", "模式编号1表示睡眠模式,0表示正常模式", "number")],
                self.bedroom.led.set_mode,
            ),
            (
                "set_bedroom_led_brightness",
                "设置卧室LED亮度",
                [Parameter("brightness", "亮度值0-255", "number")],
                self.bedroom.led.set_brightness,
            ),
            (
                "set_bedroom_humidifier_state",
                "设置卧室加湿器状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.bedroom.set_humidifier_state,
            ),
            (
                "set_bedroom_fan_state",
                "设置卧室风扇状态",
                [Parameter("state", "风扇速度0-3", "number")],
                self.bedroom.set_fan_state,
            ),
            (
                "set_bedroom_windows_state",
                "设置卧室窗户状态",
                [Parameter("state", "窗户状态0-关闭,1-打开", "number")],
                self.bedroom.set_windows_state,
            ),
            (
                "set_bathroom_led_state",
                "设置浴室LED状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.bathroom.led.set_state,
            ),
            (
                "set_bathroom_led_color",
                "设置浴室LED颜色",
                [Parameter("color", "颜色值0,0,0-255,255,255", "string")],
                self.bathroom.led.set_color,
            ),
            (
                "set_bathroom_led_mode",
                "设置浴室LED模式",
                [Parameter("mode", "模式编号1表示睡眠模式,0表示正常模式", "number")],
                self.bathroom.led.set_mode,
            ),
            (
                "set_bathroom_led_brightness",
                "设置浴室LED亮度",
                [Parameter("brightness", "亮度值0-255", "number")],
                self.bathroom.led.set_brightness,
            ),
            (
                "set_bathroom_fan_state",
                "设置浴室风扇状态",
                [Parameter("state", "true表示开启,false表示关闭", "boolean")],
                self.bathroom.set_fan_state,
            ),
        ]

        for prop_name, prop_desc, prop_getter in properties:
            self.add_property(prop_name, prop_desc, prop_getter)
        for method_name, method_desc, method_params, method_handler in methods:
            self.add_method(method_name, method_desc, method_params, method_handler)

    def connect_mqtt(self, broker="znjj.piedaochuan.top", port=1883):
        self.mqtt.connect(broker, port)
        self.mqtt.loop_start()

    async def publish(self, topic, value):
        msg = str(value)
        print(f"[MQTT] {topic} -> {msg}")
        result = self.mqtt.publish(topic, msg, qos=0, retain=False)
        result.wait_for_publish()
        print(f"[MQTT] Published with message ID: {result}")

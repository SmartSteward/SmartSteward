import paho.mqtt.client as mqtt

from src.iot.thing import Thing


class Led:
    def __init__(self):
        self.state = False
        self.color = "255,255,255"
        self.mode = 0
        self.brightness = 150

    async def set_state(self, state: bool):
        self.state = state

    async def set_color(self, color: str):
        self.color = color

    async def set_mode(self, mode: int):
        self.mode = mode

    async def set_brightness(self, brightness: int):
        self.brightness = brightness

    async def get_state(self):
        return self.state

    async def get_color(self):
        return self.color

    async def get_mode(self):
        return self.mode

    async def get_brightness(self):
        return self.brightness


class Room:
    def __init__(self):
        self.led = Led()


class LivingRoom(Room):
    DOOR = "smarthome/door"

    LED = "smarthome/livingroom/led"
    LED_COLOR = "smarthome/livingroom/ledcolor"
    LED_MODE = "smarthome/livingroom/ledmode"
    LED_BRIGHT = "smarthome/livingroom/ledbrightness"
    HUMIDIFIER = "smarthome/livingroom/humidifier"
    DOOR = "smarthome/door"
    FAN = "smarthome/livingroom/fan"
    CURTAIN = "smarthome/livingroom/curtain"
    AIRCONDITIONER = "smarthome/livingroom/airconditioner"

    def __init__(self):
        super().__init__()
        self.humidifier_state = False
        self.door_state = False
        self.fan_state = 0
        self.curtain_state = False
        self.air_conditioner_state = False

    async def set_humidifier_state(self, state: bool):
        self.humidifier_state = state

    async def set_door_state(self, state: bool):
        self.door_state = state

    async def set_fan_state(self, state: int):
        self.fan_state = state

    async def set_curtain_state(self, state: bool):
        self.curtain_state = state

    async def set_air_conditioner_state(self, state: bool):
        self.air_conditioner_state = state

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


class Kitchen(Room):
    LED = "smarthome/kitchen/led"
    LED_COLOR = "smarthome/kitchen/ledcolor"
    LED_MODE = "smarthome/kitchen/ledmode"
    LED_BRIGHT = "smarthome/kitchen/ledbrightness"
    FAN = "smarthome/kitchen/fan"

    def __init__(self):
        super().__init__()
        self.fan_state = False

    async def set_fan_state(self, state: bool):
        self.fan_state = state

    async def get_fan_state(self):
        return self.fan_state


class Bedroom(Room):
    LED = "smarthome/bedroom/led"
    LED_COLOR = "smarthome/bedroom/ledcolor"
    LED_MODE = "smarthome/bedroom/ledmode"
    LED_BRIGHT = "smarthome/bedroom/ledbrightness"
    HUMIDIFIER = "smarthome/bedroom/humidifier"
    WINDOWS = "smarthome/bedroom/windows"
    FAN = "smarthome/bedroom/fan"

    def __init__(self):
        super().__init__()
        self.humidifier_state = False
        self.fan_state = 0
        self.windows_state = 0

    async def set_humidifier_state(self, state: bool):
        self.humidifier_state = state

    async def set_fan_state(self, state: int):
        self.fan_state = state

    async def set_windows_state(self, state: int):
        self.windows_state = state

    async def get_humidifier_state(self):
        return self.humidifier_state

    async def get_fan_state(self):
        return self.fan_state

    async def get_windows_state(self):
        return self.windows_state


class Bathroom(Room):
    LED = "smarthome/bathroom/led"
    LED_COLOR = "smarthome/bathroom/ledcolor"
    LED_MODE = "smarthome/bathroom/ledmode"
    LED_BRIGHT = "smarthome/bathroom/ledbrightness"
    FAN = "smarthome/bathroom/fan"

    def __init__(self):
        super().__init__()
        self.fan_state = False

    async def set_fan_state(self, state: bool):
        self.fan_state = state

    async def get_fan_state(self):
        return self.fan_state


class SmartHome(Thing):
    def __init__(self):
        super().__init__("SmartHome", "智能家居系统")
        self.mqtt = mqtt.Client()
        self.connect_mqtt()
        self.livingroom = LivingRoom()
        self.kitchen = Kitchen()
        self.bedroom = Bedroom()
        self.bathroom = Bathroom()

    def connect_mqtt(self, broker="localhost", port=1883):
        self.mqtt.connect(broker, port)
        self.mqtt.loop_start()

    def publish(self, topic, value):
        msg = str(value)
        print(f"[MQTT] {topic} -> {msg}")
        self.mqtt.publish(topic, msg)

    def set_device_state(self, attr_name, value, topic):
        setattr(self, attr_name, value)
        self.publish(topic, value)

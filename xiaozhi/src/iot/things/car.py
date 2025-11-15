import serial

from src.iot.thing import Parameter, Thing


class Serial:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.ser = serial.Serial(port, baudrate)
        self.dic = {
            "go": b"\x41",
            # "right_forward": b"\x42",
            "right": b"\x43",
            # "right_backward": b"\x44",
            "back": b"\x45",
            # "left_backward": b"\x46",
            "left": b"\x47",
            # "left_forward": b"\x48",
            "stop": b"\x5a",
            "speedup": b"\x58",
            "speeddown": b"\x59",
        }

    def send_state(self, command: str) -> str:
        self.ser.write(self.dic[command])


class Car(Thing):
    def __init__(self):
        super().__init__("Car", "有关小车的运动控制")
        self.serial = Serial()
        self.state = "stop"

        self.add_property("state", "小车当前状态", self.get_state)
        self.add_method(
            "set_state",
            "设置小车状态",
            [Parameter("state", "小车状态go,back,left,right,speedup,speeddown,stop", "string")],
            self.set_state,
        )

    async def get_state(self):
        return self.state

    async def set_state(self, params):
        state = params["state"].get_value()
        self.state = state
        self.serial.send_state(state)
        return {"status": "success", "message": f"小车状态已更新为 {state}"}

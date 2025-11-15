import serial

from src.iot.thing import Parameter, Thing


class Serial:
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def send_state(self, command: str) -> str:
        self.ser.write(command.encode())


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

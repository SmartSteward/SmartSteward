import src.iot.things.smarthome as smarthome

home = smarthome.SmartHome()


async def test_bedroom_led():
    bedroom = home.bedroom
    led = bedroom.led

    # 打开LED
    await led.set_state(True)
    state = await led.get_state()
    print(f"Bedroom LED State after turning on: {state}")

    # 关闭LED
    await led.set_state(False)
    state = await led.get_state()
    print(f"Bedroom LED State after turning off: {state}")


if __name__ == "__main__":
    import asyncio

    asyncio.run(test_bedroom_led())

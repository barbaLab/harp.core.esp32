from __future__ import annotations

import time
from harp.serial.device import Device


# PORT = "/dev/ttyUSB0"   # Change this to your serial port.
PORT = "COM5"         # Windows example.

REG_STATUS_LED_MODE = 32
REG_STATUS_LED_RED = 33
REG_STATUS_LED_GREEN = 34
REG_STATUS_LED_BLUE = 35

LED_MODE_OFF = 0
LED_MODE_ON = 1
LED_MODE_AUTO = 2
LED_MODE_BLINK = 3
LED_MODE_FAULT = 4


def test_status_led(
    port: str = PORT,
    *,
    timeout: float = 1.0,
    step_delay: float = 1.0,
):
    print(f"Connecting to Harp device on {port!r}")

    with Device(port, timeout=timeout) as device:
        try:
            print(device.info())
        except Exception as exc:
            print(f"device.info() failed, continuing anyway: {exc}")

        print("\n[1] Set mode = Off")
        device.write_u8(REG_STATUS_LED_MODE, LED_MODE_OFF)
        time.sleep(step_delay)

        print("[2] Set RGB = default-ish (0x11, 0x55, 0x77)")
        device.write_u8(REG_STATUS_LED_RED, 0x11)
        device.write_u8(REG_STATUS_LED_GREEN, 0x55)
        device.write_u8(REG_STATUS_LED_BLUE, 0x77)
        time.sleep(step_delay)

        print("[3] Set mode = On (should show custom RGB)")
        device.write_u8(REG_STATUS_LED_MODE, LED_MODE_ON)
        time.sleep(step_delay)

        print("[4] Set RGB = red")
        device.write_u8(REG_STATUS_LED_RED, 0x40)
        device.write_u8(REG_STATUS_LED_GREEN, 0x00)
        device.write_u8(REG_STATUS_LED_BLUE, 0x00)
        time.sleep(step_delay)

        print("[5] Set RGB = green")
        device.write_u8(REG_STATUS_LED_RED, 0x00)
        device.write_u8(REG_STATUS_LED_GREEN, 0x40)
        device.write_u8(REG_STATUS_LED_BLUE, 0x00)
        time.sleep(step_delay)

        print("[6] Set RGB = blue")
        device.write_u8(REG_STATUS_LED_RED, 0x00)
        device.write_u8(REG_STATUS_LED_GREEN, 0x00)
        device.write_u8(REG_STATUS_LED_BLUE, 0x40)
        time.sleep(step_delay)

        print("[7] Set mode = Auto")
        device.write_u8(REG_STATUS_LED_MODE, LED_MODE_AUTO)
        time.sleep(step_delay)

        print("[8] Set mode = Blink")
        device.write_u8(REG_STATUS_LED_MODE, LED_MODE_BLINK)
        time.sleep(3.0)

        print("[9] Set mode = Fault")
        device.write_u8(REG_STATUS_LED_MODE, LED_MODE_FAULT)
        time.sleep(3.0)

        print("[10] Read back mode and RGB registers")
        mode = device.read_u8(REG_STATUS_LED_MODE)
        red = device.read_u8(REG_STATUS_LED_RED)
        green = device.read_u8(REG_STATUS_LED_GREEN)
        blue = device.read_u8(REG_STATUS_LED_BLUE)
        print(f"mode={mode} rgb=({red}, {green}, {blue})")

        print("[11] Set mode = Off")
        device.write_u8(REG_STATUS_LED_MODE, LED_MODE_OFF)
        time.sleep(0.5)

    print("\nStatus LED test complete.")


if __name__ == "__main__":
    test_status_led()
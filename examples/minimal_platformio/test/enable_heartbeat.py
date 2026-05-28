#!/usr/bin/env python3
from time import perf_counter as now, sleep

from harp.serial.device import Device
from harp.protocol import OperationMode
from harp.protocol import CommonRegisters as Regs

COM_PORT = "COM5" # COMxx on Windows.

device = Device(COM_PORT, "ibl.bin")

print("Setting mode to ACTIVE.")
device.set_mode(OperationMode.ACTIVE)


# Enable heartbeat (HEARTBEAT_EN bit in OPERATION_CTRL).
print("Enabling heartbeat.")
ctrl = device.read_u8(Regs.OPERATION_CTRL).payload
device.write_u8(Regs.OPERATION_CTRL, ctrl | (1 << 2))

start_time_s = now()
duration_s = 30

while now() - start_time_s < duration_s:
    events = device.get_events()
    for event_reply in events:
        print()
        print(event_reply)
    sleep(0.05)

# Cleanup:
print("Disabling heartbeat.")
ctrl = device.read_u8(Regs.OPERATION_CTRL).payload
device.write_u8(Regs.OPERATION_CTRL, ctrl & ~(1 << 2))
device.disconnect()

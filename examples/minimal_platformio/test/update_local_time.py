#!/usr/bin/env python3
from time import sleep
import os
from harp.serial.device import Device
from harp.protocol import CommonRegisters as CoreRegs

if os.name == 'posix': # check for Linux.
    device = Device("/dev/ttyACM0", "ibl.bin")
else: # assume Windows.
    device = Device("COM5", "ibl.bin")


# Get the old time.
curr_time_s = device.read_u32(CoreRegs.TIMESTAMP_SECOND).payload
print(f"Current seconds: {curr_time_s}")

# Update Harp time on the device.
set_time_seconds = int(3e9)
print(f"Setting Harp seconds to {set_time_seconds}")
_ = device.write_u32(CoreRegs.TIMESTAMP_SECOND, set_time_seconds)
sleep(1)

# Get the new time from the device:
new_time_s = device.read_u32(CoreRegs.TIMESTAMP_SECOND).payload
print(f"Updated seconds: {new_time_s}")

device.disconnect()


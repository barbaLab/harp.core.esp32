#!/usr/bin/env python3
from harp.serial.device import Device
from harp.protocol import CommonRegisters as Regs
import os


# Open the device and print the info on screen
# Open serial connection and save communication to a file
if os.name == 'posix': # check for Linux.
    device = Device("/dev/ttyACM0", "ibl.bin")
else: # assume Windows.
    device = Device("COM5", "ibl.bin")


print("Setting RST_DFU_BIT to Reset device.")
reply = device.write_u8(Regs.RESET_DEV, 0b00100000)

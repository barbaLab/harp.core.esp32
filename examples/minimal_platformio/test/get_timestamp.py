#!/usr/bin/env python3
from harp.serial.device import Device
from harp.protocol import CommonRegisters as Regs
import os
from time import sleep


# Open the device and print the info on screen
# Open serial connection and save communication to a file
if os.name == 'posix': # check for Linux.
    device = Device("/dev/ttyACM0", "ibl.bin")
else: # assume Windows.
    device = Device("COM5", "ibl.bin")


#for i in range(500):
while True:
    reply = device.read_u8(Regs.OPERATION_CTRL)
    print(reply.timestamp)
    sleep(0.05)
# Close connection
device.disconnect()

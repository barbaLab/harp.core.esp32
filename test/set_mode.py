#!/usr/bin/env python3
from harp.serial.device import Device
from harp.protocol import OperationMode
import os
from time import sleep


# Open the device and print the info on screen
# Open serial connection and save communication to a file
if os.name == 'posix': # check for Linux.
    device = Device("/dev/ttyACM0", "ibl.bin")
else: # assume Windows.
    device = Device("COM5", "ibl.bin")


print(f"Device mode is: {device.read_operation_ctrl()['OP_MODE']}")
for i in range(2):
    sleep(0.25)
    print("Setting device mode to Active.")
    reply = device.set_mode(OperationMode.ACTIVE)
    #print("reply to set mode is: ")
    #print(reply)
    print(f"Device mode is now: {device.read_operation_ctrl()['OP_MODE']}")

    print("Setting device mode to Standby.")
    reply = device.set_mode(OperationMode.STANDBY)
    #print("reply to set mode is: ")
    #print(reply)
    print(f"Device mode is now: {device.read_operation_ctrl()['OP_MODE']}")

    print()
    sleep(0.25)

# Close connection
device.disconnect()

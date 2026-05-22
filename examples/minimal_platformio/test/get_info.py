#!/usr/bin/env python3
from harp.serial.device import Device
import os


# ON THIS EXAMPLE
#
# This code opens the connection with the device and displays the information
# Also saves device's information into variables


# Open the device and print the info on screen
# Open serial connection and save communication to a file
if os.name == 'posix': # check for Linux.
    device = Device("/dev/ttyACM0", "ibl.bin")
else: # assume Windows.
    device = Device("COM5", "ibl.bin")
device.info()                           # Display device's info on screen
# dump registers.
#print("Register dump:")
#print(device.dump_registers())
# Close connection
device.disconnect()

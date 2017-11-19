#===============================================================================
# this program will print out the serial output of a (via usb) connected arduino
#===============================================================================

import serial
import serial.tools.list_ports
import time

# just takes the first available com port, does not give options
# multiple access on this port most likely will cause issues and possible disconnection
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name

print("connecting to com port: " + firstPortName)
ser = serial.Serial("/dev/" + firstPortName, 57600)

# ctrl + c to stop infinite loop
while True:
    print ser.readline()

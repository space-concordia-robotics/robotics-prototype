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
# can we automate getting the bandwidth selected by the arduino
# instead of hardcoding it?
ser = serial.Serial("/dev/" + firstPortName, 9600)

# ctrl + c to stop infinite loop
# might be better to let this break on some specific user input
# such as 'x', for a more graceful exit
while True:
    print ser.readline()

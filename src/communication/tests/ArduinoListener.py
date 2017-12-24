#===============================================================================
# this program prints out the serial output of a (via usb) connected arduino
# and logs the output to a .dat file
#===============================================================================

import serial
import serial.tools.list_ports
import time

# just takes the first available com port, does not give options
# multiple access on this port most likely will cause issues and possible disconnection
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name

print("Connecting to com port: " + firstPortName)

# can we automate getting the bandwidth selected by the arduino
# instead of hardcoding it?
ser = serial.Serial("/dev/" + firstPortName, 9600)
print("Connected to Port: " + firstPortName)

f = open('Python_Log.dat','a')

#Whenever we run the arduino, we output the time and date for the output 
#just to organize the log file for when we need to see output

f.write("=======================================================================\n")
f.write("Program Run Current Log Time: " +  time.ctime()+"\n")
f.write("=======================================================================\n")

# ctrl + c to stop infinite loop
#try catch block allows for a more graceful exit of our while loop.
print("Ctrl + c to stop")
try:
	while True:
		readData = ser.readline()
		print(readData)
		f.write(readData)
except KeyboardInterrupt:
	print("\nSerial Output Terminated!")

print("Program Terminated")

#stop writing to file
f.close()

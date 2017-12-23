
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

print("Connecting to com port: " + firstPortName)
#list of all possible baud rates(bandwiths) for the arduino, 
baud_rates = [300,600,1200,4800,14400,19200,28800,38400,57600,115200,9600]
PythonMsg="tester!"


#=============================================================================
#WARNING: This has only been tested for baud rate 9600 for Arduino Mega,please 
#test this code for other adrduinos to make sure it works.
#=============================================================================

#loop through baud rates to connect to arduino 
for i in baud_rates:

	#connect to arduino at specified baudrate, specify a timeout display baud rate number
	print("\nCheck at baud rate: "+str(i))
	ser = serial.Serial( "/dev/" + firstPortName,i,timeout=.959)

	#read arduino message and store that value 
	ArduinoMsg = ser.readline()
	ArduinoMsg = ArduinoMsg.strip()
	
	#show both arduino and python strings to see if there is a match (used for debugging) 
	print("Arduino Message: "+ ArduinoMsg)
	print("Python Message: " + PythonMsg)
	
	#check if both messages are not empty strings
	if PythonMsg and ArduinoMsg: 
		
		#check if strings are equal, if so, we are connected via the correct baud rate.
		if PythonMsg == ArduinoMsg:
			print("connection successful at baudrate: " +str(i))
			baudrate = i
			break
		
		#if not, then we are not properly connected and keep looping
		else:
			print("connection failed at baudrate: " + str(i))

	#flush the buffer and close our serial connection
	ser.flush()
	ser.close()

#show that we are connected to the proper port and write to text file
print("Connected to Port: " + firstPortName)
f = open('Python_Log.dat','w+')

#Whenever we run the arduino, we output the time and date for the output 
#just to organize the log file for when we need to see output

f.write("=======================================================================\n")
f.write("Program Run Current Log Time: " +  time.ctime()+"\n")
f.write("=======================================================================\n")


# ctrl + c to stop infinite loop
# might be better to let this break on some specific user input
# such as 'x', for a more graceful exit

#try catch block allows for a more graceful exit of our while loop.
try:
	while True:
		readData = ser.readline()
		print(readData)
		f.write(readData)
except KeyboardInterrupt:
	print("Serial Output Terminated!")

print("Program Terminated")

f.close()
#stop writing to file






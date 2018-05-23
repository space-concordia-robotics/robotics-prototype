import serial
import serial.tools.list_ports
import time

# just takes the first available com port, does not give options
# multiple access on this port most likely will cause issues and poss$
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name

print("connecting to com port: " + firstPortName)
# can we automate getting the bandwidth selected by the arduino 
# instead of hardcoding it?
ser = serial.Serial("/dev/" + firstPortName, 9600)

while True:
    for i in range (0, 4):
        if i == 0:
            ser.write('d')
            # doesn't work
            #response = ser.readline()
            #print(response)
        elif i == 1:
            ser.write('x')
        elif i == 2:
            ser.write('y')
        elif i == 3:
            ser.write('r')
        elif i == 4:
            ser.write('c')
        time.sleep(1)
    # doesn't work, but you can run ArduinoListener.py at the same time as running this program to see the response from the arduino
    #print(ser.readline())

    time.sleep(3)

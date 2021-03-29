# import RPi.GPIO as GPIO
import random
import time
import serial

ser = serial.Serial('/dev/ttyUSB0', 57600)

def main():
    while(True):
        if ser.in_waiting > 0:
            commandID = ser.read()
            print(int.from_bytes(commandID, "big"))
            argsLen = ser.read()
            print(int.from_bytes(argsLen, "big"))
            args = ser.read(int.from_bytes(argsLen, "big"))
            print(args)
            #break
    ser.close()

if __name__ == '__main__':
    main()

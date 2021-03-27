# import RPi.GPIO as GPIO
import random
import time
import serial

# Pin definitions
arm_pin = 11
wheel_pin = 13
science_pin = 15

teensy_pins = [arm_pin, wheel_pin, science_pin]

ser = serial.Serial('/dev/ttyACM0', 57600)

def main():
    ser.write(b'\x00\x02\x03\x01\x04')
    while(True):
        if ser.in_waiting > 0:
            print(ser.read(ser.in_waiting))

            # commandID = ser.read()
            # print(int.from_bytes(commandID, "big"))
            # argsLen = ser.read()
            # print(int.from_bytes(argsLen, "big"))
            # args = ser.read(int.from_bytes(argsLen, "big"))
            # print(args)
            break
    ser.close()

if __name__ == '__main__':
    main()

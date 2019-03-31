#/usr/bin/env python3
import serial

# For the odroid we use "/dev/ttySAC0"
class Uart:
    __active_ctr = 0

    def __init__(self, port, baudrate):
        type(self).__active_ctr += 1
        self.port = port
        self.baudrate = baudrate

    def tx(self, msg):
        # prep message
        msg = str(msg)
        b_msg=msg.encode()

        # open serial port
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baudrate
        ser.open()

        ser.write(b_msg)
        ser.close()

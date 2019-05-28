#/usr/bin/env python3
import serial

# For the odroid we use "/dev/ttySAC0"
class Uart:
    __active_ctr = 0

    def __init__(self, port, baudrate, timeout=-1):
        type(self).__active_ctr += 1
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        if timeout > -1:
            self.ser.timeout = timeout

    def __del__(self):
        type(self).__active_ctr -= 1

    def tx(self, msg):
        # prep message
        msg = str(msg)
        b_msg=msg.encode()

        # perform write
        self.ser.open()
        self.ser.write(b_msg)
        self.ser.close()

    def rx(self):
        # open serial port
        self.ser.open()

        data = self.ser.readline()
        data = data.decode('utf-8', errors='ignore')

        self.ser.close()

        print(data)

        return data

"""
u = Uart("/dev/ttySAC0", 9600)

u.tx("budge fwd ~ ~ ~ ~ ~\n")

u.rx()
"""

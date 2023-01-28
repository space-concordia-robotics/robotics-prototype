from os import listdir
import serial


def get_ports():
    starting_folder = "/dev/serial/by-id"
    ports = {}

    for dir in listdir(starting_folder):
        test_device = starting_folder + "/" + dir
        ser = serial.Serial(test_device, 57600, timeout=1)
        ser.write(b'\xFF\x00\x0A')
        id = ser.read()
        argslen = int.from_bytes(ser.read(), "big")
        identifier = ser.read(argslen).decode("utf-8")
        ports[identifier] = test_device
    
    print(ports)
    return ports

get_ports()
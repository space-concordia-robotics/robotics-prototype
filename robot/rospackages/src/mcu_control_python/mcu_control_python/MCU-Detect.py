import serial
import serial.tools.list_ports as list_ports

def get_ports():
    ports = {}

    for port_info in list_ports.comports():
        device_path = port_info.device
        ser = serial.Serial(device_path, 57600, timeout=1)
        ser.write(b'\xFF\x00\x0A')
        id = int.from_bytes(ser.read(), "big")
        if id == 0:
            argslen = int.from_bytes(ser.read(), "big")
            identifier = ser.read(argslen).decode("utf-8")
            if len(identifier) > 0:
                ports[identifier] = device_path
    
    print(ports)
    return ports

get_ports()
import serial
import serial.tools.list_ports


def get_open_comports():
    ports = [
        port for port in serial.tools.list_ports.comports() if port[2] != 'n/a'
    ]
    return ports


class SerialPort(serial.Serial):
    """Serial object options class wrapper.

    This class acts as a simple wrapper for the port options used when initializing
    a `Serial` class instance
    """

    # TODO: Add `type.hints` to overload without path specified
    def __init__(self, path=None, baudrate=9600, timeout=1):
        if path is None:
            try:
                path = get_open_comports()[0].device
            except Exception as e:
                print("Issue in get_open_comports()")
                print(e)
        super().__init__(path, baudrate, timeout=timeout)

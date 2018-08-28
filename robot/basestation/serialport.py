import serial
import serial.tools.list_ports


def get_open_comports():
    ports = [port for port in serial.tools.list_ports.comports()]
    return ports


class SerialPort(serial.Serial):
    """Serial object options class wrapper.

    This class acts as a simple wrapper for the port options used when initializing
    a `Serial` class instance
    """

    # TODO: Add `type.hints` to overload without path specified
    def __init__(self, path=get_open_comports()[0], baudrate=9600, timeout=1):
        self.path = path
        self.baudrate = baudrate
        self.timeout = timeout
        super().__init__(self.path, self.baudrate, timeout=self.timeout)

    @property
    def path(self):
        return self.__path

    @path.setter
    def path(self, path):
        self.__path = path

    @property
    def baudrate(self):
        return self.__baudrate

    @baudrate.setter
    def baudrate(self, baudrate):
        self.__baudrate = baudrate

    @property
    def timeout(self):
        return self.__timeout

    @timeout.setter
    def timeout(self, timeout):
        self.__timeout = timeout

import serial


class SerialPort(serial.Serial):
    """Serial object options class wrapper.

    This class acts as a simple wrapper for the port options used when initializing
    a `Serial` class instance
    """

    def __init__(self, path, baudrate, timeout):
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

class Port(object):
    """Serial object options class wrapper.

    This class acts as a simple wrapper for the port options used when initializin
    a `Serial` class instance
    """

    def __init__(self, path, baudrate, timeout):
        self.set_path(path)
        self.set_baudrate(baudrate)
        self.set_timeout(timeout)

    def set_path(self, path):
        self.path = path

    def get_path(self):
        return self.path

    def set_baudrate(self, baudrate):
        self.baudrate = baudrate

    def get_baudrate(self):
        return self.baudrate

    def set_timeout(self, timeout):
        self.timeout = timeout

    def get_timeout(self):
        return self.timeout

class Motor:
    """
    NOTE: Pylint falsely warns of a attribute defined outside `__init__`.
    The issue is reported [here](https://github.com/PyCQA/pylint/issues/409).
    Until it is fixed, expect warnings everytime the use of a setter is used
    inside the `__init__` method. Could also not use setter and simply assign the
    mangled var names, but it's not as clean, nor is using both and setting the
    mangled one to `None`.
    """

    def __init__(self,
                 name,
                 max_angle=160,
                 min_angle=0,
                 max_current=5,
                 min_current=0,
                 home_angle=0,
                 serial_port=None):
        # Attributes
        # Use of double underscore to avoid attribute name collisions when subclassed.
        # Search "Python name mangling" for more info.
        self.__max_angle = max_angle
        self.__min_angle = min_angle
        self.__max_current = max_current
        self.__min_current = min_current

        # Properties
        self.name = name
        self.angle_position = home_angle
        self.electric_current = None  # TODO: What should this be?
        # self.alive depends on currents, no setter
        self.refresh_rate = None  # TODO: Use default value?
        self.serial_port = serial_port

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def angle_position(self):
        return self.__angle_position

    # We don't want to intentionally try to set angle positions out of the possible ranges
    @angle_position.setter
    def angle_position(self, angle_position):
        if self.__min_angle <= angle_position <= self.__max_angle:
            self.__angle_position = angle_position
            return True

        print("Unable to set angle position to " + str(angle_position) +
              " for motor: " + self.name)
        return False

    @property
    def electric_current(self):
        # get_current_sensor_reading()
        return self.__electric_current

    # No restriction otherwise we can't detect when out of bounds
    @electric_current.setter
    def electric_current(self, electric_current):
        self.__electric_current = electric_current

    @property
    def alive(self):
        return self.electric_current <= self.__max_current

    # We'll have to callibrate the refresh rates for each motor during testing motors
    @property
    def refresh_rate(self):
        return self.__refresh_rate

    @refresh_rate.setter
    def refresh_rate(self, refresh_rate):
        self.__refresh_rate = refresh_rate

    @property
    def serial_port(self):
        return self.__serial_port

    @serial_port.setter
    def serial_port(self, serial_port):
        self.__serial_port = serial_port

#@TODO: add error handling so that can be tested independent of serial port available or not?
    def rotate(self, angle, direction=None):
        if angle < 0 and direction is None:
            direction = 'ccw'
        elif angle > 0 and direction is None:
            direction = 'cw'
        elif angle > 0 and direction == 'ccw':
            angle = -angle
        elif angle < 0 and direction == 'cw':
            raise ValueError('`angle` cannot be negative and direction `cw`')

        self.angle_position = self.angle_position + angle
        # this is the line that hangs if the teensy that is being tested with is not configured
        # i.e. testing without trying to make motors move, comment the following line for quick fix
        self.serial_port.write(str.encode(str(self.angle_position) + '\n'))

    def write(self, serial_port, angle):
        print('Name' + self.name)
        print('motor ' + self.name + ' direction ' + angle)
        serial_port.write(
            str.encode('motor ' + self.name + ' direction ' + angle +
                       ' speed 0 time 500'))

    def read(self, serial_port):
        str1 = serial_port.readline()
        print(str1.decode())
        return int(self.angle_position)

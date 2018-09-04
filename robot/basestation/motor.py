class Motor:
    def __init__(self,
                 name,
                 max_angle=160,
                 min_angle=0,
                 max_current=5,
                 min_current=0,
                 home_angle=0,
                 serial_port=None):
        self.name = name
        self.__max_angle = max_angle
        self.__min_angle = min_angle
        self.__max_current = max_current
        self.__min_current = min_current
        self.angle_position = home_angle
        self.serial_port = serial_port

    @property
    def angle_position(self):
        return self.__angle_position

    # We don't want to intentionally try to set angle positions out of the possible ranges
    @angle_position.setter
    def angle_position(self, angle_position):
        if angle_position >= self.__min_angle and angle_position <= self.__max_angle:
            self.__angle_position = angle_position
            return True
        else:
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
        if self.electric_current > self.__max_current:
            return False
        else:
            return True

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
        self.serial_port.write(str.encode(str(self.angle_position) + '\n'))

    def write(self, serial_port, angle):
        print('Name' + self.name)
        print(f'motor {self.name} direction {angle}')
        serial_port.write(
            str.encode(
                f'motor {self.name} direction {angle} speed 0 time 500'))

    def read(self, serial_port):
        str1 = serial_port.readline()
        print(str1.decode())
        return int(self.angle_position)

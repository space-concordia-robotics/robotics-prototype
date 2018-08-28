# Based off UML diagram in SoAD:
# https://docs.google.com/document/d/1QEaR5sP7aWUMgJukAuCMeTT6QKM80vOXxO7XymeWWns/edit
# you will need to be signed into your spaceconcordia email account to be able to
# view/edit this document feel free to add feedback in the form of gdocs comment


class Motor:
    # May or may not be used, pretty sure there's no overloading methods/constructors
    # in python without needing to assign default values for the params
    # def __init__(self, min_angle, max_angle, home_angle, min_current, max_current,
    # angle_position, electric_current, status, refresh_rate):
    #self.min_angle = min_angle
    #self.max_angle = max_angle
    #self.home_angle = home_angle
    #self.angle_position = angle_position
    #self.min_current = min_current
    #self.max_current = max_current
    #self.electric_current = electric_current
    #self.alive = alive
    #self.refresh_rate = refresh_rate

    def __init__(self, name, max_angle, min_angle, max_current, min_current,
                 home_angle):
        self.name = name
        self.__max_angle = max_angle
        self.__min_angle = min_angle
        self.__max_current = max_current
        self.__min_current = min_current
        self.angle_position = home_angle

    # Very dangerous, we must know exactly what these values are beforehand
    # Should only be set in constructor and are private attributes.
    # def set_max_min_angles(self, maxval, minval):
    #     self.max_angle = maxval
    #     self.min_angle = minval

    @property
    def angle_position(self):
        return self.__angle_position

    # We don't want to intentionally try to set angle positions out of the possible ranges
    @angle_position.setter
    def angle_position(self, angle_position):
        if angle_position > self.__min_angle and angle_position < self.__max_angle:
            self.__angle_position = angle_position
            return True
        else:
            print("unable to set angle position to " + str(angle_position) +
                  " for motor: " + self.name)
            return False

    # Very dangerous, we must know exactly what these values are beforehand
    # Should only be set in constructor and are private attributes.
    # def set_max_min_currents(self, maxval, minval):
    #     self.max_current = maxval
    #     self.min_current = minval

    @property
    def electric_current(self):
        # get_current_sensor_reading()
        return self.__electric_current

    # No restriction otherwise we can't detect when out of bounds
    @electric_current.setter
    def electric_current(self, electric_current):
        self.__electric_current = electric_current

    # Logic based off of electric current being in safety range (for now)
    # --------------------------------------------------------------------
    # To fully implement logic of motor status, we need to establish a way
    # to keep track of when the motor is moving (perhaps by using
    # a method that sees if the angle position of the motor is changing)
    # If it is moving and the current is within expected value range
    # then it is considered alive.
    # Otherwise, if the current is within the expected ranges for movement,
    # but the motor isn't moving, then it is considered dead.
    # If it is in expected current range for moving, and is moving,
    # then it is considered alive.
    # If it is in expected idle current range or and moving,
    # it may be that there is a power issue, and perhaps the motor
    # has just died and is turning because the arm is "falling down"
    # at that point.
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

    def write(self, serial_port, angle):
        # self.set_angle_position(angle)
        # print(int(self.get_angle_position()))
        # serial.write(struct.pack('>B', int(self.get_angle_position())))
        # serial.write(str.encode(str(self.get_angle_position())))
        print('Name' + self.name)
        print(f'motor {self.name} direction {angle}')
        serial_port.write(
            str.encode(
                f'motor {self.name} direction {angle} speed 0 time 1000 '))
        # serial.write(180)

    def read(self, serial_port):
        str1 = serial_port.readline()
        print(str1.decode())
        return int(self.angle_position)

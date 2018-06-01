# based off UML diagram in SoAD: https://docs.google.com/document/d/1QEaR5sP7aWUMgJukAuCMeTT6QKM80vOXxO7XymeWWns/edit
# you will need to be signed into your spaceconcordia email account to be able to view/edit this document
# feel free to add feedback in the form of gdocs comment

# agile manifesto:
# individuals and interactions over processes and tools
# working software over comprehensive documentation
# customer collaboration over contract negotiation
# responding to change over following a plan
import struct

'''
This class represents a motor component that will be used inside
the mother `Microcontroller` class container (such as the Arduino/ODROID).
'''


class Motor:
    # may or may not be used, pretty sure there's no overloading methods/constructors in python without needing to assign default values for the params
    # def __init__(self, min_angle, max_angle, home_angle, min_current, max_current, angle_position, electric_current, status, refresh_rate):
                #self.min_angle = min_angle
                #self.max_angle = max_angle
                #self.home_angle = home_angle
                #self.min_current = min_current
                #self.max_current = max_current
                #self.angle_position = angle_position
                #self.electric_current = electric_current
                #self.alive = alive
                #self.refresh_rate = refresh_rate

    def __init__(self, name, max_angle, min_angle, max_current, min_current, home_angle):
        self.name = name
        self.set_max_min_angles(max_angle, min_angle)
        self.set_max_min_currents(max_current, min_current)
        self.set_angle_position(home_angle)

    # will definitely have to be configured during testing
    # def home_position(self):
        # self.angle_position = self.home_angle

    # we'll have to callibrate the refreshrates for each motor during testing motors
    def set_refresh_rate(self, rate):
        self.refresh_rate = rate

    # very dangerous, we must know exactly what these values are beforehand
    def set_max_min_angles(self, max, min):
        self.max_angle = max
        self.min_angle = min

    # very dangerous, we must know exactly what these values are beforehand
    def set_max_min_currents(self, max, min):
        self.max_current = max
        self.min_current = min

    # we don't want to intentionally try to set angle positions out of the possible ranges
    def set_angle_position(self, angle):
        if int(angle) >= self.min_angle and int(angle) <= self.max_angle:
            self.angle_position = angle
            return True
        else:
            print("unable to set angle position to " +
                  str(angle) + " for motor: " + self.name)
            return False

    def get_angle_position(self):
        return self.angle_position

    def get_electric_current(self):
        # get_current_sensor_reading()
        return self.electric_current

    # no restriction otherwise we can't detect when out of bounds
    def set_electric_current(self, current):
        self.electric_current = current

    # logic based off of electric current being in safety range (for now)
    # --------------------------------------------------------------------
    # to fully implement logic of motor status, we need to establish a way
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
    # at that point
    def update_status(self):
        if self.electric_current > self.max_current:
            self.alive = False
        else:
            self.alive = True

    def write(self, serial, angle):
        self.set_angle_position(angle)
        serial.write(struct.pack('>B', int(self.get_angle_position())))

    def read(self, serial):
        return int(serial.read())

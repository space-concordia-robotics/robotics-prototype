import pygame
import time

class Astro_Joy():
    joy_axis = [None] * 6
    controls = None
    throttle = 0
    steer = 0
    def __init__(self,T,S):
        pygame.init()
        self.controls = pygame.joystick.Joystick(0)
        self.controls.init()

        self.max_throttle = T
        self.max_steer = S



    def wheels(self):
        time.sleep(0.05)
        pygame.event.pump()
        for i in range(6):
            self.joy_axis[i] = self.controls.get_axis(i)
        if self.joy_axis[2] < 0:
            self.joy_axis[2] = 0
        if self.joy_axis[5] < 0:
            self.joy_axis[5] = 0

        if self.joy_axis[5] == 0 and self.joy_axis[2] == 0:
            self.throttle = int(0)
        elif self.joy_axis[5] > 0 and self.joy_axis[2] == 0:
            self.throttle = int(round(self.joy_axis[5] * self.max_throttle))
        elif self.joy_axis[2] > 0 and self.joy_axis[5] == 0:
            self.throttle = int(round(self.joy_axis[2] * self.max_throttle)) * (-1)
        else:
            self.throttle = int(0)

        self.steer = int(round(self.joy_axis[0] * self.max_steer))

        msg = str(self.throttle) + ':' + str(self.steer) + '\n'
        #return (self.throttle,self.steer)
        return msg


my_joy = Astro_Joy(45,45)

try:
    while True:
        print(my_joy.wheels())

        # time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting now")

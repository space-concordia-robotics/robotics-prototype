import pygame
import time

class Astro_Joy():
    joy_axis = [None] * 6
    joy_buttons = [None] * 13
    controls = None
    throttle_target = 0
    throttle_actual = 0
    steer_max = 45
    steer_actual = 0

    #controlled_acceleration = False
    acceleration_lapse = 0.08
    deceleration_lapse = 0.04

    def __init__(self,T,S,C):
        pygame.init()
        self.controls = pygame.joystick.Joystick(0)
        self.controls.init()

        self.max_throttle = T
        self.steer_fine = S
        self.controlled_acceleration = C


    def wheels(self):

        pygame.event.pump()
        for i in range(6):
            self.joy_axis[i] = self.controls.get_axis(i)
        if self.joy_axis[2] < 0:
            self.joy_axis[2] = 0
        if self.joy_axis[5] < 0:
            self.joy_axis[5] = 0

        for j in range(13):
            self.joy_buttons[j] = self.controls.get_button(j)

        if self.joy_axis[5] == 0 and self.joy_axis[2] == 0:
            self.throttle_target = int(0)
        elif self.joy_axis[5] > 0 and self.joy_axis[2] == 0:
            self.throttle_target = int(round(self.joy_axis[5] * self.max_throttle))
        elif self.joy_axis[2] > 0 and self.joy_axis[5] == 0:
            self.throttle_target = int(round(self.joy_axis[2] * self.max_throttle)) * (-1)
        else:
            self.throttle_target = int(0)

        if self.controlled_acceleration:
            if self.throttle_actual > 0 and self.throttle_target > self.throttle_actual:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual += 1
            elif self.throttle_actual > 0 and self.throttle_target < self.throttle_actual:
                time.sleep(self.deceleration_lapse)
                self.throttle_actual -= 1
            elif self.throttle_actual < 0 and self.throttle_target < self.throttle_actual:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual -= 1
            elif self.throttle_actual < 0 and self.throttle_target > self.throttle_actual:
                time.sleep(self.deceleration_lapse)
                self.throttle_actual += 1
            elif self.throttle_actual == 0 and self.throttle_target > 0:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual += 1
            elif self.throttle_actual == 0 and self.throttle_target < 0:
                self.throttle_actual -= 1
            else:
                time.sleep(self.deceleration_lapse)
        else:
            time.sleep(self.deceleration_lapse)
            self.throttle_actual = self.throttle_target


        if self.joy_buttons[5] == 1 and self.joy_buttons[4] == 0:
            self.steer_actual = self.steer_max
        elif self.joy_buttons[5] == 0 and self.joy_buttons[4] == 1:
            self.steer_actual = self.steer_max * (-1)
        elif self.joy_buttons[5] == 0 and self.joy_buttons[4] == 0:
            self.steer_actual = int(round(self.joy_axis[0] * self.steer_fine))
        else:
            self.steer_actual = int(0)

        msg = str(self.throttle_actual) + ':' + str(self.steer_actual) + '\n'
        #return (self.throttle,self.steer)
        return msg


my_joy = Astro_Joy(45,25,True)

try:
    while True:
        print(my_joy.wheels())

        # time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting now")

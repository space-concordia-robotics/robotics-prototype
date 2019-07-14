#!/usr/bin/env python3

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
    notch_f = 1
    notch_r = 4

    acceleration_lapse = 0.08
    deceleration_lapse = 0.04

    def __init__(self, T, S, C):
        pygame.init()
        self.controls = pygame.joystick.Joystick(0)
        self.controls.init()

        self.throttle_max = T
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

        rotation = (self.joy_buttons[4], self.joy_buttons[5])

        if self.joy_axis[5] == 0 and self.joy_axis[2] == 0:
            self.throttle_target = int(0)
        elif self.joy_axis[5] > 0 and self.joy_axis[2] == 0:
            self.throttle_target = int(round(self.joy_axis[5] * self.throttle_max))
        elif self.joy_axis[2] > 0 and self.joy_axis[5] == 0:
            self.throttle_target = int(round(self.joy_axis[2] * self.throttle_max)) * (-1)
        else:
            self.throttle_target = int(0)


        if rotation == (0,1):
            self.steer_actual = self.steer_max
        elif rotation == (1,0):
            self.steer_actual = self.steer_max * (-1)
        elif rotation == (0,0):
            self.steer_actual = int(round(self.joy_axis[0] * self.steer_fine))
        else:
            self.steer_actual = int(0)


        if self.controlled_acceleration and (rotation == (0,0) or rotation == (1,1)):
            if self.throttle_actual > 0 and self.throttle_target > self.throttle_actual:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual += self.notch_f
            elif self.throttle_actual > 0 and self.throttle_target < self.throttle_actual:
                time.sleep(self.deceleration_lapse)
                self.throttle_actual -= self.notch_f
            elif self.throttle_actual < 0 and self.throttle_target < self.throttle_actual:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual -= self.notch_f
            elif self.throttle_actual < 0 and self.throttle_target > self.throttle_actual:
                time.sleep(self.deceleration_lapse)
                self.throttle_actual += self.notch_f
            elif self.throttle_actual == 0 and self.throttle_target > 0:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual += self.notch_f
            elif self.throttle_actual == 0 and self.throttle_target < 0:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual -= self.notch_f
            else:
                time.sleep(self.deceleration_lapse)

        elif self.controlled_acceleration and (rotation == (0,1) or rotation == (1,0)):
            if self.throttle_actual > 0 and self.throttle_target > self.throttle_actual:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual += self.notch_r
            elif self.throttle_actual > 0 and self.throttle_target < self.throttle_actual:
                time.sleep(self.deceleration_lapse)
                self.throttle_actual -= self.notch_r
            elif self.throttle_actual < 0 and self.throttle_target < self.throttle_actual:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual -= self.notch_r
            elif self.throttle_actual < 0 and self.throttle_target > self.throttle_actual:
                time.sleep(self.deceleration_lapse)
                self.throttle_actual += self.notch_r
            elif self.throttle_actual == 0 and self.throttle_target > 0:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual += self.notch_r
            elif self.throttle_actual == 0 and self.throttle_target < 0:
                time.sleep(self.acceleration_lapse)
                self.throttle_actual -= self.notch_r
            else:
                time.sleep(self.deceleration_lapse)
        else:
            time.sleep(self.deceleration_lapse)
            self.throttle_actual = self.throttle_target

        if self.throttle_actual > 0:
            while self.throttle_actual > 45:
                self.throttle_actual -= 1
        elif self.throttle_actual < 0:
            while self.throttle_actual < -45:
                self.throttle_actual += 1

        if self.joy_axis[2] == 0 and self.joy_axis[5] == 0 and abs(self.throttle_actual) <= self.notch_r:
            self.throttle_actual = int(0)


        msg = str(self.throttle_actual) + ':' + str(self.steer_actual) + '\n'
        return msg


if __name__ == '__main__':
    my_joy = Astro_Joy(45, 25, True)

    try:
        while True:
            print(my_joy.wheels())

            # time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting now")

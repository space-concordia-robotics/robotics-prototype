#!/usr/bin/env python3

import pygame
import time
import sys



class Astro_Joy():
    # General Attributes :
    controls = None
    joy_axis = [None] * 6
    joy_buttons = [None] * 13
    joy_hat = None
    timeout_max = 10
    timeout = 0
    long_lapse = 0.08
    short_lapse = 0.04
    isRover = True
    isArm = False
    switch_button = False


    # Wheels related attributes :
    throttle_target = 0
    throttle_actual = 0
    steer_max = 45
    steer_actual = 0
    notch_f = 1
    notch_r = 3
    rotating_right = False
    rotating_left = False
    moving_forward = False
    moving_backward = False
    wheels_activate = False
    wheels_deactivate = False


    # Cameras related attributes :
    front_cam_position_actual = 60
    front_cam_position_max = 90
    front_cam_position_min = 0
    top_cam_position_actual = 35
    top_cam_position_max = 115
    top_cam_position_min = 0
    front_cam_contiuous = 93
    top_cam_continuous = 93
    continuous_motion = 5
    cameras_halt = True
    cameras_timeout_max = 10
    cameras_timeout = 0


    # Arm related attributes :
    arm_forward = 'fwd'
    arm_backward = 'back'
    arm_halt = '~'
    arm_motor1 = arm_halt
    arm_motor2 = arm_halt
    arm_motor3 = arm_halt
    arm_motor4 = arm_halt
    arm_motor5 = arm_halt
    arm_motor6 = arm_halt
    arm_reset_button = False


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

        if self.steer_actual == self.steer_max and self.throttle_actual > 0:
            self.rotating_right = True
            self.rotating_left = False
            self.moving_forward = False
            self.moving_backward = False
        elif self.steer_actual == (-1) * self.steer_max and self.throttle_actual > 0:
            self.rotating_right = False
            self.rotating_left = True
            self.moving_forward = False
            self.moving_backward = False
        elif abs(self.steer_actual) < self.steer_max and self.throttle_actual > 0:
            self.rotating_right = False
            self.rotating_left = False
            self.moving_forward = True
            self.moving_backward = False
        elif abs(self.steer_actual) < self.steer_max and self.throttle_actual < 0:
            self.rotating_right = False
            self.rotating_left = False
            self.moving_forward = False
            self.moving_backward = True
        elif self.throttle_actual == 0:
            self.rotating_right = False
            self.rotating_left = False
            self.moving_forward = False
            self.moving_backward = False

        if not (self.moving_forward or self.moving_backward or self.rotating_right or self.rotating_left):
            if rotation == (0, 1):
                self.steer_actual = self.steer_max
                self.throttle_actual += self.notch_r
                time.sleep(self.short_lapse)
            elif rotation == (1, 0):
                self.steer_actual = (-1) * self.steer_max
                self.throttle_actual += self.notch_r
                time.sleep(self.short_lapse)
            elif rotation == (1, 1):
                time.sleep(self.short_lapse)
            else:
                if self.throttle_target > 0:
                    self.throttle_actual += self.notch_r + self.notch_f
                    time.sleep(self.long_lapse)
                elif self.throttle_target < 0:
                    self.throttle_actual -= self.notch_r + self.notch_f
                else:
                    if self.timeout != 0:
                        time.sleep(self.short_lapse)
                    else:
                        pass

        if self.rotating_right:
            if rotation == (0, 1):
                self.throttle_actual += self.notch_r
                time.sleep(self.short_lapse)
            else:
                if self.throttle_actual > 0:
                    self.throttle_actual -= self.notch_r
                    time.sleep(self.short_lapse)

        if self.rotating_left:
            if rotation == (1, 0):
                self.throttle_actual += self.notch_r
                time.sleep(self.short_lapse)
            else:
                if self.throttle_actual > 0:
                    self.throttle_actual -= self.notch_r
                    time.sleep(self.short_lapse)

        if self.moving_forward or self.moving_backward:
            if self.controlled_acceleration:
                if self.throttle_actual > 0 and self.throttle_target > self.throttle_actual:
                    time.sleep(self.long_lapse)
                    self.throttle_actual += self.notch_f
                elif self.throttle_actual > 0 and self.throttle_target < self.throttle_actual:
                    time.sleep(self.short_lapse)
                    self.throttle_actual -= self.notch_f
                elif self.throttle_actual < 0 and self.throttle_target < self.throttle_actual:
                    time.sleep(self.long_lapse)
                    self.throttle_actual -= self.notch_f
                elif self.throttle_actual < 0 and self.throttle_target > self.throttle_actual:
                    time.sleep(self.short_lapse)
                    self.throttle_actual += self.notch_f
                elif self.throttle_actual == 0 and self.throttle_target > 0:
                    time.sleep(self.long_lapse)
                    self.throttle_actual += self.notch_f + self.notch_r
                elif self.throttle_actual == 0 and self.throttle_target < 0:
                    time.sleep(self.long_lapse)
                    self.throttle_actual -= self.notch_f + self.notch_r
                else:
                    time.sleep(self.short_lapse)

            else:
                time.sleep(self.short_lapse)
                self.throttle_actual = self.throttle_target

            self.steer_actual = int(round(self.joy_axis[0] * self.steer_fine))

        if self.throttle_actual > 0:
            while self.throttle_actual > self.throttle_max:
                self.throttle_actual -= 1
        elif self.throttle_actual < 0:
            while self.throttle_actual < self.throttle_max * (-1):
                self.throttle_actual += 1

        if rotation == (0, 0) and abs(self.throttle_actual) <= self.notch_r:
            self.throttle_actual = int(0)
        if self.rotating_right and rotation == (1, 0) and self.throttle_actual <= self.notch_r:
            self.throttle_actual = int(0)
        if self.rotating_left and rotation == (0, 1) and self.throttle_actual <= self.notch_r:
            self.throttle_actual = int(0)

        if self.throttle_actual == 0:
            self.steer_actual = 0
            if self.timeout > 0:
                self.timeout -= 1
        else:
            self.timeout = self.timeout_max

        if self.timeout != 0:
            msg = str(self.throttle_actual) + ':' + str(self.steer_actual)
        else:
            msg = None

        return msg

    def camera(self):
        pygame.event.pump()
        self.joy_hat = self.controls.get_hat(0)
        for j in range(13):
            self.joy_buttons[j] = self.controls.get_button(j)

        top_camera_ctr = (self.joy_buttons[0], self.joy_buttons[1], self.joy_buttons[2], self.joy_buttons[3])

        if self.joy_hat == (0, 0) and top_camera_ctr == (0, 0, 0, 0):
            self.cameras_halt = True
        else:
            self.cameras_halt = False

        if self.joy_hat == (1, 0) and top_camera_ctr == (0, 0, 0, 0):
            msg = "!" + str(self.front_cam_contiuous + self.continuous_motion)
        elif self.joy_hat == (-1, 0) and top_camera_ctr == (0, 0, 0, 0):
            msg = "!" + str(self.front_cam_contiuous - self.continuous_motion)
        elif self.joy_hat == (0, 1) and top_camera_ctr == (0, 0, 0, 0):
            if self.front_cam_position_actual < self.front_cam_position_max:
                self.front_cam_position_actual += 1
            msg = "@" + str(self.front_cam_position_actual)
        elif self.joy_hat == (0, -1) and top_camera_ctr == (0, 0, 0, 0):
            if self.front_cam_position_actual > self.front_cam_position_min:
                self.front_cam_position_actual -= 1
            msg = "@" + str(self.front_cam_position_actual)
        elif self.joy_hat == (0, 0) and top_camera_ctr == (0, 1, 0, 0):
            msg = "#" + str(self.top_cam_continuous + self.continuous_motion)
        elif self.joy_hat == (0, 0) and top_camera_ctr == (0, 0, 0, 1):
            msg = "#" + str(self.top_cam_continuous - self.continuous_motion)
        elif self.joy_hat == (0, 0) and top_camera_ctr == (0, 0, 1, 0):
            if self.top_cam_position_actual < self.top_cam_position_max:
                self.top_cam_position_actual += 1
            msg = "$" + str(self.top_cam_position_actual)
        elif self.joy_hat == (0, 0) and top_camera_ctr == (1, 0, 0, 0):
            if self.top_cam_position_actual > self.top_cam_position_min:
                self.top_cam_position_actual -= 1
            msg = "$" + str(self.top_cam_position_actual)
        else:
            msg = None

        if self.cameras_halt == False:
            self.cameras_timeout = self.cameras_timeout_max
            time.sleep(self.long_lapse)
            return msg
        else:
            if self.cameras_timeout > 0:
                self.cameras_timeout -= 1
                if self.cameras_timeout % 2 == 0:
                    msg = "!" + str(self.front_cam_contiuous)
                else:
                    msg = "#" + str(self.top_cam_continuous)
                time.sleep(self.long_lapse)
                return msg
            else:
                return None

    def arm(self):
        pygame.event.pump()
        self.joy_hat = self.controls.get_hat(0)

        for i in range(6):
            self.joy_axis[i] = self.controls.get_axis(i)

        self.joy_axis[1] *= -1
        self.joy_axis[4] *= -1

        for j in range(13):
            self.joy_buttons[j] = self.controls.get_button(j)

        twist = (self.joy_buttons[4], self.joy_buttons[5])
        clutch = (self.joy_buttons[2], self.joy_buttons[3])

        if self.joy_hat[0] == int(0):
            self.arm_motor1 = self.arm_halt
        elif self.joy_hat[0] == int(1):
            self.arm_motor1 = self.arm_backward
        elif self.joy_hat[0] == int(-1):
            self.arm_motor1 = self.arm_forward

        if self.joy_hat[1] == int(0):
            self.arm_motor2 = self.arm_halt
        elif self.joy_hat[1] == int(1):
            self.arm_motor2 = self.arm_forward
        elif self.joy_hat[1] == int(-1):
            self.arm_motor2 = self.arm_backward

        if self.joy_axis[1] > 0.4:
            self.arm_motor3 = self.arm_forward
        elif self.joy_axis[1] < -0.4:
            self.arm_motor3 = self.arm_backward
        else:
            self.arm_motor3 = self.arm_halt

        if self.joy_axis[4] > 0.4:
            self.arm_motor4 = self.arm_forward
        elif self.joy_axis[4] < -0.4:
            self.arm_motor4 = self.arm_backward
        else:
            self.arm_motor4 = self.arm_halt

        if twist == (0, 1):
            self.arm_motor5 = self.arm_backward
        elif twist == (1, 0):
            self.arm_motor5 = self.arm_forward
        else:
            self.arm_motor5 = self.arm_halt

        if clutch == (0, 1):
            self.arm_motor6 = self.arm_backward
        elif clutch == (1, 0):
            self.arm_motor6 = self.arm_forward
        else:
            self.arm_motor6 = self.arm_halt

        #time.sleep(self.long_lapse)

        msg = "budge " + self.arm_motor1 + " " + self.arm_motor2 + " " + self.arm_motor3 + " " + self.arm_motor4 + " " + self.arm_motor5 + " " + self.arm_motor6

        if msg == "budge ~ ~ ~ ~ ~ ~":
            if self.timeout > 0:
                self.timeout -= 1
        else:
            self.timeout = self.timeout_max

        if self.timeout == 0:
            return None
        else:
            time.sleep(self.long_lapse)
            return msg

    def arm_reset(self):
        pygame.event.pump()
        temp = self.controls.get_button(11)
        if self.arm_reset_button == False and temp == 1:
            self.arm_reset_button = True
            return None
        elif self.arm_reset_button == True and temp == 0:
            self.arm_reset_button = False
            return "reset"
        else:
            return None


    def wheels_act(self):
        pygame.event.pump()
        temp = self.controls.get_button(12)
        if self.wheels_activate == False and temp == 1:
            self.wheels_activate = True
            return None
        elif self.wheels_activate == True and temp == 0:
            self.wheels_activate = False
            return "activate"
        else:
            return None

    def wheels_deact(self):
        pygame.event.pump()
        temp = self.controls.get_button(11)
        if self.wheels_deactivate == False and temp == 1:
            self.wheels_deactivate = True
            return None
        elif self.wheels_deactivate == True and temp == 0:
            self.wheels_deactivate = False
            return "deactivate"
        else:
            return None

    def switch(self):
        pygame.event.pump()
        temp = self.controls.get_button(9)

        if self.switch_button == False and temp == 1:
            self.switch_button = True
        elif self.switch_button == True and temp == 0:
            if self.isRover:
                self.isRover = False
                self.isArm = True
            else:
                self.isRover = True
                self.isArm = False
            self.switch_button = False
        else:
            pass


if __name__ == '__main__':
    my_joy = Astro_Joy(45, 25, True)

    try:
        while True:

            # data = my_joy.wheels()
            # data = my_joy.arm()
            # data = my_joy.camera()

            # if data != None:
            #   print(data)
            my_joy.switch()
            time.sleep(0.01)
            # time.sleep(0.05)
            # print("Rover mode : {}".format(my_joy.isRover))
            # print("Arm mode : {}".format(my_joy.isArm))
            if my_joy.isRover:
                temp = my_joy.wheels_act()
                if temp is not None:
                    print(temp)
                temp = my_joy.wheels_deact()
                if temp is not None:
                    print(temp)
                temp = my_joy.wheels()
                if temp is not None:
                    print(temp)
                temp = my_joy.camera()
                if temp is not None:
                    print(temp)
                #print("currently controlling : Rover")
            else:
                temp = my_joy.arm_reset()
                if temp is not None:
                    print(temp)
                temp = my_joy.arm()
                if temp is not None:
                    print(temp)
                #print("currently controlling : Arm")


    except KeyboardInterrupt:
        print("Exiting now")

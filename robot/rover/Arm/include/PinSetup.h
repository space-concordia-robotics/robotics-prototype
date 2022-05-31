#ifndef PINSETUP_H
#define PINSETUP_H

// pin constants defined in core_pins.h, pins_teensy.c, pins_arduino.h

/*
   m1 base dc (rotation/swivel)
   m2 shoulder dc (flexion)
   m3 elbow dc (flexion)
   m4 wrist stepper (flexion): to be replaced by dc later
   m5 wrist servo (rotation/twisting)
   m6 end effector servo (pinching)
*/

// 6 pwm pins (currently 5), 4 direction pins
// 8 to 10 limit switch interrupt pins (m1,2,3,4, hopefully m6)
// 8 or 12 encoder interrupt pins (12 if servo mod happens)

#define NUM_MOTORS 6 //!< used in parsing for commands for multiple motors

/* brief angle limits for all motors
 *
 * TODO I forget if I fixed this, but these angle limits need to be ignored
 * for the wrist that rotates as it shouldn't have limits
*/
#define MIN_JOINT_ANGLE -100000
#define MAX_JOINT_ANGLE 100000
#define STEPPER_PID_PERIOD 25 * 1000 // initial value for constant speed, but adjusted in variable speed modes
#define DC_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define SERVO_STOP 1500 // microsecond count which stops continuous rotation servos 

// DO NOT CHANGE THIS BECAUSE THE CIRCUITRY IS DESIGNED FOR PULLPUP RESISTORS ON LIMIT SWITCHES
#define LIM_SWITCH_FALL 1 // triggered by falling edge
//#define LIM_SWITCH_RISE 2 // triggered by rising edge

#if defined(LIM_SWITCH_FALL)
#define LIM_SWITCH_PULLSTATE INPUT_PULLUP
#elif defined(LIM_SWITCH_RISE)
#define LIM_SWITCH_PULLSTATE INPUT_PULLDOWN
#endif

#define LIM_SWITCH_DIR CHANGE // interrupts occur on changing pin state because it's software debounced

#define TRIGGER_DELAY 10 // how long to wait to make sure a limit switch was pressed and ignore bouncing

#define V_SENSE 39 // battery sensing pin

// DC motors
// Motor 1 Shoulder swivel
#define M1_DIR_PIN          7 // chooses the direction the motor turns in
#define M1_PWM_PIN          9 // the speed of the motor is controlled by the pwm signal

#define M1_MIN_HARD_ANGLE     -175.0 // angle at which the join presses the limit switch
#define M1_MAX_HARD_ANGLE     175.0
#define M1_MIN_SOFT_ANGLE     -170.0 // hard angle with safety margin to avoid hitting limit switch after homing
#define M1_MAX_SOFT_ANGLE     170.0

// Motor 2 Shoulder bend
#define M2_DIR_PIN            3
#define M2_PWM_PIN            5
#define M2_MIN_HARD_ANGLE     -65.0
#define M2_MAX_HARD_ANGLE     23.0
#define M2_MIN_SOFT_ANGLE     -62.0
#define M2_MAX_SOFT_ANGLE     20.0

//Motor 3 Elbow bend
#define M3_DIR_PIN         8
#define M3_PWM_PIN         10
#define M3_MIN_HARD_ANGLE     -145.0
#define M3_MAX_HARD_ANGLE     65.0
#define M3_MIN_SOFT_ANGLE     -140.0
#define M3_MAX_SOFT_ANGLE     60.0

//Motor 4 Wrist bend
#define M4_DIR_PIN        4
#define M4_PWM_PIN        6
#define M4_MIN_HARD_ANGLE  -90.0
#define M4_MAX_HARD_ANGLE  75.0
#define M4_MIN_SOFT_ANGLE  -85.0
#define M4_MAX_SOFT_ANGLE  70.0

// SERVOS
// No encoder/pwm/dir pins because smart servos use UART alone

// Motor 5 Wrist swivel
#define M5_GEAR_RATIO      27.0 // worm gear
// no angle limits/limit switches because it can spin indefinitely

// Motor 6 Pinch

#define M6_GEAR_RATIO      27.0
#define M6_MIN_HARD_ANGLE   -75.0 //-120.0
#define M6_MAX_HARD_ANGLE   75.0 // 150.0 //30.0
#define M6_MIN_SOFT_ANGLE   -75.0//-65.0 //-120.0
#define M6_MAX_SOFT_ANGLE   75.0//65.0 // 150.0 //30.0


void pinSetup(void);

#endif

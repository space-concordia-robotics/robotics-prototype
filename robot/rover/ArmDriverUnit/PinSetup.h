#ifndef PINSETUP_H
#define PINSETUP_H

// pin constants defined in core_pins.h, pins_teensy.c, pins_arduino.h

/*
   m1 base stepper (rotation)
   m2 shoulder dc (flexion)
   m3 elbow stepper (flexion)
   m4 wrist stepper (flexion
   m5 wrist servo (rotation)
   m6 end effector servo (pinching)
*/

enum motor_code
{
  MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6
}; // defines 6 motors

// 3 pwm pins
// 4-6 limit switch interrupt pins
// 8-14 encoder interrupt pins
// 3 direction pins, 3 step pins

#define NUM_MOTORS 6 // used in parsing for commands for multiple motors
#define MIN_JOINT_ANGLE -760 // min angle for all motors
#define MAX_JOINT_ANGLE 760 // max angle for all motors
#define STEPPER_PID_PERIOD 25 * 1000 // initial value for constant speed, but adjusted in variable speed modes
#define DC_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define STEPPER_CHECK_INTERVAL 2000 // much longer period, used for testing/debugging
// #define STEPPER_CHECK_INTERVAL 250 // every 250ms check if the stepper is in the right spot

#define LIM_SWITCH_FALL 1 // triggered by falling edge
//#define LIM_SWITCH_RISE 2 // triggered by rising edge

#if defined(LIM_SWITCH_FALL)
#define LIM_SWITCH_PULLSTATE INPUT_PULLUP
#elif defined(LIM_SWITCH_RISE)
#define LIM_SWITCH_PULLSTATE INPUT_PULLDOWN
#endif

#define SERVO_STOP 189 // 3.3v // motor is supposed to stop at 50% duty cycle (127/255)
//#define SERVO_STOP 127 // 5v // motor is supposed to stop at 50% duty cycle (127/255)
//only needed for sabertooth
//#define DC_STOP 189 // 3.3v // motor is supposed to stop at 50% duty cycle (127/255)
//#define DC_STOP 127 // 5v // motor is supposed to stop at 50% duty cycle (127/255)

// can go into ArmMotor
// limits in ms for amount of time the motor can budge
#define MAX_BUDGE_TIME 3000
#define MIN_BUDGE_TIME 100
#define DEFAULT_BUDGE_TIME 500
#define DEFAULT_SPEED 1
#define MAX_SPEED 4
#define MAX_COUNTS 4

// steppers

/*
  #define M1_ENABLE_PIN       2
  #define M1_DIR_PIN          5
  #define M1_STEP_PIN         6
  // 7&8 are on port D with bits 2&3 respectively
  #define M1_ENCODER_PORT    GPIOD_PDIR
  #define M1_ENCODER_SHIFT   CORE_PIN7_BIT
  #define M1_ENCODER_A        7
  #define M1_ENCODER_B        8
  #define M1_LIMIT_SW_CW      9
  #define M1_LIMIT_SW_CCW    10
  #define M1_ENCODER_RESOLUTION 2000 // temporary, unknown
  #define M1_STEP_RESOLUTION 1.8 // I think it's the same for all our steppers
  #define M1_GEAR_RATIO      40.0 // temporary, unknown
  #define M1_MINIMUM_ANGLE   -175.0
  #define M1_MAXIMUM_ANGLE   175.0
*/

#define M3_ENABLE_PIN      17
#define M3_STEP_PIN        21
#define M3_DIR_PIN         20
// 19&18 are on port B with bits 2&3 respectively
#define M3_ENCODER_PORT    GPIOB_PDIR
#define M3_ENCODER_SHIFT   CORE_PIN19_BIT
#define M3_ENCODER_A       19
#define M3_ENCODER_B       18
#define M3_LIMIT_SW_FLEX   22
#define M3_LIMIT_SW_EXTEND 23
#define M3_ENCODER_RESOLUTION 2000
#define M3_STEP_RESOLUTION 1.8 // I think it's the same for all our steppers
#define M3_GEAR_RATIO      36.0 // belt reduction chained to worm gear drive
#define M3_MINIMUM_ANGLE   -115.0
#define M3_MAXIMUM_ANGLE   35.0

#define M4_ENABLE_PIN      24
#define M4_STEP_PIN        14
#define M4_DIR_PIN         25
// 11&12 are on port C with bits 6&7 respectively
#define M4_ENCODER_PORT    GPIOC_PDIR
#define M4_ENCODER_SHIFT   CORE_PIN11_BIT
#define M4_ENCODER_A       11
#define M4_ENCODER_B       12
#define M4_LIMIT_SW_FLEX   15
#define M4_LIMIT_SW_EXTEND 16
#define M4_ENCODER_RESOLUTION 2000
#define M4_STEP_RESOLUTION 1.8 // I think it's the same for all our steppers
#define M4_GEAR_RATIO      35.55555555 // belt reduction chained to worm gear drive
#define M4_MINIMUM_ANGLE   -55.0
#define M4_MAXIMUM_ANGLE   46.0//40.0

// DC motors

#define M1_DIR_PIN          5
#define M1_PWM_PIN          6
// 7&8 are on port D with bits 2&3 respectively
#define M1_ENCODER_PORT    GPIOD_PDIR
#define M1_ENCODER_SHIFT   CORE_PIN7_BIT
#define M1_ENCODER_A        7
#define M1_ENCODER_B        8
#define M1_LIMIT_SW_CW      9
#define M1_LIMIT_SW_CCW    10
#define M1_ENCODER_RESOLUTION 7*2 // temporary, unknown
#define M1_GEAR_RATIO      40.0 // temporary, unknown
#define M1_MINIMUM_ANGLE   -175.0
#define M1_MAXIMUM_ANGLE   175.0

#define M2_PWM_PIN         30
#define M2_DIR_PIN         31 // for new driver
//#define M2_UART_TX       CORE_TXD4_PIN  // 32 for sabertooth
//#define M2_UART_RX       CORE_RXD4_PIN  // 31 for sabertooth
// 26&27 are on port A with bits 14&15 respectively
#define M2_ENCODER_PORT    GPIOA_PDIR
#define M2_ENCODER_SHIFT   CORE_PIN26_BIT
#define M2_ENCODER_A       26
#define M2_ENCODER_B       27
#define M2_LIMIT_SW_FLEX   28
#define M2_LIMIT_SW_EXTEND 29
#define M2_ENCODER_RESOLUTION 7*2 //7*4 //48 // counts per motor shaft revolution
#define M2_GEAR_RATIO      99.508*20.0 //99.508*33.888 //188 planetary gear chained to worm gear drive
#define M2_MINIMUM_ANGLE   -75.0
#define M2_MAXIMUM_ANGLE   55.0

// servos

#define M5_PWM_PIN         35
/*
  // 33&34 are on port E with bits 24&25 respectively
  #define M5_ENCODER_PORT    GPIOE_PDIR
  #define M5_ENCODER_A       33
  #define M5_ENCODER_B       34
*/
#define M5_GEAR_RATIO      20.0 // there is a ratio here that I don't know yet
//#define M5_MINIMUM_ANGLE
//#define M5_MAXIMUM_ANGLE
// no angle limits because this one can be used as a screwdriver

#define M6_PWM_PIN         36
/*
  // 37&38 are on port C with bits 10&11 respectively
  #define M6_ENCODER_PORT    GPIOC_PDIR
  #define M6_ENCODER_A       37
  #define M6_ENCODER_B       38
*/
#define M6_GEAR_RATIO      20.0 // there is a ratio here that I don't know yet
#define M6_MINIMUM_ANGLE   -120.0
#define M6_MAXIMUM_ANGLE   30.0

void pinSetup(void);

#endif

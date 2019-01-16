#ifndef PINSETUP_H
#define PINSETUP_H

// pin constants defined in core_pins.h, pins_teensy.c, pins_arduino.h

enum motor_code
{
  MOTOR1 = 1, MOTOR2
}; // defines 6 motors

// 1 pwm pin
// 1 direction pin
// 2 limit switch interrupt pins
// 2 encoder interrupt pins

#define NUM_MOTORS 2 // used in parsing for commands for multiple motors
#define MIN_JOINT_ANGLE -760 // min angle for all motors
#define MAX_JOINT_ANGLE 760 // max angle for all motors
#define DC_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods

#define LIM_SWITCH_FALL 1 // triggered by falling edge
//#define LIM_SWITCH_RISE 2 // triggered by rising edge

#if defined(LIM_SWITCH_FALL)
#define LIM_SWITCH_PULLSTATE INPUT_PULLUP
#elif defined(LIM_SWITCH_RISE)
#define LIM_SWITCH_PULLSTATE INPUT_PULLDOWN
#endif

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
#define M2_GEAR_RATIO      189.0*20.0 //99.508*20 //188 planetary gear chained to worm gear drive
#define M2_MINIMUM_ANGLE   -75.0
#define M2_MAXIMUM_ANGLE   55.0

void pinSetup(void);

#endif

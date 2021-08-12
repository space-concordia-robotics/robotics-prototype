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
#define M1_ENCODER_A       37 // pin both pins need to be in the same port and next to each other in the register
#define M1_ENCODER_B       38 // pin
#define M1_ENCODER_PORT    GPIOC_PDIR // GPIO Port C Data Input Register (changes based on pins chosen)
#define M1_ENCODER_SHIFT   CORE_PIN37_BIT // the position of the lower encoder pin bit in the input register. 

#define M1_LIMIT_SW_CCW       23 // LS 1 limit switch, triggers when join over rotates
#define M1_LIMIT_SW_CCW_PORT  GPIOC_PDIR // Pin port data input register
#define M1_LIMIT_SW_CCW_SHIFT CORE_PIN23_BIT // location of pin in register
#define M1_LIMIT_SW_CW        22 // LS 2
#define M1_LIMIT_SW_CW_PORT   GPIOC_PDIR
#define M1_LIMIT_SW_CW_SHIFT  CORE_PIN22_BIT

#define M1_ENCODER_RESOLUTION 48
#define M1_GEAR_RATIO         1.4*99.508*40.0 //TODO fix this based on gear reduction inside and outside motor
#define M1_MIN_HARD_ANGLE     -175.0 // angle at which the join presses the limit switch
#define M1_MAX_HARD_ANGLE     175.0
#define M1_MIN_SOFT_ANGLE     -170.0 // hard angle with safety margin to avoid hitting limit switch after homing
#define M1_MAX_SOFT_ANGLE     170.0

// Motor 2 Shoulder bend
#define M2_DIR_PIN            3
#define M2_PWM_PIN            5
#define M2_ENCODER_A       11
#define M2_ENCODER_B       12
#define M2_ENCODER_PORT    GPIOC_PDIR
#define M2_ENCODER_SHIFT   CORE_PIN11_BIT

#define M2_LIMIT_SW_EXTEND    20 // LS 3  
#define M2_LIMIT_SW_EXTEND_PORT  GPIOD_PDIR
#define M2_LIMIT_SW_EXTEND_SHIFT CORE_PIN20_BIT
#define M2_LIMIT_SW_FLEX      18 // LS 4
#define M2_LIMIT_SW_FLEX_PORT    GPIOB_PDIR
#define M2_LIMIT_SW_FLEX_SHIFT   CORE_PIN18_BIT

#define M2_ENCODER_RESOLUTION 48
#define M2_GEAR_RATIO         99.508*20.0*2.0 // planetary gear motor chained to worm gear drive
#define M2_MIN_HARD_ANGLE     -65.0
#define M2_MAX_HARD_ANGLE     23.0
#define M2_MIN_SOFT_ANGLE     -62.0
#define M2_MAX_SOFT_ANGLE     20.0

//Motor 3 Elbow bend
#define M3_DIR_PIN         8
#define M3_PWM_PIN         10
#define M3_ENCODER_A          29
#define M3_ENCODER_B          30
#define M3_ENCODER_PORT       GPIOB_PDIR
#define M3_ENCODER_SHIFT      CORE_PIN29_BIT

#define M3_LIMIT_SW_EXTEND        16 // LS 5
#define M3_LIMIT_SW_EXTEND_PORT   GPIOB_PDIR
#define M3_LIMIT_SW_EXTEND_SHIFT  CORE_PIN16_BIT
#define M3_LIMIT_SW_FLEX          21 // LS 6
#define M3_LIMIT_SW_FLEX_PORT     GPIOD_PDIR
#define M3_LIMIT_SW_FLEX_SHIFT    CORE_PIN21_BIT

#define M3_ENCODER_RESOLUTION 48
#define M3_GEAR_RATIO         .88*99.508*(40.0/14.0)*18.0*2.0 // belt reduction chained to worm gear drive
#define M3_MIN_HARD_ANGLE     -145.0
#define M3_MAX_HARD_ANGLE     65.0
#define M3_MIN_SOFT_ANGLE     -140.0
#define M3_MAX_SOFT_ANGLE     60.0

//Motor 4 Wrist bend
#define M4_DIR_PIN        4
#define M4_PWM_PIN        6
#define M4_ENCODER_A        27
#define M4_ENCODER_B        28
#define M4_ENCODER_PORT    GPIOA_PDIR
#define M4_ENCODER_SHIFT   CORE_PIN27_BIT

#define M4_LIMIT_SW_EXTEND 19 // LS 7
#define M4_LIMIT_SW_EXTEND_PORT  GPIOB_PDIR
#define M4_LIMIT_SW_FLEX_SHIFT   CORE_PIN17_BIT
#define M4_LIMIT_SW_FLEX   17 // LS 8
#define M4_LIMIT_SW_FLEX_PORT    GPIOB_PDIR
#define M4_LIMIT_SW_EXTEND_SHIFT CORE_PIN19_BIT

#define M4_ENCODER_RESOLUTION 48
#define M4_GEAR_RATIO      139.138*20.0 // belt reduction chained to worm gear drive
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
#define M6_LIMIT_SW_FLEX 15 // LS 9 
#define M6_LIMIT_SW_FLEX_PORT  GPIOC_PDIR
#define M6_LIMIT_SW_FLEX_SHIFT CORE_PIN15_BIT
#define M6_LIMIT_SW_EXTEND 14 // LS 10 
#define M6_LIMIT_SW_EXTEND_PORT  GPIOD_PDIR
#define M6_LIMIT_SW_EXTEND_SHIFT CORE_PIN14_BIT

#define M6_GEAR_RATIO      27.0
#define M6_MIN_HARD_ANGLE   -75.0 //-120.0
#define M6_MAX_HARD_ANGLE   75.0 // 150.0 //30.0
#define M6_MIN_SOFT_ANGLE   -75.0//-65.0 //-120.0
#define M6_MAX_SOFT_ANGLE   75.0//65.0 // 150.0 //30.0


void pinSetup(void);

#endif

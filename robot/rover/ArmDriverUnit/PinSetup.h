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
/*! \brief angle limits for all motors
 *
 * \todo I forget if I fixed this, but these angle limits need to be ignored
 * for the wrist that rotates as it shouldn't have limits
*/
#define MIN_JOINT_ANGLE -760
#define MAX_JOINT_ANGLE 760
#define STEPPER_PID_PERIOD 25 * 1000 //!< initial value for constant speed, but adjusted in variable speed modes
#define DC_PID_PERIOD 20000 //!< 20ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 20000 //!< 20ms, because typical pwm signals have 20ms periods
#define SERVO_STOP 1500 //!< microsecond count which stops continuous rotation servos 
#define STEPPER_CHECK_INTERVAL 2000 //!< much longer period, used for testing/debugging
// #define STEPPER_CHECK_INTERVAL 250 //!< every 250ms check if the stepper is in the right spot

// DO NOT CHANGE THIS BECAUSE THE CIRCUITRY IS DESIGNED FOR PULLPUP RESISTORS ON LIMIT SWITCHES
#define LIM_SWITCH_FALL 1 //!< triggered by falling edge
//#define LIM_SWITCH_RISE 2 //!< triggered by rising edge

#if defined(LIM_SWITCH_FALL)
#define LIM_SWITCH_PULLSTATE INPUT_PULLUP
#elif defined(LIM_SWITCH_RISE)
#define LIM_SWITCH_PULLSTATE INPUT_PULLDOWN
#endif

#define LIM_SWITCH_DIR CHANGE //!< interrupts occur on changing pin state because it's software debounced

#define TRIGGER_DELAY 10 //!< how long to wait to make sure a limit switch was pressed and ignore bouncing

// DC motors
/*
#define M1_DIR_PIN          5 //!< chooses the direction the motor turns in
#define M1_PWM_PIN          6 //!< the speed of the motor is controlled by the pwm signal
// 7&8 are on port D with bits 2&3 respectively
#define M1_ENCODER_PORT    GPIOD_PDIR //!< the input register for the port connected to the encoder pins
//! \brief the position of the lower encoder pin bit. The encoder interrupt code expects this to function correctly.
#define M1_ENCODER_SHIFT   CORE_PIN7_BIT
#define M1_ENCODER_A        7 //!< encoder A pin
#define M1_ENCODER_B        8 //!< encoder B pin
#define M1_LIMIT_SW_CW_PORT   GPIOC_PDIR
#define M1_LIMIT_SW_CCW_PORT  GPIOC_PDIR
#define M1_LIMIT_SW_CW_SHIFT  CORE_PIN9_BIT
#define M1_LIMIT_SW_CCW_SHIFT CORE_PIN10_BIT
#define M1_LIMIT_SW_CW        9
#define M1_LIMIT_SW_CCW       10
*/
// swapping m1 pins with m3 pins so i can control m3
// using the current setup since m1 isn't being used right now
// and I need to test m2-4 for homing
#define M1_DIR_PIN            20
#define M1_PWM_PIN            21
#define M1_ENCODER_PORT       GPIOB_PDIR
#define M1_ENCODER_SHIFT      CORE_PIN19_BIT
#define M1_ENCODER_A          19
#define M1_ENCODER_B          18
#define M1_LIMIT_SW_CW_PORT   GPIOC_PDIR
#define M1_LIMIT_SW_CCW_PORT  GPIOC_PDIR
#define M1_LIMIT_SW_CW_SHIFT  CORE_PIN22_BIT
#define M1_LIMIT_SW_CCW_SHIFT CORE_PIN23_BIT
#define M1_LIMIT_SW_CW        22
#define M1_LIMIT_SW_CCW       23
// end of swap
#define M1_ENCODER_RESOLUTION 48
#define M1_GEAR_RATIO         40.0 //!< \todo fix this based on gear reduction inside and outside motor
#define M1_MIN_HARD_ANGLE     -350.0 //!< \todo work this out maybe with max or dharik
#define M1_MAX_HARD_ANGLE     350.0 //!< \todo work this out maybe with max or dharik
#define M1_MIN_SOFT_ANGLE     -325.0
#define M1_MAX_SOFT_ANGLE     325.0

#define M2_PWM_PIN            30
#define M2_DIR_PIN            31
// 26&27 are on port A with bits 14&15 respectively
#define M2_ENCODER_PORT       GPIOA_PDIR
#define M2_ENCODER_SHIFT      CORE_PIN26_BIT
#define M2_ENCODER_A          26
#define M2_ENCODER_B          27
#define M2_LIMIT_SW_FLEX_PORT    GPIOA_PDIR //!< input register for the port associated to flex limit switch pin
#define M2_LIMIT_SW_EXTEND_PORT  GPIOB_PDIR //!< input register for the port associated to extend limit switch pin
#define M2_LIMIT_SW_FLEX_SHIFT   CORE_PIN28_BIT //!< position of the limit switch flex pin bit
#define M2_LIMIT_SW_EXTEND_SHIFT CORE_PIN29_BIT //!< position of the limit switch extend pin bit
#define M2_LIMIT_SW_FLEX      28 //!< limit switch flex pin
#define M2_LIMIT_SW_EXTEND    29 //!< limit switch extend pin
#define M2_ENCODER_RESOLUTION 48
//! planetary gear motor chained to worm gear drive
#define M2_GEAR_RATIO         188.611 //*20.0 //99.508*20
#define M2_MIN_HARD_ANGLE     -65.0 //!< the flexion angle at which the joint presses the limit switch
#define M2_MAX_HARD_ANGLE     50.0 //!< the extension angle at which the joint presses the limit switch
#define M2_MIN_SOFT_ANGLE     -55.0 //!< a safety margin is added to the flexion angle to avoid ever hitting the limit switch after homing is complete
#define M2_MAX_SOFT_ANGLE     40.0 //!< a safety margin is added to the extension angle to avoid ever hitting the limit switch after homing is complete

/*
//#define M3_ENABLE_PIN      17
//#define M3_STEP_PIN        21
#define M3_DIR_PIN         20 // stays the same
#define M3_PWM_PIN         21
// 19&18 are on port B with bits 2&3 respectively
#define M3_ENCODER_PORT    GPIOB_PDIR
#define M3_ENCODER_SHIFT   CORE_PIN19_BIT
#define M3_ENCODER_A       19
#define M3_ENCODER_B       18
#define M3_LIMIT_SW_FLEX_PORT     GPIOC_PDIR
#define M3_LIMIT_SW_EXTEND_PORT   GPIOC_PDIR
#define M3_LIMIT_SW_FLEX_SHIFT    CORE_PIN22_BIT
#define M3_LIMIT_SW_EXTEND_SHIFT  CORE_PIN23_BIT
#define M3_LIMIT_SW_FLEX          22
#define M3_LIMIT_SW_EXTEND        23
*/
// swapping m1 pins with m3 pins so i can control m3
// using the current setup since m1 isn't being used right now
// and I need to test m2-4 for homing
#define M3_DIR_PIN            5
#define M3_PWM_PIN            6
#define M3_ENCODER_PORT       GPIOD_PDIR
#define M3_ENCODER_SHIFT      CORE_PIN7_BIT
#define M3_ENCODER_A          7
#define M3_ENCODER_B          8
#define M3_LIMIT_SW_FLEX_PORT   GPIOC_PDIR
#define M3_LIMIT_SW_EXTEND_PORT  GPIOC_PDIR
#define M3_LIMIT_SW_FLEX_SHIFT  CORE_PIN9_BIT
#define M3_LIMIT_SW_EXTEND_SHIFT CORE_PIN10_BIT
#define M3_LIMIT_SW_FLEX      9
#define M3_LIMIT_SW_EXTEND    10
// end of swap
#define M3_ENCODER_RESOLUTION 48
//#define M3_STEP_RESOLUTION 1.8 // I think it's the same for all our steppers
//! belt reduction chained to worm gear drive
#define M3_GEAR_RATIO         188.611*(40.0/14.0)*18.0
#define M3_MIN_HARD_ANGLE     -145.0
#define M3_MAX_HARD_ANGLE     65.0
#define M3_MIN_SOFT_ANGLE     -135.0
#define M3_MAX_SOFT_ANGLE     55.0

// stepper

#define M4_ENABLE_PIN      16 //!< controls whether power goes to stepper or not
#define M4_STEP_PIN        14 //!< on rising edges, stepper will take a step
#define M4_DIR_PIN         15 //!< chooses the step direction
// 11&12 are on port C with bits 6&7 respectively
#define M4_ENCODER_PORT    GPIOC_PDIR 
#define M4_ENCODER_SHIFT   CORE_PIN11_BIT
#define M4_ENCODER_A       11
#define M4_ENCODER_B       12
#define M4_LIMIT_SW_FLEX_PORT    GPIOA_PDIR
#define M4_LIMIT_SW_EXTEND_PORT  GPIOE_PDIR
#define M4_LIMIT_SW_FLEX_SHIFT   CORE_PIN25_BIT
#define M4_LIMIT_SW_EXTEND_SHIFT CORE_PIN24_BIT
#define M4_LIMIT_SW_FLEX   25
#define M4_LIMIT_SW_EXTEND 24
#define M4_ENCODER_RESOLUTION 2000 //!< counts per revolution of the stepper encoder
#define M4_STEP_RESOLUTION 1.8 //!< the angle traversed by the stepper stepping once
// the 1.3846 is a value i calculated based on the discrepancy between expected and actual angles
//! belt reduction chained to worm gear drive
#define M4_GEAR_RATIO      (30.0/18.0)*20.0/1.3846 //(30.0/18.0)*20.0 
#define M4_MIN_HARD_ANGLE  -90.0
#define M4_MAX_HARD_ANGLE  75.0
#define M4_MIN_SOFT_ANGLE  -80.0
#define M4_MAX_SOFT_ANGLE  65.0

// servos

#define M5_PWM_PIN         35
/*
  // 33&34 are on port E with bits 24&25 respectively
  #define M5_ENCODER_PORT    GPIOE_PDIR
  #define M5_ENCODER_A       33
  #define M5_ENCODER_B       34
*/
#define M5_GEAR_RATIO      84.0/12.0 // there is a ratio here that I don't know yet
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
#define M6_LIMIT_SW_EXTEND_PORT  GPIOE_PDIR // i need to pick a pin
#define M6_LIMIT_SW_EXTEND_SHIFT CORE_PIN24_BIT // i need to pick a pin
#define M6_LIMIT_SW_EXTEND 99 // i need to pick a pin
#define M6_GEAR_RATIO      40.0/12.0 // there is a ratio here that I don't know yet
#define M6_MIN_HARD_ANGLE   -75.0 //-120.0
#define M6_MAX_HARD_ANGLE   75.0 // 150.0 //30.0
#define M6_MIN_SOFT_ANGLE   -65.0 //-120.0
#define M6_MAX_SOFT_ANGLE   65.0 // 150.0 //30.0

/*! \brief Sets up all the Teensy pins and sends stop commands to motors that need it.
 * 
 * \todo Perhaps pinsetup.h & pinsetup.cpp should be changed to motorsetup
 * or just setup as it's also got angle limits and gear ratios
 * \todo If I think about it, physically there's the motor and the joint,
 * so the motor params (internal reduction, encoder resolution) could be
 * separate from the joint params (external reduction, angle limits)
*/
void pinSetup(void);

#endif

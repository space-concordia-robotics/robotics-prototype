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
#define MIN_JOINT_ANGLE -100000
#define MAX_JOINT_ANGLE 100000
#define STEPPER_PID_PERIOD 25 * 1000 //!< initial value for constant speed, but adjusted in variable speed modes
#define DC_PID_PERIOD 20000 //!< 20ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 20000 //!< 20ms, because typical pwm signals have 20ms periods
#define SERVO_STOP 1500 //!< microsecond count which stops continuous rotation servos 

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

#define V_SENSE 39 // battery sensing pin

// DC motors

#define M1_DIR_PIN          11 //!< chooses the direction the motor turns in
#define M1_PWM_PIN          10 //!< the speed of the motor is controlled by the pwm signal
// 27&28 are A15&A16
#define M1_ENCODER_PORT    GPIOA_PDIR //!< the input register for the port connected to the encoder pins
//! \brief the position of the lower encoder pin bit. The encoder interrupt code expects this to function correctly.
#define M1_ENCODER_SHIFT   CORE_PIN27_BIT
#define M1_ENCODER_A       27
#define M1_ENCODER_B       28
#define M1_LIMIT_SW_CW_PORT   GPIOC_PDIR
#define M1_LIMIT_SW_CCW_PORT  GPIOC_PDIR
#define M1_LIMIT_SW_CW_SHIFT  CORE_PIN22_BIT
#define M1_LIMIT_SW_CCW_SHIFT CORE_PIN23_BIT
#define M1_LIMIT_SW_CW        22 // LS 2
#define M1_LIMIT_SW_CCW       23 // LS 1
#define M1_ENCODER_RESOLUTION 48
#define M1_GEAR_RATIO         1.4*99.508*40.0 //!< \todo fix this based on gear reduction inside and outside motor
#define M1_MIN_HARD_ANGLE     -175.0
#define M1_MAX_HARD_ANGLE     175.0
#define M1_MIN_SOFT_ANGLE     -170.0
#define M1_MAX_SOFT_ANGLE     170.0

#define M2_PWM_PIN            8
#define M2_DIR_PIN            9
#define M2_ENCODER_PORT    GPIOB_PDIR
#define M2_ENCODER_SHIFT   CORE_PIN29_BIT
#define M2_ENCODER_A       29
#define M2_ENCODER_B       30
#define M2_LIMIT_SW_FLEX_PORT    GPIOD_PDIR //!< input register for the port associated to flex limit switch pin
#define M2_LIMIT_SW_EXTEND_PORT  GPIOD_PDIR //!< input register for the port associated to extend limit switch pin
#define M2_LIMIT_SW_FLEX_SHIFT   CORE_PIN20_BIT //!< position of the limit switch flex pin bit
#define M2_LIMIT_SW_EXTEND_SHIFT CORE_PIN21_BIT //!< position of the limit switch extend pin bit
#define M2_LIMIT_SW_FLEX      20 // LS 4 //!< limit switch flex pin
#define M2_LIMIT_SW_EXTEND    21 // LS 3 //!< limit switch extend pin
#define M2_ENCODER_RESOLUTION 48
//! planetary gear motor chained to worm gear drive
#define M2_GEAR_RATIO         99.508*20.0*2.0
#define M2_MIN_HARD_ANGLE     -65.0 //!< the flexion angle at which the joint presses the limit switch
#define M2_MAX_HARD_ANGLE     23.0 //!< the extension angle at which the joint presses the limit switch
#define M2_MIN_SOFT_ANGLE     -62.0 //!< a safety margin is added to the flexion angle to avoid ever hitting the limit switch after homing is complete
#define M2_MAX_SOFT_ANGLE     20.0 //!< a safety margin is added to the extension angle to avoid ever hitting the limit switch after homing is complete

#define M3_DIR_PIN         7
#define M3_PWM_PIN         6
#define M3_ENCODER_PORT       GPIOB_PDIR
#define M3_ENCODER_SHIFT      CORE_PIN31_BIT
#define M3_ENCODER_A          31
#define M3_ENCODER_B          32
#define M3_LIMIT_SW_FLEX_PORT     GPIOB_PDIR
#define M3_LIMIT_SW_EXTEND_PORT   GPIOB_PDIR
#define M3_LIMIT_SW_FLEX_SHIFT    CORE_PIN18_BIT
#define M3_LIMIT_SW_EXTEND_SHIFT  CORE_PIN19_BIT
#define M3_LIMIT_SW_FLEX          18 // LS 6
#define M3_LIMIT_SW_EXTEND        19 // LS 5
#define M3_ENCODER_RESOLUTION 48
//! belt reduction chained to worm gear drive
#define M3_GEAR_RATIO         .88*99.508*(40.0/14.0)*18.0*2.0 //188.611*(40.0/14.0)*18.0
#define M3_MIN_HARD_ANGLE     -145.0
#define M3_MAX_HARD_ANGLE     65.0
#define M3_MIN_SOFT_ANGLE     -140.0
#define M3_MAX_SOFT_ANGLE     60.0

#define M4_DIR_PIN        5 //!< chooses the step direction
#define M4_PWM_PIN        4
// 7&8 are on port E with bits 2&3 respectively
#define M4_ENCODER_PORT    GPIOE_PDIR //!< the input register for the port connected to the encoder pins
//! \brief the position of the lower encoder pin bit. The encoder interrupt code expects this to function correctly.
#define M4_ENCODER_SHIFT   CORE_PIN33_BIT
#define M4_ENCODER_A        33 //!< encoder A pin
#define M4_ENCODER_B        34 //!< encoder B pin
#define M4_LIMIT_SW_FLEX_PORT    GPIOB_PDIR
#define M4_LIMIT_SW_EXTEND_PORT  GPIOB_PDIR
#define M4_LIMIT_SW_FLEX_SHIFT   CORE_PIN16_BIT
#define M4_LIMIT_SW_EXTEND_SHIFT CORE_PIN17_BIT
#define M4_LIMIT_SW_FLEX   16 // LS 8
#define M4_LIMIT_SW_EXTEND 17 // LS 7
#define M4_ENCODER_RESOLUTION 48
// the 1.3846 is a value i calculated based on the discrepancy between expected and actual angles
//! belt reduction chained to worm gear drive
#define M4_GEAR_RATIO      139.138*20.0 // 60 rpm plus worm gear
#define M4_MIN_HARD_ANGLE  -90.0
#define M4_MAX_HARD_ANGLE  75.0
#define M4_MIN_SOFT_ANGLE  -85.0
#define M4_MAX_SOFT_ANGLE  70.0

// servos

#define M5_PWM_PIN         3
/*
  // 33&34 are on port E with bits 24&25 respectively
  #define M5_ENCODER_PORT    GPIOE_PDIR
  #define M5_ENCODER_A       33
  #define M5_ENCODER_B       34
*/
#define M5_GEAR_RATIO      27.0 // worm gear
//#define M5_MINIMUM_ANGLE
//#define M5_MAXIMUM_ANGLE
// no angle limits because this one can be used as a screwdriver

#define M6_PWM_PIN         2
/*
  // 37&38 are on port C with bits 10&11 respectively
  #define M6_ENCODER_PORT    GPIOC_PDIR
  #define M6_ENCODER_A       37
  #define M6_ENCODER_B       38
*/
#define M6_LIMIT_SW_EXTEND_PORT  GPIOC_PDIR // i need to pick a pin
#define M6_LIMIT_SW_EXTEND_SHIFT CORE_PIN36_BIT // i need to pick a pin
#define M6_LIMIT_SW_EXTEND 36 // i need to pick a pin
#define M6_LIMIT_SW_FLEX_PORT  GPIOD_PDIR // i need to pick a pin
#define M6_LIMIT_SW_FLEX_SHIFT CORE_PIN14_BIT // i need to pick a pin
#define M6_LIMIT_SW_FLEX 14 // i need to pick a pin
#define M6_GEAR_RATIO      27.0
#define M6_MIN_HARD_ANGLE   -75.0 //-120.0
#define M6_MAX_HARD_ANGLE   75.0 // 150.0 //30.0
#define M6_MIN_SOFT_ANGLE   -75.0//-65.0 //-120.0
#define M6_MAX_SOFT_ANGLE   75.0//65.0 // 150.0 //30.0
//gripper ext :10, pin 14, gpioD -- 11: pin 36, gpioC
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

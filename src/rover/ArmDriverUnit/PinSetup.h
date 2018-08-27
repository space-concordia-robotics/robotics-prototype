#ifndef PINSETUP_H
#define PINSETUP_H

#define HEARTBEAT_PIN 13

/*
 * m1 base stepper (rotation)
 * m2 shoulder dc (flexion)
 * m3 elbow stepper (flexion)
 * m4 wrist stepper (flexion
 * m5 wrist servo (rotation)
 * m6 end effector servo (pinching)
 */
 
// 3 pwm pins
// 4-6 limit switch interrupt pins
// 8-14 encoder interrupt pins
// 3 direction pins, 3 step pins

#define NUM_STEPPERS        3
#define NUM_SERVOS          2
#define NUM_LIM_SW          4

// steppers

#define M1_ENABLE_PIN       2
#define M1_DIR_PIN          5
#define M1_STEP_PIN         6
#define M1_ENCODER_A        7
#define M1_ENCODER_B        8
#define M1_LIMIT_SW_CW      9
#define M1_LIMIT_SW_CCW    10

#define M3_ENABLE_PIN      17
#define M3_DIR_PIN         18
#define M3_STEP_PIN        19
#define M3_ENCODER_A       20
#define M3_ENCODER_B       21
#define M3_LIMIT_SW_FLEX   22
#define M3_LIMIT_SW_EXTEND 23

#define M4_ENABLE_PIN      11
#define M4_DIR_PIN         12
#define M4_STEP_PIN        25
#define M4_ENCODER_A       24
#define M4_ENCODER_B       14
#define M4_LIMIT_SW_FLEX   15
#define M4_LIMIT_SW_EXTEND 16

// DC motor

#define M2_PWM_PIN         30
#define M2_UART_TX         32
#define M2_UART_RX         31
#define M2_ENCODER_A       26
#define M2_ENCODER_B       27
#define M2_LIMIT_SW_FLEX   28
#define M2_LIMIT_SW_EXTEND 29

// servos

#define M5_PWM_PIN         35
#define M5_ENCODER_A       34
#define M5_ENCODER_B       33

#define M6_PWM_PIN         36
#define M6_ENCODER_A       37
#define M6_ENCODER_B       38

void pinSetup(void);
#endif
#ifndef PINS_H
#define PINS_H

#define V_SENSE_PIN 39 // for reading battery voltage

/***** MOTORS *****/

// R/L (right/left), F/M/B (forward, middle, back)

/* motor driver pins (cytron) */

// direction pins
#define RF_DIR   2
#define RM_DIR   11
#define RB_DIR   12
#define LF_DIR   24
#define LM_DIR   25
#define LB_DIR   26

// pwm pins
#define RF_PWM   3
#define RM_PWM   4
#define RB_PWM   5
#define LF_PWM   6
#define LM_PWM   7
#define LB_PWM   8

/* encoder pins */
//
//// right side encoders
//#define RF_EA    27
//#define RF_EB    28
//#define RM_EA    31
//#define RM_EB    32
//#define RB_EA    29
//#define RB_EB    30
//
//// left side encoders
//#define LF_EA    37
//#define LF_EB    38
//#define LM_EA    36
//#define LM_EB    35
//#define LB_EA    33
//#define LB_EB    34

// right side encoders
#define RF_EA    33
#define RF_EB    34
#define RM_EA    37
#define RM_EB    38
#define RB_EA    36
#define RB_EB    35

// left side encoders
#define LF_EA    31
#define LF_EB    32
#define LM_EA    29
#define LM_EB    30
#define LB_EA    27
#define LB_EB    28

/* camera servo pins */
// F/R (front/rear), S/B (side/base)
#define FS_SERVO 22
#define FB_SERVO 23
#define RS_SERVO 16
#define RB_SERVO 17

void initPins(void) {
  pinMode(RF_DIR, OUTPUT);
  pinMode(RF_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);
  pinMode(RB_DIR, OUTPUT);
  pinMode(RB_PWM, OUTPUT);

  pinMode(LF_DIR, OUTPUT);
  pinMode(LF_PWM, OUTPUT);
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(LB_DIR, OUTPUT);
  pinMode(LB_PWM, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(V_SENSE_PIN, INPUT);
}

#endif

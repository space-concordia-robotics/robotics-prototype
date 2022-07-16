#ifndef PINSETUP_H
#define PINSETUP_H
// rewritten by Marc

/*
   This list may be inaccurate.
   m1 base dc (rotation/swivel)
   m2 shoulder dc (flexion)
   m3 elbow stepper (flexion)
   m4 wrist stepper (flexion)
   m5 wrist servo (rotation/twisting)
   m6 end effector servo (pinching)
*/

#define NUM_MOTORS 6  // used in parsing for commands for multiple motors

#define M1_DIR_PIN 7
#define M1_PWM_PIN 9

#define M2_DIR_PIN 3
#define M2_PWM_PIN 5

#define M3_DIR_PIN 8
#define M3_PWM_PIN 10

#define M4_DIR_PIN 4
#define M4_PWM_PIN 6

// motors 5 and 6 are not used

void pinSetup(void);

#endif

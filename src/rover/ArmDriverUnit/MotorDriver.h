#ifndef MotorDriver_h
#define MotorDriver_h

#define M1_STEP_HIGH        PORTD |=  0b10000000;
#define M1_STEP_LOW         PORTD &= ~0b10000000;

#define M2_STEP_HIGH        PORTD |=  0b00100000;
#define M2_STEP_LOW         PORTD &= ~0b00100000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; //quadrature encoder matrix

unsigned int c0 = 1600;  // was 2000 * sqrt( 2 * angle / accel )

struct stepperInfo {
  void (*dirFunc)(int);
  void (*stepFunc)();
  volatile float dir = 0;
  volatile float currentAngle = 0;
  volatile float desiredAngle = 0;
  volatile bool movementDone = true;
};

struct dcInfo {
  volatile int dir = 127; // 0 to 255 where 127 is the midpoint and stops the motor
  volatile float currentAngle = 0;
  volatile float desiredAngle = 0;
  volatile bool dcMovementDone = false;
  volatile long encCount = 0;
};

struct servoInfo {
  volatile int dir = 127; // 0 to 255 where 127 is the midpoint and stops the motor
  volatile float currentAngle = 0;
  volatile float desiredAngle = 0;
  volatile bool dcMovementDone = false;
  volatile long encCount = 0;
};

volatile stepperInfo steppers[NUM_STEPPERS];
char serialBuffer[BUFFER_SIZE];
volatile dcInfo dcMotor;
bool led_status = true;
int startTime = 0;

void M1Step() {
  M1_STEP_HIGH
  M1_STEP_LOW
}
void M1Dir(int d) {
  digitalWrite(M1_DIR_PIN, d);
}

void M2Step() {
  M2_STEP_HIGH
  M2_STEP_LOW
}
void M2Dir(int d) {
  digitalWrite(M2_DIR_PIN, d);
}

#endif
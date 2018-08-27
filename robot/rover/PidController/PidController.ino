//TODO: python needs to check for message prefixes and this isn't implemented yet
  
  
// Required libraries for Adafruit Motor Shield
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// #include <Wire.h> // for now not needed but in future we might need this library for I2C

// DC motor encoder library
#include <Encoder.h>

// Servo motor library
#include <Servo.h>

// Create motorshield class objects
Adafruit_MotorShield motorShield1 = Adafruit_MotorShield(0x61); // Creates class for first shield with address 61. Hardware must be set to this address
Adafruit_MotorShield motorShield2 = Adafruit_MotorShield(0x62); // Creates class for second shield with address 62. Hardware must be set to this address

// Connect stepper motors to shields
Adafruit_StepperMotor *motor1 = motorShield2.getStepper(200, 1); // Connects the first motor to the second shield
Adafruit_DCMotor *motor2 = motorShield1.getMotor(2); // Connects the second motor to the first shield
Adafruit_StepperMotor *motor3 = motorShield1.getStepper(200, 1); // Connects the third motor to the first shield
Adafruit_StepperMotor *motor4 = motorShield2.getStepper(200, 2); // Connects the fourth motor to the first shield

// Stepper motor objects
Servo motor5;
Servo motor6;

// Encoder object for DC motor
Encoder DCEncoder(3,2);

// multiplication factors for proportional, integral, derivative 
double kp[2], ki[2], kd[2];
unsigned long lastTime;
double output;
double proportionalTerm, integralTerm, derivativeTerm;
double lastInput, error;
double outMin, outMax;
const int PI_PIN_1 = 1;
const int PI_PIN_3 = 2;
const int PI_PIN_4 = 3;
const int PI_PIN_5 = 4;

// Minimum and maximum angles for each motor
const double maxAngle[6] = {90, 90, 90, 90, 90, 90};

// Modify these lines when actual values are available
const double minAngle[6] = {0 , 0 , 0 , 0 , 0 , 0};

// Photo interrupter values below this number would indicate motor movement. To be optimized with motors
const double PI_THRESHOLD = 60.0;

// Ratio of angles per tooth. Can be calculated by dividing 360 by the total number of teeth. If PI has different number of teeth, a multiple must be initialized
const double RATIO_ANGLES_PER_TOOTH = 1.8; 

// The following booleans indicate whether a tooth is passing through the PI
bool PI1move = false;
bool PI3move = false;
bool PI4move = false;
bool PI5move = false;

// is the motor rotating clock wise ? 
// necessary for updating encoderDistances array when a tooth finishes it's pass
bool isRotatingCW = false;

double PIval1, PIval3, PIval4, PIval5;
double encoderDistance[5] = {0, 0, 0, 0, 0};
// destination angles
double motorCommandAngle[6] = {0, 0, 0, 0, 0, 0};
// current angles
double motorAngle[6] = {0, 0, 0, 0, 0, 0};
int steps;
int i;

/////////////////////////////////////////////////////////////
//////////////////////////VOID SETUP//////////////////////////
/////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600); // Set up serial library at 9600 bps
    
  // Initializes the motor shields with the default frequency of 1.6KHz
  motorShield1.begin();
  motorShield2.begin(); 
  
  // Sets speeds for motors 
  //TODO: Optimize with some quick maths
  motor1->setSpeed(150);
  motor2->setSpeed(150);
  motor2->run(RELEASE);
  motor3->setSpeed(150);
  motor4->setSpeed(150);

  // Attach servo motors
  motor5.attach(10);
  motor6.attach(9);
}

/////////////////////////////////////////////////////////////
//////////////////////////VOID LOOP//////////////////////////
/////////////////////////////////////////////////////////////

void loop() {
  // Read photo interrupter values
  PIval1 = analogRead(PI_PIN_1);
  PIval3 = analogRead(PI_PIN_3);
  PIval4 = analogRead(PI_PIN_4);
  PIval5 = analogRead(PI_PIN_5);
  
  // PI status checks
  if (PIval1 < PI_THRESHOLD && !PI1move) { // in plain english, if (we detect that the motor is moving)
    PI1move = true; 
    Serial.println("Photo interrupter 1 has detected movement. A distance of 1.8 degrees is assumed to be traveled"); // 1.8 to be changed once PIs are installed
    if (motorCommandAngle[0] - motorAngle[0] < 0) {
      encoderDistance[0] -= RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = true;
    } else {
      encoderDistance[0] += RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = false;
    }
  } else if (PIval1 > PI_THRESHOLD && PI1move) {
    PI1move = false;
    encoderDistance[0] += (isRotatingCW) ? (-1)*RATIO_ANGLES_PER_TOOTH : RATIO_ANGLES_PER_TOOTH;
  }
  
  if (PIval3 < PI_THRESHOLD && !PI3move) {
    PI3move = true; 
    Serial.println("Photo interrupter 3 has detected movement. A distance of 1.8 degrees is assumed to be traveled"); // 1.8 to be changed once PIs are installed
    if (motorCommandAngle[2] - motorAngle[2] < 0) {
      encoderDistance[2] -= RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = true;
    } else {
      encoderDistance[2] += RATIO_ANGLES_PER_TOOTH; // counter clockwise = true
      isRotatingCW = false;
    }
  } else if(PIval3 > PI_THRESHOLD && PI3move) {
    PI3move = false;
    encoderDistance[2] += (isRotatingCW) ? (-1)*RATIO_ANGLES_PER_TOOTH : RATIO_ANGLES_PER_TOOTH;
  }
  
  if (PIval4 < PI_THRESHOLD && !PI4move) {
    PI4move = true; 
    Serial.println("Photo interrupter 4 has detected movement. A distance of 1.8 degrees is assumed to be traveled"); // 1.8 to be changed once PIs are installed
    if (motorCommandAngle[3] - motorAngle[3] < 0) {
      encoderDistance[3] -= RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = true;
    } else {
      encoderDistance[3] += RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = false;
    }
  } else if (PIval4 > PI_THRESHOLD && PI4move) {
    PI4move = false;
    encoderDistance[3] += (isRotatingCW) ? (-1)*RATIO_ANGLES_PER_TOOTH : RATIO_ANGLES_PER_TOOTH;
  }
  
  if (PIval5 < PI_THRESHOLD && !PI5move) {
    PI5move = true; 
    Serial.println("Photo interrupter 5 has detected movement. A distance of 1.8 degrees is assumed to be traveled"); // 1.8 to be changed once PIs are installed
    if (motorCommandAngle[4] - motorAngle[4] < 0) {
      encoderDistance[4] -= RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = true;
    } else {
      encoderDistance[4] += RATIO_ANGLES_PER_TOOTH;
      isRotatingCW = false;
    }
  } else if (PIval5 > PI_THRESHOLD && PI5move) {
    PI5move = false;
    encoderDistance[4] += (isRotatingCW) ? (-1)*RATIO_ANGLES_PER_TOOTH : RATIO_ANGLES_PER_TOOTH;
  }
  // PI status checks ends here

  // Check DC Encoder
  motorAngle[1] = DCEncoder.read();
  
  for (i = 0; i <= 5; i++) {
    // Verify motor position with encoder position
    if ((int)encoderDistance[i] != (int)motorAngle[i] && i != 5) {
      Serial.print("ERROR: Motor ");
      Serial.print(i);
      Serial.println(" has discrepancies in position sensing. Calibrating to use encoder data.");
      Serial.print("Encoder distance: ");
      Serial.println((int)encoderDistance[i]);
      Serial.print("Motor distance: ");
      Serial.println((int)motorAngle[i]);
      motorAngle[i] = encoderDistance[i];
    }
    // Verify stepper motor command is in range
    if (motorCommandAngle[i] > maxAngle[i] || motorCommandAngle[i] < minAngle[i]) {
      Serial.println("Error: Requested angle is not within bounds");
      Serial.print("Motor #");
      Serial.println(i+1);
      Serial.print(" angle: ");
      Serial.println(motorCommandAngle[i]);
    } else {
      // compute output and move motor 2
      compute(1);
      if (output < 0) {
        motor2->setSpeed(-output);
        motor2->run(BACKWARD);
      } else {
        motor2->setSpeed(output);
        motor2->run(FORWARD);
      }
      // compute output and move motor 5
      compute(4);
      motor5.write(output);
      
      // Move stepper motors
      error = abs(motorCommandAngle[i] - motorAngle[i]);
      steps = error/RATIO_ANGLES_PER_TOOTH; 

      switch (i) {
      case 0:
        motor1->step(steps, BACKWARD, SINGLE);
        motorAngle[0] += steps;
        break;
      case 2:
        motor3->step(steps, FORWARD, SINGLE);
        motorAngle[2] += steps;
        break;
      case 3:
        motor4->step(steps, FORWARD, SINGLE);
        motorAngle[3] += steps;
        break;
      case 5: // Move finger servo motor
        motor6.write(motorCommandAngle[i]);
        break;
      }
    }
  }

  //TODO: synchronize this with the frequency of serial reads being done from the GUI
  // consider the average time it takes for all the code above to execute
  delay(50); // Delay between each loop. Could optimize depending on requirements
}


/////////////////////////////////////////////////////////////
//////////////////////////FUNCTIONS//////////////////////////
/////////////////////////////////////////////////////////////

// compute outputs for given motor, return speed of said motor in rad/s
double compute(int i) {
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   error = motorCommandAngle[i] - motorAngle[i];
   
   /* proportional term */
   proportionalTerm = error*kp[i];
   
   if (proportionalTerm > outMax) {
    proportionalTerm = outMax;
   } else if (proportionalTerm < outMin) {
    proportionalTerm = outMin;
   }
   
   /* integral term */
   integralTerm += (ki[i] * error * timeChange);
   
   if (integralTerm > outMax) {
    integralTerm = outMax;
   } else if (integralTerm < outMin) {
    integralTerm = outMin;
   }

   /* derivative term */
   derivativeTerm = kd[i]* (motorAngle[i] - lastInput) / timeChange;
  
   if (derivativeTerm > outMax) {
    derivativeTerm = outMax;
   } else if (derivativeTerm < outMin) {
    derivativeTerm = outMin;
   }
   
   /*compute PID output*/
   output = proportionalTerm + integralTerm + derivativeTerm;
   
   if (output > outMax) {
    output = outMax;
   } else if (output < outMin) {
    output = outMin;
   }
  
   /*Remember some variables for next time*/
   lastInput = motorAngle[i];
   lastTime = now;
   
   return output;
}

// helper function to callibrate motors
void setTunings(int i, double Kp, double Ki, double Kd) {
   kp[i] = Kp;
   ki[i] = Ki;
   kd[i] = Kd;
}

// helper function to set min/max angles
void setOutputLimits(double Min, double Max) {
   if (Min > Max) {
    Serial.println("ERROR: Controller output makes no sense. Min cannot be greater than Max.");
    return;
   }
   
   outMin = Min;
   outMax = Max;
    
   if (output > outMax) {
    output = outMax;
   } else if (output < outMin) {
    output = outMin;
   }
 
   if (integralTerm > outMax) {
    integralTerm = outMax;
   } else if (integralTerm < outMin) {
    integralTerm = outMin;
   }
}

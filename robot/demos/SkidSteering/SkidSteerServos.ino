#include <ArduinoBlue.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo servoFrontRight;
Servo servoMiddleRight;
Servo servoBackRight;
Servo servoFrontLeft;
Servo servoMiddleLeft;
Servo servoBackLeft;


float prevThrottle = 49;
float prevSteering = 49;
float throttle, steering;
// Initialize Left & Right Velocity variables to Rest
float velocityRight = 90;
float velocityLeft = 90;
float deg = 0;
//This value is used to shift the contorollers range to have 0 as a middle rest value. shift value depends on controllers range.
int shiftValue = 49;

unsigned int prevRead = millis();

SoftwareSerial bluetooth(7, 8);

ArduinoBlue phone(bluetooth);

// This Function is used to Re-maps a number from one range to another. In this program, we use this function to give us a ratio between Left and Right velocities for a desired steering degree.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void velocityHandler(float throttle, float steering);

void setup() {
  // Attach Servos to pins:
  // Right side servos
  servoFrontRight.attach(9);
  servoMiddleRight.attach(10);
  servoBackRight.attach(11);

  //Left side Servos
  servoFrontLeft.attach(3);
  servoMiddleLeft.attach(5);
  servoBackLeft.attach(6);

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  bluetooth.begin(9600);

  Serial.println("setup complete");
}

void loop() {


  if (millis() - prevRead > 200) {
    // Lead Velocity Value from bluetooth controller. Values range from 0 to 99 for this specific controller
    throttle = phone.getThrottle();
    // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
    steering = phone.getSteering();
    // Function that updates velocity values based on steering angle.
    velocityHandler(throttle, steering);
    prevRead = millis();
  }

}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void velocityHandler(float throttle, float steering) {
  // New Rest Value is 0, this step isn't necessary. Its main purpose is to associate positive and negative values with backword and forwad velocity for throttle and left and right directions for steering
  throttle -= shiftValue;
  steering -= shiftValue;


  // If statement for CASE 1: steering toward the RIGHT
  if (steering <= 0 ) {
    deg = mapFloat(steering, -49, 0, -1, 1);
    velocityRight = mapFloat(throttle * deg, -49, 49, 0, 180);
    velocityLeft = mapFloat((-1) * throttle, -49, 49,  0, 180);
  }

  // If statement for CASE 2: steering toward the LEFT
  if (steering > 0 ) {
    deg = mapFloat(steering, 0, 49, 1, -1);
    velocityRight = mapFloat(throttle, -49, 49, 0, 180);
    velocityLeft = mapFloat((-1) * throttle * deg, -49, 49, 0, 180);
  }

  // Print Velocity Values
  if (prevThrottle != throttle || prevSteering != steering) {
    Serial.print("Throttle: "); Serial.print(throttle); Serial.print("\tSteering: "); Serial.println(steering);
    Serial.print(steering); Serial.print(">"); Serial.print(throttle); Serial.print("o");
                                         Serial.print("Right Side: "); Serial.print(velocityRight); Serial.print("\tLeft Side: "); Serial.println(velocityLeft);
                                         //Assign New Previous Values to help detect further changes
                                         prevThrottle = throttle;
                                         prevSteering = steering;
  }


  // Write velocities for the Wheels on the RIGHT side
  servoMiddleRight.write(velocityRight);
  servoBackRight.write(velocityRight);
  servoFrontRight.write(velocityRight);

  // Write velocities for the Wheels on the Left side
  servoFrontLeft.write(velocityLeft);
  servoMiddleLeft.write(velocityLeft);
  servoBackLeft.write(velocityLeft);
}

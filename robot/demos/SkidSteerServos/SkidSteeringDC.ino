#include <ArduinoBlue.h>
#include <SoftwareSerial.h>
#include <Servo.h>


int restVelocity = 127;
//This value is used to shift the contorollers range to have 0 as a middle rest value. shift value depends on controllers range, which is equal to the ranges middle value. 
int steeringShiftValue = 49;
int throttleShiftValue = 49;
float deg = 0;
// Initialize Left & Right Velocity variables to Rest
float velocityRight = restVelocity;
float velocityLeft = restVelocity;
int throttle, steering;

int prevThrottle = 0;
int prevSteering = 0; 

float minThrottleController = 0 - throttleShiftValue;
float maxThrottleController = 0 + throttleShiftValue;
float minSteeringController = 0 - steeringShiftValue;
float maxSteeringController = 0 + steeringShiftValue;
float minDC = 0;
float maxDC = 2*restVelocity+1;

SoftwareSerial bluetooth(7, 8);
ArduinoBlue phone(bluetooth);


// This Function is used to Re-maps a number from one range to another. In this program, we use this function to give us a ratio between Left and Right velocities for a desired steering degree.
 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void velocityHandler(throttle, steering) {
// New Rest Value is 0
  throttle -= shiftValue;
  steering -= shiftValue;
     

// If statement for CASE 1: steering toward the RIGHT 
  if (steering < 0 ) {
    deg = mapFloat(steering, minSteeringController, 0, -1, 1);
    velocityRight = map(throttle*deg, minThrottleController, maxThrottleController, minDC, maxDC);
    velocityLeft = map((-1)*throttle, minThrottleController, maxThrottleController,  minDC, maxDC);
  }

// If statement for CASE 2: steering toward the LEFT 
  if (steering >= 0 ) {
    deg = mapFloat(steering, 0, maxSteeringController, 1, -1);
    velocityRight = map(throttle, minThrottleController, maxThrottleController, minDC, maxDC);
    velocityLeft = map((-1)*throttle*deg, minThrottleController, maxThrottleController, minDC, maxDC);

} 
  
// Print Velocity Values   
  if (prevThrottle != throttle || prevSteering != steering) {
    Serial.print("Throttle: "); Serial.print(throttle); Serial.print("\tSteering: "); Serial.println(steering);
    Serial.print(steering);Serial.print(">"); Serial.print(throttle);Serial.print("o");
    Serial.print("Right Side: "); Serial.print(velocityRight); Serial.print("\tLeft Side: "); Serial.println(velocityLeft);
   
    prevThrottle = throttle;
    prevSteering = steering;
  }

 
// Write velocities for the Wheels on the RIGHT side
  analogWrite(DC_PIN_FRONT_RIGHT, velocityRight);
  analogWrite(DC_PIN_MIDDLE_RIGHT, velocityRight);
  analogWrite(DC_PIN_BACK_RIGHT, velocityRight);
  
// Write velocities for the Wheels on the Left side  
  analogWrite(DC_PIN_FRONT_LEFT, velocityLeft);
  analogWrite(DC_PIN_MIDDLE_LEFT, velocityLeft);
  analogWrite(DC_PIN_BACK_LEFT, velocityLeft);
  

}
void setup() {
  // Attach Servos to pins:
  // Right side servos
  DC_PIN_FRONT_RIGHT = 9;
  DC_PIN_MIDDLE_RIGHT = 10;
  DC_PIN_BACK_RIGHT = 11;
  
  // Left side DC motors
  DC_PIN_FRONT_LEFT = 3;
  DC_PIN_MIDDLE_LEFT = 5;
  DC_PIN_BACK_LEFT = 6;

  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  bluetooth.begin(9600);

  //activate pull-up resistor on the push-button pin
  pinMode(buttonPin, INPUT_PULLUP); 
  
  Serial.println("setup complete");
}

void loop() {
  
 // Lead Velocity Value from bluetooth controller. Values range from 0 to 99 for this specific controller
  throttle = phone.getThrottle();
 // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
  steering = phone.getSteering();
  // Function that updates velocity values based on steering angle.
  velocityHandler(throttle, steering);

 
delay(100); // add some delay between reads
}

#include <ArduinoBlue.h>
#include <SoftwareSerial.h>
#include <Servo.h>

int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering;
// Initialize Left & Right Velocity variables to Rest
int VR = 90;
int VL = 90;
SoftwareSerial bluetooth(7, 8);

ArduinoBlue phone(bluetooth);

void setup() {
  // Attach Servos to pins:
  // Right side servos
  DC_PIN_1 = 9;
  DC_PIN_2 = 10;
  DC_PIN_3 = 11;
  
  //Left side Servos
  DC_PIN_4 = 3;
  DC_PIN_5 = 5;
  DC_PIN_6 = 6;

  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  bluetooth.begin(9600);

  //activate pull-up resistor on the push-button pin
  pinMode(buttonPin, INPUT_PULLUP); 
  
  Serial.println("setup complete");
}

float deg = 0;


// New map function that supports float variables.
 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  
 // Lead Velocity Value from bluetooth controller. Values range from 0 to 90
  throttle = phone.getThrottle();
 // Steering Value from bluetooth controller. Values range from 0 to 90
  steering = phone.getSteering();
 
 // New Rest Value is 0
  throttle -= 49;
  steering -= 49;
     

// If statement for CASE 1: steering toward the RIGHT 
  if (steering <= 0 ) {
    deg = mapFloat(steering, -49, 0, -1, 1);
    VR = map(throttle*deg, -49, 49,0,180);
    VL = map((-1)*throttle, -49, 49,  0,180);
  }

// If statement for CASE 1: steering toward the LEFT 
  if (steering >= 0 ) {
    deg = mapFloat(steering, 0, 49, 1, -1);
    VR = map(throttle, -49, 49, 0,180);
    VL = map((-1)*throttle*deg, -49, 49,  0,180);

} 
  
// Print Velocity Values   
  if (prevThrottle != throttle || prevSteering != steering) {
    Serial.print("Throttle: "); Serial.print(throttle); Serial.print("\tSteering: "); Serial.println(steering);
    Serial.print(steering);Serial.print('>'); Serial.print(throttle);Serial.print('o');
    Serial.print("Right Side: "); Serial.print(vr); Serial.print("\tLeft Side: "); Serial.println(vl);
   
    prevThrottle = throttle;
    prevSteering = steering;
  }


// Write velocities for the Wheels on the RIGHT side
  analogWrite(DC_PIN_1, VR);
  analogWrite(DC_PIN_2, VR);
  analogWrite(DC_PIN_3, VR);
  
// Write velocities for the Wheels on the Left side  
  analogWrite(DC_PIN_4, VL);
  analogWrite(DC_PIN_5, VL);
  analogWrite(DC_PIN_6, VL);
  
delay(100); // add some delay between reads

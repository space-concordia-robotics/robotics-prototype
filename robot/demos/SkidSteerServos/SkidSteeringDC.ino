#include <ArduinoBlue.h>

#include <SoftwareSerial.h>

#include <Servo.h>

Servo myservo;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;


int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering;
// Initialize Left & Right Velocity variables to Rest
int vr = 90;
int vl = 90;
SoftwareSerial bluetooth(7, 8);

ArduinoBlue phone(bluetooth);

void setup() {
  // Attach Servos to pins:
  // Right side servos
  myservo.attach(9);  
  myservo1.attach(10);  
  myservo2.attach(11);
  
  //Left side Servos
  myservo3.attach(3);
  myservo4.attach(5);
  myservo5.attach(6);


  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  bluetooth.begin(9600);

  //activate pull-up resistor on the push-button pin
  pinMode(buttonPin, INPUT_PULLUP); 
  
  Serial.println("setup complete");
}

float right = 90;
float left = 90;
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
     
  delay(200); // add some delay between reads

// If statement for CASE 1: steering toward the RIGHT 
  if (steering <= 0 ) {
    deg = mapFloat(steering, -49, 0, -1, 1);
    vr = map(throttle*deg, -49, 49,0,180);
    vl = map((-1)*throttle, -49, 49,  0,180);
  }

// If statement for CASE 1: steering toward the LEFT 
  if (steering >= 0 ) {
    deg = mapFloat(steering, 0, 49, 1, -1);
    vr = map(throttle, -49, 49, 0,180);
    vl = map((-1)*throttle*deg, -49, 49,  0,180);

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
  myservo.write(vr);              
  myservo1.write(vr);              
  myservo2.write(vr);
  
// Write velocities for the Wheels on the Left side  
  myservo3.write(vl);
  myservo4.write(vl);
  myservo5.write(vl);

  delay(100); // add some delay between reads

#include <ArduinoBlue.h>

#include <SoftwareSerial.h>

#include <Servo.h>

Servo myservo;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

int xPin = A1;
int yPin = A0;
int buttonPin = 2;

int xPosition = 0;
int yPosition = 0;
int buttonState = 0;

int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering;
int vr = 90;
int vl = 90;
SoftwareSerial bluetooth(7, 8);

ArduinoBlue phone(bluetooth);

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo1.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(11);
  myservo3.attach(3);
  myservo4.attach(5);
  myservo5.attach(6);


  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 


   bluetooth.begin(9600);


  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  //activate pull-up resistor on the push-button pin
  pinMode(buttonPin, INPUT_PULLUP); 
  
  // For versions prior to Arduino 1.0.1
  // pinMode(buttonPin, INPUT);
  // digitalWrite(buttonPin, HIGH);
  Serial.println("setup complete");

}

float right = 90;
float left = 90;


float deg = 0;
//int r = 0;
int v = 0;
//int w = 0;
//int width : 103;
char val;
//
//float Minr = 40/(tan(-89.9999);
//float Maxr = 40/(tan(89.9999);
//float Minvr = -48;
//float Maxvr;
//float Minvl ;
//float Maxvl ;

 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void loop() {
  
  throttle = phone.getThrottle();
  steering = phone.getSteering();
  throttle -= 49;
  steering -= 49;
     
  delay(200); // add some delay between reads



      if (steering <= 0 ) {
    v = map(throttle, 0, 99, 80,100);
     deg = mapFloat(steering, -49, 0, -1, 1);


vr = map(throttle*deg, -49, 49,0,180);
    vl = map((-1)*throttle, -49, 49,  0,180);

  }
      
    if (steering >= 0 ) {
    v = map(throttle, 49, 99, 0, 48);
     deg = mapFloat(steering, 0, 49, 1, -1);

vr = map(throttle, -49, 49, 0,180);
    vl = map((-1)*throttle*deg, -49, 49,  0,180);

}   
     if (prevThrottle != throttle || prevSteering != steering) {
    Serial.print("Throttle: "); Serial.print(throttle); Serial.print("\tSteering: "); Serial.println(steering);
    Serial.print(steering);Serial.print('>'); Serial.print(throttle);Serial.print('o');
    Serial.print("Right: "); Serial.print(vr); Serial.print("\tLeft: "); Serial.println(vl);
   
    prevThrottle = throttle;
    prevSteering = steering;
//        Serial.print("deg: "); Serial.print(deg);

  }
//  a = 
//  xPosition = analogRead(xPin);
//  yPosition = analogRead(yPin);
//  buttonState = digitalRead(buttonPin);
    
//  Serial.print("X: ");
//  Serial.print(xPosition);
//  Serial.print(" | Y: ");
//  Serial.print(yPosition);
//  Serial.print(" | Button: ");
//  Serial.println(buttonState);
//

 myservo.write(vr);              // tell servo to go to position in variable 'pos'
 myservo1.write(vr);              // tell servo to go to position in variable 'pos'
  myservo2.write(vr);
  
  myservo3.write(vl);
  myservo4.write(vl);
  myservo5.write(vl);

  delay(100); // add some delay between reads
  
}

//
//
//char val; // variable to receive data from the serial port
//int ledpin = 8; // LED connected to pin 48 (on-board LED)
//
//void setup() {
//
////  pinMode(ledpin, OUTPUT);  // pin 48 (on-board LED) as OUTPUT
//  Serial.begin(9600);       // start serial communication at 9600bps
//}
//
//void loop() {
//
//  if( Serial.available() )       // if data is available to read
//  {
//    val = Serial.read();         // read it and store it in 'val'
//    Serial.print("X: ");
//
//  }
////  if( val == 'H' )               // if 'H' was received
////  {
////    digitalWrite(ledpin, HIGH);  // turn ON the LED
////  } else { 
////    digitalWrite(ledpin, LOW);   // otherwise turn it OFF
////  }
//  delay(100);                    // wait 100ms for next reading
//}

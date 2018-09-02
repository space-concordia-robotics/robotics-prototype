#include <Servo.h> 
 
Servo myServo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int SERVO_PIN_9 = 9;
int REST = 89;
int OFFSET = 10;
int CW = REST - OFFSET;
int CCW = REST + OFFSET;
int BUDGE_TIME = 99;
int BAUD_RATE = 9600;
unsigned long startTime = 0;
unsigned long currentTime = 0;

void setup() 
{ 
    myServo.attach(SERVO_PIN_9);  // attaches the servo on pin 9 to the servo object 
    myServo.write(REST); // this actually stops the motor
    Serial.begin(BAUD_RATE);
} 
 
 
void loop() 
{ 

    if (Serial.available()) {
        char cmd = Serial.read();
        Serial.println("cmd: ");
        Serial.println(cmd);

        if (cmd == 'w') {
            Serial.println("Moving CW");
            myServo.write(CW); // rotate clockwise
            delay(BUDGE_TIME);
            Serial.println("Stop");
            myServo.write(REST);
        }
        else if (cmd == 's') {
            Serial.println("Moving CCW");
            myServo.write(CCW); // rotate counter clockwise
            delay(BUDGE_TIME);
            Serial.println("Stop");
            myServo.write(REST);
        }
    }
}

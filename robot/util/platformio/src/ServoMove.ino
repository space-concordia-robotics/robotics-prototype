#include <Servo.h> 
 
Servo leftRearServo, rightRearServo;  // create servo object to control a servo 
                                      // a maximum of eight servo objects can be created 
 
int SERVO_PIN_7 = 7;
int SERVO_PIN_9 = 9;
int REST = 90;
int OFFSET = 10;
int CW = REST - OFFSET;
int CCW = REST + OFFSET;
int BUDGE_TIME = 100;
int BAUD_RATE = 9600;
unsigned long startTime = 0;
unsigned long currentTime = 0;

void setup() 
{ 
    rightRearServo.attach(SERVO_PIN_7);  // attaches the servo on pin 7 to the servo object 
    rightRearServo.write(REST); // this actually stops the motor
    leftRearServo.attach(SERVO_PIN_9); 
    leftRearServo.write(REST);
    Serial.begin(BAUD_RATE);
} 
 
 
void loop() 
{ 

    if (Serial.available()) {
        char cmd = Serial.read();
        Serial.println("cmd: ");
        Serial.println(cmd);

        if (cmd == 'w') {
            Serial.println("Moving Forward");
            leftRearServo.write(CCW); // rotate clockwise
            rightRearServo.write(CW); // rotate counter clockwise
            delay(BUDGE_TIME);
            // stop
            leftRearServo.write(REST);
            rightRearServo.write(REST);
            Serial.println("Stop");
        }
        else if (cmd == 's') {
            Serial.println("Moving Back");
            leftRearServo.write(CW); // rotate counter clockwise
            rightRearServo.write(CCW); // rotate clockwise
            delay(BUDGE_TIME);
            // stop
            leftRearServo.write(REST);
            rightRearServo.write(REST);
            Serial.println("Stop");
        }
    }
}

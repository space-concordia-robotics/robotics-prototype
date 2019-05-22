
#include "ArduinoBlue.h"
#include "DcMotor.h"
#include <SoftwareSerial.h>

//GPS & IMU Includes
#include <SparkFun_I2C_GPS_Arduino_Library.h>
I2CGPS myI2CGPS;  // I2C object
#include "TinyGPS++.h"
TinyGPSPlus gps;   // GPS object
#include <Wire.h>
#include <LSM303.h>
LSM303 compass;
//#define TEST 1


#define PULSES_PER_REV      14
#define maxRpm              30
#define GEAR_RATIO         188.61
/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
#define DEVEL_MODE_1 1
//#define DEVEL_MODE_2 2

#if defined(DEVEL_MODE_1)
// serial communication over uart with odroid, teensy plugged into pcb and odroid
#define UART_PORT Serial
#define PRINT(a) Serial.print(a); delay(70); bluetooth.print(a); delay(70);
#define PRINTln(a) Serial.println(a); delay(70); bluetooth.println(a); delay(70);
#define PRINTRES(a,b) Serial.print(a, b); delay(70); bluetooth.print(a, b); delay(70);
#elif defined(DEVEL_MODE_2)
#define UART_PORT Serial4
#define PRINT(a) Serial4.print(a); delay(70); bluetooth.print(a); delay(70);
#define PRINTln(a) Serial4.println(a); delay(70); bluetooth.println(a); delay(70);
#define PRINTRES(a,b) Serial4.print(a, b); delay(70); bluetooth.print(a, b); delay(70);
#endif

#if defined(DEBUG_MODE) || defined(USER_MODE)
#define USE_TEENSY_HW_SERIAL 0 // this will make ArduinoHardware.h use hardware serial instead of usb serial
#endif



SoftwareSerial bluetooth(9, 10);

ArduinoBlue phone(bluetooth);




// F -> Front, B -> Back,  M -> Middle, L -> Left, R -> Right,
// DIR -> Direction pin, PWM -> Signal Pin, EA ->Encoder A, EB -> Encoder B
// DC Motor naming: Positing_purpose-pin, eg RF_PWM or LM_DIR
// Types of Pins: DIR, PWM, Enc_A, Enc_B


// Bluetooth connection TX of bluetooth goes to pin 9 and RX of bluetooth goes to pin 10
#define RF_DIR   2
#define RM_DIR   11
#define RB_DIR   12

#define LF_DIR   24
#define LM_DIR   25
#define LB_DIR   26

#define RF_PWM   3
#define RM_PWM   4
#define RB_PWM   5

#define LF_PWM   6
#define LM_PWM   7
#define LB_PWM   8

#define RF_EA    37
#define RF_EB    38

#define RM_EA    35
#define RM_EB    36

#define RB_EA    33
#define RB_EB    34

#define LF_EA    31
#define LF_EB    32

#define LM_EA    29
#define LM_EB    30

#define LB_EA    27
#define LB_EB    28


DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO, "Front Right Motor");
DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO, "Middle Right Motor");
DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO, "Rear Right Motor");

DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO, "Front Left Motor");
DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO, "Middle Left Motor");
DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO, "Rear Left Motor");


//Commands cmd;

DcMotor motorList[] = {RF, RM, RB, LF, LM, LB};
int throttle, steering, loop_state, button, i; // Input values for set velocity functions
float deg = 0;  // steering ratio between left and right wheel
int maxInputSignal = 49;  // maximum speed signal from controller
int minInputSignal = -49; // minimum speed signal from controller
int maxOutputSignal = 255;
int minOutputSignal = -255;
int motorNumber;
int leftMotorDirection; // CCW =1 or CW =-1
int rightMotorDirection; // CCW =1 or CW =-1
float desiredVelocityRight = 0;
float desiredVelocityLeft = 0;
unsigned int prevRead = millis(); // Loop Timer
unsigned int prevReadNav = millis(); // Loop Timer for nav
float d = 0.33; //distance between left and right wheels
float radius = 14;
float forwardVelocity, rotationalVelocity, rightLinearVelocity, leftLinearVelocity;

String rotation; // Rotation direction of the whole rover

// Modes of operation
// Bluetooth control - basestation control - PID - Open Loop

//boolean isActivated = false;
boolean isActivated = false;
boolean isOpenloop = true; // No PID controller
boolean bluetoothMode = true;
boolean joystickMode = true;
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void motor_encoder_interrupt(int motorNumber);
void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);
void ser_flush(void);
void toggleLed();
void toggleLed2();
String getValue(String data, char separator, int index);
void initPins(void); // Initiate pinMode for direction and pwm pins for cytron drivers
void initEncoders(void);    // Encoder initiation (attach interrups and pinModes for wheel encoders
void initPids(void);    // Initiate PID for DMotor
void velocityHandler(float throttle, float steering);
void initNav(void);
void displayGpsInfo(void);
void navHandler(void);
void roverVelocityCalculator(void);

#include "commands.h"

Commands Commands;

void setup() {
    // initialize serial communications at 115200 bps:
    UART_PORT.begin(115200); // switched from 9600 as suggested to conform with the given gps library
    UART_PORT.setTimeout(50);
    bluetooth.begin(9600);
    bluetooth.setTimeout(50);

    delay(50);  // do not print too fast!

    ser_flush();
    initPins();
    initEncoders();
    initPids();
    initNav();

    {
//    analogWrite(LB_PWM, 60);
//    analogWrite(RB_PWM, 60);
//    analogWrite(LM_PWM, 60);
//    analogWrite(RM_PWM, 60);
//    analogWrite(RF_PWM, 60);
//    analogWrite(LF_PWM, 60);
    }

    Commands.setupMessage();
}

void loop() {
    if (UART_PORT.available()) {
        String cmd = UART_PORT.readStringUntil('\n');
        ser_flush();
        PRINT("cmd: ");
        PRINTln(cmd);
        Commands.handler(cmd);
//        if (cmd == "activate") {
//            //toggleLed();
//            isActivated = true;
//            bluetoothMode = false;
//            toggleLed2();
//
//            PRINTln("Rover Wheels are Active");
//            PRINTln("BLE-Mode is OFF");
//
//        } else if (cmd == "who") {
//            PRINTln("rover");
//        }
//        else {
//            PRINTln("Please Activate Rover Wheels");

    }

    else if (bluetooth.available() && !isActivated && joystickMode) {
        button = phone.getButton();
        if (button == 0) {
            isActivated = true;
            PRINTln("Rover Wheels are Active");
            toggleLed2();

        }
        else if (button == 3){
            joystickMode = false;
            PRINTln("Bluetooth Serial mode is on");
        }
        else {
            String cmd = bluetooth.readStringUntil('\n');
            if (cmd) {
                PRINTln("\nERROR");
                PRINTln(" - Please Activate Rover Wheels ");
                delay(70);
                PRINT(" - or if you're using the Serial App ");
                delay(70);
                PRINTln("then activate Ble-Serial from ArduinoBlue \n");
            }
        }
    }
    else if (bluetooth.available() && !isActivated && !joystickMode) {
//        String cmd = bluetooth.readStringUntil('\n');
////        ser_flush();
//        PRINT("cmd: ");
//        PRINTln(cmd);
//        if (cmd == "activate") {
//            //toggleLed();
//            isActivated = true;
////            bluetoothMode = false;
//            PRINTln("Rover Wheels are Active");
//            toggleLed2();
//
//        } else if (cmd == "who") {
//            PRINTln("rover");
//        }
//        else {
//            PRINTln("Please Activate Rover Wheels");
//        }
    }


    if ((millis() - prevRead > 30)  && isActivated ) {
        // incoming format example: "5:7"
        // this represents the speed for throttle:steering
        // as well as direction by the positive/negative sign
        String cmd = "";

        // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
        if (UART_PORT.available()) {
            toggleLed();
            cmd = UART_PORT.readStringUntil('\n');
            Commands.handler(cmd);
            ser_flush();
//            if (cmd == "who") {
//                PRINTln("happy rover");
//            }
//            else if (cmd == "deactivate") {
//                //toggleLed();
//                isActivated = false;
//                PRINTln("Rover Wheels are Deactivated");
//            } else if (cmd == "CloseLoop") {   // Close loop activation command
//                minOutputSignal = -30;
//                maxOutputSignal = 30;
//                for (i = 1; i <= 6; i++) {
//                    motorList[i].isOpenLoop = false;
//                    PRINT("Motor ");
//                    PRINT(i);
//                    PRINT(" open loop status is: ");
//                    PRINTln(motorList[i].isOpenLoop);
//                    delay(80);
//
//                }
//            } else if (cmd == "OpenLoop") {
//                minOutputSignal = -255;
//                maxOutputSignal = 255;
//                for (i = 1; i <= 6; i++) {
//                    motorList[i].isOpenLoop = true;
//                    PRINT("Motor ");
//                    PRINT(i);
//                    PRINT(" open loop status is: ");
//                    PRINTln(motorList[i].isOpenLoop);
//                    delay(80);
//
//                }
//            } else if (cmd == "ble-on") {
//                minInputSignal = -49;
//                maxInputSignal = 49;
//                bluetoothMode = true;
//                PRINTln("Bluetooth Control is active");
//
//            } else if (cmd == "ble-off") {
//                minInputSignal = -49;
//                maxInputSignal = 49;
//                bluetoothMode = false;
//                PRINTln("Bluetooth Control is off");
//
//            }
//            else if ((cmd.indexOf(":") > 0) && !bluetoothMode) {
//                PRINTln("Received command");
//                throttle = getValue(cmd, ':', 0).toInt();
//                steering = getValue(cmd, ':', 1).toInt();
//                PRINT("TEENSY throttle: ");
//                PRINTln(throttle);
//                PRINT("TEENSY steering: ");
//                PRINTln(steering);
//            }
//            else if ((cmd.indexOf(":") > 0)){
//                motorNumber = getValue(cmd, ':', 0).toInt();
//                int speed = getValue(cmd, ':', 1).toInt();
//                steering = 0;
//                PRINTln(motorList[motorNumber].isOpenLoop);
//                PRINT(motorList[motorNumber].motorName);
//                PRINT("'s throttle speed: ");
//                delay(80);
//                int dir = 1;
//
//                PRINTln(speed);
//                if (throttle < 0 ){
//                    dir = - 1;
//                }
//                motorList[motorNumber].setVelocity(dir , abs(speed), motorList[motorNumber].getCurrentVelocity());
//                PRINT(motorList[motorNumber].motorName);
//                PRINT("'s desired speed: ");
//                delay(80);
//                PRINT(motorList[motorNumber].getDesiredVelocity());
//                PRINTln(" PWM");
//
//                delay(500);
//
//                PRINT("Encoder Count is ");
//                PRINTln(motorList[motorNumber].encoderCount)
//            }
//            else if (cmd == "enc") {
//
//                for (i = 1; i <= 6; i++) {
//                    motorList[i].isOpenLoop = true;
//                    PRINT("Motor ");
//                    PRINT(i);
//                    PRINT(" encoder count: ");
//                    PRINTln(motorList[i].encoderCount);
//                    delay(80);
//
//                }
//            }
//            else if (cmd == "stop"){
//                for (i = 1; i <= 6; i++) {
//                    motorList[i].setVelocity(1 , 0, motorList[i].getCurrentVelocity());
//
//
//                    PRINT("Motor ");
//                    PRINT(i);
//                    PRINT(" Speed is: ");
//                    PRINTln(motorList[i].getDesiredVelocity());
//                    delay(80);
//                }
//            }
        }
        else if (bluetoothMode) {
            cmd = Serial2.readStringUntil('\n');
            ser_flush();
            button = phone.getButton();
            if (button == 1) {
                isActivated = false;
                PRINT("Rover Wheels are disabled");
            }
            if (button == 2) {
                joystickMode = true;
                PRINTln("Joystick is active, don't use serial Bluetooth");
            }
            else if (button ==3){
                joystickMode = false;
                PRINTln("Joystick is disabled, use serial Bluetooth ");
            }

        }

        if (bluetoothMode && !joystickMode && isActivated) {
            if (cmd == "who") {
                PRINTln("happy rover");
            }
            else if (cmd == "deactivate") {
                //toggleLed();
                isActivated = false;
                PRINTln("Rover Wheels are Deactivated");
                toggleLed2();


            } else if (cmd == "CloseLoop") {   // Close loop activation command
                minOutputSignal = -30;
                maxOutputSignal = 30;
                for (i = 1; i <= 6; i++) {
                    motorList[i].isOpenLoop = false;
                    PRINT("Motor ");
                    PRINT(i);
                    PRINT(" open loop status is: ");
                    PRINTln(motorList[i].isOpenLoop);
                    delay(80);
                }
            } else if (cmd == "OpenLoop") {
                minOutputSignal = -255;
                maxOutputSignal = 255;
                for (i = 1; i <= 6; i++) {
                    motorList[i].isOpenLoop = true;
                    PRINT("Motor ");
                    PRINT(i);
                    PRINT(" open loop status is: ");
                    PRINTln(motorList[i].isOpenLoop);
                    delay(80);

                }
            }
            else if (cmd == "enc") {

                for (i = 1; i <= 6; i++) {
                    motorList[i].isOpenLoop = true;
                    PRINT("Motor ");
                    PRINT(i);
                    PRINT(" encoder count: ");
                    PRINTln(motorList[i].encoderCount);
                    delay(80);

                }
            }
            else if ((cmd.indexOf(":") > 0) && !joystickMode){
                motorNumber = getValue(cmd, ':', 0).toInt();
                int speed = getValue(cmd, ':', 1).toInt();
                steering = 0;
                PRINTln(motorList[motorNumber].isOpenLoop);
                PRINT(motorList[motorNumber].motorName);
                PRINT("'s throttle speed: ");
                delay(80);
                int dir = 1;

                PRINTln(speed);
                if (throttle < 0 ){
                    dir = - 1;
                }
                motorList[motorNumber].setVelocity(dir , abs(speed), motorList[motorNumber].getCurrentVelocity());
                PRINT(motorList[motorNumber].motorName);
                PRINT("'s desired speed: ");
                delay(80);
                PRINT(motorList[motorNumber].getDesiredVelocity());
                PRINTln(" PWM");

                delay(500);

                PRINT("Encoder Count is");
                PRINTln(motorList[motorNumber].encoderCount)
            }
            else if (cmd == "stop"){
                for (i = 1; i <= 6; i++) {
                    motorList[i].setVelocity(1 , 0, motorList[i].getCurrentVelocity());


                    PRINT("Motor ");
                    PRINT(i);
                    PRINT(" Speed is: ");
                    PRINTln(motorList[i].getDesiredVelocity());
                    delay(80);
                }
            }
        }

        else if (bluetoothMode && joystickMode && isActivated) {
            throttle = phone.getThrottle();
            steering = phone.getSteering();
            throttle -= 49.5;
            steering -= 49.5;
            PRINT("TEENSY throttle: ");
            PRINTln(throttle);
            PRINT("TEENSY steering: ");
            PRINTln(steering);
        }

        else if (!bluetoothMode){
            //PRINTln("No command received");
//            throttle = 0;
//            steering = 0;
        }

//        velocityHandler(throttle, steering);
//        roverVelocityCalculator();


        //ser_flush();
        prevRead = millis();

    }









    if (millis() - prevReadNav > 500) {
        navHandler();
        prevReadNav = millis();
    }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void velocityHandler(float throttle, float steering) {
    // If statement for CASE 1: steering toward the RIGHT
    if (steering > 0 ) {
        deg = mapFloat(steering, 0, maxInputSignal, 1, -1);
        desiredVelocityRight = map(throttle * deg, minInputSignal, maxInputSignal, minOutputSignal, maxOutputSignal);
        desiredVelocityLeft = map(throttle, minInputSignal, maxInputSignal,  minOutputSignal, maxOutputSignal);
        rotation = "CW";
    }
    // If statement for CASE 2: steering toward the LEFT or not steering
    if (steering <= 0 ) {
        deg = mapFloat(steering, minInputSignal, 0, -1, 1);
        desiredVelocityRight = map(throttle, minInputSignal, maxInputSignal, minOutputSignal, maxOutputSignal);
        desiredVelocityLeft = map(throttle * deg, minInputSignal, maxInputSignal, minOutputSignal, maxOutputSignal);
        rotation = "CCW";
    }
    if (desiredVelocityLeft < 0 ) {
        leftMotorDirection = 1;
    }
    else {
        leftMotorDirection = -1;
    }

    if (desiredVelocityRight < 0 ) {
        rightMotorDirection = -1;
    }
    else {
        rightMotorDirection = 1;
    }

    RF.calcCurrentVelocity();
    RM.calcCurrentVelocity();
    RB.calcCurrentVelocity();
    LF.calcCurrentVelocity();
    LM.calcCurrentVelocity();
    LB.calcCurrentVelocity();

    RF.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RF.getCurrentVelocity());
    RM.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RM.getCurrentVelocity());
    RB.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RB.getCurrentVelocity());
    LF.setVelocity(rightMotorDirection, abs(desiredVelocityRight), LF.getCurrentVelocity());
    LM.setVelocity(rightMotorDirection, abs(desiredVelocityRight), LM.getCurrentVelocity());
    LB.setVelocity(rightMotorDirection, abs(desiredVelocityRight), LB.getCurrentVelocity());


//    PRINT("rightMotorDirection: ");
//    PRINTln(abs(rightMotorDirection));
//    PRINT("desiredVelocityRight: ");
//    PRINTln(abs(desiredVelocityRight));
//
//    PRINT("leftMotorDirection: ");
//    PRINTln(abs(leftMotorDirection));
//    PRINT("desiredVelocityLeft: ");
//    PRINTln(abs(desiredVelocityLeft));

}

void ser_flush(void) {
    while (UART_PORT.available()) {
        UART_PORT.read();
    }
}

void toggleLed() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void toggleLed2() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(350);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}        // Parse data for throttle and steering variables

void initPins(void) {
    pinMode(RF_DIR, OUTPUT);
    pinMode(RF_PWM, OUTPUT);
    pinMode(RM_DIR, OUTPUT);
    pinMode(RM_PWM, OUTPUT);
    pinMode(RB_DIR, OUTPUT);
    pinMode(RB_PWM, OUTPUT);

    pinMode(LF_DIR, OUTPUT);
    pinMode(LF_PWM, OUTPUT);
    pinMode(LM_DIR, OUTPUT);
    pinMode(LM_PWM, OUTPUT);
    pinMode(LB_DIR, OUTPUT);
    pinMode(LB_PWM, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);

}                                           // Initiate pinModes

void initEncoders(void) {
    RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
    pinMode(RF.encoderPinB, INPUT_PULLUP);
    pinMode(RF.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, CHANGE);
    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

    RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
    pinMode(RM.encoderPinB, INPUT_PULLUP);
    pinMode(RM.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, CHANGE);
    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

    RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
    pinMode(RB.encoderPinB, INPUT_PULLUP);
    pinMode(RB.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, CHANGE);
    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

    LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
    pinMode(LF.encoderPinB, INPUT_PULLUP);
    pinMode(LF.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, CHANGE);
    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

    LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
    pinMode(LM.encoderPinB, INPUT_PULLUP);
    pinMode(LM.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, CHANGE);
    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

    LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
    pinMode(LB.encoderPinB, INPUT_PULLUP);
    pinMode(LB.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, CHANGE);
    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);
}                                       // Initiate encoder for dcMotor objects and pinModes

void initPids(void) {
    //    RF.pidController.setJointVelocityTolerance(2.0 * RF.gearRatioReciprocal);
    //    RM.pidController.setJointVelocityTolerance(2.0 * RM.gearRatioReciprocal);
    //    RB.pidController.setJointVelocityTolerance(2.0 * RB.gearRatioReciprocal);
    //
    //    LF.pidController.setJointVelocityTolerance(2.0 * LF.gearRatioReciprocal);
    //    LM.pidController.setJointVelocityTolerance(2.0 * LM.gearRatioReciprocal);
    //    LB.pidController.setJointVelocityTolerance(2.0 * LB.gearRatioReciprocal);
    //
    //    RF.pidController.setOutputLimits(-50, 50, 5.0);
    //    RM.pidController.setOutputLimits(-50, 50, 5.0);
    //    RB.pidController.setOutputLimits(-50, 50, 5.0);
    //
    //    LF.pidController.setOutputLimits(-50, 50, 5.0);
    //    LM.pidController.setOutputLimits(-50, 50, 5.0);
    //    LB.pidController.setOutputLimits(-50, 50, 5.0);
}                                           // Initiate PID objects for Dc Motors
void motor_encoder_interrupt(int motorNumber) {         // This interrupt is responsible for updating dt, prevtime and encoderCount variable for a DcMotor object which is used to calculate the velocity of the motor
    motorList[motorNumber].dt += micros() -  motorList[motorNumber].prevTime;
    motorList[motorNumber].prevTime = micros();
    motorList[motorNumber].encoderCount++;
}

void roverVelocityCalculator(void) {
    rightLinearVelocity = (RF.getDirection() * RF.getCurrentVelocity() + RM.getDirection() * RM.getCurrentVelocity() + RB.getDirection() * RB.getCurrentVelocity()) * radius * 0.10472;
    leftLinearVelocity = (LF.getDirection() * LF.getCurrentVelocity() + LM.getDirection() * LM.getCurrentVelocity() + LB.getDirection() * LB.getCurrentVelocity()) * radius * 0.10472;

    forwardVelocity = (rightLinearVelocity + leftLinearVelocity)  / 6;
    rotationalVelocity = (leftLinearVelocity - rightLinearVelocity) / d;


}
void initNav(void) {
    if (myI2CGPS.begin(Wire , 400000) == false)        // Wire1 corresponds to the SDA1,SCL1 on the Teensy 3.6 (pins 38,37)
    {
//        while (1);
// This will freeze the code to have the user check wiring
        Commands.error = true;
        Commands.errorMessage = "GPS wires aren't connected properly";
    }
    compass.init();
    compass.enableDefault();
    compass.m_min = (LSM303::vector<int16_t>) {
            -9506, -6666, -8003
    };
    compass.m_max = (LSM303::vector<int16_t>) {
            +5933, +7826, +6528
    };
}
void displayGpsInfo(void) {                     // The function that prints the info
    if (gps.location.isValid() && Commands.isGpsImu)      // checks if valid location data is available
    {
        PRINT("GPS-OK");          // string initials to allow the Pyhton code to pickup
        PRINT(" ");            // space
        PRINTRES(gps.location.lat(), 6);   // print the latitude with 6 digits after the point
        PRINT(" ");           // space
        PRINTRES(gps.location.lng(), 6);   // print the longitude with 6 digits after the point
        PRINT("--");             // new line
    }
    else if (Commands.isGpsImu)
    {
        PRINT(F("GPS-N/A"));
        PRINT("--");
    }
}
void navHandler(void) {

    compass.read();
    float heading = compass.heading(LSM303::vector<int>{-1, 0, 0});

    if (myI2CGPS.available())          // returns the number of available bytes from the GPS module
    {
        gps.encode(myI2CGPS.read());       // Feeds the GPS parser
    }

    if (gps.time.isUpdated())         // Checks to see if new GPS info is available
    {
        displayGpsInfo();                  // Print the info on the serial monitor
    }
    if (Commands.isGpsImu) {
        PRINT("Heading");
        PRINT(" ");
        PRINT(heading);
        PRINT("\n");
    }
}
void rf_encoder_interrupt(void) {
    RF.dt += micros() - RF.prevTime;
    RF.prevTime = micros();
    RF.encoderCount++;
}

void rm_encoder_interrupt(void) {
    RM.dt += micros() - RM.prevTime;
    RM.prevTime = micros();
    RM.encoderCount++;
}
void rb_encoder_interrupt(void) {
    RB.dt += micros() - RB.prevTime;
    RB.prevTime = micros();
    RB.encoderCount++;
}
void lf_encoder_interrupt(void) {
    LF.dt += micros() - LF.prevTime;
    LF.prevTime = micros();
    LF.encoderCount++;
}
void lm_encoder_interrupt(void) {
    LM.dt += micros() - LM.prevTime;
    LM.prevTime = micros();
    LM.encoderCount++;
}
void lb_encoder_interrupt(void) {
    LB.dt += micros() - LB.prevTime;
    LB.prevTime = micros();
    LB.encoderCount++;
}

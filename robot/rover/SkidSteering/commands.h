//
// Created by Fauzi Ameen on 2019-05-21.
//

#ifndef SKIDSTEERING_COMMANDS_H
#define SKIDSTEERING_COMMANDS_H

#include "DcMotor.h"
//
//#define DEVEL_MODE_1 1
////#define DEVEL_MODE_2 2
//
//#if defined(DEVEL_MODE_1)
//// serial communication over uart with odroid, teensy plugged into pcb and odroid
//#define UART_PORT Serial
//#define PRINT(a) Serial.print(a); bluetooth.print(a);
//#define PRINTln(a) Serial.println(a); bluetooth.println(a);
//#define PRINTRES(a,b) Serial.print(a, b); bluetooth.print(a, b)
//#elif defined(DEVEL_MODE_2)
//#define UART_PORT Serial4
//#define PRINT(a) Serial4.print(a); bluetooth.print(a)
//#define PRINTln(a) Serial4.println(a); bluetooth.println(a)
//#define PRINTRES(a,b) Serial4.print(a, b); bluetooth.print(a, b)
//#endif
//

class Commands {
public:
    String activate = "activate";
    String deactivate = "deactivate";
    String who = "who";
    String bleOn = "ble-on";
    String bleOff = "ble-off";
    String closeLoop = "close-loop";
    String openLoop = "open-loop";
    String joystickOn = "joystick-on";
    String joystickOff = "joystick-off";
    String steerOff = "steer-off";
    String steerOn = "steer-on";
    String gpsOff = "gps-off";
    String gpsOn = "gps-on";
    String enc = "enc";
    String status = "status";
    String stop = "stop";
//    String op[] = [String(UART_PORT)];
    bool isActivated = false;
    bool isOpenloop = true; // No PID controller
    bool bluetoothMode = true;
    bool joystickMode = true;
    bool isSteering = true;
    bool error = false;
    bool isGpsImu = true;
    String errorMessage;
    void setupMessage(void);
    void handler(String cmd);
    void systemStatus(void);


};
void Commands::setupMessage(void){
    delay(50);
    toggleLed2();
    delay(50);
    toggleLed();
    delay(70);
    PRINTln("~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    PRINTln("setup complete");
    PRINTln();
    systemStatus();

}
void Commands::systemStatus() {
    PRINTln("~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    PRINTln("Rover Wheels are " + String((isActivated) ? "ACTIVE":"INACTIVE"));
    PRINTln("Ble-Mode: " + String((bluetoothMode) ? "ON":"OFF"));
    PRINTln("Control: " + String((joystickMode) ? "ArduinoBlue Joystick":"Bluetooth Serial"));
    PRINT("Nav " + String((error) ? "ERROR:":"Success\n"));
    PRINT((errorMessage) ? errorMessage : "");

    for (i = 0; i <= 5; i++) {
        String msg = String(i) + String(": ") + String(motorList[i].motorName) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
        PRINTln(msg);
    }
    PRINTln("~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~\n");

}
void Commands::handler(String cmd){

    if (isActivated) {
        if (cmd == "who") {
            PRINTln("Paralyzed Astro");
        }
        else if (cmd == deactivate) {
            //toggleLed();
            isActivated = false;
            PRINTln("Rover Wheels are Deactivated");
        } else if (cmd == "ble-on" ) {
            minInputSignal = -49;
            maxInputSignal = 49;
            bluetoothMode = true;
            PRINTln("Bluetooth Control is active");
        } else if (cmd == "ble-off") {
            minInputSignal = -49;
            maxInputSignal = 49;
            bluetoothMode = false;
            PRINTln("Bluetooth Control is off");
        } else if (cmd == "gps-off") {
            isGpsImu = false;
            PRINTln("GPS and IMU is Serial is now INACTIVE");
        } else if (cmd == "gps-on") {
            isGpsImu = true;
            PRINTln("GPS and IMU is Serial is now ACTIVE");
        } else if ((cmd.indexOf(":") > 0) && isSteering) {
            PRINTln("Received command");
            throttle = getValue(cmd, ':', 0).toInt();
            steering = getValue(cmd, ':', 1).toInt();
            PRINT("TEENSY throttle: ");
            PRINTln(throttle);
            PRINT("TEENSY steering: ");
            PRINTln(steering);
        }
        else if (cmd == closeLoop) {   // Close loop activation command
            PRINTln("Please Deactivate Rover Wheels");
        }
        else if (cmd == openLoop) {
            PRINTln("Please Deactivate Rover Wheels");
        }
        else if (cmd == steerOff){
            isSteering = false;
            PRINTln("Individual Wheel Control is Activated");

        }
        else if ((cmd.indexOf(":") > 0) && !isSteering){
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

            PRINT("Encoder Count is ");
            PRINTln(motorList[motorNumber].encoderCount)
        }
        else if (cmd == enc) {

            for (i = 1; i <= 6; i++) {
                motorList[i].isOpenLoop = true;
                PRINT("Motor ");
                PRINT(i);
                PRINT(" encoder count: ");
                PRINTln(motorList[i].encoderCount);
                delay(80);
            }
        }
        else if (cmd == stop){
            for (i = 1; i <= 6; i++) {
                motorList[i].setVelocity(1 , 0, motorList[i].getCurrentVelocity());


                PRINT("Motor ");
                PRINT(i);
                PRINT(" Speed is: ");
                PRINTln(motorList[i].getDesiredVelocity());
                delay(80);
            }
        }
        else {
            PRINTln("Unknown command you dumb hoe");
        }
    }
    else if (!isActivated){
        if (cmd == who) {
            PRINTln("happy Astro");
        }
        else if (cmd == activate){
            isActivated = true;
            bluetoothMode = false;
            toggleLed2();
            PRINTln("Rover Wheels are Active");
//            PRINTln((bluetoothMode) ? "BLE-Mode is ON": "BLE-Mode is OFF");
        }
        else if (cmd == bleOn) {
            minInputSignal = -49;
            maxInputSignal = 49;
            bluetoothMode = true;
            PRINTln("Bluetooth Control is active");
        } else if (cmd == bleOff) {
            minInputSignal = -49;
            maxInputSignal = 49;
            bluetoothMode = false;
            PRINTln("Bluetooth Control is off");
        } else if (cmd == gpsOff) {
            isGpsImu = false;
            PRINTln("GPS and IMU is Serial is now INACTIVE");
        } else if (cmd == gpsOn) {
            isGpsImu = true;
            PRINTln("GPS and IMU is Serial is now ACTIVE");
        }
        else if (cmd == closeLoop) {   // Close loop activation command
            minOutputSignal = -30;
            maxOutputSignal = 30;
            for (i = 0; i <= 5; i++) {
                motorList[i].isOpenLoop = false;
                String msg = "Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
                PRINTln(msg);
            }
        }
        else if (cmd == openLoop) {
            minOutputSignal = -255;
            maxOutputSignal = 255;
            for (i = 0; i <= 5; i++) {
                motorList[i].isOpenLoop = true;
                String msg = "Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
                PRINTln(msg);
            }
        }
        else {
            PRINTln("Unknown command you dumb hoe");

        }
    }



}




#endif //SKIDSTEERING_COMMANDS_H

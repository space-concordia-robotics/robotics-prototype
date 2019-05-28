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
    String s[3] = {"Serial", "Ble-Serial", "Ble"};
    String activate_cmd = "activate";
    String deactivate_cmd = "deactivate";
    String who_cmd = "who";
    String bleOn_cmd = "ble-on";
    String bleOff_cmd = "ble-off";
    String closeLoop_cmd = "close-loop";
    String openLoop_cmd = "open-loop";
    String joystickOn_cmd = "joystick-on";
    String joystickOff_cmd = "joystick-off";
    String steerOff_cmd = "steer-off";
    String steerOn_cmd = "steer-on";
    String gpsOff_cmd = "gps-off";
    String gpsOn_cmd = "gps-on";
    String encOn_cmd = "enc-on";
    String encOff_cmd = "enc-off";
    String accOn_cmd = "acc-on";
    String accOff_cmd = "acc-off";
    String status_cmd = "status";
    String stop_cmd = "stop";


//    String op[] = [String(UART_PORT)];
    bool isActivated = false;
    bool isOpenloop = true; // No PID controller
    bool bluetoothMode = true;
    bool isJoystickMode = true;
    bool isSteering = true;
    bool error = false;
    bool isGpsImu = true;
    bool isEnc = true;
    String errorMessage;

    int prevThrottle = 0;
    int prevSteering = 0;


    void setupMessage(void);
    void handler(String cmd, String sender);
    void bleHandler(void);
//    void blePrint(String cmd);
//    void blePrintln(String cmd);
//    void blePrintres(float a, float b );
    void systemStatus(void);

    void activate(String sender);
    void deactivate(String sender);
    void who(void);
    void bleOn(String sender);
    void bleOff(String sender);
    void closeLoop(void);
    void openLoop(void);
    void joystickOn(void);
    void joystickOff(void);
    void steerOff(void);
    void steerOn(void);
    void gpsOff(void);
    void gpsOn(void);
    void encOn(void);
    void encOff(void);
    void accOn(void);
    void accOff(void);
    void status(void);
    void stop(void);
    void errorMsg(void);



};
void Commands::setupMessage(void){
    delay(50);
    toggleLed2();
    delay(50);
    toggleLed();
    delay(70);
    PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    PRINTln("ASTRO setup complete");
    systemStatus();

}


void Commands::handler(String cmd, String sender){
    PRINT("ASTRO cmd: ");
    PRINTln(cmd);
    if (cmd == activate_cmd){
        activate(sender);
    }
    else if (cmd == deactivate_cmd) {
        deactivate(sender);
    }
    else if (cmd == who_cmd) {
        who();
    }
    else if (cmd == bleOn_cmd ) {
        bleOff(sender);
    }
    else if (cmd == bleOff_cmd) {
        bleOff(sender);
    }
    else if (cmd == closeLoop_cmd) {   // Close loop activation command
        closeLoop();
    }
    else if (cmd == openLoop_cmd) {
        openLoop();
    }
    else if (cmd == joystickOn_cmd){
        joystickOn();
    }
    else if (cmd == joystickOff_cmd) {
        joystickOff();
    }
    else if (cmd == steerOff_cmd){
        steerOff();
    }
    else if (cmd == steerOn_cmd){
        steerOn();
    }
    else if (cmd == gpsOff_cmd) {
        gpsOff();
    }
    else if (cmd == gpsOn_cmd) {
        gpsOn();
    }
    else if (cmd == encOn_cmd) {
        encOn();

    }
    else if (cmd == encOff_cmd) {
        encOff();
    }
    else if (cmd == accOn_cmd) {
        accOn();

    }
    else if (cmd == accOff_cmd) {
        accOff();
    }
    else if (cmd == stop_cmd){
        stop();
    }
    else if (cmd == status_cmd){
        status();
    }
    else if ((cmd.indexOf(":") > 0) && isSteering) {
        throttle = getValue(cmd, ':', 0).toFloat();
        steering = getValue(cmd, ':', 1).toFloat();
        PRINTln("ASTRO Throttle: " + String(throttle) + String(" Steering: ") + String(steering));

//        if (!isOpenloop){
//            throttle = map(throttle, -30, 30, -49, 49);
//            PRINTln("ASTRO Desired Speed: " + String(throttle) + String("RPM ") + String(" Steering: ") + String(steering));
//
//        }
        velocityHandler(throttle, steering);
        PRINT("ASTRO left: " + String(desiredVelocityLeft));
        PRINT(" - right: " + String(desiredVelocityRight));
        PRINTln(" maxOutput: " + String(maxOutputSignal));
    }
    else if ((cmd.indexOf(":") > 0) && !isSteering){
        motorNumber = getValue(cmd, ':', 0).toFloat();
        int speed = getValue(cmd, ':', 1).toFloat();
        steering = 0;
        int dir = 1;
        if (throttle < 0 ){
            dir = - 1;
        }
        motorList[motorNumber].calcCurrentVelocity();
        motorList[motorNumber].setVelocity(dir , abs(speed), motorList[motorNumber].getCurrentVelocity());
        PRINTln("ASTRO" + String(motorList[motorNumber].motorName) + String("'s desired speed: ") + String(motorList[motorNumber].getDesiredVelocity()) + String(" PWM ") + String(motorList[motorNumber].direction));
    }
    else if ((cmd.indexOf("!") == 0)){
        cmd.remove(0, 1);
        if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {
            PRINTln("ASTRO Front camera continuous base is moving at rate: " + String(cmd.toInt()));
            frontBase.write(cmd.toInt());
        }
        else {
            PRINTln("ASTRO Front camera continuous base: choose values from 0 to 180");

        }
    }
    else if ((cmd.indexOf("@") == 0)){
        cmd.remove(0, 1);
        if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {

            PRINTln("ASTRO Front camera Side positional servo is moving to angle: " + String(cmd.toInt()));
            frontSide.write(cmd.toInt());
        }
        else {
            PRINTln("ASTRO Front camera Side positional servo: choose values from 0 to 180");

        }
    }
    else if ((cmd.indexOf("#") == 0)){
        cmd.remove(0, 1);
        if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {

            PRINTln("ASTRO Top camera continuous base is moving at rate: " + String(cmd.toInt()));
            topBase.write(cmd.toInt());
        }
        else {
            PRINTln("ASTRO Top camera continuous base: choose values from 0 to 180");

        }
    }
    else if ((cmd.indexOf("$") == 0)){
        cmd.remove(0, 1);
        if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {

            PRINTln("ASTRO Top camera Side positional servo is moving to angle: " + String(cmd.toInt()));
            topSide.write(cmd.toInt());
        }
        else {
            PRINTln("ASTRO Top camera Side positional servo: choose values from 0 to 180");

        }
    }
    else if (sender == s[2]){
        PRINTln("ASTRO BOOOO! You used an unregistered button");
//        PRINT("");
        PRINTln("ASTRO Here is a list of the registered ArduinoBlue buttons:");
//        PRINT("");
        PRINTln("ASTRO Button id: 0 - Button function: activate rover");
//        PRINT("");
        PRINTln("ASTRO Button id: 1 - Button function: deactivate");
//        PRINT("");
        PRINTln("ASTRO Button id: 2 - Button function: turn on joystick control");
//        PRINT("");
        PRINTln("ASTRO Button id: 3 - Button function: turn off joystick control.");
    }
    else {
        PRINTln("ASTRO BOOOOOOO!!! Wrong command, try a different one, BIAAAAAA");
        if (!isJoystickMode) {
            PRINTln("ASTRO If you using ArduinoBlue buttons then type <joystick-on>");
        }
    }
}

void Commands::bleHandler(void) {
    if (isJoystickMode) {
        button = phone.getButton();
        throttle = phone.getThrottle();
        steering = phone.getSteering();
        if (isActivated && button == -1) {
            button  = -2;
            throttle -= 49.5;
            steering -= 49.5;
            velocityHandler(throttle, steering);
        }
        else if (!isActivated && button == -1){
            throttle = 0;
            steering = 0;
        }
    }
    if (!isJoystickMode){
        button = -4 ;
    }
    if (button == 0) {
        handler(activate_cmd, "Ble");
    }
    else if (button == 1){
        handler(deactivate_cmd, "Ble");
    }
    else if (button == 2){
        handler(joystickOn_cmd, "Ble");
    }
    else if (button == 3){
        handler(joystickOff_cmd, "Ble");
        button = -3;
    }
    else if (button == 4){
        handler(gpsOn_cmd, "Ble");
    }
    else if (button == 5){
        handler(gpsOff_cmd, "Ble");
    }
    else if (button == -1){
        PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");

        PRINTln("ASTRO Astro is inactive, activate to use joystick");
        PRINT("ASTRO If you're sending a command through Bluetooth  ");
        PRINTln("ASTRO serial app, then activate this mode first (btn id=3)");

    }
    else if (button == -3){
        PRINTln("ASTRO Disable bluetooth serial to drive Rover")
    }
    else if (button == -2){
//        PRINT("throttle: ");
//        PRINT(throttle);
//        PRINT(" - steering: ");
//        PRINTln(steering);
    }
    else if (button == -4){
//        PRINTln(123);
        cmd = bluetooth.readStringUntil('\n');
        uint8_t intRead= atoi (cmd.substring(1, 3).c_str ());
        if (intRead != 251){
            handler(cmd, "Ble-Serial");
        } else{
            PRINTln("ASTRO Activate Joystick controls first");
        }
    }
    else {
        handler("wrong command", s[2]);
    }

}
//
//void Commands::blePrint(String cmd){
//    if (bluetoothMode) {
//        bluetooth.print(cmd);
//        delay(50);
//    }
//}
//
//void Commands::blePrintln(String cmd){
//    if (bluetoothMode) {
//        bluetooth.print(cmd);
//        delay(50);
//    }
//}
//void Commands::blePrintres(float a, float b){
//    if (bluetoothMode) {
//        bluetooth.print(a, b);
//        delay(50);
//    }
//}
void Commands::systemStatus(void) {
    PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
//    delay(10);
    PRINT("ASTRO Wheels: ");
//    delay(10);
    PRINTln(String((isActivated) ? "ACTIVE":"INACTIVE"));
//    delay(10);
    PRINTln("ASTRO Ble-Mode: " + String((bluetoothMode) ? "ON":"OFF"));
//    delay(10);
    PRINTln("ASTRO Control: " + String((isJoystickMode) ? "ArduinoBlue Joystick":"Bluetooth Serial"));
//    delay(10);
    PRINT("ASTRO Nav " + String((error) ? "ERROR:":"Success\n"));
//    delay(10);
    PRINT(errorMessage ? (String("ASTRO ")+ String(errorMessage)) : "");
//    delay(10);

    for (i = 0; i <= 5; i++) {
        String msg = "ASTRO " + String(i) + String(": ") + String(motorList[i].motorName) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
        PRINTln(msg);
//        delay(110);

    }
    PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~\n");

}

void Commands::activate(String sender){
    if (isActivated) {
        PRINTln("ASTRO Rover is Already ACTIVATED")
    }
    else if (!isActivated){
        isActivated = true;
        if (sender == "Serial"){
            bluetoothMode = false;
        }

        toggleLed2();
        PRINTln("ASTRO Rover Wheels are Active");
        PRINTln((bluetoothMode) ? "ASTRO BLE-Mode is ON": "ASTRO BLE-Mode is OFF");
    }
}
void Commands::deactivate(String sender){
    if (isActivated) {
        stop();
        isActivated = false;
        if (sender == "Serial"){
            bluetoothMode = true;
        }
        toggleLed2();
        PRINTln("ASTRO Rover Wheels are Inactive");
        PRINTln((bluetoothMode) ? "ASTRO BLE-Mode is ON": "ASTRO LE-Mode is OFF");
    }
    else if (!isActivated){
        isActivated = false;
        if (sender == "Serial"){
            bluetoothMode = true;

        }
        toggleLed2();
        PRINTln("ASTRO Rover Wheels are Inactive");
        PRINTln((bluetoothMode) ? "ASTRO BLE-Mode is ON": "ASTRO BLE-Mode is OFF");    }
}

void Commands::who(void){
    if (isActivated) {
        PRINTln("ASTRO Happy Astro");
        toggleLed2();
    }
    else if (!isActivated){
        PRINTln("ASTRO Paralyzed Astro");
        toggleLed2();
    }
}

void Commands::bleOn(String sender){
    if (isActivated && sender == "Serial") {
        stop();
        minInputSignal = -49;
        maxInputSignal = 49;
        bluetoothMode = true;
        PRINTln("ASTRO Bluetooth Control is ON");

    }
    else if (!isActivated && sender == "Serial"){
        minInputSignal = -49;
        maxInputSignal = 49;
        bluetoothMode = true;
        PRINTln("ASTRO Bluetooth Control is ON");
    }
}
void Commands::bleOff(String sender){
    if (isActivated && sender == s[0]) {
        stop();
//        minInputSignal = -49;
//        maxInputSignal = 49;
        bluetoothMode = false;
        PRINTln("ASTRO Bluetooth Control is OFF");
    }
    else if (!isActivated && sender == s[0]){
//        minInputSignal = -49;
//        maxInputSignal = 49;
        bluetoothMode = false;
        PRINTln("ASTRO Bluetooth Control is OFF");
    }
    else {
        PRINTln("You must deactivate from serial");
    }
}

void Commands::closeLoop(void){
    if (isActivated) {
        stop();
        minOutputSignal = -30;
        maxOutputSignal = 30;
        PRINTln("Bo!")
        for (i = 0; i <= 5; i++) {
            motorList[i].isOpenLoop = false;
            String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
            PRINTln(msg);
        }
    }
    else if (!isActivated){
        minOutputSignal = -30;
        maxOutputSignal = 30;
        PRINTln("ASTRO Bo!")
        for (i = 0; i <= 5; i++) {
            motorList[i].isOpenLoop = false;
            String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
            PRINTln(msg);
        }
    }
}
void Commands::openLoop(void){
    if (isActivated) {
        stop();
        minOutputSignal = -255;
        maxOutputSignal = 255;
        for (i = 0; i <= 5; i++) {
            motorList[i].isOpenLoop = true;
            String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
            PRINTln(msg);
        }
    }
    else if (!isActivated){
        minOutputSignal = -255;
        maxOutputSignal = 255;
        for (i = 0; i <= 5; i++) {
            motorList[i].isOpenLoop = true;
            String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
            PRINTln(msg);
        }
    }
}

void Commands::joystickOn(void){
    if (isActivated) {
        stop();
        isJoystickMode = true;
        PRINTln("ASTRO Joystick is active, don't use Serial over Bluetooth");
    }
    else if (!isActivated){
        isJoystickMode = true;
        PRINTln("ASTRO Joystick is active, don't use Serial over Bluetooth");
    }


} // not needed for gui
void Commands::joystickOff(void){
    if (isActivated) {
        stop();
        isJoystickMode = false;
        PRINTln("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
    }
    else if (!isActivated){
        isJoystickMode = false;
        PRINTln("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
    }
//    if (sender = s[1]){
//        PRINTln("Please resend your command.");
//
//    }
} // not needed for gui

void Commands::steerOff(void){
    if (isActivated ) {
        stop();
        isSteering = false;
        PRINTln("ASTRO Individual Wheel Control is Activated");
    }
    else if (!isActivated){
        isSteering = false;
        PRINTln("ASTRO Individual Wheel Control is Activated");
    }
    if (isOpenloop){
        PRINTln("ASTRO Speed range is -255 to 255")
        PRINTln("ASTRO motorNumber:PWM");
    }
    else if (!isOpenloop){
        PRINTln("ASTRO Speed range is -30 to 30")
        PRINTln("ASTRO motorNumber:RPM");
    }

}
void Commands::steerOn(void){
    if (isActivated) {
        stop();
        isSteering = true;
        PRINTln("ASTRO Manual Skid Steering is Activated");
    }
    else if (!isActivated){
        isSteering = true;
        PRINTln("ASTRO Manual Skid Steering is Activated");
    }
    if (isOpenloop){
        PRINTln("ASTRO Speed range is -49 to 49")
        PRINTln("ASTRO throttle:steering");
    }
    else if (!isOpenloop){
        PRINTln("ASTRO Speed range is -30 to 30")
        PRINTln("ASTRO Steering range is -49 to 49")
        PRINTln("ASTRO throttle:steering");
    }
}


void Commands::gpsOff(void){
    isGpsImu = false;
    PRINTln("GPS and IMU Serial Stream is now Disabled");
}
void Commands::gpsOn(void){
    isGpsImu = true;
    PRINTln("ASTRO GPS and IMU Serial Stream is now Enabled");
}
void Commands::encOn(void){
    if (isActivated) {
        isEnc = true;
        PRINTln("ASTRO Velocity Readings Stream from Motor Encoders is ON")

    }
    else if (!isActivated){
        isEnc = true;
        PRINTln("ASTRO Motor Velocity Reading Stream is ON but will start printing values once the Rover is activated")
        for (i = 0; i <= 5; i++) {
            PRINT("ASTRO Motor ");
            PRINT(i);
            PRINT(" current velocity: ");
            PRINTln(motorList[i].getCurrentVelocity());
            delay(80);
        }
    }
}
void Commands::encOff(void){
    isEnc = false;
    PRINTln("ASTRO Velocity Readings Stream from Motor Encoders is OFF");
}

void Commands::accOn(void){

    for (i = 0; i <= 5; i++) {
        motorList[i].accLimit = true;
        String msg = "ASTRO Motor " + String(i) + String(" Acceleration Limiter: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
        PRINTln(msg);
    }

}

void Commands::accOff(void){

    for (i = 0; i <= 5; i++) {
        motorList[i].accLimit = false;
        String msg = "ASTRO Motor " + String(i) + String(" Acceleration Limiter: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
        PRINTln(msg);
    }

}

void Commands::status(void){
//    PRINTln("~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
//    PRINTln("Rover Wheels are " + String((isActivated) ? "ACTIVE":"INACTIVE"));
//    PRINTln("Ble-Mode: " + String((bluetoothMode) ? "ON":"OFF"));
//    PRINTln("Control: " + String((isJoystickMode) ? "ArduinoBlue Joystick":"Bluetooth Serial"));
//    PRINT("Nav " + String((error) ? "ERROR:":"Success\n"));
//    PRINT((errorMessage) ? errorMessage : "");
//
//    for (i = 0; i <= 5; i++) {
//        String msg = String(i) + String(": ") + String(motorList[i].motorName) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open":"CLose");
//        PRINTln(msg);
//    }
//    PRINTln("~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~\n");
    systemStatus();
}


void Commands::stop(void){

    velocityHandler(0, 0);
    for (int i = 0; i <= 5; i++) {

        PRINTln(String("ASTRO") + String(motorList[i].motorName) + String("'s desired speed: ") + String(motorList[i].getCurrentVelocity()) + String(" PWM ") + String(motorList[i].direction));

    }
//    PRINTln(String(motorList[i].motorName) + String("'s desired speed: ") + String(motorList[i].getDesiredVelocity()) + String(" PWM ") + String(motorList[i].direction));

}
void Commands::errorMsg(void){
    PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");

    PRINT("ASTRO " + String((error) ? "ERROR:":"No Error\n"));
//    delay(10);
    PRINT(errorMessage ? (String("ASTRO ")+ String(errorMessage)) : "");
//    delay(10);


    PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~\n");

}


#endif //SKIDSTEERING_COMMANDS_H

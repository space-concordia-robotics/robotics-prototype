#ifndef COMMANDS_H
#define COMMANDS_H

//
// Created by Fauzi Ameen on 2019-05-21
// Bluetooth removed to simplify code and reduce sources of error by Josh Glazer on 2019-05-30.
//

#include "motors.h"
#include "helpers.h"

class Commands {
public:
    String s[4] = {"USB", "Ble-Serial", "Ble", "UART"};
    String activate_cmd = "activate";
    String deactivate_cmd = "deactivate";
    String ping_cmd = "ping";
    String who_cmd = "who";
    String reboot_cmd = "reboot";
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

    elapsedMillis sinceThrottle; // timer for reading battery, gps and imu data


    bool isActivated = false;
    bool isOpenLoop = true; // No PID controller
    bool bluetoothMode = true;//true;
    bool isJoystickMode = true;
    bool isSteering = true;
    bool isGpsImu = true;
    bool isEnc = true;

    bool imuError = false;
    String imuErrorMsg = "ASTRO IMU may be malfunctioning, timeout reached";
    bool gpsError = false;
    String gpsErrorMsg = "ASTRO GPS could not be initialized";

    int prevThrottle = 0;
    int prevSteering = 0;

    void setupMessage(void);
    void handler(String cmd, String sender);
    void bleHandler(void);
    void systemStatus(void);

    void activate(String sender);
    void deactivate(String sender);
    void ping(void);
    void who(void);
    void rebootTeensy(void); //!< reboots the teensy using the watchdog timer
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
    void stop(bool timeout = false);
    void controlWheelMotors(String cmd);
    void controlCameraMotors(String cmd);
};

void Commands::setupMessage(void) {
    println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    println("ASTRO setup complete");
    systemStatus();
}

void Commands::handler(String cmd, String sender) {

    println("ASTRO GOT: " + cmd);
    if (cmd == activate_cmd) {
        activate(sender);
    }
    else if (cmd == deactivate_cmd) {
        deactivate(sender);
    }
    else if (cmd == ping_cmd) {
        ping();
    }
    else if (cmd == who_cmd) {
        who();
    }
    else if (cmd == reboot_cmd) {
        rebootTeensy();
    }
    else if (cmd == closeLoop_cmd) {
        closeLoop();
    }
    else if (cmd == openLoop_cmd) {
        openLoop();
    }
    else if (cmd == steerOff_cmd) {
        steerOff();
    }
    else if (cmd == steerOn_cmd) {
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
    else if (cmd == stop_cmd) {
        stop();
    }
    else if (cmd == status_cmd) {
        systemStatus();
    }
        // this is not safe enough, needs much stricter verification
    else if (cmd.indexOf(":") > 0) {
        controlWheelMotors(cmd);
    }
    else if (cmd[0] == '!' || cmd[0] == '@' || cmd[0] == '#' || cmd[0] == '$') {
        controlCameraMotors (cmd);
    }
    else {
        println("ASTRO BOOOOOOO!!! Wrong command, try a different one, BIAAAAAA");
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
        else if (!isActivated && button == -1) {
            throttle = 0;
            steering = 0;
        }
    }
    if (!isJoystickMode) {
        button = -4 ;
    }
    if (button == 0) {
        handler(activate_cmd, "Ble");
    }
    else if (button == 1) {
        handler(deactivate_cmd, "Ble");
    }
    else if (button == 2) {
        handler(joystickOn_cmd, "Ble");
    }
    else if (button == 3) {
        handler(joystickOff_cmd, "Ble");
        button = -3;
    }
    else if (button == 4) {
        handler(gpsOn_cmd, "Ble");
    }
    else if (button == 5) {
        handler(gpsOff_cmd, "Ble");
    }
    else if (button == -1) {
        println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
        println("ASTRO Astro is inactive, activate to use joystick");
        print("ASTRO If you're sending a command through Bluetooth ");
        println("serial app, then activate this mode first (btn id=3)");
    }
    else if (button == -3) {
        println("ASTRO Disable bluetooth serial to drive Rover");
    }
    else if (button == -2) {
        //        PRINT("throttle: ");
        //        PRINT(throttle);
        //        PRINT(" - steering: ");
        //        PRINTln(steering);
    }
    else if (button == -4) {
        //        PRINTln(123);
        cmd = bluetooth.readStringUntil('\n');
        uint8_t intRead = atoi (cmd.substring(1, 3).c_str ());
        if (intRead != 251) {
            handler(cmd, "Ble-Serial");
        } else {
            println("ASTRO Activate Joystick controls first");
        }
    }
    else {
        handler("wrong command", s[2]);
    }
}

void Commands::systemStatus(void) {
    println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    println("ASTRO Astro has " + String(RobotMotor::numMotors) + " motors");
    println("ASTRO Wheels: " + String(isActivated ? "ACTIVE" : "INACTIVE"));
    println("ASTRO Steering: " + String(isSteering ? "Steering Control" : "Motor Control"));
    println("ASTRO Encoders: " + String(isEnc ? "ON" : "OFF"));
    println("ASTRO GPS " + String(gpsError ? "ERROR: " + gpsErrorMsg : "Success"));
    println("ASTRO IMU " + String(imuError ? "ERROR: " + imuErrorMsg : "Success"));
    println("ASTRO Nav Stream: " + String(isGpsImu ? "ON" : "OFF"));
    print("ASTRO Motor loop statuses: ");
    for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
        print(String(motorList[i].isOpenLoop ? "Open" : "CLose"));
        if (i != RobotMotor::numMotors - 1) print(", ");
    }
    println("");

    print("ASTRO Motor accel: ");
    for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
        print((motorList[i].accLimit) ? "ON" : "OFF");
        if (i != RobotMotor::numMotors - 1) print(", ");
    }
    println("");

    println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
}

void Commands::activate(String sender) {
    if (isActivated) {
        println("ASTRO Rover is Already ACTIVATED");
    }
    else {
        isActivated = true;
        if (sender == "USB" || sender == "UART") {
            bluetoothMode = false;
        }
        println("ASTRO Rover Wheels are Active");
        println((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO BLE-Mode is OFF");
    }
}
void Commands::deactivate(String sender) {
    if (isActivated) {
        stop();
        isActivated = false;
        if (sender == "USB" || sender == "UART") {
            bluetoothMode = true;
        }
        println("ASTRO Rover Wheels are Inactive");
        println((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO LE-Mode is OFF");
    }
    else{
        println("ASTRO Rover Wheels are already inactive");
    }
}

void Commands::ping(void) {
    println("ASTRO pong");
}
void Commands::who(void) {
    if (isActivated) {
        println("ASTRO Happy Astro");
    }
    else {
        println("ASTRO Paralyzed Astro");
    }
}

void Commands::rebootTeensy(void) {
    println("ASTRO rebooting wheel teensy... hang on a sec");
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    // The next 2 lines set the time-out value.
    WDOG_TOVALL = 15; // This is the value (ms) that the watchdog timer compare itself to.
    WDOG_TOVALH = 0; // End value (ms) WDT compares itself to.
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
                    WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
    WDOG_PRESC = 0; //Sets watchdog timer to tick at 1 kHz inseast of 1/4 kHz
    while (true); // infinite do nothing loop -- wait for the countdown
}

void Commands::bleOn(String sender) {
    if (isActivated && (sender == "USB" || sender == "UART")) {
        stop();
//        minInputSignal = -49;
//        maxInputSignal = 49;
        bluetoothMode = true;
        println("ASTRO Bluetooth Control is ON");
    }
    else if (!isActivated && (sender == "USB" || sender == "UART")) {
//        minInputSignal = -49;
//        maxInputSignal = 49;
        bluetoothMode = true;
        println("ASTRO Bluetooth Control is ON");
    }
}
void Commands::bleOff(String sender) {
    if (isActivated && (sender == "USB" || sender == "UART")) {
        stop();
        //        minInputSignal = -49;
        //        maxInputSignal = 49;
        bluetoothMode = false;
        println("ASTRO Bluetooth Control is OFF");
    }
    else if (!isActivated && (sender == "USB" || sender == "UART")) {
        //        minInputSignal = -49;
        //        maxInputSignal = 49;
        bluetoothMode = false;
        println("ASTRO Bluetooth Control is OFF");
    }
    else {
        println("You must deactivate from serial");
    }
}

void Commands::closeLoop(void) {
    if (isActivated) {
        stop();
    }
    println("ASTRO Turning encoders on first...");
    encOn();
    maxOutputSignal = MAX_RPM_VALUE; minOutputSignal = MIN_RPM_VALUE;
    println("ASTRO Bo!");
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].isOpenLoop = false;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += String(" loop status is: ");
        msg += String(motorList[i].isOpenLoop ? "Open" : "CLose");
        println(msg);
    }
}
void Commands::openLoop(void) {
    if (isActivated) {
        stop();
    }
    maxOutputSignal = MAX_PWM_VALUE; minOutputSignal = MIN_PWM_VALUE;
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].isOpenLoop = true;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += String(" loop status is: ");
        msg += String(motorList[i].isOpenLoop ? "Open" : "CLose");
        println(msg);
    }
}

void Commands::joystickOn(void) {
    if (isActivated) {
        stop();
        isJoystickMode = true;
        println("ASTRO Joystick is active, don't use Serial over Bluetooth");
    }
    else if (!isActivated) {
        isJoystickMode = true;
        println("ASTRO Joystick is active, don't use Serial over Bluetooth");
    }

} // To activate Joystick control in arduino blue
void Commands::joystickOff(void) {
    if (isActivated) {
        stop();
        isJoystickMode = false;
        println("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
    }
    else if (!isActivated) {
        isJoystickMode = false;
        println("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
    }
    //    if (sender = s[1]){
    //        PRINTln("Please resend your command.");
    //
    //    }
} // not needed for gui

void Commands::steerOff(void) {
    if (isActivated ) {
        stop();
    }
    isSteering = false;
    println("ASTRO Individual Wheel Control is Activated");
    if (isOpenLoop) {
        println("ASTRO Speed range is -255 to 255 (open loop)");
        println("ASTRO commands are now: motorNumber:PWM");
    }
    else {
        println("ASTRO Speed range is -30 to 30 (closed loop)");
        println("ASTRO commands are now: motorNumber:RPM");
    }
}
void Commands::steerOn(void) {
    if (isActivated) {
        stop();
    }
    isSteering = true;
    println("ASTRO Skid Steering is Activated");
    if (isOpenLoop) {
        println("ASTRO Throttle and Steering ranges are -49 to 49 (open loop)");
        println("ASTRO commands are now: throttle:steering");
    }
    else {
        println("ASTRO Throttle range is -30 to 30 (closed loop)");
        println("ASTRO Steering range is -49 to 49");
        println("ASTRO commands are now: RPM:steering");
    }
}

void Commands::gpsOff(void) {
    isGpsImu = false;
    println("ASTRO GPS and IMU Serial Stream is now Disabled");
}
void Commands::gpsOn(void) {
    isGpsImu = true;
    println("ASTRO GPS and IMU Serial Stream is now Enabled");
}

void Commands::encOn(void) {
    if (isActivated) {
        isEnc = true;
        println("ASTRO Velocity Readings Stream from Motor Encoders is ON");
    }
    else if (!isActivated) {
        isEnc = true;
        println("ASTRO Motor Velocity Reading Stream is ON but will start printing values once the Rover is activated");
        for (i = 0; i < RobotMotor::numMotors; i++) {
            print("ASTRO Motor ");
            print(i);
            print(" current velocity: ");
            println(motorList[i].getCurrentVelocity());
            //delay(80);
        }
    }
}
void Commands::encOff(void) {
    isEnc = false;
    println("ASTRO Velocity Readings Stream from Motor Encoders is OFF");
}

//void Commands::accOn(void) {
//    for (i = 0; i < RobotMotor::numMotors; i++) {
//        motorList[i].accLimit = true;
//        String msg = "ASTRO Motor " + String(i + 1);
//        msg += " Acceleration Limiter: ";
//        msg += String(motorList[i].accLimit ? "Open" : "CLose");
//        println(msg);
//    }
//}
//void Commands::accOff(void) {
//    for (i = 0; i < RobotMotor::numMotors; i++) {
//        motorList[i].accLimit = false;
//        String msg = "ASTRO Motor " + String(i + 1);
//        msg += " Acceleration Limiter: ";
//        msg += String(motorList[i].accLimit ? "Open" : "CLose");
//        println(msg);
//    }
//}

void Commands::controlWheelMotors(String cmd) {
    if (!isActivated) {
        println("ASTRO Astro isn't activated yet!!!");
    }
    else {
        if (isSteering) {
            throttle = getValue(cmd, ':', 0).toFloat();
            steering = getValue(cmd, ':', 1).toFloat();
            println("ASTRO Throttle: " + String(throttle) + String(" -- Steering: ") + String(steering));
            //        if (!isOpenLoop){
            //            throttle = map(throttle, -30, 30, -49, 49);
            //            println("ASTRO Desired Speed: " + String(throttle) + String("RPM ") + String(" Steering: ") + String(steering));
            //        }
            velocityHandler(throttle, steering);
            String msg = "ASTRO left: " + String(desiredVelocityLeft);
            msg += " -- right: " + String(desiredVelocityRight);
            msg += " maxOutput: " + String(maxOutputSignal);
            println(msg);
            sinceThrottle = 0;
        }
        else {
            motorNumber = getValue(cmd, ':', 0).toFloat();
            int motorSpeed = getValue(cmd, ':', 1).toFloat();
            steering = 0;
            int dir = 1;
            if (throttle < 0 ) {
                dir = - 1;
            }
            motorList[motorNumber].calcCurrentVelocity();
            motorList[motorNumber].setVelocity(dir , abs(motorSpeed), motorList[motorNumber].getCurrentVelocity());
            println("ASTRO " + String(motorList[motorNumber].motorName) + String("'s desired speed: ") + String(motorList[motorNumber].getCurrentVelocity()) + String(" PWM "));
        }
    }
}

//! !FB, @FS, #RB, $RS
void Commands::controlCameraMotors(String cmd) {
    if (!isActivated) {
        println("ASTRO Astro isn't activated yet!!!");
    }
    else {
        // this can be refactored with servoList[] - and more, probably
        char servoSymbol = cmd[0];
        String angleStr = cmd.remove(0, 1);
        int angleInt = cmd.toInt();
        String servoName;
        switch (servoSymbol) {
            case '!':
                servoName = "Front camera positional tilt base";
                if ( (0 <= angleInt) && (angleInt <= 180) ) {
                    println("ASTRO "+servoName+" is moving is moving to angle " + angleStr);
                    frontBase.write(angleInt);
                }
                else {
                    println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
            case '@':
                servoName = "Front camera Side continuous servo";
                if ( (0 <= angleInt) && (angleInt <= 180)) {
                    println("ASTRO "+servoName+" is moving at rate: " + angleStr);
                    frontSide.write(angleInt);
                }
                else {
                    println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
            case '#':
                servoName = "Rear camera positional tilt base";
                if ( (0 <= angleInt) && (angleInt <= 180) ) {
                    println("ASTRO "+servoName+" is moving is moving to angle " + angleStr);
                    rearBase.write(angleInt);
                }
                else {
                    println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
            case '$':
                servoName = "Rear camera Side continuous servo";
                if ( (0 <= angleInt) && (angleInt <= 180)) {
                    println("ASTRO "+servoName+" is moving at rate: " + angleStr);
                    frontSide.write(angleInt);
                }
                else {
                    println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
        }
    }
}

void Commands::stop(bool timeout) {
    if (timeout) println("ASTRO Throttle Timeout");
    velocityHandler(0, 0);
    for (int i = 0; i < RobotMotor::numMotors; i++) {
        String msg = "ASTRO " + String(motorList[i].motorName);
        msg += "'s desired speed: " + String(motorList[i].getCurrentVelocity()) + " PWM ";
        println(msg);
    }
}

#endif

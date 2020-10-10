  #include "commands.h"

Commands::Commands()
{

}
void Commands::setupMessage(void) {
    Helpers::get().println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    Helpers::get().println("ASTRO setup complete");
    systemStatus();
}
void Commands::setPhone(ArduinoBlue* phone){
  this->phone = phone;
}
void Commands::handler(String cmd, String sender) {

     Helpers::get().println("ASTRO GOT: " + cmd);
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
    else if (cmd == bleOn_cmd ) {
        bleOn(sender);
    }
    else if (cmd == bleOff_cmd) {
        bleOff(sender);
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
    else if (cmd == ttoOn_cmd) {
        ttoOn();
    }
    else if (cmd == ttoOff_cmd) {
        ttoOff();
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
        Helpers::get().println("ASTRO BOOOOOOO!!! Wrong command, try a different one, BIAAAAAA");
    }
}

void Commands::bleHandler(void) {
    if (isJoystickMode) {
        button = this->phone->getButton();
        throttle = this->phone->getThrottle();
        steering = this->phone->getSteering();
        if (isActivated && button == -1) {
            button  = -2;
            throttle -= 49.5;
            steering -= 49.5;
            DcMotor::velocityHandler(this->motorList,throttle, steering);
            Helpers::get().println(String(sinceThrottle));
            sinceThrottle = 0;
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
        Helpers::get().println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
        Helpers::get().println("ASTRO Astro is inactive, activate to use joystick");
        Helpers::get().print("ASTRO If you're sending a command through Bluetooth ");
        Helpers::get().println("serial app, then activate this mode first (btn id=3)");
    }
    else if (button == -3) {
        Helpers::get().println("ASTRO Disable bluetooth serial to drive Rover");
    }
    else if (button == -2) {
        //        PRINT("throttle: ");
        //        PRINT(throttle);
        //        PRINT(" - steering: ");
        //        PRINTln(steering);
    }
    else if (button == -4) {
        //        PRINTln(123);
        cmd = this->bluetooth->readStringUntil('\n');
        uint8_t intRead = atoi (cmd.substring(1, 3).c_str ());
        if (intRead != 251) {
            handler(cmd, "Ble-Serial");
        } else {
            Helpers::get().println("ASTRO Activate Joystick controls first");
        }
    }
    else {
        handler("wrong command", s[2]);
    }
}

void Commands::systemStatus(void) {
    Helpers::get().println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    Helpers::get().println("ASTRO Astro has " + String(RobotMotor::numMotors) + " motors");
    Helpers::get().println("ASTRO Wheels: " + String(isActivated ? "ACTIVE" : "INACTIVE"));
    Helpers::get().println("ASTRO Steering: " + String(isSteering ? "Steering Control" : "Motor Control"));
    Helpers::get().println("ASTRO Encoders: " + String(isEnc ? "ON" : "OFF"));
    Helpers::get().println("ASTRO GPS " + String(gpsError ? "ERROR: " + gpsErrorMsg : "Success"));
    Helpers::get().println("ASTRO IMU " + String(imuError ? "ERROR: " + imuErrorMsg : "Success"));
    Helpers::get().println("ASTRO Nav Stream: " + String(isGpsImu ? "ON" : "OFF"));
    Helpers::get().print("ASTRO Motor loop statuses: ");
    for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
        Helpers::get().print(String(motorList[i].isOpenLoop ? "Open" : "CLose"));
        if (i != RobotMotor::numMotors - 1) Helpers::get().print(", ");
    }
    Helpers::get().println("");

    Helpers::get().print("ASTRO Motor accel: ");
    for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
        Helpers::get().print((motorList[i].accLimit) ? "ON" : "OFF");
        if (i != RobotMotor::numMotors - 1) Helpers::get().print(", ");
    }
    Helpers::get().println("");

    Helpers::get().println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
}

void Commands::activate(String sender) {
    if (isActivated) {
        Helpers::get().println("ASTRO Rover is Already ACTIVATED");
    }
    else {
        isActivated = true;
        if (sender == "USB" || sender == "UART") {
            bluetoothMode = false;
        }
        Helpers::get().println("ASTRO Rover Wheels are Active");
        Helpers::get().println((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO BLE-Mode is OFF");
    }
}
void Commands::deactivate(String sender) {
    if (isActivated) {
        stop();
        isActivated = false;
        if (sender == "USB" || sender == "UART") {
            bluetoothMode = true;
        }
        Helpers::get().println("ASTRO Rover Wheels are Inactive");
        Helpers::get().println((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO LE-Mode is OFF");
    }
    else{
        Helpers::get().println("ASTRO Rover Wheels are already inactive");
    }
}

void Commands::ping(void) {
    Helpers::get().println("ASTRO pong");
}
void Commands::who(void) {
    if (isActivated) {
        Helpers::get().println("ASTRO Happy Astro");
    }
    else {
        Helpers::get().println("ASTRO Paralyzed Astro");
    }
}

void Commands::rebootTeensy(void) {
    Helpers::get().println("ASTRO rebooting wheel teensy... hang on a sec");
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
        Helpers::get().println("ASTRO Bluetooth Control is ON");
    }
    else if (!isActivated && (sender == "USB" || sender == "UART")) {
//        minInputSignal = -49;
//        maxInputSignal = 49;
        bluetoothMode = true;
        Helpers::get().println("ASTRO Bluetooth Control is ON");
    }
}
void Commands::bleOff(String sender) {
    if (isActivated && (sender == "USB" || sender == "UART")) {
        stop();
        //        minInputSignal = -49;
        //        maxInputSignal = 49;
        bluetoothMode = false;
        Helpers::get().println("ASTRO Bluetooth Control is OFF");
    }
    else if (!isActivated && (sender == "USB" || sender == "UART")) {
        //        minInputSignal = -49;
        //        maxInputSignal = 49;
        bluetoothMode = false;
        Helpers::get().println("ASTRO Bluetooth Control is OFF");
    }
    else {
        Helpers::get().println("You must deactivate from serial");
    }
}

void Commands::closeLoop(void) {
    if (isActivated) {
        stop();
    }
    Helpers::get().println("ASTRO Turning encoders on first...");
    encOn();
    maxOutputSignal = MAX_RPM_VALUE; minOutputSignal = MIN_RPM_VALUE;
    Helpers::get().println("ASTRO Bo!");
    motorList[0].isOpenLoop = false;
    motorList[1].isOpenLoop = false;
    motorList[2].isOpenLoop = false;
    motorList[3].isOpenLoop = false;
    motorList[4].isOpenLoop = false;
    motorList[5].isOpenLoop = false;

    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].isOpenLoop = false;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += String(" loop status is: ");
        msg += String(motorList[i].isOpenLoop ? "Open" : "CLose");
        Helpers::get().println(msg);
    }
}
void Commands::openLoop(void) {
    if (isActivated) {
        stop();
    }
    maxOutputSignal = MAX_PWM_VALUE; minOutputSignal = MIN_PWM_VALUE;

    motorList[0].isOpenLoop = true;
    motorList[1].isOpenLoop = true;
    motorList[2].isOpenLoop = true;
    motorList[3].isOpenLoop = true;
    motorList[4].isOpenLoop = true;
    motorList[5].isOpenLoop = true;
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].isOpenLoop = true;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += String(" loop status is: ");
        msg += String(motorList[i].isOpenLoop ? "Open" : "CLose");
        Helpers::get().println(msg);
    }
}

void Commands::joystickOn(void) {
    if (isActivated) {
        stop();
        isJoystickMode = true;
        Helpers::get().println("ASTRO Joystick is active, don't use Serial over Bluetooth");
    }
    else if (!isActivated) {
        isJoystickMode = true;
        Helpers::get().println("ASTRO Joystick is active, don't use Serial over Bluetooth");
    }

} // To activate Joystick control in arduino blue
void Commands::joystickOff(void) {
    if (isActivated) {
        stop();
        isJoystickMode = false;
        Helpers::get().println("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
    }
    else if (!isActivated) {
        isJoystickMode = false;
        Helpers::get().println("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
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
    Helpers::get().println("ASTRO Individual Wheel Control is Activated");
    if (isOpenLoop) {
        Helpers::get().println("ASTRO Speed range is -255 to 255 (open loop)");
        Helpers::get().println("ASTRO commands are now: motorNumber:PWM");
    }
    else {
        Helpers::get().println("ASTRO Speed range is -30 to 30 (closed loop)");
        Helpers::get().println("ASTRO commands are now: motorNumber:RPM");
    }
}
void Commands::steerOn(void) {
    if (isActivated) {
        stop();
    }
    isSteering = true;
    Helpers::get().println("ASTRO Skid Steering is Activated");
    if (isOpenLoop) {
       Helpers::get().println("ASTRO Throttle and Steering ranges are -49 to 49 (open loop)");
        Helpers::get().println("ASTRO commands are now: throttle:steering");
    }
    else {
        Helpers::get().println("ASTRO Throttle range is -49 to 49 (closed loop)");
        Helpers::get().println("ASTRO Steering range is -49 to 49");
        Helpers::get().println("ASTRO commands are now: RPM:steering");
    }
}

void Commands::gpsOff(void) {
    isGpsImu = false;
    Helpers::get().println("ASTRO GPS and IMU Serial Stream is now Disabled");
}
void Commands::gpsOn(void) {
    isGpsImu = true;
    Helpers::get().println("ASTRO GPS and IMU Serial Stream is now Enabled");
}

void Commands::encOn(void) {
    if (isActivated) {
        isEnc = true;
        Helpers::get().println("ASTRO Velocity Readings Stream from Motor Encoders is ON");
    }
    else if (!isActivated) {
        isEnc = true;
        Helpers::get().println("ASTRO Motor Velocity Reading Stream is ON but will start printing values once the Rover is activated");
        for (i = 0; i < RobotMotor::numMotors; i++) {
            Helpers::get().print("ASTRO Motor ");
            Helpers::get().print(i);
            Helpers::get().print(" current velocity: ");
            Helpers::get().println(motorList[i].getCurrentVelocity());
            //delay(80);
        }
    }
}
void Commands::encOff(void) {
    isEnc = false;
    Helpers::get().println("ASTRO Velocity Readings Stream from Motor Encoders is OFF");
}

void Commands::accOn(void) {
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].accLimit = true;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += " Acceleration Limiter: ";
        msg += String(motorList[i].accLimit ? "Open" : "CLose");
        Helpers::get().println(msg);
    }
}
void Commands::accOff(void) {
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].accLimit = false;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += " Acceleration Limiter: ";
        msg += String(motorList[i].accLimit ? "Open" : "CLose");
        Helpers::get().println(msg);
    }
}

void Commands::ttoOn(void) {
    throttleTimeOut = true;
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    Helpers::get().println(msg);

}
void Commands::ttoOff(void) {
    throttleTimeOut = false;
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    Helpers::get().println(msg);

}

void Commands::controlWheelMotors(String cmd) {
    if (!isActivated) {
        Helpers::get().println("ASTRO Astro isn't activated yet!!!");
    }
    else {
        if (isSteering) {
            throttle = Helpers::get().getValue(cmd, ':', 0).toFloat();
            steering = Helpers::get().getValue(cmd, ':', 1).toFloat();
            Helpers::get().println("ASTRO Throttle: " + String(throttle) + String(" -- Steering: ") + String(steering));
            //        if (!isOpenLoop){
            //            throttle = map(throttle, -30, 30, -49, 49);
            //            println("ASTRO Desired Speed: " + String(throttle) + String("RPM ") + String(" Steering: ") + String(steering));
            //        }
            DcMotor::velocityHandler(motorList,throttle, steering);
            String msg = "ASTRO left: " + String(desiredVelocityLeft);
            msg += " -- right: " + String(desiredVelocityRight);
            msg += " maxOutput: " + String(maxOutputSignal);
            Helpers::get().println(msg);
            sinceThrottle = 0;
        }
        else {
          // TODO: fix this.
          // 1) make sure motor numbering makes sense in both closed and open loop (1 to 6 not 0 to 5)
          // 2) make sure motor order is correct (1 to 6 actually turns motors 1 to 6)
            motorNumber = Helpers::get().getValue(cmd, ':', 0).toFloat();
            int motorSpeed = Helpers::get().getValue(cmd, ':', 1).toFloat();
            steering = 0;
            int dir = 1;
            if (motorSpeed < 0 ) {
                dir = - 1;
            }
            sinceThrottle = 0;

            if (motorNumber>=1 && motorNumber<=6){
                motorList[motorNumber-1].calcCurrentVelocity();
                motorList[motorNumber-1].setVelocity(dir , abs(motorSpeed), motorList[motorNumber-1].getCurrentVelocity());
                Helpers::get().println("ASTRO " + String(motorList[motorNumber-1].motorName) + String("'s desired speed: ") + String(motorList[motorNumber-1].desiredVelocity) + String(" PWM "));
            }
            else {
                Helpers::get().println("ASTRO invalid motor  number");
            }
//            motorList[motorNumber].desiredVelocity = motorSpeed;
//            motorList[motorNumber].desiredDirection = dir;
//            motorList[motorNumber].calcCurrentVelocity();
//            motorList[motorNumber].setVelocity(dir , abs(motorSpeed), motorList[motorNumber].getCurrentVelocity());
//            println("ASTRO " + String(motorList[motorNumber].motorName) + String("'s desired speed: ") + String(motorList[motorNumber].desiredVelocity) + String(" PWM "));
        }
    }
}

//! !FB, @FS, #RB, $RS
void Commands::controlCameraMotors(String cmd) {
    if (!isActivated) {
        Helpers::get().println("ASTRO Astro isn't activated yet!!!");
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
                    Helpers::get().println("ASTRO "+servoName+" is moving is moving to angle " + angleStr);
                    servoList[0].write(angleInt);
                }
                else {
                    Helpers::get().println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
            case '@':
                servoName = "Front camera Side continuous servo";
                if ( (0 <= angleInt) && (angleInt <= 180)) {
                    Helpers::get().println("ASTRO "+servoName+" is moving at rate: " + angleStr);
                    servoList[1].write(angleInt);
                }
                else {
                    Helpers::get().println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
            case '#':
                servoName = "Rear camera positional tilt base";
                if ( (0 <= angleInt) && (angleInt <= 180) ) {
                    Helpers::get().println("ASTRO "+servoName+" is moving is moving to angle " + angleStr);
                    servoList[2].write(angleInt);
                }
                else {
                    Helpers::get().println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
            case '$':
                servoName = "Rear camera Side continuous servo";
                if ( (0 <= angleInt) && (angleInt <= 180)) {
                    Helpers::get().println("ASTRO "+servoName+" is moving at rate: " + angleStr);
                    servoList[3].write(angleInt);
                }
                else {
                    Helpers::get().println("ASTRO "+servoName+": choose values from 0 to 180");
                }
                break;
        }
    }
}

void Commands::stop(bool timeout) {
//    if (timeout) println("ASTRO Throttle Timeout");
    DcMotor::velocityHandler(motorList,0, 0);
//    for (int i = 0; i < RobotMotor::numMotors; i++) {
//        String msg = "ASTRO " + String(motorList[i].motorName);
//        msg += "'s desired speed: " + String(motorList[i].getCurrentVelocity()) + " PWM ";
//        println(msg);

}

void Commands::setBluetooth(SoftwareSerial* bluetooth){
  this->bluetooth = bluetooth;
}
void Commands::setMotorList(DcMotor* motorList){
  this->motorList = motorList;
}
void Commands::setServoList(Servo* servoList){
  this->servoList = servoList;
}

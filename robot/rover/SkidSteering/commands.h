#ifndef COMMANDS_H
#define COMMANDS_H

//
// Created by Fauzi Ameen on 2019-05-21.
//

#include "DcMotor.h"

class Commands {
  public:
    String s[3] = {"Serial", "Ble-Serial", "Ble"};
    String activate_cmd = "activate";
    String deactivate_cmd = "deactivate";
    String ping_cmd = "ping";
    String who_cmd = "who";
    String reboot_cmd = "reboot";
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
    bool bluetoothMode = false;//true;
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
    void stop(void);
    void errorMsg(void);
};
void Commands::setupMessage(void) {
  /*delay(50);
    toggleLed2();
    delay(50);
    toggleLed();
    delay(70);*/
  PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
  PRINTln("ASTRO setup complete");
  systemStatus();
}

void Commands::handler(String cmd, String sender) {
  PRINTln("ASTRO GOT: " + cmd);
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
  else if (cmd == closeLoop_cmd) {   // Close loop activation command
    closeLoop();
  }
  else if (cmd == openLoop_cmd) {
    openLoop();
  }
  else if (cmd == joystickOn_cmd) {
    joystickOn();
  }
  else if (cmd == joystickOff_cmd) {
    joystickOff();
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
    status();
  }
  else if ((cmd.indexOf(":") > 0) && isSteering) {
    if (isActivated) {
      throttle = getValue(cmd, ':', 0).toFloat();
      steering = getValue(cmd, ':', 1).toFloat();
      PRINTln("ASTRO Throttle: " + String(throttle) + String(" Steering: ") + String(steering));
      //        if (!isOpenloop){
      //            throttle = map(throttle, -30, 30, -49, 49);
      //            PRINTln("ASTRO Desired Speed: " + String(throttle) + String("RPM ") + String(" Steering: ") + String(steering));
      //        }
      velocityHandler(throttle, steering);
      PRINT("ASTRO left: " + String(desiredVelocityLeft));
      PRINT(" - right: " + String(desiredVelocityRight));
      PRINTln(" maxOutput: " + String(maxOutputSignal));
    }
    else {
      PRINTln("ASTRO Astro isn't activated yet!!!");
    }
  }
  else if ((cmd.indexOf(":") > 0) && !isSteering) {
    if (isActivated) {
      motorNumber = getValue(cmd, ':', 0).toFloat();
      int speed = getValue(cmd, ':', 1).toFloat();
      steering = 0;
      int dir = 1;
      if (throttle < 0 ) {
        dir = - 1;
      }
      motorList[motorNumber].calcCurrentVelocity();
      motorList[motorNumber].setVelocity(dir , abs(speed), motorList[motorNumber].getCurrentVelocity());
      PRINTln("ASTRO " + String(motorList[motorNumber].motorName) + String("'s desired speed: ") + String(motorList[motorNumber].getDesiredVelocity()) + String(" PWM ") + String(motorList[motorNumber].direction));
    }
    else {
      PRINTln("ASTRO Astro isn't activated yet!!!");
    }
  }
  else if ((cmd.indexOf("!") == 0)) {
    if (isActivated) {
      cmd.remove(0, 1);
      if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {
        PRINTln("ASTRO Front camera continuous base is moving at rate: " + String(cmd.toInt()));
        frontBase.write(cmd.toInt());
      }
      else {
        PRINTln("ASTRO Front camera continuous base: choose values from 0 to 180");
      }
    }
    else {
      PRINTln("ASTRO Astro isn't activated yet!!!");
    }
  }
  else if ((cmd.indexOf("@") == 0)) {
    if (isActivated) {
      cmd.remove(0, 1);
      if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {
        PRINTln("ASTRO Front camera Side positional servo is moving to angle: " + String(cmd.toInt()));
        frontSide.write(cmd.toInt());
      }
      else {
        PRINTln("ASTRO Front camera Side positional servo: choose values from 0 to 180");
      }
    }
    else {
      PRINTln("ASTRO Astro isn't activated yet!!!");
    }
  }
  else if ((cmd.indexOf("#") == 0)) {
    if (isActivated) {
      cmd.remove(0, 1);
      if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {
        PRINTln("ASTRO Top camera continuous base is moving at rate: " + String(cmd.toInt()));
        topBase.write(cmd.toInt());
      }
      else {
        PRINTln("ASTRO Top camera continuous base: choose values from 0 to 180");
      }
    }
    else {
      PRINTln("ASTRO Astro isn't activated yet!!!");
    }
  }
  else if ((cmd.indexOf("$") == 0)) {
    if (isActivated) {
      cmd.remove(0, 1);
      if ( (0 <= cmd.toInt()) && (cmd.toInt() <= 180)) {
        PRINTln("ASTRO Top camera Side positional servo is moving to angle: " + String(cmd.toInt()));
        topSide.write(cmd.toInt());
      }
      else {
        PRINTln("ASTRO Top camera Side positional servo: choose values from 0 to 180");
      }
    }
    else {
      PRINTln("ASTRO Astro isn't activated yet!!!");
    }
  }
  else if (sender == s[2]) {
    PRINTln("ASTRO BOOOO! You used an unregistered button");
    PRINTln("ASTRO Here is a list of the registered ArduinoBlue buttons:");
    PRINTln("ASTRO Button id: 0 - Button function: activate rover");
    PRINTln("ASTRO Button id: 1 - Button function: deactivate");
    PRINTln("ASTRO Button id: 2 - Button function: turn on joystick control");
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
    PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    PRINTln("ASTRO Astro is inactive, activate to use joystick");
    PRINT("ASTRO If you're sending a command through Bluetooth ");
    PRINTln("serial app, then activate this mode first (btn id=3)");
  }
  else if (button == -3) {
    PRINTln("ASTRO Disable bluetooth serial to drive Rover");
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
      PRINTln("ASTRO Activate Joystick controls first");
    }
  }
  else {
    handler("wrong command", s[2]);
  }
}

void Commands::systemStatus(void) {
  PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
  PRINT("ASTRO Wheels: ");
  PRINTln(String((isActivated) ? "ACTIVE" : "INACTIVE"));
  PRINTln("ASTRO Ble-Mode: " + String((bluetoothMode) ? "ON" : "OFF"));
  PRINTln("ASTRO Control: " + String((isJoystickMode) ? "ArduinoBlue Joystick" : "Bluetooth Serial"));
  PRINTln("ASTRO Steering: " + String((isSteering) ? "Steering Control" : "Motor Control"));
  PRINTln("ASTRO Encoders: " + String((isEnc) ? "ON" : "OFF"));
  PRINT("ASTRO Nav " + (error ? "ERROR: " + errorMessage : "Success\n"));
  PRINTln("ASTRO Nav Stream: " + String((isGpsImu) ? "ON" : "OFF"));
  PRINT("ASTRO Motor loop statuses: ");
  for (int i = 0; i < 6; i++) { //6 is hardcoded, should be using a macro
    PRINT((motorList[i].isOpenLoop) ? "Open" : "CLose");
    if (i != 5) PRINT(", ");
  }
  PRINTln("");

  PRINT("ASTRO Motor accel: ");
  for (int i = 0; i < 6; i++) { //6 is hardcoded, should be using a macro
    PRINT((motorList[i].accLimit) ? "ON" : "OFF");
    if (i != 5) PRINT(", ");
  }
  PRINTln("");

  PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
}

void Commands::activate(String sender) {
  if (isActivated) {
    PRINTln("ASTRO Rover is Already ACTIVATED");
  }
  else if (!isActivated) {
    isActivated = true;
    if (sender == "Serial") {
      bluetoothMode = false;
    }
    PRINTln("ASTRO Rover Wheels are Active");
    PRINTln((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO BLE-Mode is OFF");
    //toggleLed2();
  }
}
void Commands::deactivate(String sender) {
  if (isActivated) {
    stop();
    isActivated = false;
    if (sender == "Serial") {
      bluetoothMode = true;
    }
    PRINTln("ASTRO Rover Wheels are Inactive");
    PRINTln((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO LE-Mode is OFF");
    //toggleLed2();
  }
  else if (!isActivated) {
    isActivated = false;
    if (sender == "Serial") {
      bluetoothMode = true;

    }
    PRINTln("ASTRO Rover Wheels are Inactive");
    PRINTln((bluetoothMode) ? "ASTRO BLE-Mode is ON" : "ASTRO BLE-Mode is OFF");
    //toggleLed2();
  }
}

void Commands::ping(void) {
  PRINTln("ASTRO pong");
}
void Commands::who(void) {
  if (isActivated) {
    PRINTln("ASTRO Happy Astro");
    //toggleLed2();
  }
  else if (!isActivated) {
    PRINTln("ASTRO Paralyzed Astro");
    //toggleLed2();
  }
}
void Commands::rebootTeensy(void) {
  PRINTln("ASTRO rebooting wheel teensy... hang on a sec");
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  // The next 2 lines set the time-out value.
  WDOG_TOVALL = 15; // This is the value (ms) that the watchdog timer compare itself to.
  WDOG_TOVALH = 0; // End value (ms) WDT compares itself to.
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
                  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
  WDOG_PRESC = 0; //Sets watchdog timer to tick at 1 kHz inseast of 1/4 kHz
  while (1); // infinite do nothing loop -- wait for the countdown
}

void Commands::bleOn(String sender) {
  if (isActivated && sender == "Serial") {
    stop();
    minInputSignal = -49;
    maxInputSignal = 49;
    bluetoothMode = true;
    PRINTln("ASTRO Bluetooth Control is ON");
  }
  else if (!isActivated && sender == "Serial") {
    minInputSignal = -49;
    maxInputSignal = 49;
    bluetoothMode = true;
    PRINTln("ASTRO Bluetooth Control is ON");
  }
}
void Commands::bleOff(String sender) {
  if (isActivated && sender == s[0]) {
    stop();
    //        minInputSignal = -49;
    //        maxInputSignal = 49;
    bluetoothMode = false;
    PRINTln("ASTRO Bluetooth Control is OFF");
  }
  else if (!isActivated && sender == s[0]) {
    //        minInputSignal = -49;
    //        maxInputSignal = 49;
    bluetoothMode = false;
    PRINTln("ASTRO Bluetooth Control is OFF");
  }
  else {
    PRINTln("You must deactivate from serial");
  }
}

void Commands::closeLoop(void) {
  if (isActivated) {
    stop();
    minOutputSignal = -30;
    maxOutputSignal = 30;
    PRINTln("Bo!");
    for (i = 0; i <= 5; i++) {
      motorList[i].isOpenLoop = false;
      String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open" : "CLose");
      PRINTln(msg);
    }
  }
  else if (!isActivated) {
    minOutputSignal = -30;
    maxOutputSignal = 30;
    PRINTln("ASTRO Bo!");
    for (i = 0; i <= 5; i++) {
      motorList[i].isOpenLoop = false;
      String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open" : "CLose");
      PRINTln(msg);
    }
  }
}
void Commands::openLoop(void) {
  if (isActivated) {
    stop();
    minOutputSignal = -255;
    maxOutputSignal = 255;
    for (i = 0; i <= 5; i++) {
      motorList[i].isOpenLoop = true;
      String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open" : "CLose");
      PRINTln(msg);
    }
  }
  else if (!isActivated) {
    minOutputSignal = -255;
    maxOutputSignal = 255;
    for (i = 0; i <= 5; i++) {
      motorList[i].isOpenLoop = true;
      String msg = "ASTRO Motor " + String(i) + String(" loop status is: ") + String((motorList[i].isOpenLoop) ? "Open" : "CLose");
      PRINTln(msg);
    }
  }
}

void Commands::joystickOn(void) {
  if (isActivated) {
    stop();
    isJoystickMode = true;
    PRINTln("ASTRO Joystick is active, don't use Serial over Bluetooth");
  }
  else if (!isActivated) {
    isJoystickMode = true;
    PRINTln("ASTRO Joystick is active, don't use Serial over Bluetooth");
  }

} // not needed for gui
void Commands::joystickOff(void) {
  if (isActivated) {
    stop();
    isJoystickMode = false;
    PRINTln("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
  }
  else if (!isActivated) {
    isJoystickMode = false;
    PRINTln("ASTRO ArduinoBlue Joystick is disabled, use serial Bluetooth ");
  }
  //    if (sender = s[1]){
  //        PRINTln("Please resend your command.");
  //
  //    }
} // not needed for gui

void Commands::steerOff(void) {
  if (isActivated ) {
    stop();
    isSteering = false;
    PRINTln("ASTRO Individual Wheel Control is Activated");
  }
  else if (!isActivated) {
    isSteering = false;
    PRINTln("ASTRO Individual Wheel Control is Activated");
  }
  if (isOpenloop) {
    PRINTln("ASTRO Speed range is -255 to 255");
    PRINTln("ASTRO motorNumber:PWM");
  }
  else if (!isOpenloop) {
    PRINTln("ASTRO Speed range is -30 to 30");
    PRINTln("ASTRO commands are now: motorNumber:RPM");
  }
}
void Commands::steerOn(void) {
  if (isActivated) {
    stop();
    isSteering = true;
    PRINTln("ASTRO Manual Skid Steering is Activated");
  }
  else if (!isActivated) {
    isSteering = true;
    PRINTln("ASTRO Manual Skid Steering is Activated");
  }
  if (isOpenloop) {
    PRINTln("ASTRO Speed range is -49 to 49");
    PRINTln("ASTRO commands are now: throttle:steering");
  }
  else if (!isOpenloop) {
    PRINTln("ASTRO Speed range is -30 to 30");
    PRINTln("ASTRO Steering range is -49 to 49");
    PRINTln("ASTRO throttle:steering");
  }
}

void Commands::gpsOff(void) {
  isGpsImu = false;
  PRINTln("GPS and IMU Serial Stream is now Disabled");
}
void Commands::gpsOn(void) {
  isGpsImu = true;
  PRINTln("ASTRO GPS and IMU Serial Stream is now Enabled");
}

void Commands::encOn(void) {
  if (isActivated) {
    isEnc = true;
    PRINTln("ASTRO Velocity Readings Stream from Motor Encoders is ON");
  }
  else if (!isActivated) {
    isEnc = true;
    PRINTln("ASTRO Motor Velocity Reading Stream is ON but will start printing values once the Rover is activated");
    for (i = 0; i < 6; i++) {
      PRINT("ASTRO Motor ");
      PRINT(i);
      PRINT(" current velocity: ");
      PRINTln(motorList[i].getCurrentVelocity());
      //delay(80);
    }
  }
}
void Commands::encOff(void) {
  isEnc = false;
  PRINTln("ASTRO Velocity Readings Stream from Motor Encoders is OFF");
}

void Commands::accOn(void) {
  for (i = 0; i <= 5; i++) {
    motorList[i].accLimit = true;
    String msg = "ASTRO Motor " + String(i) + " Acceleration Limiter: " + String((motorList[i].accLimit) ? "Open" : "CLose");
    PRINTln(msg);
  }
}
void Commands::accOff(void) {
  for (i = 0; i <= 5; i++) {
    motorList[i].accLimit = false;
    String msg = "ASTRO Motor " + String(i) + " Acceleration Limiter: " + String((motorList[i].accLimit) ? "Open" : "CLose");
    PRINTln(msg);
  }
}

void Commands::status(void) {
  systemStatus();
}

void Commands::stop(void) {
  velocityHandler(0, 0);
  for (int i = 0; i <= 5; i++) {
    PRINTln(String("ASTRO ") + String(motorList[i].motorName) + String("'s desired speed: ") + String(motorList[i].getCurrentVelocity()) + String(" PWM ") + String(motorList[i].direction));
  }
  //    PRINTln(String(motorList[i].motorName) + String("'s desired speed: ") + String(motorList[i].getDesiredVelocity()) + String(" PWM ") + String(motorList[i].direction));
}
void Commands::errorMsg(void) {
  PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
  PRINT("ASTRO " + String((error) ? "ERROR: " + String(errorMessage) : "No Error\n"));
  PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
}

#endif

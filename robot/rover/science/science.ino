#include <Arduino.h>
#include <Servo.h>
//#include <SoftwareSerial.h>

#define DEVEL_MODE_1       1     //Use with USB
//#define DEVEL_MODE_2       2   //Use with UART4

#if defined(DEVEL_MODE_1)
#define UART_PORT Serial
#elif defined(DEVEL_MODE_2)
#define UART_PORT Serial4
#endif

#define SERVO_STOP     1500
#define SERVO_MAX_CW   1250
#define SERVO_MAX_CCW  1750
#define TRIGGER_DELAY  50
#define FEED_CONSTANT  3.12 // in seconds/inch
//Multoplexer pins for to chose photoresistor
#define S0                  0
#define S1                  1
#define S2                  2
#define S3                  3
#define PHOTORESISTOR       4 //Mux_Out

#define PUMPS_LEGA         10
#define PUMP1_SPEED         9
#define PUMP2_SPEED         8
#define PUMP3_SPEED         7
#define PUMP4_SPEED         6
#define PUMP5_SPEED         5

#define TABLE_PIN          14 //Servo_speed

#define VIBRATOR1          18
#define VIBRATOR2          19
#define VIBRATOR3          20
#define VIBRATOR4          21
#define VIBRATOR5          22
#define VIBRATOR6          23

#define LIMIT_TOP          25 //limSW_1
#define LIMIT_BOTTOM       26 //limSW_2
#define TABLE_SWITCH_PIN   24 //limSW_3 

#define LED1               33
#define LED2               34

// MOTOR 2
// M2B - negative
// M2A - positive
#define DRILL              35//36 //PWM // 36 for TEENSY, 44 for Arduino Mega
#define DRILL_DIRECTION    36//35
// MOTOR 1
// M1A - negative
// M1B - positive
#define ELEVATOR           37//38 //PWM // 38 FOR TEENSY, 45 for Arduino Mega
#define ELEVATOR_DIRECTION 38//37

// debouncing variables
volatile bool isTriggered = false;
volatile bool isContacted = false;
bool isActualPress = false;
bool isPushed = false;
unsigned long triggerTime;

// other variables
bool isActivated = false;
bool turnTableFree = true;
bool deactivating = false;
bool drillInUse = false, elevatorInUse = false; // feedback for user
bool drillTimerInUse = false, elevatorTimerInUse = false; // for timing based commands
int drillDuration = 0;
int drillSpeed = 0; // in RPM
int drillDirection = 0; // 0 --> CW, 1 --> CCW
int drillSpeedPercent = 0;
int pumpDirection = 0; // 0 --> OUT, 1 --> IN
long int elevatorDuration = 0;
int elevatorDirection = 1 - 0; // 0 --> UP, 1 --> DOWN
int maxVelocity = 255;
int elevatorFeedPercent = 0;
int cuvette = 0;
int desiredPosition = 0;
int i = 0; // Used in all the for loops, don't worry no nested loops
int numberTablePositions = 8;
volatile int tablePosition[50];
float val = 0; // Photoresistor
float voltage = 0;  // Photoresistor
volatile char tableDirection = 'n'; // n for neutral, i for increasing, d for decreasing
volatile char previousElevatorState = 'n'; // n for neutral, u for up, d for down
unsigned long homingTimer;
unsigned long lastPrintTime;
unsigned long drillTimer;
unsigned long elevatorTimer;
int semiStep = 0;
bool tcwstepDone = false;
bool tccwstepDone = false;
int currentTablePosition = 0;

// forward declarations
int drill_speed(int input_drill_speed);//takes percentage returns RPM, max 165RPM
float elevator_feed(int input_elevator_feed);//takes percentage returns inch/s, max 0.107inch/s
void elevatorTopInterrupt (void); //limit switch
void elevatorBottomInterrupt (void); //limit switch
void cuvettePosition (void); //tracks the position of the cuvettes 0-25 relative to the table positions 0-25
void turnTable (int cuvette, int desiredPosition);// sends the wanted cuvette to the wanted position
void debouncing(void); //pure magic
float photoChoice(int led); // choses the photoresistor from which to read the voltage
String getValue(String data, char separator, int index);//splits input string to feed multiple variables

Servo table;

void setup() {

  UART_PORT.begin(115200); UART_PORT.setTimeout(10);
  // blink adn stay ON to signal this is indeed the science MCU and not another one
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // THIS MESSES UP PWM PINS 45/44 AND POSSIBLY OTHERS
  table.attach(TABLE_PIN);
  pinMode(DRILL, OUTPUT);
  pinMode(DRILL_DIRECTION, OUTPUT);
  analogWrite(DRILL, 35);
  pinMode(ELEVATOR, OUTPUT);
  pinMode(ELEVATOR_DIRECTION, OUTPUT);
  //photoresistor choice
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  //
  pinMode(PUMP1_SPEED, OUTPUT);
  pinMode(PUMP2_SPEED, OUTPUT);
  pinMode(PUMP3_SPEED, OUTPUT);
  pinMode(PUMP4_SPEED, OUTPUT);
  pinMode(PUMP5_SPEED, OUTPUT);
  pinMode(PUMPS_LEGA, OUTPUT);
  pinMode(VIBRATOR1, OUTPUT);
  pinMode(VIBRATOR2, OUTPUT);
  pinMode(VIBRATOR3, OUTPUT);
  pinMode(VIBRATOR4, OUTPUT);
  pinMode(VIBRATOR5, OUTPUT);
  pinMode(VIBRATOR6, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LIMIT_TOP, INPUT_PULLUP);
  pinMode(LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(PHOTORESISTOR, INPUT_PULLUP);
  pinMode(TABLE_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_TOP), debouncing, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_BOTTOM), debouncing, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TABLE_SWITCH_PIN), debouncing, CHANGE);

  analogWrite(DRILL, 0);
  analogWrite(ELEVATOR, 0);
  digitalWrite(DRILL_DIRECTION, drillDirection); // init LOW
  digitalWrite(ELEVATOR_DIRECTION, LOW);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  digitalWrite(PUMP1_SPEED, LOW);
  digitalWrite(PUMP2_SPEED, LOW);
  digitalWrite(PUMP3_SPEED, LOW);
  digitalWrite(PUMP4_SPEED, LOW);
  digitalWrite(PUMP5_SPEED, LOW);
  digitalWrite(PUMPS_LEGA, LOW);
  digitalWrite(VIBRATOR1, LOW);
  digitalWrite(VIBRATOR2, LOW);
  digitalWrite(VIBRATOR3, LOW);
  digitalWrite(VIBRATOR4, LOW);
  digitalWrite(VIBRATOR5, LOW);
  digitalWrite(VIBRATOR6, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  table.writeMicroseconds(SERVO_STOP);

  for (i = 0; i < numberTablePositions; i++) {
    tablePosition[i] = i;
  }

  //delay(1000); // commenting out because causing issues with ScienceNode initSerial()

  UART_PORT.println("\nsetup complete");
}

void loop() {
  // print relevant data for pub/sub nodes
  if (millis() - lastPrintTime > 750) {
    // update variables

    lastPrintTime = millis();
    UART_PORT.print("SCIENCE Science data:");
    UART_PORT.print("isActivated:"); UART_PORT.print(isActivated); UART_PORT.print(",");
    UART_PORT.print("drillDirection:"); UART_PORT.print(drillDirection); UART_PORT.print(",");
    UART_PORT.print("elevatorDirection:"); UART_PORT.print(elevatorDirection); UART_PORT.print(",");
    UART_PORT.print("pumpDirection:"); UART_PORT.print(pumpDirection); UART_PORT.print(",");
    UART_PORT.print("photoResistorVoltage:"); UART_PORT.print(voltage); UART_PORT.print(",");
    UART_PORT.print("LED1_ON:"); UART_PORT.print(digitalRead(LED1)); UART_PORT.print(",");
    UART_PORT.print("LED2_ON:"); UART_PORT.print(digitalRead(LED2)); UART_PORT.print(",");
    UART_PORT.print("v1:"); UART_PORT.print(digitalRead(VIBRATOR1)); UART_PORT.print(",");
    UART_PORT.print("v2:"); UART_PORT.print(digitalRead(VIBRATOR2)); UART_PORT.print(",");
    UART_PORT.print("v3:"); UART_PORT.print(digitalRead(VIBRATOR3)); UART_PORT.print(",");
    UART_PORT.print("v4:"); UART_PORT.print(digitalRead(VIBRATOR4)); UART_PORT.print(",");
    UART_PORT.print("v5:"); UART_PORT.print(digitalRead(VIBRATOR5)); UART_PORT.print(",");
    UART_PORT.print("v6:"); UART_PORT.print(digitalRead(VIBRATOR6)); UART_PORT.print(",");
    UART_PORT.print("drillSpeed:"); UART_PORT.print(drillSpeed); UART_PORT.print(",");
    UART_PORT.print("drillInUse:"); UART_PORT.print(drillInUse); UART_PORT.print(",");
    UART_PORT.print("elevatorInUse:"); UART_PORT.print(elevatorInUse); UART_PORT.print(",");
    UART_PORT.print("tcwstepDone:"); UART_PORT.print(tcwstepDone); UART_PORT.print(",");
    UART_PORT.print("tccwstepDone:"); UART_PORT.print(tccwstepDone); UART_PORT.print(",");
    UART_PORT.print("elevatorFeedPercent:"); UART_PORT.print(elevatorFeedPercent); UART_PORT.print(",");
    UART_PORT.print("currentTablePosition:"); UART_PORT.print(currentTablePosition);
    UART_PORT.println();

    // make sure to immediately set back to false for only one rotation in gui turntable
    if (tccwstepDone) {
      tccwstepDone = false;
    }
    if (tcwstepDone) {
      tcwstepDone = false;
    }
  }

  // timed stop for drill time
  if (drillTimerInUse == true && (millis() - drillTimer >= drillDuration)) {
    //stops drill
    analogWrite(DRILL, 0);
    drillTimerInUse = false;
    drillInUse = false;
    drillSpeed = drillSpeedPercent = 0;
    UART_PORT.println("SCIENCE drilltime done");
  }

  // timed stop for elevator distance
  if (elevatorTimerInUse == true && (millis() - elevatorTimer >= elevatorDuration)) {
    //stops elevator
    analogWrite(ELEVATOR, 0);
    previousElevatorState = 'n';
    elevatorTimerInUse = false;
    elevatorInUse = false;
    elevatorFeedPercent = 0;
    //UART_PORT.println("SCIENCE es done josh");
    UART_PORT.print("elevatorTimerInUse: ");
    UART_PORT.println(elevatorTimerInUse);
  }

  if (UART_PORT.available()) {
    String cmd = UART_PORT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "ping") {
      UART_PORT.println("SCIENCE pong");
    }
    if (cmd == "who") {
      UART_PORT.println("SCIENCE science");
    }
    if (cmd == "active") { // query the status
      UART_PORT.print("SCIENCE activated");
      UART_PORT.println(isActivated);
    }
    if (isActivated == false) {

      if (cmd == "activate") {
        isActivated = true;
        UART_PORT.println("SCIENCE activated");
      }
    }
    else if (isActivated == true) {

      if (cmd == "active") { // query the status
        UART_PORT.print("SCIENCE activated");
        UART_PORT.println(isActivated);
      }
      else if (cmd.startsWith("drillspeed") && (cmd.indexOf(" ") > 0)) {
        //turns drill at desired speed
        // needs input "drillspeed 100"
        drillSpeedPercent = getValue(cmd, ' ', 1).toInt();
        drillSpeed = drill_speed(drillSpeedPercent);
        int analogVal = drillSpeedPercent * 255 / 100;
        analogWrite(DRILL, analogVal);
        drillInUse = true;
        UART_PORT.println("SCIENCE drillspeed done");
        drillTimer = millis();
      }
      else if (cmd.startsWith("drilltime") && (cmd.indexOf(" ") > 0)) {
        //turns drill for desired period of time
        // needs input "drilltime 100"
        drillDuration = 1000 * (getValue(cmd, ' ', 1).toInt());
        int analogVal = maxVelocity;

        if (getCount(cmd, ' ') == 2) {
          // get desired drill speed
          int drillSpeedPercent = getValue(cmd, ' ', 2).toInt();
          analogVal = drillSpeedPercent * 255 / 100;
          drillSpeed = drill_speed(drillSpeedPercent);
        } else {
          drillSpeed = drill_speed(100);
        }

        analogWrite(DRILL, analogVal);
        UART_PORT.println("SCIENCE drilltime done");
        drillTimer = millis();
        drillInUse = true;
        drillTimerInUse = true;
      }
      else if (cmd == "dccw") {
        //turns drill counter-clockwise
        drillDirection = 1;
        digitalWrite(DRILL_DIRECTION, drillDirection);
        UART_PORT.println("SCIENCE dccw done");
      }
      else if (cmd == "dcw") {
        //turns drill clockwise
        drillDirection = 0;
        digitalWrite(DRILL_DIRECTION, drillDirection);
        UART_PORT.println("SCIENCE dcw done");
      }
      else if (cmd == "dd") { // drill direction query
        UART_PORT.print("SCIENCE ");
        UART_PORT.println((drillDirection) ? "CCW" : "CW");
      }
      else if (cmd == "dgo") {
        //turns drill in whatever direction is currently set
        analogWrite(DRILL, maxVelocity);
        drillInUse = true;
        drillSpeed = drill_speed(maxVelocity);
        drillSpeedPercent = 100;
        UART_PORT.println("SCIENCE dgo done");
      }
      else if (cmd == "ds") {
        //stops drill
        analogWrite(DRILL, 0);
        drillInUse = false;
        drillSpeed = drillSpeedPercent = 0;
        UART_PORT.println("SCIENCE ds done");
      }
      if (cmd.startsWith("ef") && (cmd.indexOf(" ") > 0)) {
        //turns elevator at desired feed
        // needs input "elevatorfeed 100"
        elevatorFeedPercent = getValue(cmd, ' ', 1).toInt();
        analogWrite(ELEVATOR, elevatorFeedPercent * 255 / 100);
        elevatorInUse = true  ;
        UART_PORT.println("SCIENCE elevatorfeed done");
        //@TODO: include this in the feedback, figure out which units (inches per s/min)
        //UART_PORT.println(elevator_feed(elevatorFeedPercent));
        //@TODO: figure out why this is useful to keep track of
        if (digitalRead(ELEVATOR_DIRECTION) == HIGH)previousElevatorState = 'd';
        else if (digitalRead(ELEVATOR_DIRECTION) == LOW)previousElevatorState = 'u';
      }
      if (cmd.startsWith("elevatordistance") && (cmd.indexOf(" ") > 0)) {
        // turns elevator for desired distance
        // needs input "elevatordistance 100"
        int analogVal = maxVelocity;
        double elevatorFeed = 0;

        if (getCount(cmd, ' ') == 2) {
          // get desired drill speed
          elevatorFeedPercent = getValue(cmd, ' ', 2).toInt();
          analogVal = elevatorFeedPercent * 255 / 100;
          elevatorFeed = elevator_feed(elevatorFeedPercent);
          UART_PORT.print("elevatorFeed: ");
          UART_PORT.println(elevatorFeed);
        } else {
          elevatorFeed = elevator_feed(100);
        }
        elevatorDuration = elevatorFeed * 1000 * (getValue(cmd, ' ', 1).toInt());
        UART_PORT.print("ELEVATOR DURATION: ");
        UART_PORT.println(elevatorDuration);
        analogWrite(ELEVATOR, analogVal);
        UART_PORT.println("SCIENCE elevatordistance done");
        elevatorTimer = millis();
        elevatorTimerInUse = true;
        elevatorInUse = true;
      }
      else if (cmd == "eup") {
        // sets elevator direction clockwise
        elevatorDirection = 1- 0;
        digitalWrite(ELEVATOR_DIRECTION, 1 - elevatorDirection);
        UART_PORT.println("SCIENCE eup done");
      }
      else if (cmd == "edown") {
        // sets elevator direction counter-clockwise
        elevatorDirection = 1 - 1;
        digitalWrite(ELEVATOR_DIRECTION, 1 - elevatorDirection);
        UART_PORT.println("SCIENCE edown done");
      }
      else if (cmd == "ed") {
        UART_PORT.print("SCIENCE ");
        UART_PORT.println((elevatorDirection) ? "DOWN" : "UP");
      }
      else if (cmd == "ego") {
        analogWrite(ELEVATOR, maxVelocity);
        elevatorInUse = true;

        if (digitalRead(ELEVATOR_DIRECTION) == HIGH) {
          previousElevatorState = 'd';
        }
        else if (digitalRead(ELEVATOR_DIRECTION) == LOW) {
          previousElevatorState = 'u';
        }

        UART_PORT.println("SCIENCE ego done");
      }
      else if (cmd == "es") {
        //stops elevator
        analogWrite(ELEVATOR, 0);
        elevatorInUse == false;
        previousElevatorState = 'n';
        elevatorFeedPercent = 0;
        UART_PORT.println("SCIENCE es done");
      }
      else if (cmd.endsWith("goto") && (cmd.indexOf(" ") > 0)) {
        //sends table to wanted position

        cuvette = (getValue(cmd, ' ', 0).toInt()) * 2; //multiplied by 2 only for ERC because there are 8 actual positions but only 4 needed
        UART_PORT.print("SCIENCE cuvette ");
        UART_PORT.println(cuvette);

        desiredPosition = (getValue(cmd, ' ', 1).toInt()) * 2; //multiplied by 2 only for ERC because there are 8 actual positions but only 4 needed
        UART_PORT.println(desiredPosition);

        if (cuvette >= numberTablePositions || cuvette < 0) {
          UART_PORT.print("Error. Chose cuvette number from 0 to ");
          UART_PORT.println((numberTablePositions - 1) / 2); //divided by 2 only for ERC because there are 8 actual positions but only 4 needed
        }
        if (desiredPosition >= numberTablePositions || desiredPosition < 0) {
          UART_PORT.print("Error. Chose position number from 0 to ");
          UART_PORT.println((numberTablePositions - 1) / 2); //divided by 2 only for ERC because there are 8 actual positions but only 4 needed
        }
        else if ((cuvette < numberTablePositions && cuvette >= 0) && (desiredPosition < numberTablePositions && desiredPosition >= 0)) {
          turnTable (cuvette, desiredPosition);
        }
      }
      else if (cmd == "tccw") {
        //turns table counter-clockwise
        table.writeMicroseconds(SERVO_STOP);
        delay(100);
        table.writeMicroseconds(SERVO_MAX_CCW);
        tableDirection = 'i';
        UART_PORT.println("SCIENCE tccw");
      }
      else if (cmd == "tcw") {
        //turns table clockwise
        table.writeMicroseconds(SERVO_STOP);
        delay(100);
        table.writeMicroseconds(SERVO_MAX_CW);
        tableDirection = 'd';
        UART_PORT.println("SCIENCE tcw");
      }
      else if (cmd == "tccwstep") {
        //turns table counter-clockwise
        table.writeMicroseconds(SERVO_STOP);
        delay(10);
        cuvette = tablePosition[2];
        desiredPosition = 0;
        turnTable(cuvette, desiredPosition);
        // let it be known that roation is initiated
        UART_PORT.println("SCIENCE tccwstep done");
      }
      else if (cmd == "tcwstep") {
        //turns table clockwise
        table.writeMicroseconds(SERVO_STOP);
        delay(10);
        cuvette = tablePosition[0];
        desiredPosition = 2;
        turnTable(cuvette, desiredPosition);
        // let it be known that roation is initiated
        UART_PORT.println("SCIENCE tcwstep done");
      }
      else if (cmd == "trefresh") { // sets whatever position facing the drill to zero and numbers increasingly clockwise
        //stops table
        for (i = 0; i < numberTablePositions; i++) {
          tablePosition[i] = i;
        }
        UART_PORT.print("SCIENCE tablePosition[0]");
        UART_PORT.println(tablePosition[0]);
        UART_PORT.println("SCIENCE trefresh");
      }
      else if (cmd == "ts") {
        //stops table
        table.writeMicroseconds(SERVO_STOP);
        tableDirection = 'n';
        turnTableFree = true;
        UART_PORT.println("SCIENCE ts");
      }
      else if (cmd == "pd0") {
        pumpDirection = 0;
        digitalWrite(PUMPS_LEGA, pumpDirection);
        UART_PORT.println("SCIENCE OUT");
      }
      else if (cmd == "pd1") {
        pumpDirection = 1;
        digitalWrite(PUMPS_LEGA, pumpDirection);
        UART_PORT.println("SCIENCE IN");
      }
      else if (cmd == "p1") {
        // actuate pump 1
        digitalWrite(PUMP1_SPEED, HIGH);
        UART_PORT.println("SCIENCE p1 done");
      }
      else if (cmd == "p2") {
        // actuate pump 2
        digitalWrite(PUMP1_SPEED, HIGH);
        UART_PORT.println("SCIENCE p2 done");
      }
      else if (cmd == "p3") {
        // actuate pump 3
        digitalWrite(PUMP2_SPEED, HIGH);
        UART_PORT.println("SCIENCE p3 done");
      }
      else if (cmd == "p4") {
        // actuate pump 4
        digitalWrite(PUMP2_SPEED, HIGH);
        UART_PORT.println("SCIENCE p4 done");
      }
      else if (cmd == "p5") {
        // actuate pump 5;
        digitalWrite(PUMP3_SPEED, HIGH);
        UART_PORT.println("SCIENCE p5 done");
      }
      else if (cmd == "ps") {
        // stop all pumps
        digitalWrite(PUMP1_SPEED, LOW);
        digitalWrite(PUMP2_SPEED, LOW);
        digitalWrite(PUMP3_SPEED, LOW);
        digitalWrite(PUMP4_SPEED, LOW);
        digitalWrite(PUMP5_SPEED, LOW);
        digitalWrite(PUMPS_LEGA, LOW);
        UART_PORT.println("SCIENCE ps done");
      }
      else if (cmd == "v1") {
        digitalWrite(VIBRATOR1, HIGH);
        UART_PORT.println("SCIENCE v1 done");
      }
      else if (cmd == "v2") {
        digitalWrite(VIBRATOR2, HIGH);
        UART_PORT.println("SCIENCE v2 done");
      }
      else if (cmd == "v3") {
        digitalWrite(VIBRATOR3, HIGH);
        UART_PORT.println("SCIENCE v3 done");
      }
      else if (cmd == "v4") {
        digitalWrite(VIBRATOR4, HIGH);
        UART_PORT.println("SCIENCE v4 done");
      }
      else if (cmd == "v5") {
        digitalWrite(VIBRATOR5, HIGH);
        UART_PORT.println("SCIENCE v5 done");
      }
      else if (cmd == "v6") {
        digitalWrite(VIBRATOR6, HIGH);
        UART_PORT.println("SCIENCE v6 done");
      }
      else if (cmd == "vs") {
        digitalWrite(VIBRATOR1, LOW);
        digitalWrite(VIBRATOR2, LOW);
        digitalWrite(VIBRATOR3, LOW);
        digitalWrite(VIBRATOR4, LOW);
        digitalWrite(VIBRATOR5, LOW);
        digitalWrite(VIBRATOR6, LOW);
        UART_PORT.println("SCIENCE vs done");
      }
      else if (cmd == "led1") {
        digitalWrite(LED1, HIGH);
        photoChoice(1);
        //delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);

        UART_PORT.println("SCIENCE led1 done");
      }
      else if (cmd == "led1s") {
        digitalWrite(LED1, LOW);
        photoChoice(1);
        //delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);
        digitalWrite(S0, 0);
        digitalWrite(S1, 0);
        digitalWrite(S2, 0);
        digitalWrite(S3, 0);
        UART_PORT.println("SCIENCE led1s done");
      }
      else if (cmd == "led2") {
        digitalWrite(LED2, HIGH);
        photoChoice(2);
        //delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);

        UART_PORT.println("SCIENCE led2 done");
      }
      else if (cmd == "led2s") {
        digitalWrite(LED2, LOW);
        photoChoice(2);
        //delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);
        digitalWrite(S0, 0);
        digitalWrite(S1, 0);
        digitalWrite(S2, 0);
        digitalWrite(S3, 0);

        UART_PORT.println("SCIENCE led2s done");
      }

      else if (cmd == "deactivate") {
        //stops all
        //        UART_PORT.print("cmd: ");
        //        UART_PORT.println(cmd);
        //        UART_PORT.print("Homing Table");
        homingTimer = millis();
        cuvette = 0;
        desiredPosition = 0;
        turnTable (cuvette, desiredPosition);
        deactivating = true;
        UART_PORT.println("SCIENCE deactivated");
      }
      if (cmd == "stop" || (deactivating == true && (millis() - homingTimer > 30000))) {
        analogWrite(ELEVATOR, 0);
        analogWrite(DRILL, 0);
        digitalWrite(PUMP1_SPEED, LOW);
        digitalWrite(PUMP2_SPEED, LOW);
        digitalWrite(PUMP3_SPEED, LOW);
        digitalWrite(PUMP4_SPEED, LOW);
        digitalWrite(PUMP5_SPEED, LOW);
        digitalWrite(PUMPS_LEGA, LOW);
        digitalWrite(VIBRATOR1, LOW);
        digitalWrite(VIBRATOR2, LOW);
        digitalWrite(VIBRATOR3, LOW);
        digitalWrite(VIBRATOR4, LOW);
        digitalWrite(VIBRATOR5, LOW);
        digitalWrite(VIBRATOR6, LOW);
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(S0, 0);
        digitalWrite(S1, 0);
        digitalWrite(S2, 0);
        digitalWrite(S3, 0);
        table.writeMicroseconds(SERVO_STOP);
        tableDirection = 'n';
        turnTableFree = true;
        previousElevatorState = 'n';
        isActivated = false;
        deactivating = false;
        UART_PORT.println("SCIENCE stopped");
      }
    }
  }
  // this works but doesn't consider the possibility of both switches
  // being triggered at the same time which shouldn't ever actually happen
  if (isTriggered) {
    if ((millis() - triggerTime) >= TRIGGER_DELAY) {
      // if the last interrupt was a press (meaning it's stabilized and in contact)
      // then there's a real press
      if (isContacted) {
        isActualPress = true;   // otherwise it's not a real press
        //triggerTime = 0;
      }                         // so the limit switch state should stay whatever it used to be
      // and so should actualPress
      isTriggered = false;      // either way, we should reset the triggered bool in wait for the next trigger
    }
  }
  if (isActualPress) {
    if (previousElevatorState == 'n')cuvettePosition();
    else if (previousElevatorState == 'u')elevatorTopInterrupt();
    else if (previousElevatorState == 'd')elevatorBottomInterrupt();
    // now that the behaviour is complete we can reset these in wait for the next trigger to be confirmed
    isActualPress = false;

    if ((turnTableFree == false) && (tablePosition[desiredPosition] == cuvette)) {
      table.writeMicroseconds(SERVO_STOP);
      tableDirection = 'n';
      turnTableFree = true;
    }
  }
}

// elevatorDirection
// 0 --> UP, 1 --> DOWN

/*
  NOTE: It is bad practice to have interrupts which are not super quick
  These interrupts ideally should only be setting flags and the main code
  should be dealing with causing delays and whatnot
*/
void elevatorTopInterrupt () {
  //stops elevator
  unsigned long timer = millis();

  analogWrite(ELEVATOR, 0);
  digitalWrite(ELEVATOR_DIRECTION, HIGH);
  //previousElevatorState = 'd';
  analogWrite(ELEVATOR, maxVelocity);
  while ((millis() - timer) < 500) {
    ;
  }
  analogWrite(ELEVATOR, 0);
  previousElevatorState = 'n';
  elevatorDirection = 1 - 1;
}

void elevatorBottomInterrupt () {
  //stops elevator
  unsigned long timer = millis();

  analogWrite(ELEVATOR, 0);
  digitalWrite(ELEVATOR_DIRECTION, LOW);
  //previousElevatorState = 'u';
  analogWrite(ELEVATOR, maxVelocity);
  while ((millis() - timer) < 500) {
    ;
  }
  analogWrite(ELEVATOR, 0);
  previousElevatorState = 'n';
  elevatorDirection = 1- 0;
}

void cuvettePosition() {
  // gives the integer value of the cuvette of the table 1 to "numberTablePositions" , cuvettes are only on even numbers, chute is cuvettePosition 0
  if (tableDirection == 'n') {
  }
  else if (tableDirection == 'i') {
    for (i = 0; i < (numberTablePositions - 1); i++) {
      tablePosition[i] = (tablePosition[i] + 1) % numberTablePositions;
    }
    semiStep++;

    if (semiStep == 2) {
      // reset
      semiStep = 0;
      // for gui to turn table accordingly
      tccwstepDone = true;
      // update current table position feedback variable
      currentTablePosition = tablePosition[0] / 2;
    }
  }
  else if (tableDirection == 'd') {
    int temp = tablePosition[0];
    for (i = 0; i <= (numberTablePositions - 1); i++) {
      tablePosition[i] = (tablePosition[i] - 1) % numberTablePositions;
      if (tablePosition[i] == -1) {
        tablePosition[i] = (numberTablePositions - 1);
      }
    }
    semiStep++;

    if (semiStep == 2) {
      // reset
      semiStep = 0;
      // for gui to turn table accordingly
      tcwstepDone = true;
      // update current table position feedback variable
      currentTablePosition = tablePosition[0] / 2;
    }
  }
}

void turnTable (int cuvette, int desiredPosition) {
  int initialPosition = 0;
  int difference = 0;

  for (i = 0; i <= (numberTablePositions - 1); i++) {
    initialPosition = i;
    if (tablePosition[i] == cuvette)break;
  }

  UART_PORT.print("initialPosition");
  UART_PORT.println(initialPosition);

  difference = desiredPosition - initialPosition;

  UART_PORT.print("difference: ");
  UART_PORT.println(difference);

  if ( (difference >= -(numberTablePositions / 2) && difference < 0) || (difference > (numberTablePositions / 2) && difference < numberTablePositions)) {
    tableDirection = 'i';
    table.writeMicroseconds(SERVO_MAX_CCW);
  }
  else if ( (difference > -numberTablePositions && difference < -(numberTablePositions / 2)) || (difference > 0 && difference <= (numberTablePositions / 2))) {
    tableDirection = 'd';
    table.writeMicroseconds(SERVO_MAX_CW);
  }
  turnTableFree = false;
}

void debouncing(void) {
  // if this is the first time the switch was pressed in a while,
  // alert loop() that the switch was pressed and set up the timer
  if (!isTriggered && !isContacted) {
    isTriggered = true;
    triggerTime = millis();
  }

  /* every interrupt, update isContacted based on the pin state */
  // if the contact is connected, the pin will read low so set isContacted to true
  if (previousElevatorState == 'n') {
    if (digitalRead(TABLE_SWITCH_PIN) == LOW) {
      isContacted = true;
    }
    // otherwise the contact is bouncing so set it to false
    else if (digitalRead(TABLE_SWITCH_PIN) == HIGH) {
      isContacted = false;
    }
  }
  else if (previousElevatorState == 'u') {
    if (digitalRead(LIMIT_TOP) == LOW) {
      isContacted = true;
    }
    // otherwise the contact is bouncing so set it to false
    else if (digitalRead(LIMIT_TOP) == HIGH) {
      isContacted = false;
    }
  }
  else if (previousElevatorState == 'd') {
    if (digitalRead(LIMIT_BOTTOM) == LOW) {
      isContacted = true;
    }
    // otherwise the contact is bouncing so set it to false
    else if (digitalRead(LIMIT_BOTTOM) == HIGH) {
      isContacted = false;
    }
  }
}

/*
   Defaults input values to 0 or 100 if out of those bounds
   Converts percentage value into estimate RPM
*/
int drill_speed(int input_drill_speed) {
  if (input_drill_speed < 0) {
    input_drill_speed = 0;
  }
  if (input_drill_speed > 100) {
    input_drill_speed = 100;
  }
  return (input_drill_speed * 165 / 100);
}

float elevator_feed(int input_elevator_feed) {
  if (input_elevator_feed < 15) { //Below 15% the equation is not reliable
    input_elevator_feed = 0;     //and the motor has a tendancy to stall
  }
  if (input_elevator_feed > 100) {
    input_elevator_feed = 100;
  }

  //  double correctedDuration = FEED_CONSTANT * (1 / ( 0.0000356 * pow(input_elevator_feed,2) + 0.00644 * input_elevator_feed));
  //  double correctedDuration = FEED_CONSTANT * (1/(-0.00000130911*pow(input_elevator_feed,3)+0.000220708*pow(input_elevator_feed,2)+0.0010203*input_elevator_feed));
  double correctedDuration = FEED_CONSTANT * (3 / (-0.00000010682 * pow(input_elevator_feed, 4) + 0.0000215 * pow(input_elevator_feed, 3) - 0.00134 * pow(input_elevator_feed, 2) + 0.06 * input_elevator_feed - 0.418));
  // double correctedDuration = FEED_CONSTANT;
  UART_PORT.print("correctedDuration: ");
  UART_PORT.println(correctedDuration);

  return correctedDuration;
}

float photoChoice(int led) {
  //choses which photoresistor to read
  int binary[5] = {0};
  int n = 0;
  int photoNumber = 0;
  if (led == 1) {
    photoNumber = (tablePosition[9]); //position of led1 is 9
  }
  else if (led == 2) {
    photoNumber = tablePosition[11]; //position of led2 is 11
  }
  n = photoNumber / 2 ;// divided by 2 because photoresistors numbered 1-12 and cuvettes 2-24

  for (i = 0; n > 0; i++) {
    binary[i] = n % 2;
    n = n / 2;
  }
  digitalWrite(S0, binary[0]);
  digitalWrite(S1, binary[1]);
  digitalWrite(S2, binary[2]);
  digitalWrite(S3, binary[3]);
}

/**
   Tokenizes strings and returns section by index

   Usage: getValue("hello_robotics_world", '_', 2)
   This will return the string "world"
*/
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
   Returns the amount of times the target character appears in a given string
*/
int getCount(String data, char target) {
  int ctr = 0, i = 0;

  while (data[i] != '\0') {
    if (data[i] == target) {
      ctr++;
    }
    i++;
  }

  return ctr;
}

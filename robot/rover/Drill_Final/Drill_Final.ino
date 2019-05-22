#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define SERVO_STOP     1500
#define SERVO_MAX_CW   1250
#define SERVO_MAX_CCW  1750
#define TRIGGER_DELAY  50

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

#define LIMIT_TOP          24 //limSW_1
#define LIMIT_BOTTOM       25 //limSW_2
#define TABLE_SWITCH_PIN   26 //limSW_3

#define LED1               33
#define LED2               34

#define DRILL              36
#define DRILL_DIRECTION    35
#define ELEVATOR           38
#define ELEVATOR_DIRECTION 37

//debouncing variables
volatile bool isTriggered = false;
volatile bool isContacted = false;
bool isActualPress = false;
bool isPushed = false;
unsigned long triggerTime;

//Other variables
bool isActivated = false;
bool turnTableFree = true;
bool elevatorInUse = false;

int maxVelocity = 255;
int drillSpeed;
int elevatorSpeed;
int cuvette = 0;
int desiredPosition = 0;
int i = 0;
float val = 0;
float voltage = 0;
volatile int tablePosition[26];
volatile char tableDirection = 'n'; // n for neutral, i for increasing, d for decreasing
volatile char previousElevatorState = 'n'; // n for neutral, u for up, d for down

//Functions

int drill_speed(int input_drill_speed);//max 165RPM
float elevator_feed(int input_elevator_feed);//max 0.107inch/s
void elevatorTopInterrupt (void);
void elevatorBottomInterrupt (void);
void cuvettePosition (void);
void turnTable (int cuvette, int desiredPosition);
void debouncing(void);
float photoChoice(int led);

Servo table;

void setup() {

  Serial.begin(115200); Serial.setTimeout(10);

  table.attach(TABLE_PIN);
  pinMode(DRILL, OUTPUT);
  pinMode(DRILL_DIRECTION, OUTPUT);
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
  attachInterrupt(digitalPinToInterrupt(LIMIT_TOP), debouncing, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_BOTTOM), debouncing, FALLING);
  attachInterrupt(digitalPinToInterrupt(TABLE_SWITCH_PIN), debouncing, CHANGE);

  analogWrite(DRILL, 0);
  analogWrite(ELEVATOR, 0);
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

  for (i = 0; i <= 12; i++) {
    tablePosition[i] = i;
  }

  delay(1000);

  Serial.print("\nsetup complete");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (isActivated == false) {

      Serial.print("cmd: ");
      Serial.println(cmd);

    }
    if (cmd == "activate") {
      isActivated = true;
    }
    else if (cmd == "who") {
      Serial.println("drill");
    }
    else if (isActivated == true) {

      if (cmd == "drill") {
        //turns drill at desired speed
        int drillDigiDirection = 0;
        int drillSpeedPercent = 0;
        Serial.println("What is the desired drill direction? CCW=1 CW=0");
        while (!Serial.available()) {
          ;
        }
        drillDigiDirection = (Serial.readStringUntil('\n')).toInt();
        Serial.println(drillDigiDirection);

        Serial.println("What percentage of max speed?");
        while (!Serial.available()) {
          ;
        }
        drillSpeedPercent = (Serial.readStringUntil('\n')).toInt();

        Serial.println(drill_speed(drillSpeedPercent));
        Serial.println("RPM\n");
        if (drillDigiDirection == 1)Serial.println("CCW\n");
        else if (drillDigiDirection == 0)Serial.println("CW\n");
        analogWrite(DRILL, 0);
        delay(50);
        digitalWrite(DRILL_DIRECTION, drillDigiDirection);
        analogWrite(DRILL, drillSpeedPercent * 255 / 100);
      }
      if (cmd == "dccw") {
        //turns drill counter-clockwise
        analogWrite(DRILL, 0);
        delay(100);
        digitalWrite(DRILL_DIRECTION, HIGH);
        analogWrite(DRILL, maxVelocity);
        Serial.println("dccw");
      }
      else if (cmd == "dcw") {
        //turns drill clockwise
        analogWrite(DRILL, 0);
        delay(100);
        digitalWrite(DRILL_DIRECTION, LOW);
        analogWrite(DRILL, maxVelocity);
        Serial.println("dcw");
      }
      else if (cmd == "ds") {
        //stops drill
        analogWrite(DRILL, 0);
        Serial.println("ds");
      }
      if (cmd == "elevator") {
        //turns drill at desired speed
        int elevatorDigiDirection = 0;
        int elevatorFeedPercent = 0;
        Serial.println("What is the desired elevator direction? Up=1 Down=0");
        while (!Serial.available()) {
          ;
        }
        elevatorDigiDirection = (Serial.readStringUntil('\n')).toInt();
        Serial.println(elevatorDigiDirection);

        Serial.println("What percentage of max feed?");
        while (!Serial.available()) {
          ;
        }
        elevatorFeedPercent = (Serial.readStringUntil('\n')).toInt();

        Serial.println(elevator_feed(elevatorFeedPercent));
        Serial.println("inch/min\n");
        if (elevatorDigiDirection == 1)Serial.println("Up\n");
        else if (elevatorDigiDirection == 0)Serial.println("Down\n");
        analogWrite(ELEVATOR, 0);
        delay(50);
        digitalWrite(ELEVATOR_DIRECTION, elevatorDigiDirection);
        analogWrite(ELEVATOR, elevatorFeedPercent * 255 / 100);
        if (elevatorDigiDirection == 1)previousElevatorState = 'u';
        else previousElevatorState = 'd';
      }
      else if (cmd == "eup") {
        //turns elevator clockwise
        analogWrite(ELEVATOR, 0);
        delay(100);
        digitalWrite(ELEVATOR_DIRECTION, HIGH);
        analogWrite(ELEVATOR, maxVelocity);
        previousElevatorState = 'u';
        Serial.println("eup");
      }
      else if (cmd == "edown") {
        //turns elevator counter-clockwise
        analogWrite(ELEVATOR, 0);
        delay(100);
        digitalWrite(ELEVATOR_DIRECTION, LOW);
        analogWrite(ELEVATOR, maxVelocity);
        previousElevatorState = 'd';
        Serial.println("edown");
      }
      else if (cmd == "es") {
        //stops elevator
        analogWrite(ELEVATOR, 0);
        previousElevatorState = 'n';
        Serial.println("es");
      }
      else if (cmd == "goto") {
        //sends table to wanted position
        //eventually need split function for a single string command

        Serial.println("What is the cuvette of interest?");
        while (!Serial.available()) {
          ;
        }
        cuvette = (Serial.readStringUntil('\n')).toInt();
        Serial.println(cuvette);

        Serial.println("What is the desired position?");
        while (!Serial.available()) {
          ;
        }
        desiredPosition = (Serial.readStringUntil('\n')).toInt();
        Serial.println(desiredPosition);

        if (cuvette >= 26) {
          Serial.println("Error. Chose cuvette number from 0 to 25");
        }
        if (desiredPosition >= 26) {
          Serial.println("Error. Chose position number from 0 to 25");
        }
        turnTable (cuvette, desiredPosition);

      }
      else if (cmd == "tccw") {
        //turns table counter-clockwise
        table.writeMicroseconds(SERVO_STOP);
        delay(100);
        table.writeMicroseconds(SERVO_MAX_CCW);
        tableDirection = 'i';
        Serial.println("tccw");
      }
      else if (cmd == "tcw") {
        //turns table clockwise
        table.writeMicroseconds(SERVO_STOP);
        delay(100);
        table.writeMicroseconds(SERVO_MAX_CW);
        tableDirection = 'd';
        Serial.println("tcw");
      }
      else if (cmd == "ts") {
        //stops table
        table.writeMicroseconds(SERVO_STOP);
        tableDirection = 'n';
        turnTableFree = true;
        Serial.println("ts");
      }
      else if (cmd == "p1ccw") {               //pump1
        //turns pump1 counter-clockwise
        digitalWrite(PUMPS_LEGA, LOW);
        digitalWrite(PUMP1_SPEED, HIGH);
        delay(100);
        Serial.println("p1ccw");
      }
      else if (cmd == "p1cw") {               //pump1
        //turns pump1 clockwise
        digitalWrite(PUMPS_LEGA, HIGH);
        digitalWrite(PUMP1_SPEED, HIGH);
        delay(100);
        Serial.println("p1cw");
      }
      else if (cmd == "p2ccw") {               //pump2
        //turns drill counter-clockwise
        digitalWrite(PUMPS_LEGA, LOW);
        digitalWrite(PUMP2_SPEED, HIGH);
        delay(100);
        Serial.println("p2ccw");
      }
      else if (cmd == "p2cw") {               //pump2
        //turns drill clockwise
        digitalWrite(PUMPS_LEGA, HIGH);
        digitalWrite(PUMP2_SPEED, HIGH);
        delay(100);
        Serial.println("p2cw");
      }
      else if (cmd == "p3ccw") {               //pump3
        //turns pump1 counter-clockwise
        digitalWrite(PUMPS_LEGA, LOW);
        digitalWrite(PUMP3_SPEED, HIGH);
        delay(100);
        Serial.println("p3ccw");
      }
      else if (cmd == "p3cw") {               //pump3
        //turns pump1 clockwise
        digitalWrite(PUMPS_LEGA, HIGH);
        digitalWrite(PUMP3_SPEED, HIGH);
        delay(100);
        Serial.println("p3cw");
      }
      else if (cmd == "p4ccw") {               //pump4
        //turns drill counter-clockwise
        digitalWrite(PUMPS_LEGA, LOW);
        digitalWrite(PUMP4_SPEED, HIGH);
        delay(100);
        Serial.println("p4ccw");
      }
      else if (cmd == "p4cw") {               //pump4
        //turns drill clockwise
        digitalWrite(PUMPS_LEGA, HIGH);
        digitalWrite(PUMP4_SPEED, HIGH);
        delay(100);
        Serial.println("p4cw");
      }
      else if (cmd == "p5ccw") {               //pump5
        //turns drill counter-clockwise
        digitalWrite(PUMPS_LEGA, LOW);
        digitalWrite(PUMP5_SPEED, HIGH);
        delay(100);
        Serial.println("p5ccw");
      }
      else if (cmd == "p5cw") {               //pump5
        //turns drill clockwise
        digitalWrite(PUMPS_LEGA, HIGH);
        digitalWrite(PUMP5_SPEED, HIGH);
        delay(100);
        Serial.println("p5cw");
      }
      else if (cmd == "ps") {
        //stops pumps
        digitalWrite(PUMP1_SPEED, LOW);
        digitalWrite(PUMP2_SPEED, LOW);
        digitalWrite(PUMP3_SPEED, LOW);
        digitalWrite(PUMP4_SPEED, LOW);
        digitalWrite(PUMP5_SPEED, LOW);
        digitalWrite(PUMPS_LEGA, LOW);
        Serial.println("ps");
      }
      else if (cmd == "v1") {
        digitalWrite(VIBRATOR1, HIGH);
        Serial.println("v1");
      }
      else if (cmd == "v2") {
        digitalWrite(VIBRATOR2, HIGH);
        Serial.println("v2");
      }
      else if (cmd == "v3") {
        digitalWrite(VIBRATOR3, HIGH);
        Serial.println("v3");
      }
      else if (cmd == "v4") {
        digitalWrite(VIBRATOR4, HIGH);
        Serial.println("v4");
      }
      else if (cmd == "v5") {
        digitalWrite(VIBRATOR5, HIGH);
        Serial.println("v5");
      }
      else if (cmd == "v6") {
        digitalWrite(VIBRATOR6, HIGH);
        Serial.println("v6");
      }
      else if (cmd == "vs") {
        digitalWrite(VIBRATOR1, LOW);
        digitalWrite(VIBRATOR2, LOW);
        digitalWrite(VIBRATOR3, LOW);
        digitalWrite(VIBRATOR4, LOW);
        digitalWrite(VIBRATOR5, LOW);
        digitalWrite(VIBRATOR6, LOW);
        Serial.println("v1");
      }
      else if (cmd == "led1") {
        digitalWrite(LED1, HIGH);
        photoChoice(1);
        delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);

        Serial.print("Voltage On =");
        Serial.print(voltage);
        Serial.println("led1");
      }
      else if (cmd == "led1s") {
        digitalWrite(LED1, LOW);
        delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);
        Serial.print("Voltage Off =");
        Serial.print(voltage);
        digitalWrite(S0, 0);
        digitalWrite(S1, 0);
        digitalWrite(S2, 0);
        digitalWrite(S3, 0);
        Serial.println("led1s");
      }
      else if (cmd == "led2") {
        digitalWrite(LED2, HIGH);
        photoChoice(2);
        delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);

        Serial.print("Voltage On =");
        Serial.print(voltage);
        Serial.println("led2");
      }
      else if (cmd == "led2s") {
        digitalWrite(LED2, LOW);
        delay(100);
        val = analogRead(PHOTORESISTOR);
        voltage = val * (5.0 / 1023.0);
        Serial.print("Voltage Off =");
        Serial.print(voltage);
        digitalWrite(S0, 0);
        digitalWrite(S1, 0);
        digitalWrite(S2, 0);
        digitalWrite(S3, 0);
        Serial.println("led2s");
      }

      else if (cmd == "deactivate") {
        //stops all
        table.writeMicroseconds(SERVO_STOP);
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
        Serial.print("Homing Table");
        turnTable (0, 0);
        tableDirection = 'n';
        turnTableFree = true;
        previousElevatorState = 'n';

        isActivated = false;
        Serial.print("cmd: ");
        Serial.println(cmd);
      }
    }
  }
  // this works but doesn't consider the possibility of both switches
  // being triggered at the same time which shouldn't ever actually happen
  if (isTriggered) {
    if ( (millis() - triggerTime) >= TRIGGER_DELAY) {
      // if the last interrupt was a press (meaning it's stabilized and in contact)
      // then there's a real press
      if (isContacted) {
        isActualPress = true;   // otherwise it's not a real press
      }                         // so the limit switch state should stay whatever it used to be
      // and so should actualPress
      isTriggered = false;      // either way, we should reset the triggered bool in wait for the next trigger
    }
  }
  if (isActualPress) {
    if (previousElevatorState == 'n')cuvettePosition();
    else if (previousElevatorState == 'u')elevatorBottomInterrupt();
    else if (previousElevatorState == 'd')elevatorBottomInterrupt();
  }
  // now that the behaviour is complete we can reset these in wait for the next trigger to be confirmed
  isActualPress = false;

  if ((turnTableFree == false) && (tablePosition[desiredPosition] == cuvette)) {
    table.writeMicroseconds(SERVO_STOP);
    tableDirection = 'n';
    turnTableFree = true;
  }
}

void elevatorTopInterrupt () {
  //stops elevator
  unsigned long timer = millis();

  analogWrite(ELEVATOR, 0);
  digitalWrite(ELEVATOR_DIRECTION, LOW);
  analogWrite(ELEVATOR, maxVelocity);
  while ((millis() - timer) < 500) {
    ;
  }
  analogWrite(ELEVATOR, 0);
  previousElevatorState = 'n';
}

void elevatorBottomInterrupt () {
  //stops elevator
  unsigned long timer = millis();

  analogWrite(ELEVATOR, 0);
  digitalWrite(ELEVATOR_DIRECTION, HIGH);
  analogWrite(ELEVATOR, maxVelocity);
  while ((millis() - timer) < 500) {
    ;
  }
  analogWrite(ELEVATOR, 0);
  previousElevatorState = 'n';
}

void cuvettePosition() {
  //gives the integer value of the cuvette of the table 1 to 25, cuvettes are only on even numbers,4 chute is cuvettePosition 0
  if (tableDirection == 'n') {
  }
  else if (tableDirection == 'i') {
    for (i = 0; i <= 25; i++) {
      tablePosition[i] = (tablePosition[i] + 1) % 26;
    }
    Serial.print("tablePosition[0]: ");
    Serial.println(tablePosition[0]);
  }
  else if (tableDirection == 'd') {
    int temp = tablePosition[0];
    for (i = 0; i <= 25; i++) {
      tablePosition[i] = (tablePosition[i] - 1) % 26;
      if (tablePosition[i] == -1) {
        tablePosition[i] = 25;
      }
    }
    Serial.print("tablePosition[0]: ");
    Serial.println(tablePosition[0]);
  }
}

void turnTable (int cuvette, int desiredPosition) {
  int initialPosition = 0;
  int difference = 0;

  for (i = 0; i <= 25; i++) {
    initialPosition = i;
    if (tablePosition[i] == cuvette)break;
  }

  Serial.print("initialPosition: ");
  Serial.println(initialPosition);

  difference = desiredPosition - initialPosition;

  Serial.print("difference: ");
  Serial.println(difference);

  if ( (difference > -13 && difference < 0) || (difference > 13 && difference < 26)) {
    tableDirection = 'i';
    table.writeMicroseconds(SERVO_MAX_CCW);
  }
  else if ( (difference > -26 && difference < -13) || (difference > 0 && difference < 13)) {
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
  if (input_elevator_feed < 0) {
    input_elevator_feed = 0;
  }
  if (input_elevator_feed > 100) {
    input_elevator_feed = 0.1066;
  }
  return input_elevator_feed * 0.1066 / 100;
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


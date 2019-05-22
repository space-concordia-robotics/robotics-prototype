#include <Servo.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>

#define SERVO_STOP 1500
#define SERVO_MAX_CW 1600
#define SERVO_MAX_CCW 1400
/*
  int drill_speed(input_drill_speed);//max 165RPM
  int elevator_feed(input_elevator_feed);//max 0.107inch/s
*/
void elevatorInterrupt (void);

const int BLUETOOTH_TX_PIN = 10;
const int BLUETOOTH_RX_PIN = 11;

SoftwareSerial bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
ArduinoBlue phone(bluetooth);

int test = 0;
int button;
int drill = 5;
int drill_direction = 4;
int elevator = 7;
int elevator_direction = 6;
int tablePin = 13;
int pump1A = 34;
int pump1B = 36;
int pump2A = 38;
int pump2B = 40;
/*int pump3A = 42;
  int pump3B = 44;
  int pump4A = 46;
  int pump4B = 48;
  int pump5A = 50;
  int pump5B = 52;*/
int limit_top = 21;
int limit_bottom = 43;
int table_position = 45;
int led1 = 12;
int led2 = 13;
int photoresistor = A2;
int maxVelocity = 255;
int drillSpeed;
int elevatorSpeed;
float val = 0;
float voltage = 0;

Servo table;

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(50);
  bluetooth.begin(9600);

  table.attach(tablePin);
  pinMode(drill, OUTPUT);
  pinMode(drill_direction, OUTPUT);
  pinMode(elevator, OUTPUT);
  pinMode(elevator_direction, OUTPUT);
  pinMode(pump1A, OUTPUT);
  pinMode(pump1B, OUTPUT);
  pinMode(pump2A, OUTPUT);
  pinMode(pump2B, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(limit_top, INPUT_PULLUP);
  pinMode(limit_bottom, INPUT_PULLUP);
  pinMode(photoresistor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(limit_top), elevatorInterrupt, RISING);

  delay(100);

  analogWrite(drill, 0);
  analogWrite(elevator, 0);
  digitalWrite(pump1A, LOW);
  digitalWrite(pump1B, LOW);
  digitalWrite(pump2A, LOW);
  digitalWrite(pump2B, LOW);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  table.writeMicroseconds(SERVO_STOP);
  delay(1000);

  Serial.print("\nsetup complete");

}

void loop() {
  Serial.println(test);

  button = phone.getButton();

  if (button == 0) {
    //turns drill counter-clockwise
    analogWrite(drill, 0);
    delay(100);
    digitalWrite(drill_direction, HIGH);
    analogWrite(drill, maxVelocity);
    Serial.println("button0");
  }
  else if (button == 1) {
    //turns drill clockwise
    analogWrite(drill, 0);
    delay(100);
    digitalWrite(drill_direction, LOW);
    analogWrite(drill, maxVelocity);
    Serial.println("button1");
  }
  else if (button == 2) {
    //stops drill
    analogWrite(drill, 0);
    Serial.println("button2");
  }
  else if (button == 3) {
    //turns elevator clockwise
    analogWrite(elevator, 0);
    delay(100);
    digitalWrite(elevator_direction, HIGH);
    analogWrite(elevator, maxVelocity);
    Serial.println("button3");
  }
  else if (button == 4) {
    //turns elevator counter-clockwise
    analogWrite(elevator, 0);
    delay(100);
    digitalWrite(elevator_direction, LOW);
    analogWrite(elevator, maxVelocity);
    Serial.println("button4");
  }
  else if (button == 5) {
    //stops elevator
    analogWrite(elevator, 0);
    Serial.println("button5");
  }
  else if (button == 7) {
    //turns table clockwise
    table.writeMicroseconds(SERVO_STOP);
    delay(100);
    table.writeMicroseconds(SERVO_MAX_CW);
    Serial.println("button7");
  }
  else if (button == 6) {
    //turns table counter-clockwise
    table.writeMicroseconds(SERVO_STOP);
    delay(100);
    table.writeMicroseconds(SERVO_MAX_CCW);
    Serial.println("button6");
  }
  else if (button == 8) {
    //stops table
    table.writeMicroseconds(SERVO_STOP);
    Serial.println("button8");
  }
  else if (button == 9) {
    //turns pump1 counter-clockwise
    digitalWrite(pump1A, HIGH);
    digitalWrite(pump1B, LOW);
    delay(100);
    Serial.println("button9");
  }
  else if (button == 10) {
    //turns pump1 clockwise
    digitalWrite(pump1A, LOW);
    digitalWrite(pump1B, HIGH);
    delay(100);
    Serial.println("button10");
  }
  else if (button == 11) {
    //turns drill counter-clockwise
    digitalWrite(pump2A, HIGH);
    digitalWrite(pump2B, LOW);
    delay(100);
    Serial.println("button11");
  }
  else if (button == 12) {
    //turns drill clockwise
    digitalWrite(pump2A, LOW);
    digitalWrite(pump2B, HIGH);
    delay(100);
    Serial.println("button12");
  }
  else if (button == 13) {
    //stops pumps
    digitalWrite(pump1A, LOW);
    digitalWrite(pump1B, LOW);
    digitalWrite(pump2A, LOW);
    digitalWrite(pump2B, LOW);
    Serial.println("button13");
  }
  else if (button == 14) {
    digitalWrite(led1, HIGH);
    delay(100);
    val = analogRead(photoresistor);
    voltage = val * (5.0 / 1023.0);

    Serial.print("Voltage On =");
    Serial.print(voltage);
    Serial.println();
    phone.sendMessage(String(voltage));
  }
  else if (button == 15) {
    digitalWrite(led1, LOW);
    delay(100);
    val = analogRead(photoresistor);
    voltage = val * (5.0 / 1023.0);

    Serial.print("Voltage Off =");
    Serial.println(voltage);
    phone.sendMessage(String(voltage));
  }
  else if (button == 16) {
    digitalWrite(led2, HIGH);
    delay(100);
    val = analogRead(photoresistor);
    voltage = val * (5.0 / 1023.0);

    Serial.print("Voltage On =");
    Serial.print(voltage);
    Serial.println();
    phone.sendMessage(String(voltage));
  }
  else if (button == 17) {
    digitalWrite(led2, LOW);
    delay(100);
    val = analogRead(photoresistor);
    voltage = val * (5.0 / 1023.0);

    Serial.print("Voltage Off =");
    Serial.println(voltage);
    phone.sendMessage(String(voltage));
  }
  else if (button == 18) {
    //stops all
    table.writeMicroseconds(SERVO_STOP);
    analogWrite(elevator, 0);
    analogWrite(drill, 0);
    digitalWrite(pump1A, LOW);
    digitalWrite(pump1B, LOW);
    digitalWrite(pump2A, LOW);
    digitalWrite(pump2B, LOW);
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    Serial.println("button18");
  }
}

void elevatorTopInterrupt () {
  //stops elevator
  analogWrite(elevator, 0);
  delay(100);
  digitalWrite(elevator_direction, HIGH);
  analogWrite(elevator, maxVelocity);
  delay(1000);
  analogWrite(elevator, 0);
}
void elevatorBottomInterrupt () {
  //stops elevator
  analogWrite(elevator, 0);
    delay(100);
  digitalWrite(elevator_direction, LOW);
  analogWrite(elevator, maxVelocity);
  delay(1000);
  analogWrite(elevator, 0);
}
/*
  int drill_speed() {
  int input_drill_speed=0;
  if (input_drill_speed < 0) {
    input_drill_speed = 0;
  }
  if (input_drill_speed > 165) {
    input_drill_speed = 165;
  }
  return input_drill_speed * 255 / 165;
  }

  int elevator_feed() {
  float input_elevator_feed=0;
  if (input_elevator_feed < 0) {
    input_drill_speed = 0;
  }
  if (input_drill_speed > 0.1066) {
    input_drill_speed = 0.1066;
  }
  return input_drill_speed * 255 / 0.1066;
  }
*/



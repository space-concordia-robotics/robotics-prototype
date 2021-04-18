
/*
   Concordia University - Space Concordia URC 2019
   Contributors:
   - Abtin Ghodoussi
   - Philippe Carvajal
   - Nicholas Harris
   Version 1.1 - May 2019 (Updated August 2019)

   Tutorial for uploading code through ICSP and setting the internal 8MHz clock:
   https://www.instructables.com/id/How-to-change-fuse-bits-of-AVR-Atmega328p-8bit-mic/
*/

#define HeartBeat_Pin 16 // PC2

#define Fan_A_Speed_Pin 9 // PB1
#define Fan_B_Speed_Pin 10 // PB2
#define M1_Enable_Pin 8 // PB0
#define M2_Enable_Pin 7 // PD7
#define M3_Enable_Pin 6 // PD6
#define M4_Enable_Pin 17 // PC3
#define M5_Enable_Pin 19 // PC5
#define M6_Enable_Pin 18 // PC4

//! Pin to read sensor value from
#define Mux_out_Pin A0 // PC0, 14ide

// Mux selector pins
// (NOTE: S2 and S3 are routed incorectly on the PCB and the schematic Rev 2.2. Swapping their pin values corrects this.)
#define S0_PIN 3 // PD3
#define S1_PIN 2 // PD2
#define S2_PIN 5 // PD5
#define S3_PIN 4 // PD4

#define HEARTBEAT_DELAY 1000 //!< in ms
#define CURRENT_DELAY 200 //!< in microseconds
#define TEMPERATURE_DELAY 200 //!< in microseconds
#define NUM_CURRENT_SENSORS 6
#define NUM_TEMP_SENSORS 3
#define NUM_MOTORS 6

// communication
#define SERIAL_BAUD_RATE 19200 // The MCU on the PDS is running with the internal 8MHz clock. The factor of 2 compensates for the slower baurate.
#define CHAR_BUFF_SIZE 150
#define SERIAL_TIMEOUT_DELAY 50 //!< in ms

// absolute limits
#define MAX_BATTERY_VOLTAGE (double)16.8 //!< 4S14P battery pack
#define MIN_BATTERY_VOLTAGE (double)12.0 //!< 4S14P battery pack
#define OVERCURRENT_LIMIT (double)10.0

#define VOLTAGE_REF (double)3.3
#define THERMISTOR_BIAS_RES (double)10000.0 //!< 10kOhm from the PDS schematic
#define SENSE_RESISTOR_VALUE (double)0.001  //!< 10mOhm from the PDS schematic
#define CURRENT_SENSOR_GAIN (double)20.0    //!< INA193 current sensor internal gain
#define CURRENT_RECIPROCAL (double)0.016129032 // VOLTAGE_REFERENCE/1023.00*(CURRENT_SENSOR_GAIN*SENSE_RESISTOR_VALUE)
#define VOLTAGE_RECIPROCAL (double)0.01935 // 0.01935V (3.3V / 1023) * (12KOhm / 2KOhm)
#define TEMP_RECIPROCAL (double)VOLTAGE_REF/1023.0

// Steinhart–Hart constants for a 10K NTC thermistor
#define A (double)0.00112531
#define B (double)0.000234712
#define C (double)0.0000000856635

struct { //!< 8 bit field to check for most errors
  uint8_t noLoop : 1;
  uint8_t thermOutOfRange : 1;
  uint8_t OV : 1;
  uint8_t UV : 1;
  uint8_t criticalError : 1;
} errorFlagGeneral;

struct { //!< 8 bit field to check for OverCurrent
  uint8_t motor1OC : 1;
  uint8_t motor2OC : 1;
  uint8_t motor3OC : 1;
  uint8_t motor4OC : 1;
  uint8_t motor5OC : 1;
  uint8_t motor6OC : 1;
} errorFlagCurrentReadings;

uint8_t currentMuxOrder[NUM_CURRENT_SENSORS] = {2, 1, 0, 15, 14, 13}; //!< corresponds to the multiplexer channel they are attached to from the PDS schematic

uint8_t motorPins[NUM_MOTORS] = {M1_Enable_Pin, M2_Enable_Pin, M3_Enable_Pin, M4_Enable_Pin, M5_Enable_Pin, M6_Enable_Pin};

bool autoProtection = true; //!< automatically shut off vs just send warning

int MAX_FAN_SPEED = 255; //!< 8 bit pwm
int fanASpeed = 0; int fanBSpeed = 0;

int rawCurrentReadings[NUM_CURRENT_SENSORS]; //!< current values measured from ADC
double currentReadings[NUM_CURRENT_SENSORS]; //!< current values in Amps

int rawTempReadings[NUM_TEMP_SENSORS]; //!< temperature values measured from ADC
double tempReadings[NUM_TEMP_SENSORS]; //!< temperature values in degrees celcius

double batteryVoltage = 0;

uint8_t motorState[NUM_MOTORS]; //!< power to motor enable/disable

unsigned long startTime = 0; //!< for task scheduler which is not implemented yet. \todo rename startTime to something more clear e.g. heartBeatTime or something? apparently its used for other stuff tho
unsigned long currentTime = 0; // for current timer
unsigned long tempTime = 0; // for temperature timer

int messagesReceived, messagesSent, badMessages;
char BUFFER[CHAR_BUFF_SIZE]; //!< used for sending/receiving/reading messages

// string/message handling
void parseCommand();
void generateSensorFeedback();
void printMotorState();
// sensor processing
void updateMotorState();
float readMultiplexer(uint8_t channel);

internal_comms::CommandCenter* commandCenter = new PDSCommandCenter();

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_DELAY);

  pinMode(Fan_A_Speed_Pin, OUTPUT);
  pinMode(Fan_B_Speed_Pin, OUTPUT);
  pinMode(M1_Enable_Pin, OUTPUT);
  pinMode(M2_Enable_Pin, OUTPUT);
  pinMode(M3_Enable_Pin, OUTPUT);
  pinMode(M4_Enable_Pin, OUTPUT);
  pinMode(M5_Enable_Pin, OUTPUT);
  pinMode(M6_Enable_Pin, OUTPUT);
  pinMode(HeartBeat_Pin, OUTPUT);
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);

  digitalWrite(HeartBeat_Pin, LOW);

  analogWrite(Fan_A_Speed_Pin,0);
  analogWrite(Fan_B_Speed_Pin,0);

  // Turn all motors on after 3 seconds.
  delay(3000);
  PORTB |= 0B00000001; //motor one
  PORTD |= 0B11000000; //motor two and three
  PORTC |= 0B00111000; //motor four, five and six

  startTime  = millis();
  currentTime = micros(); // for current timer
  tempTime = micros(); // for temperature timer

  Serial.println("Done");
}

void loop() {
  /***************************************************************************************************************/
  // listen to the serial port and parse an incoming message
  if (Serial.available()) {
    byte num = Serial.readBytesUntil('\n', BUFFER, CHAR_BUFF_SIZE); // The terminator character is discarded from the serial buffer
    if (num > 0) {
      Serial.println(BUFFER); // echo the received command
      parseCommand(); // check the command and (in some cases) react to it
      memset(BUFFER, 0, CHAR_BUFF_SIZE);
      messagesReceived++;
    } else {
      Serial.println("Command error: PDS received empty message");
      badMessages++;
    }
  }
  /***************************************************************************************************************/
  // read raw current values
  for (uint8_t i = 0; i < NUM_CURRENT_SENSORS; i++) {
    rawCurrentReadings[i] = readMultiplexer(currentMuxOrder[i]);
    while(micros() < CURRENT_DELAY + currentTime){
      ; // wait...
    }
    currentTime = micros();
  }
  // convert all current values to Amps
  for (uint8_t i = 0; i < NUM_CURRENT_SENSORS; i++) {
    // Conversion to voltage = ( raw value * voltage ref / 1023 )
    // Convertion to current = volatge * current_reciprocal
    currentReadings[i] = rawCurrentReadings[i] * CURRENT_RECIPROCAL;
  }
  /***************************************************************************************************************/
  // read the raw battery voltage and convert it
  batteryVoltage =  readMultiplexer(4) * VOLTAGE_RECIPROCAL;

  if (batteryVoltage < MIN_BATTERY_VOLTAGE) {
    errorFlagGeneral.UV = 1;
  } else if (batteryVoltage > MAX_BATTERY_VOLTAGE) {
    errorFlagGeneral.OV = 1;
  }
  if (batteryVoltage < 0 || batteryVoltage > 17.0) {
    errorFlagGeneral.criticalError = 1;
    batteryVoltage = 999.9; // make error case visible
  }

  // read the temperature values
  rawTempReadings[0] = readMultiplexer(7);     // thermistor 1
  while(micros() < TEMPERATURE_DELAY + tempTime){
    ; // wait...
  }
  tempTime = micros();
  rawTempReadings[1] = readMultiplexer(6);     // thermistor 2
  while(micros() < TEMPERATURE_DELAY + tempTime){
    ; // wait...
  }
  tempTime = micros();
  rawTempReadings[2] = readMultiplexer(5);     // thermistor 3

  // calculate all temperature values in degrees celcius using Steinhart–Hart approximation
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (rawTempReadings[i] <= 0 || rawTempReadings[i] >= 1023.0) {
      errorFlagGeneral.thermOutOfRange = 1;
      tempReadings[i] = 999.9; //make error case visible
    } else {
      errorFlagGeneral.thermOutOfRange = 0;
      double v = (double)rawTempReadings[i] * TEMP_RECIPROCAL;
      double Rt = THERMISTOR_BIAS_RES * (VOLTAGE_REF - v) / v;
      double ln = log(Rt);
      tempReadings[i] = (1.0 / (A + B * ln + C * pow(ln, 3))) - 273.15;
    }
  }

  /***************************************************************************************************************/
  if (errorFlagGeneral.UV && autoProtection) {
    errorFlagGeneral.criticalError = 1;
    PORTB &= 0B11111110; //motor one
    PORTD &= 0B00111111; //motor two and three
    PORTC &= 0B11000111; //motor four, five and six
  }
  /***************************************************************************************************************/
  updateMotorState(); //update all motor enable pin states by reading them
  /***************************************************************************************************************/
  if (millis() - startTime > HEARTBEAT_DELAY) {
    digitalWrite(HeartBeat_Pin, !digitalRead(HeartBeat_Pin));
    startTime = millis();
    // if (Serial.available()) {
      generateSensorFeedback();  //generate status message containing current, voltage, temp, speed
      //Serial.print(BUFFER);
      memset(BUFFER, 0, CHAR_BUFF_SIZE);
      messagesSent++;
    // }
  }
}//end of void loop

void parseCommand() {

  char *token = strtok(BUFFER, " ");

  while (token != NULL) {
    if (strcmp(token, "PDS") == 0) {
      token = strtok(NULL, " "); //find the next token
      if (strcmp(token, "who") == 0) { // request for identification
        Serial.println("PDS pds");
      } else if (strcmp(token, "ping") == 0) { // simple ping request
        Serial.println("PDS pong");
      }
      else if (*token == 'S') {  //disable all motors
        PORTB &= 0B11111110; //motor one
        PORTD &= 0B00111111; //motor two and three
        PORTC &= 0B11000111; //motor four, five and six
        Serial.println("Command: PDS disabling power to all motors");
        updateMotorState();
        printMotorState();
        break;
      } else if (*token == 'A') { //enable all motors
        PORTB |= 0B00000001; //motor one
        PORTD |= 0B11000000; //motor two and three
        PORTC |= 0B00111000; //motor four, five and six
        Serial.println("Command: PDS enabling power to all motors");
        updateMotorState();
        printMotorState();
        break;
      } else if (*token == 'M') { //single motor on/off
        token = strtok(NULL, " "); //find the next token
        int motorNum = (int)*token - 48;
        if (motorNum < 1 || motorNum > 6) {
          badMessages++;
          Serial.println("Command error: PDS invalid motor number");
        } else {
          token = strtok(NULL, " "); //find the next token
          int state = (int)*token - 48;
          if (state == 0 || state == 1) {
            digitalWrite(motorPins[motorNum - 1], state);
            Serial.print("Command: PDS toggling power state to motor: ");
            Serial.println(motorNum);
          } else {
            Serial.println("Command error: PDS invalid motor state");
            badMessages++;
          }
        }
        updateMotorState();
        printMotorState();
        break;
      } else if (*token == 'F') { //set fan speed in percentage 0-100%
        token = strtok(NULL, " "); //find the next token
        int fanNum = (int)*token - 48;
        if (fanNum == 1 || fanNum == 2) {
          token = strtok(NULL, " "); //find the next token
          int fanSpeed = atoi(token);
          if (fanSpeed <  0 || fanSpeed > 100) {
            Serial.println("Command error: PDS fan speed out of range");
            badMessages++;
            break;
          }
          MAX_FAN_SPEED = (int)(12.0 * 255.0 / batteryVoltage); // calculate the max allowable fan speed
          fanSpeed = map(fanSpeed, 0, 100, 0, MAX_FAN_SPEED); // To check that the value returned is indeed correct
          switch (fanNum) {
            case 1:
              fanASpeed = fanSpeed;
              analogWrite(Fan_A_Speed_Pin,fanASpeed);
              break;
            case 2:
              fanBSpeed = fanSpeed;
              analogWrite(Fan_B_Speed_Pin,fanBSpeed);
              break;
          }
          Serial.println("Command: PDS updating fan speed");
        } else {
          Serial.println("Command error: PDS invalid fan number"); 
          badMessages++;
        }
      } else if (*token == 'R') { // Reset errorFlagGeneral

        Serial.println("Command: PDS Resetting flags");
        token = strtok(NULL, " "); //find the next token

        if (*token == 'G')
        {

          Serial.println("Command: PDS Resetting General Error flags");
          errorFlagGeneral.OV = 0;
          errorFlagGeneral.UV = 0;
          errorFlagGeneral.criticalError = 0;
          errorFlagGeneral.thermOutOfRange = 0;
          errorFlagGeneral.noLoop = 0;

        } else if (*token == 'C') {

          Serial.println("Command: PDS Resetting Current Reading Error flags");
          errorFlagCurrentReadings.motor1OC = 0;
          errorFlagCurrentReadings.motor2OC = 0;
          errorFlagCurrentReadings.motor3OC = 0;
          errorFlagCurrentReadings.motor4OC = 0;
          errorFlagCurrentReadings.motor5OC = 0;
          errorFlagCurrentReadings.motor6OC = 0;

        }
      } else if (*token == 'T') { // Toggle auto mode
        token = strtok(NULL, " "); //find the next token
        Serial.println("Command: AUTO has been sent");

        if (*token == '1')
        {
          Serial.println("Command: PDS set autoMode ON");
          autoProtection = true;
        } else if(*token == '0') {
          Serial.println("Command: PDS set autoMode OFF");
          autoProtection = false;
        } else {
          Serial.println("Command: PDS query autoMode status");
        }
      }
      break;
    } else {
      Serial.println("Command error: PDS invalid command");
      badMessages++;
      break;
    } // end of main "PDS" token check
  } // end of while loop
} // end of parseCommand

void generateSensorFeedback() {
  Serial.print("PDS ");
  Serial.print(batteryVoltage); Serial.print(", ");
  Serial.print(currentReadings[0]); Serial.print(", ");
  Serial.print(currentReadings[1]); Serial.print(", ");
  Serial.print(currentReadings[2]); Serial.print(", ");
  Serial.print(currentReadings[3]); Serial.print(", ");
  Serial.print(currentReadings[4]); Serial.print(", ");
  Serial.print(currentReadings[5]); Serial.print(", ");
  Serial.print(tempReadings[0]); Serial.print(", ");
  Serial.print(tempReadings[1]); Serial.print(", ");
  Serial.print(tempReadings[2]); Serial.print(", ");
  Serial.print(fanASpeed); Serial.print(", ");
  Serial.print(fanBSpeed); Serial.print(", ");
  Serial.print(errorFlagGeneral.OV); Serial.print(", ");
  Serial.print(errorFlagGeneral.UV); Serial.print(", ");
  Serial.println(errorFlagGeneral.criticalError);
}

void updateMotorState() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motorState[i] = digitalRead(motorPins[i]);
  }
}

float readMultiplexer(uint8_t channel) {

  if (channel < 0 || channel > 15) {
    channel = 0; //! \todo not the best implementation, should return a negative value or a false bool
  }

  // //! this is done because S0 and S1 are swapped on PORTD. This swaps those bits in the channel variable
  // uint8_t bit0 = (channel >> 0) & 1; // save bit 0
  // uint8_t bit1 = (channel >> 1) & 1; // save bit 1
  // uint8_t mask = bit0 ^ bit1; // if 11 or 00 then mask is 0, else mask is 1
  // mask = (mask << 0) | (mask << 1); // turn mask of 0 into 00, turn mask of 1 into 11
  // channel = channel ^ mask; // 0 XOR 0 preserves the bit, anything XOR 1 flips the bit, so if mask is 11 it flips the lower 2 bits and regardless it preserves everything else

  // // set bits two to five on PORTD for mux select line
  // PORTD = (PORTD & B11000011) | (channel << 2);

  switch (channel) {
    case 0:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, LOW);
      break;
    case 1:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, LOW);
      break;
    case 2:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, LOW);
      break;
    case 3:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, LOW);
      break;
    case 4:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, LOW);
      break;
    case 5:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, LOW);
      break;
    case 6:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, LOW);
      break;
    case 7:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, LOW);
      break;
    case 8:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 9:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 10:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 11:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, LOW);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 12:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 13:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, LOW);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 14:
      digitalWrite(S0_PIN, LOW);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, HIGH);
      break;
    case 15:
      digitalWrite(S0_PIN, HIGH);
      digitalWrite(S1_PIN, HIGH);
      digitalWrite(S2_PIN, HIGH);
      digitalWrite(S3_PIN, HIGH);
      break;
  }

  delay(10);
  return (double)analogRead(Mux_out_Pin);
}

void printMotorState() {
   // Print Motor state
  for (int i = 1; i <= NUM_MOTORS; ++i)
  {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" State: ");
    Serial.println(motorState[i-1]);
  }
}

/*
   Concordia University - Space Concordia URC 2019
   Contributors:
   - Abtin Ghodoussi
   - Philippe Carvajal
   - Nicholas Harris
   Version 1.1 - May 2019

   Tutorial for uploading code through ICSP and setting the internal 8MHz clock:
   https://www.instructables.com/id/How-to-change-fuse-bits-of-AVR-Atmega328p-8bit-mic/
*/

#define HeartBeat_Pin PC2

#define Fan_A_Speed_Pin PB1
#define Fan_B_Speed_Pin PB2
#define M1_Enable_Pin PB0
#define M2_Enable_Pin PD7
#define M3_Enable_Pin PD6
#define M4_Enable_Pin PC6
#define M5_Enable_Pin PC5
#define M6_Enable_Pin PC4

//! Pin to read sensor value from
#define Mux_out_Pin PC0

// Mux selector pins
#define S0_PIN PD3
#define S1_PIN PD2
#define S2_PIN PD4
#define S3_PIN PD5

#define HEARTBEAT_DELAY 1000 //!< in ms
#define NUM_CURRENT_SENSORS 6
#define NUM_TEMP_SENSORS 3
#define NUM_MOTORS 6

// communication
#define SERIAL_BAUD_RATE 9600
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
#define VOLTAGE_RECIPROCAL (double)((12.2*VOLTAGE_REF)/(2.2*1023.0)) //12.2k and 2.2k are resistor ratio from the PDS schematic
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

unsigned long prevBlinkTime = 0; //!< for task scheduler which is not implemented yet. \todo rename startTime to something more clear e.g. heartBeatTime or something? apparently its used for other stuff tho

int messagesReceived, messagesSent, badMessages;
char IN_BUFFER[CHAR_BUFF_SIZE]; //!< used for receiving/reading messages
char OUT_BUFFER[CHAR_BUFF_SIZE]; //!< used for sending/reading messages

// string/message handling
void parseCommand(char *inBuffer);
void generateSensorFeedback(char *outBuffer);
// sensor processing
void updateMotorState();
float readMultiplexer(uint8_t channel);

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
  pinMode(Mux_out_Pin, INPUT);

  digitalWrite(HeartBeat_Pin, LOW);
}

void loop() {
  /***************************************************************************************************************/
  // listen to the serial port and parse an incoming message
  if (Serial.available()) {
    byte num = Serial.readBytesUntil('\n', IN_BUFFER, CHAR_BUFF_SIZE); // The terminator character is discarded from the serial buffer
    if (IN_BUFFER[num-1] == '\r') { // catch carriage returns and ELIMINATE them >:(
      IN_BUFFER[num-1] = '\0'; // replace it with the char array terminator character
    }
    if (num > 0) { // not 100% sure if this catches every case
      Serial.println(IN_BUFFER); // echo the received command
      parseCommand(IN_BUFFER); // check the command and (in some cases) react to it
      messagesReceived++;
    }
    else {
      badMessages++;
      // should also respond to bad messages
    }
    memset(IN_BUFFER, 0, CHAR_BUFF_SIZE);
  }
  /***************************************************************************************************************/
  // read raw current values
  for (uint8_t i = 0; i < NUM_CURRENT_SENSORS; i++) {
    rawCurrentReadings[i] = readMultiplexer(currentMuxOrder[i]);
    delayMicroseconds(200); //!< \todo replace with timer
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
  }
  else if (batteryVoltage > MAX_BATTERY_VOLTAGE) {
    errorFlagGeneral.OV = 1;
  }
  if (batteryVoltage < 0 || batteryVoltage > 17.0) {
    errorFlagGeneral.criticalError = 1;
    batteryVoltage = 999.9;// make error case visible
  }
  /***************************************************************************************************************/
  // read the temperature values
  rawTempReadings[0] = readMultiplexer(7);     // thermistor 1
  delayMicroseconds(200); //!< \todo replace with timer
  rawTempReadings[1] = readMultiplexer(6);     // thermistor 2
  delayMicroseconds(200); //!< \todo replace with timer
  rawTempReadings[2] = readMultiplexer(5);     // thermistor 3

  // calculate all temperature values in degrees celcius using Steinhart–Hart approximation
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (rawTempReadings[i] <= 0 || rawTempReadings[i] >= 1023.0) {
      errorFlagGeneral.thermOutOfRange = 1;
      tempReadings[i] = 999.9; //make error case visible
    }
    else {
      errorFlagGeneral.thermOutOfRange = 0;
      double v = (double)rawTempReadings[i] * TEMP_RECIPROCAL;
      double Rt = THERMISTOR_BIAS_RES * (VOLTAGE_REF - v) / v;
      double ln = log(Rt);
      tempReadings[i] = (1.0 / (A + B * ln + C * pow(ln, 3))) - 273.15;
    }
  }
  /***************************************************************************************************************/
  if (errorFlagGeneral.UV) {
    errorFlagGeneral.criticalError = 1;
    PORTB &= 0B11111110; //motor one
    PORTD &= 0B00111111; //motor two and three
    PORTC &= 0B11000111; //motor four, five and six
  }
  /***************************************************************************************************************/
  if (!errorFlagGeneral.UV) {
    // convert speed percentage to PWM based on battery voltage
    MAX_FAN_SPEED = (int)(12 * 255 / batteryVoltage); // calculate the max allowable fan speed
    fanASpeed = map(fanASpeed, 0, 255, 0, MAX_FAN_SPEED); // attempt to keep the pwm voltage at 12v
    fanBSpeed = map(fanBSpeed, 0, 255, 0, MAX_FAN_SPEED); // attempt to keep the pwm voltage at 12v
    analogWrite(Fan_A_Speed_Pin, fanASpeed);
    analogWrite(Fan_B_Speed_Pin, fanBSpeed);
  }
  else {
    analogWrite(Fan_A_Speed_Pin, 0);
    analogWrite(Fan_B_Speed_Pin, 0);
  }
  /***************************************************************************************************************/
    updateMotorState(); //update all motor enable pin states by reading them
    generateSensorFeedback(OUT_BUFFER);  //generate status message containing current, voltage and temperature values
  /***************************************************************************************************************/
  if (millis() - prevBlinkTime > HEARTBEAT_DELAY) {
    digitalWrite(HeartBeat_Pin, !digitalRead(HeartBeat_Pin));
    prevBlinkTime = millis();
    Serial.println(OUT_BUFFER);
    messagesSent++;
  }
}//end of void loop

void parseCommand(char *inBuffer) {
  char *token = strtok_r(inBuffer, " ", &inBuffer);
    if (strcmp(token, "PDS") == 0) {
      Serial.println(inBuffer);
      if (strcmp(inBuffer, "S") == 0) { //disable all motors
        Serial.println("Testing S"); //Testing
        PORTB &= 0B11111110; //motor one
        PORTD &= 0B00111111; //motor two and three
        PORTC &= 0B11000111; //motor four, five and six
      }
      else if (strcmp(inBuffer, "A") == 0) { //enable all motors
        Serial.print("Testing A"); //Testing
        if (!errorFlagGeneral.UV) { //if there is no undervoltage error
          PORTB |= 0B00000001; //motor one
          PORTD |= 0B11000000; //motor two and three
          PORTC |= 0B00111000; //motor four, five and six
        }
      }
      else if (strcmp(inBuffer, "M") == 0) { //single motor on/off
        Serial.print("Testing M"); //Testing
        if (!errorFlagGeneral.UV) {
          token = strtok_r(NULL, " ", &inBuffer); //find the next token
          int motorNum = (int)token;
          if (motorNum < 1 || motorNum > 6) {
            badMessages++;
          }
          else {
            token = strtok_r(NULL, " ", &inBuffer); //find the next token
            int state = (int)token;
            if (state != 0 || state != 1) {
              badMessages++;
            }
            else {
              digitalWrite(motorPins[motorNum - 1], state);
            }
          }
        }
      }
      else if (strcmp(inBuffer, "F") == 0) { //set fan speed in percentage 0-100%
        token = strtok_r(NULL, " ", &inBuffer); //find the next token
        int fanNum = (int)token;
        if (fanNum < 1 || fanNum > 2) {
          badMessages++;
        }
        else {
          int fanSpeed = (int)token;
          if (fanSpeed < 0 || fanSpeed > 100) {
            badMessages++;
            return;
          }

          MAX_FAN_SPEED = (int)(12 * 255 / batteryVoltage); // calculate the max allowable fan speed
          fanSpeed = map(fanSpeed, 0, 100, 0, MAX_FAN_SPEED); // To check that the value returned is indeed correct
          switch (fanNum) {
            case 1:
              fanASpeed = fanSpeed;
              break;
            case 2:
              fanBSpeed = fanSpeed;
              break;
          }
        }
      }
      else { // nonexistent PDS command
        badMessages++;
        // this should do something
      }
    } // end of main "PDS" token check
    else {
      badMessages++;
    }
} // end of parseCommand

void generateSensorFeedback(char *outBuffer) {
  memset(OUT_BUFFER, 0, CHAR_BUFF_SIZE);
  sprintf(outBuffer, "PDS %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    batteryVoltage, currentReadings[0],
    currentReadings[1], currentReadings[2],
    currentReadings[3], currentReadings[4],
    currentReadings[5], tempReadings[0],
    tempReadings[1], tempReadings[2]);
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

  //! this is done because S0 and S1 are swapped on PORTD. This swaps those bits in the channel variable
  uint8_t bit0 = (channel >> 0) & 1; // save bit 0
  uint8_t bit1 = (channel >> 1) & 1; // save bit 1
  uint8_t mask = bit0 ^ bit1; // if 11 or 00 then mask is 0, else mask is 1
  mask = (mask << 0) | (mask << 1); // turn mask of 0 into 00, turn mask of 1 into 11
  channel = channel ^ mask; // 0 XOR 0 preserves the bit, anything XOR 1 flips the bit, so if mask is 11 it flips the lower 2 bits and regardless it preserves everything else

  // set bits two to five on PORTD for mux select line
  PORTD = (PORTD & B11000011) | (channel << 2);

  return (double)analogRead(Mux_out_Pin);
}

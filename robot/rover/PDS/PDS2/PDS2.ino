// USB : Debug, UART : Production
#define USB

#include <Arduino.h>
#include <Stepper.h> // Stepper Motor library (Won't be used in future application)
#include "variables.h"
#include "sensor_read.h"
#include "load_setup.h"
#include "parse_command.h"
#include "PDSCommandCenter.h"
#include "Serial.h"

const uint8_t TX_PIN = 1;
const uint8_t RX_PIN = 0;

internal_comms::CommandCenter* commandCenter = new PDSCommandCenter();

void setup() 
{
    mux_settings();
    load_settings();
    internal_comms::startSerial(RX_PIN, TX_PIN);
}

void loop() 
{
    // listen to the serial port and parse an incoming message
    
    if (Serial.available() > 0) 
    {
        internal_comms::readCommand(commandCenter);
    }


    // Read Load Voltage
    enable_multisense();
    float load_value = load_voltage();
    Serial.print("Load Value after conversion: ");
    Serial.print(load_value); 
    Serial.println(" V");     
    
    // Read Temperature Value
    Temp1 = temp_read(0, 0, 0);
  
    Serial.print("Temperature: "); 
    Serial.print(Temp1);
    Serial.println(" C"); 
    delay(1000);
}
void pong()
{
    Serial.println("PDS pong");
}
void error()
{
    Serial.println("No command found"); 
}
void disableAllMotors()
{
        PORTB &= 0B11111110; //motor one
        PORTD &= 0B00111111; //motor two and three
        PORTC &= 0B11000111; //motor four, five and six
        Serial.println("Command: PDS disabling power to all motors");
}
void enableAllMotors()
{
        PORTB |= 0B00000001; //motor one
        PORTD |= 0B11000000; //motor two and three
        PORTC |= 0B00111000; //motor four, five and six
        Serial.println("Command: PDS enabling power to all motors");
}
void motor(uint8_t motorPin, uint8_t state)
{
    // Need check to see if pin given is a valid motor pin
    Serial.print("Motor pin: ");
    Serial.println(motorPin, DEC);
    Serial.print("On or off: ");
    Serial.println(state, DEC);
    digitalWrite(motorPin, state);
}
void fan(uint8_t state)
{
    Serial.print("Fan state: ");
    Serial.println(state, DEC);
    digitalWrite(Fans, state);
}

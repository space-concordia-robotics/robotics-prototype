
#include <Arduino.h>
#include <Stepper.h> // Stepper Motor library (Won't be used in future application)
#include "variables.h"
#include "sensor_read.h"
#include "load_setup.h"
#include "parse_command.h"



void setup() 
{
    Serial.begin(SERIAL_BAUD_RATE);  
    mux_settings();
    load_settings();
}

void loop() 
{
    // listen to the serial port and parse an incoming message
    if (Serial.available()) 
    {
        byte num = Serial.readBytesUntil('\n', BUFFER, CHAR_BUFF_SIZE); // Reads the message until it reaches the character '\n'
        if (num > 0) 
        {
            Serial.println(BUFFER); // Prints the received message
            parseCommand(); // Check the command
            memset(BUFFER, 0, CHAR_BUFF_SIZE); // Copies 0 to the variable BUFFER
        } 
        else 
        {
            Serial.println("Command error: PDS received empty message");
        }
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

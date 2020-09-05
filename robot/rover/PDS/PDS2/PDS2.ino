
#include <Arduino.h>
#include "sensor_read.h"


#define CHAR_BUFF_SIZE 150
#define SERIAL_BAUD_RATE 9600


char BUFFER[CHAR_BUFF_SIZE]; //!< used for sending/receiving/reading messages

float Temp1 = 0;


void setup() 
{
    Serial.begin(SERIAL_BAUD_RATE);
  
    mux_settings();

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
            memset(BUFFER, 0, CHAR_BUFF_SIZE); // Copies 0 to the variable BUFFER
        } 
        else 
        {
            Serial.println("Command error: PDS received empty message");
        }
    }

    Temp1 = temp_read(0, 0, 0);
  
    Serial.print("Temperature: "); 
    Serial.print(Temp1);
    Serial.println(" C"); 
    delay(1000);
  
}
